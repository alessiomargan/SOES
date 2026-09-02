/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

 /** \file
 * \brief
 * CAN over EtherCAT (CoE) module.
 *
 * SDO read / write and SDO service functions
 */

#include <stdlib.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <cc.h>
#include "esc.h"
#include "esc_coe.h"

#define BITS2BYTES(b) ((b + 7U) >> 3)
#define BITSPOS2BYTESOFFSET(b) (b >> 3)

/* Fetch value from object dictionary */
#define OBJ_VALUE_FETCH(v, o) \
   ((o).data ? *(__typeof__ (v) *)(o).data : (__typeof__ (v))(o).value)

#define SDO_COMMAND(byte)         \
   (byte & 0xe0)
#define SDO_COMPLETE_ACCESS(byte) \
   ((byte & COE_COMPLETEACCESS) == COE_COMPLETEACCESS)

#define READ_ACCESS(access, state) \
      (((access & ATYPE_Rpre) && (state == ESCpreop))   || \
       ((access & ATYPE_Rsafe) && (state == ESCsafeop)) || \
       ((access & ATYPE_Rop) && (state == ESCop)))

#define WRITE_ACCESS(access, state) \
      (((access & ATYPE_Wpre) && (state == ESCpreop))   || \
       ((access & ATYPE_Wsafe) && (state == ESCsafeop)) || \
       ((access & ATYPE_Wop) && (state == ESCop)))

typedef enum { UPLOAD, DOWNLOAD } load_t;

static const _objd *coe_segmented_obj;

/* CoE fields are offsets in EtherCAT octets, not C addressable units. */
enum
{
   COE_O_NUMBER_SERVICE = ESC_MBXHSIZE,
   COE_O_COMMAND        = ESC_MBXHSIZE + 2U,
   COE_O_INDEX          = ESC_MBXHSIZE + 3U,
   COE_O_SUBINDEX       = ESC_MBXHSIZE + 5U,
   COE_O_SIZE           = ESC_MBXHSIZE + 6U,
   COE_O_NORMAL_DATA    = ESC_MBXHSIZE + 10U,
   COE_O_SEGMENT_DATA   = ESC_MBXHSIZE + 3U,
   COE_INFO_O_OPCODE    = ESC_MBXHSIZE + 2U,
   COE_INFO_O_INCOMPLETE = ESC_MBXHSIZE + 3U,
   COE_INFO_O_FRAGMENTS = ESC_MBXHSIZE + 4U,
   COE_INFO_O_INDEX     = ESC_MBXHSIZE + 6U,
   COE_OBJ_O_DATATYPE   = ESC_MBXHSIZE + 8U,
   COE_OBJ_O_MAXSUB     = ESC_MBXHSIZE + 10U,
   COE_OBJ_O_OBJECTCODE = ESC_MBXHSIZE + 11U,
   COE_OBJ_O_NAME       = ESC_MBXHSIZE + 12U,
   COE_ENT_O_SUBINDEX   = ESC_MBXHSIZE + 8U,
   COE_ENT_O_VALUEINFO  = ESC_MBXHSIZE + 9U,
   COE_ENT_O_DATATYPE   = ESC_MBXHSIZE + 10U,
   COE_ENT_O_BITLENGTH  = ESC_MBXHSIZE + 12U,
   COE_ENT_O_ACCESS     = ESC_MBXHSIZE + 14U,
   COE_ENT_O_NAME       = ESC_MBXHSIZE + 16U
};

static uint8_t coe_get8 (const esc_octet_t *frame, uint16_t offset)
{
   return esc_octet_get (frame, offset);
}

static void coe_put8 (esc_octet_t *frame, uint16_t offset, uint8_t value)
{
   esc_octet_set (frame, offset, value);
}

static uint16_t coe_get16 (const esc_octet_t *frame, uint16_t offset)
{
   return esc_get_le16 (frame, offset);
}

static void coe_put16 (esc_octet_t *frame, uint16_t offset, uint16_t value)
{
   esc_put_le16 (frame, offset, value);
}

static uint32_t coe_get32 (const esc_octet_t *frame, uint16_t offset)
{
   return esc_get_le32 (frame, offset);
}

static void coe_put32 (esc_octet_t *frame, uint16_t offset, uint32_t value)
{
   esc_put_le32 (frame, offset, value);
}

static void coe_init_frame (esc_octet_t *frame, uint8_t service,
                            uint8_t command, uint16_t index,
                            uint8_t subindex)
{
   esc_mbx_set_length (frame, COE_DEFAULTLENGTH);
   esc_mbx_set_type (frame, MBXCOE);
   coe_put16 (frame, COE_O_NUMBER_SERVICE, (uint16_t)service << 12);
   coe_put8 (frame, COE_O_COMMAND, command);
   coe_put16 (frame, COE_O_INDEX, index);
   coe_put8 (frame, COE_O_SUBINDEX, subindex);
}

static void coe_info_init (esc_octet_t *frame, uint8_t opcode,
                           bool incomplete, uint16_t fragmentsleft)
{
   esc_mbx_set_type (frame, MBXCOE);
   coe_put16 (frame, COE_O_NUMBER_SERVICE,
              (uint16_t)COE_SDOINFORMATION << 12);
   coe_put8 (frame, COE_INFO_O_OPCODE,
             opcode | (incomplete ? 0x80U : 0U));
   coe_put8 (frame, COE_INFO_O_OPCODE + 1U, 0);
   coe_put16 (frame, COE_INFO_O_FRAGMENTS, fragmentsleft);
}

static uint64_t coe_native_value_get (const _objd *obj)
{
   uint64_t value = obj->value;

   if (obj->data == NULL)
   {
      return value;
   }

   switch (obj->datatype)
   {
   case DTYPE_BIT1: case DTYPE_BIT2: case DTYPE_BIT3: case DTYPE_BIT4:
   case DTYPE_BIT5: case DTYPE_BIT6: case DTYPE_BIT7: case DTYPE_BIT8:
   case DTYPE_BOOLEAN: case DTYPE_UNSIGNED8: case DTYPE_INTEGER8:
   case DTYPE_BITARR8:
      value = *(const uint8_t *)obj->data;
      break;
   case DTYPE_UNSIGNED16: case DTYPE_INTEGER16: case DTYPE_BITARR16:
      value = *(const uint16_t *)obj->data;
      break;
   case DTYPE_REAL32: case DTYPE_UNSIGNED24: case DTYPE_INTEGER24:
   case DTYPE_UNSIGNED32: case DTYPE_INTEGER32:
   case DTYPE_BITARR32: case DTYPE_PDO_MAPPING:
      value = *(const uint32_t *)obj->data;
      break;
   case DTYPE_REAL64: case DTYPE_UNSIGNED64: case DTYPE_INTEGER64:
      memcpy (&value, obj->data, sizeof (value));
      break;
   default:
      value = 0;
      break;
   }
   return value;
}

static void coe_native_value_set (const _objd *obj, uint64_t value)
{
   switch (obj->datatype)
   {
   case DTYPE_BIT1: case DTYPE_BIT2: case DTYPE_BIT3: case DTYPE_BIT4:
   case DTYPE_BIT5: case DTYPE_BIT6: case DTYPE_BIT7: case DTYPE_BIT8:
   case DTYPE_BOOLEAN: case DTYPE_UNSIGNED8: case DTYPE_INTEGER8:
   case DTYPE_BITARR8:
      *(uint8_t *)obj->data = (uint8_t)value;
      break;
   case DTYPE_UNSIGNED16: case DTYPE_INTEGER16: case DTYPE_BITARR16:
      *(uint16_t *)obj->data = (uint16_t)value;
      break;
   case DTYPE_REAL32: case DTYPE_UNSIGNED24: case DTYPE_INTEGER24:
   case DTYPE_UNSIGNED32: case DTYPE_INTEGER32:
   case DTYPE_BITARR32: case DTYPE_PDO_MAPPING:
      *(uint32_t *)obj->data = (uint32_t)value;
      break;
   case DTYPE_REAL64: case DTYPE_UNSIGNED64: case DTYPE_INTEGER64:
      memcpy (obj->data, &value, sizeof (value));
      break;
   default:
      break;
   }
}

static bool coe_is_scalar (const _objd *obj)
{
   switch (obj->datatype)
   {
   case DTYPE_BIT1: case DTYPE_BIT2: case DTYPE_BIT3: case DTYPE_BIT4:
   case DTYPE_BIT5: case DTYPE_BIT6: case DTYPE_BIT7: case DTYPE_BIT8:
   case DTYPE_BOOLEAN: case DTYPE_INTEGER8: case DTYPE_INTEGER16:
   case DTYPE_INTEGER24: case DTYPE_INTEGER32: case DTYPE_UNSIGNED8:
   case DTYPE_UNSIGNED16: case DTYPE_UNSIGNED24:
   case DTYPE_UNSIGNED32: case DTYPE_REAL32: case DTYPE_INTEGER64:
   case DTYPE_UNSIGNED64: case DTYPE_REAL64: case DTYPE_BITARR8:
   case DTYPE_BITARR16: case DTYPE_BITARR32: case DTYPE_PDO_MAPPING:
      return true;
   default:
      return false;
   }
}

static void coe_object_to_wire (const _objd *obj, esc_octet_t *dest,
                                uint32_t offset, uint32_t count)
{
   uint32_t i;
   if (coe_is_scalar (obj))
   {
      uint64_t value = coe_native_value_get (obj);
      for (i = 0; i < count; i++)
      {
         esc_octet_set (dest, i, (uint8_t)(value >> (8U * (offset + i))));
      }
   }
   else
   {
      const uint8_t *source = (const uint8_t *)obj->data;
      for (i = 0; i < count; i++)
      {
         esc_octet_set (dest, i, source[offset + i]);
      }
   }
}

static void coe_object_from_wire (const _objd *obj, const esc_octet_t *source,
                                  uint32_t offset, uint32_t count)
{
   uint32_t i;
   if (coe_is_scalar (obj))
   {
      uint64_t value = coe_native_value_get (obj);
      for (i = 0; i < count; i++)
      {
         uint32_t shift = 8U * (offset + i);
         value = (value & ~((uint64_t)0xffU << shift)) |
                 ((uint64_t)esc_octet_get (source, i) << shift);
      }
      coe_native_value_set (obj, value);
   }
   else
   {
      uint8_t *dest = (uint8_t *)obj->data;
      for (i = 0; i < count; i++)
      {
         dest[offset + i] = esc_octet_get (source, i);
      }
   }
}

/** Search for an object sub-index.
 *
 * @param[in] nidx   = local array index of object we want to find sub-index to
 * @param[in] subindex   = value on sub-index of object we want to locate
 * @return local array index if we succeed, -1 if we didn't find the index.
 */
int16_t SDO_findsubindex (int32_t nidx, uint8_t subindex)
{
   const _objd *objd;
   int16_t n = 0;
   uint8_t maxsub;
   objd = SDOobjects[nidx].objdesc;
   maxsub = SDOobjects[nidx].maxsub;

   /* Since most objects contain all subindexes (i.e. are not sparse),
    * check the most likely scenario first
    */
   if ((subindex <= maxsub) && ((objd + subindex)->subindex == subindex))
   {
      return subindex;
   }

   while (((objd + n)->subindex < subindex) && (n < maxsub))
   {
      n++;
   }
   if ((objd + n)->subindex != subindex)
   {
      return -1;
   }
   return n;
}

/** Search for an object index matching the wanted value in the Object List.
 *
 * @param[in] index   = value on index of object we want to locate
 * @return local array index if we succeed, -1 if we didn't find the index.
 */
int32_t SDO_findobject (uint16_t index)
{
   int32_t n = 0;
   while (SDOobjects[n].index < index)
   {
      n++;
   }
   if (SDOobjects[n].index != index)
   {
      return -1;
   }
   return n;
}

/**
 * Calculate the size in Bytes of RxPDO or TxPDOs by adding the
 * objects in SyncManager SDO 1C1x.
 *
 * A list of mapped objects is created for fast lookup of
 * dynamically mapped process data. The max size of the list (@a
 * max_mappings) can be set to 0 if dynamic processdata is not
 * supported.
 *
 * The output variable @a nmappings is set to 0 if dynamic processdata
 * is not supported. It is set to the number of mapped objects if
 * dynamic processdata is supported, or -1 if the mapping was
 * incorrect.

 * @param[in] index = SM index
 * @param[out] nmappings = number of mapped objects in SM, or -1 if
 *   mapping is invalid
 * @param[out] mappings = list of mapped objects in SM
 * @param[out] max_mappings = max number of mapped objects in SM
 * @return size of RxPDO or TxPDOs in Bytes.
 */
uint16_t sizeOfPDO (uint16_t index, int * nmappings, _SMmap * mappings,
                    int max_mappings)
{
   uint32_t offset = 0;
   uint16_t hobj;
   uint8_t si, sic, c;
   int32_t nidx;
   const _objd *objd;
   const _objd *objd1c1x;
   int mapIx = 0;

   if ((index != RX_PDO_OBJIDX) && (index != TX_PDO_OBJIDX))
   {
      return 0;
   }

   nidx = SDO_findobject (index);
   if(nidx < 0)
   {
      return 0;
   }

   objd1c1x = SDOobjects[nidx].objdesc;

   si = OBJ_VALUE_FETCH (si, objd1c1x[0]);
   if (si)
   {
      for (sic = 1; sic <= si; sic++)
      {
         hobj = OBJ_VALUE_FETCH (hobj, objd1c1x[sic]);
         nidx = SDO_findobject (hobj);
         if (nidx >= 0)
         {
            uint8_t maxsub;

            objd = SDOobjects[nidx].objdesc;
            maxsub = OBJ_VALUE_FETCH (maxsub, objd[0]);

            for (c = 1; c <= maxsub; c++)
            {
               uint32_t value = OBJ_VALUE_FETCH (value, objd[c]);
               uint8_t bitlength = value & 0xFF;

               if (max_mappings > 0)
               {
                  uint16_t index = (uint16_t)(value >> 16);
                  uint8_t subindex = (value >> 8) & 0xFF;
                  const _objd * mapping;

                  if (mapIx == max_mappings)
                  {
                     /* Too many mapped objects */
                     *nmappings = -1;
                     return 0;
                  }

                  DPRINT ("%04"PRIx32":%02"PRIx32" @ %"PRIu32"\n",
                        index,
                        subindex,
                        offset);

                  if (index == 0 && subindex == 0)
                  {
                     /* Padding element */
                     mapping = NULL;
                  }
                  else
                  {
                     nidx = SDO_findobject (index);
                     if (nidx >= 0)
                     {
                        int16_t nsub;

                        nsub = SDO_findsubindex (nidx, subindex);
                        if (nsub < 0)
                        {
                           /* Mapped subindex does not exist */
                           *nmappings = -1;
                           return 0;
                        }

                        mapping = &SDOobjects[nidx].objdesc[nsub];
                     }
                     else
                     {
                        /* Mapped index does not exist */
                        *nmappings = -1;
                        return 0;
                     }
                  }

                  mappings[mapIx].obj = mapping;
                  /* Save object list reference */
                  if(mapping != NULL)
                  {
                     mappings[mapIx].objectlistitem = &SDOobjects[nidx];
                  }
                  else
                  {
                     mappings[mapIx].objectlistitem = NULL;
                  }
                  mappings[mapIx++].offset = offset;
               }

               offset += bitlength;
            }
         }
      }
   }

   if (max_mappings > 0)
   {
      *nmappings = mapIx;
   }
   else
   {
      *nmappings = 0;
   }

   return BITS2BYTES (offset) & 0xFFFF;
}

/** Function for sending an SDO Abort reply.
 *
 * @param[in] reusembx   = mailbox buffer to use (if 0 then claim a new buffer)
 * @param[in] index      = index of object causing abort reply
 * @param[in] sub-index  = sub-index of object causing abort reply
 * @param[in] abortcode  = abort code to send in reply
 */
static void SDO_abort (uint8_t reusembx, uint16_t index, uint8_t subindex, uint32_t abortcode)
{
   uint8_t MBXout;
   esc_octet_t *coeres;
   if (reusembx)
      MBXout = reusembx;
   else
      MBXout = ESC_claimbuffer ();
   if (MBXout)
   {
      coeres = &MBX[MBXout * ESC_MBXSIZE];
      coe_init_frame (coeres, COE_SDOREQUEST, COE_COMMAND_SDOABORT,
                      index, subindex);
      coe_put32 (coeres, COE_O_SIZE, abortcode);
      MBXcontrol[MBXout].state = MBXstate_outreq;
   }
}

static void set_state_idle (uint8_t reusembx,
                           uint16_t index,
                           uint8_t subindex,
                           uint32_t abortcode)
{
   if (abortcode != 0)
   {
      SDO_abort (reusembx, index, subindex, abortcode);
   }

   MBXcontrol[0].state = MBXstate_idle;
   ESCvar.xoe = 0;
}

/** Function for responding on requested SDO Upload, sending the content
 *  requested in a free Mailbox buffer. Depending of size of data expedited,
 *  normal or segmented transfer is used. On error an SDO Abort will be sent.
 */
static void SDO_upload (void)
{
   esc_octet_t *coesdo, *coeres;
   uint16_t index;
   uint8_t subindex;
   int32_t nidx;
   int16_t nsub;
   uint8_t MBXout;
   uint32_t size;
   uint8_t dss;
   uint32_t abort = 1;
   const _objd *objd;
   coesdo = &MBX[0];
   index = coe_get16 (coesdo, COE_O_INDEX);
   subindex = coe_get8 (coesdo, COE_O_SUBINDEX);
   nidx = SDO_findobject (index);
   if (nidx >= 0)
   {
      nsub = SDO_findsubindex (nidx, subindex);
      if (nsub >= 0)
      {
         objd = SDOobjects[nidx].objdesc;
         uint8_t access = (objd + nsub)->flags & 0x3f;
         uint8_t state = ESCvar.ALstatus & 0x0f;
         if (!READ_ACCESS(access, state))
         {
            set_state_idle (0, index, subindex, ABORT_WRITEONLY);
            return;
         }
         MBXout = ESC_claimbuffer ();
         if (MBXout)
         {
            coeres = &MBX[MBXout * ESC_MBXSIZE];
            size = (objd + nsub)->bitlength;
            /* expedited bits used calculation */
            dss = 0x0c;
            if (size > 8)
            {
               dss = 0x08;
            }
            if (size > 16)
            {
               dss = 0x04;
            }
            if (size > 24)
            {
               dss = 0x00;
            }
            coe_init_frame (coeres, COE_SDORESPONSE,
                            COE_COMMAND_UPLOADRESPONSE | COE_SIZE_INDICATOR,
                            index, subindex);
            /* convert bits to bytes */
            size = BITS2BYTES(size);
            if (size <= 4)
            {
               /* expedited response i.e. length<=4 bytes */
               coe_put8 (coeres, COE_O_COMMAND,
                         coe_get8 (coeres, COE_O_COMMAND) |
                         COE_EXPEDITED_INDICATOR | dss);
               void *dataptr = ((objd + nsub)->data) ?
                     (objd + nsub)->data : (void *)&((objd + nsub)->value);
               abort = ESC_upload_pre_objecthandler (index, subindex,
                     dataptr, (size_t *)&size, (objd + nsub)->flags);
               if (abort == 0)
               {
                  coe_object_to_wire (objd + nsub, &coeres[COE_O_SIZE], 0,
                                      size);
               }
               else
               {
                  set_state_idle (MBXout, index, subindex, abort);
                  return;
               }
            }
            else
            {
               /* normal response i.e. length>4 bytes */
               abort = ESC_upload_pre_objecthandler (index, subindex,
                     (objd + nsub)->data, (size_t *)&size, (objd + nsub)->flags);
               if (abort == 0)
               {
                  /* set total size in bytes */
                  ESCvar.frags = size;
                  coe_put32 (coeres, COE_O_SIZE, size);
                  if ((size + COE_HEADERSIZE) > ESC_MBXDSIZE)
                  {
                     /* segmented transfer needed */
                     /* limit to mailbox size */
                     size = ESC_MBXDSIZE - COE_HEADERSIZE;
                     /* number of bytes done */
                     ESCvar.fragsleft = size;
                     /* signal segmented transfer */
                     ESCvar.segmented = MBXSEU;
                     ESCvar.data = (objd + nsub)->data;
                     coe_segmented_obj = objd + nsub;
                     ESCvar.flags = (objd + nsub)->flags;
                  }
                  else
                  {
                     ESCvar.segmented = 0;
                  }
                  esc_mbx_set_length (coeres, COE_HEADERSIZE + size);

                  /* use dynamic data */
                  coe_object_to_wire (objd + nsub,
                                      &coeres[COE_O_NORMAL_DATA], 0, size);
               }
               else
               {
                  set_state_idle (MBXout, index, subindex, abort);
                  return;
               }
            }
            if ((abort == 0) && (ESCvar.segmented == 0))
            {
               abort = ESC_upload_post_objecthandler (index, subindex,
                                                      (objd + nsub)->flags);
               if (abort != 0)
               {
                  set_state_idle (MBXout, index, subindex, abort);
                  return;
               }
            }
            MBXcontrol[MBXout].state = MBXstate_outreq;
         }
      }
      else
      {
         SDO_abort (0, index, subindex, ABORT_NOSUBINDEX);
      }
   }
   else
   {
      SDO_abort (0, index, subindex, ABORT_NOOBJECT);
   }
   MBXcontrol[0].state = MBXstate_idle;
   ESCvar.xoe = 0;
}

static uint32_t complete_access_get_variables(esc_octet_t *coesdo, uint16_t *index,
                                              uint8_t *subindex, int32_t *nidx,
                                              int16_t *nsub)
{
   *index = coe_get16 (coesdo, COE_O_INDEX);
   *subindex = coe_get8 (coesdo, COE_O_SUBINDEX);

   /* A Complete Access must start with Subindex 0 or Subindex 1 */
   if (*subindex > 1)
   {
      return ABORT_UNSUPPORTED;
   }

   *nidx = SDO_findobject (*index);
   if (*nidx < 0)
   {
      return ABORT_NOOBJECT;
   }

   *nsub = SDO_findsubindex (*nidx, *subindex);
   if (*nsub < 0)
   {
      return ABORT_NOSUBINDEX;
   }

   return 0;
}

static uint32_t complete_access_subindex_loop(const _objd *objd,
                                              int32_t nidx,
                                              int16_t nsub,
                                              esc_octet_t *mbxdata,
                                              load_t load_type,
                                              uint32_t max_bytes)
{
   /* Objects with dynamic entries cannot be accessed with Complete Access */
   if ((objd->datatype == DTYPE_VISIBLE_STRING) ||
       (objd->datatype == DTYPE_OCTET_STRING)   ||
       (objd->datatype == DTYPE_UNICODE_STRING))
   {
      return ABORT_CA_NOT_SUPPORTED;
   }

   uint32_t size = 0;

   /* Clear padded mbxdata byte [1] on upload */
   if ((load_type == UPLOAD) && (mbxdata != NULL))
   {
      esc_octet_set (mbxdata, 1, 0);
   }

   while (nsub <= SDOobjects[nidx].maxsub)
   {
      uint16_t bitlen = (objd + nsub)->bitlength;
      uint8_t bitoffset = size % 8;
      uint8_t access = (objd + nsub)->flags & 0x3f;
      uint8_t state = ESCvar.ALstatus & 0x0f;

      if ((bitlen % 8) == 0)
      {
         if (bitoffset != 0)
         {
            /* move on to next byte boundary */
            size += (8U - bitoffset);
         }
         if (mbxdata != NULL)
         {
            /* copy a non-bit data type to a byte boundary */
            if (load_type == UPLOAD)
            {
               if (READ_ACCESS(access, state))
               {
                  coe_object_to_wire (objd + nsub,
                        &mbxdata[BITS2BYTES(size)], 0, BITS2BYTES(bitlen));
               }
               else
               {
                  /* return zeroes for upload of WO objects */
                  esc_octet_clear (&mbxdata[BITS2BYTES(size)],
                                    BITS2BYTES(bitlen));
               }
            }
            /* download of RO objects shall be ignored */
            else if (WRITE_ACCESS(access, state))
            {
               coe_object_from_wire (objd + nsub,
                     &mbxdata[BITS2BYTES(size)], 0, BITS2BYTES(bitlen));
            }
         }
      }
      else if (mbxdata != NULL)
      {
         uint32_t bit;
         uint64_t value = coe_native_value_get (objd + nsub);

         if (load_type == UPLOAD)
         {
            for (bit = 0; bit < bitlen; bit++)
            {
               uint32_t target = size + bit;
               uint16_t octet = target >> 3;
               uint8_t mask = (uint8_t)(1U << (target & 7U));
               uint8_t current = coe_get8 (mbxdata, octet);
               bool set = READ_ACCESS(access, state) &&
                          (((value >> bit) & 1U) != 0U);
               coe_put8 (mbxdata, octet,
                         set ? (current | mask) : (current & ~mask));
            }
         }
         else if (WRITE_ACCESS(access, state))
         {
            value = 0;
            for (bit = 0; bit < bitlen; bit++)
            {
               uint32_t source = size + bit;
               if ((coe_get8 (mbxdata, source >> 3) &
                    (uint8_t)(1U << (source & 7U))) != 0U)
               {
                  value |= (uint64_t)1U << bit;
               }
            }
            coe_native_value_set (objd + nsub, value);
         }
      }

      /* Subindex 0 is padded to 16 bit if not object type VARIABLE.
       * For VARIABLE use true bitsize.
       */
      size +=
      ((nsub == 0) && (SDOobjects[nidx].objtype != OTYPE_VAR)) ? 16 : bitlen;
      nsub++;

      if ((max_bytes > 0) && (BITS2BYTES(size) >= max_bytes))
      {
         break;
      }
   }

   return size;
}

static void init_coesdo(esc_octet_t *coesdo,
                        uint8_t sdoservice,
                        uint8_t command,
                        uint16_t index,
                        uint8_t subindex)
{
   coe_init_frame (coesdo, sdoservice, command, index, subindex);
}

/** Function for responding on requested SDO Upload with Complete Access,
 *  sending the content requested in a free Mailbox buffer. Depending of
 *  size of data expedited, normal or segmented transfer is used.
 *  On error an SDO Abort will be sent.
 */
static void SDO_upload_complete_access (void)
{
   esc_octet_t *coesdo = &MBX[0];
   uint16_t index;
   uint8_t subindex;
   int32_t nidx;
   int16_t nsub;
   uint32_t abortcode = complete_access_get_variables
                           (coesdo, &index, &subindex, &nidx, &nsub);
   if (abortcode != 0)
   {
      set_state_idle (0, index, subindex, abortcode);
      return;
   }

   uint8_t MBXout = ESC_claimbuffer ();
   if (MBXout == 0)
   {
      /* It is a bad idea to call SDO_abort when ESC_claimbuffer fails,
       * because SDO_abort will also call ESC_claimbuffer ...
       */
      set_state_idle (0, index, subindex, 0);
      return;
   }

   const _objd *objd = SDOobjects[nidx].objdesc;

   /* loop through the subindexes to get the total size */
   uint32_t size = complete_access_subindex_loop(objd, nidx, nsub, NULL, UPLOAD, 0);

   /* expedited bits used calculation */
   uint8_t dss = (size > 24) ? 0 : (uint8_t)(4U * (3U - ((size - 1U) >> 3)));

   /* convert bits to bytes */
   size = BITS2BYTES(size);

   if (size > 0xffff)
   {
      /* 'size' is in this case actually an abort code */
      set_state_idle (MBXout, index, subindex, size);
      return;
   }

   /* check that upload data fits in the preallocated buffer */
   if ((size + PREALLOC_FACTOR * COE_HEADERSIZE) > PREALLOC_BUFFER_SIZE)
   {
      set_state_idle (MBXout, index, subindex, ABORT_CA_NOT_SUPPORTED);
      return;
   }
   abortcode = ESC_upload_pre_objecthandler(index, subindex,
         objd->data, (size_t *)&size, objd->flags | COMPLETE_ACCESS_FLAG);
   if (abortcode != 0)
   {
      set_state_idle (MBXout, index, subindex, abortcode);
      return;
   }

   /* copy subindex data into the preallocated buffer */
   complete_access_subindex_loop(objd, nidx, nsub, ESCvar.mbxdata, UPLOAD, 0);

   esc_octet_t *coeres = &MBX[MBXout * ESC_MBXSIZE];
   init_coesdo(coeres, COE_SDORESPONSE,
         COE_COMMAND_UPLOADRESPONSE | COE_COMPLETEACCESS | COE_SIZE_INDICATOR,
         index, subindex);

   ESCvar.segmented = 0;

   if (size <= 4)
   {
      /* expedited response, i.e. length <= 4 bytes */
      coe_put8 (coeres, COE_O_COMMAND,
                coe_get8 (coeres, COE_O_COMMAND) |
                COE_EXPEDITED_INDICATOR | dss);
      esc_octet_copy (&coeres[COE_O_SIZE], ESCvar.mbxdata, size);
   }
   else
   {
      /* normal response, i.e. length > 4 bytes */
      coe_put32 (coeres, COE_O_SIZE, size);

      if ((size + COE_HEADERSIZE) > ESC_MBXDSIZE)
      {
         /* segmented transfer needed */
         /* set total size in bytes */
         ESCvar.frags = size;
         /* limit to mailbox size */
         size = ESC_MBXDSIZE - COE_HEADERSIZE;
         /* number of bytes done */
         ESCvar.fragsleft = size;
         /* signal segmented transfer */
         ESCvar.segmented = MBXSEU;
         ESCvar.data = ESCvar.mbxdata;
         ESCvar.flags = COMPLETE_ACCESS_FLAG;
      }

      esc_mbx_set_length (coeres, COE_HEADERSIZE + size);
      esc_octet_copy (&coeres[COE_O_NORMAL_DATA], ESCvar.mbxdata, size);
   }

   if (ESCvar.segmented == 0)
   {
      abortcode = ESC_upload_post_objecthandler (index, subindex,
            objd->flags | COMPLETE_ACCESS_FLAG);

      if (abortcode != 0)
      {
         set_state_idle (MBXout, index, subindex, abortcode);
         return;
      }
   }

   MBXcontrol[MBXout].state = MBXstate_outreq;

   set_state_idle (MBXout, index, subindex, 0);
}

/** Function for handling the following SDO Upload if previous SDOUpload
 * response was flagged it needed to be segmented.
 */
static void SDO_uploadsegment (void)
{
   esc_octet_t *coesdo, *coeres;
   uint8_t MBXout;
   uint32_t size, offset, abort;
   coesdo = &MBX[0];
   MBXout = ESC_claimbuffer ();
   if (MBXout)
   {
      coeres = &MBX[MBXout * ESC_MBXSIZE];
      offset = ESCvar.fragsleft;
      size = ESCvar.frags - ESCvar.fragsleft;
      uint8_t command = COE_COMMAND_UPLOADSEGMENT |
            (coe_get8 (coesdo, COE_O_COMMAND) & COE_TOGGLEBIT);
      init_coesdo(coeres, COE_SDORESPONSE, command,
            coe_get16 (coesdo, COE_O_INDEX),
            coe_get8 (coesdo, COE_O_SUBINDEX));
      if ((size + COE_SEGMENTHEADERSIZE) > ESC_MBXDSIZE)
      {
         /* more segmented transfer needed */
         /* limit to mailbox size */
         size = ESC_MBXDSIZE - COE_SEGMENTHEADERSIZE;
         /* number of bytes done */
         ESCvar.fragsleft += size;
         esc_mbx_set_length (coeres, COE_SEGMENTHEADERSIZE + size);
      }
      else
      {
         /* last segment */
         ESCvar.segmented = 0;
         ESCvar.frags = 0;
         ESCvar.fragsleft = 0;
         coe_put8 (coeres, COE_O_COMMAND,
                   coe_get8 (coeres, COE_O_COMMAND) |
                   COE_COMMAND_LASTSEGMENTBIT);
         if (size >= 7)
         {
            esc_mbx_set_length (coeres, COE_SEGMENTHEADERSIZE + size);
         }
         else
         {
            coe_put8 (coeres, COE_O_COMMAND,
                      coe_get8 (coeres, COE_O_COMMAND) |
                      (uint8_t)((7U - size) << 1));
            esc_mbx_set_length (coeres, COE_DEFAULTLENGTH);
         }
      }
      if (ESCvar.flags == COMPLETE_ACCESS_FLAG)
      {
         esc_octet_copy (&coeres[COE_O_SEGMENT_DATA],
                         &((esc_octet_t *)ESCvar.data)[offset], size);
      }
      else
      {
         coe_object_to_wire (coe_segmented_obj,
                             &coeres[COE_O_SEGMENT_DATA], offset, size);
      }

      if (ESCvar.segmented == 0)
      {
         abort = ESC_upload_post_objecthandler (
               coe_get16 (coesdo, COE_O_INDEX),
               coe_get8 (coesdo, COE_O_SUBINDEX), ESCvar.flags);
         if (abort != 0)
         {
            set_state_idle (MBXout, coe_get16 (coesdo, COE_O_INDEX),
                            coe_get8 (coesdo, COE_O_SUBINDEX), abort);
            return;
         }
      }

      MBXcontrol[MBXout].state = MBXstate_outreq;
   }
   MBXcontrol[0].state = MBXstate_idle;
   ESCvar.xoe = 0;
}


/** Function for handling incoming requested SDO Download, validating the
 * request and sending an response. On error an SDO Abort will be sent.
 */
static void SDO_download (void)
{
   esc_octet_t *coesdo, *coeres;
   uint16_t index;
   uint8_t subindex;
   int32_t nidx;
   int16_t nsub;
   uint8_t MBXout;
   uint32_t size, actsize;
   const _objd *objd;
   esc_octet_t *mbxdata;
   uint32_t abort;

   coesdo = &MBX[0];
   index = coe_get16 (coesdo, COE_O_INDEX);
   subindex = coe_get8 (coesdo, COE_O_SUBINDEX);
   nidx = SDO_findobject (index);
   if (nidx >= 0)
   {
      nsub = SDO_findsubindex (nidx, subindex);
      if (nsub >= 0)
      {
         objd = SDOobjects[nidx].objdesc;
         uint8_t access = (objd + nsub)->flags & 0x3f;
         uint8_t state = ESCvar.ALstatus & 0x0f;
         if (WRITE_ACCESS(access, state))
         {
            /* expedited? */
            if (coe_get8 (coesdo, COE_O_COMMAND) & COE_EXPEDITED_INDICATOR)
            {
               size = 4U - ((coe_get8 (coesdo, COE_O_COMMAND) & 0x0CU) >> 2);
               mbxdata = &coesdo[COE_O_SIZE];
            }
            else
            {
               /* normal download */
               size = coe_get32 (coesdo, COE_O_SIZE) & 0xffffU;
               mbxdata = &coesdo[COE_O_NORMAL_DATA];
            }
            actsize = BITS2BYTES((objd + nsub)->bitlength);
            if (actsize != size)
            {
               /* entries with data types VISIBLE_STRING, OCTET_STRING,
                * UNICODE_STRING, ARRAY_OF_INT, ARRAY_OF_SINT,
                * ARRAY_OF_DINT, and ARRAY_OF_UDINT may have flexible length
                */
               uint16_t type = (objd + nsub)->datatype;
               if (type == DTYPE_VISIBLE_STRING)
               {
                  /* pad with zeroes up to the maximum size of the entry */
                  uint32_t i;
                  uint8_t *data = (uint8_t *)(objd + nsub)->data;
                  for (i = size; i < actsize; i++)
                  {
                     data[i] = 0;
                  }
               }
               else if ((type != DTYPE_OCTET_STRING) &&
                        (type != DTYPE_UNICODE_STRING) &&
                        (type != DTYPE_ARRAY_OF_INT) &&
                        (type != DTYPE_ARRAY_OF_SINT) &&
                        (type != DTYPE_ARRAY_OF_DINT) &&
                        (type != DTYPE_ARRAY_OF_UDINT))
               {
                  set_state_idle (0, index, subindex, ABORT_TYPEMISMATCH);
                  return;
               }
            }
            abort = ESC_download_pre_objecthandler (
                  index,
                  subindex,
                  mbxdata,
                  size,
                  (objd + nsub)->flags
            );
            if (abort == 0)
            {
               if ((size > 4) &&
                     (size > (esc_mbx_length (coesdo) - COE_HEADERSIZE)))
               {
                  uint32_t totalsize = size;
                  size = esc_mbx_length (coesdo) - COE_HEADERSIZE;
                  /* signal segmented transfer */
                  ESCvar.segmented = MBXSED;
                  ESCvar.frags = totalsize;
                  ESCvar.fragsleft = size;
                  esc_octet_copy (ESCvar.mbxdata, mbxdata, size);
                  ESCvar.data = ESCvar.mbxdata + size;
                  coe_segmented_obj = objd + nsub;
                  ESCvar.index = index;
                  ESCvar.subindex = subindex;
                  ESCvar.flags = (objd + nsub)->flags;
               }
               else
               {
                  ESCvar.segmented = 0;
               }
               if (ESCvar.segmented == 0)
               {
                  coe_object_from_wire (objd + nsub, mbxdata, 0, size);
               }
               MBXout = ESC_claimbuffer ();
               if (MBXout)
               {
                  coeres = &MBX[MBXout * ESC_MBXSIZE];
                  coe_init_frame (coeres, COE_SDORESPONSE,
                                  COE_COMMAND_DOWNLOADRESPONSE,
                                  index, subindex);
                  coe_put32 (coeres, COE_O_SIZE, 0);
                  MBXcontrol[MBXout].state = MBXstate_outreq;
               }
               if (ESCvar.segmented == 0)
               {
                  /* external object write handler */
                  abort = ESC_download_post_objecthandler (index, subindex, (objd + nsub)->flags);
                  if (abort != 0)
                  {
                     SDO_abort (MBXout, index, subindex, abort);
                  }
               }
            }
            else
            {
               SDO_abort (0, index, subindex, abort);
            }
         }
         else
         {
            if (access == ATYPE_RO)
            {
               SDO_abort (0, index, subindex, ABORT_READONLY);

            }
            else
            {
               SDO_abort (0, index, subindex, ABORT_NOTINTHISSTATE);
            }
         }
      }
      else
      {
         SDO_abort (0, index, subindex, ABORT_NOSUBINDEX);
      }
   }
   else
   {
      SDO_abort (0, index, subindex, ABORT_NOOBJECT);
   }
   MBXcontrol[0].state = MBXstate_idle;
   ESCvar.xoe = 0;
}

/** Function for handling incoming requested SDO Download with Complete Access,
 *  validating the request and sending a response. On error an SDO Abort will
 *  be sent.
 */
static void SDO_download_complete_access (void)
{
   esc_octet_t *coesdo = &MBX[0];
   uint16_t index;
   uint8_t subindex;
   int32_t nidx;
   int16_t nsub;
   uint32_t abortcode = complete_access_get_variables
                           (coesdo, &index, &subindex, &nidx, &nsub);
   if (abortcode != 0)
   {
      set_state_idle (0, index, subindex, abortcode);
      return;
   }

   uint32_t bytes;
   esc_octet_t *mbxdata = &coesdo[COE_O_SIZE];

   if (coe_get8 (coesdo, COE_O_COMMAND) & COE_EXPEDITED_INDICATOR)
   {
      /* expedited download */
      bytes = 4U - ((coe_get8 (coesdo, COE_O_COMMAND) & 0x0CU) >> 2);
   }
   else
   {
      /* normal download */
      bytes = coe_get32 (coesdo, COE_O_SIZE) & 0xffffU;
      mbxdata = &coesdo[COE_O_NORMAL_DATA];
   }

   const _objd *objd = SDOobjects[nidx].objdesc;

   /* loop through the subindexes to get the total size */
   uint32_t size = complete_access_subindex_loop(objd, nidx, nsub, NULL, DOWNLOAD, 0);
   size = BITS2BYTES(size);
   if (size > 0xffff)
   {
      /* 'size' is in this case actually an abort code */
      set_state_idle (0, index, subindex, size);
      return;
   }
   /* The document ETG.1020 S (R) V1.3.0, chapter 12.2, states that
    * "The SDO Download Complete Access data length shall always match
    * the full current object size (defined by SubIndex0)".
    * But EtherCAT Conformance Test Tool doesn't follow this rule for some test
    * cases, which is the reason to here only check for 'less than or equal'.
    */
   else if (bytes <= size)
   {
      abortcode = ESC_download_pre_objecthandler(index, subindex, mbxdata,
            size, objd->flags | COMPLETE_ACCESS_FLAG);
      if (abortcode != 0)
      {
         set_state_idle (0, index, subindex, abortcode);
         return;
      }

      if ((bytes + COE_HEADERSIZE) > ESC_MBXDSIZE)
      {
         /* check that download data fits in the preallocated buffer */
         if ((bytes + PREALLOC_FACTOR * COE_HEADERSIZE) > PREALLOC_BUFFER_SIZE)
         {
             set_state_idle(0, index, subindex, ABORT_CA_NOT_SUPPORTED);
             return;
         }
         /* set total size in bytes */
         ESCvar.frags = bytes;
         /* limit to mailbox size */
         size = ESC_MBXDSIZE - COE_HEADERSIZE;
         /* number of bytes done */
         ESCvar.fragsleft = size;
         ESCvar.segmented = MBXSED;
         ESCvar.data = ESCvar.mbxdata + size;
         ESCvar.index = index;
         ESCvar.subindex = subindex;
         ESCvar.flags = COMPLETE_ACCESS_FLAG;
         /* Store the data */
         esc_octet_copy (ESCvar.mbxdata, mbxdata, size);
      }
      else
      {
         ESCvar.segmented = 0;
         /* copy download data to subindexes */
         complete_access_subindex_loop(objd, nidx, nsub, mbxdata,
                                       DOWNLOAD, bytes);

         abortcode = ESC_download_post_objecthandler(index, subindex,
               objd->flags | COMPLETE_ACCESS_FLAG);
         if (abortcode != 0)
         {
            set_state_idle (0, index, subindex, abortcode);
            return;
         }
      }
   }
   else
   {
      set_state_idle (0, index, subindex, ABORT_TYPEMISMATCH);
      return;
   }

   uint8_t MBXout = ESC_claimbuffer ();
   if (MBXout > 0)
   {
      esc_octet_t *coeres = &MBX[MBXout * ESC_MBXSIZE];
      init_coesdo(coeres, COE_SDORESPONSE,
                  COE_COMMAND_DOWNLOADRESPONSE | COE_COMPLETEACCESS,
                  index, subindex);

      coe_put32 (coeres, COE_O_SIZE, 0);
      MBXcontrol[MBXout].state = MBXstate_outreq;
   }

   set_state_idle (MBXout, index, subindex, 0);
}

static void SDO_downloadsegment (void)
{
   esc_octet_t *coesdo = &MBX[0];
   uint8_t MBXout = ESC_claimbuffer ();
   if (MBXout)
   {
      esc_octet_t *coeres = &MBX[MBXout * ESC_MBXSIZE];
      uint32_t size = esc_mbx_length (coesdo) - COE_SEGMENTHEADERSIZE;
      if (size == 7)
      {
         size = 7U - ((coe_get8 (coesdo, COE_O_COMMAND) >> 1) & 7U);
      }
      uint8_t command = COE_COMMAND_DOWNLOADSEGRESP;
      uint8_t command2 = coe_get8 (coesdo, COE_O_COMMAND) & COE_TOGGLEBIT;
      command |= command2;
      init_coesdo(coeres, COE_SDORESPONSE, command, 0, 0);

      esc_octet_copy ((esc_octet_t *)ESCvar.data,
                      &coesdo[COE_O_SEGMENT_DATA], size);

      if (coe_get8 (coesdo, COE_O_COMMAND) & COE_COMMAND_LASTSEGMENTBIT)
      {
         if(ESCvar.flags == COMPLETE_ACCESS_FLAG)
         {
            int32_t nidx;
            int16_t nsub;

            if(ESCvar.frags > ESCvar.fragsleft + size)
            {
               set_state_idle (0, ESCvar.index, ESCvar.subindex, ABORT_TYPEMISMATCH);
               return;
            }

            nidx = SDO_findobject(ESCvar.index);
            nsub = SDO_findsubindex (nidx, ESCvar.subindex);

            if ((nidx < 0) || (nsub < 0))
            {
               set_state_idle (0, ESCvar.index, ESCvar.subindex, ABORT_NOOBJECT);
               return;
            }

            /* copy download data to subindexes */
            const _objd *objd = SDOobjects[nidx].objdesc;
            complete_access_subindex_loop(objd,
                  nidx,
                  nsub,
                  ESCvar.mbxdata,
                  DOWNLOAD,
                  ESCvar.frags);

         }
         else
         {
            coe_object_from_wire (coe_segmented_obj, ESCvar.mbxdata, 0,
                                  ESCvar.fragsleft + size);
         }
         /* last segment */
         ESCvar.segmented = 0;
         ESCvar.frags = 0;
         ESCvar.fragsleft = 0;
         /* external object write handler */
         uint32_t abort = ESC_download_post_objecthandler
               (ESCvar.index, ESCvar.subindex, ESCvar.flags);
         if (abort != 0)
         {
            set_state_idle (MBXout, ESCvar.index, ESCvar.subindex, abort);
            return;
         }
      }
      else
      {
         /* more segmented transfer needed: increase offset */
         ESCvar.data = (esc_octet_t *)ESCvar.data + size;
         /* number of bytes done */
         ESCvar.fragsleft += size;
      }

      MBXcontrol[MBXout].state = MBXstate_outreq;
   }

   set_state_idle (0, 0, 0, 0);
}

/** Function for sending an SDO Info Error reply.
 *
 * @param[in] abortcode  = = abort code to send in reply
 */
static void SDO_infoerror (uint32_t abortcode)
{
   uint8_t MBXout;
   esc_octet_t *coeres;
   MBXout = ESC_claimbuffer ();
   if (MBXout)
   {
      coeres = &MBX[MBXout * ESC_MBXSIZE];
      esc_mbx_set_length (coeres, COE_HEADERSIZE);
      coe_info_init (coeres, COE_INFOERROR, false, 0);
      coe_put32 (coeres, COE_INFO_O_INDEX, abortcode);
      MBXcontrol[MBXout].state = MBXstate_outreq;
      MBXcontrol[0].state = MBXstate_idle;
      ESCvar.xoe = 0;
   }
}

#define ODLISTSIZE  ((uint32_t)(ESC_MBX1_sml - 14U) & 0xfffeU)

/** Function for handling incoming requested SDO Get OD List, validating the
 * request and sending an response. On error an SDO Info Error will be sent.
 */
static void SDO_getodlist (void)
{
   uint32_t frags;
   uint8_t MBXout = 0;
   uint16_t entries = 0;
   uint16_t i, n;
   uint16_t offset;
   esc_octet_t *coel, *coer;

   while (SDOobjects[entries].index != 0xffff)
   {
      entries++;
   }
   ESCvar.entries = entries;
   frags = ((uint32_t)(entries << 1) + ODLISTSIZE - 1U);
   frags /= ODLISTSIZE;
   coer = &MBX[0];
   /* check for unsupported opcodes */
   if (coe_get16 (coer, COE_INFO_O_INDEX) > 0x01)
   {
      SDO_infoerror (ABORT_UNSUPPORTED);
   }
   else
   {
      MBXout = ESC_claimbuffer ();
   }
   if (MBXout)
   {
      coel = &MBX[MBXout * ESC_MBXSIZE];
      /* number of objects request */
      if (coe_get16 (coer, COE_INFO_O_INDEX) == 0x00)
      {
         coe_info_init (coel, COE_GETODLISTRESPONSE, false, 0);
         coe_put16 (coel, COE_INFO_O_INDEX, 0);
         MBXcontrol[0].state = MBXstate_idle;
         ESCvar.xoe = 0;
         ESCvar.frags = frags;
         ESCvar.fragsleft = frags - 1;
         coe_put16 (coel, COE_OBJ_O_DATATYPE, entries);
         esc_octet_clear (&coel[COE_OBJ_O_DATATYPE + 2U], 8U);
         esc_mbx_set_length (coel, 0x08U + (5U << 1));
      }
      /* only return all objects */
      if (coe_get16 (coer, COE_INFO_O_INDEX) == 0x01)
      {
         if (frags > 1)
         {
            coe_info_init (coel, COE_GETODLISTRESPONSE, true,
                           (uint16_t)(frags - 1U));
            ESCvar.xoe = MBXCOE + MBXODL;
            n = ODLISTSIZE >> 1;
         }
         else
         {
            coe_info_init (coel, COE_GETODLISTRESPONSE, false, 0);
            MBXcontrol[0].state = MBXstate_idle;
            ESCvar.xoe = 0;
            n = entries;
         }
         ESCvar.frags = frags;
         ESCvar.fragsleft = frags - 1;
         coe_put16 (coel, COE_INFO_O_FRAGMENTS,
                    (uint16_t)ESCvar.fragsleft);
         coe_put16 (coel, COE_INFO_O_INDEX, 0x01);

         offset = COE_OBJ_O_DATATYPE;
         for (i = 0; i < n; i++)
         {
            coe_put16 (coel, offset, SDOobjects[i].index);
            offset += 2U;
         }

         esc_mbx_set_length (coel, 0x08U + (n << 1));
      }
      MBXcontrol[MBXout].state = MBXstate_outreq;
   }
}
/** Function for continuing sending left overs from previous requested
 * SDO Get OD List, validating the request and sending an response.
 */
static void SDO_getodlistcont (void)
{
   uint8_t MBXout;
   uint16_t i, n, s;
   uint16_t offset;
   esc_octet_t *coel;

   MBXout = ESC_claimbuffer ();
   if (MBXout)
   {
      coel = &MBX[MBXout * ESC_MBXSIZE];
      s = (uint16_t)((ESCvar.frags - ESCvar.fragsleft) * (ODLISTSIZE >> 1));
      if (ESCvar.fragsleft > 1)
      {
         coe_info_init (coel, COE_GETODLISTRESPONSE, true,
                        (uint16_t)(ESCvar.fragsleft - 1U));
         n = (uint16_t)(s + (ODLISTSIZE >> 1));
      }
      else
      {
         coe_info_init (coel, COE_GETODLISTRESPONSE, false, 0);
         MBXcontrol[0].state = MBXstate_idle;
         ESCvar.xoe = 0;
         n = ESCvar.entries;
      }
      ESCvar.fragsleft--;
      coe_put16 (coel, COE_INFO_O_FRAGMENTS,
                 (uint16_t)ESCvar.fragsleft);
      /* pointer 2 bytes back to exclude index */
      offset = COE_INFO_O_INDEX;
      for (i = s; i < n; i++)
      {
         coe_put16 (coel, offset, SDOobjects[i].index);
         offset += 2U;
      }
      esc_mbx_set_length (coel, 0x06U + ((n - s) << 1));
      MBXcontrol[MBXout].state = MBXstate_outreq;
   }
}

/** Function for handling incoming requested SDO Get Object Description,
 * validating the request and sending an response. On error an
 * SDO Info Error will be sent.
 */
static void SDO_getod (void)
{
   uint8_t MBXout;
   uint16_t index;
   int32_t nidx;
   const char *s;
   uint8_t n = 0;
   esc_octet_t *coer, *coel;
   coer = &MBX[0];
   index = coe_get16 (coer, COE_INFO_O_INDEX);
   nidx = SDO_findobject (index);
   if (nidx >= 0)
   {
      MBXout = ESC_claimbuffer ();
      if (MBXout)
      {
         coel = &MBX[MBXout * ESC_MBXSIZE];
         coe_info_init (coel, COE_GETODRESPONSE, false, 0);
         coe_put16 (coel, COE_INFO_O_INDEX, index);
         if (SDOobjects[nidx].objtype == OTYPE_VAR)
         {
            int32_t nsub = SDO_findsubindex (nidx, 0);
            const _objd *objd = SDOobjects[nidx].objdesc;
            coe_put16 (coel, COE_OBJ_O_DATATYPE,
                       (objd + nsub)->datatype);
            coe_put8 (coel, COE_OBJ_O_MAXSUB, SDOobjects[nidx].maxsub);
         }
         else if (SDOobjects[nidx].objtype == OTYPE_ARRAY)
         {
            int32_t nsub = SDO_findsubindex (nidx, 0);
            const _objd *objd = SDOobjects[nidx].objdesc;
            coe_put16 (coel, COE_OBJ_O_DATATYPE,
                       (objd + nsub)->datatype);
            coe_put8 (coel, COE_OBJ_O_MAXSUB,
                      (uint8_t)SDOobjects[nidx].objdesc->value);
         }
         else
         {
            coe_put16 (coel, COE_OBJ_O_DATATYPE, 0);
            coe_put8 (coel, COE_OBJ_O_MAXSUB,
                      (uint8_t)SDOobjects[nidx].objdesc->value);
         }
         coe_put8 (coel, COE_OBJ_O_OBJECTCODE,
                   (uint8_t)SDOobjects[nidx].objtype);
         s = SDOobjects[nidx].name;
         while (*s && (n < (ESC_MBXDSIZE - 0x0c)))
         {
            coe_put8 (coel, COE_OBJ_O_NAME + n, (uint8_t)*s);
            n++;
            s++;
         }
         coe_put8 (coel, COE_OBJ_O_NAME + n, (uint8_t)*s);
         esc_mbx_set_length (coel, 0x0CU + n);
         MBXcontrol[MBXout].state = MBXstate_outreq;
         MBXcontrol[0].state = MBXstate_idle;
         ESCvar.xoe = 0;
      }
   }
   else
   {
      SDO_infoerror (ABORT_NOOBJECT);
   }
}

/** Function for handling incoming requested SDO Get Entry Description,
 * validating the request and sending an response. On error an
 * SDO Info Error will be sent.
 */
static void SDO_geted (void)
{
   uint8_t MBXout;
   uint16_t index;
   int32_t nidx;
   int16_t nsub;
   uint8_t subindex;
   const char *s;
   const _objd *objd;
   uint8_t n = 0;
   esc_octet_t *coer, *coel;
   coer = &MBX[0];
   index = coe_get16 (coer, COE_INFO_O_INDEX);
   subindex = coe_get8 (coer, COE_ENT_O_SUBINDEX);
   nidx = SDO_findobject (index);
   if (nidx >= 0)
   {
      nsub = SDO_findsubindex (nidx, subindex);
      if (nsub >= 0)
      {
         objd = SDOobjects[nidx].objdesc;
         MBXout = ESC_claimbuffer ();
         if (MBXout)
         {
            coel = &MBX[MBXout * ESC_MBXSIZE];
            coe_info_init (coel, COE_ENTRYDESCRIPTIONRESPONSE, false, 0);
            coe_put16 (coel, COE_INFO_O_INDEX, index);
            coe_put8 (coel, COE_ENT_O_SUBINDEX, subindex);
            coe_put8 (coel, COE_ENT_O_VALUEINFO,
                      COE_VALUEINFO_ACCESS + COE_VALUEINFO_OBJECT +
                      COE_VALUEINFO_MAPPABLE);
            coe_put16 (coel, COE_ENT_O_DATATYPE,
                       (objd + nsub)->datatype);
            coe_put16 (coel, COE_ENT_O_BITLENGTH,
                       (objd + nsub)->bitlength);
            coe_put16 (coel, COE_ENT_O_ACCESS, (objd + nsub)->flags);
            s = (objd + nsub)->name;
            while (*s && (n < (ESC_MBXDSIZE - 0x10)))
            {
               coe_put8 (coel, COE_ENT_O_NAME + n, (uint8_t)*s);
               n++;
               s++;
            }
            coe_put8 (coel, COE_ENT_O_NAME + n, (uint8_t)*s);
            esc_mbx_set_length (coel, 0x10U + n);
            MBXcontrol[MBXout].state = MBXstate_outreq;
            MBXcontrol[0].state = MBXstate_idle;
            ESCvar.xoe = 0;
         }
      }
      else
      {
         SDO_infoerror (ABORT_NOSUBINDEX);
      }
   }
   else
   {
      SDO_infoerror (ABORT_NOOBJECT);
   }
}

/** Main CoE function checking the status on current mailbox buffers carrying
 * data, distributing the mailboxes to appropriate CoE functions.
 * On Error an MBX_error or SDO Abort will be sent depending on error cause.
 */
void ESC_coeprocess (void)
{
   esc_octet_t *coesdo;
   uint8_t command;
   uint16_t service;
   if (ESCvar.MBXrun == 0)
   {
      return;
   }
   if (!ESCvar.xoe && (MBXcontrol[0].state == MBXstate_inclaim))
   {
      if (esc_mbx_type (&MBX[0]) == MBXCOE)
      {
         if (esc_mbx_length (&MBX[0]) < COE_MINIMUM_LENGTH)
         {
            MBX_error (MBXERR_INVALIDSIZE);
         }
         else
         {
            ESCvar.xoe = MBXCOE;
         }
      }
   }
   if ((ESCvar.xoe == (MBXCOE + MBXODL)) && (!ESCvar.mbxoutpost))
   {
      /* continue get OD list */
      SDO_getodlistcont ();
   }
   if (ESCvar.xoe == MBXCOE)
   {
      coesdo = &MBX[0];
      command = coe_get8 (coesdo, COE_O_COMMAND);
      service = coe_get16 (coesdo, COE_O_NUMBER_SERVICE) >> 12;
      if (service == COE_SDOREQUEST)
      {
         if ((SDO_COMMAND(command) == COE_COMMAND_UPLOADREQUEST)
               && (esc_mbx_length (coesdo) == COE_HEADERSIZE))
         {
            /* initiate SDO upload request */
            if (SDO_COMPLETE_ACCESS(command))
            {
               SDO_upload_complete_access ();
            }
            else
            {
               SDO_upload ();
            }
         }
         else if (((command & 0xefU) == COE_COMMAND_UPLOADSEGREQ)
               && (esc_mbx_length (coesdo) == COE_HEADERSIZE)
               && (ESCvar.segmented == MBXSEU))
         {
            /* SDO upload segment request */
            SDO_uploadsegment ();
         }
         else if (SDO_COMMAND(command) == COE_COMMAND_DOWNLOADREQUEST)
         {
            /* initiate SDO download request */
            if (SDO_COMPLETE_ACCESS(command))
            {
               SDO_download_complete_access ();
            }
            else
            {
               SDO_download ();
            }
         }
         else if (SDO_COMMAND(command) == COE_COMMAND_DOWNLOADSEGREQ)
         {
            /* SDO download segment request */
            SDO_downloadsegment ();
         }
      }
      /* initiate SDO get OD list */
      else
      {
         if ((service == COE_SDOINFORMATION)
               && ((coe_get8 (coesdo, COE_INFO_O_OPCODE) & 0x7fU) == 0x01U))
         {
            SDO_getodlist ();
         }
         /* initiate SDO get OD */
         else
         {
            if ((service == COE_SDOINFORMATION)
                  && ((coe_get8 (coesdo, COE_INFO_O_OPCODE) & 0x7fU) == 0x03U))
            {
               SDO_getod ();
            }
            /* initiate SDO get ED */
            else
            {
               if ((service == COE_SDOINFORMATION)
                     && ((coe_get8 (coesdo, COE_INFO_O_OPCODE) & 0x7fU) == 0x05U))
               {
                  SDO_geted ();
               }
               else
               {
                  /* COE not recognised above */
                  if (ESCvar.xoe == MBXCOE)
                  {
                     if (service == 0)
                     {
                        MBX_error (MBXERR_INVALIDHEADER);
                     }
                     else
                     {
                        SDO_abort (0, coe_get16 (coesdo, COE_O_INDEX),
                                   coe_get8 (coesdo, COE_O_SUBINDEX),
                                   ABORT_UNSUPPORTED);
                     }
                     MBXcontrol[0].state = MBXstate_idle;
                     ESCvar.xoe = 0;
                  }
               }
            }
         }
      }
   }
}

/**
 * Get value from bitmap
 *
 * This function gets a value from a bitmap.
 *
 * @param[in] bitmap = bitmap containing value
 * @param[in] offset = start offset
 * @param[in] length = number of bits to get
 * @return bitslice value
 */
static uint64_t COE_bitsliceGet (const esc_octet_t *bitmap,
                                 unsigned int offset, unsigned int length)
{
   uint64_t value = 0;
   unsigned int bit;

   for (bit = 0; bit < length; bit++)
   {
      unsigned int source = offset + bit;
      if ((esc_octet_get (bitmap, source >> 3) &
           (uint8_t)(1U << (source & 7U))) != 0U)
      {
         value |= (uint64_t)1U << bit;
      }
   }
   return value;
}

/**
 * Set value in bitmap
 *
 * This function sets a value in a bitmap.
 *
 * @param[in] bitmap = bitmap to contain value
 * @param[in] offset = start offset
 * @param[in] length = number of bits to set
 * @param[in] value  = value to set
 */
static void COE_bitsliceSet (esc_octet_t *bitmap, unsigned int offset,
                             unsigned int length,
                             uint64_t value)
{
   unsigned int bit;
   for (bit = 0; bit < length; bit++)
   {
      unsigned int target = offset + bit;
      uint16_t octet = target >> 3;
      uint8_t mask = (uint8_t)(1U << (target & 7U));
      uint8_t current = esc_octet_get (bitmap, octet);
      current = ((value >> bit) & 1U) ? (current | mask) : (current & ~mask);
      esc_octet_set (bitmap, octet, current);
   }
}

/**
 * Get object value
 *
 * This function atomically gets an object value.
 *
 * @param[in] obj   = object description
 * @return object value
 */
static uint64_t COE_getValue (const _objd * obj)
{
   return coe_native_value_get (obj);
}

/**
 * Set object value
 *
 * This function atomically sets an object value.
 *
 * @param[in] obj   = object description
 * @param[in] value = new value
 */
static void COE_setValue (const _objd * obj, uint64_t value)
{
   coe_native_value_set (obj, value);
}

/**
 * Init default values for SDO objects
 */
void COE_initDefaultValues (void)
{
   int i;
   const _objd *objd;
   int n;
   uint8_t maxsub;

   /* Let application decide if initialization will be skipped */
   if (ESCvar.skip_default_initialization)
   {
      return;
   }

   /* Set default values from object descriptor */
   for (n = 0; SDOobjects[n].index != 0xffff; n++)
   {
      objd = SDOobjects[n].objdesc;
      maxsub = SDOobjects[n].maxsub;

      i = 0;
      do
      {
         if (objd[i].data != NULL)
         {
            COE_setValue (&objd[i], objd[i].value);
            DPRINT ("%04"PRIx32":%02"PRIx32" = %"PRIx32"\n",
                  SDOobjects[n].index,
                  objd[i].subindex,
                  objd[i].value);
         }
      } while (objd[i++].subindex < maxsub);
   }

   /* Let application override default values */
   if (ESCvar.set_defaults_hook != NULL)
   {
      ESCvar.set_defaults_hook();
   }
}

/**
 * Pack process data
 *
 * This function reads mapped objects and constructs the process data
 * inputs (TXPDO).
 *
 * @param[in] buffer     = input process data
 * @param[in] nmappings  = number of mappings in sync manager
 * @param[in] mappings   = list of mapped objects in sync manager
 */
void COE_pdoPack (uint8_t * buffer, int nmappings, _SMmap * mappings)
{
   int ix;

   for (ix = 0; ix < nmappings; ix++)
   {
      const _objd * obj = mappings[ix].obj;
      uint32_t offset = mappings[ix].offset;

      if (obj != NULL)
      {
         if (obj->bitlength > 64)
         {
            const uint8_t *source = (const uint8_t *)obj->data;
            uint32_t bit;
            for (bit = 0; bit < obj->bitlength; bit++)
            {
               COE_bitsliceSet (buffer, offset + bit, 1U,
                  (source[bit >> 3] >> (bit & 7U)) & 1U);
            }
         }
         else
         {
            /* Atomically get object value */
            uint64_t value = COE_getValue (obj);
            COE_bitsliceSet (
               buffer,
               offset,
               obj->bitlength,
               value
            );
         }
      }
   }
}

/**
 * Unpack process data
 *
 * This function unpacks process data output (RXPDO) and writes to the
 * mapped objects.
 *
 * @param[in] buffer    = output process data
 * @param[in] nmappings = number of mappings in sync manager
 * @param[in] mappings  = list of mapped objects in sync manager
 */
void COE_pdoUnpack (uint8_t * buffer, int nmappings, _SMmap * mappings)
{
   int ix;

   for (ix = 0; ix < nmappings; ix++)
   {
      const _objd * obj = mappings[ix].obj;
      uint32_t offset = mappings[ix].offset;

      if (obj != NULL)
      {
         if (obj->bitlength > 64)
         {
            uint8_t *dest = (uint8_t *)obj->data;
            uint32_t bit;
            for (bit = 0; bit < obj->bitlength; bit++)
            {
               uint8_t mask = (uint8_t)(1U << (bit & 7U));
               uint8_t current = dest[bit >> 3];
               current = COE_bitsliceGet (buffer, offset + bit, 1U) ?
                         (current | mask) : (current & ~mask);
               dest[bit >> 3] = current;
            }
         }
         else
         {
            /* Atomically set object value */
            uint64_t value = COE_bitsliceGet (
               buffer,
               offset,
               obj->bitlength
            );
            COE_setValue (obj, value);
         }
      }
   }
}

/**
 * Fetch max subindex
 *
 * This function fetches the value of subindex 0 (max subindex).
 *
 * @param[in] index = object index
 */
uint8_t COE_maxSub (uint16_t index)
{
   int32_t nidx;
   uint8_t maxsub;

   nidx = SDO_findobject (index);
   if (nidx == -1)
      return 0;

   maxsub = OBJ_VALUE_FETCH (maxsub, SDOobjects[nidx].objdesc[0]);
   return maxsub;
}
