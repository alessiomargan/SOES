/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

 /** \file
 * \brief
 * File over EtherCAT (FoE) module.
 */


#include <cc.h>
#include "esc.h"
#include "esc_foe.h"
#include <string.h>

#ifdef ESC_DEBUG
#ifndef FOE_DEBUG
    #undef DPRINT
    #define DPRINT(...)
#endif
#endif

 /** \file
 * \brief
 * File over EtherCAT (FoE) module.
 *
 * FOE read / write and FOE service functions
 */

//define if FOE_read should be supported
//#define FOE_READ_SUPPORTED

/** Variable holding current filename read at FOE Open.
 */
static char foe_file_name[FOE_FN_MAX + 1];


/** Main FoE configuration pointer data array. Structure is allocated and filled
 * by the application defining what preferences it requires.
 */
static foe_cfg_t * foe_cfg;
/** Pointer to current file configuration item used by FoE.
 */
static foe_file_cfg_t * foe_file;
/** Main FoE status data array. Structure gets filled with current status
 * variables during FoE usage.
 */
static _FOEvar FOEvar;

/* FoE wire offsets, expressed in EtherCAT octets. */
#define FOE_O_OPCODE   ESC_MBXHSIZE
#define FOE_O_RESERVED (FOE_O_OPCODE + 1U)
#define FOE_O_VALUE    (FOE_O_OPCODE + 2U)
#define FOE_O_DATA     (ESC_MBXHSIZE + ESC_FOEHSIZE)

static uint8_t foe_opcode (const esc_octet_t *mailbox)
{
   return (uint8_t)esc_octet_get (mailbox, FOE_O_OPCODE);
}

static void foe_set_header (esc_octet_t *mailbox, uint8_t opcode,
                            uint32_t value)
{
   esc_octet_set (mailbox, FOE_O_OPCODE, opcode);
   esc_octet_set (mailbox, FOE_O_RESERVED, 0U);
   esc_put_le32 (mailbox, FOE_O_VALUE, value);
}

static uint32_t foe_value (const esc_octet_t *mailbox)
{
   return esc_get_le32 (mailbox, FOE_O_VALUE);
}

static esc_octet_t *foe_data (esc_octet_t *mailbox)
{
   return &mailbox[FOE_O_DATA];
}

/** Validate a write or read request by checking filename and password.
 *
 * @param[in] name      = Filename
 * @param[in] num_chars = Length of filename
 * @param[in] pass      = Numeric variable of password
 * @param[in] op        = Request op-code
 * @return 0= if we succeed, FOE_ERR_NOTFOUND something wrong with filename or
 * password
 */
static uint32_t FOE_fopen (const esc_octet_t *name, uint8_t num_chars,
                           uint32_t pass, uint8_t op)
{
   uint32_t i;

   /* Unpack the file name into characters we can look at. */
   if (num_chars > FOE_FN_MAX)
   {
      num_chars = FOE_FN_MAX;
   }

   for (i = 0; i < num_chars; i++)
   {
      foe_file_name[i] = (char)esc_octet_get (name, i);
   }
   foe_file_name[i] = '\0';

   DPRINT("%s %d 0x%"PRIX32"\n", foe_file_name, num_chars, pass);

   /* Figure out what file they're talking about. */
   for (i = 0; i < foe_cfg->n_files; i++)
   {
      if (0 == strncmp (foe_file_name, foe_cfg->files[i].name, num_chars))
      {
         if (pass != foe_cfg->files[i].filepass)
         {
            return FOE_ERR_NORIGHTS;
         }

         if (op == FOE_OP_WRQ &&
             (foe_cfg->files[i].write_only_in_boot) &&
             (ESCvar.ALstatus != ESCboot))
         {
            return FOE_ERR_NOTINBOOTSTRAP;
         }

         foe_file = &foe_cfg->files[i];
         foe_file->address_offset = 0;
         foe_file->total_size = 0;

         if ( foe_file->on_foe_open != 0 ) {
        	 if ( foe_file->on_foe_open( op ) ) {
        		 return FOE_ERR_PROGERROR;
        	 }
         }

         switch (op)
         {
            case FOE_OP_RRQ:
            {
            	if ( foe_file->read_function == 0) {
            		return FOE_ERR_ACCESS;
            	}
            	FOEvar.fposition = 0;
            	FOEvar.fend = foe_cfg->files[i].max_data;
            	return 0;
            }
            case FOE_OP_WRQ:
            {
               FOEvar.fposition = 0;
               FOEvar.fend = foe_cfg->files[i].max_data;
               return 0;
            }
         }
      }
   }

   return FOE_ERR_NOTFOUND;
}

#ifdef FOE_READ_SUPPORTED
/** Function writing local data to mailbox buffer to be sent as next FoE frame.
 * It will try to fill the Mailbox buffer available if there is enough data
 * left to read.
 *
 * @param[in] data      = pointer to buffer
 * @param[in] maxlength = max length of data possible to read, controlled by
 * Mailbox - FoE and Mailbox frame headers.

 * @return Number of copied bytes.
 */
static uint32_t FOE_fread (esc_octet_t * data, uint32_t maxlength)
{
   uint32_t ncopied = 0;

   DPRINT("%s %d %d %d %d\n", __FUNCTION__,
		   maxlength, FOEvar.fend, FOEvar.fposition, FOEvar.fbufposition);
   FOEvar.fprevposition = FOEvar.fposition;
   while (maxlength && (FOEvar.fend - FOEvar.fposition))
   {
       if( (foe_file->address_offset == 0) ||
    	   (FOEvar.fbufposition >= foe_cfg->buffer_size) )
       {
          foe_file->read_function (foe_file, foe_cfg->fbuffer, FOEvar.fbufposition);
          FOEvar.fbufposition = 0;
          foe_file->address_offset += foe_cfg->buffer_size;
       }

       maxlength--;
       esc_octet_set (data++, 0U,
                      foe_cfg->fbuffer[FOEvar.fbufposition++]);

	   FOEvar.fposition++;
	   ncopied++;
   }

   return ncopied;
}
#endif

/** Function reading mailbox buffer to local buffer to be handled by
 * application write hook. Ex. flash routine used by software update.
 * It will consume the buffer and call the write hook every time the configured
 * flush buffer limit is reached.
 *
 *
 * @param[in] data   = Pointer to buffer
 * @param[in] length = Length of data to read

 * @return Number of copied bytes.
 */
static uint32_t FOE_fwrite (const esc_octet_t *data, uint32_t length)
{
    uint32_t ncopied = 0;
    uint32_t failed = 0;

    DPRINT("%s %d %d %d %d\n", __FUNCTION__,
 		   length, FOEvar.fend, FOEvar.fposition, FOEvar.fbufposition);
    FOEvar.fprevposition = FOEvar.fposition;
    while (length && (FOEvar.fend - FOEvar.fposition) && !failed)
    {
       length--;
       foe_cfg->fbuffer[FOEvar.fbufposition++] =
          (uint8_t)esc_octet_get (data++, 0U);
       if(FOEvar.fbufposition >= foe_cfg->buffer_size)
       {
          failed = foe_file->write_function (foe_file, foe_cfg->fbuffer, FOEvar.fbufposition);
          FOEvar.fbufposition = 0;
          foe_file->address_offset += foe_cfg->buffer_size;
       }
       FOEvar.fposition++;
       if(failed)
       {
          DPRINT("Failed FOE_fwrite ncopied=%"PRIu32"\n", ncopied);
       }
       else
       {
          ncopied++;
       }
    }

    foe_file->total_size += ncopied;

    DPRINT("FOE_fwrite END with : %"PRIu32"\n",ncopied);
    return ncopied;
}


/** Function handling the final FOE_fwrite when we close up regardless
 * if we have filled the buffers or not.
 *
 * @return Number of copied bytes on success, 0= if failed.
 */
static uint32_t FOE_fclose (void)
{
   uint32_t i;
   uint32_t failed = 0;

   DPRINT("%s\n",__FUNCTION__);
   if (FOEvar.fbufposition)
   {

      DPRINT("FOE_fclose EXTRA write\n");
      for(i = FOEvar.fbufposition; i < foe_cfg->buffer_size; i++)
      {
         foe_cfg->fbuffer[FOEvar.fbufposition++] = 0xFF;
      }
      failed = foe_file->write_function (foe_file, foe_cfg->fbuffer, FOEvar.fbufposition);
      FOEvar.fbufposition = 0;
      foe_file->address_offset += foe_cfg->buffer_size;
      DPRINT("FOE_fclose EXTRA write ended\n");
   }
   if ( failed == 0 && foe_file->on_foe_close != 0) {
   	   foe_file->on_foe_close();
   }
   return failed;
}

/** Initialize by clearing all current status variables.
 *
 */
void FOE_init ()
{
   DPRINT("FOE_init\n");
   FOEvar.foepacket = 0;
   FOEvar.foestate = FOE_READY;
   FOEvar.fposition = 0;
   FOEvar.fprevposition = 0;
   FOEvar.fbufposition = 0;
}

/** Function for sending an FOE abort frame.
 *
 * @param[in] code   = abort code
 */
static void FOE_abort (uint32_t code)
{
   esc_octet_t *mailbox;
   uint8_t mbxhandle;

   if (code)
   {
      /* Send back an error packet. */
      mbxhandle = ESC_claimbuffer ();
      if (mbxhandle)
      {
         mailbox = &MBX[mbxhandle * ESC_MBXSIZE];
         /* Don't bother with error text for now. */
         esc_mbx_set_length (mailbox, ESC_FOEHSIZE);
         esc_mbx_set_type (mailbox, MBXFOE);
         foe_set_header (mailbox, FOE_OP_ERR, code);
         MBXcontrol[mbxhandle].state = MBXstate_outreq;
      }
      /* Nothing we can do if we can't get an outbound mailbox. */
   }
   DPRINT("FOE_abort: 0x%"PRIX32"\n", code);
   FOE_init ();
}

#ifdef FOE_READ_SUPPORTED
/** Sends an FoE data frame, returning the number of data bytes
 * written or an error number.
 * Error numbers will be greater than FOE_DATA_SIZE.

 * @param[in] data   = pointer to buffer
 * @param[in] length = length of data to read

 * @return Number of data bytes written or an error number. Error numbers
 * will be greater than FOE_DATA_SIZE.
 */
static uint32_t FOE_send_data_packet ()
{
   esc_octet_t *mailbox;
   uint32_t data_len;
   uint8_t mbxhandle;

   mbxhandle = ESC_claimbuffer ();
   if (mbxhandle)
   {
      mailbox = &MBX[mbxhandle * ESC_MBXSIZE];
      data_len = FOE_fread (foe_data (mailbox), ESC_FOE_DATA_SIZE);
      foe_set_header (mailbox, FOE_OP_DATA, FOEvar.foepacket);
      FOEvar.foepacket++;
      esc_mbx_set_length (mailbox, (uint16_t)(data_len + ESC_FOEHSIZE));
      esc_mbx_set_type (mailbox, MBXFOE);
      /* Mark the outbound mailbox as filled. */
      MBXcontrol[mbxhandle].state = MBXstate_outreq;
      return data_len;
   }
   else
   {
      return FOE_ERR_PROGERROR;
   }
}
#endif

/** Sends an FoE ack data frame.

 * @return 0= or error number.
 */
static uint32_t FOE_send_ack ()
{
   esc_octet_t *mailbox;
   uint8_t mbxhandle;

   mbxhandle = ESC_claimbuffer ();
   if (mbxhandle)
   {
      DPRINT("FOE_send_ack\n");
      mailbox = &MBX[mbxhandle * ESC_MBXSIZE];
      esc_mbx_set_length (mailbox, ESC_FOEHSIZE);
      esc_mbx_set_type (mailbox, MBXFOE);
      foe_set_header (mailbox, FOE_OP_ACK, FOEvar.foepacket);
      FOEvar.foepacket++;
      MBXcontrol[mbxhandle].state = MBXstate_outreq;
      return 0;
   }
   else
   {
      DPRINT("ERROR:FOE_send_ack\n");
      return FOE_ERR_PROGERROR;
   }
}

/* Handlers for various FoE states. */

#ifdef FOE_READ_SUPPORTED
/** FoE read request handler. Starts with Initialize, Open and Sending one frame.
 * When first frame have been sent we will send data from Ack.
 * On error we will send FOE Abort.
 *
 */
static void FOE_read ()
{
   esc_octet_t *mailbox;
   uint32_t password;
   uint32_t res;
   uint8_t data_len;

   if (FOEvar.foestate != FOE_READY)
   {
      FOE_abort (FOE_ERR_ILLEGAL);
      return;
   }

   FOE_init ();
   mailbox = &MBX[0];
   /* Get the length of the file name in octets. */
   data_len = (uint8_t)(esc_mbx_length (mailbox) - ESC_FOEHSIZE);
   password = foe_value (mailbox);

   res = FOE_fopen (foe_data (mailbox), data_len, password, FOE_OP_RRQ);
   if (res == 0)
   {
      FOEvar.foepacket = 1;
      /*
       * Attempt to send the packet
       */
      res = FOE_send_data_packet ();
      if (res <= ESC_FOE_DATA_SIZE)
      {
         FOEvar.foestate = FOE_WAIT_FOR_ACK;
      }
      else
      {
         FOE_abort (res);
      }
   }
   else
   {
      FOE_abort (res);
   }
}

/** FoE data ack handler. Will continue sending next frame until finished.
 * On error we will send FOE Abort.
 */
static void FOE_ack ()
{
   uint32_t res;

   /* Make sure we're able to take this. */
   if (FOEvar.foestate == FOE_WAIT_FOR_FINAL_ACK)
   {
      /* Move us back to ready state. */
      FOE_init ();
      return;
   }
   else if (FOEvar.foestate != FOE_WAIT_FOR_ACK)
   {
      FOE_abort (FOE_ERR_ILLEGAL);
      return;
   }
   res = FOE_send_data_packet ();
   if (res < ESC_FOE_DATA_SIZE)
   {
      FOEvar.foestate = FOE_WAIT_FOR_FINAL_ACK;
   }
   else if (res >= FOE_ERR_NOTDEFINED)
   {
      FOE_abort (FOE_ERR_PROGERROR);
   }
}
#endif

/** FoE write request handler. Starts with Initialize, Open and Ack that we can/will
 * receive data. On error we will send FOE Abort.
 *
 */
static void FOE_write ()
{
   esc_octet_t *mailbox;
   uint32_t password;
   uint32_t res;
   uint8_t data_len;

   if (FOEvar.foestate != FOE_READY)
   {
      FOE_abort (FOE_ERR_ILLEGAL);
      return;
   }

   FOE_init ();
   mailbox = &MBX[0];
   data_len = (uint8_t)(esc_mbx_length (mailbox) - ESC_FOEHSIZE);
   password = foe_value (mailbox);

   /* Get an address we can write the file to, if possible. */
   res = FOE_fopen (foe_data (mailbox), data_len, password, FOE_OP_WRQ);
   DPRINT("%s %sOK, file \"%s\"\n", __func__, (res == 0) ? "" : "N", foe_file_name);
   if (res == 0)
   {
      res = FOE_send_ack ();
      if (res)
      {
         FOE_abort (res);
      }
      else
      {
         FOEvar.foestate = FOE_WAIT_FOR_DATA;
      }
   }
   else
   {
      FOE_abort (res);
   }
}
/** FoE data request handler. Validates and reads data until we're finished. Every
 * read frame followed by an Ack frame. On error we will send FOE Abort.
 *
 */
static void FOE_data ()
{
   esc_octet_t *mailbox;
   uint32_t packet;
   uint32_t data_len, ncopied;
   uint32_t res;

   if(FOEvar.foestate != FOE_WAIT_FOR_DATA)
   {
      FOE_abort(FOE_ERR_ILLEGAL);
      return;
   }

   mailbox = &MBX[0];
   data_len = esc_mbx_length (mailbox) - ESC_FOEHSIZE;
   packet = foe_value (mailbox);

   if (packet != FOEvar.foepacket)
   {
      DPRINT("FOE_data packet error, packet: %"PRIu32", foeheader.packet: %"PRIu32"\n",
            packet,
            FOEvar.foepacket);
      FOE_abort (FOE_ERR_PACKETNO);
   }
   else if (data_len == 0)
   {
      DPRINT("FOE_data completed\n");
      FOE_fclose ();
      res = FOE_send_ack ();
      FOE_init ();
   }
   else if (FOEvar.fposition + data_len > FOEvar.fend)
   {
      DPRINT("FOE_data disk full\n");
      FOE_abort (FOE_ERR_DISKFULL);
   }
   else
   {
      ncopied = FOE_fwrite (foe_data (mailbox), data_len);
      if (!ncopied)
      {
         DPRINT("FOE_data no copied\n");
         FOE_abort (FOE_ERR_PROGERROR);
      }
      else if (data_len == ESC_FOE_DATA_SIZE)
      {
         DPRINT("FOE_data data_len == FOE_DATA_SIZE\n");
         if (ncopied != data_len)
         {
            DPRINT("FOE_data only %"PRIu32" of %"PRIu32" copied\n",ncopied, data_len);
            FOE_abort (FOE_ERR_PROGERROR);
         }
         res = FOE_send_ack ();
         if (res)
         {
            FOE_abort (res);
         }
      }
      else
      {
         if ((ncopied != data_len) || FOE_fclose ())
         {
            DPRINT("FOE_fclose failed to write extra buffer\n");
            FOE_abort (FOE_ERR_PROGERROR);
         }
         else
         {
            DPRINT("FOE_data completed\n");
            res = FOE_send_ack ();
            FOE_init ();
         }
      }
   }
}

#ifdef FOE_READ_SUPPORTED
/** FoE read request busy handler. Send an Ack of last frame again. On error
 * we will send FOE Abort.
 *
 */
static void FOE_busy ()
{
   /* Only valid if we're servicing a read request. */
   if (FOEvar.foestate != FOE_WAIT_FOR_ACK)
   {
      FOE_abort (FOE_ERR_ILLEGAL);
   }
   else
   {
      /* Send the last part again. */
      FOEvar.fposition = FOEvar.fprevposition;
      FOEvar.foepacket--;
      FOE_ack ();
   }
}
#endif

/** FoE error requesthandler. Send an FOE Abort.
 *
 */
static void FOE_error ()
{
   /* Master panic! abort the transfer. */
   FOE_abort (0);
}

/** Function copying the application configuration variable
 * to the FoE module local pointer variable.
 *
 * @param[in] cfg       = Pointer to by the Application static declared
 * configuration variable holding application specific details.
 */
void FOE_config (foe_cfg_t * cfg)
{
   foe_cfg = cfg;
}

/** Main FoE function checking the status on current mailbox buffers carrying
 * data, distributing the mailboxes to appropriate FOE functions depending
 * on requested opcode.
 * On Error an FoE Error or FoE Abort will be sent.
 */
void ESC_foeprocess (void)
{
   esc_octet_t *mailbox = &MBX[0];

   if (ESCvar.MBXrun == 0)
   {
      return;
   }
   if (!ESCvar.xoe && (MBXcontrol[0].state == MBXstate_inclaim))
   {
      if (esc_mbx_type (mailbox) == MBXFOE)
      {
         ESCvar.xoe = MBXFOE;
      }
   }
   if (ESCvar.xoe == MBXFOE)
   {
      /* Verify the size of the file data. */
      if (esc_mbx_length (mailbox) < ESC_FOEHSIZE)
      {
         FOE_abort (MBXERR_SIZETOOSHORT);
      }
      else if ((ESCvar.ALstatus != ESCboot) && (ESCvar.ALstatus != ESCpreop))
      {
         FOE_abort (FOE_ERR_NOTINBOOTSTRAP);
      }
      else
      {
         switch (foe_opcode (mailbox))
         {
            case FOE_OP_WRQ:
            {
               DPRINT("FOE_OP_WRQ\n");
               FOE_write ();
               break;
            }
            case FOE_OP_DATA:
            {
               DPRINT("FOE_OP_DATA\n");
               FOE_data ();
               break;
            }
#ifdef FOE_READ_SUPPORTED
            case FOE_OP_RRQ:
            {
               DPRINT("FOE_OP_RRQ\n");
               FOE_read ();
               break;
            }
            case FOE_OP_ACK:
            {
               DPRINT("FOE_OP_ACK\n");
               FOE_ack ();
               break;
            }

            case FOE_OP_BUSY:
            {
               DPRINT("FOE_OP_BUSY\n");
               FOE_busy ();
               break;
            }
#endif
            case FOE_OP_ERR:
            {
               DPRINT("FOE_OP_ERR\n");
               FOE_error ();
               break;
            }
            default:
            {
               DPRINT("FOE_ERR_NOTDEFINED\n");
               FOE_abort (FOE_ERR_NOTDEFINED);
               break;
            }
         }
      }
      MBXcontrol[0].state = MBXstate_idle;
      ESCvar.xoe = 0;
   }
}
