/*
 * Portable EtherCAT-octet access helpers.
 *
 * EtherCAT lengths are expressed in 8-bit octets.  On C28x a C byte is
 * 16 bits, so wire buffers must be accessed through these helpers rather
 * than through C structure overlays or sizeof-based wire lengths.
 */
#ifndef SOES_ESC_OCTET_H
#define SOES_ESC_OCTET_H

#include <stdint.h>

typedef uint8_t esc_octet_t;

/* EtherCAT mailbox header offsets, expressed in 8-bit octets. */
#define ESC_MBX_O_LENGTH       0U
#define ESC_MBX_O_ADDRESS      2U
#define ESC_MBX_O_CHANNEL_PRIO 4U
#define ESC_MBX_O_TYPE_COUNT   5U
#define ESC_MBX_WIRE_SIZE      6U

static inline uint16_t esc_octet_get(const esc_octet_t *buffer,
                                     uint32_t offset)
{
   return (uint16_t)buffer[offset] & 0x00ffU;
}

static inline void esc_octet_set(esc_octet_t *buffer,
                                 uint32_t offset,
                                 uint16_t value)
{
   buffer[offset] = (esc_octet_t)(value & 0x00ffU);
}

static inline uint16_t esc_get_le16(const esc_octet_t *buffer,
                                    uint32_t offset)
{
   return esc_octet_get(buffer, offset) |
          (uint16_t)(esc_octet_get(buffer, offset + 1U) << 8U);
}

static inline void esc_put_le16(esc_octet_t *buffer,
                                uint32_t offset,
                                uint16_t value)
{
   esc_octet_set(buffer, offset, value);
   esc_octet_set(buffer, offset + 1U, value >> 8U);
}

static inline uint32_t esc_get_le32(const esc_octet_t *buffer,
                                    uint32_t offset)
{
   return (uint32_t)esc_get_le16(buffer, offset) |
          ((uint32_t)esc_get_le16(buffer, offset + 2U) << 16U);
}

static inline void esc_put_le32(esc_octet_t *buffer,
                                uint32_t offset,
                                uint32_t value)
{
   esc_put_le16(buffer, offset, (uint16_t)value);
   esc_put_le16(buffer, offset + 2U, (uint16_t)(value >> 16U));
}

static inline void esc_octet_copy(esc_octet_t *destination,
                                  const esc_octet_t *source,
                                  uint32_t count)
{
   uint32_t i;
   for (i = 0U; i < count; i++)
   {
      esc_octet_set(destination, i, esc_octet_get(source, i));
   }
}

static inline void esc_octet_clear(esc_octet_t *destination, uint32_t count)
{
   uint32_t i;
   for (i = 0U; i < count; i++)
   {
      esc_octet_set(destination, i, 0U);
   }
}

static inline uint16_t esc_mbx_length(const esc_octet_t *mailbox)
{
   return esc_get_le16(mailbox, ESC_MBX_O_LENGTH);
}

static inline void esc_mbx_set_length(esc_octet_t *mailbox, uint16_t value)
{
   esc_put_le16(mailbox, ESC_MBX_O_LENGTH, value);
}

static inline uint16_t esc_mbx_address(const esc_octet_t *mailbox)
{
   return esc_get_le16(mailbox, ESC_MBX_O_ADDRESS);
}

static inline void esc_mbx_set_address(esc_octet_t *mailbox, uint16_t value)
{
   esc_put_le16(mailbox, ESC_MBX_O_ADDRESS, value);
}

static inline uint16_t esc_mbx_type(const esc_octet_t *mailbox)
{
   return esc_octet_get(mailbox, ESC_MBX_O_TYPE_COUNT) & 0x0fU;
}

static inline void esc_mbx_set_type(esc_octet_t *mailbox, uint16_t value)
{
   uint16_t type_count = esc_octet_get(mailbox, ESC_MBX_O_TYPE_COUNT);
   esc_octet_set(mailbox, ESC_MBX_O_TYPE_COUNT,
                 (type_count & 0xf0U) | (value & 0x0fU));
}

static inline uint16_t esc_mbx_count(const esc_octet_t *mailbox)
{
   return (esc_octet_get(mailbox, ESC_MBX_O_TYPE_COUNT) >> 4U) & 0x07U;
}

static inline void esc_mbx_set_count(esc_octet_t *mailbox, uint16_t value)
{
   uint16_t type_count = esc_octet_get(mailbox, ESC_MBX_O_TYPE_COUNT);
   esc_octet_set(mailbox, ESC_MBX_O_TYPE_COUNT,
                 (type_count & 0x0fU) | ((value & 0x07U) << 4U));
}

#endif /* SOES_ESC_OCTET_H */
