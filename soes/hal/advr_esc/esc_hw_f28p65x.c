/*
 * Licensed under the GNU General Public License version 2 with exceptions.
 * See LICENSE in the repository root for full license information.
 */

/**
 * \file esc_hwf28p65x.c
 * \brief SOES hardware adapter for the F28P65x integrated EtherCAT ESC.
 *
 * SOES addresses and lengths are expressed in EtherCAT octets. The F28P65x
 * HAL performs the required packing for block transfers on the C28x core.
 */

#include <soes/esc.h>
#include "ethercat_subdevice_cpu1_hal.h"

static inline void ESC_refreshALEvent(void) {

	uint16_t alevent;
	alevent = ESC_readWordISR(ESCREG_ALEVENT);
	CC_ATOMIC_SET(ESCvar.ALevent, (etohs(alevent)));
	return;
}

static uint16_t ESC_readByte(uint16_t address)
{
    uint16_t word = ESC_readWord(address & 0xFFFEU);

    return ((address & 1U) != 0U)
           ? ((word >> 8U) & 0x00FFU)
           : (word & 0x00FFU);
}

static void ESC_writeByte(uint16_t value, uint16_t address)
{
    uint16_t aligned = address & 0xFFFEU;
    uint16_t word = ESC_readWord(aligned);

    if((address & 1U) != 0U)
    {
        word = (word & 0x00FFU) | ((value & 0x00FFU) << 8U);
    }
    else
    {
        word = (word & 0xFF00U) | (value & 0x00FFU);
    }

    ESC_writeWord(word, aligned);
}

void ESC_read_octets(uint16_t address, esc_octet_t *buf, uint16_t len)
{
    if(((address & 1U) != 0U) && (len != 0U))
    {
        *buf++ = (esc_octet_t)ESC_readByte(address++);
        len--;
    }

    while(len >= 2U)
    {
        uint16_t word = ESC_readWord(address);
        *buf++ = (esc_octet_t)(word & 0x00ffU);
        *buf++ = (esc_octet_t)(word >> 8U);
        address += 2U;
        len -= 2U;
    }

    if(len != 0U)
    {
        *buf = (esc_octet_t)ESC_readByte(address);
    }

    ESC_refreshALEvent();
}

void ESC_write_octets(uint16_t address, const esc_octet_t *buf, uint16_t len)
{
    if(((address & 1U) != 0U) && (len != 0U))
    {
        ESC_writeByte((uint16_t)*buf++, address++);
        len--;
    }

    while(len >= 2U)
    {
        uint16_t word = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8U);
        ESC_writeWord(word, address);
        buf += 2;
        address += 2U;
        len -= 2U;
    }

    if(len != 0U)
    {
        ESC_writeByte((uint16_t)*buf, address);
    }

    ESC_refreshALEvent();
}

void ESC_read(uint16_t address, void *buf, uint16_t len)
{
    if(len == 1U)
    {
        *((uint8_t *)buf) = (uint8_t)ESC_readByte(address);
    }
    else if(len == 2U)
    {
        *((uint16_t *)buf) = ESC_readWord(address);
    }
    else if(len == 4U)
    {
        *((uint32_t *)buf) = ESC_readDWord(address);
    }
    else
    {
        ESC_readBlock((ESCMEM_ADDR *)buf, address, len);
    }

    ESC_refreshALEvent();
}

void ESC_write(uint16_t address, void *buf, uint16_t len)
{
    if(len == 1U)
    {
        ESC_writeByte(*((uint8_t *)buf), address);
    }
    else if(len == 2U)
    {
        ESC_writeWord(*((uint16_t *)buf), address);
    }
    else if(len == 4U)
    {
        ESC_writeDWord(*((uint32_t *)buf), address);
    }
    else
    {
        ESC_writeBlock((ESCMEM_ADDR *)buf, address, len);
    }

    ESC_refreshALEvent();
}

void ESC_reset(void)
{
    /* ESC_initHW() owns the F28P65x ESC reset and startup sequence. */
}

void ESC_init(const esc_cfg_t *config)
{
#if 0
    uint32_t eventMask;
    uint16_t pdiControl;

    (void)config;

    pdiControl = ESC_readWord(ESC_O_PDI_CONTROL);
    if((pdiControl & ESC_PDI_CONTROL_ASYNC16) != ESC_PDI_CONTROL_ASYNC16)
    {
        DPRINT("F28P65x ESC: ASYNC16 PDI is not active (0x%04X)\n",
               pdiControl);
    }

    /* Enable AL-control events; SOES adds further event sources as needed. */
    do
    {
        eventMask = ESCREG_ALEVENT_CONTROL;
        ESC_writeDWord(eventMask, ESCREG_ALEVENTMASK);
        eventMask = ESC_readDWord(ESCREG_ALEVENTMASK);
    }
    while(eventMask != ESCREG_ALEVENT_CONTROL);

    ESC_refreshALEvent();
#endif

    uint32_t value;
	uint8_t type;
	uint8_t PDI_ctrl;

	// busy wait for EEPROM_LOAD pin if present
	//assert_EE_LOAD();

	ESC_read (0x0000, &type, 1U);
	ESC_read (0x0140, &PDI_ctrl, 1U);
	DPRINT ("0x%02X 0x%02X\n",type, PDI_ctrl);

	do {
	   value = 0x4;
	   ESC_write (ESCREG_ALEVENTMASK, &value, 4U);
	   value = 0;
	   ESC_read (ESCREG_ALEVENTMASK, &value, 4U);
	} while ( value != 0x4);

}
