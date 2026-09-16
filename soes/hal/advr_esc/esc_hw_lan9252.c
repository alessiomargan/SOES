/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

 /** \file
 * \brief
 * ESC hardware layer functions for LAN9252.
 *
 * Function to read and write commands to the ESC. Used to read/write ESC
 * registers and memory.
 */

#include <string.h>
#include <soes/esc.h>

#include "hw_lan9252.h"


static inline uint16_t check_addr_size(uint16_t address, uint16_t len) {

	/* We write maximum 4 bytes at the time */
	uint16_t size = (len > 4) ? 4 : len;
	/* Make size aligned to address according to LAN9252 datasheet
	* Table 12-14 EtherCAT CSR Address VS size and MicroChip SDK code */
	/* If we got an odd address size is 1 , 01b 11b is captured */
	if(address & BIT(0)) {
		size = 1;
	} /* If address 1xb and size != 1 and 3 , allow size 2 else size 1 */
	else if (address & BIT(1)) {
		size = (size & BIT(0)) ? 1 : 2;
	} /* size 3 not valid */
	else if (size == 3) {
		size = 1;
	}
	return size;
}

/** ESC read function used by the Slave stack.
 *
 * @param[in]   address     = address of ESC register to read
 * @param[out]  buf         = pointer to buffer to read in
 * @param[in]   len         = number of bytes to read
 */
CC_RAMFUNC
void ESC_read (uint16_t address, void *buf, uint16_t len)
{
    uint8_t *temp_buf = (uint8_t *)buf;
    /* Select Read function depending on address, process data ram or not */
    if (address >= 0x1000) {
        ESC_read_pram(address, buf, len);        
    } else {
        while(len > 0) {
        	uint16_t size = check_addr_size(address, len);
            ESC_read_csr(address, temp_buf, size);
            /* next address */
            len -= size;
            temp_buf += size;
            address += size;
        }
    }
    /* To mimic the ET1100 always providing AlEvent on every read or write */
    ESC_read_csr(ESCREG_ALEVENT,(void *)&ESCvar.ALevent,sizeof(ESCvar.ALevent));
    ESCvar.ALevent = etohs (ESCvar.ALevent);

}

/** ESC write function used by the Slave stack.
 *
 * @param[in]   address     = address of ESC register to write
 * @param[out]  buf         = pointer to buffer to write from
 * @param[in]   len         = number of bytes to write
 */
CC_RAMFUNC
void ESC_write (uint16_t address, void *buf, uint16_t len)
{
	uint8_t *temp_buf = (uint8_t *)buf;
	/* Select Write function depending on address, process data ram or not */
	if (address >= 0x1000) {
		ESC_write_pram(address, buf, len);
	} else {
		while(len > 0)
		{
			uint16_t size = check_addr_size(address, len);
			ESC_write_csr(address, temp_buf, size);
			/* next address */
			len -= size;
			temp_buf += size;
			address += size;
		}
	}
	/* To mimic the ET1x00 always providing AlEvent on every read or write */
	ESC_read_csr(ESCREG_ALEVENT,(void *)&ESCvar.ALevent,sizeof(ESCvar.ALevent));
	ESCvar.ALevent = etohs (ESCvar.ALevent);
}

/* Un-used due to evb-lan9252-digio not havning any possability to
 * reset except over SPI.
 */
void ESC_reset (void)
{
	uint32_t values[] = {0x52, 0x45, 0x53, 0x0}, *pval, ret;

	pval = values;
	do {
		lan9252_write_32 (ESC_RESET_PDI_REG, *pval);
		ret = lan9252_read_32 (ESC_RESET_PDI_REG);
		DPRINT("%s reset_ecat_reg 0x%08"PRIX32" --> 0x%08"PRIX32"\n",
			__FUNCTION__, *pval, ret);
	} while ( *(++pval) );

}

void ESC_init (const esc_cfg_t * config)
{
	uint32_t value;

	value = 0;
	do {
		value = lan9252_read_32(ESC_BYTE_TEST_REG);
	} while(value != 0x87654321);
	DPRINT("%s test_byte_reg 0x%08"PRIX32" ok\n", __FUNCTION__, value);

	// select which events will be sent to PDI irq
	// set bit 2 : state of DC sync0
	do {
		value = 0x4;
		ESC_write (ESCREG_ALEVENTMASK, (void*)&value, sizeof(value));
		value = 0;
		ESC_read (ESCREG_ALEVENTMASK, (void*)&value, sizeof(value));
	} while ( value != 0x4);

	//IRQ enable,IRQ polarity, IRQ buffer type in Interrupt Configuration register.
	//Wrte 0x54 - 0x00000101
	value = 0x00000101;
	lan9252_write_32(ESC_IRQ_CFG_REG, value);
	//Write in Interrupt Enable register -->
	//Write 0x5c - 0x00000001
	value = 0x00000001;
	lan9252_write_32(ESC_INT_EN_REG, value);

	lan9252_read_32(ESC_INT_STS_REG);

}
