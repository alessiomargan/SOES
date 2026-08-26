#include "ethercat_subdevice_cpu1_hal.h"
#include <soes/esc.h>

static inline void ESC_Read_ALEVENT(void) {

	uint16_t alevent;
	alevent = ESC_readWordISR(ESCREG_ALEVENT);
	CC_ATOMIC_SET(ESCvar.ALevent, (etohs(alevent)));
	return;
}

/**
 * 
 * @author amargan 
 * 
 * @param addr 
 * @param data 
 * @param len 
 * 
 */
void ESC_read(uint16_t addr, void * data, uint16_t len) {

	switch(len) {
		case 1:
			{
				uint8_t *p = data;
				*p = ESC_readByte(addr);
				break;
			}
			case 2:
			{
				uint16_t *p = data;
				*p = ESC_readWord(addr);
				break;
			}
			case 4:
			{
				uint32_t *p = data;
				*p = ESC_readDWord(addr);
				break;
			}
			default:
			{
				ESC_readBlock((ESCMEM_ADDR*)data, addr, len);
				break;
			}
	}

	ESC_Read_ALEVENT();
	return;
}

/**
 * 
 * @author amargan
 * 
 * @param addr 
 * @param data 
 * @param len 
 * 
 */
void ESC_write(uint16_t addr, void * data, uint16_t len) {
	
	switch(len) {
		case 1:
			{
				uint8_t *p = data;
				ESC_writeByte(*p, addr);
				break;
			}
			case 2:
			{
				uint16_t *p = data;
				ESC_writeWord(*p, addr);
				break;
			}
			case 4:
			{
				uint32_t *p = data;
				ESC_writeDWord(*p, addr);
				break;
			}
			default:
			{
				ESC_writeBlock((ESCMEM_ADDR*)data, addr, len);
				break;
			}
	}

	ESC_Read_ALEVENT();
	return;
}

void ESC_reset (void)
{
    
}

void ESC_init (const esc_cfg_t * config)
{
	uint32_t value;
	uint8_t type;
	uint8_t PDI_ctrl;

	// busy wait for EEPROM_LOAD pin if present
	//assert_EE_LOAD();

	ESC_read (0x0000, &type, sizeof(type));
	ESC_read (0x0140, &PDI_ctrl, sizeof(PDI_ctrl));
	DPRINT ("0x%02X 0x%02X\n",type, PDI_ctrl);

	do {
	   value = 0x4;
	   ESC_write (ESCREG_ALEVENTMASK, &value, sizeof(value));
	   value = 0;
	   ESC_read (ESCREG_ALEVENTMASK, &value, sizeof(value));
	} while ( value != 0x4);

}

