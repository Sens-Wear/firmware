#ifndef MAX30208_H_
#define MAX30208_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <stdbool.h>
#include <stdint.h>

static uint8_t ID_ADDR = 0xFF; // IDpart register Address = 0xff

static uint8_t Status_Register = 0x00; // Status register Address = 0x00
static uint8_t FiFo_OverFlow = 0x06;   // FiFo OverFlow register Address = 0x06
static uint8_t FIFoConfig1 = 0x09; // FiFo_Config1 Register Address: 0x09 Value 0x0F

static uint8_t SETUP_ADDR = 0x14;		// setup_reg_add = 0x14 --> value = 0xC1
static uint8_t TEMP_ADDR = 0x08;

static uint8_t SETUP_VALUE = 0xC1;
static uint8_t FIFoConfig1_VALUE = 0x0F;
static uint8_t PART_ID_VALUE = 0x30; // IDpart register Address = 0xff

/**
 * \brief Status Register
 */
union max30208_status_register_t {
	int value;
	struct max30208_status_register_bits {
		unsigned int a_full : 1;
		unsigned int reserved : 4;
		unsigned int temp_lo : 1;
		unsigned int temp_hi : 1;
		unsigned int temp_ready : 1;
	} bits;
};

 /*****************************************************************************************************
 * read_PartID
 * @brief: READ Part_ID Register from Max30208 
 * @param[out] data - PartID data from FIFO data register on successful read
 * @return 0 on success, non-zero on failure
 * Expecting print out Result: 0x30
******************************************************************************************************/
int read_PartID (uint8_t *data);

 /****************************************************************************************************************
 * read_OverFlow
 * @brief: READ FiFo_OverFlow Register from Max30208
 * @return 0 on success, non-zero on failure
****************************************************************************************************************/
void read_OverFlow (uint8_t *data);

 /***************************************************************************************************
 * read_StatusRegister
 * @brief: Read the Status Register (Max30208)
 * @param[out] data - data from status Register on successful read
 ***************************************************************************************************/
void read_StatusRegister (uint8_t *data);

 /************************************************************************************************************
 * write_SetupRegister
 * @brief: WRITE Setup Register (Address:0x14; Value: C1)  ; Default value = 0xC0, then  0xC0---> 0XC1
 * @Note:  (the last bit use for convert temp and we need to set it to one every time we want to read).
*************************************************************************************************************/
void write_SetupRegister (void);

/************************************************************************************************************
 * write_FiFoConfig1
 * @brief: WRITE Into FiFo Configuration_1 Register (Address:0x09; Value: 0x0F) ;
 * @return 0 on success, non-zero on failure
 * @Note: configure the Push and Pop on FiFo Register of Max30208
************************************************************************************************************/
int write_FiFoConfig1 (void);

/**************************************************************************************************************
 * read_FiFoDataRegister
 * @brief:	Reading 16-bits temperature Sample from FiFoData
 * @param[out] data -  2-data from FIFO Register on successful read
 * @return 0 on success, non-zero on failure
 * @Note:  READ FIFO DATA REG (2 BYTE)-->The Final result is 16-Bits data
 * -------->unint8_t FiFo_data[n] + unint8_t FiFo_data[n+1] = uint16_t result 
**************************************************************************************************************/
int read_FiFoDataRegister (uint16_t *data);

/***************************************************************************************************************
 * read_Temperature
 * @brief:	Convert unsigned 16-bits from Data FiFo register to Celsius unit
 * @param[in] data_fifo.
 * @return temperature in double type on successful call
***************************************************************************************************************/
double read_Temperature (double data_fifo);

bool temperature_init();


#endif /* MAX30208_H_ */
