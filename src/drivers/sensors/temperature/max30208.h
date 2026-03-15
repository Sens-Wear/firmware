#ifndef MAX30208_H_
#define MAX30208_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <stdbool.h>
#include <stdint.h>

#define MAX30208_ID_ADDR 0xFF
#define MAX30208_STATUS_REGISTER_ADDR 0x00
#define MAX30208_FIFO_OVERFLOW_ADDR 0x06
#define MAX30208_FIFO_CONFIG1_ADDR 0x09
#define MAX30208_FIFO_CONFIG2_ADDR 0x0A
#define MAX30208_TEMP_ADDR 0x08
#define MAX30208_SETUP_ADDR 0x14

#define MAX30208_SETUP_VALUE 0xC1
#define MAX30208_FIFO_CONFIG1_VALUE 0x0F
#define MAX30208_FIFO_CONFIG2_FLUSH_FIFO BIT(4)
#define MAX30208_PART_ID_VALUE 0x30

struct temperature_sample {
	int32_t temperature_mdeg_c;
};

typedef void (*temperature_sample_cb_t)(const struct temperature_sample *sample,
					 void *user_data);

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
int read_PartID(uint8_t *data);

 /****************************************************************************************************************
 * read_OverFlow
 * @brief: READ FiFo_OverFlow Register from Max30208
 * @return 0 on success, non-zero on failure
****************************************************************************************************************/
void read_OverFlow(uint8_t *data);

 /***************************************************************************************************
 * read_StatusRegister
 * @brief: Read the Status Register (Max30208)
 * @param[out] data - data from status Register on successful read
 ***************************************************************************************************/
void read_StatusRegister(uint8_t *data);

 /************************************************************************************************************
 * write_SetupRegister
 * @brief: WRITE Setup Register (Address:0x14; Value: C1)  ; Default value = 0xC0, then  0xC0---> 0XC1
 * @Note:  (the last bit use for convert temp and we need to set it to one every time we want to read).
*************************************************************************************************************/
void write_SetupRegister(void);

/************************************************************************************************************
 * write_FiFoConfig1
 * @brief: WRITE Into FiFo Configuration_1 Register (Address:0x09; Value: 0x0F) ;
 * @return 0 on success, non-zero on failure
 * @Note: configure the Push and Pop on FiFo Register of Max30208
************************************************************************************************************/
int write_FiFoConfig1(void);
int flush_FiFo(void);

/**************************************************************************************************************
 * read_FiFoDataRegister
 * @brief:	Reading 16-bits temperature Sample from FiFoData
 * @param[out] data -  2-data from FIFO Register on successful read
 * @return 0 on success, non-zero on failure
 * @Note:  READ FIFO DATA REG (2 BYTE)-->The Final result is 16-Bits data
 * -------->unint8_t FiFo_data[n] + unint8_t FiFo_data[n+1] = uint16_t result 
**************************************************************************************************************/
int read_FiFoDataRegister(uint16_t *data);

/***************************************************************************************************************
 * read_Temperature
 * @brief:	Convert unsigned 16-bits from Data FiFo register to Celsius unit
 * @param[in] data_fifo.
 * @return temperature in double type on successful call
***************************************************************************************************************/
double read_Temperature(double data_fifo);

int temperature_sensor_init(void);
int temperature_sensor_start(void);
void temperature_sensor_stop(void);
void temperature_sensor_set_streaming_enabled(bool enabled);
void temperature_sensor_set_sampling_rate(uint16_t new_sampling_rate);
void temperature_sensor_set_transfer_interval(uint16_t new_transfer_interval);
int temperature_sensor_register_callback(temperature_sample_cb_t cb, void *user_data);


#endif /* MAX30208_H_ */
