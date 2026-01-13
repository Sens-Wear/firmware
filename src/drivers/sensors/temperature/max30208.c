#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <assert.h>
#include <string.h>

#include "max30208.h"

LOG_MODULE_REGISTER(SENS_WEAR_TEMPERATURE_SENSOR_LOGGER);

static uint8_t *raw_data_buffer = NULL;
static size_t base_buffer_size = 0;
static int buffer_index = 0;

// Frequency settings (default values)
static uint16_t sampling_rate = 10;	   // Default: 10 Hz (100 ms interval)
static uint16_t transfer_interval = 1; // Default: 1 Hz (every second)

static struct k_timer sensor_timer;
static bool subscription_active = false;

#define MAX30208_NODE DT_NODELABEL(max30208)
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(MAX30208_NODE);

// Function to update the buffer
static void update_sensor_buffer()
{
	size_t new_base_buffer_size = (size_t)(sampling_rate * transfer_interval * 2); // 2 bytes for 16-bit temperature data

	uint8_t *new_raw_data_buffer = k_malloc(new_base_buffer_size);

	if (new_raw_data_buffer == NULL)
	{
		LOG_ERR("Memory allocation failed!\n");
		return;
	}

	// Copy existing data (if needed)
	if (raw_data_buffer != NULL)
	{
		size_t copy_size = (base_buffer_size < new_base_buffer_size) ? base_buffer_size : new_base_buffer_size * 2;
		memcpy(new_raw_data_buffer, raw_data_buffer, copy_size);
		k_free(raw_data_buffer); // Free the old buffer
	}

	raw_data_buffer = new_raw_data_buffer;
	base_buffer_size = new_base_buffer_size + 10;
	buffer_index = 0;
	LOG_INF("Buffer updated. New size (%d): %zu bytes\n", base_buffer_size, base_buffer_size);
}

__STATIC_INLINE void
max30208_i2c_write_registers(uint8_t startAddress,
							 uint8_t *value,
							 size_t count)
{
	// Allocate a buffer to hold the register address and data
	uint8_t buffer[count + 1];
	buffer[0] = (uint8_t)startAddress; // First byte is the register address

	// Copy the data to the buffer
	memcpy(&buffer[1], value, count);

	// Perform the I2C write operation
	int ret = i2c_write_dt(&dev_i2c, buffer, count + 1);
	if (ret != 0)
	{
		LOG_ERR("I2C bus %s: Error while writing registers", dev_i2c.bus->name);
		return;
	}
}

__STATIC_INLINE void
max30208_i2c_read_registers(uint8_t startAddress,
							uint8_t *value,
							size_t count)
{
	// Perform the I2C read operation
	uint8_t addressRegister = (uint8_t)startAddress;
	int ret = i2c_write_read_dt(&dev_i2c, &addressRegister, 1, value, count);
	if (ret != 0)
	{
		LOG_ERR("I2C bus %s: Error while reading registers with error code of %d", dev_i2c.bus->name, ret);
		return;
	}
}

void set_temperature_sensor_sampling_rate(uint16_t new_sampling_rate)
{
	if (new_sampling_rate > 0)
	{
		sampling_rate = new_sampling_rate;
	}
	update_sensor_buffer();
}

void set_temperature_sensor_transfer_interval(uint16_t new_transfer_interval)
{
	if (transfer_interval > 0)
	{
		transfer_interval = new_transfer_interval;
	}
	update_sensor_buffer();
}

// Sensor thread function
void sensor_data_work_handler(void)
{
	write_SetupRegister();
	int value = 0;
	union max30208_status_register_t *statusRegister = (void *)&value;
	/* Wait for reset to be cleared */
	do
	{
		k_msleep(100);
		read_StatusRegister((void *)&value);
	} while (statusRegister->bits.temp_ready != 0);
	uint16_t FiFo_data[1];
	read_FiFoDataRegister(FiFo_data);
	double temperature= 0.00;
	temperature = read_Temperature((double)FiFo_data[0]);
	LOG_INF("Temperature: %f", temperature);
}


bool temperature_init()
{
	if (!device_is_ready(dev_i2c.bus))
	{
		LOG_ERR("I2C bus %s is not ready!\n\r", dev_i2c.bus->name);
		return;
	}
	uint8_t ID_Reg[1];
	read_PartID(ID_Reg);
	if (ID_Reg[0] != PART_ID_VALUE)
	{
		LOG_ERR("MAX30208 not connected\n\r");
		return;
	}
	write_FiFoConfig1();
	sensor_data_work_handler();
}

/*
 * @brief: Read the Id_Part Register (Max30208)
 */
int read_PartID(uint8_t *data)
{
	uint8_t ID[1] = {0};
	max30208_i2c_read_registers(ID_ADDR, ID, 1);
	*data = ID[0];
	return ID[0];
}

/*
 * @brief: Read the OverFlow Register (Max30208)
 */
void read_OverFlow(uint8_t *data)
{
	uint8_t value[1];
	max30208_i2c_read_registers(FiFo_OverFlow, value, 1);
	*data = value[0];
}

/*
 * @brief: Read the Status Register (Max30208)
 */
void read_StatusRegister(uint8_t *data)
{
	uint8_t status[1];
	max30208_i2c_read_registers(Status_Register, status, 1);
	*data = status[0];
}

/*
 * @brief: WRITE Setup Register Max30208-->Default value = 0xC0, then  0xC0---> 0XC1
 */
void write_SetupRegister(void)
{
	max30208_i2c_write_registers(SETUP_ADDR, &SETUP_VALUE, 1);
}

/*
 * @brief: WRITE Into FiFo Configuration_1 Register (Address:0x09; Value: 0x0F) ;
 */
int write_FiFoConfig1(void)
{
	max30208_i2c_write_registers(FIFoConfig1, &FIFoConfig1_VALUE, 1);
}

/*
 * @brief:	Reading 16-bits temperature Sample from FiFoData
 */
int read_FiFoDataRegister(uint16_t *data)
{
	int numberOfByte = 0;
	int error = 0;
	uint8_t temp[2];
	max30208_i2c_read_registers(TEMP_ADDR, temp, 2);
	uint16_t combine = ((uint16_t)temp[0] << 8) | temp[1]; // COMBINE TO 16 BIT
	*data = combine;
	return data;
}

/*
 * @brief:	Convert unsigned 16-bits from Data FiFo register to Celsius unit
 */
double read_Temperature(double data_fifo)
{
	double final = 0.000;
	final = data_fifo * 0.005;
	return final;
}