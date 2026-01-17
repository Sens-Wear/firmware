#include "bq27427.h"
#include <zephyr/logging/log.h>
#include <assert.h>
#include <stdbool.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>


// --------------------------------------------------------------------------
 // #define BQ27427_I2C_DRIVER (hi2c1)
 // #define BQ27427_I2C_PORT (I2C1)
 // #define BQ27427_I2C_TIMING (0x00300F38)
 //
 // #define BQ27427_I2C_LOCK(...) true
 // #define BQ27427_I2C_UNLOCK(...) (void)0
 // #include <stdio.h>
 // #define sys_debug_print printf
 // #pragma pack(1)
 // union uint16_array_t {
 // 	uint8_t array[sizeof(uint16_t)];
 // 	uint16_t value;
 // };
 // #pragma pack()
// --------------------------------------------------------------------------
LOG_MODULE_REGISTER(bq27427, CONFIG_LOG_DEFAULT_LEVEL);

#define BQ27427_NODE DT_NODELABEL(bq27427)
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(BQ27427_NODE);

static struct bq27427_t bq27427 = {0};

static inline uint16_t swap_bytes16(uint16_t val) {
	uint8_t* pVal = (uint8_t*) &val;
	register const uint8_t temp = (uint8_t) val;
	pVal[0] = pVal[1];
	pVal[1] = temp;
	return val;
}

static inline
void bq27427_i2c_write_2byte_increment(uint8_t address, uint16_t value) {
	/* Most TI gauges use little-endian for 16-bit registers; confirm in datasheet. */
    uint8_t tx[3];
    tx[0] = address;
    sys_put_le16(value, &tx[1]);

    int err = i2c_write_dt(&bq27427.i2c, tx, sizeof(tx));
    if (err) {
        return err;
    }

    /* k_msleep(1) -> Zephyr sleep (don’t busy-wait unless required). */
    k_msleep(1);
    return 0;
}

void bq27427_i2c_write_control(enum bq27427_control_subcommand_type subcommand) {
	uint8_t payload[2];
    sys_put_le16((uint16_t)subcommand, payload);  /* use sys_put_be16 if required */

    int err = i2c_burst_write_dt(&bq27427.i2c,
                                 (uint8_t)bq27427_command_Control,
                                 payload,
                                 sizeof(payload));
    if (err) {
        return err;
    }

    k_msleep(1);
    return 0;
}

static inline
uint16_t bq27427_i2c_read_w_command(enum bq27427_command_type command) {
	uint8_t buf[2];
    uint16_t value = 0;
	int err = i2c_burst_read_dt(&bq27427.i2c,
                                (uint8_t)command,
                                buf,
                                sizeof(buf));
    if (err) {
        return 0;
    }

    value = sys_get_le16(buf);  /* or sys_get_be16 */

    return value;
}

static inline
uint16_t bq27427_i2c_read_control(const enum bq27427_control_subcommand_type subcommand) {
	uint8_t wr[2];
    sys_put_le16((uint16_t)subcommand, wr);

    if (i2c_burst_write_dt(&bq27427.i2c, (uint8_t)bq27427_command_Control, wr, sizeof(wr)) != 0) {
        return 0;
    }

    k_msleep(1);

    uint8_t rd[2];
    if (i2c_burst_read_dt(&bq27427.i2c, (uint8_t)bq27427_command_Control, rd, sizeof(rd)) != 0) {
        return 0;
    }

    return sys_get_le16(rd);
}

static inline void bq27427_i2c_write_byte(const uint8_t address, uint16_t value) {
	uint8_t v8 = (uint8_t)value;

    int err = i2c_burst_write_dt(&bq27427.i2c, address, &v8, 1);
    if (err) {
        return err;
    }

    k_msleep(1);
    return 0;
}

static inline uint16_t bq27427_i2c_read_memory(const uint8_t address, const size_t size) {
	uint16_t value;
	assert((address >= bq27427_extended_command_BlockDataStart && address <= bq27427_extended_command_BlockDataEnd) ||
		   (address == bq27427_extended_command_BlockDataChecksum));
	assert(size > 0 && size < 3);
	uint8_t buf[2] = { 0 };

    int err = i2c_burst_read_dt(&bq27427.i2c, address, buf, size);
    if (err) {
        return 0;
    }

    k_msleep(1);
	 if (size == 1) {
        return buf[0];
    } 
	uint16_t v = ((uint16_t)buf[0]) | ((uint16_t)buf[1] << 8);
	return swap_bytes16(v);
}

static inline union bq27427_control_status_register_t bq27427_i2c_read_control_status(void) {
	union bq27427_control_status_register_t controlStatus = {.value = 0};
	controlStatus.value = bq27427_i2c_read_control(bq27427_control_subcommand_ControlStatus);
	return controlStatus;
}

static inline void clear_port_or_reset(void) {
	union bq27427_flags_register_t flags;
	flags.value = bq27427_i2c_read_w_command(bq27427_command_Flags);
	while(flags.bits.bPOROrReset != 0) {
		// exit from configuration mode
		bq27427_i2c_write_control(bq27427_control_subcommand_SoftReset);
		k_msleep(2);
		flags.value = bq27427_i2c_read_w_command(bq27427_command_Flags);
	}
}

static inline void unseal_gauge(void) {
	union bq27427_control_status_register_t controlStatus = {.value = 0};
	union bq27427_flags_register_t flags = {.value = 0};
	assert(bq27427.state.bits.bInitialized != 0);
	flags.value = bq27427_i2c_read_w_command(bq27427_command_Flags);
	if(flags.bits.bPOROrReset != 0) {
		clear_port_or_reset();
		k_msleep(1200);
	}
	flags.value = bq27427_i2c_read_w_command(bq27427_command_Flags);
	controlStatus.value = bq27427_i2c_read_control(bq27427_control_subcommand_ControlStatus);
	while(controlStatus.bits.bSealedMode != 0) {
		k_msleep(10);
		bq27427_i2c_write_2byte_increment(bq27427_command_Control, BQ27427_UNSEAL_KEY);
		k_msleep(10);
		bq27427_i2c_write_2byte_increment(bq27427_command_Control, BQ27427_UNSEAL_KEY);
		k_msleep(100);
		controlStatus.value = bq27427_i2c_read_control(bq27427_control_subcommand_ControlStatus);
	}
}

static inline void seal_gauge(void) {
	union bq27427_control_status_register_t controlStatus = {.value = 0};
	assert(bq27427.state.bits.bInitialized != 0);

	controlStatus.value = bq27427_i2c_read_control(bq27427_control_subcommand_ControlStatus);
	if(controlStatus.bits.bSealedMode == 0) {
		bq27427_i2c_write_control(bq27427_control_subcommand_Sealed);
	}
}

static inline bool is_gauge_sealed(void) {
	union bq27427_control_status_register_t controlStatus = {.value = 0};
	assert(bq27427.state.bits.bInitialized != 0);

	controlStatus.value = bq27427_i2c_read_control(bq27427_control_subcommand_ControlStatus);
	return controlStatus.bits.bSealedMode != 0;
}
static inline bool is_config_update(void){
	union bq27427_flags_register_t flags;
	assert(bq27427.state.bits.bInitialized != 0);
	flags.value = bq27427_i2c_read_w_command(bq27427_command_Flags);
	return flags.bits.bConfigUpdateMode != 0;
}

static inline void enter_configuration_update_mode(void) {
	bool sealed = is_gauge_sealed();
	bool configUpdateMode = is_config_update();
	assert(sealed == false);
	if(configUpdateMode != false){
		return;
	}
	// send cfg update subcommand
	bq27427_i2c_write_control(bq27427_control_subcommand_SetConfigurationUpdate);
	int cntr = 0;
	union bq27427_flags_register_t flags = {.value = 0};
	do {
		k_msleep(10);
		flags.value = bq27427_i2c_read_w_command(bq27427_command_Flags);
		cntr++;
	} while(flags.bits.bConfigUpdateMode == 0 && cntr < 120);

	assert(flags.bits.bConfigUpdateMode != 0);
}

static inline void exit_configuration_update_mode(void) {
	union bq27427_flags_register_t flags;
	// exit from configuration mode
	bq27427_i2c_write_control(bq27427_control_subcommand_SoftReset);
	do {
		k_msleep(10);
		flags.value = bq27427_i2c_read_w_command(bq27427_command_Flags);
	} while(flags.bits.bConfigUpdateMode != 0);
}

static inline void update_chemistry_id(const enum bq27427_control_subcommand_type chemistryType, const enum bq27427_chemistry_type chemistryId) {
	// wait for chemistry changed bit in control status register to be set
	union bq27427_control_status_register_t controlStatus = {.value = 0};
	bq27427_i2c_write_control(chemistryType);
	int cntr = 0;
	do {
		// send the control command
		controlStatus = bq27427_i2c_read_control_status();
		cntr ++;
		uint16_t chemId = bq27427_i2c_read_control(bq27427_control_subcommand_ChemistryIdentifier);
		LOG_INF("BQ27427: Chemistry ID: %04x Required: %04x\r\n", chemId, chemistryId);
		if(chemId == chemistryId) {
			return;
		}
	} while(controlStatus.bits.bChemistryChanged == 0 && cntr < 120);
	assert(controlStatus.bits.bChemistryChanged != 0);
}

static inline void battery_insert(void) {
	bq27427_i2c_write_control(bq27427_control_subcommand_BatteryInsert);
}
static inline void battery_remove(void) {
	bq27427_i2c_write_control(bq27427_control_subcommand_BatteryRemove);
}

static inline enum bq27427_chemistry_type read_chemistry_id(void) {
	return (enum bq27427_chemistry_type) bq27427_i2c_read_control(bq27427_control_subcommand_ChemistryIdentifier);
}

/*
static inline uint16_t read_configuration_code(void) {
	return bq27427_i2c_read_control(bq27427_control_subcommand_DataMemoryCode);
}
*/
static inline void enable_data_memory_access(enum bq27427_flash_class_type flashClass) {
	// first enable Enable Block Data Memory Control
	bq27427_i2c_write_byte((uint8_t) bq27427_extended_command_BlockDataControl, 0x00);
	// send Data Block Class command and State subclass
	bq27427_i2c_write_byte((uint8_t) bq27427_extended_command_DataClass, (uint8_t) flashClass);
	// send the Data Block offset
	bq27427_i2c_write_byte((uint8_t) bq27427_extended_command_DataBlock, 0x00);
}

static inline void read_data_memory(uint8_t offset, uint16_t* pBuffer, size_t size) {
	while(size > 0) {
		size_t rdSize = (size < 2) ? 1 : 2;
		if(rdSize == 2) {
			uint16_t value =
				bq27427_i2c_read_memory((uint8_t) bq27427_extended_command_BlockDataStart + offset, sizeof(uint16_t));

			*pBuffer = swap_bytes16(value);
			size -= sizeof(uint16_t);
			offset += sizeof(uint16_t);
			pBuffer += 1;
		} else {
			uint16_t value =
				bq27427_i2c_read_memory((uint8_t) bq27427_extended_command_BlockDataStart + offset, sizeof(uint8_t));

			*((uint8_t*) pBuffer) = (uint8_t) value;
			size -= sizeof(uint8_t);
			offset += sizeof(uint8_t);
		}
	}
}

static inline void write_data_memory(uint8_t offset, const uint16_t* pBuffer, size_t size) {

	uint8_t checksum =
		(uint8_t) bq27427_i2c_read_memory((uint8_t) bq27427_extended_command_BlockDataChecksum, sizeof(uint8_t));
	uint8_t origChecksum = checksum;
	checksum = (255 - checksum);
	while(size > 0) {
		size_t wrSize = (size < 2) ? 1 : 2;
		if(wrSize == 2) {

			union uint16_array_t uint16Array;
			uint16_t prevValue =
				bq27427_i2c_read_memory((uint8_t) bq27427_extended_command_BlockDataStart + offset, wrSize);
			// prev value bytes are already swept
			uint16_t newValue = *pBuffer;
			if(newValue != prevValue) {
				// if they are the same we don't need to write anything
				newValue = swap_bytes16(*pBuffer);
				bq27427_i2c_write_2byte_increment((uint8_t) bq27427_extended_command_BlockDataStart + offset, newValue);
				// update the checksum
				uint16Array.value = prevValue;
				checksum -= uint16Array.array[0];
				checksum -= uint16Array.array[1];
				uint16Array.value = newValue;
				checksum += uint16Array.array[0];
				checksum += uint16Array.array[1];
				k_msleep(2);
			}
			// update the loop parameters
			pBuffer += 1;
		} else {
			uint16_t prevValue =
				bq27427_i2c_read_memory((uint8_t) bq27427_extended_command_BlockDataStart + offset, sizeof(uint8_t));
			// prev value bytes are already swept
			uint8_t newValue = *((uint8_t*) pBuffer);
			if(newValue != prevValue) {
				// if they are the same we don't need to write anything
				bq27427_i2c_write_byte((uint8_t) bq27427_extended_command_BlockDataStart + offset, newValue);
				// update the checksum
				checksum -= prevValue;
				checksum += newValue;
				k_msleep(2);
			}
		}
		size -= wrSize;
		offset += wrSize;
	}
	// finalize the checksum and write it to the memory
	checksum = 255 - checksum;
	if(checksum != origChecksum) {
		bq27427_i2c_write_byte((uint8_t) bq27427_extended_command_BlockDataChecksum, checksum);
	}
}
/*
 * \brief Initializes BQ27427 driver
 *
 * \param i2c I2C device to be used by the driver
 */
bool bq27427_init(void) {
	bq27427.i2c = dev_i2c;
	if (!device_is_ready(bq27427.i2c.bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}
	bq27427_get_default_config(&(bq27427.config));
	// since we can access the device, we can assume that battery presents
	battery_insert();
	// reset the device
	bq27427_i2c_write_control(bq27427_control_subcommand_SoftReset);
	bq27427.state.bits.bInitialized = 1;

	return true;
}

static inline void bq27427_program_chemistry_id(const enum bq27427_chemistry_type chemistryType) {
	enum bq27427_chemistry_type prevChemistry = read_chemistry_id();
	if(prevChemistry == chemistryType) {
		return;
	}
	enum bq27427_control_subcommand_type subcommand = bq27427_control_subcommand_Chemistry_A;
	switch(chemistryType) {
	case bq27427_chemistry_LiPo4V20:
		subcommand = bq27427_control_subcommand_Chemistry_B;
		break;
	case bq27427_chemistry_Lipo4V35:
		subcommand = bq27427_control_subcommand_Chemistry_A;
		break;
	case bq27427_chemistry_LiPo4V4:
		subcommand = bq27427_control_subcommand_Chemistry_C;
		break;
	default:
		assert(false);
		break;
	}
	bool sealed = is_gauge_sealed();
	assert(sealed == false);
	bool configUpdateMode = is_config_update();
	assert(configUpdateMode != false);
	// we must perform subcommand write and wait for configuration update to finish before continue
	// with the operation.
	int cntr = 0;
	do {
		// update the chemistry identifier
		update_chemistry_id(subcommand, chemistryType);
		k_msleep(100);
		// check the chemistry identifier
		prevChemistry =
			(enum bq27427_chemistry_type) bq27427_i2c_read_control(bq27427_control_subcommand_ChemistryIdentifier);
		cntr ++;
	} while(prevChemistry != chemistryType && cntr < 120);
	assert(prevChemistry == chemistryType);
}

static inline void program_2_byte_parameter_in_memory(enum bq27427_flash_class_type cls,
														uint8_t offset,
														uint16_t value) {
	bool sealed = is_gauge_sealed();
	assert(sealed == false);
	bool configUpdateMode = is_config_update();
	assert(configUpdateMode != false);
	// read the current capacity from the memory
	enable_data_memory_access(cls);
	// write the design parameters first
	uint16_t prevVal = 0;
	do {
		write_data_memory(offset, &value, sizeof(uint16_t));
		//		k_msleep(1200);
		read_data_memory(offset, &prevVal, sizeof(uint16_t));
		prevVal = swap_bytes16(prevVal);
	} while(prevVal != value);
}

static inline void program_byte_parameter_in_memory(enum bq27427_flash_class_type cls,
													  uint8_t offset,
													  uint8_t value) {
	bool sealed = is_gauge_sealed();
	assert(sealed == false);
	bool configUpdateMode = is_config_update();
	assert(configUpdateMode != false);
	// read the current capacity from the memory
	enable_data_memory_access(cls);
	// write the design parameters first
	uint8_t prevVal = 0;
	do {
		write_data_memory(offset, (uint16_t*) &value, sizeof(uint8_t));
		//		k_msleep(1200);
		read_data_memory(offset, (uint16_t*) &prevVal, sizeof(uint8_t));
	} while(prevVal != value);
}

static inline void bq27427_program_design_parameters(struct bq27427_config_t* config) {
	// now we have to check whether the device is sealed or now
	bool sealed = is_gauge_sealed();
	assert(sealed == false);
	bool configUpdateMode = is_config_update();
	assert(configUpdateMode != false);
	// -----------------------------------------------------------------------------
	// write the configuration parameters first
	union bq27427_op_config_register_t opConfig = {.value = 0};
	uint16_t shortValue;
	enable_data_memory_access(bq27427_flash_class_ConfigurationRegisters);
	read_data_memory(BQ27427_OP_CONFIG_MEMORY_OFFSET, &shortValue, sizeof(uint16_t));
	opConfig.value = swap_bytes16(shortValue);
	opConfig.bits.bBatteryInsertionEnable = 0;
	opConfig.bits.bBatteryLowEnable = 1;
	// opConfig.bits.bSleepEnable = 0;
	program_2_byte_parameter_in_memory(bq27427_flash_class_ConfigurationRegisters,
									   BQ27427_OP_CONFIG_MEMORY_OFFSET,
									   opConfig.value);

	uint8_t byteValue = 0x02; //--> this is fixed for our designs
	program_byte_parameter_in_memory(bq27427_flash_class_GasGaugeState, BQ27427_LOAD_MODE_MEMORY_OFFSET, byteValue);
	program_2_byte_parameter_in_memory(bq27427_flash_class_GasGaugeState,
									   BQ27427_DESIGN_CAPACITY_MEMORY_OFFSET,
									   config->battery_capacity);
	program_2_byte_parameter_in_memory(bq27427_flash_class_GasGaugeState, BQ27427_DESIGN_ENERGY_MEMORY_OFFSET, config->battery_energy);
	program_2_byte_parameter_in_memory(bq27427_flash_class_GasGaugeState,
									   BQ27427_TERMINATION_VOLTAGE_MEMORY_OFFSET,
									   config->battery_termination_voltage);
	program_2_byte_parameter_in_memory(bq27427_flash_class_GasGaugeState,
									   BQ27427_TAPER_RATE_MEMORY_OFFSET,
									   config->taper_rate);
	program_2_byte_parameter_in_memory(bq27427_flash_class_GasGaugeState,
									   BQ27427_SLEEP_CURRENT_MEMORY_OFFSET,
									   config->gauge_sleep_current);
	// -----------------------------------------------------------------------------
	// write current thresholds
	program_2_byte_parameter_in_memory(bq27427_flash_class_CurrentThreshold,
									   BQ27427_DISCHARGE_CURRENT_THRESHOLD_MEMORY_OFFSET,
									   config->discharge_current_threshold);
	program_2_byte_parameter_in_memory(bq27427_flash_class_CurrentThreshold,
									   BQ27427_CHARGE_CURRENT_THRESHOLD_MEMORY_OFFSET,
									   config->charge_current_threshold);
	program_2_byte_parameter_in_memory(bq27427_flash_class_CurrentThreshold,
									   BQ27427_QUIT_CURRENT_THRESHOLD_MEMORY_OFFSET,
									   config->quit_current_threshold);
	// -----------------------------------------------------------------------------
	// write the chemistry parameters
	program_2_byte_parameter_in_memory(bq27427_flash_class_ChemistryData,
									   BQ27427_V_AT_CHARGE_TERM_MEMORY_OFFSET,
									   config->voltage_at_charge_termination);
	program_2_byte_parameter_in_memory(bq27427_flash_class_ChemistryData,
									   BQ27427_TAPER_VOLTAGE_MEMORY_OFFSET,
									   config->taper_voltage);
	// -----------------------------------------------------------------------------
	// write the RAM table values
	// --- This part cannot be done like this.
	// for(uint8_t i = 0; i < BQ27427_RAM_TABLE_SIZE; i++) {
	// 	shortValue = config->ra_values[i];
	// 	program_2_byte_parameter_in_memory(bq27427_flash_class_RaTables, i * 2, shortValue);
	// }
}

static inline void bq27427_flip_current_gain(void) {
	// now we have to check whether the device is sealed or now
	bool sealed = is_gauge_sealed();
	assert(sealed == false);
	bool configUpdateMode = is_config_update();
	assert(configUpdateMode != false);
	// read the current capacity from the memory
	enable_data_memory_access(bq27427_flash_class_Calibration);
	// write the design parameters first
	uint8_t ccGain = 0;
	read_data_memory(BQ27427_CC_GAIN_MEMORY_OFFSET + 1, (uint16_t*) &ccGain, sizeof(uint8_t));
	if((ccGain & 0x80) != 0) {
		ccGain ^= 0x80;
		program_byte_parameter_in_memory(bq27427_flash_class_Calibration, BQ27427_CC_GAIN_MEMORY_OFFSET + 1, ccGain);
	}
}

static inline void gauge_initialization_configure(struct bq27427_config_t* config){
	// now we have to check whether the device is sealed or now
	const bool sealed = is_gauge_sealed();
	if(sealed != false) {
		// unseal the device
		unseal_gauge();
	}
	enter_configuration_update_mode();
	k_msleep(1200);
	// now put the device into config update mode

	// we must first check the chemistry id. It is crucial to update the chemistry identifier before
	// writing anything to the device
	bq27427_program_chemistry_id(config->battery_type);
	// we can now configure the design parameters
	bq27427_flip_current_gain();
	bq27427_program_design_parameters(config);
	exit_configuration_update_mode();
	// seal the gauge
	seal_gauge();
}

bool bq27427_config(struct bq27427_config_t* config) {
	assert(bq27427.state.bits.bInitialized != 0);
	bool resetDetected = false;
	bool configure = false;
	bool initializing = false;
	// bool ret = BQ27427_I2C_LOCK(&bq27427, &bq27427.i2c_config, BQ27427_I2C_TIMEOUT);
	// if(ret == false) {
	// 	return false;
	// }

	union bq27427_control_status_register_t controlStatus = {.value = 0};
	union bq27427_flags_register_t flags = {.value = 0};
	enum bq27427_chemistry_type prevChemistry = read_chemistry_id();
	LOG_INF("BQ27427: Chemistry ID 0x%04X required 0x%04X\r\n", prevChemistry, config->battery_type);
	// check whether the battery is just inserted
	flags.value = bq27427_i2c_read_w_command(bq27427_command_Flags);
	// check the state
	if(flags.bits.bPOROrReset != 0){
		// we are in the POR state, so we must clear this bit by sending soft reset command
		clear_port_or_reset();
		configure = true;
		flags.value = bq27427_i2c_read_w_command(bq27427_command_Flags);
		controlStatus = bq27427_i2c_read_control_status();

		// check whether the gauge is in initialization state
		initializing = controlStatus.bits.bInitComplete == 0;
		resetDetected = true;
	} else {
		// check whether the previous configuration has failed
		if(prevChemistry != config->battery_type){
			configure = true;
		}
	}
	// configure if needed
	if(configure != false){
		gauge_initialization_configure(config);
		battery_remove();
		k_msleep(4000);
		battery_insert();
		k_msleep(2000);
	}
	flags.value = bq27427_i2c_read_w_command(bq27427_command_Flags);
	if(flags.bits.bBatteryDetected == 0 && resetDetected == false) {
		LOG_INF("BQ27427: battery not detected\r\n");
		bq27427_reset(NULL);
		LOG_INF("BQ27427: Gauge Reset\r\n");
		k_msleep(2000);
	}
	if(initializing != false) {
		// check whether the gauge is in initialization state
		do {
			controlStatus = bq27427_i2c_read_control_status();
		} while(controlStatus.bits.bInitComplete == 0);
	}

	memcpy(&(bq27427.config), config, sizeof(struct bq27427_config_t));
	bq27427.state.bits.bConfigured = 1;
	// BQ27427_I2C_UNLOCK(&bq27427);
	return true;
}

/*
*	uint16_t battery_capacity; // offset 6
	uint16_t battery_energy; // offset 8
	uint16_t battery_termination_voltage; // offset 10
	uint16_t taper_rate; // offset 21
	uint16_t gauge_sleep_current; // offset 23
	// -----------------------------------------------------------------------------
	// Gas gauging class -- Current thresholds subclass -- 81
	uint16_t discharge_current_threshold; // offset 0
	uint16_t charge_current_threshold; // offset 2
	uint16_t quit_current_threshold; // offset 4
	// -----------------------------------------------------------------------------
	// Chemistry Info class -- Chem Data subclass -- 109
	uint16_t voltage_at_charge_termination; // offset 6
	uint16_t taper_voltage; // offset 8
	// -----------------------------------------------------------------------------
	// Ra Tables class -- Ra0 RAM -- 89
	uint16_t ra_values[15];
 */
void bq27427_get_default_config(struct bq27427_config_t* config) {
	// const uint16_t ra_defaults[] = BQ27427_DEFAULT_RA_VALUES;

	config->battery_type = BQ27427_DEFAULT_BATTERY_TYPE;
	config->battery_capacity = BQ27427_DEFAULT_BATTERY_CAPACITY;
	config->battery_energy = BQ27427_DEFAULT_BATTERY_ENERGY;
	config->battery_termination_voltage = BQ27427_DEFAULT_BATTERY_TERMINATION_VOLTAGE;
	config->taper_rate = BQ27427_DEFAULT_TAPER_RATE;
	config->gauge_sleep_current = BQ27427_DEFAULT_GAUGE_SLEEP_CURRENT;

	config->discharge_current_threshold = BQ27427_DEFAULT_DISCHARGE_CURRENT_THRESHOLD;
	config->charge_current_threshold = BQ27427_DEFAULT_CHARGE_CURRENT_THRESHOLD;
	config->quit_current_threshold = BQ27427_DEFAULT_QUIT_CURRENT_THRESHOLD;

	config->voltage_at_charge_termination = BQ27427_DEFAULT_V_AT_CHARGE_TERM;
	config->taper_voltage = BQ27427_DEFAULT_TAPER_VOLTAGE;

	// memcpy(config->ra_values, ra_defaults, sizeof(ra_defaults));
}

bool bq27427_update_state(struct bq27427_battery_state_t* state) {
	assert(bq27427.state.bits.bConfigured != 0);
	// bool ret = BQ27427_I2C_LOCK(&bq27427, &bq27427.i2c_config, BQ27427_I2C_TIMEOUT);
	// if(ret == false) {
	// 	return false;
	// }
	// bool configure = false;
	union bq27427_flags_register_t flags = {.value = 0};

	// check whether the battery is just inserted
	flags.value = bq27427_i2c_read_w_command(bq27427_command_Flags);
	// check the state
	if(flags.bits.bPOROrReset != 0){
		// we are in the POR state, so we must clear this bit by sending soft reset command
		clear_port_or_reset();
		// configure = true;
		LOG_INF("BQ27427: POR detected while reading the state --- This CANNIT happen!!!!\r\n");
		assert(false);
	}
	// configure if needed
	// if(configure != false){
	//
	// }

	bq27427.battery_state.temperature = bq27427_i2c_read_w_command(bq27427_command_Temperature) - 2730; // in Celsius

	bq27427.battery_state.voltage = bq27427_i2c_read_w_command(bq27427_command_Voltage);

	bq27427.battery_state.average_current = (int16_t) bq27427_i2c_read_w_command(bq27427_command_AverageCurrent);

	bq27427.battery_state.average_power = (int16_t) bq27427_i2c_read_w_command(bq27427_command_AveragePower);

	bq27427.battery_state.state_of_charge = bq27427_i2c_read_w_command(bq27427_command_StateOfCharge) * 10; // in 0.1%

	bq27427.battery_state.remaining_capacity = bq27427_i2c_read_w_command(bq27427_command_RemainingCapacity);

	bq27427.battery_state.full_battery_capacity = bq27427_i2c_read_w_command(bq27427_command_FullAvailableCapacity);

	bq27427.battery_state.nominal_available_capacity =
		bq27427_i2c_read_w_command(bq27427_command_NominalAvailableCapacity);
	// BQ27427_I2C_UNLOCK(&bq27427);
	if(state != NULL) {
		memcpy(state, &(bq27427.battery_state), sizeof(struct bq27427_battery_state_t));
	}
	return true;
}

/*
 * \brief Prints the latest battery state
 */
void bq27427_print_state(void) {
	union bq27427_control_status_register_t controlStatus = {.value = 0};
	controlStatus.value = bq27427_i2c_read_control(bq27427_control_subcommand_ControlStatus);
	union bq27427_flags_register_t flags;
	flags.value = bq27427_i2c_read_w_command(bq27427_command_Flags);
	LOG_INF("%d", flags.value);
	LOG_INF("Battery state: T=%d V=%d A=%d P=%d SOC=%d AvailCap=%d "
					"FullCap=%d RemainingCap=%d FLAGS: 0x%04x CS: 0x%04x\r\n",
					bq27427.battery_state.temperature,
					bq27427.battery_state.voltage,
					bq27427.battery_state.average_current,
					bq27427.battery_state.average_power,
					bq27427.battery_state.state_of_charge,
					bq27427.battery_state.nominal_available_capacity,
					bq27427.battery_state.full_battery_capacity,
					bq27427.battery_state.remaining_capacity,
					flags.value,
					controlStatus.value);
}

bool bq27427_reset(struct bq27427_config_t* config) {
	union bq27427_flags_register_t flags = {.value = 0};
	/* Driver must be initialised before we can issue a reset */
	assert(bq27427.state.bits.bInitialized != 0);

	/* ---------------------------------------------------------------------
	 * 1. Issue the RESET (0x0041) control sub-command
	 * -------------------------------------------------------------------*/
	// bool ret = BQ27427_I2C_LOCK(&bq27427, &bq27427.i2c_config, BQ27427_I2C_TIMEOUT);
	// if(ret == false) {
	// 	return false;
	// }

	/* Write the reset control word (helper takes care of endianess) */
	bq27427_i2c_write_control(bq27427_control_subcommand_Reset);
	k_msleep(10000); /* ~1 ms is sufficient */
	flags.value = bq27427_i2c_read_w_command(bq27427_command_Flags);

	/* ---------------------------------------------------------------------
	 * 2. Give the fuel-gauge a brief moment to reboot and become ready
	 * -------------------------------------------------------------------*/
	k_msleep(2); /* ~1 ms is sufficient */
	int ready;
	do {
		ready = device_is_ready(bq27427.i2c.bus);
		k_msleep(2); /* ~1 ms is sufficient */
	} while(!ready);

	/* ---------------------------------------------------------------------
	 * 3. Local bookkeeping
	 * -------------------------------------------------------------------*/
	bq27427.state.bits.bConfigured = 0; /* reset cleared cfg */

	/* ---------------------------------------------------------------------
	 * 4. Re-apply configuration if non-null
	 * -------------------------------------------------------------------*/
	if(config != NULL) {
		union bq27427_control_status_register_t controlStatus = {.value = 0};
		enum bq27427_chemistry_type prevChemistry = read_chemistry_id();
		LOG_INF("BQ27427: Chemistry ID 0x%04X required 0x%04X\r\n", prevChemistry, config->battery_type);
		// check whether the battery is just inserted
		flags.value = bq27427_i2c_read_w_command(bq27427_command_Flags);
		// check the state
		// we are in the POR state, so we must clear this bit by sending soft reset command
		clear_port_or_reset();
		flags.value = bq27427_i2c_read_w_command(bq27427_command_Flags);
		controlStatus = bq27427_i2c_read_control_status();
		// configure
		gauge_initialization_configure(config);
		battery_remove();
		k_msleep(4000);
		battery_insert();
		k_msleep(2000);
		flags.value = bq27427_i2c_read_w_command(bq27427_command_Flags);
		while(flags.bits.bBatteryDetected == 0) {
			LOG_INF("BQ27427: battery not detected\r\n");
			k_msleep(2000);
			flags.value = bq27427_i2c_read_w_command(bq27427_command_Flags);
		}
		// check whether the gauge is in initialization state
		while(controlStatus.bits.bInitComplete == 0) {
			controlStatus = bq27427_i2c_read_control_status();
		}

		memcpy(&(bq27427.config), config, sizeof(struct bq27427_config_t));
		bq27427.state.bits.bConfigured = 1;
	}
	// BQ27427_I2C_UNLOCK(&bq27427);
	return true;
}
