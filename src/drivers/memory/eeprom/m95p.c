
#include "m95p.h"
#include <assert.h>
#include <stdbool.h>
#include <string.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(SENSE_WEAR_MEMORY_DRIVER_LOGGER);
#if HAVE_FATFS

#include "ff.h"
#include "sys_debug.h"

#define FS_SUPPORTED_SECTOR_SIZE (FF_MAX_SS)
#else
#define FS_SUPPORTED_SECTOR_SIZE (M95P_SECTOR_SIZE)
#endif

#define SYS_MEMORY_SIZE (M95P_SIZE)
#define SYS_MEMORY_PAGE_SIZE (M95P_PAGE_SIZE)
#define SYS_MEMORY_SECTOR_SIZE (M95P_SECTOR_SIZE)
#define SYS_MEMORY_BLOCK_SIZE (M95P_BLOCK_SIZE)
#define SYS_MEMORY_TOTAL_SECTOR_COUNT (M95P_SECTOR_COUNT)
#define SYS_MEMORY_PAGE_COUNT (SYS_MEMORY_SIZE / SYS_MEMORY_PAGE_SIZE)
#define SYS_MEMORY_PAGES_PER_SECTOR \
    (SYS_MEMORY_SECTOR_SIZE / SYS_MEMORY_PAGE_SIZE)
#define SYS_MEMORY_SECTORS_PER_BLOCK \
    (SYS_MEMORY_BLOCK_SIZE / SYS_MEMORY_SECTOR_SIZE)
#define SYS_MEMORY_MAX_PROTECTION_BLOCK_COUNT M95P_MAX_PROTECTION_BLOCK_COUNT

#define SYS_MEMORY_SUPPORTED_SECTOR_SIZE FS_SUPPORTED_SECTOR_SIZE
#define SYS_MEMORY_SECTOR_USAGE_RATIO \
    (SYS_MEMORY_SECTOR_SIZE / SYS_MEMORY_SUPPORTED_SECTOR_SIZE)

#ifndef SYS_MEMORY_GOLDEN_SECTION_BLOCK_COUNT
#define SYS_MEMORY_GOLDEN_SECTION_BLOCK_COUNT (0)
#endif

#if SYS_MEMORY_GOLDEN_SECTION_BLOCK_COUNT > 0
#if SYS_MEMORY_GOLDEN_SECTION_BLOCK_COUNT > \
    SYS_MEMORY_MAX_PROTECTION_BLOCK_COUNT
#error "Defined GOLDEN SECTION block count is larger than what can be protected"
#endif
#endif

#define SYS_MEMORY_GOLDEN_SECTION_SIZE \
    (SYS_MEMORY_GOLDEN_SECTION_BLOCK_COUNT * SYS_MEMORY_BLOCK_SIZE)

#define SYS_MEMORY_GOLDEN_SECTION_SECTOR_COUNT \
    (SYS_MEMORY_GOLDEN_SECTION_SIZE / SYS_MEMORY_SECTOR_SIZE)
#if SYS_MEMORY_GOLDEN_SECTION_SECTOR_COUNT >= SYS_MEMORY_TOTAL_SECTOR_COUNT
#error "GOLDEN section configuration requires larger memory"
#endif

#define SYS_MEMORY_GOLDEN_SECTION_SECTOR_START \
    (SYS_MEMORY_TOTAL_SECTOR_COUNT - SYS_MEMORY_GOLDEN_SECTION_SECTOR_COUNT)

#define SYS_MEMORY_GOLDEN_SECTION_BLOCK_START \
    (SYS_MEMORY_GOLDEN_SECTION_SECTOR_START / SYS_MEMORY_SECTORS_PER_BLOCK)

#define SYS_MEMORY_GOLDEN_SECTION_PAGE_COUNT \
    (SYS_MEMORY_GOLDEN_SECTION_SECTOR_COUNT * SYS_MEMORY_PAGES_PER_SECTOR)
#define SYS_MEMORY_GOLDEN_SECTION_PAGE_START \
    (SYS_MEMORY_GOLDEN_SECTION_SECTOR_START * SYS_MEMORY_PAGES_PER_SECTOR)

#define SYS_MEMORY_GOLDEN_SECTION_ADDRESS_START \
    (SYS_MEMORY_GOLDEN_SECTION_PAGE_START * SYS_MEMORY_PAGE_SIZE)

#define SYS_MEMORY_SECTOR_COUNT \
    (SYS_MEMORY_TOTAL_SECTOR_COUNT - SYS_MEMORY_GOLDEN_SECTION_SECTOR_COUNT)

bool M95P_SPI_LOCK (void*, void*, int) {
    return true;
}

bool M95P_SPI_UNLOCK (void*) {
    return true;
}

// void __NOP(void) {
//     (void)0;
// }

/**
 * \brief The m95p device driver structure
 */
static struct m95p_t {
    const struct device *cs_gpio;
    const struct spi_dt_spec *spi_driver; //! SPI driver
    const struct spi_config *spi_config;       //! SPI driver configuration
    union m95p_status_register_t status_register;
    struct m95p_configuration_safety_registers_t config_safety_registers;
    union m95p_volatile_register_t volatile_register;

    union m95p_jedec_id_t jedec_id;
} m95p = {0};

union address_t {
    uint32_t value;
    uint8_t bytes[4];
};

static inline void m95p_command_with_address(uint8_t *buffer,
                                               uint8_t command,
                                               uint32_t address) {
    union address_t address_union;
    address_union.value = address;
    buffer[0] = command;
    buffer[1] = address_union.bytes[2];
    buffer[2] = address_union.bytes[1];
    buffer[3] = address_union.bytes[0];
}

static inline void m95p_cs_high() {
    gpio_pin_set(m95p.cs_gpio, M95P_CS_PIN, 1);
}

static inline void m95p_cs_low() {
    gpio_pin_set(m95p.cs_gpio, M95P_CS_PIN, 0);
}

static inline void mp95p_spi_read(uint8_t *command,
                                    size_t commandSize,
                                    uint8_t *buffer,
                                    size_t readSize) {
    struct spi_buf tx_buf = {
        .buf = command,
        .len = commandSize
    };
    struct spi_buf_set tx_bufs = {
        .buffers = &tx_buf,
        .count = 1
    };

    struct spi_buf rx_buf[2] = {
        {
            .buf = NULL,
            .len = commandSize // Skip bytes while sending command
        },
        {
            .buf = buffer,
            .len = readSize    // Actual data to read
        }
    };
    struct spi_buf_set rx_bufs = {
        .buffers = rx_buf,
        .count = 2
    };

    spi_transceive(m95p.spi_driver->bus, m95p.spi_config, &tx_bufs, &rx_bufs);
}

static inline void mp95p_spi_write(uint8_t *command,
                                     size_t commandSize,
                                     const uint8_t *buffer,
                                     size_t writeSize) {
    int ret;
    
    // We can have up to 2 buffers: the command and the data payload
    struct spi_buf tx_bufs_array[2];
    uint8_t buf_count = 0;

    // 1. Add the command buffer
    tx_bufs_array[buf_count].buf = command;
    tx_bufs_array[buf_count].len = commandSize;
    buf_count++;

    // 2. Add the data buffer if it exists and has size
    if (writeSize > 0 && buffer != NULL) {
        tx_bufs_array[buf_count].buf = (void *)buffer;
        tx_bufs_array[buf_count].len = writeSize;
        buf_count++;
    }

    // Wrap the array in a buffer set
    struct spi_buf_set tx_bufs = {
        .buffers = tx_bufs_array,
        .count = buf_count
    };

    // 3. Execute the write. 
    // Passing NULL for rx_bufs indicates a write-only operation.
    ret = spi_write(m95p.spi_driver->bus, m95p.spi_config, &tx_bufs);
    assert(ret == 0);
}

static inline void m95p_spi_write_enable() {
    uint8_t command = m95p_instruction_WREN;
    m95p_cs_low();
    mp95p_spi_write(&command, 1, NULL, 0);
    m95p_cs_high();
}
/*
static inline void m95p_spi_write_disable() {
    uint8_t command = m95p_instruction_WRDI;
    m95p_cs_low();
    mp95p_spi_write(&command, 1, NULL, 0);
    m95p_cs_high();
}
*/
static inline void
m95p_spi_read_status_register(union m95p_status_register_t *status) {
    uint8_t command = m95p_instruction_RDSR;
    uint8_t value;
    m95p_cs_low();
    mp95p_spi_read(&command, 1, &value, 1);
    status->value = value;
    m95p_cs_high();
}

static inline void m95p_spi_read_configuration_safety_register(
        struct m95p_configuration_safety_registers_t *config) {
    uint8_t buffer[2];
    uint8_t command = m95p_instruction_RDCR;
    m95p_cs_low();
    mp95p_spi_read(&command, 1, buffer, 2);
    m95p_cs_high();
    config->configuration_register.value = buffer[0];
    config->safety_register.value = buffer[1];
}

static inline void m95p_clear_safety_flags() {
    uint8_t command = m95p_instruction_CLRSF;
    m95p_cs_low();
    mp95p_spi_write(&command, 1, NULL, 0);
    m95p_cs_high();
}
/*
static inline void
m95p_read_volatile_register(union m95p_volatile_register_t *reg) {
    uint8_t command = m95p_instruction_RDVR;
    m95p_cs_low();
    mp95p_spi_read(&command, 1, (uint8_t *) reg, 1);
    m95p_cs_high();
}

static inline void
m95p_write_volatile_register(union m95p_volatile_register_t *reg) {
    uint8_t command = m95p_instruction_WRVR;
    m95p_cs_low();
    mp95p_spi_write(&command, 1, (uint8_t *) reg, 1);
    m95p_cs_high();
}
*/
static inline void m95p_write_status_and_configuration_register(union m95p_status_register_t statusRegister,
                                                                  union m95p_configuration_register_t configurationRegister,
                                                                  bool bStatusOnly) {
    uint8_t buffer[4];
    size_t size = 1;
    uint8_t command = m95p_instruction_WRSR;
    buffer[0] = statusRegister.value;
    if (bStatusOnly == false) {
        buffer[1] = configurationRegister.value;
        buffer[2] = 0x00;
        buffer[3] = 0x00;
        size += 1;
    }
    m95p_cs_low();
    mp95p_spi_write(&command, 1, buffer, size);
    m95p_cs_high();
}

static inline void m95p_read_jedec_id(union m95p_jedec_id_t *id) {
    uint8_t command[4];
    m95p_command_with_address(command, m95p_instruction_RDID, 0);
    m95p_cs_low();
    mp95p_spi_read(command, 4, id->data, 3);
    m95p_cs_high();
}
/*
static inline void m95p_deep_power_down_enter(void) {
    uint8_t command = m95p_instruction_DPD;
    m95p_cs_low();
    mp95p_spi_write(&command, 1, NULL, 0);
    m95p_cs_high();
}

static inline void m95p_deep_power_down_exit(void) {
    uint8_t command = m95p_instruction_RDPD;
    m95p_cs_low();
    mp95p_spi_write(&command, 1, NULL, 0);
    m95p_cs_high();
}
*/
static inline void m95p_reset(void) {
    uint8_t command = m95p_instruction_RSTEN;
    m95p_cs_low();
    mp95p_spi_write(&command, 1, NULL, 0);
    m95p_cs_high();
    command = m95p_instruction_RESET;
    for (int i = 10; i > 0; i--) {
        __NOP();
    }
    m95p_cs_low();
    mp95p_spi_write(&command, 1, NULL, 0);
    m95p_cs_high();
}

static inline bool m95p_is_busy(void) {
    union m95p_status_register_t status;
    m95p_spi_read_status_register(&status);
    return status.bits.WIP != 0;
}

static inline bool m95p_is_error(void) {
    struct m95p_configuration_safety_registers_t config;
    m95p_spi_read_configuration_safety_register(&config);
    return (config.safety_register.bits.ERF != 0) ||
           (config.safety_register.bits.PRF != 0);
}

static inline void m95p_clear_error(void) {
    m95p_clear_safety_flags();
}

static inline void m95p_wait_until_not_busy(void) {
    bool busy = m95p_is_busy();
    bool error;
    while (busy != false) {
        __NOP();
        busy = m95p_is_busy();
    }
    error = m95p_is_error();
    if (error != false) {
        m95p_clear_error();
    }
}

static inline bool m95p_is_write_protected(void) {
    union m95p_status_register_t status;
    m95p_spi_read_status_register(&status);
    return status.bits.SRWD != 0;
}

bool sys_memory_init(void *arg) {
    (void) arg;
    if (m95p.cs_gpio == NULL) {
        static const struct device *cs_gpio = DEVICE_DT_GET(M95P_CS_GPIO_NODE);
        if (!device_is_ready(cs_gpio)) {
            return -ENODEV;
        }
        int ret = gpio_pin_configure(cs_gpio,
                              M95P_CS_PIN,
                              GPIO_OUTPUT_ACTIVE);
        assert(ret == 0);
        m95p.cs_gpio = cs_gpio;
    }
    if (m95p.spi_driver == NULL) {
        static const struct spi_dt_spec spi_dev =
            SPI_DT_SPEC_GET(DT_NODELABEL(eeprom0),
                            SPI_OP_MODE_MASTER |
                            SPI_WORD_SET(8),
                            0);
        if (!spi_is_ready_dt(&spi_dev)) {
            LOG_ERR("SPI device not ready");
            return -ENODEV;
        }
        m95p.spi_driver = &spi_dev;
        m95p.spi_config = &spi_dev.config;
        //		m95p_deep_power_down_exit();
        m95p_reset();
		for(int ii =0 ; ii < 10000; ii++){
			__NOP();
		}
        //---------------------------------------------------------------------
        // first read the device type information
        m95p_read_jedec_id(&(m95p.jedec_id));
        if (m95p.jedec_id.fields.manufacturer_id != M95P_MANUFACTURER_ID ||
            m95p.jedec_id.fields.memory_type != M95P_FAMILY_CODE ||
            m95p.jedec_id.fields.capacity != M95P_MEMORY_DENSITY) {
            LOG_ERR("M95P JEDEC identification is invalid!\r\n");
            assert(false);
        }
        //---------------------------------------------------------------------
        // now read the unique identifier
        //---------------------------------------------------------------------
        // if it is needed further configuration can be transmitted to the
        // device here. For now, we ignore these, and just indicate the device
        // is configured and exit the function.
        //---------------------------------------------------------------------
        // enable writing to the device
        m95p_spi_write_enable();

        // clear any pending error flags
        m95p_clear_error();
    }
    return true;
}

bool sys_memory_is_ready(void) {
    return (m95p.spi_driver != NULL && m95p.jedec_id.fields.manufacturer_id == M95P_MANUFACTURER_ID);
}

union m95p_jedec_id_t m95p_get_jedec_id(void) {
    assert(m95p.spi_driver != NULL);
    return m95p.jedec_id;
}

static inline void m95p_chip_erase(void) {
    uint8_t command = m95p_instruction_CHER;
    assert(m95p.spi_driver != NULL);
    // --------------------------------------------------------------------------------------------
    // wait till not busy
    m95p_wait_until_not_busy();
    m95p_spi_write_enable();
    // start erase
    m95p_cs_low();
    mp95p_spi_write(&command, 1, NULL, 0);
    m95p_cs_high();
    // wait till it ends
    m95p_wait_until_not_busy();
}

static inline void m95p_block_erase(uint32_t address) {
    uint8_t command[4];
    assert(m95p.spi_driver != NULL);
    //	address /= M95P_PAGE_SIZE;
    m95p_command_with_address(command, m95p_instruction_BKER, address);
    // wait till not busy
    m95p_wait_until_not_busy();
    m95p_spi_write_enable();
    // start erase
    m95p_cs_low();
    mp95p_spi_write(command, 4, NULL, 0);
    m95p_cs_high();
    // wait till it ends
    m95p_wait_until_not_busy();
}

static inline void m95p_page_erase(uint32_t address) {
    uint8_t command[4];
    assert(m95p.spi_driver != NULL);

    m95p_command_with_address(command, m95p_instruction_PGER, address);
    // wait till not busy
    m95p_wait_until_not_busy();
    m95p_spi_write_enable();
    // start erase
    m95p_cs_low();
    mp95p_spi_write(command, 4, NULL, 0);
    m95p_cs_high();
    // wait till it ends
    m95p_wait_until_not_busy();
}

bool  sys_memory_program_page(uint32_t page, const void *data, size_t size) {
    uint8_t command[4];
    uint32_t address = page * M95P_PAGE_SIZE;
    assert(m95p.spi_driver != NULL);
    // Lock SPI interface
    bool bRet = M95P_SPI_LOCK(&m95p, &(m95p.spi_config), M95P_SPI_TIMEOUT);
    if (bRet == false) {
        return false;
    }
    // --------------------------------------------------------------------------------------------
    // wait till not busy
    m95p_wait_until_not_busy();
    m95p_spi_write_enable();
    // start write
    m95p_command_with_address(command, m95p_instruction_PGPR, address);
    m95p_cs_low();
    mp95p_spi_write(command, 4, data, size);
    m95p_cs_high();
    // wait till it ends
    m95p_wait_until_not_busy();
    // --------------------------------------------------------------------------------------------
    M95P_SPI_UNLOCK(&m95p);
    return true;
}

static inline void   m95p_page_write(uint32_t address, const uint8_t *data, size_t size) {
    // wait till not busy
    uint8_t command[4];
    assert(m95p.spi_driver != NULL);
    //	address /= M95P_PAGE_SIZE;
    m95p_wait_until_not_busy();
    m95p_spi_write_enable();
    // start write
    m95p_command_with_address(command, m95p_instruction_PGWR, address);
    m95p_cs_low();
    mp95p_spi_write(command, 4, data, size);
    m95p_cs_high();
    // wait till it ends
    m95p_wait_until_not_busy();
}

static inline void  m95p_read(uint32_t address, uint8_t *buffer, size_t size) {
    uint8_t command[4];
    assert(m95p.spi_driver != NULL);
    // wait till previous operation ends
    m95p_wait_until_not_busy();
    m95p_command_with_address(command, m95p_instruction_READ, address);
    m95p_cs_low();
    mp95p_spi_read(command, 4, buffer, size);
    m95p_cs_high();
    // wait till it ends
    m95p_wait_until_not_busy();
}


size_t sys_memory_get_size(void) {
    return SYS_MEMORY_SECTOR_COUNT*SYS_MEMORY_PAGES_PER_SECTOR*SYS_MEMORY_PAGE_SIZE;
}

size_t sys_memory_get_sector_size(void) {
    return SYS_MEMORY_PAGE_SIZE;
}

size_t sys_memory_get_supported_sector_size(void) {
    return SYS_MEMORY_SUPPORTED_SECTOR_SIZE;
}

size_t sys_memory_get_page_size(void) {
    return SYS_MEMORY_PAGE_SIZE;
}

size_t sys_memory_get_block_size(void) {
    return SYS_MEMORY_BLOCK_SIZE;
}

size_t sys_memory_get_sector_count(void) {
    // we can perform page erase and program --> SECTOR == PAGE for this device
    return SYS_MEMORY_SECTOR_COUNT*SYS_MEMORY_PAGES_PER_SECTOR;
}

size_t sys_memory_get_page_count(void) {
    return SYS_MEMORY_SECTOR_COUNT*SYS_MEMORY_PAGES_PER_SECTOR;
}

size_t sys_memory_get_block_count(void) {
    return SYS_MEMORY_SECTOR_COUNT/SYS_MEMORY_SECTORS_PER_BLOCK;
}

size_t sys_memory_get_max_protected_block_count(void) {
    return SYS_MEMORY_MAX_PROTECTION_BLOCK_COUNT;
}

size_t sys_memory_get_golden_section_size(void) {
    return SYS_MEMORY_GOLDEN_SECTION_BLOCK_COUNT * SYS_MEMORY_BLOCK_SIZE;
}

size_t sys_memory_get_golden_section_page_count(void) {
    return SYS_MEMORY_GOLDEN_SECTION_BLOCK_COUNT * SYS_MEMORY_BLOCK_SIZE /
           SYS_MEMORY_PAGE_SIZE;
}

size_t sys_memory_get_golden_section_sector_count(void) {
    return SYS_MEMORY_GOLDEN_SECTION_BLOCK_COUNT * SYS_MEMORY_BLOCK_SIZE /
           SYS_MEMORY_PAGE_SIZE;
}

bool sys_memory_write_sector(uint32_t sector, const void *data, size_t size) {
    // M95P sector write is not needed since we can perform page level erase and program instructions
    // THUS: Sector erase actually performs page erase
    assert(m95p.spi_driver != NULL);
    uint32_t address = sector * SYS_MEMORY_PAGE_SIZE;
    // Lock SPI interface
    bool bRet = M95P_SPI_LOCK(&m95p, &(m95p.spi_config), M95P_SPI_TIMEOUT);
    if (bRet == false) {
        return false;
    }
    // --------------------------------------------------------------------------------------------
    m95p_page_write(address, data, size);
    // --------------------------------------------------------------------------------------------
    M95P_SPI_UNLOCK(&m95p);
    return true;
}

bool sys_memory_read(uint32_t address, void *data, size_t size) {
    assert(m95p.spi_driver != NULL);
    // Lock SPI interface
    bool bRet = M95P_SPI_LOCK(&m95p, &(m95p.spi_config), M95P_SPI_TIMEOUT);
    if (bRet == false) {
        return false;
    }
    // --------------------------------------------------------------------------------------------
    m95p_read(address, data, size);
    // --------------------------------------------------------------------------------------------
    M95P_SPI_UNLOCK(&m95p);
    return true;
}

bool sys_memory_erase_sector(uint32_t sector) {
    // M95P sector erase is not needed since we can perform page level erase and program instructions
    // THUS: Sector erase actually performs page erase
    assert(m95p.spi_driver != NULL);
    // Lock SPI interface
    bool bRet = M95P_SPI_LOCK(&m95p, &(m95p.spi_config), M95P_SPI_TIMEOUT);
    if (bRet == false) {
        return false;
    }
    // --------------------------------------------------------------------------------------------
    m95p_page_erase(sector * SYS_MEMORY_PAGE_SIZE);
    // --------------------------------------------------------------------------------------------
    M95P_SPI_UNLOCK(&m95p);
    return true;
}

bool sys_memory_erase_block(uint32_t block) {
    assert(m95p.spi_driver != NULL);
    // Lock SPI interface
    bool bRet = M95P_SPI_LOCK(&m95p, &(m95p.spi_config), M95P_SPI_TIMEOUT);
    if (bRet == false) {
        return false;
    }
    // --------------------------------------------------------------------------------------------
    m95p_block_erase(block * SYS_MEMORY_BLOCK_SIZE);
    // --------------------------------------------------------------------------------------------
    M95P_SPI_UNLOCK(&m95p);
    return true;
}

bool sys_memory_reset_to_factory_defaults(void) {
    assert(m95p.spi_driver != NULL);
    // Lock SPI interface
    bool bRet = M95P_SPI_LOCK(&m95p, &(m95p.spi_config), M95P_SPI_TIMEOUT);
    if (bRet == false) {
        return false;
    }
    // --------------------------------------------------------------------------------------------
    union m95p_status_register_t statusRegister = {.value = 0};
    struct m95p_configuration_safety_registers_t configurationSafetyRegisters = {0};
    m95p_spi_read_status_register(&statusRegister);
    statusRegister.bits.SRWD = 0;
    statusRegister.bits.BP = 0;

    m95p_spi_read_configuration_safety_register(&configurationSafetyRegisters);
    configurationSafetyRegisters.configuration_register.bits.LID = 0;

	m95p_spi_write_enable();
    m95p_write_status_and_configuration_register(statusRegister, configurationSafetyRegisters.configuration_register,
                                                 false);
    m95p_wait_until_not_busy();
    m95p_chip_erase();
    // --------------------------------------------------------------------------------------------
    M95P_SPI_UNLOCK(&m95p);
    return true;
}

bool sys_memory_reset(void) {
    assert(m95p.spi_driver != NULL);
    // Lock SPI interface
    bool bRet = M95P_SPI_LOCK(&m95p, &(m95p.spi_config), M95P_SPI_TIMEOUT);
    if (bRet == false) {
        return false;
    }
    // --------------------------------------------------------------------------------------------
    m95p_reset();
    // --------------------------------------------------------------------------------------------
    M95P_SPI_UNLOCK(&m95p);
    return true;
}

bool sys_memory_set_write_protection_state(bool bWriteProtect) {
    assert(m95p.spi_driver != NULL);
    // Lock SPI interface
    bool bRet = M95P_SPI_LOCK(&m95p, &(m95p.spi_config), M95P_SPI_TIMEOUT);
    if (bRet == false) {
        return false;
    }
    union m95p_status_register_t statusRegister = {.value = 0};
    m95p_spi_read_status_register(&statusRegister);

    bool prevState = statusRegister.bits.SRWD != 0;
    if (prevState != bWriteProtect) {
        statusRegister.bits.SRWD = bWriteProtect == false ? 0 : 1;
        // write enable
        m95p_wait_until_not_busy();
        m95p_spi_write_enable();
        // write the status register
        m95p_write_status_and_configuration_register(statusRegister, (union m95p_configuration_register_t) {.value = 0},
                                                     true);
        m95p_wait_until_not_busy();
    }

    M95P_SPI_UNLOCK(&m95p);
    return true;
}

bool sys_memory_is_write_protected(void) {
    assert(m95p.spi_driver != NULL);
    // Lock SPI interface
    bool bRet = M95P_SPI_LOCK(&m95p, &(m95p.spi_config), M95P_SPI_TIMEOUT);
    assert(bRet != false);
    // --------------------------------------------------------------------------------------------
    bool wpState = m95p_is_write_protected();
    // --------------------------------------------------------------------------------------------
    M95P_SPI_UNLOCK(&m95p);
    return wpState;
}

uint32_t sys_memory_write_protected_blocks_count(void) {
    assert(m95p.spi_driver != NULL);
    // Lock SPI interface
    bool bRet = M95P_SPI_LOCK(&m95p, &(m95p.spi_config), M95P_SPI_TIMEOUT);
    if (bRet == false) {
        return false;
    }
    // wait till previous operation ends
    union m95p_status_register_t statusRegister = {.value = 0};
    m95p_spi_read_status_register(&statusRegister);

    unsigned int bpValue = statusRegister.bits.BP;
    uint32_t wpBlocksCount = 1 << (bpValue - 1);

    M95P_SPI_UNLOCK(&m95p);
    return wpBlocksCount;;
}

void sys_memory_write_protect_blocks(uint32_t count, bool permanent) {
    (void) permanent;
    assert(m95p.spi_driver != NULL);
    // Lock SPI interface
    bool bRet = M95P_SPI_LOCK(&m95p, &(m95p.spi_config), M95P_SPI_TIMEOUT);
    assert(bRet != false);
    union m95p_status_register_t statusRegister = {.value = 0};
    m95p_spi_read_status_register(&statusRegister);

    unsigned int prevCount = 1 << (statusRegister.bits.BP);

    if (prevCount != count) {
        unsigned int bpValue = 0;

        // validate count is power of 2 and smaller than supported count
        if (count > 0) {
            for (unsigned int i = 1; i < 7; i++) {
                uint32_t mask = 1 << (i - 1);
                if (mask == count) {
                    bpValue = i;
                    break;
                }
            }
            assert(bpValue != 0);
        }
        statusRegister.bits.BP = bpValue;
		statusRegister.bits.TB = 0;
        // write enable
        m95p_wait_until_not_busy();
        m95p_spi_write_enable();
        // write the status register
        m95p_write_status_and_configuration_register(statusRegister, (union m95p_configuration_register_t) {.value = 0},
                                                     true);
        m95p_wait_until_not_busy();
    }

    M95P_SPI_UNLOCK(&m95p);
}

bool sys_memory_is_busy(void) {
    assert(m95p.spi_driver != NULL);
    // Lock SPI interface
    bool bRet = M95P_SPI_LOCK(&m95p, &(m95p.spi_config), M95P_SPI_TIMEOUT);
    assert(bRet != false);
    // --------------------------------------------------------------------------------------------
    bool busy = m95p_is_busy();
    // --------------------------------------------------------------------------------------------
    M95P_SPI_UNLOCK(&m95p);
    return busy;;
}


bool sys_memory_golden_section_read(uint32_t address, void *data, size_t size) {
#if (SYS_MEMORY_GOLDEN_SECTION_SIZE == 0)
    return false;
#else
    address += SYS_MEMORY_GOLDEN_SECTION_ADDRESS_START;
    assert((address + size) < SYS_MEMORY_SIZE);

    return sys_memory_read(address, data, size);
#endif
}

bool sys_memory_golden_section_program_page(uint32_t page,
                                            const void *data,
                                            size_t size) {
#if (SYS_MEMORY_GOLDEN_SECTION_SIZE == 0)
    return true;
#else
    page += SYS_MEMORY_GOLDEN_SECTION_PAGE_START;
    assert(((page * SYS_MEMORY_PAGE_SIZE) + size) < SYS_MEMORY_SIZE);

    return sys_memory_write_sector(page, data, size);
#endif
}

bool sys_memory_golden_section_erase_sector(uint32_t sector) {
#if (SYS_MEMORY_GOLDEN_SECTION_SIZE == 0)
    return true;
#else
	assert(false); // --> This must be aligned with page based erase/program of M95P
    sector += SYS_MEMORY_GOLDEN_SECTION_SECTOR_START;
    assert(sector < SYS_MEMORY_TOTAL_SECTOR_COUNT);
    return sys_memory_erase_sector(sector);
#endif
}

bool sys_memory_golden_section_erase(void) {
#if (SYS_MEMORY_GOLDEN_SECTION_SIZE == 0)
    return true;
#else
//    uint32_t page = SYS_MEMORY_GOLDEN_SECTION_PAGE_START;
//    for (int i = 0; i < SYS_MEMORY_GOLDEN_SECTION_PAGE_COUNT; i++, page++) {
//        bool ret = sys_memory_erase_sector(page);
//        if (ret == false) {
//            return false;
//        }
//    }
    return true;
#endif
}

bool sys_memory_golden_section_is_page_in(uint32_t page) {
    uint32_t address = page * SYS_MEMORY_PAGE_SIZE;
    return ((address >= SYS_MEMORY_GOLDEN_SECTION_ADDRESS_START) &&
            (address < SYS_MEMORY_SIZE))
           ? true
           : false;
}

bool sys_memory_golden_section_lock(void) {
    sys_memory_write_protect_blocks(SYS_MEMORY_GOLDEN_SECTION_BLOCK_COUNT, true);
    return true;
}

bool sys_memory_golden_section_unlock(void) {
    sys_memory_write_protect_blocks(0, false);
    return true;
}

void test_memory () {
    #define EEPROM_SAMPLE_OFFSET 0
    #define EEPROM_SAMPLE_MAGIC  0xEE9703    
    struct perisistant_values {
        uint32_t magic;
        uint32_t boot_count;
    };

    struct perisistant_values values;
    m95p_read(EEPROM_SAMPLE_OFFSET, &values, sizeof(values));
    if (values.magic != EEPROM_SAMPLE_MAGIC) {
		values.magic = EEPROM_SAMPLE_MAGIC;
		values.boot_count = 0;
	}
    values.boot_count++;
    LOG_INF("Device booted %d times.\n", values.boot_count);
    m95p_page_write(EEPROM_SAMPLE_OFFSET, &values, sizeof(values));
    LOG_INF("Reset the MCU to see the increasing boot counter.\n\n");
}