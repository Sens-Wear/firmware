/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file max30208_config.h
 * @brief Compile-time tunables for the SensWear MAX30208 temperature driver.
 *
 * Applications may override any of these by defining the macro before the driver
 * is built.
 */

#ifndef MAX30208_CONFIG_H_
#define MAX30208_CONFIG_H_

/** Default per-sensor sampling rate in hertz when streaming is enabled. */
#ifndef MAX30208_DEFAULT_SAMPLING_RATE_HZ
#define MAX30208_DEFAULT_SAMPLING_RATE_HZ (1u)
#endif

/** Poll interval, in milliseconds, while waiting for a conversion to complete. */
#ifndef MAX30208_CONVERSION_POLL_MS
#define MAX30208_CONVERSION_POLL_MS (10u)
#endif

/** Maximum time, in milliseconds, to wait for a single conversion to complete. */
#ifndef MAX30208_CONVERSION_TIMEOUT_MS
#define MAX30208_CONVERSION_TIMEOUT_MS (200u)
#endif

/** Default FIFO almost-full threshold (FIFO_CONFIG1 A_FULL field). */
#ifndef MAX30208_FIFO_A_FULL_THRESHOLD
#define MAX30208_FIFO_A_FULL_THRESHOLD (0x0Fu)
#endif

/** Default FIFO roll-over enable (FIFO_CONFIG2 FIFO_RO). */
#ifndef MAX30208_FIFO_ROLLOVER
#define MAX30208_FIFO_ROLLOVER (1u)
#endif

/** Default FIFO almost-full assertion type (FIFO_CONFIG2 A_FULL_TYPE). */
#ifndef MAX30208_FIFO_A_FULL_TYPE
#define MAX30208_FIFO_A_FULL_TYPE (0u)
#endif

/** Default clear-status-on-FIFO-read behaviour (FIFO_CONFIG2 FIFO_STAT_CLR). */
#ifndef MAX30208_FIFO_STATUS_CLEAR
#define MAX30208_FIFO_STATUS_CLEAR (0u)
#endif

/** Default TEMP_RDY interrupt enable (INTERRUPT_ENABLE). */
#ifndef MAX30208_INT_TEMP_READY_ENABLE
#define MAX30208_INT_TEMP_READY_ENABLE (0u)
#endif

/** Default A_FULL interrupt enable (INTERRUPT_ENABLE). */
#ifndef MAX30208_INT_A_FULL_ENABLE
#define MAX30208_INT_A_FULL_ENABLE (0u)
#endif

#endif /* MAX30208_CONFIG_H_ */
