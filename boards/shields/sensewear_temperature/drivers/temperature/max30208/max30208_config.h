/**
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file max30208_config.h
 * @brief Compile-time tunables for the SenseWear MAX30208 temperature driver.
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

#endif /* MAX30208_CONFIG_H_ */
