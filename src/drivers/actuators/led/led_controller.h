
/*
 * pingbit_led_controller.h
 *
 *  Created on: Jul 19, 2017
 *      Author: yusei
 */

#ifndef _MATEONE_LED_CONTROLLER_H_
#define _MATEONE_LED_CONTROLLER_H_
#include <zephyr/drivers/i2c.h>
#include "lp5562.h"
#include "led_controller_config.h"

#define LED_PWM_STATE_COUNT (4u)

/**
 * \brief LP5562 driver status fields.
 */
union led_controller_status_t{
	int value;
	struct driver_status_bits{
		int bInitialized : 1;
		int bConfigured : 1;
	} bits;
};


/** \brief	Values that represent tag LED states. */
enum led_state_type{
	led_state_Off = 0,  /*!< An enum constant representing the LED state off option */
	led_state_On = 1,   /*!< An enum constant representing the LED state on option */
	led_state_PwmBlink = 2  /*!< An enum constant representing the LED state pwm blink option */
};

/** \brief	Values that represent tag LED gesture states. */
enum led_request_state_type{
	led_request_state_Inactive = 0, /*!< An enum constant representing the LED gesture state inactive option */
	led_request_state_Pending = 1,  /*!< An enum constant representing the LED gesture state pending option */
	led_request_state_WaitingApproval = 2,  /*!< An enum constant representing the LED gesture state waiting approval option */
	led_request_state_Active = 3	/*!< An enum constant representing the LED gesture state active option */
};

/** \brief	Values that represent tag LED gesture PWM states. */
enum led_gesture_pwm_state_type{
	led_pwm_FadeOn = 0, /*!< An enum constant representing the LED PWM fade on option */
	led_pwm_FullyOn = 1,	/*!< An enum constant representing the LED PWM fully on option */
	led_pwm_FadeOff = 2,	/*!< An enum constant representing the LED PWM fade off option */
	led_pwm_FullyOff = 3	/*!< An enum constant representing the LED PWM fully off option */
};

/** \brief	Values that represent tag LED PWM state durations. */
enum led_pwm_duration_type{
	led_pwm_duration_64ms = 1,  /*!< An enum constant representing the LED gesture PWM state duration 64ms option */
	led_pwm_duration_0ms = 0,	/*!< An enum constant representing the LED gesture PWM state duration state 0ms option */
	led_pwm_duration_128ms = 2,  /*!< An enum constant representing the LED gesture PWM state duration 128ms option */
	led_pwm_duration_256ms = 3,  /*!< An enum constant representing the LED gesture PWM state duration 256ms option */
	led_pwm_duration_512ms = 4,  /*!< An enum constant representing the LED gesture PWM state duration 512ms option */
	led_pwm_duration_1024ms = 5, /*!< An enum constant representing the LED gesture PWM state duration 1024ms option */
	led_pwm_duration_2048ms = 6, /*!< An enum constant representing the LED gesture PWM state duration 2048ms option */
	led_pwm_duration_3072ms = 7, /*!< An enum constant representing the LED gesture PWM state duration 3072ms option */
	led_pwm_duration_4096ms = 8, /*!< An enum constant representing the LED gesture PWM state duration 4096ms option */
	led_pwm_duration_16320ms = 9 /*!< An enum constant representing the LED gesture PWM state duration 16320ms option */
};

/**
 * \brief LED colors as a 4 byte value.
 */
union led_color_t{
	uint8_t intensities[4]; //!< Intensities of RGBW sources.
	uint32_t color; //!< Color
	struct led_intensities{
		uint32_t red : 8; //!< The red light intensity
		uint32_t green : 8; //!< The green light intensity
		uint32_t blue : 8; //!< The blue light intensity
		uint32_t white : 8; //!< The (background) white light intensity.
	}leds;
};
/** \brief	A LED PWM. */
struct led_pwm_t{
	uint32_t id;	/*!< The LED PWM identifier */
	enum led_pwm_duration_type state_durations[LED_PWM_STATE_COUNT];	/*!< The LED PWM states */
	union led_color_t color; /*!< The LED color. */
};

/** \brief	Values that represent tag LED events. */
enum led_event_type{
	led_event_TurnOff = 0,  /*!< An enum constant representing the LED event turn off option */
	led_event_Request= 1   /*!< An enum constant representing the LED event Request option */
};

/** \brief	A led. */
struct led_t{
	int32_t id; /*!< The LED identifier */
	union led_color_t color; /*!< The LED color. */
	struct led_pwm_t* pwm; /*!< The pwm of the LED. NULL if it has no LED.*/

	uint32_t start_time;  /*!< The start time of the active state in seconds.*/
	uint32_t duration;  /*!< The state duration in seconds. __INFINITE for permanent states. */
	enum led_state_type state; /*!< The state of the LED. */
};

struct led_controller_t{
	struct i2c_dt_spec* device;	  //!< The I2C driver of the LED controller.
	/*-------------------------------------------------------------------------*/
	union led_controller_status_t status; //!< the controller state
	struct lp5562_t led_driver;
};

extern struct led_controller_t* pLedController;

bool led_controller_init(void);
/**
 * \brief Configures the LED controllers of the system.
 * @return
 */
bool led_controller_configure(void);

/**
 * \brief Turns on the specified LED
 * @param led The LED identifier. It must be less than PINGBIT_LED_COUNT
 * @param color The color of the LED when turn on.
 * @return TRUE if the LED is successfully turned on, FALSE otherwise.
 */
bool led_controller_turn_on_leds(uint8_t led, union led_color_t color);
/**
 * \brief Turns off the specified LED
 * @param led The LED identifier. It must be less than PINGBIT_LED_COUNT
 * @return TRUE if the LED is successfully turned off, FALSE otherwise.
 */
bool led_controller_turn_off_leds(uint8_t led);
/**
 * \brief Sets a PWM operation for the specified LED
 * @param led The LED identifier. It must be less than PINGBIT_LED_COUNT
 * @param pwm A pointer to the PWM object.
 * @return TRUE if the LED is successfully turned off, FALSE otherwise.
 */
bool led_controller_set_led_pwm(uint8_t led, struct led_pwm_t* pwm);

/**
 * \brief turns off all the leds.
 */
void led_controller_shutdown(void);

#endif // _MATEONE_LED_CONTROLLER_H_
