#ifndef      MTCHh6102_H
#define      MTCHh6102_H
#include <stdbool.h>
#include <zephyr/kernel.h>

#include 	"mtch6102_registers.h"

struct touch_sensor_sample {
	struct mtch6102_position position;
	uint8_t gesture_state;
};

typedef void (*touch_sensor_sample_cb_t)(const struct touch_sensor_sample *sample,
					 void *user_data);

int mtch6102_get_position(const struct i2c_dt_spec *i2c, struct mtch6102_position *pos);

/*  
   @Brief         Set Core Register
   @Description   Set Core Register Settings
   @Parameter     struct i2c_dt_spec*                  ->  HAL_I2C Handle
                  MTCH6102_Core_Ram_Memory           ->  MTCH6102_Core
                  uint8_t                            ->  value
   @Return value  None
 */
void MTCH6102_set_Core(const struct i2c_dt_spec *i2c, MTCH6102_Core_Ram_Memory MTCH6102_Core, uint8_t value);

/*  
   @Brief         Set Touch Register
   @Description   Set Touch Register Settings
   @Parameter     struct i2c_dt_spec*                  ->  HAL_I2C Handle
                  MTCH6102_Touch_Ram_Memory          ->  MTCH6102_Touch
                  uint8_t                            ->  value
   @Return value  None
 */
void MTCH6102_set_Touch(const struct i2c_dt_spec *i2c, MTCH6102_Touch_Ram_Memory MTCH6102_Touch, uint8_t value);

/*  
   @Brief         Get Touch Register
   @Description   Get Touch Register Settings
   @Parameter     struct i2c_dt_spec*                  ->  HAL_I2C Handle
                  MTCH6102_Touch_Ram_Memory          ->  MTCH6102_Touch
   @Return value  uint8_t
 */
uint8_t MTCH6102_get_Touch(const struct i2c_dt_spec *i2c, MTCH6102_Touch_Ram_Memory MTCH6102_Touch);

/*  
   @Brief         Get Compensation Value
   @Description   Get Compensation Register Value
   @Parameter     struct i2c_dt_spec*                  ->  HAL_I2C Handle
                  MTCH6102_Compensation_Ram_Memory   ->  MTCH6102_Compensation
   @Return value  uint8_t
 */
uint8_t MTCH6102_get_Compensation(const struct i2c_dt_spec *i2c, MTCH6102_Compensation_Ram_Memory MTCH6102_Compensation);

/*  
   @Brief         Get Acquisition Value
   @Description   Get Acquisition Register Value
   @Parameter     struct i2c_dt_spec*                  ->  HAL_I2C Handle
                  MTCH6102_Acquisition_Ram_Memory    ->  MTCH6102_Acquisition
   @Return value  uint8_t
 */
uint8_t MTCH6102_get_Acquisition(const struct i2c_dt_spec *i2c, MTCH6102_Acquisition_Ram_Memory MTCH6102_Acquisition);

/*  
   @Brief         Set Configuration Registers
   @Description   Get Configuration Register Value
   @Parameter     struct i2c_dt_spec                  ->  HAL_I2C Handle
                  MTCH6102_Configuration_Ram_Memory  ->  MTCH6102_Configuration
                  uint8_t                            ->  value
   @Return value  None
 */
void MTCH6102_set_Configuration(const struct i2c_dt_spec *i2c, MTCH6102_Configuration_Ram_Memory MTCH6102_Configuration, uint8_t value);

/*  
   @Brief         Set Default Settings
   @Description   Set Default Settings
   @Parameter     struct i2c_dt_spec*                  ->  HAL_I2C Handle
   @Return value  None
 */
void MTCH6102_InitializeDEFAULT(const struct i2c_dt_spec *i2c);

/*  
   @Brief         Set Custom Settings
   @Description   Set Custom Settings
   @Parameter     struct i2c_dt_spec*                  ->  HAL_I2C Handle
   @Return value  None
 */
void MTCH6102_Initialize(const struct i2c_dt_spec *i2c);

int touch_sensor_init(void);
int touch_sensor_start(void);
void touch_sensor_stop(void);
void touch_sensor_deinit(void);
void touch_sensor_set_streaming_enabled(bool enabled);
void touch_sensor_set_sampling_rate(uint16_t new_sampling_rate);
void touch_sensor_set_transfer_interval(uint16_t new_transfer_interval);
int touch_sensor_register_callback(touch_sensor_sample_cb_t cb, void *user_data);

#endif
