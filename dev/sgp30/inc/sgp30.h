#ifndef SGP30_H_
#define SGP30_H_

#define SGP30_I2C       I2C1
#define SGP30_PIN_PORT  GPIOC
#define SGP30_PIN_SDA   GPIO_Pin_0
#define SGP30_PIN_SCL   GPIO_Pin_1

#define I2C_TIMEOUT     (uint32_t)0x1FFF
#define I2C_SPEED       100000  ///< 100 KHz

//#define SGP30_DELAY_USEC    10
#define SGP30_READ_DATA_RETRY_TIMES 4

int sgp30_on(void);
void sgp30_init(void);
int sgp30_off(void);

/**
 * @brief 读取传感器数据
 *
 * @param[out]  pValue  t_sgp30_value 类型指针
 *
 * @retval  SENSORS_OK
 * @retval  SENSORS_ERROR
 */
int sgp30_co2_tvoc(void *pt_value);

#endif /* SGP30_H_ */
