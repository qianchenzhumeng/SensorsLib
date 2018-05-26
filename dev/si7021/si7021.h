#ifndef SI7021_H_
#define SI7021_H_

#define SI7021_I2C        I2C1
#define SI7021_VCC_PORT   GPIOC
#define SI7021_VCC_PIN    GPIO_Pin_4

void si7021_init(void);
void si7021_off(void);

int si7021_temp(float *pTemp);
int si7021_humidity(float *pRh);

#endif /* SI7021_H_ */
