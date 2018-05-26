#include "si7021.h"
#include "stm8l15x.h"
#include "i2c.h"
#include "sensors.h"

#define DEBUG 0

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

                /* adr   command  r/w */
#define Si7021_ADDR     0x80
#define MEASURE_TEMP    0xE3
#define MEASURE_HUMI    0xE5

//#define CRC_CHECK
#ifdef CRC_CHECK
/* BEWARE: Bit reversed CRC8 using polynomial ^8 + ^5 + ^4 + 1 */
static uint8_t crc8_add(uint8_t acc, uint8_t byte)
{
    uint8_t i;
    acc ^= byte;
    for(i = 0; i < 8; i++)
    {
        if(acc & 0x80)
        {
            acc = (acc << 1) ^ 0x31;
        }
        else
        {
            acc <<= 1;
        }
    }
    return acc & 0xff;
}
#endif /* CRC_CHECK */

/*
 * Power up the device and initialize the iic interface.
 */
void si7021_init(void)
{
    GPIO_Init(SI7021_VCC_PORT, SI7021_VCC_PIN, GPIO_Mode_Out_PP_High_Fast);
    GPIO_SetBits(SI7021_VCC_PORT, SI7021_VCC_PIN);
    I2C1_Init();
}

/*
 * Power up device.
 */
static void si7021_on(void)
{
    GPIO_SetBits(SI7021_VCC_PORT, SI7021_VCC_PIN);
}

/*
 * Power off device.
 */
void si7021_off(void)
{
    GPIO_ResetBits(SI7021_VCC_PORT, SI7021_VCC_PIN);
}

/*
 * Only commands MEASURE_HUMI or MEASURE_TEMP!
 */
static int8_t scmd(uint8_t cmd, uint32_t *pOut)
{
    uint8_t t0 = 0;
    uint8_t t1 = 0;
    uint8_t rcrc = 0;
    uint32_t TimeOut=0; 

    if(cmd != MEASURE_HUMI && cmd != MEASURE_TEMP) {
        PRINTF("Illegal command: %d\n", cmd);
        return SENSORS_ERROR;
    }

    /*!< While the bus is busy */
    while (I2C_GetFlagStatus(SI7021_I2C, I2C_FLAG_BUSY))
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }

    I2C_AcknowledgeConfig(SI7021_I2C, ENABLE);

    /*!< Send STRAT condition */
    I2C_GenerateSTART(SI7021_I2C, ENABLE);

    /*!< Test on EV5 and clear it */
    while (!I2C_CheckEvent(SI7021_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }

    /*!< Send SI7021 address for write */
    I2C_Send7bitAddress(SI7021_I2C, Si7021_ADDR, I2C_Direction_Transmitter);

    /*!< Test on EV6 and clear it */
    while (!I2C_CheckEvent(SI7021_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }

    /* !< Send measure cmd */
    I2C_SendData(SI7021_I2C, cmd);

    /*!< Test on EV8 and clear it */
    while (!I2C_CheckEvent(SI7021_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }

    /*!< Send STRAT condition */
    I2C_GenerateSTART(SI7021_I2C, ENABLE);

    /*!< Test on EV5 and clear it */
    while(!I2C_CheckEvent(SI7021_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }

    /*!< Send SI7021 address for read */
    I2C_Send7bitAddress(SI7021_I2C, Si7021_ADDR, I2C_Direction_Receiver);


    /*!< Test on EV6 and clear it */
    while (!I2C_CheckEvent(SI7021_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }

    /*!< Test on EV7 and clear it */
    while (!I2C_CheckEvent(SI7021_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }

    t0 = I2C_ReceiveData(SI7021_I2C);

    /* Test on RXNE flag */
    while (I2C_GetFlagStatus(SI7021_I2C, I2C_FLAG_RXNE) == RESET)
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }

    t1 = I2C_ReceiveData(SI7021_I2C);

    I2C_AcknowledgeConfig(SI7021_I2C, DISABLE);

    /* Test on RXNE flag */
    while (I2C_GetFlagStatus(SI7021_I2C, I2C_FLAG_RXNE) == RESET)
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }

    rcrc = I2C_ReceiveData(SI7021_I2C);

    I2C_GenerateSTOP(SI7021_I2C, ENABLE);

    PRINTF("Si7021: scmd - read 0x%02x, 0x%02x, 0x%02x\n", (uint16_t)t0, (uint16_t)t1,(uint16_t)rcrc);

    *pOut = (t0 << 8) | t1;

#ifdef CRC_CHECK
    uint8_t crc;
    crc = crc8_add(0x0, t0);
    crc = crc8_add(crc, t1);
    if(crc != rcrc)
    {
        PRINTF("Si7021: scmd - crc check failed 0x02%x vs 0x%02x\n",
                (uint16_t)crc, (uint16_t)rcrc);
        return CRC_ERROR;
    }
#endif

    return SENSORS_OK;

    fail:
        I2C_DeInit(SI7021_I2C);
        I2C_SoftwareResetCmd(SI7021_I2C, ENABLE); 
        I2C1_Init();
        return SENSORS_ERROR;
}

int si7021_temp(float *pTemp)
{
    uint32_t temp_code;
    int8_t ret;

    ret = scmd(MEASURE_TEMP, &temp_code);

    if(SENSORS_ERROR != ret)
    {
        *pTemp = (int32_t)temp_code * 175.72 / 65536 - 46.85;
        return SENSORS_OK;
    }

    return SENSORS_ERROR;
}

int si7021_humidity(float *pRh)
{
    uint32_t rh_code;
    int8_t ret;

    ret = scmd(MEASURE_HUMI, &rh_code);

    if(SENSORS_ERROR != ret)
    {
        *pRh = (float)rh_code * 125 / 65536 - 6;
        return SENSORS_OK;
    }

    return SENSORS_ERROR;
}
