#include "sgp30.h"
#include "stm8l15x.h"
#include "sensors.h"
#include "sgp30_sensor.h"
#include "delay.h"

#define SGP30_DEBUG
#ifdef SGP30_DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define SGP30_ADDR   0x58
#define GENERAL_CALL_ADDRESS 0
#define SGP30_CMD_IAQ_INIT 0x2003
#define SGP30_CMD_SOFT_RESET 0x06
#define SGP30_MEASURE_IAQ 0x2008

#define CRC_CHECK
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

void sgp30_init(void)
{
    CLK_PeripheralClockConfig(CLK_Peripheral_I2C1,ENABLE);
    GPIO_Init(SGP30_PIN_PORT, SGP30_PIN_SDA, GPIO_Mode_Out_OD_HiZ_Slow);
    GPIO_Init(SGP30_PIN_PORT, SGP30_PIN_SCL, GPIO_Mode_Out_OD_HiZ_Slow);
    I2C_Init(SGP30_I2C, I2C_SPEED, 0X00, I2C_Mode_I2C, I2C_DutyCycle_2, I2C_Ack_Enable, I2C_AcknowledgedAddress_7bit);
    I2C_Cmd(SGP30_I2C, ENABLE);
}

/*
 * Power up or wake up device.
 */
int sgp30_on(void)
{
    uint32_t TimeOut = 0;
    uint8_t cmdMsb = (uint8_t)(SGP30_CMD_IAQ_INIT >> 8);
    uint8_t cmdLsb = (uint8_t)SGP30_CMD_IAQ_INIT;

    /*!< While the bus is busy */
    while (I2C_GetFlagStatus(SGP30_I2C, I2C_FLAG_BUSY))
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }

    I2C_AcknowledgeConfig(SGP30_I2C, ENABLE);

    /*!< Send STRAT condition */
    I2C_GenerateSTART(SGP30_I2C, ENABLE);

    /*!< Test on EV5 and clear it */
    while (!I2C_CheckEvent(SGP30_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }

    /*!< Send SGP30 address for write */
    I2C_Send7bitAddress(SGP30_I2C, SGP30_ADDR<<1, I2C_Direction_Transmitter);

    /*!< Test on EV6 and clear it */
    while (!I2C_CheckEvent(SGP30_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }
    /* !< Send iaq init cmd MSB */
    I2C_SendData(SGP30_I2C, cmdMsb);

    /*!< Test on EV8 and clear it */
    while (!I2C_CheckEvent(SGP30_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }

    /* !< Send iaq init cmd LSB */
    I2C_SendData(SGP30_I2C, cmdLsb);

    /*!< Test on EV8 and clear it */
    while (!I2C_CheckEvent(SGP30_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }

    I2C_GenerateSTOP(SGP30_I2C, ENABLE);

    /* delay ? */

    return SENSORS_OK;

    fail:
        I2C_AcknowledgeConfig(SGP30_I2C, DISABLE);
        I2C_GenerateSTOP(SGP30_I2C, ENABLE);
        return SENSORS_ERROR;
}

/*
 * @brief Power off device.
 * @details
 * 使用 I2C 协议的 General Call Address
 */
int sgp30_off(void)
{
    uint32_t TimeOut = 0;
    uint8_t cmd = (uint8_t)SGP30_CMD_SOFT_RESET;

    /*!< While the bus is busy */
    while (I2C_GetFlagStatus(SGP30_I2C, I2C_FLAG_BUSY))
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }

    I2C_AcknowledgeConfig(SGP30_I2C, ENABLE);

    /*!< Send STRAT condition */
    I2C_GenerateSTART(SGP30_I2C, ENABLE);

    /*!< Test on EV5 and clear it */
    while (!I2C_CheckEvent(SGP30_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }

    /*!< Send SGP30 address for write */
    I2C_Send7bitAddress(SGP30_I2C, GENERAL_CALL_ADDRESS, I2C_Direction_Transmitter);

    /*!< Test on EV6 and clear it */
    while (!I2C_CheckEvent(SGP30_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }
    /* !< Send soft reset cmd */
    I2C_SendData(SGP30_I2C, cmd);

    /*!< Test on EV8 and clear it */
    while (!I2C_CheckEvent(SGP30_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }

    I2C_GenerateSTOP(SGP30_I2C, ENABLE);

    /* delay ? */

    return SENSORS_OK;

    fail:
        I2C_AcknowledgeConfig(SGP30_I2C, DISABLE);
        I2C_GenerateSTOP(SGP30_I2C, ENABLE);
        return SENSORS_ERROR;
}

/*
 * Only commands SGP30_MEASURE_IAQ!
 */
static int8_t scmd(uint16_t cmd, void *pt_value)
{
    uint8_t co2Msb, co2Lsb, tvocMsb, tvocLsb;
    t_sgp30_value *pt_sgp30_value = (t_sgp30_value *)pt_value;
    uint32_t TimeOut = 0;
    uint8_t cmdMsb = (uint8_t)(cmd >> 8);
    uint8_t cmdLsb = (uint8_t)cmd;
    uint8_t co2Crc;
    uint8_t tvocCrc;
    uint8_t read_data_retry_times = SGP30_READ_DATA_RETRY_TIMES;

    if((void *)0 == pt_value)
    {
        return SENSORS_ERROR;
    }

    if(cmd != SGP30_MEASURE_IAQ)
    {
        PRINTF("Illegal command: %d\n", cmd);
        return SENSORS_ERROR;
    }

    I2C_ClearFlag(SGP30_I2C, I2C_FLAG_BUSY);
    /*!< While the bus is busy */
    while (I2C_GetFlagStatus(SGP30_I2C, I2C_FLAG_BUSY))
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }

    I2C_AcknowledgeConfig(SGP30_I2C, ENABLE);

    /*!< Send STRAT condition */
    I2C_GenerateSTART(SGP30_I2C, ENABLE);

    /*!< Test on EV5 and clear it */
    while (!I2C_CheckEvent(SGP30_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }

    /*!< Send SGP30 address for write */
    I2C_Send7bitAddress(SGP30_I2C, SGP30_ADDR<<1, I2C_Direction_Transmitter);

    /*!< Test on EV6 and clear it */
    while (!I2C_CheckEvent(SGP30_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }
    /* !< Send measure cmd MSB */
    I2C_SendData(SGP30_I2C, cmdMsb);

    /*!< Test on EV8 and clear it */
    while (!I2C_CheckEvent(SGP30_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }

    /* !< Send measure cmd LSB */
    I2C_SendData(SGP30_I2C, cmdLsb);

    /*!< Test on EV8 and clear it */
    while (!I2C_CheckEvent(SGP30_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }

    /* 1. 需要等待传感器测量完成，否则读取会失败（数据手册 6.2：aborts the communication 
       with a XCK condition.）。发送完 "sgp30_iaq_int" 后的 15s 内读取没问题，但是 15s 
       之后会出现传感器不回应 Sr 之后的读命令的情况（可能就是手册 6.2 提到的）。
       2. 实测 15s 后，不管延时不延时，第一次读取总是失败，需要再次读取。
    */

retry:
    /*!< Send RESTRAT condition */
    I2C_GenerateSTART(SGP30_I2C, ENABLE);

    /*!< Test on EV5 and clear it */
    TimeOut = 0;
    while(!I2C_CheckEvent(SGP30_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }

    /*!< Send SGP30 address for read */
    I2C_Send7bitAddress(SGP30_I2C, SGP30_ADDR<<1, I2C_Direction_Receiver);

    /*!< Test on EV6 and clear it */
    TimeOut = 0;
    while (!I2C_CheckEvent(SGP30_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            read_data_retry_times--;
            if(0 != read_data_retry_times)
            {
              goto retry;
            }
            goto fail;
        }
    }

    /*!< Test on EV7 and clear it */
    while (!I2C_CheckEvent(SGP30_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }

    co2Msb = I2C_ReceiveData(SGP30_I2C);

    /* Test on RXNE flag */
    while (I2C_GetFlagStatus(SGP30_I2C, I2C_FLAG_RXNE) == RESET)
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }

    co2Lsb = I2C_ReceiveData(SGP30_I2C);

    /* Test on RXNE flag */
    while (I2C_GetFlagStatus(SGP30_I2C, I2C_FLAG_RXNE) == RESET)
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }

    co2Crc = I2C_ReceiveData(SGP30_I2C);

    /* Test on RXNE flag */
    while (I2C_GetFlagStatus(SGP30_I2C, I2C_FLAG_RXNE) == RESET)
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }

    tvocMsb = I2C_ReceiveData(SGP30_I2C);

    /* Test on RXNE flag */
    while (I2C_GetFlagStatus(SGP30_I2C, I2C_FLAG_RXNE) == RESET)
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }

    tvocLsb = I2C_ReceiveData(SGP30_I2C);

    I2C_GenerateSTOP(SGP30_I2C, ENABLE);

    /* Test on RXNE flag */
    while (I2C_GetFlagStatus(SGP30_I2C, I2C_FLAG_RXNE) == RESET)
    {
        if(++TimeOut>I2C_TIMEOUT)
        {
            goto fail;
        }
    }

    tvocCrc = I2C_ReceiveData(SGP30_I2C);
    I2C_AcknowledgeConfig(SGP30_I2C, DISABLE);

    PRINTF("SGP30: scmd - read 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x\n",
            (uint16_t)co2Msb, (uint16_t)co2Lsb, (uint16_t)co2Crc,
            (uint16_t)tvocMsb, (uint16_t)tvocLsb, (uint16_t)tvocCrc);

    pt_sgp30_value->co2 = (co2Msb << 8) | co2Lsb;
    pt_sgp30_value->tvoc = (tvocMsb << 8) | tvocLsb;

#ifdef CRC_CHECK
    uint8_t crc;
    crc = crc8_add(0xff, co2Msb);
    crc = crc8_add(crc, co2Lsb);
    if(crc != co2Crc)
    {
        PRINTF("Si7021: scmd - tCrc check failed 0x%02x vs 0x%02x\n",
                (uint16_t)crc, (uint16_t)co2Crc);
        return CRC_ERROR;
    }

    crc = crc8_add(0xff, tvocMsb);
    crc = crc8_add(crc, tvocLsb);
    if(crc != tvocCrc)
    {
        PRINTF("Si7021: scmd - rhCrc check failed 0x%02x vs 0x%02x\n",
                (uint16_t)crc, (uint16_t)tvocCrc);
        return CRC_ERROR;
    }
#endif

    return SENSORS_OK;

    fail:
        I2C_AcknowledgeConfig(SGP30_I2C, DISABLE);
        I2C_GenerateSTOP(SGP30_I2C, ENABLE);
        return SENSORS_ERROR;
}

int sgp30_co2_tvoc(void *pt_value)
{
    uint16_t sCo2, sTvoc;
    int8_t ret;

    ret = scmd(SGP30_MEASURE_IAQ, pt_value);
    
    if(SENSORS_ERROR != ret)
    {
        return SENSORS_OK;
    }

    return SENSORS_ERROR;
}
