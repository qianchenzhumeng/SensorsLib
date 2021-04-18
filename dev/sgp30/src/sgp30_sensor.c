#include "sensors.h"
#include "sgp30.h"
#include "sgp30_sensor.h"

const struct sensors_sensor sgp30_sensor;

enum {
    ON, OFF
};

static unsigned char state = OFF;

/**
 * @brief 读取传感器数据
 *
 * @param[out]  pValue  sgp30_value 类型指针
 *
 * @retval  SENSORS_OK
 * @retval  SENSORS_ERROR
 */
static int value(int type, void *p_value)
{
    int ret = SENSORS_ERROR;

    switch(type)
    {
        case SGP30_SENSOR_CO2_TVOC:
            ret = sgp30_co2_tvoc(p_value);
            break;
        default:
            break;
    }

    return ret;
}

static int status(int type)
{
    switch(type) {
        case SENSORS_ACTIVE:
        case SENSORS_READY:
            return (state == ON);
    }

    return 0;
}

static int configure(int type, int c)
{
    int ret = 0;

    switch(type) {
        case SENSORS_HW_INIT:
            sgp30_init();
            break;
        case SENSORS_ACTIVE:
            if(c)
            {
                if(!status(SENSORS_ACTIVE))
                {
                    ret = sgp30_on();
                    state = ON;
                }
            }
            else
            {
                ret = sgp30_off();
                state = OFF;
            }
            break;
        default:
            break;
    }
    return ret;
}

SENSORS_SENSOR(sgp30_sensor, "sgp30",
           value, configure, status);
