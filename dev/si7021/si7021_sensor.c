#include "sensors.h"
#include "si7021.h"
#include "si7021_sensor.h"

const struct sensors_sensor si7021_sensor;

enum {
    ON, OFF
};

static unsigned char state = OFF;

static int value(int type, float *pValue)
{
    int ret = SENSORS_ERROR;

    switch(type)
    {
        case SI7021_SENSOR_TEMP:
            ret = si7021_temp(pValue);
            break;
        case SI7021_SENSOR_HUMIDITY:
            ret = si7021_humidity(pValue);
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
    switch(type) {
        case SENSORS_ACTIVE:
            if(c)
            {
                if(!status(SENSORS_ACTIVE))
                {
                    si7021_init();
                    state = ON;
                }
            }
            else
            {
                si7021_off();
                state = OFF;
            }
    }
    return 0;
}

SENSORS_SENSOR(si7021_sensor, "si7021",
           value, configure, status);
