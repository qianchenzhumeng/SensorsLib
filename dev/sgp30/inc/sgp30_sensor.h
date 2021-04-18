#ifndef SGP30_SENSOR_H_
#define SGP30_SENSOR_H_

#include "sensors.h"

extern const struct sensors_sensor sgp30_sensor;

typedef struct {
    unsigned short co2;
    unsigned short tvoc;
}t_sgp30_value;

#define SGP30_SENSOR_CO2_TVOC   0

#endif /* SGP30_SENSOR_H_ */