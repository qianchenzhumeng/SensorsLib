#ifndef SENSORS_H_
#define SENSORS_H_

/* some constants for the configure API */
#define SENSORS_HW_INIT 128 /* internal - used only for initialization */
#define SENSORS_ACTIVE 129 /* ACTIVE => 0 -> turn off, 1 -> turn on */
#define SENSORS_READY 130 /* read only */

#define SENSORS_OK       ((int)0)
#define SENSORS_ERROR    ((int)-1)
#define CRC_ERROR       ((int)-2)

#define SENSORS_INIT(sensor) (sensor).configure(SENSORS_HW_INIT, 0)
#define SENSORS_ACTIVATE(sensor) (sensor).configure(SENSORS_ACTIVE, 1)
#define SENSORS_DEACTIVATE(sensor) (sensor).configure(SENSORS_ACTIVE, 0)

#define SENSORS_SENSOR(name, type, value, configure, status)        \
const struct sensors_sensor name = { type, value, configure, status }

struct sensors_sensor {
  char *        type;
  int           (* value)     (int type, void *p_value);
  int           (* configure) (int type, int value);
  int           (* status)    (int type);
};


#endif /* SENSORS_H_ */
