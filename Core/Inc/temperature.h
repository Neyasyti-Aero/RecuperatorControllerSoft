#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include "ds18b20.h"

extern DS18B20_Status TemperatureInitSensors(DS18B20 *sensors, uint8_t count);
extern DS18B20_Status UpdateTemperatures(DS18B20 *sensors, uint8_t count);

#endif // #ifndef TEMPERATURE_H
