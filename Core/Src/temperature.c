#include "temperature.h"
#include "stm32f4xx_hal.h"

// for alarming
#define TEMPERATURE_LIMIT_HIGH 85
#define TEMPERATURE_LIMIT_LOW -30

extern UART_HandleTypeDef huart2;
extern uint8_t TEMPERATURE_SENSORS_ID[][8];

DS18B20_Status TemperatureInitSensors(DS18B20 *sensors, uint8_t count)
{
  DS18B20_Status status;

  for (int i = 0; i < count; i++)
  {
    DS18B20_Init(&sensors[i], &huart2);
  }

  status = DS18B20_InitializationCommand(&sensors[0]);

  if (status == DS18B20_OK)
  {
    for (int i = 0; i < count; i++)
    {
      sensors[i].isConnected = 1;
			sensors[i].temperatureLimitHigh = TEMPERATURE_LIMIT_HIGH;
			sensors[i].temperatureLimitLow = TEMPERATURE_LIMIT_LOW;
			sensors[i].configRegister = DS18B20_12_BITS_CONFIG;
			DS18B20_SetRom(&sensors[i], TEMPERATURE_SENSORS_ID[i]);
    }
  }
	else
		return status;
	
	int8_t settings[3];
  settings[0] = TEMPERATURE_LIMIT_HIGH; // temperatureLimitHigh
  settings[1] = TEMPERATURE_LIMIT_LOW; // temperatureLimitLow
  settings[2] = DS18B20_12_BITS_CONFIG;

  DS18B20_InitializationCommand(&sensors[0]);
  DS18B20_SkipRom(&sensors[0]);
  DS18B20_WriteScratchpad(&sensors[0], settings);
	
  return status;
}

DS18B20_Status UpdateTemperatures(DS18B20 *sensors, uint8_t count)
{
  DS18B20_Status status;
	
	DS18B20_InitializationCommand(&sensors[0]);
	DS18B20_SkipRom(&sensors[0]);
	DS18B20_ConvertT(&sensors[0], DS18B20_DATA);
	
	for (int i = 0; i < count; i++)
	{
		DS18B20_InitializationCommand(&sensors[i]);
		DS18B20_MatchRom(&sensors[i]);
		DS18B20_ReadScratchpad(&sensors[i]);
	}
	
  return status;
}
