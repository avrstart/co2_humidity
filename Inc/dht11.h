/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DHT11_H
#define __DHT11_H

/* Includes ------------------------------------------------------------------*/
//#include "utils.h"
//#include "pin.h"
//#include "tim.h"

#include <stdint.h>

/* Exported constants --------------------------------------------------------*/
#define MAX_TICS 10000
#define DHT11_OK 0
#define DHT11_NO_CONN 1
#define DHT11_CS_ERROR 2

struct dht11_operations {
	uint8_t (*readPin)(void);
	void (*setPinOut)(void);
	void (*setPinIn)(void);
	void (*pinWrite)(uint8_t state);
	void (*delay_ms)(uint32_t ms);
	void (*delay_us)(uint32_t us);
};

typedef struct  
{
	uint8_t temper;
    uint8_t humidity;
}dht11_data_t;

//#define DHT11_PORT GPIOC
//#define DHT11_PIN GPIO_Pin_2

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
uint8_t DHT11_getData(struct dht11_operations *dht11_tops, dht11_data_t *dht11_data);
float DHT22_Humidity(uint8_t *buf);
float DHT22_Temperature(uint8_t *buf);
uint8_t DHT11_Humidity(uint8_t *buf);
uint8_t DHT11_Temperature(uint8_t *buf);
uint8_t DHT11_pwm_Read(uint8_t *buf, uint32_t *dt, uint32_t *cnt);

#endif /* __DHT11_H */
