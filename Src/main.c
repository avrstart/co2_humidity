#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"

#include "ili9163lcd.h"
#include "dht11.h"
#include "delay.h"

SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);

uint8_t dht11_readPin(void) {
	return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
}

void dht11_setPinIn(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void dht11_setPinOut(void) {
    
    GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void dht11_pinWrite(uint8_t state) {
	if (state == 0) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	}
}




void write_cs(uint8_t state) {
	if (state == 0) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	}
}

void write_datacom(uint8_t state) {
	if (state == 0) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	}
}

void write_reset(uint8_t state) {
	if (state == 0) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	}
}

void write_spi(uint8_t *data) {
	HAL_SPI_Transmit(&hspi2, data, 1, 100);
}

struct ili9163_operations ili9163_ops = { .ili9163_delay_ms = HAL_Delay,
		.ili9163_spi_send = write_spi, .ili9163_write_cs = write_cs,
		.ili9163_write_reset = write_reset, .ili9163_write_datacom =
				write_datacom };

                
struct dht11_operations dht11_ops = { 
    .readPin = dht11_readPin, 
    .setPinOut = dht11_setPinOut, 
    .setPinIn = dht11_setPinIn,
	.pinWrite = dht11_pinWrite, 
    .delay_ms = delay_ms, 
    .delay_us = delay_us 
};

uint8_t receiveBuffer[32];
uint16_t get_counter = 0;
uint16_t set_counter = 0;

#define FIFO_SIZE  1024
uint8_t fifo_buf[FIFO_SIZE];

void poll_gas_sensor(void);
void poll_humidity(void);

int main(void) {
	
	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_SPI2_Init();
	MX_USART2_UART_Init();

	ili9163_init(&ili9163_ops, LCD_ORIENTATION0);
	ili9163_clear(decodeRgbValue(0, 0, 0));

	while (1) {

        poll_gas_sensor();
        poll_humidity();
	}

}

void poll_humidity(void) 
{
    static uint8_t dht_mode = 0;
    static uint32_t start_time;
    dht11_data_t dht11_data;
    uint8_t tbuf[32];
    
    switch(dht_mode) {
    case 0:
        start_time = HAL_GetTick();
        dht_mode = 1;
    
        DHT11_getData(&dht11_ops, &dht11_data);
    
        sprintf((char *) tbuf, "Humidity %d  ", dht11_data.humidity);
        ili9163_puts((char *) tbuf, lcdTextX(3), lcdTextY(5),
						decodeRgbValue(31, 0, 0), decodeRgbValue(0, 0, 0));
        sprintf((char *) tbuf, "Temper %d  ", dht11_data.temper);
        ili9163_puts((char *) tbuf, lcdTextX(3), lcdTextY(1),
						decodeRgbValue(31, 0, 0), decodeRgbValue(0, 0, 0));
        
        break;
       
    case 1:
        if(get_timeout(start_time) > 10000) {
            dht_mode = 0;
        }
        break;
    default:break;
    }   
}

uint8_t co2_txdata[9] = { 0xff, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79 };
void poll_gas_sensor(void)
{
    static uint8_t co2_txmode = 0;
    static uint32_t start_time;
    static uint8_t co2_rxmode = 0;
    static uint16_t co2_ppm; 
    uint8_t tbuf[32];
    
    switch(co2_txmode) {
    case 0:
        start_time = HAL_GetTick();
        co2_txmode = 1;
        HAL_UART_Transmit(&huart2, co2_txdata, sizeof(co2_txdata), 1000);
        HAL_UART_Receive_IT(&huart2, receiveBuffer, 1); 
        break;
       
    case 1:
        if(get_timeout(start_time) > 10000) {
            co2_txmode = 0;
        }
        break;
    default:break;
    }
    
    if(get_counter != set_counter)  {
    uint8_t uart_byte = fifo_buf[set_counter++];
        
    switch(co2_rxmode) {
        case 0:
            if(uart_byte == 0xff) {
                co2_rxmode = 1;
            }
            break;
        case 1:
            if(uart_byte == 0x86) {
                co2_rxmode = 2;
            }
            else 
                co2_rxmode = 0;
            break;
        case 2:
            co2_ppm = uart_byte *256;
            co2_rxmode = 3;
            break;
        case 3:
            co2_ppm += uart_byte;
            co2_rxmode = 0;
            sprintf((char *) tbuf, "PPM = %d  ", co2_ppm);
            ili9163_puts((char *) tbuf, lcdTextX(3), lcdTextY(9),
						decodeRgbValue(31, 0, 0), decodeRgbValue(0, 0, 0));
            break;
        default:break;
    }
    if(set_counter == FIFO_SIZE) set_counter = 0;
    }
        
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    fifo_buf[get_counter++] = receiveBuffer[0];
    if (get_counter == FIFO_SIZE) get_counter = 0; 
    HAL_UART_Receive_IT(&huart2, receiveBuffer, 1);    
}

void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

static void MX_USART2_UART_Init(void) {

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

static void MX_SPI2_Init(void) {
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, RESET_Pin | A0_Pin | CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : RESET_Pin A0_Pin CS_Pin */
	GPIO_InitStruct.Pin = RESET_Pin | A0_Pin | CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
}

void _Error_Handler(char * file, int line) {
	while (1) {
	}
}

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{
}

#endif
