#include "main.h"
#include <stdio.h>
#include <string.h>

uint8_t RH1, RH2, TC1, TC2, SUM, CHECK;
uint32_t pMillis, cMillis;
float tCelsius = 0;
float tFahrenheit = 0;
float RH = 0;
int fire = 0, dark =0;
char rx_char;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void uart_init(void);
void USART1_IRQHandler(void);
void DWT_Init(void);
void delay_us(uint32_t us);


// UART send
void uartSend(uint8_t *data, uint8_t size){
    for (uint8_t i = 0; i < size; i++) {
        USART1->DR = *(data + i);
        while (!(USART1->SR & USART_SR_TXE));  
    }
}

// LED
void led_on(void){
  GPIOB->ODR |= (1<<7);
	dark = 1;
}

void led_off(void){
	GPIOB->ODR &= ~(1<<7);
	dark = 0;
}

// Buzzer
void buzzer_on(void){
	GPIOB->ODR |= (1<<4);
	fire = 1;	
}

void buzzer_off(void){
	GPIOB->ODR &= ~(1<<4);
	fire = 0;	
}

// LCD
void send8BitLCD(char D) {
	GPIOA->BSRR = ((0xFF) << 16);
	GPIOA->BSRR = (D & 0xFF); 
}

void sendCMD2LCD(char cmd) {
	GPIOC->ODR &= ~(1<<14);
  send8BitLCD(cmd);
  GPIOC->ODR &= ~(1<<15);
	GPIOC->ODR |= (1<<15);
  delay_us(1000);
}

void sendChar2LCD(char Char) {
  GPIOC->ODR |= (1<<14);
  send8BitLCD(Char);
  GPIOC->ODR &= ~(1<<15);
	GPIOC->ODR |= (1<<15);
  delay_us(1000);
}

void sendString2LCD(char *str) {
    for (int i = 0; str[i] != '\0'; i++) {
        sendChar2LCD(str[i]);
    }
}

// DHT22
uint8_t DHT22_Start(void) {
  uint8_t Response = 0;
	GPIOB->CRH &= ~(0xF << 4); 
	GPIOB->CRH |= (0x2 << 4); 
	GPIOB->BSRR = (1 << 9) << 16; // Reset pin 9
	delay_us(1300);
	GPIOB->BSRR = (1 << 9); // Set pin 9
	delay_us(30);
	//GPIOB_PIN_9 input with pull-up
	GPIOB->CRH &= ~(0xF << 4);
	GPIOB->CRH |= (0x8 << 4);  

	// GPIOB_PIN_9 pull-up
	GPIOB->ODR |= (1 << 9);
  delay_us(120);
  if (GPIOB->IDR & (1<<9)) Response = 1;
  delay_us(40);
  return Response;
}

uint8_t DHT22_Read(void) {
  uint8_t a, b;
  for (a = 0; a < 8; a++) {
      pMillis = HAL_GetTick();
      cMillis = HAL_GetTick();
      while (!(GPIOB->IDR & (1<<9)) && pMillis + 2 > cMillis) {  
          cMillis = HAL_GetTick();
      }
      delay_us(40);
      if (!(GPIOB->IDR & (1<<9)))
          b &= ~(1 << (7 - a));
      else
          b |= (1 << (7 - a));
      pMillis = HAL_GetTick();
      cMillis = HAL_GetTick();
      while ((GPIOB->IDR & (1<<9)) && pMillis + 2 > cMillis) {  
          cMillis = HAL_GetTick();
      }
  }
  return b;
}

int main(void) {
  char str[100];
  float c;
  DWT_Init();
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  uart_init();
  sendCMD2LCD(0x01);  // Clear display
  sendCMD2LCD(0x0C);  // Display on, cursor off
  while (1) {
   if (!(GPIOB->IDR & (1<<3))){
		fire = 1;	
   } else{
		fire = 0;	
		}
    if (GPIOB->IDR & (1<<6)){
		dark = 1;
    } else {
			dark = 0;
		}
   if (DHT22_Start()) {
      RH1 = DHT22_Read();
      RH2 = DHT22_Read();
      TC1 = DHT22_Read();
      TC2 = DHT22_Read();
      SUM = DHT22_Read();
      CHECK = RH1 + RH2 + TC1 + TC2;
      if (CHECK == SUM) {
				if (TC1 > 127) {
					tCelsius = (float)TC2 / 10 * (-1);
        } else { 
          tCelsius = (float)((TC1 << 8) | TC2) / 10;
        }
          RH = (float)((RH1 << 8) | RH2) / 10;
          sprintf(str, " %gC %g%%", tCelsius, RH);
          sendCMD2LCD(0x01);  // Clear display
          sendString2LCD(str);
					sprintf(str, "%.1f,%.1f,%d,%d\n\r", tCelsius, RH, fire, dark);
          uartSend((uint8_t*)str, strlen(str));
        }
    } else {
					sendCMD2LCD(0x01);  // Clear display
          sendString2LCD("DHT22 Error");
					sprintf(str, "DHT22 Error\r\n");
          uartSend((uint8_t*)str, strlen(str));
    }
      delay_us(1000000);
  }
}

void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
      Error_Handler();
  }
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
      Error_Handler();
  }

  RCC->CR |= RCC_CR_HSION;
  while (!(RCC->CR & RCC_CR_HSIRDY));
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

  SystemCoreClockUpdate();
}

void uart_init(void) {
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE |USART_CR1_RXNEIE| USART_CR1_UE;
    USART1->BRR = (uint16_t) 0x1D4C;  
    NVIC_EnableIRQ(USART1_IRQn);
}

void USART1_IRQHandler(void) {
    if (USART1->SR & USART_SR_RXNE) {
      rx_char = USART1->DR;
			switch (rx_char){
				case 'A' : {
					led_off();
					buzzer_off();
					break;
				}
				case 'B' : {
					led_on();
					buzzer_off();
					break;
				}
				case 'C' : {
					led_off();
					buzzer_on();
					break;
				}
				case 'D' : {
					led_on();
					buzzer_on();
					break;
				}
			}
			USART2->SR&=~USART_SR_RXNE;
    }
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  // Enable GPIO Clocks
	RCC->APB2ENR |= (1<<2) | (1<<3)|(1<<4)|(1<<5);

  // Reset GPIOB settings
  GPIOB->CRL &= 0;
  GPIOB->CRH &= 0;
  GPIOB->ODR &= 0;
  // Flame Sensor Init (GPIOB PIN 3: Read Data, GPIOB PIN 4: Buzzer)
  GPIOB->CRL |= (1<<15) | (1<<16);
  GPIOB->ODR |= (1<<3);

  // Light Sensor Init (GPIOB PIN 6: Read Data, GPIOB PIN 7: Led)
  GPIOB->CRL |= (1<<27) | (1<<28);
  GPIOB->ODR |= (1<<6);

  // DHT22 Pin Init (GPIOB Pin 9)
	GPIOB -> CRH |= (1<<5);	

  // LCD Pins Init (GPIOA Pins 0-7, GPIOC Pins 14-15)
  // GPIOA: Data, GPIOC Pin 14 RS, GPIO Pin 15 E
	GPIOA->ODR &= 0;
  GPIOC->ODR &= 0;
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
	GPIOA->CRL &= ~(0xFFFFFFFF);   
	GPIOA->CRL |= 0x33333333;     
	GPIOA->ODR &= ~(0xFF); 
  GPIOC->CRH &= ~((0xF<<24)|(0xF)<<28);
	GPIOC->CRH |= (0x2 << 24) | (0x2 << 28);
  // UART1
  GPIOA->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9);
  GPIOA->CRH |= (GPIO_CRH_MODE9_1 | GPIO_CRH_CNF9_1);
}

void DWT_Init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; 
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; 
}

void delay_us(uint32_t us) {
  uint32_t start = DWT->CYCCNT;
  uint32_t count = us * (SystemCoreClock / 1000000);
  while ((DWT->CYCCNT - start) < count);
}


void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}
