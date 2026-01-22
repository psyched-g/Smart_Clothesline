#include "main.h"
#include <string.h>

/* ---- Pin Definitions ---- */
#define USART2_TX_PIN 2      // PA2
#define USART2_RX_PIN 3      // PA3
#define USART1_TX_PIN 9      // PA9
#define USART1_RX_PIN 10     // PA10
#define IN1_PIN 4            // Motor IN1 → PA4
#define IN2_PIN 5            // Motor IN2 → PA5
#define IN3_PIN 6  			 // Motor IN3 → PA6
#define IN4_PIN 7   		 // Motor IN4 → PA7

#define RAIN_ADC_CHANNEL 0   // PA0 as ADC input
#define HALL_PIN 8           // PA8 as digital input for Hall sensor

#define THRESHOLD 2500       // Rain sensor threshold
#define MAX_CMD_LEN 20

/* ---- Variables ---- */
volatile uint16_t adc_value = 0;
volatile uint8_t rain_detected = 0;
volatile uint8_t mode = 0;   // 0 = Auto, 1 = Manual
volatile uint8_t magnet_detected = 0;

char command[MAX_CMD_LEN];
uint8_t cmd_index = 0;

/* ---- Function Prototypes ---- */
void SystemClock_Config(void);
void GPIO_Init(void);
void ADC1_Init(void);
void USART1_Init(void);
void USART2_Init(void);
void TIM2_Init(void);
void Serial_Print(const char *msg);
void Motor_Forward(void);
void Motor_Stop(void);
void delay_ms(uint32_t ms);

/* ------------------------------------------------------------------ */
/* ---------------------------- MAIN -------------------------------- */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    ADC1_Init();
    TIM2_Init();
    USART1_Init();   // Serial monitor
    USART2_Init();   // Command interface

    __enable_irq(); // Enable global interrupts

    Serial_Print("System Initialized\r\n");

    while (1)
    {
        /* Read Hall sensor state */
        magnet_detected = ((GPIOA->IDR & (1 << HALL_PIN)) == 0); // Active LOW

        /* If magnet detected → Stop motor immediately */
        if (!magnet_detected)
        {
            Motor_Stop();
            Serial_Print("Magnet detected - Motor stopped\r\n");
            delay_ms(500);
            continue;
        }
        else
        	   Serial_Print("Magnet not detected \r\n");

        /* Automatic mode logic (rain sensor control) */
        if (mode == 0)
        {
            if (rain_detected)
            {
                Motor_Forward();
                Serial_Print("Rain detected - Motor started\r\n");
            }
            else
            {
                Motor_Stop();
            }
        }

        delay_ms(100);
    }
}

/* ------------------------------------------------------------------ */
/* ------------------------ Motor Control --------------------------- */
void Motor_Forward(void)
{
    GPIOA->BSRR = (1 << IN1_PIN);        // IN1 HIGH
    GPIOA->BSRR = (1 << (IN2_PIN + 16)); // IN2 LOW

    /* Motor B: Opposite direction */
    GPIOA->BSRR = (1 << (IN3_PIN + 16));   // IN3 LOW
    GPIOA->BSRR = (1 << IN4_PIN);          // IN4 HIGH
}
void Motor_Reverse(void)
{
    GPIOA->BSRR = (1 << IN2_PIN);          // IN2 HIGH
    GPIOA->BSRR = (1 << (IN1_PIN + 16));   // IN1 LOW

    GPIOA->BSRR = (1 << (IN4_PIN + 16));   // IN4 LOW
    GPIOA->BSRR = (1 << IN3_PIN);          // IN3 HIGH
}

void Motor_Stop(void)
{
    GPIOA->BSRR = (1 << (IN1_PIN + 16)); // IN1 LOW
    GPIOA->BSRR = (1 << (IN2_PIN + 16)); // IN2 LOW
    GPIOA->BSRR = (1 << (IN3_PIN + 16)); // IN3 LOW
    GPIOA->BSRR = (1 << (IN4_PIN + 16)); // IN4 LOW
}

/* ------------------------------------------------------------------ */
/* --------------------------- GPIO INIT ---------------------------- */
void GPIO_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* ---- Motor pins PA4, PA5 output ---- */
    GPIOA->MODER &= ~((3 << (2 * IN1_PIN)) | (3 << (2 * IN2_PIN)));
    GPIOA->MODER |= ((1 << (2 * IN1_PIN)) | (1 << (2 * IN2_PIN))); // Output
    GPIOA->OTYPER &= ~((1 << IN1_PIN) | (1 << IN2_PIN));           // Push-pull
    GPIOA->OSPEEDR &= ~((3 << (2 * IN1_PIN)) | (3 << (2 * IN2_PIN)));

    /* ---- Motor pins PA6, PA7 output ---- */
    GPIOA->MODER &= ~((3 << (2 * IN3_PIN)) | (3 << (2 * IN4_PIN)));   // Clear mode bits
    GPIOA->MODER |=  ((1 << (2 * IN3_PIN)) | (1 << (2 * IN4_PIN)));   // Set as output mode

    GPIOA->OTYPER &= ~((1 << IN3_PIN) | (1 << IN4_PIN));              // Push-pull

    GPIOA->OSPEEDR &= ~((3 << (2 * IN3_PIN)) | (3 << (2 * IN4_PIN))); // Low speed

    /* ---- ADC pin PA0 analog ---- */
    GPIOA->MODER |= (3 << (2 * RAIN_ADC_CHANNEL));

    /* ---- USART2 pins (PA2 TX, PA3 RX) ---- */
    GPIOA->MODER &= ~((3 << (2 * USART2_TX_PIN)) | (3 << (2 * USART2_RX_PIN)));
    GPIOA->MODER |= ((2 << (2 * USART2_TX_PIN)) | (2 << (2 * USART2_RX_PIN))); // AF
    GPIOA->AFR[0] |= (7 << (4 * USART2_TX_PIN)) | (7 << (4 * USART2_RX_PIN));  // AF7
    GPIOA->OSPEEDR |= (3 << (2 * USART2_TX_PIN)) | (3 << (2 * USART2_RX_PIN));

    /* ---- USART1 pins (PA9 TX, PA10 RX) ---- */
    GPIOA->MODER &= ~((3 << (2 * USART1_TX_PIN)) | (3 << (2 * USART1_RX_PIN)));
    GPIOA->MODER |= ((2 << (2 * USART1_TX_PIN)) | (2 << (2 * USART1_RX_PIN))); // AF
    GPIOA->AFR[1] |= (7 << ((USART1_TX_PIN - 8) * 4)) | (7 << ((USART1_RX_PIN - 8) * 4)); // AF7
    GPIOA->OSPEEDR |= (3 << (2 * USART1_TX_PIN)) | (3 << (2 * USART1_RX_PIN));

    /* ---- PA7 (Hall sensor) input with internal pull-up ---- */
    GPIOA->MODER &= ~(3 << (2 * HALL_PIN));  // Input
    GPIOA->PUPDR &= ~(3 << (2 * HALL_PIN));
    GPIOA->PUPDR |= (1 << (2 * HALL_PIN));   // Pull-up
}

/* ------------------------------------------------------------------ */
/* --------------------------- ADC INIT ----------------------------- */
void ADC1_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC->CCR |= (1 << 16);              // Prescaler PCLK2/4
    ADC1->SMPR2 |= (3 << (3 * RAIN_ADC_CHANNEL));
    ADC1->SQR3 = RAIN_ADC_CHANNEL;
    ADC1->CR2 &= ~ADC_CR2_EXTEN;        // Clear bits
    ADC1->CR2 |=  (1 << 28);            // EXTEN = 01 → rising edge trigger

     // EXTSEL = TIM2 TRGO
    ADC1->CR2 &= ~(0xF << 24);
    ADC1->CR2 |=  (0x3 << 24);          // EXTSEL = 011 → TIM2 TRGO
    ADC1->CR2 |= ADC_CR2_CONT;
    ADC1->CR1 |= ADC_CR1_EOCIE;
    ADC1->CR2 |= ADC_CR2_ADON;

    NVIC_EnableIRQ(ADC_IRQn);
    NVIC_SetPriority(ADC_IRQn, 0);

    ADC1->CR2 |= ADC_CR2_SWSTART;
}

/* ------------------------------------------------------------------ */
/* --------------------------- ADC IRQ ------------------------------ */
void ADC_IRQHandler(void)
{
    if (ADC1->SR & ADC_SR_EOC)
    {
        adc_value = ADC1->DR;

        if (adc_value < THRESHOLD)
            rain_detected = 1;
        else
            rain_detected = 0;

        ADC1->CR2 |= ADC_CR2_SWSTART;
    }
}

/* ------------------------------------------------------------------ */
/* --------------------------- USART1 (Serial) ---------------------- */
void USART1_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;     // Enable USART1 clock
    USART1->BRR = 0x683;;                     // 115200 baud @16MHz
    USART1->CR1 = USART_CR1_TE | USART_CR1_UE; // Enable TX and USART
}

void Serial_Print(const char *msg)
{
    while (*msg)
    {
        while (!(USART1->SR & USART_SR_TXE)); // Wait for TX buffer empty
        USART1->DR = (*msg++ & 0xFF);
    }
}

/* ------------------------------------------------------------------ */
/* --------------------------- USART2 (Command) --------------------- */
void USART2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    USART2->BRR = 0x683; // 9600 baud @16MHz
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_SetPriority(USART2_IRQn, 0);
}


/* ------------------------------------------------------------------ */
/* --------------------------- USART2 IRQ ---------------------------- */
void USART2_IRQHandler(void)
{
    if (USART2->SR & USART_SR_RXNE)
    {
        char c = USART2->DR;

        if (c == '\n' || c == '\r')
        {
            command[cmd_index] = '\0';

            if (strcmp(command, "MODE_AUTO") == 0)
                mode = 0;
            else if (strcmp(command, "MODE_MANUAL") == 0)
                {
            	mode = 1;
            	Motor_Stop();
                }

            else if (mode == 1)
            {
            	if (strcmp(command, "FORWARD_ON") == 0)
            	                {
            	                    Motor_Forward();
            	                    Serial_Print("forward on\r\n");
            	                }
            	                else if (strcmp(command, "FORWARD_STOP") == 0)
            	                {
            	                    Motor_Stop();
            	                    Serial_Print("forward stop\r\n");
            	                }
            	                else if (strcmp(command, "REVERSE_ON") == 0)
            	                {
            	                    Motor_Reverse();
            	                    Serial_Print("reverse on\r\n");
            	                }
            	                else if (strcmp(command, "REVERSE_STOP") == 0)
            	                {
            	                    Motor_Stop();
            	                    Serial_Print("reverse stop\r\n");
            	                }
            }

            cmd_index = 0;
            memset(command, 0, sizeof(command));
        }
        else
        {
            if (cmd_index < MAX_CMD_LEN - 1)
                command[cmd_index++] = c;
        }
    }
}

void TIM2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;    // Enable TIM2 clock

    TIM2->PSC = 16000 - 1;                 // 16 MHz / 16000 = 1000 Hz
    TIM2->ARR = 500 - 1;                   // 500 ms period

    // Master mode: update event → TRGO
    TIM2->CR2 &= ~TIM_CR2_MMS;
    TIM2->CR2 |=  (2 << 4);                // MMS = 010 → Update event

    TIM2->CR1 |= TIM_CR1_CEN;              // Enable timer
}


/* ------------------------------------------------------------------ */
/* --------------------------- Delay Function ----------------------- */
void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms * 4000; i++)
        __NOP();
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
