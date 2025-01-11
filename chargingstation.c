#include "stm32f4xx_hal_conf.h"  // I include the HAL library to manage hardware abstractions for my STM32F767ZI

// I define the GPIO pin connected to the LED
#define LED_PIN GPIO_PIN_13
#define LED_GPIO_PORT GPIOB

// I define the ADC channel connected to the ACS711 sensor
#define ACS711_ADC_CHANNEL ADC_CHANNEL_1

// I define the current threshold in digital units (ADC reading)
#define CURRENT_THRESHOLD 2000  // This threshold needs calibration based on actual sensor output and ADC reference voltage

ADC_HandleTypeDef hadc1; // Handle for the ADC, helps manage the ADC settings
GPIO_InitTypeDef GPIO_InitStruct = {0}; // Structure for GPIO settings, initialized to zero

void SystemClock_Config(void); // Declaration of system clock configuration function
void MX_GPIO_Init(void); // Declaration of GPIO initialization function
void MX_ADC_Init(void); // Declaration of ADC initialization function
uint32_t Read_Battery_Current(void); // Declaration of function to read current from the sensor

int main(void) {
    HAL_Init();  // I initialize the HAL Library which setups the hardware
    SystemClock_Config();  // I configure the system clock for optimal power efficiency and performance

    MX_GPIO_Init();  // I initialize GPIO for the LED to show status
    MX_ADC_Init();  // I initialize ADC for reading current from the ACS711

    // The main loop starts here
    while (1) {
        uint32_t current = Read_Battery_Current();  // I read the current from the ACS711 sensor

        // I turn on the LED if the current is below the threshold, indicating charging is complete
        if (current < CURRENT_THRESHOLD) {
            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET);
        }

        HAL_Delay(1000);  // I delay for 1 second for debouncing and to give time to visually check the LED status
    }
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // I configure the main internal regulator output voltage
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // I set up the oscillators according to the specified parameters for reliable operation
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    // I configure the clocks for the CPU, AHB, and APB buses
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();  // I enable the clock for GPIOB port
    GPIO_InitStruct.Pin = LED_PIN;  // I set the LED pin
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // I set the pin as an output
    GPIO_InitStruct.Pull = GPIO_NOPULL; // No pull-up or pull-down resistors
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // I set the speed to low
    HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct); // I initialize the LED GPIO with these settings
}

void MX_ADC_Init(void) {
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; // I set the clock prescaler
    hadc1.Init.Resolution = ADC_RESOLUTION_12B; // I choose 12-bit resolution for the ADC
    hadc1.Init.ScanConvMode = DISABLE; // I disable scan mode
    hadc1.Init.ContinuousConvMode = ENABLE; // I enable continuous conversion mode
    hadc1.Init.DiscontinuousConvMode = DISABLE; // I disable discontinuous conversion mode
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE; // No external trigger
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START; // I use software to start conversions
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT; // I align data to the right
    hadc1.Init.NbrOfConversion = 1; // I set number of conversions to 1
    hadc1.Init.DMAContinuousRequests = DISABLE; // I disable DMA requests
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV; // End of conversion flag raised at each single conversion
    HAL_ADC_Init(&hadc1); // I initialize the ADC with these settings

    // I configure the ADC channel
    sConfig.Channel = ACS711_ADC_CHANNEL; // I set the channel to the ACS711 sensor channel
    sConfig.Rank = 1; // I set this channel as the first rank
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES; // I set the sampling time
    HAL_ADC_ConfigChannel(&hadc1, &sConfig); // I apply these channel settings
}

uint32_t Read_Battery_Current(void) {
    HAL_ADC_Start(&hadc1);  // I start the ADC conversion
    if (HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK) { // I wait for conversion to complete
        return HAL_ADC_GetValue(&hadc1);  // I return the ADC value
    }
    return 0;  // I return 0 if there was an error during conversion
}
