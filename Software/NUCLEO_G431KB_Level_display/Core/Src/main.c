/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sk6812.h"
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_NUM_CONVERSIONS 1
#define ADC_RAW_TO_VOLTAGE(adc_value) ((adc_value * 3.3) / 4095) // Assuming a 12-bit ADC and 3.3V reference voltage
#define SENSOR_VOLTAGE_LOW_THRESHOLD_DEFAULT 0.5 // Voltage threshold for low level indication (adjust as needed)
#define SENSOR_VOLTAGE_HIGH_THRESHOLD_DEFAULT 4.5 // Voltage threshold for high level indication (adjust as needed)
#define SENSOR_VOLTAGE_LOW_ERROR_THRESHOLD 0.4 // Voltage threshold for low level error indication (adjust as needed)
#define SENSOR_VOLTAGE_HIGH_ERROR_THRESHOLD 4.7 // Voltage threshold for high level error indication (adjust as needed)

#define LED_COUNT 8 // Adjust this to the number of LEDs in your setup
#define BRIGHTNESS_MIN 0.1 // Minimum brightness (10%)
#define BRIGHTNESS_MAX 1.0 // Maximum brightness (100%)
#define BRIGHTNESS_STEP 0.05 // Step size for brightness adjustment (5%)
#define BRIGHTNESS_DEFAULT 0.3 // Default brightness (30%)
#define BRIGHTNESS_SAVE_DELAY_MS 1500U
#define CAL_FULL_VOLTAGE_HOLD_MS  4000U
#define CAL_EMPTY_VOLTAGE_HOLD_MS 7000U
#define CAL_BRIGHTNESS_HOLD_MS    10000U
#define CAL_RESET_HOLD_MS         15000U
#define CALIBRATION_TIMEOUT_MS    6000U
#define BRIGHTNESS_NV_MAGIC 0x42524631U
#define BRIGHTNESS_NV_ADDRESS (FLASH_BASE + (128U * 1024U) - FLASH_PAGE_SIZE)
#define CALIBRATION_NV_MARKER 0xA55AF00DU
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;
DMA_HandleTypeDef hdma_tim2_ch3;

/* USER CODE BEGIN PV */
volatile uint16_t adcData[ADC_NUM_CONVERSIONS];
volatile float potVoltage[ADC_NUM_CONVERSIONS];
volatile float sensorVoltage[ADC_NUM_CONVERSIONS];
typedef struct {
  uint8_t stable_state;
  uint8_t last_raw;
  uint32_t last_change_ms;
} DebouncedInput;

volatile uint8_t sw1_state = 0;
volatile uint8_t sw2_state = 0;
static DebouncedInput sw1_db = {0, 0, 0};
static DebouncedInput sw2_db = {0, 0, 0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

uint32_t hsl_to_rgb(uint8_t h, uint8_t s, uint8_t l);
static uint8_t debounce_read_gpio(DebouncedInput *db, GPIO_TypeDef *port, uint16_t pin, uint32_t debounce_ms);
static double brightness_clamp(double brightness);
static uint16_t brightness_to_milli(double brightness);
static double brightness_from_milli(uint16_t milli);
static uint16_t voltage_to_milli(double voltage);
static double voltage_from_milli(uint16_t milli);
static void normalize_sensor_thresholds(double *low_threshold, double *high_threshold);
static void calibration_values_load_from_flash(double *brightness, double *low_threshold, double *high_threshold);
static void calibration_values_write_to_flash(double brightness, double low_threshold, double high_threshold);
static void brightness_save_to_flash(double brightness);
static void low_threshold_save_to_flash(double low_threshold);
static void high_threshold_save_to_flash(double high_threshold);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static double brightness_clamp(double brightness) {
  if (brightness < BRIGHTNESS_MIN) {
    return BRIGHTNESS_MIN;
  }
  if (brightness > BRIGHTNESS_MAX) {
    return BRIGHTNESS_MAX;
  }
  return brightness;
}

static uint16_t brightness_to_milli(double brightness) {
  return (uint16_t)(brightness_clamp(brightness) * 1000.0 + 0.5);
}

static double brightness_from_milli(uint16_t milli) {
  return brightness_clamp((double)milli / 1000.0);
}

static uint16_t voltage_to_milli(double voltage) {
  if (voltage < 0.0) {
    voltage = 0.0;
  }
  if (voltage > 5.0) {
    voltage = 5.0;
  }
  return (uint16_t)(voltage * 1000.0 + 0.5);
}

static double voltage_from_milli(uint16_t milli) {
  return (double)milli / 1000.0;
}

static void normalize_sensor_thresholds(double *low_threshold, double *high_threshold) {
  if (*low_threshold < 0.0) {
    *low_threshold = 0.0;
  }
  if (*low_threshold > 5.0) {
    *low_threshold = 5.0;
  }
  if (*high_threshold < 0.0) {
    *high_threshold = 0.0;
  }
  if (*high_threshold > 5.0) {
    *high_threshold = 5.0;
  }

  if (*high_threshold <= *low_threshold) {
    *high_threshold = *low_threshold + 0.1;
    if (*high_threshold > 5.0) {
      *high_threshold = 5.0;
      *low_threshold = 4.9;
    }
  }
}

static void calibration_values_load_from_flash(double *brightness, double *low_threshold, double *high_threshold) {
  const uint64_t raw0 = *(const uint64_t *)BRIGHTNESS_NV_ADDRESS;
  const uint64_t raw1 = *(const uint64_t *)(BRIGHTNESS_NV_ADDRESS + 8U);
  const uint32_t magic = (uint32_t)(raw0 & 0xFFFFFFFFULL);
  const uint16_t brightness_milli = (uint16_t)((raw0 >> 32) & 0xFFFFULL);
  const uint16_t low_milli = (uint16_t)((raw0 >> 48) & 0xFFFFULL);
  const uint16_t high_milli = (uint16_t)(raw1 & 0xFFFFULL);
  const uint16_t check = (uint16_t)((raw1 >> 16) & 0xFFFFULL);
  const uint32_t marker = (uint32_t)((raw1 >> 32) & 0xFFFFFFFFULL);
  const uint16_t expected_check = (uint16_t)(brightness_milli ^ low_milli ^ high_milli ^ 0xA5A5U);

  if ((magic == BRIGHTNESS_NV_MAGIC) &&
      (marker == CALIBRATION_NV_MARKER) &&
      (check == expected_check)) {
    *brightness = brightness_from_milli(brightness_milli);
    *low_threshold = voltage_from_milli(low_milli);
    *high_threshold = voltage_from_milli(high_milli);
    normalize_sensor_thresholds(low_threshold, high_threshold);
  }
}

static void calibration_values_write_to_flash(double brightness, double low_threshold, double high_threshold) {
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t page_error = 0;
  const uint16_t brightness_milli = brightness_to_milli(brightness);
  uint16_t low_milli = 0;
  uint16_t high_milli = 0;
  uint16_t check = 0;
  uint64_t payload0 = 0;
  uint64_t payload1 = 0;

  normalize_sensor_thresholds(&low_threshold, &high_threshold);

  low_milli = voltage_to_milli(low_threshold);
  high_milli = voltage_to_milli(high_threshold);
  check = (uint16_t)(brightness_milli ^ low_milli ^ high_milli ^ 0xA5A5U);

  payload0 = ((uint64_t)low_milli << 48)
           | ((uint64_t)brightness_milli << 32)
           | (uint64_t)BRIGHTNESS_NV_MAGIC;

  payload1 = ((uint64_t)CALIBRATION_NV_MARKER << 32)
           | ((uint64_t)check << 16)
           | (uint64_t)high_milli;

  HAL_FLASH_Unlock();

  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Banks = FLASH_BANK_1;
  erase.Page = (BRIGHTNESS_NV_ADDRESS - FLASH_BASE) / FLASH_PAGE_SIZE;
  erase.NbPages = 1;

  if (HAL_FLASHEx_Erase(&erase, &page_error) == HAL_OK) {
    (void)HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, BRIGHTNESS_NV_ADDRESS, payload0);
    (void)HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, BRIGHTNESS_NV_ADDRESS + 8U, payload1);
  }

  HAL_FLASH_Lock();
}

static void brightness_save_to_flash(double brightness) {
  double current_brightness = BRIGHTNESS_DEFAULT;
  double current_low_threshold = SENSOR_VOLTAGE_LOW_THRESHOLD_DEFAULT;
  double current_high_threshold = SENSOR_VOLTAGE_HIGH_THRESHOLD_DEFAULT;

  calibration_values_load_from_flash(&current_brightness, &current_low_threshold, &current_high_threshold);
  calibration_values_write_to_flash(brightness, current_low_threshold, current_high_threshold);
}

static void low_threshold_save_to_flash(double low_threshold) {
  double current_brightness = BRIGHTNESS_DEFAULT;
  double current_low_threshold = SENSOR_VOLTAGE_LOW_THRESHOLD_DEFAULT;
  double current_high_threshold = SENSOR_VOLTAGE_HIGH_THRESHOLD_DEFAULT;

  calibration_values_load_from_flash(&current_brightness, &current_low_threshold, &current_high_threshold);
  calibration_values_write_to_flash(current_brightness, low_threshold, current_high_threshold);
}

static void high_threshold_save_to_flash(double high_threshold) {
  double current_brightness = BRIGHTNESS_DEFAULT;
  double current_low_threshold = SENSOR_VOLTAGE_LOW_THRESHOLD_DEFAULT;
  double current_high_threshold = SENSOR_VOLTAGE_HIGH_THRESHOLD_DEFAULT;

  calibration_values_load_from_flash(&current_brightness, &current_low_threshold, &current_high_threshold);
  calibration_values_write_to_flash(current_brightness, current_low_threshold, high_threshold);
}

static uint8_t debounce_read_gpio(DebouncedInput *db, GPIO_TypeDef *port, uint16_t pin, uint32_t debounce_ms) {
  uint8_t raw = (uint8_t)HAL_GPIO_ReadPin(port, pin);
  uint32_t now = HAL_GetTick();

  if (raw != db->last_raw) {
    db->last_raw = raw;
    db->last_change_ms = now;
  }

  if ((now - db->last_change_ms) >= debounce_ms) {
    db->stable_state = raw;
  }

  return db->stable_state;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  for (uint8_t i = 0; i < ADC_NUM_CONVERSIONS; ++i) {
    potVoltage[i] = ADC_RAW_TO_VOLTAGE(adcData[i]);
    //sensor votage is the adcVoltage scaled to 5v
    sensorVoltage[i]  = potVoltage[i] * (5.0 / 3.3);
    
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  bool sensor_voltage_low_error = false;
  bool sensor_voltage_high_error = false;
  bool calibration_mode = false;
  bool locked_for_calibration = false;
  bool full_voltage_calibration_mode = false;
  bool empty_voltage_calibration_mode = false;
  bool brightness_calibration_mode = false;

  uint32_t sw1_held_since_ms = 0;
  uint8_t sw1_held_active = 0;
  uint32_t last_button_activity_ms = 0;

  double sensor_voltage_low_threshold = SENSOR_VOLTAGE_LOW_THRESHOLD_DEFAULT;
  double sensor_voltage_high_threshold = SENSOR_VOLTAGE_HIGH_THRESHOLD_DEFAULT;
  double brightness_fraction = BRIGHTNESS_DEFAULT; // Brightness fraction constrained to BRIGHTNESS_MIN..BRIGHTNESS_MAX
  double brightness_saved_fraction = brightness_fraction;
  double low_threshold_saved = sensor_voltage_low_threshold;
  double high_threshold_saved = sensor_voltage_high_threshold;
  uint16_t flash_interval_ms = 500; // Flash interval in milliseconds
  uint8_t r, g, b; // Variables to hold RGB values for each LED
  uint8_t sw1_prev = 0;
  uint8_t sw2_prev = 0;

  calibration_values_load_from_flash(&brightness_fraction, &sensor_voltage_low_threshold, &sensor_voltage_high_threshold);
  brightness_saved_fraction = brightness_fraction;
  low_threshold_saved = sensor_voltage_low_threshold;
  high_threshold_saved = sensor_voltage_high_threshold;

  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adcData, ADC_NUM_CONVERSIONS);
  HAL_TIM_Base_Start(&htim8);

  sw1_db.last_raw = (uint8_t)HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin);
  sw1_db.stable_state = sw1_db.last_raw;
  sw1_db.last_change_ms = HAL_GetTick();

  sw2_db.last_raw = (uint8_t)HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin);
  sw2_db.stable_state = sw2_db.last_raw;
  sw2_db.last_change_ms = HAL_GetTick();

  sw1_prev = sw1_db.stable_state;
  sw2_prev = sw2_db.stable_state;
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    sw1_state = debounce_read_gpio(&sw1_db, SW1_GPIO_Port, SW1_Pin, 20U);
    sw2_state = debounce_read_gpio(&sw2_db, SW2_GPIO_Port, SW2_Pin, 20U);



    // Track any button activity for calibration timeout
    if ((sw1_state != sw1_prev) || (sw1_state == 1U) || (sw2_state != sw2_prev)) {
      last_button_activity_ms = HAL_GetTick();
    }

    // SW1 hold-duration calibration mode selection
    if (sw1_state == 1U ) {
      if (!sw1_held_active) {
        sw1_held_active = 1U;
        sw1_held_since_ms = HAL_GetTick();
      } else {
        uint32_t held_ms = HAL_GetTick() - sw1_held_since_ms;
        if (locked_for_calibration) {
          // If already locked in a calibration mode.
          //save full value after holding for 4 seconds.
          if (full_voltage_calibration_mode && held_ms >= 4000U) {
            sensor_voltage_high_threshold = sensorVoltage[0];
            normalize_sensor_thresholds(&sensor_voltage_low_threshold, &sensor_voltage_high_threshold);
            if (voltage_to_milli(high_threshold_saved) != voltage_to_milli(sensor_voltage_high_threshold)) {
              high_threshold_save_to_flash(sensor_voltage_high_threshold);
              high_threshold_saved = sensor_voltage_high_threshold;
            }
            //set calibration modes back to false
            full_voltage_calibration_mode = false;
            empty_voltage_calibration_mode = false;
            brightness_calibration_mode = false;
            calibration_mode = false;
            locked_for_calibration = false;
            sw1_held_active = 0U; // Reset hold state to prevent multiple saves without a new button press
          }
          //save empty value after holding for 4 seconds.
          if (empty_voltage_calibration_mode && held_ms >= 4000U) {
            sensor_voltage_low_threshold = sensorVoltage[0];
            normalize_sensor_thresholds(&sensor_voltage_low_threshold, &sensor_voltage_high_threshold);
            if (voltage_to_milli(low_threshold_saved) != voltage_to_milli(sensor_voltage_low_threshold)) {
              low_threshold_save_to_flash(sensor_voltage_low_threshold);
              low_threshold_saved = sensor_voltage_low_threshold;
            }
            //set calibration modes back to false
            full_voltage_calibration_mode = false;
            empty_voltage_calibration_mode = false;
            brightness_calibration_mode = false;
            calibration_mode = false;
            locked_for_calibration = false;
            sw1_held_active = 0U; // Reset hold state to prevent multiple saves without a new button press
          }

  
        } else {

        
        if (held_ms >= CAL_FULL_VOLTAGE_HOLD_MS && held_ms < CAL_EMPTY_VOLTAGE_HOLD_MS) {
          full_voltage_calibration_mode = true;
          empty_voltage_calibration_mode = false;
          brightness_calibration_mode = false;
          calibration_mode = true;
        }  
        if (held_ms >= CAL_EMPTY_VOLTAGE_HOLD_MS && held_ms < CAL_BRIGHTNESS_HOLD_MS) {
          full_voltage_calibration_mode = false;
          empty_voltage_calibration_mode = true;
          brightness_calibration_mode = false;
          calibration_mode = true;
        }  
        if (held_ms >=  CAL_BRIGHTNESS_HOLD_MS && held_ms < CAL_RESET_HOLD_MS) {
          full_voltage_calibration_mode = false;
          empty_voltage_calibration_mode = false;
          brightness_calibration_mode = true;
          calibration_mode = true;
        }  
        if (held_ms >= CAL_RESET_HOLD_MS) {
          // Reset all calibrations
          full_voltage_calibration_mode = false;
          empty_voltage_calibration_mode = false;
          brightness_calibration_mode = false;
          calibration_mode = false;
          locked_for_calibration = false;
          sensor_voltage_low_threshold = SENSOR_VOLTAGE_LOW_THRESHOLD_DEFAULT;
          sensor_voltage_high_threshold = SENSOR_VOLTAGE_HIGH_THRESHOLD_DEFAULT;
          brightness_fraction = BRIGHTNESS_DEFAULT;
          brightness_save_to_flash(brightness_fraction);
          low_threshold_save_to_flash(sensor_voltage_low_threshold);
          high_threshold_save_to_flash(sensor_voltage_high_threshold);
          brightness_saved_fraction = brightness_fraction;
          low_threshold_saved = sensor_voltage_low_threshold;
          high_threshold_saved = sensor_voltage_high_threshold;
        }
      }
    }
    } else {
      sw1_held_active = 0U;
      if (calibration_mode && !locked_for_calibration) {
        // Lock in the calibration mode on button release to prevent accidental changes
        locked_for_calibration = true;
      } else if (!calibration_mode) {
        // If not in calibration mode, reset the locked state
        locked_for_calibration = false;
      }
    }

    // In brightness calibration mode, each SW1 press cycles brightness through the allowed range.
    if (brightness_calibration_mode && (sw1_state == 1U) && (sw1_prev == 0U)) {
      brightness_fraction += BRIGHTNESS_STEP;
      if (brightness_fraction > BRIGHTNESS_MAX) {
        brightness_fraction = BRIGHTNESS_MIN;
      }
    }

    // Exit all calibration modes after CALIBRATION_TIMEOUT_MS with no button activity
    if (calibration_mode &&
        ((HAL_GetTick() - last_button_activity_ms) >= CALIBRATION_TIMEOUT_MS)) {


      // Save brightness if it has changed and we are exiting calibration mode and     brightness_calibration_mode was active
      if (brightness_calibration_mode && (brightness_to_milli(brightness_saved_fraction) != brightness_to_milli(brightness_fraction))) {
        brightness_save_to_flash(brightness_fraction);
        brightness_saved_fraction = brightness_fraction;
      }



      calibration_mode = false;
      full_voltage_calibration_mode = false;
      empty_voltage_calibration_mode = false;
      brightness_calibration_mode = false;
      locked_for_calibration = false;
    }

    // if ((sw1_state == 1U) && (sw1_prev == 0U)) {
    //   brightness_fraction = brightness_clamp(brightness_fraction + BRIGHTNESS_STEP);
    // }

    // if ((sw2_state == 1U) && (sw2_prev == 0U)) {
    //   brightness_fraction = brightness_clamp(brightness_fraction - BRIGHTNESS_STEP);
    // }

    // if (brightness_to_milli(previous_brightness) != brightness_to_milli(brightness_fraction)) {
    //   brightness_save_pending = 1U;
    //   brightness_last_change_ms = HAL_GetTick();
    // }

    // if (brightness_save_pending && ((HAL_GetTick() - brightness_last_change_ms) >= BRIGHTNESS_SAVE_DELAY_MS)) {
    //   if (brightness_to_milli(brightness_saved_fraction) != brightness_to_milli(brightness_fraction)) {
    //     brightness_save_to_flash(brightness_fraction);
    //     brightness_saved_fraction = brightness_fraction;
    //   }
    //   brightness_save_pending = 0U;
    // }

    sw1_prev = sw1_state;
    sw2_prev = sw2_state;

    


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    sensor_voltage_low_error = (sensorVoltage[0] < SENSOR_VOLTAGE_LOW_ERROR_THRESHOLD);
    sensor_voltage_high_error = (sensorVoltage[0] > SENSOR_VOLTAGE_HIGH_ERROR_THRESHOLD);

    double level_percent_fraction = (sensorVoltage[0] - sensor_voltage_low_threshold) / (sensor_voltage_high_threshold - sensor_voltage_low_threshold); // Map between calibrated empty/full thresholds
    //clamp to 0..1
    if (level_percent_fraction < 0) {
      level_percent_fraction = 0;
    }
    if (level_percent_fraction > 1) {
      level_percent_fraction = 1;
    }
    double level_percent = level_percent_fraction * 100; // Convert to percentage
    
    // Update the LEDs based on the level percentage
    // if level is <2% the first two leds should flash red. If the level is 100% all leds should be green.

      

      for (uint8_t i = 0; i < LED_COUNT; i++) {
        // The leds need to act like a level display. if the level is 0% the first two leds should flash red. 
        // If the level is 100% all leds should be green.
        if (full_voltage_calibration_mode) {
          // Flash top two LEDs green for full voltage calibration
          if (i >= LED_COUNT - 2) {
            if ((HAL_GetTick() / flash_interval_ms) % 2 == 0) {
              led_set_RGB(i, 0, (uint8_t)(255 * brightness_fraction), 0);
            } else {
              led_set_RGB(i, 0, 0, 0);
            }
          } else {
            led_set_RGB(i, 0, 0, 0);
          }
        } else if (empty_voltage_calibration_mode) {
          // Flash bottom two LEDs green for empty voltage calibration
          if (i < 2) {
            if ((HAL_GetTick() / flash_interval_ms) % 2 == 0) {
              led_set_RGB(i, 0, (uint8_t)(255 * brightness_fraction), 0);
            } else {
              led_set_RGB(i, 0, 0, 0);
            }
          } else {
            led_set_RGB(i, 0, 0, 0);
          }
        } else if (brightness_calibration_mode) {
          // solid white for brightness calibration
          led_set_RGB(i, (uint8_t)(255 * brightness_fraction), (uint8_t)(255 * brightness_fraction), (uint8_t)(255 * brightness_fraction));
        } else if (sensor_voltage_low_error) {
          // Flash first and last LED yellow on sensor low error
          if (i == 0 || i == LED_COUNT - 1) {
            if ((HAL_GetTick() / flash_interval_ms) % 2 == 0) {
              led_set_RGB(i, (uint8_t)(255 * brightness_fraction), (uint8_t)(255 * brightness_fraction), 0);
            } else {
              led_set_RGB(i, 0, 0, 0);
            }
          } else {
            led_set_RGB(i, 0, 0, 0);
          }
        } else if (sensor_voltage_high_error) {
          // Flash first and last LED red on sensor high error
          if (i == 0 || i == LED_COUNT - 1) {
            if ((HAL_GetTick() / flash_interval_ms) % 2 == 0) {
              led_set_RGB(i, (uint8_t)(255 * brightness_fraction), 0, 0);
            } else {
              led_set_RGB(i, 0, 0, 0);
            }
          } else {
            led_set_RGB(i, 0, 0, 0);
          }
        } else if (level_percent < 2) {
          // Flash red for 0-2%
          if ((HAL_GetTick() / flash_interval_ms) % 2 == 0) {
            r = 255 * brightness_fraction;
            g = 0;
            b = 0;
          } else {
            r = 0;
            g = 0;
            b = 0;
          }
          if (i < 2) {
            // This led should be on
            led_set_RGB(i, r, g, b);
          } else {
            // This led should be off
            led_set_RGB(i, 0, 0, 0);
          }
        } else if (level_percent >= 2 && level_percent < 25) {
          // solid red for 2-25%
          r = 255 * brightness_fraction;
          g = 0;
          b = 0;
          if (i < level_percent_fraction * LED_COUNT) {
            // This led should be on
            led_set_RGB(i, r, g, b);
          } else {
            // This led should be off
            led_set_RGB(i, 0, 0, 0);
          }
        } else if (level_percent >= 25 && level_percent < 50) {
          // solid yellow for 25-50%
          r = 255 * brightness_fraction; // red
          g = 255 * brightness_fraction; // green
          b = 0;
          if (i < level_percent_fraction * LED_COUNT) {
            // This led should be on
            led_set_RGB(i, r, g, b);
          } else {
            // This led should be off
            led_set_RGB(i, 0, 0, 0);
          }
        } else {
          // solid green for 50-100%
          r = 0;
          g =  255 * brightness_fraction; // green
          b = 0;
          if (i < level_percent_fraction * LED_COUNT) {
            // This led should be on
            led_set_RGB(i, r, g, b);
          } else {
            // This led should be off
            led_set_RGB(i, 0, 0, 0);
          }
        }
      }
		led_render();
		// Some delay
		HAL_Delay(10);

   

  }
  /* USER CODE END 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T8_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 2-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 16000-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = SW1_Pin | SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Fast hsl2rgb algorithm: https://stackoverflow.com/questions/13105185/fast-algorithm-for-rgb-hsl-conversion
uint32_t hsl_to_rgb(uint8_t h, uint8_t s, uint8_t l) {
	if(l == 0) return 0;

	volatile uint8_t  r, g, b, lo, c, x, m;
	volatile uint16_t h1, l1, H;
	l1 = l + 1;
	if (l < 128)    c = ((l1 << 1) * s) >> 8;
	else            c = (512 - (l1 << 1)) * s >> 8;

	H = h * 6;              // 0 to 1535 (actually 1530)
	lo = H & 255;           // Low byte  = primary/secondary color mix
	h1 = lo + 1;

	if ((H & 256) == 0)   x = h1 * c >> 8;          // even sextant, like red to yellow
	else                  x = (256 - h1) * c >> 8;  // odd sextant, like yellow to green

	m = l - (c >> 1);
	switch(H >> 8) {       // High byte = sextant of colorwheel
	 case 0 : r = c; g = x; b = 0; break; // R to Y
	 case 1 : r = x; g = c; b = 0; break; // Y to G
	 case 2 : r = 0; g = c; b = x; break; // G to C
	 case 3 : r = 0; g = x; b = c; break; // C to B
	 case 4 : r = x; g = 0; b = c; break; // B to M
	 default: r = c; g = 0; b = x; break; // M to R
	}

	return (((uint32_t)r + m) << 16) | (((uint32_t)g + m) << 8) | ((uint32_t)b + m);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/**
  * @}
  */

/**
  * @}
  */

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
