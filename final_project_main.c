//*****************************************************************************
//  DESIGNER NAME:  Tyler Mulvehill and Hakan Demirtas
//
//   PROJECT NAME:  Smart Thermostat Fan
//
//      FILE NAME:  final_project_main.c
//
//-----------------------------------------------------------------------------
//
// DESCRIPTION:
//   Smart thermostat fan demo on the LP_MSPM0G3507 + CSC202 Expansion board.
//   - PB2: start / stop fan control
//   - PB1: toggle real thermistor vs demo potentiometer input
//   - ADC ch 7: setpoint pot (target temperature)
//   - ADC ch 5: thermistor (actual temperature, in real mode)
//   - Fan PWM on PB22 (LP_RGB_BLU pin) using software PWM
//   - Active buzzer on PB27 (LP_RGB_GRN pin)
//   - LED bar via SPI shows duty as bar graph
//   - 7-segment shows duty percent (two digits)
//   - LCD shows current / target temp and mode
//
//*****************************************************************************

//-----------------------------------------------------------------------------
// Loads standard C include files
//-----------------------------------------------------------------------------
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

//-----------------------------------------------------------------------------
// Loads MSP launchpad board support macros and definitions
//-----------------------------------------------------------------------------
#include <ti/devices/msp/msp.h>
#include "clock.h"
#include "LaunchPad.h"
#include "lcd1602.h"
#include "adc.h"
#include "spi.h"
#include "uart.h"

//-----------------------------------------------------------------------------
// Define function prototypes used by the program
//-----------------------------------------------------------------------------
void run_smart_thermostat(void);
void config_pb1_interrupt(void);
void config_pb2_interrupt(void);
void GROUP1_IRQHandler(void);

// PB27 buzzer
void buzzer_init(void);
static inline void buzzer_on(void);
static inline void buzzer_off(void);

// PB22 motor PWM
void fan_init_pb22(void);
void fan_pwm_init_pb22(uint32_t pwm_freq_hz);
void fan_set_pwm_dc_pb22(uint8_t duty);

static void uart_write_string(const char *s);
static void update_lcd(uint16_t actual_f,
                       uint16_t target_f,
                       uint8_t duty,
                       bool running,
                       bool demo_mode);
static void update_led_bar(uint8_t duty);
static void update_seg7(uint8_t duty);
static uint16_t map_adc_to_temp_f(uint16_t adc_value);
static uint16_t read_actual_temp_f(bool demo_mode);
static void set_fan_duty(uint8_t duty);
static void alarm_update(uint16_t actual_f, uint16_t target_f);

//-----------------------------------------------------------------------------
// Define symbolic constants used by the program
//-----------------------------------------------------------------------------
#define BAUD_RATE              (115200)

#define ADC_CH_THERMISTOR      (0x05)
#define ADC_CH_POT             (0x07)

#define CONTROL_PERIOD_MS      (100)
#define START_DELAY            (100)

#define TEMP_MIN_F             (60)
#define TEMP_MAX_F             (90)
#define TEMP_ALARM_DELTA_F     (10)

#define FAN_ERROR_MAX_F        (30)
#define FAN_MIN_DUTY           (20)
#define FAN_PWM_FREQ_HZ        (4000)

#define NEWLINE                "\r\n"

// Assumes clock_init_40mhz() sets BUSCLK to 40 MHz
#define CPU_CLOCK_HZ           (40000000UL)

// PB22 motor pin and PB27 buzzer pin from LaunchPad.h
#define FAN_PIN_MASK           (LP_RGB_BLU_MASK)   // PB22
#define FAN_PIN_IOMUX          (LP_RGB_BLU_IOMUX)

#define BUZZER_PIN_MASK        (LP_RGB_GRN_MASK)   // PB27
#define BUZZER_PIN_IOMUX       (LP_RGB_GRN_IOMUX)

// 7-segment digit codes
static const uint8_t seg7_digit_code[10] = {
  0x3F,  // 0
  0x06,  // 1
  0x5B,  // 2
  0x4F,  // 3
  0x66,  // 4
  0x6D,  // 5
  0x7D,  // 6
  0x07,  // 7
  0x7F,  // 8
  0x6F   // 9
};

//-----------------------------------------------------------------------------
// Define global variables and structures here.
// NOTE: when possible avoid using global variables
//-----------------------------------------------------------------------------
bool pb1_pressed = false;
bool pb2_pressed = false;

//-----------------------------------------------------------------------------
// Software PWM state for PB22 motor
//-----------------------------------------------------------------------------
static volatile uint32_t g_fan_period_cycles = 0;
static volatile uint32_t g_fan_on_cycles = 0;
static volatile uint32_t g_fan_off_cycles = 0;
static volatile bool g_fan_phase_high = false;
static volatile uint8_t g_fan_duty = 0;

//-----------------------------------------------------------------------------
// DESCRIPTION:
//   Drives the PB22 fan pin HIGH (sets the output bit).
//
// INPUT PARAMETERS:
//   none
//
// OUTPUT PARAMETERS:
//   none
//
// RETURN:
//   none
//-----------------------------------------------------------------------------
static inline void fan_pin_high(void) {
  GPIOB->DOUT31_0 |= FAN_PIN_MASK;
}

//-----------------------------------------------------------------------------
// DESCRIPTION:
//   Drives the PB22 fan pin LOW (clears the output bit).
//
// INPUT PARAMETERS:
//   none
//
// OUTPUT PARAMETERS:
//   none
//
// RETURN:
//   none
//-----------------------------------------------------------------------------
static inline void fan_pin_low(void) {
  GPIOB->DOUT31_0 &= ~FAN_PIN_MASK;
}

//-----------------------------------------------------------------------------
// DESCRIPTION:
//   Disables SysTick and clears its current value.
//
// INPUT PARAMETERS:
//   none
//
// OUTPUT PARAMETERS:
//   none
//
// RETURN:
//   none
//-----------------------------------------------------------------------------
static inline void systick_stop(void) {
  SysTick->CTRL = 0;
  SysTick->VAL = 0;
}

//-----------------------------------------------------------------------------
// DESCRIPTION:
//   Starts SysTick using the provided cycle count. Ensures the load value
//   is never set below 1 cycle.
//
// INPUT PARAMETERS:
//   cycles  - number of CPU cycles until the next SysTick interrupt
//
// OUTPUT PARAMETERS:
//   none
//
// RETURN:
//   none
//-----------------------------------------------------------------------------
static inline void systick_start(uint32_t cycles) {
  if (cycles < 1) {
    cycles = 1;
  }
  SysTick->LOAD = (cycles - 1) & 0x00FFFFFF;
  SysTick->VAL = 0;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                  SysTick_CTRL_TICKINT_Msk   |
                  SysTick_CTRL_ENABLE_Msk;
}

//-----------------------------------------------------------------------------
// DESCRIPTION:
//   SysTick ISR used to implement software PWM on PB22. Toggles the fan
//   output HIGH and LOW based on the current duty cycle settings.
//
// INPUT PARAMETERS:
//   none
//
// OUTPUT PARAMETERS:
//   none
//
// RETURN:
//   none
//-----------------------------------------------------------------------------
void SysTick_Handler(void) {
  if (g_fan_duty == 0) {
    fan_pin_low();
    systick_stop();
    return;
  }

  if (g_fan_duty >= 100) {
    fan_pin_high();
    systick_stop();
    return;
  }

  if (g_fan_phase_high) {
    fan_pin_low();
    g_fan_phase_high = false;
    systick_start(g_fan_off_cycles);
  } else {
    fan_pin_high();
    g_fan_phase_high = true;
    systick_start(g_fan_on_cycles);
  }
}

//-----------------------------------------------------------------------------
// DESCRIPTION:
//   Program entry point. Initializes board peripherals, LCD, UART, ADC, SPI,
//   buzzer (PB27), and software PWM fan output (PB22). Then runs the main
//   smart thermostat loop.
//
// INPUT PARAMETERS:
//   none
//
// OUTPUT PARAMETERS:
//   none
//
// RETURN:
//   int - never returns (loops forever)
//-----------------------------------------------------------------------------
int main(void) {
  clock_init_40mhz();
  launchpad_gpio_init();

  lp_leds_init();
  dipsw_init();
  seg7_init();
  seg7_off();
  leds_init();
  leds_disable();
  I2C_init();
  lcd1602_init();
  lcd_clear();
  UART_init(BAUD_RATE);

  ADC0_init(ADC12_MEMCTL_VRSEL_VDDA_VSSA);

  // PB27 buzzer and PB22 motor
  buzzer_init();
  fan_init_pb22();
  fan_pwm_init_pb22(FAN_PWM_FREQ_HZ);
  fan_set_pwm_dc_pb22(0);

  spi1_init();
  IOMUX->SECCFG.PINCM[LP_SPI_CS0_IOMUX] =
      (IOMUX_PINCM_PC_CONNECTED | PINCM_GPIO_PIN_FUNC);
  GPIOB->DOE31_0 |= LP_SPI_CS0_MASK;
  GPIOB->DOUT31_0 &= ~LP_SPI_CS0_MASK;

  msec_delay(START_DELAY);
  config_pb1_interrupt();
  config_pb2_interrupt();

  run_smart_thermostat();

  while (1) {
  }
} /* main */

//-----------------------------------------------------------------------------
// DESCRIPTION:
//   Main application loop. Reads target and actual temperature (demo or real),
//   computes a fan duty based on error, updates PWM output, LED bar, 7-seg,
//   LCD, and buzzer alarm. PB2 toggles running state. PB1 toggles demo mode.
//
// INPUT PARAMETERS:
//   none
//
// OUTPUT PARAMETERS:
//   none
//
// RETURN:
//   none
//-----------------------------------------------------------------------------
void run_smart_thermostat(void) {
  bool running = false;
  bool demo_mode = false;

  uint16_t target_temp_f = 72;
  uint16_t actual_temp_f = 72;
  uint8_t fan_duty = 0;

  lcd_clear();
  uart_write_string("Smart Thermostat Fan" NEWLINE);
  uart_write_string("PB2 = Start/Stop, PB1 = Demo/Real" NEWLINE);

  while (1) {
    if (pb2_pressed) {
      running = !running;
      pb2_pressed = false;
    }

    if (pb1_pressed) {
      demo_mode = !demo_mode;
      pb1_pressed = false;

      if (demo_mode) {
        uart_write_string("Demo temperature input enabled." NEWLINE);
      } else {
        uart_write_string("Real thermistor temperature enabled." NEWLINE);
      }
    }

    if (!demo_mode) {
      uint16_t adc_pot = ADC0_in(ADC_CH_POT);
      target_temp_f = map_adc_to_temp_f(adc_pot);
    }

    actual_temp_f = read_actual_temp_f(demo_mode);

    if (running) {
      int16_t error_f = (int16_t)actual_temp_f - (int16_t)target_temp_f;

      if (error_f <= 0) {
        fan_duty = 0;
      } else {
        if (error_f > FAN_ERROR_MAX_F) {
          error_f = FAN_ERROR_MAX_F;
        }
        fan_duty = (uint8_t)((error_f * 100) / FAN_ERROR_MAX_F);
        if (fan_duty < FAN_MIN_DUTY) {
          fan_duty = FAN_MIN_DUTY;
        }
      }
    } else {
      fan_duty = 0;
    }

    set_fan_duty(fan_duty);
    update_led_bar(fan_duty);
    update_seg7(fan_duty);
    update_lcd(actual_temp_f, target_temp_f, fan_duty, running, demo_mode);
    alarm_update(actual_temp_f, target_temp_f);

    msec_delay(CONTROL_PERIOD_MS);
  }
}

//-----------------------------------------------------------------------------
// DESCRIPTION:
//   Sends a NUL-terminated C string over UART0, one character at a time.
//
// INPUT PARAMETERS:
//   s  - pointer to NUL-terminated string
//
// OUTPUT PARAMETERS:
//   none
//
// RETURN:
//   none
//-----------------------------------------------------------------------------
static void uart_write_string(const char *s) {
  while (*s) {
    UART_out_char(*s++);
  }
}

//-----------------------------------------------------------------------------
// DESCRIPTION:
//   Converts an 8-bit value to a 3-character, right aligned ASCII string
//   with spaces for leading blanks. Buffer must be at least 4 bytes.
//
// INPUT PARAMETERS:
//   value - 0 to 255
//   buf   - output buffer, 4 bytes (3 chars + NUL)
//
// OUTPUT PARAMETERS:
//   buf filled with "  0"..."255"
//
// RETURN:
//   none
//-----------------------------------------------------------------------------
static void byte_to_ascii(uint8_t value, char buf[4]) {
  if (value >= 100) {
    buf[0] = '0' + (value / 100);
  } else {
    buf[0] = ' ';
  }

  if (value >= 10) {
    buf[1] = '0' + ((value / 10) % 10);
  } else {
    buf[1] = ' ';
  }

  buf[2] = '0' + (value % 10);
  buf[3] = '\0';
}

//-----------------------------------------------------------------------------
// DESCRIPTION:
//   Maps a 12-bit ADC reading (0-4095) into a temperature in Fahrenheit using
//   a linear scale from TEMP_MIN_F to TEMP_MAX_F.
//
// INPUT PARAMETERS:
//   adc_value  - raw 12-bit ADC reading
//
// OUTPUT PARAMETERS:
//   none
//
// RETURN:
//   uint16_t - mapped temperature in degrees F
//-----------------------------------------------------------------------------
static uint16_t map_adc_to_temp_f(uint16_t adc_value) {
  uint32_t span = TEMP_MAX_F - TEMP_MIN_F;
  uint32_t temp = TEMP_MIN_F + ((uint32_t)adc_value * span) / 4095;
  return (uint16_t)temp;
}

//-----------------------------------------------------------------------------
// DESCRIPTION:
//   Returns the actual temperature in Fahrenheit. In demo mode, the pot is
//   used as a fake temperature input. In real mode, the thermistor ADC is
//   converted to Celsius by thermistor_calc_temperature and then to Fahrenheit.
//
// INPUT PARAMETERS:
//   demo_mode  - true uses pot input, false uses thermistor input
//
// OUTPUT PARAMETERS:
//   none
//
// RETURN:
//   uint16_t - actual temperature in degrees F
//-----------------------------------------------------------------------------
static uint16_t read_actual_temp_f(bool demo_mode) {
  if (demo_mode) {
    uint16_t adc_demo = ADC0_in(ADC_CH_POT);
    return map_adc_to_temp_f(adc_demo);
  } else {
    uint16_t adc_temp = ADC0_in(ADC_CH_THERMISTOR);
    uint16_t celsius_temp = (uint16_t)thermistor_calc_temperature(adc_temp);
    uint16_t fahrenheit_temp = (uint16_t)((celsius_temp * 9) / 5) + 32;
    return fahrenheit_temp;
  }
}

//-----------------------------------------------------------------------------
// DESCRIPTION:
//   Updates the LCD1602 with current temperature, target temperature, duty,
//   and a mode/status label.
//
// INPUT PARAMETERS:
//   actual_f   - current temperature in degrees F
//   target_f   - target temperature in degrees F
//   duty       - fan duty percent (0-100)
//   running    - true if fan control is active
//   demo_mode  - true if using demo input
//
// OUTPUT PARAMETERS:
//   none
//
// RETURN:
//   none
//-----------------------------------------------------------------------------
static void update_lcd(uint16_t actual_f,
                       uint16_t target_f,
                       uint8_t duty,
                       bool running,
                       bool demo_mode)
{
  char buf3[4] = "   ";

  lcd_set_ddram_addr(LCD_LINE1_ADDR);
  lcd_write_string("CUR=");

  byte_to_ascii((uint8_t)actual_f, buf3);
  lcd_write_string(buf3);
  lcd_write_char('F');
  lcd_write_char(' ');

  lcd_write_string("AIM=");
  byte_to_ascii((uint8_t)target_f, buf3);
  lcd_write_string(buf3);
  lcd_write_char('F');

  lcd_set_ddram_addr(LCD_LINE2_ADDR);
  lcd_write_string("DUTY=");

  byte_to_ascii(duty, buf3);
  lcd_write_string(buf3);
  lcd_write_char(' ');

  if (running) {
    if (demo_mode) {
      lcd_write_string("DEMO ON");
    } else {
      lcd_write_string("RUN  ON");
    }
  } else {
    if (demo_mode) {
      lcd_write_string("DEMOOFF");
    } else {
      lcd_write_string("STOP   ");
    }
  }
}

//-----------------------------------------------------------------------------
// DESCRIPTION:
//   Updates the LED bar via SPI to show duty as a simple bar graph.
//
// INPUT PARAMETERS:
//   duty  - fan duty percent (0-100)
//
// OUTPUT PARAMETERS:
//   none
//
// RETURN:
//   none
//-----------------------------------------------------------------------------
static void update_led_bar(uint8_t duty) {
  uint8_t level;
  uint8_t pattern = 0;

  leds_off();
  if (duty >= 100) {
    level = 8;
  } else {
    level = (uint8_t)((duty + 11) / 12);
  }

  for (uint8_t i = 0; i < level; i++) {
    pattern |= (1 << i);
    leds_on(pattern);
  }

  spi1_write_data(pattern);
  (void)spi1_read_data();
}

//-----------------------------------------------------------------------------
// DESCRIPTION:
//   Updates the two-digit seven-segment display to show duty percent (00-99).
//
// INPUT PARAMETERS:
//   duty  - fan duty percent (0-100), clamped to 99 for display
//
// OUTPUT PARAMETERS:
//   none
//
// RETURN:
//   none
//-----------------------------------------------------------------------------
static void update_seg7(uint8_t duty) {
  if (duty > 99) {
    duty = 99;
  }

  uint8_t tens = duty / 10;
  uint8_t ones = duty % 10;

  seg7_on(seg7_digit_code[ones], SEG7_DIG0_ENABLE_IDX);
  seg7_on(seg7_digit_code[tens], SEG7_DIG1_ENABLE_IDX);
}

//-----------------------------------------------------------------------------
// DESCRIPTION:
//   Wrapper used by the application to set fan duty. Uses PB22 software PWM.
//
// INPUT PARAMETERS:
//   duty  - fan duty percent (0-100)
//
// OUTPUT PARAMETERS:
//   none
//
// RETURN:
//   none
//-----------------------------------------------------------------------------
static void set_fan_duty(uint8_t duty) {
  fan_set_pwm_dc_pb22(duty);
}

//-----------------------------------------------------------------------------
// DESCRIPTION:
//   Initializes PB27 as a GPIO output for the active buzzer and ensures it
//   starts in the OFF state.
//
// INPUT PARAMETERS:
//   none
//
// OUTPUT PARAMETERS:
//   none
//
// RETURN:
//   none
//-----------------------------------------------------------------------------
void buzzer_init(void) {
  uint32_t gpio_pincm = IOMUX_PINCM_PC_CONNECTED | PINCM_GPIO_PIN_FUNC;

  IOMUX->SECCFG.PINCM[BUZZER_PIN_IOMUX] = gpio_pincm;
  GPIOB->DOE31_0 |= BUZZER_PIN_MASK;

  // start OFF
  GPIOB->DOUT31_0 &= ~BUZZER_PIN_MASK;
}

//-----------------------------------------------------------------------------
// DESCRIPTION:
//   Turns the buzzer ON by driving PB27 HIGH.
//
// INPUT PARAMETERS:
//   none
//
// OUTPUT PARAMETERS:
//   none
//
// RETURN:
//   none
//-----------------------------------------------------------------------------
static inline void buzzer_on(void) {
  GPIOB->DOUT31_0 |= BUZZER_PIN_MASK;   // PB27 HIGH
}

//-----------------------------------------------------------------------------
// DESCRIPTION:
//   Turns the buzzer OFF by driving PB27 LOW.
//
// INPUT PARAMETERS:
//   none
//
// OUTPUT PARAMETERS:
//   none
//
// RETURN:
//   none
//-----------------------------------------------------------------------------
static inline void buzzer_off(void) {
  GPIOB->DOUT31_0 &= ~BUZZER_PIN_MASK;  // PB27 LOW
}

//-----------------------------------------------------------------------------
// DESCRIPTION:
//   Initializes PB22 as a GPIO output for software PWM fan control and drives
//   it LOW initially.
//
// INPUT PARAMETERS:
//   none
//
// OUTPUT PARAMETERS:
//   none
//
// RETURN:
//   none
//-----------------------------------------------------------------------------
void fan_init_pb22(void) {
  uint32_t gpio_pincm = IOMUX_PINCM_PC_CONNECTED | PINCM_GPIO_PIN_FUNC;

  IOMUX->SECCFG.PINCM[FAN_PIN_IOMUX] = gpio_pincm;
  GPIOB->DOE31_0 |= FAN_PIN_MASK;

  fan_pin_low();
}

//-----------------------------------------------------------------------------
// DESCRIPTION:
//   Initializes software PWM timing values for PB22 based on the requested PWM
//   frequency and the assumed CPU clock.
//
// INPUT PARAMETERS:
//   pwm_freq_hz  - desired PWM frequency in Hz
//
// OUTPUT PARAMETERS:
//   none
//
// RETURN:
//   none
//-----------------------------------------------------------------------------
void fan_pwm_init_pb22(uint32_t pwm_freq_hz) {
  if (pwm_freq_hz == 0) {
    pwm_freq_hz = 1;
  }

  g_fan_period_cycles = CPU_CLOCK_HZ / pwm_freq_hz;
  if (g_fan_period_cycles < 2) {
    g_fan_period_cycles = 2;
  }

  g_fan_duty = 0;
  g_fan_on_cycles = 1;
  g_fan_off_cycles = g_fan_period_cycles - 1;
  g_fan_phase_high = false;

  systick_stop();
  fan_pin_low();
}

//-----------------------------------------------------------------------------
// DESCRIPTION:
//   Sets the PB22 software PWM duty cycle (0-100). Updates on and off cycle
//   counts and restarts SysTick so the PWM begins on a HIGH phase.
//
// INPUT PARAMETERS:
//   duty  - duty percent (0-100)
//
// OUTPUT PARAMETERS:
//   none
//
// RETURN:
//   none
//-----------------------------------------------------------------------------
void fan_set_pwm_dc_pb22(uint8_t duty) {
  if (duty > 100) {
    duty = 100;
  }

  g_fan_duty = duty;

  if (duty == 0) {
    systick_stop();
    fan_pin_low();
    g_fan_phase_high = false;
    return;
  }

  if (duty >= 100) {
    systick_stop();
    fan_pin_high();
    g_fan_phase_high = true;
    return;
  }

  uint32_t on_cycles = (g_fan_period_cycles * (uint32_t)duty) / 100;
  if (on_cycles < 1) {
    on_cycles = 1;
  }
  if (on_cycles >= g_fan_period_cycles) {
    on_cycles = g_fan_period_cycles - 1;
  }

  g_fan_on_cycles = on_cycles;
  g_fan_off_cycles = g_fan_period_cycles - on_cycles;

  // restart PWM cleanly on a HIGH phase
  g_fan_phase_high = true;
  fan_pin_high();
  systick_start(g_fan_on_cycles);
}

//-----------------------------------------------------------------------------
// DESCRIPTION:
//   Updates the alarm buzzer based on how far actual temperature exceeds the
//   target temperature. If actual is at least TEMP_ALARM_DELTA_F above target,
//   the buzzer is turned ON.
//
// INPUT PARAMETERS:
//   actual_f  - current temperature in degrees F
//   target_f  - target temperature in degrees F
//
// OUTPUT PARAMETERS:
//   none
//
// RETURN:
//   none
//-----------------------------------------------------------------------------
static void alarm_update(uint16_t actual_f, uint16_t target_f) {
  if (actual_f >= target_f + TEMP_ALARM_DELTA_F) {
    buzzer_on();
  } else {
    buzzer_off();
  }
}

//-----------------------------------------------------------------------------
// DESCRIPTION:
//   Configures PB1 interrupt (DIO18 on GPIOB) for rising edge detection.
//   Enables the interrupt in the NVIC.
//
// INPUT PARAMETERS:
//   none
//
// OUTPUT PARAMETERS:
//   none
//
// RETURN:
//   none
//-----------------------------------------------------------------------------
void config_pb1_interrupt(void) {
  GPIOB->POLARITY31_16 = GPIO_POLARITY31_16_DIO18_RISE;
  GPIOB->CPU_INT.ICLR = GPIO_CPU_INT_ICLR_DIO18_CLR;
  GPIOB->CPU_INT.IMASK = GPIO_CPU_INT_IMASK_DIO18_SET;

  NVIC_SetPriority(GPIOB_INT_IRQn, 2);
  NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

//-----------------------------------------------------------------------------
// DESCRIPTION:
//   Configures PB2 interrupt (DIO15 on GPIOA) for rising edge detection.
//   Enables the interrupt in the NVIC.
//
// INPUT PARAMETERS:
//   none
//
// OUTPUT PARAMETERS:
//   none
//
// RETURN:
//   none
//-----------------------------------------------------------------------------
void config_pb2_interrupt(void) {
  GPIOA->POLARITY15_0 = GPIO_POLARITY15_0_DIO15_RISE;
  GPIOA->CPU_INT.ICLR = GPIO_CPU_INT_ICLR_DIO15_CLR;
  GPIOA->CPU_INT.IMASK = GPIO_CPU_INT_IMASK_DIO15_SET;

  NVIC_SetPriority(GPIOA_INT_IRQn, 2);
  NVIC_EnableIRQ(GPIOA_INT_IRQn);
}

//-----------------------------------------------------------------------------
// DESCRIPTION:
//   Group 1 interrupt handler. Checks which GPIO interrupt fired and sets the
//   corresponding global press flag, then clears the interrupt.
//
// INPUT PARAMETERS:
//   none
//
// OUTPUT PARAMETERS:
//   pb1_pressed and pb2_pressed are updated based on the interrupt source
//
// RETURN:
//   none
//-----------------------------------------------------------------------------
void GROUP1_IRQHandler(void) {
  uint32_t group_gpio_iidx;
  uint32_t gpio_mis;

  do {
    group_gpio_iidx = CPUSS->INT_GROUP[1].IIDX;

    switch (group_gpio_iidx) {
      case (CPUSS_INT_GROUP_IIDX_STAT_INT0):
        gpio_mis = GPIOA->CPU_INT.MIS;

        if ((gpio_mis & GPIO_CPU_INT_MIS_DIO15_MASK) == GPIO_CPU_INT_MIS_DIO15_SET) {
          pb2_pressed = true;
          GPIOA->CPU_INT.ICLR = GPIO_CPU_INT_ICLR_DIO15_CLR;
        }
        break;

      case (CPUSS_INT_GROUP_IIDX_STAT_INT1):
        gpio_mis = GPIOB->CPU_INT.MIS;

        if ((gpio_mis & GPIO_CPU_INT_MIS_DIO18_MASK) == GPIO_CPU_INT_MIS_DIO18_SET) {
          pb1_pressed = true;
          GPIOB->CPU_INT.ICLR = GPIO_CPU_INT_ICLR_DIO18_CLR;
        }
        break;

      default:
        break;
    }
  } while (group_gpio_iidx != 0);
}
