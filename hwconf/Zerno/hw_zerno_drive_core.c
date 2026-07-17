/*
 *  Copyright 2022 Benjamin Vedder	benjamin@vedder.se
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "hw.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "utils_math.h"
#include "mc_interface.h"
#include "terminal.h"
#include "commands.h"
#include "spi.h"
#include "spi_bb.h"
#include "timeout.h"
#include "mempools.h"
#include "mcpwm_foc.h"
#include "main.h"
#include "app.h"

#include <string.h>
#include <math.h>
#include <stdlib.h>

#define EEPROM_ADDR_ENCODER_VALUE                 (2)
#define EEPROM_ADDR_CALIBRATION_CHECK             (6)
#define EEPROM_ADDR_MIN_CALIBRATED_VALUE          (8)
#define EEPROM_ADDR_STEPS_VALUE                   (10)
#define EEPROM_ADDR_ADC_MAX_VALUE                 (12)
#define EEPROM_ADDR_GRIND_TH_VALUE                (14)
#define EEPROM_ADDR_NO_GRIND_TH_VALUE             (16)
#define CURRENT_MOTOR_TIMEOUT_MS                  (250)
#define OVERLOAD_CLEAR_DELAY_MS                   (250)
#define MOTOR_SELECTED                            (2)
#define GRIND_TIMEOUT_SEC                         (600)
#define CUTOFF_CURRENT_AMPS                       (4.0f)
#define NO_GRIND_CURRENT_DEFAULT_AMPS             (0.7f)
#define NO_GRIND_TH_AUTO_MARGIN_AMPS              (0.15f)
#define OFFSET_FACTOR_CORRECTION                  (0.05f)
#define SPEED_MIN_ERPM                            (800.0f)
#define SPEED_ERPM_STEP                           (200.0f)
#define SPEED_ERPM_PID_CHANGE                     (1600)
#define MAX_ADC_VALUE_IN_VOLTS                    (3.27f)
#define MAX_ENCODER_VALUE_IN_VOLTS                (2.9f)
#define THRESHOLD_VALUE                           (0.05f)
#define DEFAULT_VALUE                             (0.0f)
#define KNOB_STEPS                                (13)
#define MIN_KNOB_INDEX                            (0.0f)
#define MAX_KNOB_INDEX                            (13.0f)
#define SWITCH_STOP_POSITION_IN_VOLTS             (2.8f)
#define SWITCH_ON_POSITION_1_IN_VOLTS             (1.2f)
#define SWITCH_ON_POSITION_2_IN_VOLTS             (1.6f)
#define SWITCH_MOMENTARY_POSITION_IN_VOLTS        (0.4f)
#define SPEED_ERPM_MOMENTARY                      (8000)
#define NTC_BETA_PARAMETER                        (3455.0f)
#define NTC_RESISTANCE_VALUE                      (10000)
#define NTC_TEMP_1_REL                            (1.0f / 298.15f)
#define NTC_TEMP_2                                (273.15f)
#define UNIT_CONSTANT                             (1.0f)
#define TEMP_FILTER_CONSTANT                      (0.1f)
#define ADC_FILTER_CONSTANT                       (0.01f)
#define SWITCH_FILTER_CONSTANT                    (0.1f)
#define ZERO_VECTOR_FREQ                          (10000.0f)
#define FOC_KP_CONSTANT                           (0.01f)
#define FOC_KI_CONSTANT                           (10.0f)
#define CALIBRATION_CURRENT                       (2.0f)
#define CALIBRATION_RATIO_VALUE                   (0.0f)
#define CALIBRATION_OFFSET_VALUE                  (0.0f)
#define SPEED_PID_KP_LOW                          (0.008f)
#define SPEED_PID_KP_HIGH                         (0.008f)
#define SPEED_ERPM_RAMP_HIGH                      (10000.0f)
#define SPEED_ERPM_RAMP_LOW                       (5000.0f)
#define SYSTICK_ZERO_VALUE                        (0.0f)
#define SPEED_THREAD_STACK_SIZE                   (1024)
#define ENCODER_THREAD_STACK_SIZE                 (1024)
#define SAMPLES                                   (15)
#define ADC_RANK_SEQUENCER_1                      (1)
#define ADC_RANK_SEQUENCER_2                      (2)
#define ADC_RANK_SEQUENCER_3                      (3)
#define ADC_RANK_SEQUENCER_4                      (4)
#define ADC_RANK_SEQUENCER_5                      (5)
#define ADC_RANK_SEQUENCER_6                      (6)
#define PIN_0                                     (0)
#define PIN_1                                     (1)
#define PIN_2                                     (2)
#define PIN_3                                     (3)
#define PIN_4                                     (4)
#define PIN_5                                     (5)
#define PIN_6                                     (6)
#define PIN_7                                     (7)
#define PIN_8                                     (8)
#define PIN_9                                     (9)
#define PIN_10                                    (10)
#define PIN_11                                    (11)
#define PIN_12                                    (12)
#define PIN_13                                    (13)
#define PIN_14                                    (14)
#define PIN_15                                    (15)
#define GRIND_CURRENT_DEFAULT_TH_AMPS             (1.0f) // this would be set to 2.5A
#define GRIND_CURRENT_HYSTERESIS_AMPS             (0.3f)
#define GRIND_PID_DEFAULT_KP_MULTIPLIER           (4.0f)
#define GRIND_ENGAGE_DELAY_MS                     (200)
#define GRIND_RELEASE_DELAY_MS                    (200)
#define GRIND_PID_RAMP_STEPS                      (30) // 30 × 10 ms loop = 300 ms transition
#define GRIND_CURRENT_FILTER_CONSTANT             (0.05f) // smooths current ripple so it doesn't chatter across the engage/release thresholds
#define GRIND_TH_AUTO_MARGIN_AMPS                 (0.4f)
#define GRIND_TH_SETTLE_MS                        (300)
#define GRIND_TH_SAMPLE_COUNT                     (3)
#define GRIND_TH_SAMPLE_INTERVAL_MS               (150)
#define GRIND_TH_SANITY_CEILING_AMPS              (2.0f)
#define STOP_BEEP_CHANNEL                         (0)
#define STOP_BEEP_FREQ_HZ                         (2000.0f)
#define STOP_BEEP_VOLTAGE_DEFAULT                 (80.2f)
#define STOP_BEEP_VOLTAGE_MAX                     (120.0f)
#define STOP_BEEP_TONE_MS                         (150)
#define STOP_BEEP_GAP_MS                          (150)

static THD_FUNCTION(speed_thread, arg);
static THD_FUNCTION(encoder_thread, arg);

static THD_WORKING_AREA(speed_thread_wa, SPEED_THREAD_STACK_SIZE);
static THD_WORKING_AREA(encoder_thread_wa, ENCODER_THREAD_STACK_SIZE);

static volatile float encoder_min_value_in_volts;
static volatile float encoder_total_value_volts;
static volatile float main_switch_value_in_volts;
static volatile float speed_erpm_setpoint;
static volatile float knob_read_in_volts;
static volatile float encoder_min_calibrated_value;
static volatile float steps;

static float encoder_calibrated_value_in_volts;
static float get_maximum_adc_value_in_volts;
static float knob_index;
static float read_buffer;

static volatile bool safety_calibration = false;
static volatile bool is_erpm_done = false;
static volatile bool is_momentary_position_status = false;
static volatile bool store_minimum_value = false;
static volatile bool is_encoder_done = false;
static volatile bool i2c_running = false;

static bool is_stop_state = false;
static bool is_motor_grinding_enable = true;
static bool is_in_maximum_detection = false;
static bool change_erpm_ramp_pid_on_state = false;
static bool change_erpm_ramp_pid_mom_state = false;

static float grind_current_th_amps = GRIND_CURRENT_DEFAULT_TH_AMPS;
static float no_grind_current_amps = NO_GRIND_CURRENT_DEFAULT_AMPS;
static bool grind_th_manual_override = false;
static float grind_kp_multiplier = GRIND_PID_DEFAULT_KP_MULTIPLIER;
static float stop_beep_voltage = STOP_BEEP_VOLTAGE_DEFAULT;
static float grind_original_kp = -1.0f;
static float grind_original_kd = -1.0f;
static float grind_original_ki = -1.0f;
static bool grind_pid_active = false;
static float grind_kp_ramp_from = 0.0f;
static float grind_kp_ramp_to = 0.0f;
static float grind_kd_ramp_from = 0.0f;
static float grind_kd_ramp_to = 0.0f;
static int grind_ramp_step = -1;
static bool grind_ramp_is_restore = false;
static float grind_current_filtered = 0.0f;

static uint8_t is_calibration_done = 0;
static uint8_t head = 0;
static uint8_t tail = 0;
static uint8_t circular_counter = 0;
static float circular_buffer_in_volts[SAMPLES];
static float switch_positions_in_volts;

static void adc_read_callback(void);
static void define_default_values(void);
static void knob_encoder_calibrate_offset(void);
static void adc_get_maximum_value(void);
static void write_adc_value_in_volts(void);
static void read_adc_value_in_volts(void);


static bool is_pfc_ok(void);
void hw_zerno_configure_brownout(uint8_t BOR_level);

static void terminal_print_info(int argc, const char** argv);
static void enable_grind_pid(void);
static void disable_grind_pid(void);
static void reset_grind_pid_instant(void);
static void start_grind_pid_ramp(float kp_to, float kd_to);
static void grind_pid_ramp_step(void);
static void terminal_set_grind_pid(int argc, const char** argv);
static void terminal_get_grind_pid(int argc, const char** argv);
static void terminal_set_beep_volume(int argc, const char** argv);
static void terminal_get_beep_volume(int argc, const char** argv);

static const float erpm_lut[14] = {
	1000.0,
	1200.0,
	1600.0,
	2000.0,
	2400.0,
	2800.0,
	3200.0,
	3600.0,
	4000.0,
	4800.0,
	5600.0,
	6400.0,
	7200.0,
	8000.0,
};


// I2C configuration
static const I2CConfig i2cfg = {
	OPMODE_I2C,
	100000,
	STD_DUTY_CYCLE
};

void hw_init_gpio(void) {
	hw_zerno_configure_brownout(OB_BOR_LEVEL3);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	palSetPadMode(LED_GREEN_GPIO, LED_GREEN_PIN,
				  PAL_MODE_OUTPUT_PUSHPULL |
				  PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(LED_RED_GPIO, LED_RED_PIN,
				  PAL_MODE_OUTPUT_PUSHPULL |
				  PAL_STM32_OSPEED_HIGHEST);

	// GPIOA Configuration: Channel 1 to 3 as alternate function push-pull
	palSetPadMode(GPIOA, PIN_8, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
				  PAL_STM32_OSPEED_HIGHEST |
				  PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, PIN_9, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
				  PAL_STM32_OSPEED_HIGHEST |
				  PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, PIN_10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
				  PAL_STM32_OSPEED_HIGHEST |
				  PAL_STM32_PUDR_FLOATING);

	palSetPadMode(GPIOB, PIN_13, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
				  PAL_STM32_OSPEED_HIGHEST |
				  PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, PIN_14, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
				  PAL_STM32_OSPEED_HIGHEST |
				  PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, PIN_15, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
				  PAL_STM32_OSPEED_HIGHEST |
				  PAL_STM32_PUDR_FLOATING);


	// Hall sensors
	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT);
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT);
	palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_INPUT);

	// Phase filters

	palSetPadMode(PHASE_FILTER_GPIO, PHASE_FILTER_PIN,
				  PAL_MODE_OUTPUT_PUSHPULL |
				  PAL_STM32_OSPEED_HIGHEST);
	PHASE_FILTER_OFF();

	// Sensor port voltage
	SENSOR_PORT_3V3();
	palSetPadMode(SENSOR_VOLTAGE_GPIO, SENSOR_VOLTAGE_PIN,
				  PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

	// ADC Pins
	palSetPadMode(GPIOA, PIN_0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, PIN_1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, PIN_2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, PIN_3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, PIN_5, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, PIN_6, PAL_MODE_INPUT_ANALOG);

	// Switch input pins
	palSetPadMode(HW_SW_PORT, HW_SW_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_MOMENTARY_PORT, HW_MOMENTARY_PIN, PAL_MODE_INPUT_PULLUP);

	// PFC interface signals
	palSetPadMode(PFC_STATUS_PORT, PFC_STATUS_PIN, PAL_MODE_INPUT);
	palSetPadMode(PFC_ENABLE_PORT, PFC_ENABLE_PIN, PAL_MODE_OUTPUT_PUSHPULL);

	palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_ANALOG);

	palSetPadMode(GPIOC, PIN_0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, PIN_1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, PIN_2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, PIN_3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, PIN_4, PAL_MODE_INPUT_ANALOG);

	palSetPadMode(GPIOA, PIN_4, PAL_MODE_INPUT_ANALOG);

	mc_interface_set_pwm_callback(adc_read_callback);

	define_default_values();

	chThdCreateStatic(speed_thread_wa, sizeof(speed_thread_wa), NORMALPRIO, speed_thread, NULL);

	chThdCreateStatic(encoder_thread_wa, sizeof(encoder_thread_wa), NORMALPRIO, encoder_thread, NULL);

	terminal_register_command_callback(
		"encoder_status",
		"Value",
		0,
		terminal_print_info);

	terminal_register_command_callback(
		"set_grind_pid",
		"Set grinding PID params: set_grind_pid <current_th_amps> <kp_multiplier>",
		"[current_th] [kp_mult]",
		terminal_set_grind_pid);

	terminal_register_command_callback(
		"get_grind_pid",
		"Print current grinding PID parameters and state",
		0,
		terminal_get_grind_pid);

	terminal_register_command_callback(
		"set_beep_volume",
		"Set the stop-mode beep volume: set_beep_volume <volume>",
		"[volume]",
		terminal_set_beep_volume);

	terminal_register_command_callback(
		"get_beep_volume",
		"Print current stop-mode beep volume",
		0,
		terminal_get_beep_volume);
}

void hw_setup_adc_channels(void) {
	// ADC1 regular channels														// index
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, ADC_RANK_SEQUENCER_1, ADC_SampleTime_15Cycles); // 0
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, ADC_RANK_SEQUENCER_2, ADC_SampleTime_15Cycles); // 3
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, ADC_RANK_SEQUENCER_3, ADC_SampleTime_15Cycles); // 6
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, ADC_RANK_SEQUENCER_4, ADC_SampleTime_15Cycles); // 9 TEMP MOTOR
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, ADC_RANK_SEQUENCER_5, ADC_SampleTime_15Cycles); // 12
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, ADC_RANK_SEQUENCER_6, ADC_SampleTime_15Cycles); // 15 PA4 PFC temperature.

	// ADC2 regular channels
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11, ADC_RANK_SEQUENCER_1, ADC_SampleTime_15Cycles); // 1
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, ADC_RANK_SEQUENCER_2, ADC_SampleTime_15Cycles); // 4
	ADC_RegularChannelConfig(ADC2, ADC_Channel_6, ADC_RANK_SEQUENCER_3, ADC_SampleTime_15Cycles); // 7
	ADC_RegularChannelConfig(ADC2, ADC_Channel_0, ADC_RANK_SEQUENCER_5, ADC_SampleTime_15Cycles); // 13
	ADC_RegularChannelConfig(ADC2, ADC_Channel_9, ADC_RANK_SEQUENCER_6, ADC_SampleTime_15Cycles); // 16

	// ADC3 regular channels
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, ADC_RANK_SEQUENCER_1, ADC_SampleTime_15Cycles); // 2
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, ADC_RANK_SEQUENCER_2, ADC_SampleTime_15Cycles); // 5
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3, ADC_RANK_SEQUENCER_3, ADC_SampleTime_15Cycles); // 8
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, ADC_RANK_SEQUENCER_4, ADC_SampleTime_15Cycles); //11
	ADC_RegularChannelConfig(ADC3, ADC_Channel_1, ADC_RANK_SEQUENCER_5, ADC_SampleTime_15Cycles); // 15
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, ADC_RANK_SEQUENCER_6, ADC_SampleTime_15Cycles); // 17

	// Injected channels
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, ADC_RANK_SEQUENCER_1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, ADC_RANK_SEQUENCER_1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, ADC_RANK_SEQUENCER_1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, ADC_RANK_SEQUENCER_2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, ADC_RANK_SEQUENCER_2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, ADC_RANK_SEQUENCER_2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, ADC_RANK_SEQUENCER_3, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, ADC_RANK_SEQUENCER_3, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, ADC_RANK_SEQUENCER_3, ADC_SampleTime_15Cycles);
}

void hw_start_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (!i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
					  PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
					  PAL_STM32_OTYPE_OPENDRAIN |
					  PAL_STM32_OSPEED_MID1 |
					  PAL_STM32_PUDR_PULLUP);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
					  PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
					  PAL_STM32_OTYPE_OPENDRAIN |
					  PAL_STM32_OSPEED_MID1 |
					  PAL_STM32_PUDR_PULLUP);

		i2cStart(&HW_I2C_DEV, &i2cfg);
		i2c_running = true;
	}

	i2cReleaseBus(&HW_I2C_DEV);
}

void hw_stop_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN, PAL_MODE_INPUT);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN, PAL_MODE_INPUT);

		i2cStop(&HW_I2C_DEV);
		i2c_running = false;
	}

	i2cReleaseBus(&HW_I2C_DEV);
}

/**
 * Try to restore the i2c bus
 */
void hw_try_restore_i2c(void) {
	if (i2c_running) {
		i2cAcquireBus(&HW_I2C_DEV);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
					  PAL_STM32_OTYPE_OPENDRAIN |
					  PAL_STM32_OSPEED_MID1 |
					  PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
					  PAL_STM32_OTYPE_OPENDRAIN |
					  PAL_STM32_OSPEED_MID1 |
					  PAL_STM32_PUDR_PULLUP);

		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		chThdSleep(1);

		for (uint8_t i = 0; i < 16; i++) {
			palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
			palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
		}

		// Generate start then stop condition
		palClearPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);
		chThdSleep(1);
		palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
					  PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
					  PAL_STM32_OTYPE_OPENDRAIN |
					  PAL_STM32_OSPEED_MID1 |
					  PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
					  PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
					  PAL_STM32_OTYPE_OPENDRAIN |
					  PAL_STM32_OSPEED_MID1 |
					  PAL_STM32_PUDR_PULLUP);

		HW_I2C_DEV.state = I2C_STOP;
		i2cStart(&HW_I2C_DEV, &i2cfg);

		i2cReleaseBus(&HW_I2C_DEV);
	}
}

bool is_momentary_position(void) {
	return (bool)palReadPad(HW_MOMENTARY_PORT, HW_MOMENTARY_PIN);
}

bool is_sw_position(void) {
	return (bool)palReadPad(HW_SW_PORT, HW_SW_PIN);
}

static bool is_pfc_ok(void) {
	return (bool)palReadPad(PFC_STATUS_PORT, PFC_STATUS_PIN);
}

/* Configure the STM32 option-byte brown-out reset level. The MCU is held
 * under reset until VDD reaches the selected level, which prevents code
 * (and in particular an in-progress EEPROM/flash write) from running on a
 * marginal supply during a brown-out. This is stored in flash option bytes
 * and persists across reprogramming of the application.
 */
void hw_zerno_configure_brownout(uint8_t BOR_level) {
	if ((FLASH_OB_GetBOR() & 0x0C) != BOR_level) {
		/* Get BOR Option Bytes */
		FLASH_OB_Unlock();

		/* Select the desired V(BOR) Level -------------------------------------*/
		FLASH_OB_BORConfig(BOR_level);

		/* Launch the option byte loading */
		FLASH_OB_Launch();

		/* Locks the option bytes block access */
		FLASH_OB_Lock();
	}
}

/* Load the stored values during start-up
 *
 */
static void define_default_values(void) {
	eeprom_var default_offset;
	eeprom_var default_calibration;
	eeprom_var min_calibrated_stored;
	eeprom_var step_stored;
	eeprom_var adc_maximum_value_stored;
	eeprom_var grind_th_stored;
	eeprom_var no_grind_th_stored;

	conf_general_read_eeprom_var_hw(&default_offset, EEPROM_ADDR_ENCODER_VALUE);
	encoder_min_value_in_volts = default_offset.as_float;

	conf_general_read_eeprom_var_hw(&default_calibration, EEPROM_ADDR_CALIBRATION_CHECK);
	is_calibration_done = default_calibration.as_i32;

	conf_general_read_eeprom_var_hw(&min_calibrated_stored, EEPROM_ADDR_MIN_CALIBRATED_VALUE);
	encoder_min_calibrated_value = min_calibrated_stored.as_float;

	conf_general_read_eeprom_var_hw(&step_stored, EEPROM_ADDR_STEPS_VALUE);
	steps = step_stored.as_float;

	conf_general_read_eeprom_var_hw(&adc_maximum_value_stored, EEPROM_ADDR_ADC_MAX_VALUE);
	get_maximum_adc_value_in_volts = adc_maximum_value_stored.as_float;

	grind_th_stored.as_float = GRIND_CURRENT_DEFAULT_TH_AMPS;
	conf_general_read_eeprom_var_hw(&grind_th_stored, EEPROM_ADDR_GRIND_TH_VALUE);
	grind_current_th_amps = (grind_th_stored.as_float > GRIND_CURRENT_DEFAULT_TH_AMPS) ? grind_th_stored.as_float : GRIND_CURRENT_DEFAULT_TH_AMPS;

	no_grind_th_stored.as_float = NO_GRIND_CURRENT_DEFAULT_AMPS;
	conf_general_read_eeprom_var_hw(&no_grind_th_stored, EEPROM_ADDR_NO_GRIND_TH_VALUE);
	no_grind_current_amps = (no_grind_th_stored.as_float > NO_GRIND_CURRENT_DEFAULT_AMPS) ? no_grind_th_stored.as_float : NO_GRIND_CURRENT_DEFAULT_AMPS;
}

static void knob_encoder_calibrate_offset(void) {
	eeprom_var offset_value;
	eeprom_var calibration_check;

	encoder_total_value_volts = knob_read_in_volts;

	if ((switch_positions_in_volts < SWITCH_MOMENTARY_POSITION_IN_VOLTS) && !is_in_maximum_detection) {
		encoder_min_value_in_volts = encoder_total_value_volts;
		offset_value.as_float = encoder_min_value_in_volts;
		conf_general_store_eeprom_var_hw(&offset_value, EEPROM_ADDR_ENCODER_VALUE);
		is_calibration_done = 1;
		calibration_check.as_i32 = is_calibration_done;
		conf_general_store_eeprom_var_hw(&calibration_check, EEPROM_ADDR_CALIBRATION_CHECK);
		safety_calibration = false;
		is_in_maximum_detection = true;
	}
}

static void set_erpm_ramp_pid_response(void) {
	mc_configuration* mcconf = mempools_alloc_mcconf();

	*mcconf = *mc_interface_get_configuration();
	mc_configuration* mcconf_previous = mempools_alloc_mcconf();
	*mcconf_previous = *mcconf;

	if (change_erpm_ramp_pid_on_state) {
		mcconf->s_pid_kp = SPEED_PID_KP_HIGH;
		mcconf->s_pid_ramp_erpms_s = SPEED_ERPM_RAMP_LOW;
		change_erpm_ramp_pid_on_state = false;
	}

	if (change_erpm_ramp_pid_mom_state) {
		mcconf->s_pid_kp = SPEED_PID_KP_LOW;
		mcconf->s_pid_ramp_erpms_s = SPEED_ERPM_RAMP_HIGH;
		change_erpm_ramp_pid_mom_state = false;
	}

	mc_interface_set_configuration(mcconf_previous);
	mc_interface_set_configuration(mcconf);

	mempools_free_mcconf(mcconf);
	mempools_free_mcconf(mcconf_previous);

	is_erpm_done = true;
}

static void enable_grind_pid(void) {
	volatile const mc_configuration* conf = mc_interface_get_configuration();

	if (grind_original_kp < 0.0f) {
		grind_original_kp = conf->s_pid_kp;
		grind_original_kd = conf->s_pid_kd;
	}

	grind_ramp_is_restore = false;
	float kd_target = (speed_erpm_setpoint <= 2000.0f) ? 0.000100f : 0.000020f;
	start_grind_pid_ramp(grind_original_kp * grind_kp_multiplier,
						 kd_target);
}

static void disable_grind_pid(void) {
	if ((grind_original_kp < 0.0f) && (grind_original_kd < 0.0f)) {
		grind_ramp_step = -1;
		return;
	}

	// draining the integrator windup accumulated during grinding before the ramp starts.
	mc_configuration* mcconf = mempools_alloc_mcconf();
	*mcconf = *mc_interface_get_configuration();
	grind_original_ki = mcconf->s_pid_ki;
	mcconf->s_pid_ki = 0.0f; // a quick way to avoid erpm wind-up.
	mc_interface_set_configuration(mcconf);
	mempools_free_mcconf(mcconf);

	grind_ramp_is_restore = true;
	start_grind_pid_ramp(grind_original_kp, grind_original_kd);
}

static void start_grind_pid_ramp(float kp_to, float kd_to) {
	volatile const mc_configuration* conf = mc_interface_get_configuration();

	grind_kp_ramp_from = conf->s_pid_kp;
	grind_kd_ramp_from = conf->s_pid_kd;
	grind_kp_ramp_to = kp_to;
	grind_kd_ramp_to = kd_to;
	grind_ramp_step = 0;
}

static void grind_pid_ramp_step(void) {
	if (grind_ramp_step < 0) {
		return;
	}

	grind_ramp_step++;
	bool done = (grind_ramp_step >= GRIND_PID_RAMP_STEPS);
	float t1 = done ? 1.0f : ((float)grind_ramp_step / (float)GRIND_PID_RAMP_STEPS);

	float kp_next = grind_kp_ramp_from + (grind_kp_ramp_to - grind_kp_ramp_from) * t1;
	float kd_next = grind_kd_ramp_from + (grind_kd_ramp_to - grind_kd_ramp_from) * t1;

	mc_configuration* mcconf = mempools_alloc_mcconf();
	*mcconf = *mc_interface_get_configuration();
	mcconf->s_pid_kp = kp_next;
	mcconf->s_pid_kd = kd_next;

	// integrator is guaranteed zero. Restore Ki now.
	if (grind_ramp_is_restore && done && (grind_original_ki >= 0.0f)) {
		mcconf->s_pid_ki = grind_original_ki;
		grind_original_ki = -1.0f;
	}

	mc_interface_set_configuration(mcconf);
	mempools_free_mcconf(mcconf);

	if (done) {
		grind_ramp_step = -1;

		if (grind_ramp_is_restore) {
			grind_original_kp = -1.0f;
			grind_original_kd = -1.0f;
		}
	}
}

static void reset_grind_pid_instant(void) {
	grind_ramp_step = -1;

	if (grind_original_kp < 0.0f) {
		grind_original_ki = -1.0f;
		return;
	}

	mc_configuration* mcconf = mempools_alloc_mcconf();
	*mcconf = *mc_interface_get_configuration();
	mcconf->s_pid_kp = grind_original_kp;
	mcconf->s_pid_kd = grind_original_kd;

	if (grind_original_ki >= 0.0f) {
		mcconf->s_pid_ki = grind_original_ki;
		grind_original_ki = -1.0f;
	}

	mc_interface_set_configuration(mcconf);
	mempools_free_mcconf(mcconf);
	grind_original_kp = -1.0f;
	grind_original_kd = -1.0f;
}

static void motor_encoder_calibrate_offset(void) {
	mc_configuration* mcconf = mempools_alloc_mcconf();

	*mcconf = *mc_interface_get_configuration();
	mc_configuration* mcconf_previous = mempools_alloc_mcconf();
	*mcconf_previous = *mcconf;

	mcconf->motor_type = MOTOR_TYPE_FOC;
	mcconf->foc_f_zv = ZERO_VECTOR_FREQ;
	mcconf->foc_current_kp = FOC_KP_CONSTANT;
	mcconf->foc_current_ki = FOC_KI_CONSTANT;
	mc_interface_set_configuration(mcconf);

	float current = CALIBRATION_CURRENT;
	float offset = CALIBRATION_OFFSET_VALUE;
	float ratio = CALIBRATION_RATIO_VALUE;
	bool inverted = false;

	mcpwm_foc_encoder_detect(current, false, &offset, &ratio, &inverted);

	mcconf_previous->foc_encoder_offset = offset;
	mcconf->foc_encoder_offset = offset;

	conf_general_store_mc_configuration(mcconf_previous, mc_interface_get_motor_thread() == MOTOR_SELECTED);

	mc_interface_set_configuration(mcconf);
	mc_interface_set_configuration(mcconf_previous);

	mempools_free_mcconf(mcconf);
	mempools_free_mcconf(mcconf_previous);

	is_encoder_done = true;
}

static void play_stop_beep(void) {
	mcpwm_foc_play_tone(STOP_BEEP_CHANNEL, STOP_BEEP_FREQ_HZ, stop_beep_voltage);
	chThdSleepMilliseconds(STOP_BEEP_TONE_MS);
	mcpwm_foc_play_tone(STOP_BEEP_CHANNEL, STOP_BEEP_FREQ_HZ, 0.0f);
	chThdSleepMilliseconds(STOP_BEEP_GAP_MS);
	mcpwm_foc_play_tone(STOP_BEEP_CHANNEL, STOP_BEEP_FREQ_HZ, stop_beep_voltage);
	chThdSleepMilliseconds(STOP_BEEP_TONE_MS);
	mcpwm_foc_stop_audio(true);
}

static void calibrate_grind_threshold(void) {
	if (grind_th_manual_override) {
		return;
	}

	timeout_reset();
	mc_interface_set_pid_speed(SPEED_ERPM_MOMENTARY);
	chThdSleepMilliseconds(GRIND_TH_SETTLE_MS);

	float idle_current_sum = 0.0f;

	for (int i = 0; i < GRIND_TH_SAMPLE_COUNT; i++) {
		idle_current_sum += mc_interface_get_tot_current_filtered();
		timeout_reset();
		mc_interface_set_pid_speed(SPEED_ERPM_MOMENTARY);
		chThdSleepMilliseconds(GRIND_TH_SAMPLE_INTERVAL_MS);
	}

	float idle_current_avg = idle_current_sum / GRIND_TH_SAMPLE_COUNT;

	if (idle_current_avg < GRIND_TH_SANITY_CEILING_AMPS) {
		float calibrated_th = idle_current_avg + GRIND_TH_AUTO_MARGIN_AMPS;
		grind_current_th_amps = (calibrated_th > GRIND_CURRENT_DEFAULT_TH_AMPS) ? calibrated_th : GRIND_CURRENT_DEFAULT_TH_AMPS;

		float calibrated_no_grind_th = idle_current_avg + NO_GRIND_TH_AUTO_MARGIN_AMPS;
		no_grind_current_amps = (calibrated_no_grind_th > NO_GRIND_CURRENT_DEFAULT_AMPS) ? calibrated_no_grind_th : NO_GRIND_CURRENT_DEFAULT_AMPS;

		eeprom_var grind_th_store;
		grind_th_store.as_float = grind_current_th_amps;
		conf_general_store_eeprom_var_hw(&grind_th_store, EEPROM_ADDR_GRIND_TH_VALUE);

		eeprom_var no_grind_th_store;
		no_grind_th_store.as_float = no_grind_current_amps;
		conf_general_store_eeprom_var_hw(&no_grind_th_store, EEPROM_ADDR_NO_GRIND_TH_VALUE);
	}
}

static void write_adc_value_in_volts(void) {
	circular_buffer_in_volts[head] = knob_read_in_volts;
	head = (head + 1) % SAMPLES;

	if (circular_counter < SAMPLES) {
		circular_counter++;
	} else {
		tail = (tail + 1) % SAMPLES;
	}
}

static void read_adc_value_in_volts(void) {
	read_buffer = circular_buffer_in_volts[tail];
	tail = (tail + 1) % SAMPLES;
	circular_counter--;
}

static void adc_get_maximum_value(void) {
	eeprom_var adc_maximum_value_in_volts;

	if (knob_read_in_volts > get_maximum_adc_value_in_volts) {
		if ((get_maximum_adc_value_in_volts >= MAX_ADC_VALUE_IN_VOLTS)) {
			get_maximum_adc_value_in_volts = knob_read_in_volts;
		} else {
			get_maximum_adc_value_in_volts = MAX_ADC_VALUE_IN_VOLTS;
		}

		adc_maximum_value_in_volts.as_float = get_maximum_adc_value_in_volts;
		conf_general_store_eeprom_var_hw(&adc_maximum_value_in_volts, EEPROM_ADDR_ADC_MAX_VALUE);
	}
}

bool is_hw_fault(void) {
	bool custom_fault = false;

	//TODO: Add a custom fault here.

	return custom_fault;
}

static void adc_read_callback(void) {
	static float filter_knob = 0.0;

	filter_knob = ADC_VOLTS(ADC_IND_EXT);

	UTILS_LP_FAST(knob_read_in_volts, filter_knob, ADC_FILTER_CONSTANT);
}

float get_pfc_temp(void) {
	static float temp_pfc_filtered = 0.0;

	float temp_pfc = (UNIT_CONSTANT / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_PFC]) / NTC_RESISTANCE_VALUE) / NTC_BETA_PARAMETER) + NTC_TEMP_1_REL) - NTC_TEMP_2);

	UTILS_LP_FAST(temp_pfc_filtered, temp_pfc, TEMP_FILTER_CONSTANT);
	return temp_pfc_filtered;
}

static void terminal_set_grind_pid(int argc, const char** argv) {
	if (argc == 3) {
		float th = strtof(argv[1], NULL);
		float kp_mult = strtof(argv[2], NULL);

		if ((th <= 0.0f) || (kp_mult <= 0.0f)) {
			commands_printf("Error: all values must be > 0");
			return;
		}

		grind_current_th_amps = th;
		grind_kp_multiplier = kp_mult;
		grind_th_manual_override = true;

		commands_printf("Grind PID set: threshold=%.2f A (manual), kp_mult=%.3f",
						(double)grind_current_th_amps, (double)grind_kp_multiplier);
	} else {
		commands_printf("Usage: set_grind_pid <current_th_amps> <kp_multiplier>");
		commands_printf("Example: set_grind_pid 5.0 0.5");
	}
}

static void terminal_get_grind_pid(int argc, const char** argv) {
	(void)argc;
	(void)argv;

	commands_printf("Grind PID threshold: %.2f A (%s)", (double)grind_current_th_amps, grind_th_manual_override ? "manual" : "auto");
	commands_printf("Grind PID Kp multiplier: %.3f", (double)grind_kp_multiplier);
	commands_printf("Grind PID active: %s", grind_pid_active ? "YES" : "NO");
	commands_printf("Motor current (grind filter): %.2f A", (double)grind_current_filtered);
	commands_printf("Release point: %.2f A", (double)(grind_current_th_amps - GRIND_CURRENT_HYSTERESIS_AMPS));
	commands_printf("No-grind timeout current: %.2f A", (double)no_grind_current_amps);
}

static void terminal_set_beep_volume(int argc, const char** argv) {
	if (argc == 2) {
		float volume = strtof(argv[1], NULL);

		if ((volume < 0.0f) || (volume > STOP_BEEP_VOLTAGE_MAX)) {
			commands_printf("Error: volume must be between 0 and %.1f", (double)STOP_BEEP_VOLTAGE_MAX);
			return;
		}

		stop_beep_voltage = volume;

		commands_printf("Stop beep volume set: %.2f", (double)stop_beep_voltage);
	} else {
		commands_printf("Usage: set_beep_volume <volume>");
		commands_printf("Example: set_beep_volume 5.0 (0 = silent, max %.1f)", (double)STOP_BEEP_VOLTAGE_MAX);
	}
}

static void terminal_get_beep_volume(int argc, const char** argv) {
	(void)argc;
	(void)argv;

	commands_printf("Stop beep volume: %.2f", (double)stop_beep_voltage);
}

static void terminal_print_info(int argc, const char** argv) {
	(void)argc;
	(void)argv;

	eeprom_var data_stored;
	eeprom_var check_cal;

	conf_general_read_eeprom_var_hw(&data_stored, EEPROM_ADDR_ENCODER_VALUE);
	commands_printf("Encoder stored value: %f", (double)(data_stored.as_float));
	conf_general_read_eeprom_var_hw(&check_cal, EEPROM_ADDR_CALIBRATION_CHECK);
	commands_printf("Calibration status: %d", check_cal.as_i32);

	(is_pfc_ok()) ? commands_printf("PFC:OK") : commands_printf("PFC:OFF");

	commands_printf("ADC: %f", (double)knob_read_in_volts);
	commands_printf("ADC_cal: %f", (double)encoder_calibrated_value_in_volts);
	commands_printf("Encoder min: %f", (double)encoder_min_value_in_volts);
	commands_printf("min cal: %f", (double)encoder_min_calibrated_value);
	commands_printf("linear: %f", (double)knob_index);
	commands_printf("step: %f", (double)steps);
	commands_printf("speed (eRPM): %f", (double)speed_erpm_setpoint);
	commands_printf("speed (RPM): %f", (double)(speed_erpm_setpoint / 4));
	commands_printf("max adc: %f", (double)get_maximum_adc_value_in_volts);
}

float get_knob_read(void) {
	return encoder_calibrated_value_in_volts;
}

/* Thread to read encoder function and switch position */
static THD_FUNCTION(speed_thread, arg) {
	(void)arg;
	(void)speed_thread;

	chRegSetThreadName("speed_pid");

	chThdSleepMilliseconds(2000);

	static systime_t overload_time_in_systicks = SYSTICK_ZERO_VALUE;
	static systime_t overload_clear_time = SYSTICK_ZERO_VALUE;
	static systime_t no_grind_time_in_systicks = SYSTICK_ZERO_VALUE;
	static systime_t grind_engage_time = SYSTICK_ZERO_VALUE;
	static systime_t grind_release_time = SYSTICK_ZERO_VALUE;

	for (;;) {
		// TODO: Add a safety condition, just to avoid undesired behavior when main switch is disconnected.
		switch_positions_in_volts = ADC_VOLTS(ADC_IND_EXT2);

		if (is_pfc_ok()) {
			palSetPad(PFC_ENABLE_PORT, PFC_ENABLE_PIN);

			if (switch_positions_in_volts > SWITCH_STOP_POSITION_IN_VOLTS) {
				if (is_stop_state) {
					if (grind_pid_active || (grind_ramp_step >= 0)) {
						reset_grind_pid_instant();
						grind_pid_active = false;
					}

					grind_engage_time = SYSTICK_ZERO_VALUE;
					grind_release_time = SYSTICK_ZERO_VALUE;
					mc_interface_release_motor();
					is_stop_state = false;
					is_momentary_position_status = false;
					is_motor_grinding_enable = true;
					no_grind_time_in_systicks = SYSTICK_ZERO_VALUE;
					grind_current_filtered = 0.0;
				}

				safety_calibration = true;
				is_erpm_done = false;
				is_encoder_done = false;
			}

			if ((switch_positions_in_volts > SWITCH_ON_POSITION_1_IN_VOLTS) && (switch_positions_in_volts < SWITCH_ON_POSITION_2_IN_VOLTS) && is_calibration_done && is_motor_grinding_enable) {
				change_erpm_ramp_pid_mom_state = false;
				change_erpm_ramp_pid_on_state = true;

				if (!is_erpm_done) {
					set_erpm_ramp_pid_response();
				}

				timeout_reset();

				float current_actual = mc_interface_get_tot_current_filtered();
				UTILS_LP_FAST(grind_current_filtered, current_actual, GRIND_CURRENT_FILTER_CONSTANT);

				// Grinding current PID adaptation: adjust Kp/Kd when motor hits high load
				if (!grind_pid_active) {
					if (grind_current_filtered >= grind_current_th_amps) {
						if (grind_engage_time == SYSTICK_ZERO_VALUE) {
							grind_engage_time = chVTGetSystemTime();
						} else if (chVTTimeElapsedSinceX(grind_engage_time) > MS2ST(GRIND_ENGAGE_DELAY_MS)) {
							enable_grind_pid();
							grind_pid_active = true;
							grind_engage_time = SYSTICK_ZERO_VALUE;
						}
					} else {
						grind_engage_time = SYSTICK_ZERO_VALUE;
					}
				} else {
					if (grind_current_filtered < (grind_current_th_amps - GRIND_CURRENT_HYSTERESIS_AMPS)) {
						if (grind_release_time == SYSTICK_ZERO_VALUE) {
							grind_release_time = chVTGetSystemTime();
						} else if (chVTTimeElapsedSinceX(grind_release_time) > MS2ST(GRIND_RELEASE_DELAY_MS)) {
							disable_grind_pid();
							grind_pid_active = false;
							grind_release_time = SYSTICK_ZERO_VALUE;
						}
					} else {
						grind_release_time = SYSTICK_ZERO_VALUE;
					}
				}

				grind_pid_ramp_step();
				mc_interface_set_pid_speed(speed_erpm_setpoint);

				if (grind_current_filtered < no_grind_current_amps) {
					if (no_grind_time_in_systicks == SYSTICK_ZERO_VALUE) {
						no_grind_time_in_systicks = chVTGetSystemTime();
					} else {
						if (chVTTimeElapsedSinceX(no_grind_time_in_systicks) > S2ST(GRIND_TIMEOUT_SEC)) {
							mc_interface_release_motor();

							if (grind_pid_active || (grind_ramp_step >= 0)) {
								reset_grind_pid_instant();
								grind_pid_active = false;
							}

							is_motor_grinding_enable = false;
							no_grind_time_in_systicks = SYSTICK_ZERO_VALUE;
							play_stop_beep();
						}
					}
				} else {
					no_grind_time_in_systicks = SYSTICK_ZERO_VALUE;
				}

				is_stop_state = true;
			}

			if ((switch_positions_in_volts < SWITCH_MOMENTARY_POSITION_IN_VOLTS)) {
				if (grind_pid_active || (grind_ramp_step >= 0)) {
					reset_grind_pid_instant();
					grind_pid_active = false;
				}

				grind_engage_time = SYSTICK_ZERO_VALUE;
				grind_release_time = SYSTICK_ZERO_VALUE;

				if (safety_calibration) {
					change_erpm_ramp_pid_mom_state = true;
					change_erpm_ramp_pid_on_state = false;

					if (!is_erpm_done) {
						set_erpm_ramp_pid_response();
					}

					speed_erpm_setpoint = SPEED_ERPM_MOMENTARY;
					is_momentary_position_status = true;

					timeout_reset();
					mc_interface_set_pid_speed(speed_erpm_setpoint);
					is_stop_state = true;
				} else {
					if (!is_encoder_done) {
						while (!main_init_done()) { // here wait until the whole main configuration finish otherwise the encoder calibration won't work properly.
							chThdSleepMilliseconds(10);
						}

						knob_encoder_calibrate_offset();
						motor_encoder_calibrate_offset();
						calibrate_grind_threshold();
						store_minimum_value = true;
					}

					adc_get_maximum_value();
				}
			}

			if ((mc_interface_get_tot_current() >= CUTOFF_CURRENT_AMPS)) {
				overload_clear_time = SYSTICK_ZERO_VALUE;

				if (overload_time_in_systicks == SYSTICK_ZERO_VALUE) {
					overload_time_in_systicks = chVTGetSystemTime();
				} else {
					if (chVTTimeElapsedSinceX(overload_time_in_systicks) > MS2ST(CURRENT_MOTOR_TIMEOUT_MS)) {
						mc_interface_release_motor();

						if (grind_pid_active || (grind_ramp_step >= 0)) {
							reset_grind_pid_instant();
							grind_pid_active = false;
						}

						is_motor_grinding_enable = false;
						overload_time_in_systicks = SYSTICK_ZERO_VALUE;
					}
				}
			} else {
				if (overload_time_in_systicks != SYSTICK_ZERO_VALUE) {
					if (overload_clear_time == SYSTICK_ZERO_VALUE) {
						overload_clear_time = chVTGetSystemTime();
					} else if (chVTTimeElapsedSinceX(overload_clear_time) > MS2ST(OVERLOAD_CLEAR_DELAY_MS)) {
						overload_time_in_systicks = SYSTICK_ZERO_VALUE;
						overload_clear_time = SYSTICK_ZERO_VALUE;
					}
				} else {
					overload_clear_time = SYSTICK_ZERO_VALUE;
				}
			}
		}

		chThdSleepMilliseconds(10);
	}
}

static THD_FUNCTION(encoder_thread, arg) {
	(void)arg;
	(void)encoder_thread;

	chRegSetThreadName("enc_read");

	eeprom_var encoder_min_value_stored;
	eeprom_var step_value_stored;

	const float encoder_max_calibrated_value = MAX_ENCODER_VALUE_IN_VOLTS;
	float diff;
	float get_encoder_sample_in_volts = 0.0;

	for (;;) {
		if (!is_momentary_position_status) {
			write_adc_value_in_volts();

			if (circular_counter > 0) {
				read_adc_value_in_volts();
			}

			uint8_t last_written = (head == 0) ? (SAMPLES - 1) : (head - 1);
			diff = fabs(circular_buffer_in_volts[last_written] - read_buffer);

			if (diff < THRESHOLD_VALUE) {
				get_encoder_sample_in_volts = read_buffer - OFFSET_FACTOR_CORRECTION;
			}

			encoder_calibrated_value_in_volts = (encoder_min_value_in_volts - get_encoder_sample_in_volts);

			if (encoder_calibrated_value_in_volts < 0.0) {
				encoder_calibrated_value_in_volts += get_maximum_adc_value_in_volts - OFFSET_FACTOR_CORRECTION;
			}

			if (store_minimum_value) {
				encoder_min_calibrated_value = encoder_calibrated_value_in_volts;
				steps = (encoder_max_calibrated_value - encoder_min_calibrated_value) / KNOB_STEPS;
				encoder_min_value_stored.as_float = encoder_min_calibrated_value;
				conf_general_store_eeprom_var_hw(&encoder_min_value_stored, EEPROM_ADDR_MIN_CALIBRATED_VALUE);
				step_value_stored.as_float = steps;
				conf_general_store_eeprom_var_hw(&step_value_stored, EEPROM_ADDR_STEPS_VALUE);
				store_minimum_value = false;
			}

			if (steps > 0.0f) {
				knob_index = roundf(((encoder_calibrated_value_in_volts - encoder_min_calibrated_value) / steps));
			} else {
				knob_index = MIN_KNOB_INDEX;
			}

			if (knob_index < MIN_KNOB_INDEX) {
				knob_index = MIN_KNOB_INDEX;
			} else if (knob_index > MAX_KNOB_INDEX) {
				knob_index = MAX_KNOB_INDEX;
			}

			speed_erpm_setpoint = erpm_lut[(int)(knob_index)];
		}

		chThdSleepMilliseconds(2);
	}
}
