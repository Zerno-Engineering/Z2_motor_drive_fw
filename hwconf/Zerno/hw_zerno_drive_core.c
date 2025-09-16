/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
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

#include <string.h>
#include <math.h>
#include <stdlib.h>

#define	EEPROM_ADDR_ENCODER_VALUE	2
#define EEPROM_ADDR_CALIBRATION_CHECK	6
#define MAGNET_TIMEOUT_MS 1000

static THD_FUNCTION(speed_thread, arg);
static THD_WORKING_AREA(speed_thread_wa, 1024);

static bool speed_thread_running = false;

volatile uint16_t encoder_max_value = 16384; // 14 bit max value
volatile uint16_t encoder_min_value = 0; //not sure if will be 0, but need to be tested in hardware.
volatile uint16_t encoder_value_high;
volatile uint16_t encoder_value_low;
volatile uint16_t encoder_total_value;
volatile uint16_t encoder_value_filtered;
volatile uint16_t encoder_setpoint;
volatile uint16_t encoder_magnet_check;
volatile float_t encoder_rel = 0.0;
volatile float speed_setpoint = 0.0;
volatile float encoder_rel_ema;
volatile float encoder_setpoint_ema = 0.0; // Add this line at the top with other globals

// variable for test purposes
int is_calibration_done = 0 ;

//static uint16_t mt6816_spi_transfer(uint16_t out);
//static uint16_t mt6816_read_register(uint8_t reg_addr);
float_t encoder_relative_val(uint16_t data_encoder);
static void zerno_callback(void);

void spi_delay(void);
void cs_delay(void);
void define_default_values(void);
void encoder_calibrate_offset(void);
void pid_speed(float set_rpm);

bool is_pfc_ok(void);

bool motor_start = false;
bool parity_check = false;
bool enable_spi = false;
bool safety_calibration = false;

float get_pfc_temp(void);

// Variables
static volatile bool i2c_running = false;
static mutex_t shutdown_mutex;

static bool shutdown_pressed = false;
static int shutdown_pressed_time = 0;

// I2C configuration
static const I2CConfig i2cfg = {
		OPMODE_I2C,
		100000,
		STD_DUTY_CYCLE
};

// Private functions
static void terminal_shutdown_now(int argc, const char **argv);
static void terminal_button_test(int argc, const char **argv);
static void terminal_print_info(int argc, const char **argv);
static void terminal_motor_run(int argc , const char **argv);

void hw_init_gpio(void) {
	chMtxObjectInit(&shutdown_mutex);

	// GPIO clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// LEDs
	palSetPadMode(LED_GREEN_GPIO, LED_GREEN_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(LED_RED_GPIO, LED_RED_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// GPIOA Configuration: Channel 1 to 3 as alternate function push-pull
	palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	//INIT_BR();

	// Hall sensors
	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT);
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT);
	palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_INPUT);

	// Phase filters

	palSetPadMode(PHASE_FILTER_GPIO, PHASE_FILTER_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	PHASE_FILTER_OFF();

	/*
	// Current filter
	palSetPadMode(CURRENT_FILTER_GPIO, CURRENT_FILTER_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	CURRENT_FILTER_OFF();
	*/
	// Sensor port voltage
	SENSOR_PORT_3V3();
	palSetPadMode(SENSOR_VOLTAGE_GPIO, SENSOR_VOLTAGE_PIN,
			PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

	// ZCD-pin
	palSetPadMode(ZCD_GPIO, ZCD_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palClearPad(ZCD_GPIO, ZCD_PIN);

	// CAN_EN-pin
	palSetPadMode(CAN_EN_GPIO, CAN_EN_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palClearPad(CAN_EN_GPIO, CAN_EN_PIN);

	//Shutdown
	palSetPadMode(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(HW_SHUTDOWN_SENSE_GPIO, HW_SHUTDOWN_SENSE_PIN, PAL_MODE_INPUT);

	// ADC Pins
    palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG);

	// Magnetic encoder pins 
	palSetPadMode(MT6816_MISO_PORT, MT6816_MISO_PIN, PAL_MODE_INPUT);
	palSetPadMode(MT6816_MOSI_PORT, MT6816_MOSI_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(MT6816_CLK_PORT, MT6816_CLK_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(MT6816_CS_PORT, MT6816_CS_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

	palSetPad(MT6816_CLK_PORT,MT6816_CLK_PIN); // starts with clock in HIGH state. Otherwise spi (bit banged) won't work.

	// Switch input pins
	palSetPadMode(HW_SW_PORT, HW_SW_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_MOMENTARY_PORT, HW_MOMENTARY_PIN, PAL_MODE_INPUT_PULLUP);

	// PFC interface signals
	palSetPadMode(PFC_STATUS_PORT, PFC_STATUS_PIN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(PFC_ENABLE_PORT, PFC_ENABLE_PIN, PAL_MODE_INPUT);

	//palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_ANALOG);

	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG);

	// DAC as voltage reference for shunt amps
	palSetPadMode(GPIOA, 4, PAL_MODE_INPUT_ANALOG);
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	//DAC->CR |= DAC_CR_EN1;
	//DAC->DHR12R1 = 2047;
	define_default_values();

	encoder_calibrate_offset();

	if(is_pfc_ok())
		palSetPad(PFC_ENABLE_PORT, PFC_ENABLE_PIN);
	else
		palClearPad(PFC_ENABLE_PORT, PFC_ENABLE_PIN);

	if (!speed_thread_running) {
				chThdCreateStatic(speed_thread_wa, sizeof(speed_thread_wa), NORMALPRIO, speed_thread, NULL);
				speed_thread_running = true;
			}

	terminal_register_command_callback(
			"shutdown",
			"Shutdown VESC now.",
			0,
			terminal_shutdown_now);

	terminal_register_command_callback(
			"test_button",
			"Try sampling the shutdown button",
			0,
			terminal_button_test);

	terminal_register_command_callback(
			"encoder_status",
			"Value",
			0,
			terminal_print_info);

	terminal_register_command_callback(
				"motor_state",
				"on/off",
				0,
				terminal_motor_run);

	mc_interface_set_pwm_callback (zerno_callback); // Set a function that should be called after each PWM cycle.

}

void hw_setup_adc_channels(void) {
	// ADC1 regular channels														// index
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_15Cycles);		// 0
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 2, ADC_SampleTime_15Cycles);		// 3
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 3, ADC_SampleTime_15Cycles); 		// 6
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 4, ADC_SampleTime_15Cycles); 	// 9 TEMP MOTOR
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 5, ADC_SampleTime_15Cycles);// 12
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 6, ADC_SampleTime_15Cycles); 		// 15 PA4 PFC temperature.
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 6, ADC_SampleTime_15Cycles);

	// ADC2 regular channels
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_15Cycles);		// 1
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 2, ADC_SampleTime_15Cycles);		// 4
	ADC_RegularChannelConfig(ADC2, ADC_Channel_6, 3, ADC_SampleTime_15Cycles);		// 7
	//ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 4, ADC_SampleTime_15Cycles);	// 10
	ADC_RegularChannelConfig(ADC2, ADC_Channel_0, 5, ADC_SampleTime_15Cycles);		// 13
	ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 6, ADC_SampleTime_15Cycles);		// 16

	// ADC3 regular channels
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_15Cycles);		// 2
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 2, ADC_SampleTime_15Cycles);		// 5
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 3, ADC_SampleTime_15Cycles);		// 8
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 4, ADC_SampleTime_15Cycles);		//11
	ADC_RegularChannelConfig(ADC3, ADC_Channel_1, 5, ADC_SampleTime_15Cycles);		// 15
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 6, ADC_SampleTime_15Cycles);		// 17

	// Injected channels
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 3, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 3, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 3, ADC_SampleTime_15Cycles);
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

		for(int i = 0;i < 16;i++) {
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

bool hw_sample_shutdown_button(void) {

	chMtxLock(&shutdown_mutex);
	shutdown_pressed = palReadPad(HW_SHUTDOWN_SENSE_GPIO, HW_SHUTDOWN_SENSE_PIN) == PAL_HIGH;
	chMtxUnlock(&shutdown_mutex);
	if(shutdown_pressed){
		shutdown_pressed_time += 10;
		return true;
	}else{
		if(shutdown_pressed_time > 1000){
			shutdown_pressed_time = 0;
			return false;
		}else{
			shutdown_pressed_time = 0;
			return true;
		}
	}
}

static void terminal_shutdown_now(int argc, const char **argv) {
	(void)argc;
	(void)argv;
	DISABLE_GATE();
	HW_SHUTDOWN_HOLD_OFF();
}

static void terminal_button_test(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	//for (int i = 0;i < 40;i++) {
	//	commands_printf("BT: %d %.2f", HW_SAMPLE_SHUTDOWN(), (double)bt_diff);
	//	chThdSleepMilliseconds(100);
	//}
}

void spi_delay(void) {
	// ~167ns long..
	for (volatile int i = 0; i < 1; i++) { // for 1 : 3.5MHZ spi clock
		__NOP();
	}
}

void cs_delay(void) {

	for (volatile int i = 0; i < 1; i++) {
			__NOP();
	}
}

uint16_t mt6816_spi_transfer(uint16_t out) {
	uint16_t in = 0;
	for (int i = 15; i >= 0; i--) {

		if (out & (1 << i)) {
			MT6816_MOSI_HIGH();
		} else {
			MT6816_MOSI_LOW();
		}
		MT6816_CLK_LOW();
		spi_delay();
		in <<= 1;
		if (MT6816_MISO_READ()) {
			in |= 1;
		}
		MT6816_CLK_HIGH();
		spi_delay();
	}
	return in;
}

/*
	Read register from MT6816:
	According data sheet, the encoder value is 14Bits (16384)
	and the data structure is shown like this:
	to send: [R/W:1] [ADRESS = 0b00000011(0x03)] to get that value.
*/
uint16_t mt6816_read_register(uint8_t reg_addr) {
	uint16_t cmd = 0x8000 | ((reg_addr & 0x7F) << 8); // R/W=1, 7-bit addr, rest 0 - prepare the frame to be sent.
	uint16_t reg_val;

	MT6816_CS_LOW();
	cs_delay();//__NOP;//chThdSleepMicroseconds(1); // originally it was commented
	reg_val = mt6816_spi_transfer(cmd);
	MT6816_CS_HIGH();
	cs_delay(); //__NOP;//chThdSleepMicroseconds(1);

	return reg_val;
}

float_t encoder_relative_val(uint16_t data_encoder) {
	float relative;
	float calibrated_val;

	data_encoder += 300;

	calibrated_val = (float)(data_encoder-encoder_min_value); // need to add a correction factor

	if(calibrated_val < 0) {
		calibrated_val += encoder_max_value;
	}

	relative = calibrated_val/encoder_max_value; // comment this to send just the raw calibrated value from the encoder.

	return relative;
}

bool is_momentary_position(void) {
	return (bool)palReadPad(HW_MOMENTARY_PORT, HW_MOMENTARY_PIN);
}

bool is_sw_position(void) {
	return (bool)palReadPad(HW_SW_PORT, HW_SW_PIN);
}

bool is_pfc_ok(void) {
	return (bool)palReadPad(PFC_STATUS_PORT, PFC_STATUS_PIN);
}

/* Load the stored values during boot
 *
 */
void define_default_values(void) {
	eeprom_var default_offset, default_calibration;

	conf_general_read_eeprom_var_hw(&default_offset, EEPROM_ADDR_ENCODER_VALUE);
	encoder_min_value = default_offset.as_i32;

	conf_general_read_eeprom_var_hw(&default_calibration, EEPROM_ADDR_CALIBRATION_CHECK);
	is_calibration_done = default_calibration.as_i32;
}

void encoder_calibrate_offset(void) {

	eeprom_var offset_value, calibration_check;

	encoder_value_high = mt6816_read_register(0x03);
	encoder_value_low = mt6816_read_register(0x04);

	encoder_total_value = (encoder_value_high << 6) | (encoder_value_low & (0xfc));
	encoder_magnet_check = (encoder_value_low & 0x02);

	if(!is_momentary_position()) { //&& !encoder_magnet_check) { // perform a calibration during boot. Just check momentary positions. Check the initial value
		encoder_min_value = encoder_total_value;
		offset_value.as_i32 = encoder_min_value;
		conf_general_store_eeprom_var_hw(&offset_value, EEPROM_ADDR_ENCODER_VALUE);
		encoder_max_value = 16384;
		is_calibration_done = 1;
		calibration_check.as_i32 = is_calibration_done;
		conf_general_store_eeprom_var_hw(&calibration_check, EEPROM_ADDR_CALIBRATION_CHECK);
		safety_calibration = true;
	}
}

bool is_hw_fault(void) {

bool magnet_error = false;
bool pfc_error = false;

	if(encoder_magnet_check) { // commented for test purposes
		mc_interface_set_fault_info("FAULT_ENCODER_NO_MAGNET", 0, 0, 0);
		magnet_error = true;
	}
	if(!is_pfc_ok()) {
		mc_interface_set_fault_info("FAULT_PFC_ERROR",0,0,0);
		pfc_error = true;
	}

	return (pfc_error | magnet_error );
}

float get_pfc_temp(void) {
	static float temp_pfc_filtered = 0.0;

	float temp_pfc = (1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS_2]) / 10000.0) / 3455.0) + (1.0 / 298.15)) - 273.15);
	UTILS_LP_FAST(temp_pfc_filtered, temp_pfc, 0.1);
	return temp_pfc_filtered;
}

void pid_speed(float set_rpm) {
	timeout_reset();
    mc_interface_set_pid_speed(set_rpm); //
}

static void zerno_callback(void) {
	// Called for every control iteration in interrupt context.
    //enable_spi = true;
	// add a flag here
}


static void terminal_print_info(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	eeprom_var data_stored, check_cal;

	conf_general_read_eeprom_var_hw(&data_stored, EEPROM_ADDR_ENCODER_VALUE);
	//commands_printf("Encoder stored value: %d", data_stored.as_i32);
	conf_general_read_eeprom_var_hw(&check_cal, EEPROM_ADDR_CALIBRATION_CHECK);
	//commands_printf("Calibration status: %d", check_cal.as_i32);

	if(is_momentary_position())
		commands_printf("Momentary_pos:OFF");
	else
		commands_printf("Momentary_pos:ON");

	if(is_sw_position())
			commands_printf("sw_pos:OFF");
		else
			commands_printf("sw_pos:ON");


	if(encoder_magnet_check) {
		commands_printf("Magnet status: MAGNET ERROR"); // This can be added as a custom error
		commands_printf("Encoder value: %d", encoder_total_value);
		commands_printf("Encoder value filtered; %d" , encoder_value_filtered);
		commands_printf("Encoder rel: %f", (double)encoder_rel);
		commands_printf("EMA filter: %f", (double)encoder_rel_ema);
		commands_printf("speed: %f", (double)speed_setpoint);
	}
		else {
	    commands_printf("Magnet status: MAGNET OK");
		commands_printf("Encoder value: %d", encoder_total_value);
		commands_printf("Encoder value filtered; %d" , encoder_value_filtered);
		commands_printf("Encoder rel: %f", (double)encoder_rel);
		commands_printf("EMA filter: %f", (double)encoder_rel_ema);
		commands_printf("speed: %f", (double)speed_setpoint);
	}
}

static void terminal_motor_run(int argc , const char **argv) {
	(void)argc;
	(void)argv;

	if(strcmp(argv[1], "ON") == 0) {
		commands_printf("Motor ON");
		motor_start = true;
	}
	if(strcmp(argv[1], "OFF") == 0) {
		commands_printf("Motor OFF");
		motor_start = false;
	}

	if(motor_start)
		commands_printf("Running...");

	else
		commands_printf("Stop...");
}

/* Thread to read encoder function and switch position */

static THD_FUNCTION(speed_thread, arg) {
    (void)arg;

    chRegSetThreadName("speed_pid");

    chThdSleepMilliseconds(5000);

    static systime_t last_magnet_ok_time = 0;

    uint8_t reg_addr_1 = 0x03; // address to read the angle from the magnetic encoder.
    uint8_t reg_addr_2 = 0x04; // Register to check the magnetic flux and parity check. And get angle data from the latest 6 bit.

    for(;;) {
        uint16_t encoder_samples[5];
        int valid_samples = 0;

    // take 5 samples from encoder.
        for (int i = 0; i < 5; i++) {
            uint16_t value_high = mt6816_read_register(reg_addr_1);
            uint16_t value_low = mt6816_read_register(reg_addr_2);
            uint16_t value_total = (value_high << 8) | value_low;

            if (spi_bb_check_parity(value_total)) { // Just keep valid data.
                encoder_samples[valid_samples++] = value_total;
            }
            chThdSleepMilliseconds(5);
        }

        uint16_t filtered_encoder = 0;
        if (valid_samples == 0) {
            parity_check = false;
        } else if (valid_samples == 1) {
            filtered_encoder = encoder_samples[0];
            parity_check = true;
        } else {
            // Here the samples are sorted to get the median value..(based on AN4515 - median filter)
        	for (int i = 0; i < valid_samples - 1; i++) {
                for (int j = i + 1; j < valid_samples; j++) {
                    if (encoder_samples[i] > encoder_samples[j]) {
                        uint16_t tmp = encoder_samples[i];
                        encoder_samples[i] = encoder_samples[j];
                        encoder_samples[j] = tmp;
                    }
                }
            }
            // get the median value of the 5 samples..
            filtered_encoder = encoder_samples[valid_samples / 2];
            parity_check = true;
        }

        encoder_total_value = filtered_encoder;

        if(parity_check && !(filtered_encoder & 0x02)) {
        	encoder_magnet_check = 0;
        	encoder_setpoint = filtered_encoder >> 2;
            last_magnet_ok_time = chVTGetSystemTimeX();
        }

        // No magnet timeout
        if ((filtered_encoder & 0x02) && (chVTTimeElapsedSinceX(last_magnet_ok_time) > MS2ST(MAGNET_TIMEOUT_MS))) {
            encoder_magnet_check = 1;
        }

        if(is_pfc_ok()) {
            palSetPad(PFC_ENABLE_PORT, PFC_ENABLE_PIN);

            if(!is_sw_position() && !encoder_magnet_check && parity_check && is_calibration_done) {
                encoder_rel = encoder_relative_val (encoder_setpoint);
                speed_setpoint = utils_map(encoder_rel, 0.0 , 0.99, 0.0, 6400);
                speed_setpoint = round(speed_setpoint/400)*400; // round the values . steps of 400
                timeout_reset();
                mc_interface_set_pid_speed(speed_setpoint);
            }

            if(!is_momentary_position() && is_calibration_done) {
            	if(!safety_calibration) {
            		speed_setpoint = 8000;
            		timeout_reset();
            		mc_interface_set_pid_speed(speed_setpoint);
            	}
            }
            else {
            	safety_calibration = false;
            }
        }
        else {
            palClearPad(PFC_ENABLE_PORT, PFC_ENABLE_PIN);
        }
        chThdSleepMilliseconds(75); //
    }
}
