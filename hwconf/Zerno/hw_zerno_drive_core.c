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

#include <math.h>

#define	EEPROM_ADDR_ENCODER_VALUE	2
#define EEPROM_ADDR_CALIBRATION_CHECK	6

static THD_FUNCTION(zerno_thread, arg);
static THD_WORKING_AREA(zerno_thread_wa, 1024);
static bool zerno_thread_running = false;

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

uint16_t encoder_max_value = 16384; // 14 bit max value
uint16_t encoder_min_value = 0; //not sure if will be 0, but need to be tested in hardware.
uint16_t encoder_value_high;
uint16_t encoder_value_low;
uint16_t encoder_total_value;
uint16_t encoder_magnet_check;
float_t encoder_rel = 0.0;

// variable for test purposes
int calib_aux;
int32_t encoder_min;

int is_calibration_done = 0 ;

// Define default values before the encoder calibration.
void define_default_values(void) {
	eeprom_var default_offset, default_calibration;

	conf_general_read_eeprom_var_hw(&default_offset, EEPROM_ADDR_ENCODER_VALUE);
	encoder_min = default_offset.as_i32; // encoder_min_value

	conf_general_read_eeprom_var_hw(&default_calibration, EEPROM_ADDR_CALIBRATION_CHECK);
	calib_aux = default_calibration.as_i32; // is_calibration_done
}

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

	//palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_ANALOG);

	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG);

	// DAC as voltage reference for shunt amps
	//palSetPadMode(GPIOA, 4, PAL_MODE_INPUT_ANALOG);
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	//DAC->CR |= DAC_CR_EN1;
	//DAC->DHR12R1 = 2047;
	define_default_values();

	if (!zerno_thread_running) {
			chThdCreateStatic(zerno_thread_wa, sizeof(zerno_thread_wa), NORMALPRIO, zerno_thread, NULL);
			zerno_thread_running = true;
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
			"encoder_value",
			"Value",
			0,
			terminal_print_info);

}

void hw_setup_adc_channels(void) {
	// ADC1 regular channels
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 5, ADC_SampleTime_15Cycles);
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 6, ADC_SampleTime_15Cycles);

	// ADC2 regular channels
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_6, 3, ADC_SampleTime_15Cycles);
	//ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_0, 5, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 6, ADC_SampleTime_15Cycles);

	// ADC3 regular channels
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_1, 5, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 6, ADC_SampleTime_15Cycles);

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

void spi_delay(void) {
	// ~167ns long..
	for (volatile int i = 0; i < 500; i++) {
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
	//chThdSleepMicroseconds(1);
	reg_val = mt6816_spi_transfer(cmd);
	MT6816_CS_HIGH();
	chThdSleepMicroseconds(1);

	//MT6816_CS_LOW();
	//spi_delay();//chThdSleepMicroseconds(1);
	//reg_val = mt6816_spi_transfer(0x0000); // Read response
	//MT6816_CS_HIGH();

	return reg_val;
}


float_t encoder_calibration(uint16_t data_encoder) {
	float relative;
	float calibrated_val;
    eeprom_var offset_value, calibration_check;

    if(!is_calibration_done) { // this function needs to be performed with some condition., and executed just once for calibration if needed.
    	encoder_min_value = data_encoder;
    	offset_value.as_i32 = encoder_min_value;
    	conf_general_store_eeprom_var_hw(&offset_value, EEPROM_ADDR_ENCODER_VALUE);
    	encoder_max_value = 16384;
    	is_calibration_done = 1;
    	calibration_check.as_i32 = is_calibration_done;
    	conf_general_store_eeprom_var_hw(&calibration_check, EEPROM_ADDR_CALIBRATION_CHECK);
    }

    calibrated_val = (float)(data_encoder-encoder_min_value) ;

    if(calibrated_val < 0) {
		calibrated_val += encoder_max_value;
	}

    relative = calibrated_val/encoder_max_value;

    return relative;
}

bool is_momentary_position(void) {
	if(palReadPad(HW_MOMENTARY_PORT, HW_MOMENTARY_PIN))
		return true;
	else
		return false;
}

bool is_sw_position(void) {
	if(palReadPad(HW_SW_PORT, HW_SW_PIN))
		return true;
	else
		return false;
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

static void terminal_print_info(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	eeprom_var data_stored, check_cal;

	conf_general_read_eeprom_var_hw(&data_stored, EEPROM_ADDR_ENCODER_VALUE);
	commands_printf("Encoder stored_value: %d", data_stored.as_i32);
	conf_general_read_eeprom_var_hw(&check_cal, EEPROM_ADDR_CALIBRATION_CHECK);
	commands_printf("Calibration status: %d", check_cal.as_i32);
	commands_printf("Initial encoder value:%d", encoder_min);
	commands_printf("Initial calibration status:%d", calib_aux);


	if(encoder_magnet_check)
		commands_printf("MAGNET ERROR"); // This can be added as a custom error
	else {
		commands_printf("Encoder value: %d", encoder_total_value);
		commands_printf("Encoder rel: %f", (double)encoder_rel);
	}
}

/* This thread is used for magnetic encoder and switch input.
 */
static THD_FUNCTION(zerno_thread, arg) {
	(void)arg;

	chRegSetThreadName("zerno_drive");
	chThdSleepMilliseconds(3000);

	// encoder address
	uint8_t reg_addr_1 = 0x03; // address to read the angle from the magnetic encoder.
	uint8_t reg_addr_2 = 0x04; // Register to check the magnetic flux and parity check. And get angle data from the latest 6 bit.

	for(;;) {

		encoder_value_high = mt6816_read_register(reg_addr_1);
		encoder_value_low = mt6816_read_register(reg_addr_2);

		// The data is concatenated, since part of the angle information comes in registers 0x03 [13:6] and 0x04 [5:0] to form the 14 bits
		encoder_total_value = (encoder_value_high << 6) | (encoder_value_low & (0xfc));
		encoder_magnet_check = (encoder_value_low & 0x02);

		encoder_rel = encoder_calibration(encoder_total_value); // we don't know about the encoder position

		chThdSleepMilliseconds(20);
	}
}
