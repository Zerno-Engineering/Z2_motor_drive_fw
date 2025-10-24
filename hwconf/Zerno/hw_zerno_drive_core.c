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
#include "mempools.h"
#include "mcpwm_foc.h"
#include "main.h"
#include "app.h"

#include <string.h>
#include <math.h>
#include <stdlib.h>

#define	EEPROM_ADDR_ENCODER_VALUE	2
#define EEPROM_ADDR_CALIBRATION_CHECK	6
#define CURRENT_MOTOR_TIMEOUT 2000
#define GRIND_TIMEOUT 600 // value in seconds
#define CUTOFF_CURRENT 3.0 // current for a stalled motor
#define NO_GRIND_CURRENT 0.6
#define GRIND_ATTEMPS 3

static THD_FUNCTION(speed_thread, arg);
static THD_FUNCTION(encoder_thread, arg);

static THD_WORKING_AREA(speed_thread_wa, 1024);
static THD_WORKING_AREA(encoder_thread_wa, 1024);

static bool speed_thread_running = false;
static bool encoder_thread_running = false;

volatile float encoder_max_value = 3.2;
volatile float encoder_min_value = 0.0; //not sure if will be 0, but need to be tested in hardware.
volatile float encoder_total_value;
volatile float main_switch_value;
volatile float speed_setpoint = 0.0;
volatile float Knob_read;

static void adc_read_callback(void);
// variable for test purposes
int is_calibration_done = 0 ;

float get_pfc_temp(void);
float calib;

void define_default_values(void);
void encoder_calibrate_offset(void);
void pid_speed(float set_rpm);

bool is_pfc_ok(void);
bool motor_start = false;
bool parity_check = false;
bool enable_spi = false;
bool safety_calibration = false;
bool is_erpm_done = false;
bool is_default_erpm = true;
bool is_encoder_done = false;
bool is_stop_state = false ;
bool is_pid_kd_change_up = false;
bool is_pid_kd_change_down = false;
bool is_momentary_position_status = false;
bool is_motor_stalled_fault = false;
bool is_motor_grinding_enable = true;
// Variables
static volatile bool i2c_running = false;

// I2C configuration
static const I2CConfig i2cfg = {
		OPMODE_I2C,
		100000,
		STD_DUTY_CYCLE
};

// Private functions
static void terminal_print_info(int argc, const char **argv);
static void terminal_motor_run(int argc , const char **argv);

void hw_init_gpio(void) {

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

	// ADC Pins
    palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG);

	// Switch input pins
	palSetPadMode(HW_SW_PORT, HW_SW_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_MOMENTARY_PORT, HW_MOMENTARY_PIN, PAL_MODE_INPUT_PULLUP);

	// PFC interface signals
	palSetPadMode(PFC_STATUS_PORT, PFC_STATUS_PIN, PAL_MODE_INPUT);
	palSetPadMode(PFC_ENABLE_PORT, PFC_ENABLE_PIN, PAL_MODE_OUTPUT_PUSHPULL);

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

	mc_interface_set_pwm_callback (adc_read_callback);

	define_default_values();

	encoder_calibrate_offset(); // disable calibrate offset to avoid  data encoder readings

	if (!speed_thread_running) {
				chThdCreateStatic(speed_thread_wa, sizeof(speed_thread_wa), NORMALPRIO, speed_thread, NULL);
				speed_thread_running = true;
			}

	if (!encoder_thread_running) {
					chThdCreateStatic(encoder_thread_wa, sizeof(encoder_thread_wa), NORMALPRIO, encoder_thread, NULL);
					encoder_thread_running = true;
				}

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

bool is_momentary_position(void) {
	return (bool)palReadPad(HW_MOMENTARY_PORT, HW_MOMENTARY_PIN);
}

bool is_sw_position(void) {
	return (bool)palReadPad(HW_SW_PORT, HW_SW_PIN);
}

bool is_pfc_ok(void) {
	return (bool)palReadPad(PFC_STATUS_PORT, PFC_STATUS_PIN);
}

/* Enable adc readings for main switch */

float main_switch_adc_value(void) {
	static float main_switch = 0.0;
	static float main_switch_filtered = 0.0;

	main_switch = ADC_VOLTS(ADC_IND_EXT2);
	UTILS_LP_FAST(main_switch_filtered, main_switch, 0.1);

	return main_switch_filtered;
}
/* Load the stored values during start-up
 *
 */
void define_default_values(void) {
	eeprom_var default_offset, default_calibration;

	conf_general_read_eeprom_var_hw(&default_offset, EEPROM_ADDR_ENCODER_VALUE);
	encoder_min_value = default_offset.as_float;

	conf_general_read_eeprom_var_hw(&default_calibration, EEPROM_ADDR_CALIBRATION_CHECK);
	is_calibration_done = default_calibration.as_i32;
}

void encoder_calibrate_offset(void) {

	eeprom_var offset_value, calibration_check;

    encoder_total_value = ADC_VOLTS(ADC_IND_EXT); // get the knob position values
    main_switch_value = ADC_VOLTS(ADC_IND_EXT2); // get the switch position values

	if(!is_momentary_position()) { // Digital and analog detection for calibration mode.
		encoder_min_value = encoder_total_value;
		offset_value.as_float = encoder_min_value;
		conf_general_store_eeprom_var_hw(&offset_value, EEPROM_ADDR_ENCODER_VALUE);
		encoder_max_value = 3.2;
		is_calibration_done = 1;
		calibration_check.as_i32 = is_calibration_done;
		conf_general_store_eeprom_var_hw(&calibration_check, EEPROM_ADDR_CALIBRATION_CHECK);
		safety_calibration = true;
	}
}

void set_erpm_ramp_response(void) {
	mc_configuration *mcconf = mempools_alloc_mcconf();
	*mcconf = *mc_interface_get_configuration();
	mc_configuration *mcconf_old = mempools_alloc_mcconf();
	*mcconf_old = *mcconf;

	if(!is_default_erpm)
		mcconf->s_pid_ramp_erpms_s = 10000.0;
	else
		mcconf->s_pid_ramp_erpms_s = 20000.0;

	mc_interface_set_configuration(mcconf_old);
	mc_interface_set_configuration(mcconf);

	mempools_free_mcconf(mcconf);
	mempools_free_mcconf(mcconf_old);

	is_erpm_done = true;
}

void set_pid_constant(void) {

	mc_configuration *mcconf = mempools_alloc_mcconf();
	*mcconf = *mc_interface_get_configuration();
	mc_configuration *mcconf_old = mempools_alloc_mcconf();
	*mcconf_old = *mcconf;

	if(speed_setpoint < 3600) {
		mcconf-> s_pid_kd = 0.000400;
	}
	else {
		mcconf-> s_pid_kd = 0.000020;
	}

	mc_interface_set_configuration(mcconf_old);
	mc_interface_set_configuration(mcconf);

	mempools_free_mcconf(mcconf);
	mempools_free_mcconf(mcconf_old);
}

void encoder_cal_detection(void) {

	mc_configuration *mcconf = mempools_alloc_mcconf();
	*mcconf = *mc_interface_get_configuration();
	mc_configuration *mcconf_old = mempools_alloc_mcconf();
	*mcconf_old = *mcconf;

	mcconf->motor_type = MOTOR_TYPE_FOC;
	mcconf->foc_f_zv = 10000.0;
	mcconf->foc_current_kp = 0.01;
	mcconf->foc_current_ki = 10.0;
	mc_interface_set_configuration(mcconf);

	float current = 2.0;
	float offset = 0.0;
	float ratio = 0.0;
	bool inverted = false;

	mcpwm_foc_encoder_detect(current, false, &offset, &ratio, &inverted);

	mcconf_old->foc_encoder_offset = offset;
	mcconf->foc_encoder_offset = offset;

	mc_interface_set_configuration(mcconf);
	mc_interface_set_configuration(mcconf_old);

	mempools_free_mcconf(mcconf);
	mempools_free_mcconf(mcconf_old);

	is_encoder_done = true;
}

bool is_hw_fault(void) {
	bool custom_fault = false;

	//TODO: Add a custom fault here.

	return (custom_fault);
}

static void adc_read_callback(void) {

	float filter_knob = 0.0;

	filter_knob = ADC_VOLTS(ADC_IND_EXT);

	UTILS_LP_FAST(Knob_read, filter_knob, 0.1);

}

float get_pfc_temp(void) {
	static float temp_pfc_filtered = 0.0;

	float temp_pfc = (1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_PFC]) / 10000.0) / 3455.0) + (1.0 / 298.15)) - 273.15);
	UTILS_LP_FAST(temp_pfc_filtered, temp_pfc, 0.1);
	return temp_pfc_filtered;
}

void pid_speed(float set_rpm) {
	timeout_reset();
    mc_interface_set_pid_speed(set_rpm); //
}

static void terminal_print_info(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	eeprom_var data_stored, check_cal;

	conf_general_read_eeprom_var_hw(&data_stored, EEPROM_ADDR_ENCODER_VALUE);
	commands_printf("Encoder stored value: %f", (double)(data_stored.as_float));
	conf_general_read_eeprom_var_hw(&check_cal, EEPROM_ADDR_CALIBRATION_CHECK);
	commands_printf("Calibration status: %d", check_cal.as_i32);

	(is_pfc_ok())? commands_printf("PFC:OK") : commands_printf("PFC:OFF");

	commands_printf("ADC: %f", (double)Knob_read);
	commands_printf("ADC_cal: %f", (double)calib);
	commands_printf("Encoder min: %f", (double)encoder_min_value);
	commands_printf("speed: %f", (double)speed_setpoint);
}

float get_knob_read(void) {
	return(calib);
}

static void terminal_motor_run(int argc , const char **argv) {
	(void)argc;
	(void)argv;

	if(strcmp(argv[1], "ON") == 0) {
		commands_printf("Motor ON");
		motor_start = true;
		encoder_cal_detection();
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

    float sw_main = 0.0;
    static systime_t overload_start = 0;
    static systime_t grind_start = 0;
    static int grind_attemp = 0;

    for(;;) {
   // TODO: Add a safety condition, just to avoid undesired behavior when main switch is disconnected.
    	sw_main = ADC_VOLTS(ADC_IND_EXT2);

        if(is_pfc_ok()) {
            palSetPad(PFC_ENABLE_PORT, PFC_ENABLE_PIN);

            if(sw_main > 2.8) {//if(is_sw_position() && is_momentary_position()) {
                   if(is_stop_state) {
            			timeout_reset();
                       	mc_interface_set_pid_speed(0.0);
                       	is_stop_state = false;
                       	is_momentary_position_status = false;
                       	is_motor_stalled_fault = false;
                       	is_motor_grinding_enable = true;
                   	   }
                   }

           if(sw_main > 1.2 && sw_main < 1.6 &&  is_calibration_done && !is_motor_stalled_fault && is_motor_grinding_enable) {// if(!is_sw_position() && is_calibration_done) {
        	   timeout_reset();
        	   mc_interface_set_pid_speed(speed_setpoint);

        	   if(mc_interface_get_tot_current() < NO_GRIND_CURRENT) {
        		   if(grind_start == 0) {
        			   grind_start = chVTGetSystemTime();
        		   }
        		   else {
        			   if (chVTTimeElapsedSinceX(grind_start) > S2ST(GRIND_TIMEOUT)) {
        				   timeout_reset();
        				   mc_interface_set_pid_speed(0.0);
        				   is_motor_grinding_enable = false;
        				   grind_start = 0;
        			   }
        		   }
        	   }
        	   else {
        		   grind_start = 0;
        	   }
        	   is_stop_state = true;
            }

            if(sw_main < 0.4 && is_calibration_done) {//if(!is_momentary_position() && is_calibration_done) { // && is_calibration_done
            	if(!safety_calibration) {
            		if(!is_erpm_done) {
            			is_default_erpm = false;
            			set_erpm_ramp_response();
            		}
            		speed_setpoint = 8000;
            		is_momentary_position_status = true;
            		timeout_reset();
            		mc_interface_set_pid_speed(speed_setpoint);
            		is_stop_state = true;
            	}
            	else {
            	    if(!is_encoder_done) {
            	    	while(!main_init_done()) { // here wait until the whole main configuration finish otherwise the encoder calibration won't work properly.
            	    			chThdSleepMilliseconds(10);
            	    		}
            	    	is_default_erpm = true;
            	    	set_erpm_ramp_response();
            	    	encoder_calibrate_offset();// added here, need to wait for the ADC readings to calibrate the offset
            	    	encoder_cal_detection();// perform the encoder_foc_calibration. Here will perform at first time.
            	    }
            	}
            }
            else {
            	safety_calibration = false;
            	is_erpm_done = false;
            	is_encoder_done = false;
            	if(!is_default_erpm) {
            		is_default_erpm = true;
            		set_erpm_ramp_response();
            	}
            }

            if (mc_interface_get_tot_current() >= CUTOFF_CURRENT) { // perform a overload protection. Set at 4A just to test the algorithm
            	if (overload_start == 0) {
            		overload_start = chVTGetSystemTime();
            	}
            	else {
            		if (chVTTimeElapsedSinceX(overload_start) > MS2ST(CURRENT_MOTOR_TIMEOUT)) {
            			timeout_reset();
            			mc_interface_set_pid_speed(0.0);
            			grind_attemp++;
            			if(grind_attemp == GRIND_ATTEMPS) {
            				is_motor_stalled_fault = true;
            				grind_attemp = 0;
            			}
            			chThdSleepMilliseconds(2000); // wait for seconds and start again.
            			overload_start = 0;
            		}
            	}
            }
            else {
            	overload_start = 0;
            }
        }
        else {
           // palClearPad(PFC_ENABLE_PORT, PFC_ENABLE_PIN); // if pfc is not ok, do nothing...
        }

        chThdSleepMilliseconds(100);
    }
}

static THD_FUNCTION(encoder_thread, arg) {
    (void)arg;

    chRegSetThreadName("encoder_readings");

    chThdSleepMilliseconds(1000);

  for(;;) {

	  float samples[15];
	  float diff;
	  float aux= 0.0;

	  if(!is_momentary_position_status) {
		  for( int i = 0 ; i<15 ; i++) {
			  // Knob_read = ADC_VOLTS(ADC_IND_EXT); // get the knob voltage readings.
			  samples[i] = Knob_read;
			  chThdSleepMilliseconds(25);
		  }

		  for (int i=0; i<15; i++) {
			  if(i!=0) {
				  diff = fabs(samples[i] - samples[i-1]);
				  if(diff > 0.2) {
					  aux = 0.0;
				  }
			  else
				  aux = samples[i-1]- 0.05;
			  }
		  }

		  calib = (encoder_min_value - aux); // enable this to use the encoder calibration

		  if(calib < 0) {
			  calib += 3.22;
		  }

		  speed_setpoint = utils_map(calib, 0.0, 2.9, 800, 6400);
		  speed_setpoint = (round(speed_setpoint/400)*400);
	  }

	  if(speed_setpoint >= 3600) {
		  if(!is_pid_kd_change_up) {
			  set_pid_constant();
			  is_pid_kd_change_up = true;
			  is_pid_kd_change_down = false;
		  }
	  }
	  if(speed_setpoint < 3600) {
		  if(!is_pid_kd_change_down) {
			  set_pid_constant();
			  is_pid_kd_change_down = true;
			  is_pid_kd_change_up = false;
		  }
	  }

	  chThdSleepMilliseconds(10);
  }
}
