/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef HW_ZERNO_DRIVE_CORE_H_
#define HW_ZERNO_DRIVE_CORE_H_

#define HW_NAME					"ZERNO_DRIVE"
#include "mcconf_zerno_drive.h"
#include "appconf_zerno_drive.h"

// HW properties
#define HW_HAS_3_SHUNTS
//#define HW_HAS_PHASE_SHUNTS
#define HW_HAS_PHASE_FILTERS
#define HW_HAS_NO_CAN
//#define HW_HAS_DRV8313

// Macros
#define LED_GREEN_GPIO			GPIOB
#define LED_GREEN_PIN			4
#define LED_RED_GPIO			GPIOB
#define LED_RED_PIN				5

#define LED_GREEN_ON()			palSetPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_GREEN_OFF()			palClearPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_RED_ON()			palSetPad(LED_RED_GPIO, LED_RED_PIN)
#define LED_RED_OFF()			palClearPad(LED_RED_GPIO, LED_RED_PIN)

#define HW_SW_PORT				GPIOD
#define HW_SW_PIN				2
#define HW_MOMENTARY_PORT		GPIOB
#define HW_MOMENTARY_PIN		6     

// PFC status and enables pin
#define PFC_STATUS_PORT		GPIOB
#define PFC_STATUS_PIN		0
#define PFC_ENABLE_PORT		GPIOC
#define PFC_ENABLE_PIN		5

#define IS_DRV_FAULT()		is_hw_fault()
/*
// For power stages with enable pins (e.g. DRV8313)
#define ENABLE_BR1()			palSetPad(GPIOB, 13)
#define ENABLE_BR2()			palSetPad(GPIOB, 14)
#define ENABLE_BR3()			palSetPad(GPIOB, 15)
#define DISABLE_BR1()			palClearPad(GPIOB, 13)
#define DISABLE_BR2()			palClearPad(GPIOB, 14)
#define DISABLE_BR3()			palClearPad(GPIOB, 15)
#define ENABLE_BR()				palWriteGroup(GPIOB, PAL_GROUP_MASK(3), 13, 7)
#define DISABLE_BR()			palWriteGroup(GPIOB, PAL_GROUP_MASK(3), 13, 0)

#define INIT_BR()				palSetPadMode(GPIOB, 13, \
								PAL_MODE_OUTPUT_PUSHPULL | \
								PAL_STM32_OSPEED_HIGHEST); \
								palSetPadMode(GPIOB, 14, \
								PAL_MODE_OUTPUT_PUSHPULL | \
								PAL_STM32_OSPEED_HIGHEST); \
								palSetPadMode(GPIOB, 15, \
								PAL_MODE_OUTPUT_PUSHPULL | \
								PAL_STM32_OSPEED_HIGHEST); \
								DISABLE_BR();
*/
#define PHASE_FILTER_GPIO		GPIOC
#define PHASE_FILTER_PIN		9
#define PHASE_FILTER_ON()		palSetPad(PHASE_FILTER_GPIO, PHASE_FILTER_PIN)
#define PHASE_FILTER_OFF()		palClearPad(PHASE_FILTER_GPIO, PHASE_FILTER_PIN)


// Sensor port voltage control
#define SENSOR_VOLTAGE_GPIO		GPIOC
#define SENSOR_VOLTAGE_PIN		12
#define SENSOR_PORT_5V()		palSetPad(SENSOR_VOLTAGE_GPIO, SENSOR_VOLTAGE_PIN)
#define SENSOR_PORT_3V3()		palClearPad(SENSOR_VOLTAGE_GPIO, SENSOR_VOLTAGE_PIN)

/*
#define CURRENT_FILTER_GPIO		GPIOD
#define CURRENT_FILTER_PIN		2
#define CURRENT_FILTER_ON()		palSetPad(CURRENT_FILTER_GPIO, CURRENT_FILTER_PIN)
#define CURRENT_FILTER_OFF()	palClearPad(CURRENT_FILTER_GPIO, CURRENT_FILTER_PIN)
*/

// Vdiv en
#define PHASE_VDIV_GPIO			GPIOA
#define PHASE_VDIV_PIN			15

/*
 * ADC Vector
 *
 * 0  (1):	IN0		SENS1
 * 1  (2):	IN1		SENS2
 * 2  (3):	IN2		SENS3
 * 3  (1):	IN10	CURR1
 * 4  (2):	IN11	CURR2
 * 5  (3):	IN12	CURR3
 * 6  (1):	IN5		ADC_EXT1
 * 7  (2):	IN6		ADC_EXT2
 * 8  (3):	IN3		TEMP_MOS
 * 9  (1):	IN14	TEMP_MOTOR
 * 10 (2):	IN15	Shutdown
 * 11 (3):	IN13	AN_IN
 * 12 (1):	Vrefint
 * 13 (2):	IN0		SENS1
 * 14 (3):	IN1		SENS2
 * 15 (1):  IN8		TEMP_MOS_2
 * 16 (2):  IN9		TEMP_MOS_3
 * 17 (3):  IN3		SENS3
 */

#define HW_ADC_CHANNELS			18
#define HW_ADC_INJ_CHANNELS		3
#define HW_ADC_NBR_CONV			6

// ADC Indexes
#define ADC_IND_SENS1			3
#define ADC_IND_SENS2			4
#define ADC_IND_SENS3			5
#define ADC_IND_CURR1			0
#define ADC_IND_CURR2			1
#define ADC_IND_CURR3			2
#define ADC_IND_VIN_SENS		11
#define ADC_IND_EXT				6
#define ADC_IND_EXT2			7
#define ADC_IND_SHUTDOWN		10
#define ADC_IND_TEMP_MOS		8
#define ADC_IND_TEMP_PFC		15 // use for PFC temperature
#define ADC_IND_TEMP_MOTOR		9
#define ADC_IND_VREFINT			12

// ADC macros and settings

// Component parameters (can be overridden)
#ifndef V_REG
#define V_REG					3.3
#endif
#ifndef VIN_R1
#define VIN_R1					660000.0
#endif
#ifndef VIN_R2
#define VIN_R2					5000.0
#endif
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN		20.0
#endif
//Factor correction due to error on shunt resistors.
#ifndef CURRENT_SHUNT_RES
//#define CURRENT_SHUNT_RES		(0.0005 * 0.80)
#define CURRENT_SHUNT_RES		0.005//0.003
#endif

// Input voltage
#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

// NTC Termistors
#define NTC_RES(adc_val)		(10000.0 * adc_val / ( 4095.0 - adc_val))
#define NTC_TEMP(adc_ind)		(1.0 / ((logf(NTC_RES(ADC_Value[adc_ind]) / 10000.0) / 3455.0) + (1.0 / 298.15)) - 273.15)

#define NTC_RES_MOTOR(adc_val)	(10000.0 / ((4095.0 / (float)adc_val) - 1.0))
#define NTC_TEMP_MOTOR(beta)	(1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)

// log PFC temperature data
#define NTC_TEMP_MOS2()			get_pfc_temp()

// log the knob adc input

//#define NTC_TEMP_MOS3()			get_knob_read()

// Voltage on ADC channel
#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4096.0 * V_REG)

// COMM-port ADC GPIOs
#define HW_ADC_EXT_GPIO			GPIOA
#define HW_ADC_EXT_PIN			5
#define HW_ADC_EXT2_GPIO		GPIOA
#define HW_ADC_EXT2_PIN			6

// UART Peripheral
#define HW_UART_DEV				SD3
#define HW_UART_GPIO_AF			GPIO_AF_USART3
#define HW_UART_TX_PORT			GPIOB
#define HW_UART_TX_PIN			10
#define HW_UART_RX_PORT			GPIOB
#define HW_UART_RX_PIN			11

// ICU Peripheral for servo decoding
#define HW_USE_SERVO_TIM4
#define HW_ICU_TIMER			TIM4
#define HW_ICU_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ICU_DEV				ICUD4
#define HW_ICU_CHANNEL			ICU_CHANNEL_1
#define HW_ICU_GPIO_AF			GPIO_AF_TIM4
#define HW_ICU_GPIO				GPIOB
#define HW_ICU_PIN				6

// I2C Peripheral
#define HW_I2C_DEV				I2CD2
#define HW_I2C_GPIO_AF			GPIO_AF_I2C2
#define HW_I2C_SCL_PORT			GPIOB
#define HW_I2C_SCL_PIN			10
#define HW_I2C_SDA_PORT			GPIOB
#define HW_I2C_SDA_PIN			11

//Change this to other pins, 
// IMU LSM6DS3
/*
#define LSM6DS3_SDA_GPIO		GPIOB
#define LSM6DS3_SDA_PIN			11
#define LSM6DS3_SCL_GPIO		GPIOB
#define LSM6DS3_SCL_PIN			10
*/

// Hall/encoder pins
#define HW_HALL_ENC_GPIO1		GPIOC
#define HW_HALL_ENC_PIN1		6
#define HW_HALL_ENC_GPIO2		GPIOC
#define HW_HALL_ENC_PIN2		7
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		8
#define HW_ENC_TIM				TIM3
#define HW_ENC_TIM_AF			GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource8
#define HW_ENC_EXTI_CH			EXTI9_5_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line8
#define HW_ENC_EXTI_ISR_VEC		EXTI9_5_IRQHandler
#define HW_ENC_TIM_ISR_CH		TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM3_IRQHandler

// SPI pins
#define HW_SPI_DEV				SPID3
#define HW_SPI_GPIO_AF			GPIO_AF_SPI3
#define HW_SPI_PORT_NSS			GPIOA
#define HW_SPI_PIN_NSS			15
#define HW_SPI_PORT_SCK			GPIOC
#define HW_SPI_PIN_SCK			10
#define HW_SPI_PORT_MOSI		GPIOC
#define HW_SPI_PIN_MOSI			12
#define HW_SPI_PORT_MISO		GPIOC
#define HW_SPI_PIN_MISO			11

// Measurement macros
#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

#define HW_DEAD_TIME_NSEC		250.0

// Setting limits
#define HW_LIM_CURRENT			-8.0, 8.0
#define HW_LIM_CURRENT_IN		-6.0, 6.0
#define HW_LIM_CURRENT_ABS		0.0, 10.0
#define HW_LIM_VIN				100.0, 450.0
#define HW_LIM_ERPM				-60e3, 60e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.99
#define HW_LIM_TEMP_FET			-40.0, 110.0

// Functions
bool is_momentary_position(void);
bool is_sw_position(void);
bool is_hw_fault(void);
float get_pfc_temp(void);
float get_adc_filtered(void);
float get_knob_read(void);


#endif/* HW_ZERNO_DRIVE_CORE_H_ */
