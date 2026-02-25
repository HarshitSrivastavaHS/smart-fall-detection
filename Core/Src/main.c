  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) CG2028 Teaching Team
  ******************************************************************************/


/*--------------------------- Includes ---------------------------------------*/
#include "main.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include <math.h>

#include "stdio.h"
#include "string.h"
#include <sys/stat.h>

static void UART1_Init(void);

extern void initialise_monitor_handles(void);	// for semi-hosting support (printf). Will not be required if transmitting via UART

extern int mov_avg(int N, int* accel_buff); // asm implementation

int mov_avg_C(int N, int* accel_buff); // Reference C implementation

UART_HandleTypeDef huart1;

/*
 * Defined Threshold Values
 */
#define FREEFALL_THRESHOLD 3
#define IMPACT_THRESHOLD 9
#define GYRO_THRESHOLD 1000
#define NOT_MOVING_THRESHOLD 600

#define BUTTON_PRESSED 0
#define BUTTON_NOT_PRESSED 1

typedef enum {
		NORMAL,
		FREEFALL,
		IMPACT,
		FALL_CONFIRMED,
		LONG_LIE
} states;

const char* state_to_string(states state){ // helper function for printing current state
    switch(state)	    {
    case NORMAL: return "NORMAL";
    case FREEFALL: return "FREEFALL";
    case IMPACT: return "IMPACT";
    case FALL_CONFIRMED: return "FALL_CONFIRMED";
    case LONG_LIE: return "LONG_LIE";
    default: return "UNKNOWN";
    }
}


int main(void)
{
	const int N=4;

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* UART initialization  */
	UART1_Init();

	/* Peripheral initializations using BSP functions */
	BSP_LED_Init(LED2);
	BSP_ACCELERO_Init();
	BSP_GYRO_Init();
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);

	/*Set the initial LED state to off*/
	BSP_LED_Off(LED2);

	int accel_buff_x[4]={0};
	int accel_buff_y[4]={0};
	int accel_buff_z[4]={0};
	uint32_t i=0;
	int delay_ms=50; // 20Hz delay

	states current_state = NORMAL;
	int high_rotation_detected = 0;
	uint32_t fall_confirmed_time = 0;
	uint32_t button_press_start = 0;
	uint32_t last_led_toggle_timestamp = 0;
	uint32_t now = 0;



	// Setting UP GPIO Pin D7 (PortA, Pin4)
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	while (1)
	{

		// BSP_LED_Toggle(LED2);		// This function helps to toggle the current LED state

		int16_t accel_data_i16[3] = { 0 };			// array to store the x, y and z readings of accelerometer
		/********Function call to read accelerometer values*********/
		BSP_ACCELERO_AccGetXYZ(accel_data_i16);

		//Copy the values over to a circular style buffer
		accel_buff_x[i%4]=accel_data_i16[0]; //acceleration along X-Axis
		accel_buff_y[i%4]=accel_data_i16[1]; //acceleration along Y-Axis
		accel_buff_z[i%4]=accel_data_i16[2]; //acceleration along Z-Axis


		// ********* Read gyroscope values *********/
		float gyro_data[3]={0.0};
		float* ptr_gyro=gyro_data;
		BSP_GYRO_GetXYZ(ptr_gyro);

		//The output of gyro has been made to display in dps(degree per second)
		float gyro_velocity[3]={0.0};
		gyro_velocity[0]=(gyro_data[0]*9.8/(1000));
		gyro_velocity[1]=(gyro_data[1]*9.8/(1000));
		gyro_velocity[2]=(gyro_data[2]*9.8/(1000));


		//Preprocessing the filtered outputs  The same needs to be done for the output from the C program as well
		float accel_filt_asm[3]={0}; // final value of filtered acceleration values

		accel_filt_asm[0]= (float)mov_avg(N,accel_buff_x) * (9.8/1000.0f);
		accel_filt_asm[1]= (float)mov_avg(N,accel_buff_y) * (9.8/1000.0f);
		accel_filt_asm[2]= (float)mov_avg(N,accel_buff_z) * (9.8/1000.0f);


		//Preprocessing the filtered outputs  The same needs to be done for the output from the assembly program as well
//		float accel_filt_c[3]={0};
//
//		accel_filt_c[0]=(float)mov_avg_C(N,accel_buff_x) * (9.8/1000.0f);
//		accel_filt_c[1]=(float)mov_avg_C(N,accel_buff_y) * (9.8/1000.0f);
//		accel_filt_c[2]=(float)mov_avg_C(N,accel_buff_z) * (9.8/1000.0f);

		float accel_filt_mag = sqrtf(accel_filt_asm[0]*accel_filt_asm[0]
								+ accel_filt_asm[1]*accel_filt_asm[1]
								+ accel_filt_asm[2]*accel_filt_asm[2]);

		float gyro_mag = sqrtf(gyro_velocity[0]*gyro_velocity[0]
								+ gyro_velocity[1]*gyro_velocity[1]
								+ gyro_velocity[2]*gyro_velocity[2]);

		/***************************UART transmission*******************************************/
		char buffer[150]; // Create a buffer large enough to hold the text

		/******Transmitting results of C execution over UART*********/
		if(i>=3)
		{
//			// 1. First printf() Equivalent
//			sprintf(buffer, "Results of C execution for filtered accelerometer readings:\r\n");
//			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
//
//			// 2. Second printf() (with Floats) Equivalent
//			// Note: Requires -u _printf_float to be enabled in Linker settings
//			sprintf(buffer, "Averaged X : %f; Averaged Y : %f; Averaged Z : %f;\r\n",
//					accel_filt_c[0], accel_filt_c[1], accel_filt_c[2]);
//			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
//
//			/******Transmitting results of asm execution over UART*********/
//
//			// 1. First printf() Equivalent
//			sprintf(buffer, "Results of assembly execution for filtered accelerometer readings:\r\n");
//			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
//
//			// 2. Second printf() (with Floats) Equivalent
//			// Note: Requires -u _printf_float to be enabled in Linker settings
//			sprintf(buffer, "Averaged X : %f; Averaged Y : %f; Averaged Z : %f;\r\n",
//					accel_filt_asm[0], accel_filt_asm[1], accel_filt_asm[2]);
//			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
//
//			/******Transmitting Gyroscope readings over UART*********/
//
//			// 1. First printf() Equivalent
//			sprintf(buffer, "Gyroscope sensor readings:\r\n");
//			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
//
//			// 2. Second printf() (with Floats) Equivalent
//			// Note: Requires -u _printf_float to be enabled in Linker settings
//			sprintf(buffer, "Averaged X : %f; Averaged Y : %f; Averaged Z : %f;\r\n\n",
//					gyro_velocity[0], gyro_velocity[1], gyro_velocity[2]);
//			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

			sprintf(buffer, "Accelerometer Magnitude: %f\r\n", accel_filt_mag);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
			sprintf(buffer, "Gyro Magnitude: %f\r\n", gyro_mag);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
			sprintf(buffer, "Current State: %s\r\n", state_to_string(current_state));
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

		}
		now = HAL_GetTick();
		switch (current_state) {
		case NORMAL:
			BSP_LED_Off(LED2);
			high_rotation_detected = 0;
			if (accel_filt_mag < FREEFALL_THRESHOLD) {
				current_state = FREEFALL;
			}
			break;
		case FREEFALL:
			if (accel_filt_mag > IMPACT_THRESHOLD) {
				current_state = IMPACT;
			}
			if (gyro_mag > GYRO_THRESHOLD) {
				high_rotation_detected = 1;
			}
			break;
		case IMPACT:
			if(high_rotation_detected == 1) {
				current_state = FALL_CONFIRMED;
				fall_confirmed_time = now;
				last_led_toggle_timestamp = now;
			}
			else {
				current_state = NORMAL;
			}
			break;
		case FALL_CONFIRMED:
			if (now - last_led_toggle_timestamp >= 200) {  // LED Blinking at 2.5Hz
				BSP_LED_Toggle(LED2);
				last_led_toggle_timestamp = now;
			}
			if (gyro_mag > NOT_MOVING_THRESHOLD) {
				fall_confirmed_time = now;
			}
			if (now - fall_confirmed_time >= 10*1000) { //10seconods
				current_state = LONG_LIE;
				last_led_toggle_timestamp = now;
			}
			if (BSP_PB_GetState(BUTTON_USER) == BUTTON_PRESSED) {
				current_state = NORMAL;
			}
			break;
		case LONG_LIE:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			if (now - last_led_toggle_timestamp >= 50) {
				BSP_LED_Toggle(LED2);  // LED Blinking at 10Hz
				last_led_toggle_timestamp = now;
			}
			if (BSP_PB_GetState(BUTTON_USER) == BUTTON_PRESSED) {
				if (button_press_start == 0) {
					button_press_start = now;
				}
				if (now - button_press_start >= 5000) {
					current_state = NORMAL;
					button_press_start = 0;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
				}
			}
			else {
				button_press_start = 0;
			}
			break;
		}


		HAL_Delay(delay_ms);	// 50ms delay

		i++;

	}


}

int mov_avg_C(int N, int* accel_buff)
{ 	// The implementation below is inefficient and meant only for verifying your results.
	int result=0;
	for(int i=0; i<N;i++)
	{
		result+=accel_buff[i];
	}

	result=result/4;

	return result;
}

static void UART1_Init(void)
{
        /* Pin configuration for UART. BSP_COM_Init() can do this automatically */
        __HAL_RCC_GPIOB_CLK_ENABLE();
         __HAL_RCC_USART1_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* Configuring UART1 */
        huart1.Instance = USART1;
        huart1.Init.BaudRate = 115200;
        huart1.Init.WordLength = UART_WORDLENGTH_8B;
        huart1.Init.StopBits = UART_STOPBITS_1;
        huart1.Init.Parity = UART_PARITY_NONE;
        huart1.Init.Mode = UART_MODE_TX_RX;
        huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
        huart1.Init.OverSampling = UART_OVERSAMPLING_16;
        huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
        huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
        if (HAL_UART_Init(&huart1) != HAL_OK)
        {
          while(1);
        }

}


// Do not modify these lines of code. They are written to supress UART related warnings
int _read(int file, char *ptr, int len) { return 0; }
int _fstat(int file, struct stat *st) { return 0; }
int _lseek(int file, int ptr, int dir) { return 0; }
int _isatty(int file) { return 1; }
int _close(int file) { return -1; }
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { return -1; }
