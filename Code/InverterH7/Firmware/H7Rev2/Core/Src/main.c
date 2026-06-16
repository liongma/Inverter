/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

//#include "can.h"
//#include "mtpva.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */




/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
char msg[400] = {};
uint16_t iu = 0;
uint16_t iv = 0;
uint16_t iw = 0;


uint32_t motor = 0;

uint16_t hallOffset[3] = {0};
uint16_t hallValue[3] = {0};

uint32_t adcOffset[10] = {0};
uint32_t adcValue[10] = {0};
uint32_t adc2_value[10] = {0};
uint32_t adc3_value[10] = {0};

float AMK_temp = 0;
float board_temp[3] = {0};
float i_phase[3] = {0,0,0};
uint32_t tI = 0;
float freq = 0;
double Ts = 0.0001;

float theta = 0;
float thetaI = 0;
float omega = 0;

float v_bus = 0;
float power = 0;
float theta_e = 0;
float test_angle_1 = 0;;
float theta_e_f = 0;
float theta_e_1 = 0;
float omega_e = 0;
float omega_e_1 = 0;
float accel_e = 0;
float acl_e = 0;
float accel_e_1 = 0;
float jerk_e = 0;
float jrk_e = 0;

float theta_e_est = 0;

DQ_Components integral = {0,0};
DQ_Components error = {0,0};
float integral_w;
float error_w;

DQ_Components ref = {.d = 0 , .q = 0};
DQ_Components ref_aw = {.d = 0 , .q = 0};

DQ_Components v = {.d = 0 , .q = 0};
DQ_Components v_sat = {.d = 0 , .q = 0};
DQ_Components aw = {.d = 0 , .q = 0};

const uint8_t au8PermuataionMatrix[6][3] =
{
	{ 1, 2, 0 },
	{ 3, 1, 0 },
	{ 0, 1, 2 },
	{ 0, 3, 1 },
	{ 2, 0, 1 },
	{ 1, 0, 3 }
};

int8_t can_transmit[8] = {0};
uint8_t can_recieve[8]  = {0};
FDCAN_RxHeaderTypeDef RxHeader = {0};

float ref_w = 10.0;
int32_t pwm_max =8192; //15.5kHz
#define PWM_MAX  8192
float cmd = 0;
uint32_t dt = 0;
float af = 0;
uint32_t current_time;

DQ_Components error_i = {.d = 0, .q = 0}; // V/A
DQ_Components dError = {.d = 0, .q = 0}; // V/A

float error_w_i = 0;

float vmag = 0;
float imag = 0;

float i_sat = 0;
float i_set = 0;

float psi_c = 0;



DQ_Components Kp = {.d = 2, .q = 3}; // V/A
DQ_Components Ki = {.d = 100, .q = 120}; // V/A
DQ_Components Kd = {.d = 0.0001, .q = 0.0001}; // V/A

DQ_Components If = {.d = 0, .q = 0}; // V/A
DQ_Components dI = {.d = 0, .q = 0}; // delta V/A


DQ_Components i_dq = {.d = 0, .q = 0};

DQ_Components f1 = {.d = 0, .q = 0};
DQ_Components f2 = {.d = 0, .q = 0};

AlphaBeta_Components v_static = {.alpha  = 0, .beta = 0};

DQ_Components i_dq_k1_hat = {.d = 0, .q = 0};
DQ_Components est_error = {.d = 0, .q = 0};

DQ_Components integral_est = {.d = 0, .q = 0};

float test_time = 0;



float test_angle = 0;

float w_e = 0;
float w_e_est = 0;
int8_t w_sign = 1;

float Rs_det = 0.1000f;//0.0675;
float Ld_det = 0.00024f;
float Lq_det = 0.0003f;
float Psi_det = 0.0285f;
float dV_det = 0.0f;


uint8_t inverter_state = 10;
uint8_t inverter_fault = 0;
uint16_t inverter_error = 0;

float residual = 0;
float k_alpha = 0.4;
float k_beta = 0.15;
float k_gamma = 0.03;

float Lq_f = 0.0;
float Ld_f = 0.0;
float Psi_f = 0.0;

uint8_t can_timer = 0;
can_msg_1 can_1;
can_msg_2 can_2;
can_msg_3 can_3;
float sch = 1.0f;

float cmd_d = 0;
float cmd_q = 0;

float flux_q = 0;
float flux_d = 0;

uint8_t can_rx_fail = 0;


float pwm_period = PWM_MAX/127;
inverter_cmd_decoded_t vcu_cmd;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CORDIC_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */





void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CORDIC_Init();
  MX_FDCAN1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();
  MX_ADC3_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  DISABLE_OUT;
  can_init();

  HAL_TIM_Base_Start(&htim16);
  __HAL_TIM_SET_AUTORELOAD(&htim1, pwm_max);
  TIM1->CNT = 0;
  TIM1->CCR4 = pwm_max-1000;
//  DISABLE_OUT;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  UPHASE = 0;
//  UPHASE = 0;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  VPHASE = 0;
//  VPHASE = 0;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  WPHASE = 0;
//  WPHASE = 0;
  HAL_ADC_Start_DMA(&hadc1, adcValue, 5);
  HAL_ADC_Start_DMA(&hadc2, adc2_value, 2);
  HAL_ADC_Start_DMA(&hadc3, adc3_value, 2);


  HAL_Delay(300);
//  callibrateADC(&hadc1,adcValue, adcOffset);
  adcOffset[0] = 8355;
  adcOffset[1] = 8317;
  adcOffset[2] = 8317;
  adcOffset[3] = 8317;
  adcOffset[4] = 500;
  sprintf(msg,"%lu, %lu, %lu, %lu, %lu \n", adcOffset[0],adcOffset[1],adcOffset[2],adcOffset[3], adcOffset[4]);
  for(int i = 0; i < 5; i++){
	  HAL_UART_Transmit_DMA(&huart1, (uint8_t*)msg, strlen(msg));
	  HAL_Delay(100);

  }

  appMath_Init();
  fault_init();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#ifdef SERIAL_PRINT
  uint32_t printCounter = 0;
#endif
#ifdef CURRENT_SWEEP

	uint8_t swp_state = 0;


	int8_t swp_step_r = -1; //
	int8_t swp_step_c = 1;

	uint32_t swp_cnt_r = 0;

#endif

	int32_t cooldown = 3000;
	int8_t enc_failure = 0;
  while (1)
  {
	  //===========================================
	  //===========================================
	  //       WHILE LOOP BEGIN
	  //===========================================
	  //===========================================


#ifdef DUMMY
#ifdef DUMMY_TESTPWM
	  ENABLE_OUT;
	  ACTIVE_OUT;
	  UPHASE = 1000;
	  VPHASE = 1000;
	  WPHASE = 1000;
	  continue;
#else
	  DISABLE_OUT;
	  IDLE_OUT;
	  UPHASE = 0;
	  VPHASE = 0;
	  WPHASE = 0;

  break;
#endif
#endif

//#if defined(RS_DET) || defined(LQ_DET) || defined(LD_DET) || defined(PSI_DET) || defined(LQ_DET_HF)
  run_detect();
//#endif

	//Failure Buffer

	// Frequency Determination

	current_time = __HAL_TIM_GET_COUNTER(&htim16);
	dt = current_time;
	while(dt < pwm_period){
		current_time = __HAL_TIM_GET_COUNTER(&htim16);
		dt = current_time;
	}
	TIM16->CNT = 0;
	Ts = (dt * 0.000001f);
//	Ts = 0.00005f;
//	Ts = 0.00005f;
	if(Ts != 0) freq = 1/Ts;

	// Encoder
	readEncoder(&motor);

	theta = normalizeAngle((motor)*ENDAT2RAD);
	omega = shortenAngle(theta - thetaI)*(freq);
	thetaI = theta;
	theta_e = normalizeAngle((theta) * 5);


  	fault_buffer();


#ifdef KALMAN

	//Discrete
	omega_e = shortenAngle(theta_e - theta_e_1)*freq;
	accel_e = (omega_e - w_e)*freq;
	jerk_e = (accel_e - acl_e)*freq;
	jrk_e = jerk_e*0.1 + 0.9*jrk_e;


	//Kalman
	theta_e_est = normalizeAngle(theta_e_1 + w_e*Ts + 0.5f*acl_e*Ts*Ts);
	w_e_est = w_e + acl_e*Ts;

	residual = shortenAngle(theta_e - theta_e_est);

	theta_e_f = normalizeAngle(theta_e_est + k_alpha * residual);
	w_e = w_e_est + k_beta  * residual * freq;
	acl_e = acl_e + k_gamma * residual * freq * freq;


	if(fabs(residual) > 0.01){ //0.5 degrees of error
		theta_e = theta_e_est;
//		if(inverter_state > 0) fault_dump_output(209);
	}

	test_angle = theta_e;

	//Z-1
	theta_e = theta_e_f;
	theta_e_1 = theta_e;


#else
	test_angle = theta_e;

	fault_if_over_predict();
	fault_if_encoder();


	omega_e = shortenAngle(test_angle - test_angle_1)*freq;
	accel_e = (omega_e - omega_e_1);
	jerk_e = (accel_e - accel_e_1);

	if(fabs(omega_e) > 12000 || fabs(accel_e) > 2000){
		test_angle = theta_e_est;
		omega_e = shortenAngle(test_angle - test_angle_1)*freq;
		accel_e = (omega_e - omega_e_1);
		jerk_e = (accel_e - accel_e_1);
		enc_failure++;
		if(enc_failure > 10){
			fault_manual();
		}
	}
	enc_failure = 0;


	w_e = omega_e*0.05 + w_e*0.95;
	acl_e = accel_e*0.05 + 0.95*acl_e;
	jrk_e = jerk_e*0.05 + 0.95*jrk_e;

	//Z-1
	test_angle_1 = test_angle;
	theta_e_1 = theta_e;
	omega_e_1 = omega_e;
	accel_e_1 = accel_e;



#endif


	fault_if_overspeed();
	fault_if_overaccel();
	fault_if_overjerk();

	if(w_e >= 0) w_sign = 1;
	else w_sign = -1;

	//Command
#ifdef THROTTLE
	cmd = 0.2*pollRef(adcValue, adcOffset)+0.8*cmd;
#endif

	//Get VI
	pollVI(adcValue, adcOffset,  &v_bus, i_phase);
	float v_limit = v_bus*0.57735026919f;



#ifdef SPEED_CONTROL 	//SPEED CONTROL LOOP
	speed_control();
#endif


	// Current Controller
	AlphaBeta_Components i_ab = clarkeTransform(i_phase[0], i_phase[1], i_phase[2]);
	i_dq = parkTransform(i_ab.alpha, i_ab.beta, test_angle);

	pollAMKT(adc3_value, &AMK_temp);
	pollPTC(adc2_value,adc3_value, board_temp);

//	membuf[0] = (int)(theta_e*100);//omega_e;//(int)(v_bus);

	//FAULT DETECTION

	fault_if_undervolt();
	fault_if_overvolt();
//	fault_if_overcurrent();
	fault_if_low_power();
	fault_if_slow();
//	fault_if_voltage_drop();
	fault_if_diverge();
#ifndef IGNORE_EXTERN
	fault_if_extern();
	fault_if_can_dead();
#endif

	warn_if_overvolt();
	warn_if_undervolt();
	warn_if_voltage_drop();
	warn_if_overtemp_board();
	warn_if_undertemp_board();
	warn_if_overtemp_motor();
	warn_if_undertemp_motor();
	warn_if_lowpower();

	//Parameter Gather
	Psi_f = Psi_det*0.01 + 0.99*Psi_det; //0.028;//Psi_det;//0.028;// 0.0343f;//0.0359;// v/(rad/s) (0.26/5)
	Lq_f = 0.00025f; //estimate_Lq(If.q, If.d)*0.01 + 0.99*Lq_f;
	Ld_f = 0.00025f; //estimate_Ld(If.q, If.d)*0.01 + 0.99*Ld_f;
	float R = 0.1000;

	float Lq = Lq_f;
	float Ld = Ld_f;
	float psi = Psi_f;

	compute_Fd_Fq(If.d, If.q, &flux_d, &flux_q);


#if defined(CURRENT_CONTROL)
	#if defined(DYNO_CONTROL)
	remote_rx();
	#endif



	float max_i = vcu_cmd.current_out_limit_q;

//	ref.q = cmd*max_i*mtpva_q(cmd*max_i,omega_e,v_bus);
//	ref.d = cmd*max_i*mtpva_d(cmd*max_i,omega_e,v_bus);
	float in_current_limit_p = vcu_cmd.current_in_limit_p + 0.01;
	float in_current_limit_n = vcu_cmd.current_in_limit_n + 0.01;




    float i_step_size_d = 0.3;
    float i_step_size_q_i = 0.3;
    float i_step_size_q = 0.3;

#ifdef DYNO_CONTROL_THROTTLE_DIRECT
    cmd_d = -0.1*cmd*max_i;
    cmd_q = 1*cmd*max_i;
#endif

	if(cmd_q < -0.5){
		in_current_limit_p = vcu_cmd.current_in_limit_n + 0.01 ;
		in_current_limit_n = vcu_cmd.current_in_limit_p + 0.01;

	}

    clip(cmd_q,-max_i,max_i);
    clip(cmd_d,-max_i,max_i);

#ifdef CW
    cmd_d = -cmd_d;
    cmd_q = -cmd_q;
#else
    cmd_d = cmd_d;
    cmd_q = cmd_q;
#endif

    float cc_scale = 1;
	float cc_v_d = 2*R*cmd_d;
	float cc_v_q = 2*R*cmd_q;
	cc_v_d = R*cmd_d - omega_e*flux_q;
	cc_v_q = R*cmd_q + omega_e*flux_d;


    power = 1.5f * (cc_v_d*cmd_d + cc_v_q*cmd_q);

    float input_current = power/v_bus;

    if(input_current > in_current_limit_p) cc_scale = fabs(in_current_limit_p/input_current);
    if(input_current < -in_current_limit_n)cc_scale = fabs((in_current_limit_n+0.01)/input_current);

    cmd_d = cmd_d*cc_scale;
    cmd_q = cmd_q*cc_scale;


    if(fabs(cmd_d) > 2){
		if(cmd_d > ref.d+i_step_size_d)ref.d+=i_step_size_d;
		else if(cmd_d < ref.d-i_step_size_d)ref.d-=i_step_size_d;
    }
    else{
    	ref.d = cmd_d;

    }

    if(fabs(cmd_q) > 2){
		if(cmd_q > ref.q+i_step_size_q_i)ref.q+=i_step_size_q_i;
		else if(cmd_q < ref.q-i_step_size_q)ref.q-=i_step_size_q;
		else if(ref.q < i_step_size_q_i && ref.q > -i_step_size_q && cmd_q == 0)ref.q = 0;
    }
    else{
    	ref.q = cmd_q;


    }



#endif

#ifdef CURRENT_SWEEP
    if(cmd > 0.5){
    	//snake scan

		float step_count_max = 15000;
		int swp_rows = 10;//starts at mid
		int swp_cols = 5;//starts at 0

		int swp_mult = 5;

		if(swp_cnt_r < step_count_max/2){
			printCounter = 0;
		}

		if(swp_cnt_r++ > step_count_max){// D AXIS
			swp_cnt_r = 0;
			switch(swp_state){
				case 0://increasing
					swp_step_r++;
					if(swp_step_r > swp_rows) swp_state = 1;
					break;
				case 1://decreasing
					swp_step_r--;
					if(swp_step_r < -swp_rows) swp_state = 2;
					break;
				case 2://increasing again
					swp_step_r++;
					if(swp_step_r > 0){
						swp_state = 0;
						swp_step_c ++;
						swp_step_r = 0;
						if(swp_step_c > swp_cols){
							swp_step_c = 0;
							swp_state = 3;
						}
					}
					break;
				case 3://increasing
					swp_step_r++;
					if(swp_step_r > swp_rows) swp_state = 4;
					break;
				case 4://decreasing
					swp_step_r--;
					if(swp_step_r < -swp_rows) swp_state = 5;
					break;
				case 5://increasing again
					swp_step_r++;
					if(swp_step_r > 0){
						swp_state = 3;
						swp_step_c--;
						swp_step_r = 0;
						if(swp_step_c < -swp_cols){
							swp_step_c = 0;
							swp_state = 0;
						}
					}
					break;
			}
		}
//		ref.q = (swp_step_r * swp_mult);
//		if(swp_step_c != 0){
//			ref.d = (swp_step_c * swp_mult);
//		}
//		else{
//			ref.d = 3;
//		}
		ref.q = 5;
		ref.d = swp_step_r*swp_mult;
    }else{
    	ref.d = 0;
    	ref.q = 0;
    }
#endif

#ifdef HF_INJECTION
    float cmd_amp_inj = cmd * 100000;
    float out_inj = cmd_amp_inj*sin_inj;
    if(cmd_amp_inj > amp_inj) out_inj = amp_inj*sin_inj;
#if defined(LD_DET)
    ref.d += out_inj;
#elif defined(LQ_DET_HF)
    ref.q += out_inj;
#endif

#endif


    dI.d = i_dq.d - If.d;
    dI.q = i_dq.q - If.q;


#ifdef CURRENT_FILTER
	If.d = i_dq.d*0.9f + If.d*0.1;
	If.q = i_dq.q*0.9f + If.q*0.1;
#else
	If.d = i_dq.d;
	If.q = i_dq.q;
#endif



	// -------------
	// CONTROL LOOP
	// -------------


	DQ_Components v_dq_vn1 = {
			.q = v_sat.q,
			.d = v_sat.d
	};

	est_error.q = (If.q - i_dq_k1_hat.q)*0.1 + est_error.q*0.9;
	est_error.d = (If.d - i_dq_k1_hat.d)*0.1 + est_error.d*0.9;

	float ki_pre = 0;


	integral_est.q += ki_pre * Ts * est_error.q;
	integral_est.d += ki_pre * Ts * est_error.d;

	i_dq_k1_hat.q = Ts/Lq * (v_dq_vn1.q - R*If.q - omega_e * (Ld * If.d + psi)) + If.q + est_error.q + integral_est.q;
	i_dq_k1_hat.d = Ts/Ld * (v_dq_vn1.d - R*If.d + omega_e * (Lq * If.q)) + If.d + est_error.d + integral_est.d;

	if(fabs(w_e) < 10.0f){
		i_dq_k1_hat.q = If.q;
		i_dq_k1_hat.d = If.d;
		est_error.q = 0;
		integral_est.q = 0;
		est_error.d = 0;
		integral_est.d = 0;
	}



	error.d = (ref.d - i_dq_k1_hat.d);
	error.q = (ref.q - i_dq_k1_hat.q);



//	error.d = (ref.d - (i_dq.d*0 + If.d*1));
//	error.q = (ref.q - (i_dq.q*0 + If.q*1));


//	Ki.d = 50* af;//50
///	Ki.q = 25* af;//50
//	Ki.d = 50;
//	Ki.q = 25;
	Ki.q = 150.0;

//	if(imag > 20.0f && fabs(omega_e) > 1000){
//		float T_int = (1.0f + 3.0f * (imag - 20.0f));
//		float scalar= (1.0f - 0.8f * (imag - 20.0f));
//
//		Ki.q = 100.0f * scalar/T_int;
//	}
	Ki.d = 150.0;
//	Ki.d = 0;
//	Ki.q = 0;

	integral.d += error.d * Ts * Ki.d;
	integral.q += error.q * Ts * Ki.q;

	float integral_limit = v_bus;

	if(integral.d > integral_limit) integral.d = integral_limit;
	if(integral.d < -integral_limit) integral.d = -integral_limit;

	if(integral.q > integral_limit) integral.q = integral_limit;
	if(integral.q < -integral_limit) integral.q= -integral_limit;


#ifdef DBPFC

  	//robustness: precalculate and normalize prediction angles
  	float angle_k1 = normalizeAngle(test_angle + omega_e * safe_Ts);
  	float angle_k2 = normalizeAngle(test_angle + 2.0f * omega_e * safe_Ts);

	AlphaBeta_Components v_ab_k = {.alpha = v_static.alpha, .beta = v_static.beta};

	AlphaBeta_Components psi_ab_k1 = inverseParkTransform(
			safe_Ld * If.d + psi,
			safe_Lq * If.q,
			(test_angle)
	);

	psi_ab_k1.alpha += safe_Ts*v_ab_k.alpha - R*safe_Ts*i_ab.alpha;
	psi_ab_k1.beta  += safe_Ts*v_ab_k.beta  - R*safe_Ts*i_ab.beta;

	DQ_Components i_dq_k1 = parkTransform(
			psi_ab_k1.alpha,
			psi_ab_k1.beta,
			(angle_k1) //use normalized angle k1
	);

	i_dq_k1.d = (i_dq_k1.d - psi)/safe_Ld;
	i_dq_k1.q = (i_dq_k1.q)/safe_Lq;

	AlphaBeta_Components psi_ab_k2 = inverseParkTransform(
			safe_Ld * ref.d + psi,
			safe_Lq * ref.q,
			(angle_k2) //use normalized angle k2
	);

	AlphaBeta_Components v_ab_k1 = 	inverseParkTransform(
			R*i_dq_k1.d,
			R*i_dq_k1.q,
			(angle_k1) //use normalized angle k1
	);

  	//calculate required deadbeat voltage vector
	v_ab_k1.alpha += (psi_ab_k2.alpha - psi_ab_k1.alpha)/safe_Ts;
	v_ab_k1.beta  += (psi_ab_k2.beta - psi_ab_k1.beta)/safe_Ts;

  	//fidelity: Hexagon Voltage Saturation (paper sec.IV.C)
	//v_static = sat_ab_circle(v_ab_k1,v_limit); // replaced
  	//v_static = sat_ab_hexagon(v_ab_k1, v_limit);
  	//pass v_bus directly, NOT v_limit (which is v_bus * 0.577)
  	v_static = sat_ab_hexagon(v_ab_k1, v_bus);

#elif defined(FOC)

	Kd.q = 0.000;
	Kd.d = 0.000;
	dError.q = (error.q - error_i.q)*0.1 + 0.9*dError.q;
	dError.d = (error.d - error_i.d)*0.1 + 0.9*dError.d;

	error_i.q = error.q;
	error_i.d = error.d;

	Kp.q = 0.64;
	Kp.d = 0.58;
	sch =1;
//	if(imag > 65.0f){
//		sch = 0.2;
//	}
//	else if(imag > 20.0f){
//		sch = (1 - 0.016*(imag - 20.0f));
//	}

	v.q = If.q*R + sch*Kp.q * (ref.q - If.q) + sch*integral.q + omega_e * (flux_d);// *(Ld*If.d + psi);
	v.d = If.d*R + 1.0*Kp.d * (ref.d - If.d) + 1.0*integral.d - omega_e * (flux_q);// * Lq*If.q;

#elif defined(DBPCC)
	sch =0.8;

  	// robustness: clamp the instantaneous error to prevent massive dI/dt voltage spikes
  	// at 15.5kHz, the current cannot exceed a few amps for one cycle
  	// if ref steps massively, we feed the error to the deadbeat controller in safe chunks
  	float max_error_step = 3.0f; // tune: max amps of error trusted in a single cycle
  	float safe_error_q = fminf(fmaxf(error.q, -max_error_step), max_error_step);
  	float safe_error_d = fminf(fmaxf(error.d, -max_error_step), max_error_step);

  	// robustness: scale feedforward terms at high speeds
  	// prevents minor parameter mismatch (Ld, Lq, Psi) from dominating the voltage output
  	float ff_scale = 1.0f;
  	if(fabs(omega_e) > 1000.0f){
  		ff_scale = 0.90f; // tune: 0.85f to 0.95f to back off the feedforward dominance
  	}

	//DBPCC
//	v.q = R*If.q + Lq/Ts * (ref.q - If.q) + omega_e *(Ld * If.d + psi);
//	v.d = R*If.d + Ld/Ts * (ref.d - If.d) - omega_e * Lq * If.q;

	///DBPCC - ESO
//	v.q = R*If.q + sch*Lq/Ts * (ref.q - If.q) + sch*integral.q + omega_e *(Ld * If.d + psi);
//	v.d = R*If.d + sch*Ld/Ts * (ref.d - If.d) + sch*integral.d - omega_e * Lq * If.q;

	//DBPCC - 1SD
//	v.q = If.q*R + Lq/Ts * error.q + omega_e *(Ld*(i_dq_k1_hat.d + est_error.d + integral_est.d) + psi);
//	v.d = If.d*R + Ld/Ts * error.d - omega_e * Lq*(i_dq_k1_hat.q + est_error.q + integral_est.q);


	//DBPCC-ESO-1SD
//	v.q = If.q*R + sch*Lq/Ts * error.q + sch*integral.q + omega_e *(Ld*(i_dq_k1_hat.d + est_error.d + integral_est.d) + psi);
//	v.d = If.d*R + sch*Ld/Ts * error.d + sch*integral.d - omega_e * Lq*(i_dq_k1_hat.q + est_error.q + integral_est.q);

	//DBPCC_ESO-1SD-SCH
	if(fabs(omega_e) < 1000){
		// low speed: standard PI-like behavior with feedforward
		v.q = If.q*R + sch*Kp.q * (ref.q - If.q) + sch*integral.q + omega_e *(Ld*If.d + psi);
		v.d = If.d*R +  1 *Kp.d * (ref.d - If.d) +  1 *integral.d - omega_e * Lq*If.q;
	}
	else{
		// high speed: DBPCC with 1-step delay compensation and ESO
		float v_q_ff = omega_e * (Ld * (i_dq_k1_hat.d + est_error.d + integral_est.d) + psi);
		float v_d_ff = -omega_e * Lq * (i_dq_k1_hat.q + est_error.q + integral_est.q);

		v.q = If.q*R + sch*Lq/Ts * error.q + sch*integral.q + (ff_scale * v_q_ff);
		v.d = If.d*R + sch*Ld/Ts * error.d + sch*integral.d + (ff_scale * v_d_ff);
	}


#endif


//	v.d = Ld/Ts * ref.d + R * f1.d - f2.d;
//	v.q = Lq/Ts * ref.q + R * f1.q - f2.q + omega_e*psi;
//		v.d = R*f1.d + Ld/Ts * (ref.d - f1.d) - f2.d;
//		v.q = R*f1.q + Lq/Ts * (ref.q - f1.q) + omega_e * psi - f2.q;

#if !defined(DBPFC)

	v_sat = sat_dq_circle(v, v_limit);


	float K = 2.0f*sinf(omega_e*Ts*0.5f)/(omega_e*Ts);

	if(fabs(omega_e) > 1000){
		v_static = inverseParkTransform(v_sat.d, v_sat.q, normalizeAngle(test_angle + 1.5f * omega_e * Ts));
		v_static.alpha = v_static.alpha/K ;
		v_static.beta = v_static.beta/K ;
	}
	else{
		v_static = inverseParkTransform(v_sat.d, v_sat.q, test_angle);
	}

//	v_static.alpha = 1;
//	v_static.beta = 0;

#endif

//	vmag = hypotf(v_sat.d,v_sat.q);
	imag = hypotf(i_dq.d,i_dq.q);
	power = 1.5*((v_sat.d*If.d) + (v_sat.q*If.q))*0.01 + power*0.99;
    float Kaw_i= 1;

    aw.d = (v_sat.d - v.d);
    aw.q = (v_sat.q - v.q);

	integral.d += aw.d * Ts * Kaw_i * Ki.d;
	integral.q += aw.q * Ts * Kaw_i * Ki.q;


//	ThreePhase_Components v_uvw = inverseClarkeTransform(v_alpha_beta.alpha, v_alpha_beta.beta);

	// Scale to PWM

//	int U = (int)(fmaxf(-PWM_MAX/2.0, fminf(PWM_MAX/2,v_uvw.a*V2PWM))) + pwm_max/2 ;
//	int V = (int)(fmaxf(-PWM_MAX/2.0, fminf(PWM_MAX/2,v_uvw.b*V2PWM))) + pwm_max/2 ;
//	int W = (int)(fmaxf(-PWM_MAX/2.0, fminf(PWM_MAX/2,v_uvw.c*V2PWM))) + pwm_max/2 ;



	//------------------------
	//STATE MACHINE
	//------------------------

	if(inverter_state == 10){ //STARTUP
		IDLE_OUT;
		DISABLE_OUT;
		cooldown --;
		if(cooldown <= 0) inverter_state = 11;
	}
	else if(inverter_state == 11){ //IDLE-LV
		IDLE_OUT;
		DISABLE_OUT;
		UPHASE = 0;
		VPHASE = 0;
		WPHASE = 0;
		HAL_GPIO_WritePin(PROBE_GPIO_Port, PROBE_Pin, 1);
		HAL_GPIO_WritePin(PROBE_GPIO_Port, PROBE_Pin, 0);
		integral.d = 0;
		integral.q = 0;
		integral_w = 0;
		f1.d = 0;
		f2.d = 0;
		f1.q = 0;
		f2.q = 0;
		integral_est.d = 0;
		integral_est.q = 0;
#ifdef THROTTLE
		inverter_state = 0;
#endif
		if(vcu_cmd.state & VCU_STATE_IDLE2DRIVE){
			inverter_state = 0;
		}
	}
	else if(inverter_state == 0){ //IDLE-HV
		IDLE_OUT;
		DISABLE_OUT;
		UPHASE = 0;
		VPHASE = 0;
		WPHASE = 0;
		HAL_GPIO_WritePin(PROBE_GPIO_Port, PROBE_Pin, 1);
		HAL_GPIO_WritePin(PROBE_GPIO_Port, PROBE_Pin, 0);
		integral.d = 0;
		integral.q = 0;
		integral_w = 0;
		f1.d = 0;
		f2.d = 0;
		f1.q = 0;
		f2.q = 0;
		integral_est.d = 0;
		integral_est.q = 0;

		if(vcu_cmd.state & VCU_STATE_DRIVE2IDLE){
			inverter_state = 11;
		}
		else if((fabs(ref.q) > 0.2 || fabs(ref.d) > 0.2 || fabs(w_e) > 5 || fabs(cmd) > 0.05) && v_bus > 5.0f)
			inverter_state = 1;
	}
	else if(inverter_state == 1){// RAMP UP
		ENABLE_OUT;
		ACTIVE_OUT;
		UPHASE += 10;
		VPHASE += 10;
		WPHASE += 10;
		integral.d = 0;
		integral.q = 0;
		integral_w = 0;
		f1.d = 0;
		f2.d = 0;
		f1.q = 0;
		f2.q = 0;
		integral_est.d = 0;
		integral_est.q = 0;
		if(UPHASE >= pwm_max/2)
			inverter_state = 2;
	}
	else if(inverter_state == 2){ // TRACTION
		ENABLE_OUT;
		ACTIVE_OUT;

		float afTi[4] = {0};

		float v_mag = hypotf(v_static.beta,v_static.alpha);
		float v_ang = normalizeAngle(atan2f(v_static.beta,v_static.alpha));

		float v_norm = v_mag/v_limit;
		uint8_t v8_sector = (uint8_t)(v_ang * M_3_OVER_PI);
		float v8_alpha =  normalizeAngle(v_ang - v8_sector * M_PI3);

		float Tb1 = v_norm * sinf(M_PI3 - v8_alpha);
		float Tb2 = v_norm * sinf(v8_alpha);

		afTi[0] = (1.0f - Tb1 - Tb2)*0.5f;
		afTi[1] = Tb1 + Tb2 + afTi[0];
		afTi[2] = Tb2 + afTi[0];
		afTi[3] = Tb1 + afTi[0];

		int32_t u_aim = (uint32_t)(pwm_max * afTi[au8PermuataionMatrix[v8_sector][0]]);
		int32_t v_aim = (uint32_t)(pwm_max * afTi[au8PermuataionMatrix[v8_sector][1]]);
		int32_t w_aim = (uint32_t)(pwm_max * afTi[au8PermuataionMatrix[v8_sector][2]]);

		//		UPHASE = (int)(v_uvw.a*V2PWM) + pwm_max/2;
		//		VPHASE = (int)(v_uvw.b*V2PWM) + pwm_max/2;
		//		WPHASE = (int)(v_uvw.c*V2PWM) + pwm_max/2;
		//
		//		UPHASE = (int)(fmaxf(0, fminf((float)pwm_max, U)));
		//		VPHASE = (int)(fmaxf(0, fminf((float)pwm_max, V)));
		//		WPHASE = (int)(fmaxf(0, fminf((float)pwm_max, W)));
		if(v_bus < 5.0f){
			fault_drive_voltage_drop();
		}
		if((ref_w == 0 && cmd == 0 && fabs(w_e) < 100)||(fabs(ref.q) <= 0.5 && fabs(ref.d) <= 0.5 && fabs(w_e) < 5 && fabs(cmd) == 0)){//RAMP_DOWN
			int32_t phase_avg = (UPHASE + VPHASE + WPHASE)/3;
			UPHASE = phase_avg;
			VPHASE = phase_avg;
			WPHASE = phase_avg;
			inverter_state = 3;
		}
		else{
			UPHASE = u_aim;///pwm_max * afTi[au8PermuataionMatrix[v8_sector][0]];
			VPHASE = v_aim;//pwm_max * afTi[au8PermuataionMatrix[v8_sector][1]];
			WPHASE = w_aim;//pwm_max * afTi[au8PermuataionMatrix[v8_sector][2]];
		}
	}

	else if(inverter_state == 3){ // TURN OFF
		ENABLE_OUT;
		ACTIVE_OUT;

		UPHASE -= 3;
		VPHASE -= 3;
		WPHASE -= 3;
		integral.d = 0;
		integral.q = 0;
		integral_w = 0;
		f1.d = 0;
		f2.d = 0;
		f1.q = 0;
		f2.q = 0;
		integral_est.d = 0;
		integral_est.q = 0;
		if(UPHASE <= 5){
			inverter_state = 0;

//			HAL_GPIO_WritePin(PROBE_GPIO_Port, PROBE_Pin, 1);
//			HAL_GPIO_WritePin(PROBE_GPIO_Port, PROBE_Pin, 0);
		}
	}
	else{
		Error_Handler();
	}


	int omega_p  = (int)omega_e;
	int freq_p = (int)freq;
	int8_t i_dq_d = (int8_t)i_dq.d;
	int8_t i_dq_q = (int8_t)i_dq.q;
	int vbus_p = (int)v_bus;
	int power_p = (int)power;
//	int temp_p = (int)board_temp;
	/*
	int ref_p = (int)(cmd*100);
	int a_p = (int)(i_phase[0]);
	int b_p = (int)(i_phase[1]);
	int c_p = (int)(i_phase[2]);

	int vmag_p = (int)vmag;
	int imag_p = (int)imag;
	int vd_p = (int)(v_sat.d);
	int vq_p = (int)v_sat.q;
	int val_p = (int)(integral.q);
	int val2_p = (int)(integral.d);

	int errq_p = (int)(100*(est_error.q + integral_est.q));//(int)(est_error.q * 100);
	int errd_p = (int)(100*(0.5f*est_error.d + integral_est.d));//(int)(est_error.d * 100);
*/

	if(omega_e != 0){
	psi_c = (v_sat.q/omega_e)*0.1f + 0.9f * psi_c;}
	// Debug Printing
//	sprintf(msg,"omega: %f",omega_e);

//	if(printCounter == 0){
//		sprintf(msg, "omega: %d, freq: %d, iq: %d, id: %d, v: %d, P: %d, cmd: %d, Vo: %d, Io: %d, res: %d,res2: %d \n"
//				,omega_p,freq_p, i_dq_q,i_dq_d,vbus_p,power_p,ref_p,vmag_p,imag_p, val_p,val2_p);
//	}
//	else{
//		sprintf(err, "omega: %d, freq: %d, iq: %d, id: %d, v: %d, P: %d, cmd: %d, Vo: %d, Io: %d \n"
//				,omega_p,freq_p, i_dq_q,i_dq_d,vbus_p,power_p,ref_p,vmag_p,imag_p);
//	}
//

#ifdef CAN
	can_rx(can_recieve,&can_rx_fail);
	inverter_can_receive(can_recieve);
//    inverter_send_t *rx = (const inverter_send_t *)can_recieve;


	if (can_timer == (CAN_PERIOD)/3)can_report(2);
	if(can_timer == ((CAN_PERIOD)*2)/3) can_report(1);
	if(can_timer == (CAN_PERIOD)){
		can_report(0);
		can_timer = 0;
	}
	can_timer++;

#endif


//		sprintf(msg, "w: %d, iq: %d, id: %d, cmd: %d, errq: %d, errd: %d \n",
//					omega_p,i_dq_q, i_dq_d,	ref_p,	errq_p, errd_p
//		);
//	sprintf(msg, " q: %d, d: %d, w: %d, iq: %d, id: %d, cmd: %d, \n",
//			errq_p, errd_p,omega_p,i_dq_q, i_dq_d,	ref_p
//			);

//	sprintf(msg, "omega: %d, vd_p: %d, vq_p: %d, psi: %f                end \n"
//			,omega_p,vd_p, vq_p, psi_c);


//	if(0){

#ifdef SERIAL_PRINT

	printCounter++;

	if(printCounter == 1000){

//		sprintf(msg,"Iq: %f , Id: %f, Ld: %f, didt: %f \n",If.q, If.d, Ld_det,local_max);
//		sprintf(msg,"Iq: %2.1f , Id: %2.1f, Det: %f\n",If.q, If.d, Psi_det);
//		sprintf(msg,"Id: %d, Ld: %5.1f \n",swp_step_r*5, (Psi_det*1000000));
//		sprintf(msg, "t: %f, t_e: %f, w: %f, a: %f, j: %f, r: %f \n",
//				theta_e, theta_e_est, omega_e, accel_e*0.0001, jerk_e*0.00000001, residual);
		sprintf(msg,"temp:%f, cmdi: %2.1f, cmd: %f, iq:%+2.1f, id:%+2.1f, vq:%+2.1f, vd:%+3.1f, v:%2.1f, omega:%2.2f, f:%5.0f\n"
				,AMK_temp,		ref.q		,cmd, 		i_dq.q, 	i_dq.d, 	v.q,		 v.d, 	v_bus, w_e,freq);
//		sprintf(msg,"error:%d, fault: %d, i_s: %d, vcu_s: %d, vcu_cmd: %f, vcu_i_n: %f, vcu_i_p: %f, vcu_o_q: %f \n"
//				,inverter_error, inverter_fault, inverter_state, vcu_cmd.state, cmd_q,
//				vcu_cmd.current_in_limit_n, vcu_cmd.current_in_limit_p, vcu_cmd.current_out_limit_q);

//		sprintf(msg,"Iq: %d , Id: %d, Ld: %5.1f \n",swp_step_r, swp_step_c, (Ld_det*1000000));
//		sprintf(msg,"%lu, %f, %f, %f, %f \n",motor,i_phase[0],i_phase[1],i_phase[2],v_bus  );
//		sprintf(msg, "m: %f, b1: %f, b2: %f, b3: %f",
//				AMK_temp,
//				board_temp[0],
//				board_temp[1],
//				board_temp[2]);
	}
	if(printCounter >=2000){
//	if(aw.d != 0 || aw.d != 0){

//	if(fabs(i_phase[0]) > 0.4 || fabs(i_phase[1]) > 0.4|| fabs(i_phase[2]) > 0.4){
//		sprintf(msg,"%f, %f, %f, %f \n",max_U,max_V,max_W,v_bus);

		printCounter = 0;
		HAL_UART_Transmit_DMA(&huart1, (uint8_t*)msg, strlen(msg));

//		sprintf(msg,"%lu, %lu, %lu, %lu, %lu \n", adcValue[0],adcValue[1],adcValue[2],adcValue[3], adcValue[4]);
//		sprintf(msg,"%lu, %lu, %lu, %lu",motor,UPHASE,VPHASE,WPHASE);
//		sprintf(msg,"%lu, %f, %f, %f \n",motor,v_uvw.a,v_uvw.b,v_uvw.c);
//		sprintf(msg,"%f, %f",i_dq.d,i_dq.q);
//		sprintf(msg,"%lu, %f, %f, %f, %f \n",motor,i_phase[0],i_phase[1],i_phase[2],v_bus  );
//		sprintf(msg,"%lu, %d, %d, %d, %d \n",motor,hallOffset[0],hallOffset[1],hallOffset[2],adcOffset[0]);

//		sprintf(msg,"angle:%2.2f, ia:%+2.1f, ib:%+2.1f, ic:%+2.1f, %d, %d, %d"
//					,theta_e,	i_phase[0],i_phase[1],i_phase[2],UPHASE,VPHASE,WPHASE);
//		sprintf(msg,"i_sat:%2.2f, cmdw: %2.1f, ref_i: %2.1f, iq:%+2.1f, id:%+2.1f, vq:%+2.1f, vd:%+3.1f, v:%2.1f, omega:%2.2f, f:%5.0f\n"
//					,i_sat	,	ref_w	,cmd, 		i_dq.q, 	i_dq.d, 	v.q,		 v.d, 	v_bus, w_e,freq);
//		sprintf(msg,"Iq: %f , Id: %f, Ld: %f, didt: %f \n",If.q, If.d, Ld_det,local_max);


//		sprintf(msg,"freq:%2.2f, omega: %2.1f, per: %d \n",
//					freq, omega_e, pwm_max);
//		float diff = theta_e - real_angle;
//		sprintf(msg, "fake:%f, real:%f, offset:%f \n",theta_e,real_angle,diff);
//		sprintf(msg,"r:%f off:%f omega:%f\n",theta_e,angle_offset,omega_e);
//		sprintf(msg, "%d %d %d",UPHASE,VPHASE,WPHASE);

//		sprintf(msg, "%d %d %d %d \n", tempValue[0],tempValue[1],tempValue[2],tempValue[3]);
	}
#endif



	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 44;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL3.PLL3M = 2;
  PeriphClkInitStruct.PLL3.PLL3N = 24;
  PeriphClkInitStruct.PLL3.PLL3P = 2;
  PeriphClkInitStruct.PLL3.PLL3Q = 2;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_3;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_14B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = 4;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_2;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_RESUMED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_18;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  hadc2.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC3_DATAALIGN_RIGHT;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.NbrOfConversion = 2;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  hadc3.Init.Oversampling.Ratio = ADC3_OVERSAMPLING_RATIO_2;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC3_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSign = ADC3_OFFSET_SIGN_NEGATIVE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_CORDIC_CLK_ENABLE();

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */

  /* nothing else to be configured */

  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 6;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 1;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 1;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 32;
  hfdcan1.Init.TxBuffersNbr = 16;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 16;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_QUEUE_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x60404E72;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_10BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
//  LL_SPI_Disable(SPI1);
//
//  LL_SPI_SetNSSMode(SPI1, LL_SPI_NSS_SOFT);
//  LL_SPI_SetInternalSSLevel(SPI1, LL_SPI_SS_LEVEL_HIGH);
//
//  LL_SPI_Enable(SPI1);
//  if (!LL_SPI_IsEnabled(SPI1))
//  {
//	  Error_Handler();
//  }
//  if (!LL_SPI_GetInternalSSLevel(SPI1))
//  {
//	  Error_Handler();
//  }
//  LL_SPI_SetFIFOThreshold(SPI1, LL_SPI_FIFO_TH_01DATA);
//  LL_SPI_EnableIT_RXP(SPI1);

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 6350;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC4REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 275-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED1_Pin|LED2_Pin|OE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_DAT_Pin|PROBE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, U_HALL_CS_Pin|V_HALL_CS_Pin|W_HALL_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_DAT_Pin PROBE_Pin */
  GPIO_InitStruct.Pin = EN_DAT_Pin|PROBE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : OE_Pin */
  GPIO_InitStruct.Pin = OE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : U_HALL_CS_Pin V_HALL_CS_Pin W_HALL_CS_Pin */
  GPIO_InitStruct.Pin = U_HALL_CS_Pin|V_HALL_CS_Pin|W_HALL_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_DETECT_Pin */
  GPIO_InitStruct.Pin = SD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FAULT_Pin */
  GPIO_InitStruct.Pin = FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FAULT_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

	while(__HAL_TIM_GET_COUNTER(&htim16) < pwm_period){
	}
	TIM16->CNT = 0;

//	can_tx(can_transmit, 0x12);

	GPIOE->BSRR = (uint32_t)(GPIO_PIN_14 << 16);
	LED1_GPIO_Port->BSRR = (uint32_t)(LED1_Pin);
	LED2_GPIO_Port->BSRR = (uint32_t)(LED2_Pin);

	inverter_state = 20;

	while(1){
		// do sending
		current_time = __HAL_TIM_GET_COUNTER(&htim16);
		dt = current_time;
		while(dt < pwm_period * CAN_PERIOD){
			current_time = __HAL_TIM_GET_COUNTER(&htim16);
			dt = current_time;
		}
		TIM16->CNT = 0;

#ifdef CAN
		can_rx(can_recieve,&can_rx_fail);
		inverter_can_receive(can_recieve);

		if (can_timer == CAN_PERIOD/4)can_report(0);
		if(can_timer == CAN_PERIOD/2) can_report(1);
		if(can_timer == CAN_PERIOD){
			can_report(2);
			can_timer = 0;
		}
		can_timer++;
#endif

		if(vcu_cmd.state & VCU_STATE_RESET_HARD){
			NVIC_SystemReset();
		}
		if(vcu_cmd.state & VCU_STATE_RESET){
			theta_e_est = 0;
			memset(&integral, 0, sizeof(integral));
			memset(&integral_est, 0, sizeof(integral_est));
			memset(&error, 0, sizeof(error));
			memset(&dError, 0, sizeof(dError));

			integral_w = 0;
			error_w = 0;
			theta_e = 0;
			theta_e_f = 0;
			theta_e_1 = 0;
			omega_e = 0;
			omega_e_1 = 0;
			accel_e = 0;
			acl_e = 0;
			accel_e_1 = 0;
			jerk_e = 0;
			jrk_e = 0;
			thetaI = 0;
			tI = 0;

			inverter_state = 10;
			inverter_fault = 0;
			inverter_error = 0;

			LED1_GPIO_Port->BSRR = (uint32_t)(LED1_Pin << 16);
			LED2_GPIO_Port->BSRR = (uint32_t)(LED2_Pin << 16);
			break;
		}

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
