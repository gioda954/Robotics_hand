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
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
#include "usb_device.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  MOTOR_STATE_IDLE = 0,
  MOTOR_STATE_GRABBING,
  MOTOR_STATE_HOLDING,
  MOTOR_STATE_RELEASING,
  MOTOR_STATE_ADJUSTING,
  MOTOR_STATE_GRABPID,
  MOTOR_STATE_LIGHT,
  MOTOR_STATE_HARD,
  MOTOR_STATE_UNEVEN,
  MOTOR_STATE_SETUP_FORWARD,
  MOTOR_STATE_SETUP_BACKWARD
} MotorState;

typedef struct
{
  TIM_HandleTypeDef *encoder_timer;
  uint32_t encoder_mask;
  TIM_HandleTypeDef *pwm_timer;
  uint32_t pwm_channel;
  GPIO_TypeDef *phase_port;
  uint16_t phase_pin;
  GPIO_PinState grab_phase;
  int32_t encoder_grab_sign;
  uint32_t pressure_index;

  uint32_t encoder_raw;
  uint32_t encoder_last_raw;
  int32_t encoder_position_counts;
  int32_t grab_start_counts;
  int32_t grab_travel_counts;
  int32_t release_start_counts;
  int32_t release_target_counts;
  int32_t adjust_start_counts;
  int32_t adjust_target_counts;
  int32_t adjust_direction_sign;
  int32_t grabpid_integral;
  int32_t grabpid_previous_error;
  int8_t grabpid_output_sign;
  uint32_t grabpid_target_duty_percent;
  uint32_t duty_percent;
  MotorState state;
} MotorRuntime;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_COUNT                      3U
#define MOTOR_TARGET_DUTY_PERCENT       35U
#define SETUP_TARGET_DUTY_PERCENT       50U
#define MOTOR_DUTY_RAMP_STEP_PERCENT    2U
#define MOTOR_DUTY_RAMP_STEP_MS         25U

#define FORWARD_OUTPUT_ROTATION_LIMIT   0.8L
#define GEARBOX_RATIO                   1L
#define ENCODER_COUNTS_PER_MOTOR_ROTATION 4096L
#define ENCODER_COUNTS_PER_OUTPUT_ROTATION \
  (GEARBOX_RATIO * ENCODER_COUNTS_PER_MOTOR_ROTATION)
#define FORWARD_TRAVEL_LIMIT_COUNTS \
  (FORWARD_OUTPUT_ROTATION_LIMIT * ENCODER_COUNTS_PER_OUTPUT_ROTATION)
#define SETUP_RETURN_OVERLAP_COUNTS     (FORWARD_TRAVEL_LIMIT_COUNTS / 5L)
#define GRABPID_GAIN_SCALE              100L
#define GRABPID_ERROR_DEADBAND_PERCENT  1L
#define GRABPID_INTEGRAL_LIMIT          1000L
#define YEAH_MOTOR_NUMBER               2U
#define YEAH_ROTATION_MICRO             2000000UL

#define ADC_MAX_COUNTS                  4095U
#define ADC_PRESSURE1_CHANNEL           ADC_CHANNEL_0
#define ADC_PRESSURE2_CHANNEL           ADC_CHANNEL_1
#define ADC_PRESSURE3_CHANNEL           ADC_CHANNEL_2
/* Pressure indexes: 0 = PA0, 1 = PA1, 2 = PA2. */
#define MOTOR1_PRESSURE_INDEX           1U
#define MOTOR2_PRESSURE_INDEX           2U
#define MOTOR3_PRESSURE_INDEX           0U
#define CONTROL_UPDATE_MS               10U
#define SERIAL_REPORT_MS                100U

#define MOTOR1_GRAB_PHASE               GPIO_PIN_RESET
#define MOTOR2_GRAB_PHASE               GPIO_PIN_SET
#define MOTOR3_GRAB_PHASE               GPIO_PIN_SET
#define MOTOR1_ENCODER_GRAB_SIGN        1L
#define MOTOR2_ENCODER_GRAB_SIGN        -1L
#define MOTOR3_ENCODER_GRAB_SIGN        -1L

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/*
 * Serial commands:
 *   grab    - close each finger until its normal pressure threshold or max travel.
 *   grabpid - independently PID-control each finger to its pressure target.
 *   light   - close each finger like grab, but with low pressure thresholds.
 *   hard    - close each finger like light, but with high pressure thresholds.
 *   uneven  - close for uneven objects; stop all fingers after enough fingers hit threshold.
 *   release - release the last recorded grab/light/uneven/grabpid travel; also accepts "relase".
 *   STOP    - immediately stop every motor with no release motion.
 *   setup   - run the setup forward/backward sequence.
 *   adj <motor> <+|-> <rotations> - move one motor by a signed rotation count.
 *   yeah    - move the configured motor in the positive direction by the configured rotations.
 *
 * Connections:
 *   PA0 pressure sensor -> finger 3, motor phase PB2, PWM PA10.
 *   PA1 pressure sensor -> finger 1, motor phase PB0, PWM PA8.
 *   PA2 pressure sensor -> finger 2, motor phase PB1, PWM PA9.
 *
 * Finger-indexed arrays use order: {finger 1, finger 2, finger 3}.
 */
static uint32_t grab_stop_percent[MOTOR_COUNT] = {40U, 55U, 40U};
static uint32_t grab_pressure_index[MOTOR_COUNT] =
{
  MOTOR1_PRESSURE_INDEX,
  MOTOR2_PRESSURE_INDEX,
  MOTOR3_PRESSURE_INDEX
};
static uint32_t light_stop_percent[MOTOR_COUNT] = {10U, 10U, 10U};
static uint32_t hard_stop_percent[MOTOR_COUNT] = {85U, 85U, 85U};
static uint32_t uneven_stop_percent[MOTOR_COUNT] = {40U, 55U, 40U};
static uint32_t uneven_primary_finger_index = 1U;
static uint32_t uneven_secondary_finger_a_index = 0U;
static uint32_t uneven_secondary_finger_b_index = 2U;
static uint32_t grabpid_target_percent[MOTOR_COUNT] = {60U, 70U, 60U};
static int32_t grabpid_kp[MOTOR_COUNT] = {200L, 200L, 200L};
static int32_t grabpid_ki[MOTOR_COUNT] = {0L, 0L, 0L};
static int32_t grabpid_kd[MOTOR_COUNT] = {0L, 0L, 0L};
static uint32_t grabpid_max_duty_percent[MOTOR_COUNT] =
{
  MOTOR_TARGET_DUTY_PERCENT,
  MOTOR_TARGET_DUTY_PERCENT,
  MOTOR_TARGET_DUTY_PERCENT
};
static uint8_t yeah_motor_number = YEAH_MOTOR_NUMBER;
static uint32_t yeah_rotation_micro = YEAH_ROTATION_MICRO;

static MotorRuntime motors[MOTOR_COUNT] =
{
  {
    .encoder_timer = &htim2,
    .encoder_mask = 0xFFFFFFFFUL,
    .pwm_timer = &htim1,
    .pwm_channel = TIM_CHANNEL_1,
    .phase_port = GPIOB,
    .phase_pin = GPIO_PIN_0,
    .grab_phase = MOTOR1_GRAB_PHASE,
    .encoder_grab_sign = MOTOR1_ENCODER_GRAB_SIGN,
    .pressure_index = MOTOR1_PRESSURE_INDEX
  },
  {
    .encoder_timer = &htim3,
    .encoder_mask = 0x0000FFFFUL,
    .pwm_timer = &htim1,
    .pwm_channel = TIM_CHANNEL_2,
    .phase_port = GPIOB,
    .phase_pin = GPIO_PIN_1,
    .grab_phase = MOTOR2_GRAB_PHASE,
    .encoder_grab_sign = MOTOR2_ENCODER_GRAB_SIGN,
    .pressure_index = MOTOR2_PRESSURE_INDEX
  },
  {
    .encoder_timer = &htim4,
    .encoder_mask = 0x0000FFFFUL,
    .pwm_timer = &htim1,
    .pwm_channel = TIM_CHANNEL_3,
    .phase_port = GPIOB,
    .phase_pin = GPIO_PIN_2,
    .grab_phase = MOTOR3_GRAB_PHASE,
    .encoder_grab_sign = MOTOR3_ENCODER_GRAB_SIGN,
    .pressure_index = MOTOR3_PRESSURE_INDEX
  }
};

static uint32_t pressure_raw[3];
static uint32_t pressure_percent[3];
static uint8_t setup_sequence_active = 0U;
static uint8_t setup_waiting_for_zero_duty = 0U;
static uint32_t setup_motor_index = 0U;
static uint8_t setup_overlap_started[MOTOR_COUNT];
static int8_t setup_next_direction_sign = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
static void App_Init(void);
static void App_Update(void);
static void App_HandleSerialCommands(void);
static void App_StartGrab(void);
static void App_StartGrabPid(void);
static void App_StartLight(void);
static void App_StartHard(void);
static void App_StartUneven(void);
static void App_StartRelease(void);
static void App_StopAllMotors(const char *command_name);
static void App_StartAdjust(uint8_t motor_number, int8_t direction_sign, uint32_t rotation_micro);
static void App_StartMotorRotation(uint8_t motor_number,
                                   int8_t direction_sign,
                                   uint32_t rotation_micro,
                                   const char *command_name);
static void App_StartYeah(void);
static void App_StartSetup(void);
static void App_StartSetupMove(uint32_t motor_index, int8_t direction_sign);
static void App_AdvanceSetupSequence(void);
static void App_StartPressureGrab(MotorState state, const char *command_name);
static void App_UpdateGrabMotor(uint32_t motor_index);
static void App_UpdatePressureGrabMotor(uint32_t motor_index, const uint32_t *stop_percent);
static void App_UpdateUnevenGrab(void);
static void App_StopMotorHolding(MotorRuntime *motor);
static void App_UpdateGrabPidMotor(uint32_t motor_index);
static void App_ReadPressureSensors(void);
static void App_PrintStatus(void);
static void Motor_SetDutyPercent(MotorRuntime *motor, uint32_t duty_percent);
static void Motor_RampDuty(MotorRuntime *motor, uint32_t target_duty_percent);
static void Motor_SetDirection(MotorRuntime *motor, GPIO_PinState phase);
static void Motor_Stop(MotorRuntime *motor);
static void Motor_UpdateEncoder(MotorRuntime *motor);
static int32_t Motor_GetLogicalForwardCounts(const MotorRuntime *motor);
static int32_t Motor_GetLogicalReleaseTravelCounts(const MotorRuntime *motor);
static int32_t Motor_GetLogicalAdjustTravelCounts(const MotorRuntime *motor);
static int32_t Motor_GetRotationMilli(const MotorRuntime *motor);
static int32_t RotationMicroToCounts(uint32_t rotation_micro);
static void Motor_ResetZero(MotorRuntime *motor);
static int32_t Abs32(int32_t value);
static uint32_t PressurePercentFromRaw(uint32_t raw);
static void Serial_SendString(const char *text);
static uint32_t ADC_ReadRaw(uint32_t channel);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void App_Init(void)
{
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  for (uint32_t i = 0U; i < MOTOR_COUNT; i++)
  {
    __HAL_TIM_SET_COUNTER(motors[i].encoder_timer, 0U);
    motors[i].encoder_raw = 0U;
    motors[i].encoder_last_raw = 0U;
    motors[i].encoder_position_counts = 0L;
    motors[i].grab_start_counts = 0L;
    motors[i].grab_travel_counts = 0L;
    motors[i].release_start_counts = 0L;
    motors[i].release_target_counts = 0L;
    motors[i].adjust_start_counts = 0L;
    motors[i].adjust_target_counts = 0L;
    motors[i].adjust_direction_sign = 1L;
    motors[i].grabpid_integral = 0L;
    motors[i].grabpid_previous_error = 0L;
    motors[i].grabpid_output_sign = 0;
    motors[i].grabpid_target_duty_percent = 0U;
    motors[i].state = MOTOR_STATE_IDLE;
    Motor_Stop(&motors[i]);
  }

  App_ReadPressureSensors();
  Serial_SendString("robotic_hand_test,start\r\n");
  Serial_SendString("commands: grab, grabpid, light, hard, uneven, STOP, relase, release, setup, yeah, adj <motor> <+|-> <rotations>\r\n");
  Serial_SendString("t_ms,m1_state,m1_enc_raw,m1_counts,m1_rot,m1_duty,m2_state,m2_enc_raw,m2_counts,m2_rot,m2_duty,m3_state,m3_enc_raw,m3_counts,m3_rot,m3_duty,p1_raw,p1_pct,p2_raw,p2_pct,p3_raw,p3_pct\r\n");
}

static void App_Update(void)
{
  static uint32_t last_control_tick = 0U;
  static uint32_t last_ramp_tick = 0U;
  static uint32_t last_report_tick = 0U;
  uint32_t now = HAL_GetTick();

  App_HandleSerialCommands();

  if ((now - last_control_tick) >= CONTROL_UPDATE_MS)
  {
    last_control_tick = now;
    App_ReadPressureSensors();

    for (uint32_t i = 0U; i < MOTOR_COUNT; i++)
    {
      Motor_UpdateEncoder(&motors[i]);

      if (motors[i].state == MOTOR_STATE_GRABBING)
      {
        App_UpdateGrabMotor(i);
      }
      else if (motors[i].state == MOTOR_STATE_LIGHT)
      {
        App_UpdatePressureGrabMotor(i, light_stop_percent);
      }
      else if (motors[i].state == MOTOR_STATE_HARD)
      {
        App_UpdatePressureGrabMotor(i, hard_stop_percent);
      }
      else if (motors[i].state == MOTOR_STATE_GRABPID)
      {
        App_UpdateGrabPidMotor(i);
      }
      else if (motors[i].state == MOTOR_STATE_RELEASING)
      {
        if (Motor_GetLogicalReleaseTravelCounts(&motors[i]) >= motors[i].grab_travel_counts)
        {
          motors[i].grab_travel_counts = 0L;
          motors[i].release_target_counts = 0L;
          motors[i].state = MOTOR_STATE_IDLE;
          Motor_Stop(&motors[i]);
        }
      }
      else if (motors[i].state == MOTOR_STATE_ADJUSTING)
      {
        if (Motor_GetLogicalAdjustTravelCounts(&motors[i]) >= motors[i].adjust_target_counts)
        {
          Motor_Stop(&motors[i]);
          Motor_ResetZero(&motors[i]);
          motors[i].state = MOTOR_STATE_IDLE;
        }
      }
      else if ((motors[i].state == MOTOR_STATE_SETUP_FORWARD) ||
               (motors[i].state == MOTOR_STATE_SETUP_BACKWARD))
      {
        int32_t setup_travel_counts = Motor_GetLogicalAdjustTravelCounts(&motors[i]);
        MotorState setup_state = motors[i].state;

        if ((setup_sequence_active != 0U) &&
            (i == setup_motor_index) &&
            (setup_state == MOTOR_STATE_SETUP_BACKWARD) &&
            (setup_overlap_started[i] == 0U) &&
            ((setup_travel_counts + SETUP_RETURN_OVERLAP_COUNTS) >= motors[i].adjust_target_counts) &&
            ((i + 1U) < MOTOR_COUNT) &&
            (motors[i + 1U].state == MOTOR_STATE_IDLE))
        {
          setup_overlap_started[i] = 1U;
          App_StartSetupMove(i + 1U, 1);
        }

        if ((i == setup_motor_index) && (setup_travel_counts >= motors[i].adjust_target_counts))
        {
          setup_next_direction_sign =
              (setup_state == MOTOR_STATE_SETUP_FORWARD) ? -1 : 0;
          motors[i].state = MOTOR_STATE_IDLE;

          if ((setup_state == MOTOR_STATE_SETUP_BACKWARD) &&
              (setup_overlap_started[i] != 0U) &&
              ((i + 1U) < MOTOR_COUNT))
          {
            setup_waiting_for_zero_duty = 0U;
            App_AdvanceSetupSequence();
          }
          else
          {
            setup_waiting_for_zero_duty = 1U;
          }
        }
      }
    }

    App_UpdateUnevenGrab();
  }

  if ((now - last_ramp_tick) >= MOTOR_DUTY_RAMP_STEP_MS)
  {
    last_ramp_tick = now;

    for (uint32_t i = 0U; i < MOTOR_COUNT; i++)
    {
      if (motors[i].state == MOTOR_STATE_GRABPID)
      {
        Motor_RampDuty(&motors[i], motors[i].grabpid_target_duty_percent);
      }
      else if ((motors[i].state == MOTOR_STATE_GRABBING) ||
          (motors[i].state == MOTOR_STATE_LIGHT) ||
          (motors[i].state == MOTOR_STATE_HARD) ||
          (motors[i].state == MOTOR_STATE_UNEVEN) ||
          (motors[i].state == MOTOR_STATE_RELEASING) ||
          (motors[i].state == MOTOR_STATE_ADJUSTING))
      {
        Motor_RampDuty(&motors[i], MOTOR_TARGET_DUTY_PERCENT);
      }
      else if ((motors[i].state == MOTOR_STATE_SETUP_FORWARD) ||
               (motors[i].state == MOTOR_STATE_SETUP_BACKWARD))
      {
        Motor_RampDuty(&motors[i], SETUP_TARGET_DUTY_PERCENT);
      }
      else
      {
        Motor_RampDuty(&motors[i], 0U);
      }
    }

    if ((setup_sequence_active != 0U) &&
        (setup_waiting_for_zero_duty != 0U) &&
        (setup_motor_index < MOTOR_COUNT) &&
        (motors[setup_motor_index].duty_percent == 0U))
    {
      App_AdvanceSetupSequence();
    }
  }

  if ((now - last_report_tick) >= SERIAL_REPORT_MS)
  {
    last_report_tick = now;
    App_PrintStatus();
  }
}

static void App_HandleSerialCommands(void)
{
  uint8_t adjust_motor_number;
  int8_t adjust_direction_sign;
  uint32_t adjust_rotation_micro;

  if (USB_CDC_PollStopCommand() != 0U)
  {
    App_StopAllMotors("STOP");
    return;
  }

  if (USB_CDC_PollReleaseCommand() != 0U)
  {
    App_StartRelease();
    return;
  }

  if (USB_CDC_PollGrabCommand() != 0U)
  {
    App_StartGrab();
  }

  if (USB_CDC_PollGrabPidCommand() != 0U)
  {
    App_StartGrabPid();
  }

  if (USB_CDC_PollLightCommand() != 0U)
  {
    App_StartLight();
  }

  if (USB_CDC_PollHardCommand() != 0U)
  {
    App_StartHard();
  }

  if (USB_CDC_PollUnevenCommand() != 0U)
  {
    App_StartUneven();
  }

  if (USB_CDC_PollYeahCommand() != 0U)
  {
    App_StartYeah();
  }

  if (USB_CDC_PollSetupCommand() != 0U)
  {
    App_StartSetup();
  }

  if (USB_CDC_PollAdjustCommand(&adjust_motor_number,
                                &adjust_direction_sign,
                                &adjust_rotation_micro) != 0U)
  {
    App_StartAdjust(adjust_motor_number, adjust_direction_sign, adjust_rotation_micro);
  }
}

static void App_StartGrab(void)
{
  App_StartPressureGrab(MOTOR_STATE_GRABBING, "grab");
}

static void App_StartLight(void)
{
  App_StartPressureGrab(MOTOR_STATE_LIGHT, "light");
}

static void App_StartHard(void)
{
  App_StartPressureGrab(MOTOR_STATE_HARD, "hard");
}

static void App_StartUneven(void)
{
  App_StartPressureGrab(MOTOR_STATE_UNEVEN, "uneven");
}

static void App_StartPressureGrab(MotorState state, const char *command_name)
{
  char line[32];

  setup_sequence_active = 0U;
  setup_waiting_for_zero_duty = 0U;
  setup_next_direction_sign = 0;

  for (uint32_t i = 0U; i < MOTOR_COUNT; i++)
  {
    Motor_UpdateEncoder(&motors[i]);
    motors[i].grab_start_counts = motors[i].encoder_position_counts;
    motors[i].grab_travel_counts = 0L;
    motors[i].release_target_counts = 0L;
    motors[i].grabpid_integral = 0L;
    motors[i].grabpid_previous_error = 0L;
    motors[i].grabpid_output_sign = 0;
    motors[i].grabpid_target_duty_percent = 0U;
    motors[i].state = state;
    Motor_SetDirection(&motors[i], motors[i].grab_phase);
  }

  snprintf(line, sizeof(line), "cmd,%s\r\n", command_name);
  Serial_SendString(line);
}

static void App_StartGrabPid(void)
{
  setup_sequence_active = 0U;
  setup_waiting_for_zero_duty = 0U;
  setup_next_direction_sign = 0;

  for (uint32_t i = 0U; i < MOTOR_COUNT; i++)
  {
    Motor_UpdateEncoder(&motors[i]);
    motors[i].grab_start_counts = motors[i].encoder_position_counts;
    motors[i].grab_travel_counts = 0L;
    motors[i].release_target_counts = 0L;
    motors[i].adjust_target_counts = 0L;
    motors[i].grabpid_integral = 0L;
    motors[i].grabpid_previous_error =
        (int32_t)grabpid_target_percent[i] -
        (int32_t)pressure_percent[motors[i].pressure_index];
    motors[i].grabpid_output_sign = 0;
    motors[i].grabpid_target_duty_percent = 0U;
    motors[i].state = MOTOR_STATE_GRABPID;
    Motor_SetDirection(&motors[i], motors[i].grab_phase);
  }

  Serial_SendString("cmd,grabpid\r\n");
}

static void App_StopAllMotors(const char *command_name)
{
  char line[32];

  setup_sequence_active = 0U;
  setup_waiting_for_zero_duty = 0U;
  setup_next_direction_sign = 0;

  for (uint32_t i = 0U; i < MOTOR_COUNT; i++)
  {
    motors[i].release_target_counts = 0L;
    motors[i].adjust_target_counts = 0L;
    motors[i].grabpid_target_duty_percent = 0U;
    motors[i].grabpid_output_sign = 0;
    motors[i].state = MOTOR_STATE_IDLE;
    Motor_Stop(&motors[i]);
  }

  snprintf(line, sizeof(line), "cmd,%s\r\n", command_name);
  Serial_SendString(line);
}

static void App_StartRelease(void)
{
  setup_sequence_active = 0U;
  setup_waiting_for_zero_duty = 0U;
  setup_next_direction_sign = 0;

  for (uint32_t i = 0U; i < MOTOR_COUNT; i++)
  {
    Motor_UpdateEncoder(&motors[i]);

    if (motors[i].grab_travel_counts > 0L)
    {
      motors[i].grabpid_target_duty_percent = 0U;
      motors[i].grabpid_output_sign = 0;
      motors[i].release_start_counts = motors[i].encoder_position_counts;
      motors[i].release_target_counts = motors[i].grab_travel_counts;
      motors[i].state = MOTOR_STATE_RELEASING;
      Motor_SetDirection(&motors[i], (motors[i].grab_phase == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
    else
    {
      motors[i].state = MOTOR_STATE_IDLE;
      Motor_Stop(&motors[i]);
    }
  }

  Serial_SendString("cmd,release\r\n");
}

static void App_StartAdjust(uint8_t motor_number, int8_t direction_sign, uint32_t rotation_micro)
{
  App_StartMotorRotation(motor_number, direction_sign, rotation_micro, "adj");
}

static void App_StartMotorRotation(uint8_t motor_number,
                                   int8_t direction_sign,
                                   uint32_t rotation_micro,
                                   const char *command_name)
{
  char line[64];
  int32_t adjust_counts = RotationMicroToCounts(rotation_micro);
  MotorRuntime *motor;

  if ((motor_number < 1U) || (motor_number > MOTOR_COUNT) ||
      ((direction_sign != 1) && (direction_sign != -1)) ||
      (adjust_counts <= 0L))
  {
    snprintf(line, sizeof(line), "cmd,%s,error\r\n", command_name);
    Serial_SendString(line);
    return;
  }

  setup_sequence_active = 0U;
  setup_waiting_for_zero_duty = 0U;
  setup_next_direction_sign = 0;
  motor = &motors[motor_number - 1U];
  Motor_UpdateEncoder(motor);
  motor->adjust_start_counts = motor->encoder_position_counts;
  motor->adjust_target_counts = adjust_counts;
  motor->adjust_direction_sign = direction_sign;
  motor->grab_travel_counts = 0L;
  motor->release_target_counts = 0L;
  motor->grabpid_target_duty_percent = 0U;
  motor->grabpid_output_sign = 0;
  motor->state = MOTOR_STATE_ADJUSTING;

  Motor_SetDirection(motor,
                     (direction_sign > 0) ?
                     motor->grab_phase :
                     ((motor->grab_phase == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET));

  snprintf(line,
           sizeof(line),
           "cmd,%s,%u,%c,%ld\r\n",
           command_name,
           (unsigned int)motor_number,
           (direction_sign > 0) ? '+' : '-',
           (long)adjust_counts);
  Serial_SendString(line);
}

static void App_StartYeah(void)
{
  App_StartMotorRotation(yeah_motor_number, 1, yeah_rotation_micro, "yeah");
}

static void App_StartSetup(void)
{
  setup_sequence_active = 1U;
  setup_waiting_for_zero_duty = 0U;
  setup_motor_index = 0U;
  setup_next_direction_sign = 0;

  for (uint32_t i = 0U; i < MOTOR_COUNT; i++)
  {
    setup_overlap_started[i] = 0U;
    Motor_UpdateEncoder(&motors[i]);
    motors[i].grab_travel_counts = 0L;
    motors[i].release_target_counts = 0L;
    motors[i].adjust_target_counts = 0L;
    motors[i].grabpid_target_duty_percent = 0U;
    motors[i].grabpid_output_sign = 0;
    motors[i].state = MOTOR_STATE_IDLE;
  }

  Serial_SendString("cmd,setup,start\r\n");
  App_StartSetupMove(setup_motor_index, 1);
}

static void App_StartSetupMove(uint32_t motor_index, int8_t direction_sign)
{
  MotorRuntime *motor;

  if (motor_index >= MOTOR_COUNT)
  {
    setup_sequence_active = 0U;
    Serial_SendString("cmd,setup,done\r\n");
    return;
  }

  motor = &motors[motor_index];
  setup_waiting_for_zero_duty = 0U;
  setup_next_direction_sign = 0;
  Motor_UpdateEncoder(motor);
  setup_overlap_started[motor_index] = 0U;
  motor->adjust_start_counts = motor->encoder_position_counts;
  motor->adjust_target_counts = FORWARD_TRAVEL_LIMIT_COUNTS;
  motor->adjust_direction_sign = direction_sign;
  motor->state = (direction_sign > 0) ? MOTOR_STATE_SETUP_FORWARD : MOTOR_STATE_SETUP_BACKWARD;

  Motor_SetDirection(motor,
                     (direction_sign > 0) ?
                     motor->grab_phase :
                     ((motor->grab_phase == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET));
}

static void App_AdvanceSetupSequence(void)
{
  if (setup_sequence_active == 0U)
  {
    return;
  }

  if (setup_motor_index >= MOTOR_COUNT)
  {
    setup_sequence_active = 0U;
    Serial_SendString("cmd,setup,done\r\n");
    return;
  }

  setup_waiting_for_zero_duty = 0U;

  if (setup_next_direction_sign != 0)
  {
    App_StartSetupMove(setup_motor_index, setup_next_direction_sign);
    return;
  }

  motors[setup_motor_index].state = MOTOR_STATE_IDLE;
  setup_motor_index++;

  if (setup_motor_index < MOTOR_COUNT)
  {
    if ((motors[setup_motor_index].state != MOTOR_STATE_SETUP_FORWARD) &&
        (motors[setup_motor_index].state != MOTOR_STATE_SETUP_BACKWARD))
    {
      App_StartSetupMove(setup_motor_index, 1);
    }
  }
  else
  {
    setup_sequence_active = 0U;
    Serial_SendString("cmd,setup,done\r\n");
  }
}

static void App_UpdateGrabMotor(uint32_t motor_index)
{
  MotorRuntime *motor = &motors[motor_index];
  int32_t forward_counts = Motor_GetLogicalForwardCounts(motor);
  int32_t command_travel_counts =
      forward_counts - (motor->grab_start_counts * motor->encoder_grab_sign);
  uint32_t finger_pressure_percent = pressure_percent[grab_pressure_index[motor_index]];

  if (command_travel_counts < 0L)
  {
    command_travel_counts = 0L;
  }
  motor->grab_travel_counts = command_travel_counts;

  if ((command_travel_counts >= FORWARD_TRAVEL_LIMIT_COUNTS) ||
      (finger_pressure_percent >= grab_stop_percent[motor_index]))
  {
    App_StopMotorHolding(motor);
  }
}

static void App_UpdatePressureGrabMotor(uint32_t motor_index, const uint32_t *stop_percent)
{
  MotorRuntime *motor = &motors[motor_index];
  int32_t forward_counts = Motor_GetLogicalForwardCounts(motor);
  int32_t command_travel_counts =
      forward_counts - (motor->grab_start_counts * motor->encoder_grab_sign);
  uint32_t finger_pressure_percent = pressure_percent[motor->pressure_index];

  if (command_travel_counts < 0L)
  {
    command_travel_counts = 0L;
  }
  motor->grab_travel_counts = command_travel_counts;

  if ((command_travel_counts >= FORWARD_TRAVEL_LIMIT_COUNTS) ||
      (finger_pressure_percent >= stop_percent[motor_index]))
  {
    App_StopMotorHolding(motor);
  }
}

static void App_UpdateUnevenGrab(void)
{
  uint32_t active_count = 0U;
  uint8_t threshold_reached[MOTOR_COUNT] = {0U, 0U, 0U};
  uint8_t stop_all = 0U;

  for (uint32_t i = 0U; i < MOTOR_COUNT; i++)
  {
    MotorRuntime *motor = &motors[i];
    int32_t forward_counts;
    int32_t command_travel_counts;

    if (motor->state != MOTOR_STATE_UNEVEN)
    {
      continue;
    }

    active_count++;
    forward_counts = Motor_GetLogicalForwardCounts(motor);
    command_travel_counts =
        forward_counts - (motor->grab_start_counts * motor->encoder_grab_sign);

    if (command_travel_counts < 0L)
    {
      command_travel_counts = 0L;
    }
    motor->grab_travel_counts = command_travel_counts;

    if (command_travel_counts >= FORWARD_TRAVEL_LIMIT_COUNTS)
    {
      App_StopMotorHolding(motor);
    }

    if (pressure_percent[motor->pressure_index] >= uneven_stop_percent[i])
    {
      threshold_reached[i] = 1U;
    }
  }

  if ((uneven_primary_finger_index < MOTOR_COUNT) &&
      (uneven_secondary_finger_a_index < MOTOR_COUNT) &&
      (uneven_secondary_finger_b_index < MOTOR_COUNT) &&
      (threshold_reached[uneven_primary_finger_index] != 0U) &&
      ((threshold_reached[uneven_secondary_finger_a_index] != 0U) ||
       (threshold_reached[uneven_secondary_finger_b_index] != 0U)))
  {
    stop_all = 1U;
  }

  if ((active_count == 0U) || (stop_all == 0U))
  {
    return;
  }

  for (uint32_t i = 0U; i < MOTOR_COUNT; i++)
  {
    if ((motors[i].state == MOTOR_STATE_UNEVEN) ||
        (motors[i].state == MOTOR_STATE_HOLDING))
    {
      App_StopMotorHolding(&motors[i]);
    }
  }
}

static void App_StopMotorHolding(MotorRuntime *motor)
{
  motor->state = MOTOR_STATE_HOLDING;
  motor->grabpid_target_duty_percent = 0U;
  motor->grabpid_output_sign = 0;
  Motor_Stop(motor);
}

static void App_UpdateGrabPidMotor(uint32_t motor_index)
{
  MotorRuntime *motor = &motors[motor_index];
  int32_t start_forward_counts = motor->grab_start_counts * motor->encoder_grab_sign;
  int32_t forward_counts = Motor_GetLogicalForwardCounts(motor);
  int32_t travel_counts = forward_counts - start_forward_counts;
  int32_t error;
  int32_t derivative;
  int32_t output;
  int8_t output_sign;
  uint32_t duty_percent;

  if (travel_counts < 0L)
  {
    travel_counts = 0L;
  }
  motor->grab_travel_counts = travel_counts;

  error = (int32_t)grabpid_target_percent[motor_index] -
          (int32_t)pressure_percent[motor->pressure_index];

  if ((Abs32(error) <= GRABPID_ERROR_DEADBAND_PERCENT) ||
      ((error > 0L) && (travel_counts >= FORWARD_TRAVEL_LIMIT_COUNTS)) ||
      ((error < 0L) && (travel_counts <= 0L)))
  {
    motor->grabpid_target_duty_percent = 0U;
    motor->grabpid_output_sign = 0;
    motor->grabpid_previous_error = error;
    return;
  }

  motor->grabpid_integral += error;
  if (motor->grabpid_integral > GRABPID_INTEGRAL_LIMIT)
  {
    motor->grabpid_integral = GRABPID_INTEGRAL_LIMIT;
  }
  else if (motor->grabpid_integral < -GRABPID_INTEGRAL_LIMIT)
  {
    motor->grabpid_integral = -GRABPID_INTEGRAL_LIMIT;
  }

  derivative = error - motor->grabpid_previous_error;
  motor->grabpid_previous_error = error;

  output = ((grabpid_kp[motor_index] * error) +
            (grabpid_ki[motor_index] * motor->grabpid_integral) +
            (grabpid_kd[motor_index] * derivative)) / GRABPID_GAIN_SCALE;

  if (output == 0L)
  {
    motor->grabpid_target_duty_percent = 0U;
    motor->grabpid_output_sign = 0;
    return;
  }

  output_sign = (output > 0L) ? 1 : -1;
  duty_percent = (uint32_t)Abs32(output);
  if (duty_percent > grabpid_max_duty_percent[motor_index])
  {
    duty_percent = grabpid_max_duty_percent[motor_index];
  }

  if (motor->grabpid_output_sign != output_sign)
  {
    Motor_Stop(motor);
    Motor_SetDirection(motor,
                       (output_sign > 0) ?
                       motor->grab_phase :
                       ((motor->grab_phase == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET));
    motor->grabpid_output_sign = output_sign;
  }

  motor->grabpid_target_duty_percent = duty_percent;
}

static void App_ReadPressureSensors(void)
{
  pressure_raw[0] = ADC_ReadRaw(ADC_PRESSURE1_CHANNEL);
  pressure_raw[1] = ADC_ReadRaw(ADC_PRESSURE2_CHANNEL);
  pressure_raw[2] = ADC_ReadRaw(ADC_PRESSURE3_CHANNEL);

  pressure_percent[0] = PressurePercentFromRaw(pressure_raw[0]);
  pressure_percent[1] = PressurePercentFromRaw(pressure_raw[1]);
  pressure_percent[2] = PressurePercentFromRaw(pressure_raw[2]);
}

static void App_PrintStatus(void)
{
  char line[320];
  int32_t m1_rot_milli = Motor_GetRotationMilli(&motors[0]);
  int32_t m2_rot_milli = Motor_GetRotationMilli(&motors[1]);
  int32_t m3_rot_milli = Motor_GetRotationMilli(&motors[2]);

  snprintf(line,
           sizeof(line),
           "%lu,%u,%lu,%ld,%ld.%03ld,%lu,%u,%lu,%ld,%ld.%03ld,%lu,%u,%lu,%ld,%ld.%03ld,%lu,%lu,%lu,%lu,%lu,%lu,%lu\r\n",
           (unsigned long)HAL_GetTick(),
           (unsigned int)motors[0].state,
           (unsigned long)motors[0].encoder_raw,
           (long)motors[0].encoder_position_counts,
           (long)(m1_rot_milli / 1000L),
           (long)Abs32(m1_rot_milli % 1000L),
           (unsigned long)motors[0].duty_percent,
           (unsigned int)motors[1].state,
           (unsigned long)motors[1].encoder_raw,
           (long)motors[1].encoder_position_counts,
           (long)(m2_rot_milli / 1000L),
           (long)Abs32(m2_rot_milli % 1000L),
           (unsigned long)motors[1].duty_percent,
           (unsigned int)motors[2].state,
           (unsigned long)motors[2].encoder_raw,
           (long)motors[2].encoder_position_counts,
           (long)(m3_rot_milli / 1000L),
           (long)Abs32(m3_rot_milli % 1000L),
           (unsigned long)motors[2].duty_percent,
           (unsigned long)pressure_raw[0],
           (unsigned long)pressure_percent[0],
           (unsigned long)pressure_raw[1],
           (unsigned long)pressure_percent[1],
           (unsigned long)pressure_raw[2],
           (unsigned long)pressure_percent[2]);
  Serial_SendString(line);
}

static void Motor_SetDutyPercent(MotorRuntime *motor, uint32_t duty_percent)
{
  uint32_t period_ticks = __HAL_TIM_GET_AUTORELOAD(motor->pwm_timer) + 1U;
  uint32_t compare_ticks;

  if (duty_percent > 100U)
  {
    duty_percent = 100U;
  }

  compare_ticks = (period_ticks * duty_percent) / 100U;
  __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, compare_ticks);
  motor->duty_percent = duty_percent;
}

static void Motor_RampDuty(MotorRuntime *motor, uint32_t target_duty_percent)
{
  uint32_t next_duty = motor->duty_percent;

  if (next_duty < target_duty_percent)
  {
    next_duty += MOTOR_DUTY_RAMP_STEP_PERCENT;
    if (next_duty > target_duty_percent)
    {
      next_duty = target_duty_percent;
    }
  }
  else if (next_duty > target_duty_percent)
  {
    if (next_duty > MOTOR_DUTY_RAMP_STEP_PERCENT)
    {
      next_duty -= MOTOR_DUTY_RAMP_STEP_PERCENT;
    }
    else
    {
      next_duty = 0U;
    }

    if (next_duty < target_duty_percent)
    {
      next_duty = target_duty_percent;
    }
  }

  Motor_SetDutyPercent(motor, next_duty);
}

static void Motor_SetDirection(MotorRuntime *motor, GPIO_PinState phase)
{
  HAL_GPIO_WritePin(motor->phase_port, motor->phase_pin, phase);
}

static void Motor_Stop(MotorRuntime *motor)
{
  Motor_SetDutyPercent(motor, 0U);
}

static void Motor_UpdateEncoder(MotorRuntime *motor)
{
  uint32_t raw = __HAL_TIM_GET_COUNTER(motor->encoder_timer) & motor->encoder_mask;
  int32_t delta;

  if (motor->encoder_mask == 0x0000FFFFUL)
  {
    delta = (int32_t)(int16_t)((uint16_t)(raw - motor->encoder_last_raw));
  }
  else
  {
    delta = (int32_t)(raw - motor->encoder_last_raw);
  }

  motor->encoder_raw = raw;
  motor->encoder_last_raw = raw;
  motor->encoder_position_counts += delta;
}

static int32_t Motor_GetLogicalForwardCounts(const MotorRuntime *motor)
{
  return motor->encoder_position_counts * motor->encoder_grab_sign;
}

static int32_t Motor_GetLogicalReleaseTravelCounts(const MotorRuntime *motor)
{
  int32_t delta = motor->release_start_counts - motor->encoder_position_counts;
  return delta * motor->encoder_grab_sign;
}

static int32_t Motor_GetLogicalAdjustTravelCounts(const MotorRuntime *motor)
{
  int32_t start_forward_counts = motor->adjust_start_counts * motor->encoder_grab_sign;
  int32_t forward_counts = Motor_GetLogicalForwardCounts(motor);
  int32_t travel_counts = (forward_counts - start_forward_counts) * motor->adjust_direction_sign;

  return (travel_counts < 0L) ? 0L : travel_counts;
}

static int32_t Motor_GetRotationMilli(const MotorRuntime *motor)
{
  if (ENCODER_COUNTS_PER_OUTPUT_ROTATION == 0L)
  {
    return 0L;
  }

  return (motor->encoder_position_counts * 1000L) / ENCODER_COUNTS_PER_OUTPUT_ROTATION;
}

static int32_t RotationMicroToCounts(uint32_t rotation_micro)
{
  int64_t counts = ((int64_t)rotation_micro * (int64_t)ENCODER_COUNTS_PER_OUTPUT_ROTATION) / 1000000LL;

  if (counts > 2147483647LL)
  {
    return 2147483647L;
  }

  return (int32_t)counts;
}

static void Motor_ResetZero(MotorRuntime *motor)
{
  __HAL_TIM_SET_COUNTER(motor->encoder_timer, 0U);
  motor->encoder_raw = 0U;
  motor->encoder_last_raw = 0U;
  motor->encoder_position_counts = 0L;
  motor->grab_start_counts = 0L;
  motor->grab_travel_counts = 0L;
  motor->release_start_counts = 0L;
  motor->release_target_counts = 0L;
  motor->adjust_start_counts = 0L;
  motor->adjust_target_counts = 0L;
  motor->adjust_direction_sign = 1L;
  motor->grabpid_integral = 0L;
  motor->grabpid_previous_error = 0L;
  motor->grabpid_output_sign = 0;
  motor->grabpid_target_duty_percent = 0U;
}

static int32_t Abs32(int32_t value)
{
  return (value < 0L) ? -value : value;
}

static uint32_t PressurePercentFromRaw(uint32_t raw)
{
  if (raw > ADC_MAX_COUNTS)
  {
    raw = ADC_MAX_COUNTS;
  }

  return (raw * 100U) / ADC_MAX_COUNTS;
}

static void Serial_SendString(const char *text)
{
  USB_CDC_SendString(text);
}

static uint32_t ADC_ReadRaw(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  uint32_t raw = 0U;

  sConfig.Channel = channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADC_PollForConversion(&hadc1, 10U) != HAL_OK)
  {
    HAL_ADC_Stop(&hadc1);
    Error_Handler();
  }

  raw = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  return raw;
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  App_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    App_Update();
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4799;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
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
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
