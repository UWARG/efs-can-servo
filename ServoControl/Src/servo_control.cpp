#include <string.h>
#include "servo_control.hpp"

#define PWM_DUTY_CYCLE_BASE 0.05f
#define PWM_DUTY_CYCLE_INC_HALF  0.25f
#define POSITION_TO_DUTY_CYCLE(POS, PERIOD) \
        ((PWM_DUTY_CYCLE_BASE + PWM_DUTY_CYCLE_INC_HALF + PWM_DUTY_CYCLE_INC_HALF*POS)*PERIOD)

#ifdef SINGLE_SERVO_DRIVER
    extern TIM_HandleTypeDef htim1;
    ServoChannel_t ServoController::SERVO_CHANNELS[ServoController::NUM_SERVOS] = {
        {htim1, TIM_CHANNEL_1, true}
    };

    const uint8_t ServoController::SERVO_IDS[ServoController::NUM_SERVOS];
#endif

#ifdef SIX_S_SERVO_MODULE
    extern TIM_HandleTypeDef htim1;
    extern TIM_HandleTypeDef htim2;
    ServoChannel_t ServoController::SERVO_CHANNELS[ServoController::NUM_SERVOS] = {
        {htim1, TIM_CHANNEL_1, false},
        {htim1, TIM_CHANNEL_2, false},
        {htim1, TIM_CHANNEL_3, false},
        {htim2, TIM_CHANNEL_1, false}
    };

    const uint8_t ServoController::SERVO_IDS[ServoController::NUM_SERVOS];
#endif

ServoController::ServoController(){

    memset(servo_positions_, 0, NUM_SERVOS*sizeof(servo_positions_[0]));
    memset(last_update_us_, 0 , NUM_SERVOS*sizeof(last_update_us_[0]));

    // fetch timer period (assume timer2 uses the same period as timer1)
    pwm_period_ = htim1.Instance->ARR;
    if ( pwm_period_ == 0 ) { // if fails to fetch the timer period
        // TODO: change this magic number to something else
        pwm_period_ = 32000; // assign a magic number to period
    } 
}

void ServoController::start_servos(){
    for (uint8_t i = 0; i < NUM_SERVOS; i++){

        HAL_TIM_PWM_Start(&(SERVO_CHANNELS[i].timer), SERVO_CHANNELS[i].channel);

        if (SERVO_CHANNELS[i].is_complementary){
            HAL_TIMEx_PWMN_Start(&(SERVO_CHANNELS[i].timer), SERVO_CHANNELS[i].channel);
        }
    }
}

bool ServoController::update_servo_position(uint8_t servo_num, float position){
    // boundary check
    if ( position< -1 || position > 1){
        return false;
    }

    // iterate through servo id list to find the corresponding servo
    for (uint8_t i = 0; i < NUM_SERVOS; i++){
        if (servo_num == SERVO_IDS[i]){ // find the corresponding servo
            uint32_t compare_register = SERVO_CHANNELS[i].is_complementary? 
                                        pwm_period_ - POSITION_TO_DUTY_CYCLE(position, pwm_period_)
                                        : POSITION_TO_DUTY_CYCLE(position, pwm_period_);

            __HAL_TIM_SET_COMPARE(&(SERVO_CHANNELS[i].timer), SERVO_CHANNELS[i].channel,
                                    compare_register);
            
            return true;
        }
    }

    // the servo num isn't on the list, failure
    return false;
}

bool ServoController::update_servo_last_update_us(uint8_t servo_num, uint64_t time_us){
    for (uint8_t i = 0; i < NUM_SERVOS; i++){
        if (servo_num == SERVO_IDS[i]){
            last_update_us_[i] = time_us;
            return true;
        }
    }

    return false;
}
