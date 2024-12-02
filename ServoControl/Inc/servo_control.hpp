#ifndef SERVO_CONTROL_HPP_
#define SERVO_CONTROL_HPP_

#include "tim.h"

// enable SSD board config
#define SINGLE_SERVO_DRIVER

typedef struct{
    TIM_HandleTypeDef timer;
    unsigned int channel;
    bool is_complementary;
} ServoChannel_t;

class ServoController {
    public:
        /**
         * constructor
         */
        ServoController();

        /**
         * function to start the PWM generation for all servos
         */
        void start_servos();

        /**
         * update the servo position member vairable, as well as the PWM
         * 
         * @param servo_num: the servo id with changed position
         * 
         * @param position: varies from -1 to 1
         * 
         * @return: update result
         */
        bool update_servo_position(uint8_t servo_num, float position);

        /**
         * update the last servo update time in us
         * 
         * @param servo_num: the servo id with new update time
         * 
         * @param time_us: last update time in us
         * 
         * @return: update result
         */
        bool update_servo_last_update_us(uint8_t servo_num, uint64_t time_us);

    private:
        // TWO PCB configurations
        #ifdef SINGLE_SERVO_DRIVER
            static const constexpr uint8_t NUM_SERVOS = 1;

            /* configure the servo ID by changing the following number! */
            static const constexpr uint8_t SERVO_IDS[NUM_SERVOS] = {5};

            /* servo channel from single servo driver board */
            static ServoChannel_t SERVO_CHANNELS[NUM_SERVOS];
        #endif

        #ifdef SIX_S_SERVO_MODULE
            static const constexpr uint8_t NUM_SERVOS = 4;

            /* configure the servo ID by changing the following number! */
            static const constexpr uint8_t SERVO_IDS[NUM_SERVOS] = {5, 6, 7, 8};

            /* servo channels from 6s servo module board */
            static const ServoChannel_t SERVO_CHANNELS[NUM_SERVOS];
        #endif

        // this position value varies fro -1 to 1
        float servo_positions_[NUM_SERVOS];
        uint64_t last_update_us_[NUM_SERVOS];
        uint32_t pwm_period_;

        // TODO: private method that calibrates the PWM frequency, now we assume it is configured correctly from ioc
};

#endif