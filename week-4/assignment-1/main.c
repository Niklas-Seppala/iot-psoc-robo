#include "project.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "IR.h"
#include "Ultra.h"
#include "Motor.h"

#define FOREVER 1
#define ERROR 0
#define SUCCESS 1
#define MOTOR_FORWARD 0
#define MOTOR_BACKWARD 1


#define LEFT 1
#define RIGHT 2

/**
* @brief Turns the robot left or right, tank style
*
* @param direction LEFT (1) or RIGHT (2)
* @param speed turn speed
* @param duration turn duration
*/
int tank_turn(u_int8_t direction, u_int16_t speed, u_int16_t duration) 
{
    switch (direction)
    {
        case LEFT:
            SetMotors(MOTOR_BACKWARD, MOTOR_FORWARD, speed, speed, duration);
            break;
        case RIGHT:
            SetMotors(MOTOR_FORWARD, MOTOR_BACKWARD, speed, speed, duration);
            break;
        default:
            fprintf(stderr, "\n%s: { %d }\n", "Invalid turn direction", direction);
            return ERROR;
    }
    return SUCCESS;
}

void zmain(void)
{
    // Setup
    motor_start();
    motor_forward(0,0);
    
    // Follow the line
    motor_forward(100, 3550);
    tank_turn(RIGHT, 50, 524);
    motor_forward(100, 2950);
    tank_turn(RIGHT, 50, 524);
    motor_forward(100, 3200);
    tank_turn(RIGHT, 50, 600);
    motor_forward(100, 500);
    motor_turn(100, 80, 2200);
    motor_forward(100, 900);
    
    // Stop
    motor_forward(0,0);
    motor_stop();
    
    while(FOREVER) { vTaskDelay(100);}
}
