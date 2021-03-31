#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Motor.h"
#include "Ultra.h"
#include <unistd.h>

#define FOREVER 1
#define ERROR 0
#define SUCCESS 1
#define MOTOR_FORWARD 0
#define MOTOR_BACKWARD 1
#define LEFT 1
#define RIGHT 2
#define DISTANCE_TRESHOLD 10
#define REFRESH_RATE 50

/**
* @brief
*   Turns the robot left or right, tank style
* @param direction
*   LEFT (1) or RIGHT (2)
* @ param speed
*   
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
            fprintf(stderr, "%s\n", "Invalid turn direction!");
            return ERROR;
    }
    return SUCCESS;
}

/**
* @brief XOR go bbrrrrrr..
*
* @param prev previous random state
*/
u_int8_t xorshift(u_int32_t *prev)
{
	uint32_t x = *prev;
	x ^= x << 14;
	x ^= x >> 19;
	x ^= x << 6;
    *prev = x;
    return x & 1;
}

void zmain(void)
{
    u_int32_t rand = 0xfea84c23;
    Ultra_Start();
    motor_start();
    motor_forward(0,0);
    while(FOREVER) 
    { 
        if (Ultra_GetDistance() < DISTANCE_TRESHOLD) 
        {
            motor_backward(50, 500);
            tank_turn(xorshift(&rand) ? RIGHT : LEFT, 50, 800);
        }
        else 
        {
            motor_forward(50, REFRESH_RATE);
        }
    }
}