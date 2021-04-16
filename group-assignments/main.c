#include <project.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Motor.h"
#include "Ultra.h"
#include "Nunchuk.h"
#include "Reflectance.h"
#include "Gyro.h"
#include "Accel_magnet.h"
#include "LSM303D.h"
#include "IR.h"
#include "Beep.h"
#include "mqtt_sender.h"
#include <time.h>
#include <sys/time.h>
#include "serial1.h"
#include <stdlib.h>
#include <unistd.h>

void print_refl_sensor(struct sensors_ *s) 
{
    printf("%d%12d%12d%13d%15d%15d\n",
        s->L3, s->L2, s->L1,
        s->R1, s->R2, s->R3);     
}

#define FOREVER 1
#define BTN_PRESSED 0
#define BTN_RELEASED 1
void wait_SW1() 
{
    while(FOREVER) {
        if (SW1_Read() == BTN_PRESSED)
        {
            while (SW1_Read() != BTN_RELEASED);
            break;
        }
        vTaskDelay(50);
    }
}

#define RIGHT 1
#define LEFT 2
#define FORWARD 0
#define BACKWARD 1
void tank_turn(int direction, int speed, int duration)
{
    if (direction == RIGHT)
    {
        SetMotors(FORWARD, BACKWARD, speed, speed, duration);
    }
    else if (direction == LEFT)
    {
        SetMotors(BACKWARD, FORWARD, speed, speed, duration);
    }
}
void smart_tank_turn(int dir, int speed, struct sensors_ * sensors)
{
    while (sensors->L1 || sensors->R1) 
    {
        tank_turn(dir, speed, 0);   
        reflectance_digital(sensors);
    }
    while (!sensors->L1 || !sensors->R1) 
    {
        tank_turn(dir, speed, 0);   
        reflectance_digital(sensors);
    }
    tank_turn(dir, speed, 24);
}


#define ASSIGNMENT_3_1 0
#define ASSIGNMENT_3_2 0
#define ASSIGNMENT_3_3 0
#define ASSIGNMENT_4_1 0
#define ASSIGNMENT_4_2 0
#define ASSIGNMENT_4_3 0


/************************************************************************/
//                                  WEEK 3
/************************************************************************/


#if ASSIGNMENT_3_1
void zmain(void)
{
    motor_start();
    motor_forward(0, 0);
    motor_forward(100, 2950);
    motor_turn(100, 50, 1050);
    motor_forward(100, 2000);
    motor_turn(100, 50, 1050);
    motor_forward(100, 2300);
    motor_turn(100, 0, 600);
    motor_forward(100, 200);
    motor_turn(100, 90, 1200);
    motor_turn(100, 70, 1100);
    motor_forward(100, 900);
    
    motor_stop();
    
    while(FOREVER) { vTaskDelay(100); }
}
#endif

#if ASSIGNMENT_3_2

#define DISTANCE_TRESHOLD 10
    
void turn() 
{
    motor_turn(0, 50, 1450);
}

void reverse() 
{
    motor_backward(50, 500);
}

void zmain(void)
{
    motor_start();
    Ultra_Start();
    
    while(FOREVER) 
    { 
        if (Ultra_GetDistance() < DISTANCE_TRESHOLD)
        {
            reverse();
            turn();
        }
        else 
        {
            motor_forward(100, 0);
        }
    }
    
    motor_stop();
}
#endif

#if ASSIGNMENT_3_3

#define DISTANCE_TRESHOLD 10
#define RIGHT 1
#define LEFT 2
#define FORWARD 0
#define BACKWARD 1
#define TURN_90_DEG 525
#define TURN_270_DEG 1575
#define SPEED 50

void reverse() 
{
    motor_backward(SPEED, 500);
}

void tank_turn(int direction, int speed, int duration)
{
    if (direction == RIGHT)
    {
        SetMotors(FORWARD, BACKWARD, speed, speed, duration);
    }
    else if (direction == LEFT)
    {
        SetMotors(BACKWARD, FORWARD, speed, speed, duration);
    }
}

int rand_range(int min, int max)
{
    return (rand() % (max - min + 1)) + min;
}

void zmain(void)
{
    motor_start();
    Ultra_Start();
    srand(time(NULL));
    
    while(FOREVER) 
    { 
        if (Ultra_GetDistance() < DISTANCE_TRESHOLD)
        {
            reverse();
            tank_turn(rand_range(RIGHT, LEFT), SPEED,
                rand_range(TURN_90_DEG, TURN_270_DEG));
        }
        else 
        {
            motor_forward(SPEED, 0);
        }
    }
}

#endif


/************************************************************************/
//                                  WEEK 4
/************************************************************************/

#if ASSIGNMENT_4_1 || ASSIGNMENT_4_2 || ASSIGNMENT_4_3

#define SPEED 50
void start() 
{
    printf("\n\n\nBOOT\n\n\n");
    motor_start();
    motor_forward(0, 0);

    IR_Start();
    IR_flush();
}

int on_line(struct sensors_* refl) 
{
    return refl->L3 == 1 && refl->R3 == 1;
}
    
#endif

#if ASSIGNMENT_4_1

#define LINE_GOAL 5

void zmain(void)
{
    // Setup
    int line_count = 0;
    struct sensors_ sensors;
    reflectance_start();
    reflectance_set_threshold(9000, 9000, 11000, 11000, 9000, 9000);
    start();
    
    // Drive to first line.
    wait_SW1();
    reflectance_digital(&sensors);
    while (!on_line(&sensors)) 
    {
        motor_forward(SPEED, 0);
        reflectance_digital(&sensors);
    }
    motor_forward(0, 0);
    
    // Drive to end after remote signal.
    IR_wait();
    int current_line_state;
    // Calibrate goal line count based on start position.
    int prev_line_state = on_line(&sensors);
    int line_count_goal = prev_line_state ? LINE_GOAL-1 : LINE_GOAL;
    while(line_count < line_count_goal)
    {
        current_line_state = on_line(&sensors);
        // Increment counter if bot drives to line.
        if (current_line_state && !prev_line_state)
                line_count++;
        
        // Store current line state.
        prev_line_state = current_line_state;
        
        motor_forward(SPEED, 0);
        reflectance_digital(&sensors);
    }
    
    // End
    motor_forward(0, 0);
    motor_stop();
    while (FOREVER) {};
}   

#endif

#if ASSIGNMENT_4_2

#define TURN_SPEED 100
    
void zmain(void)
{
    // Setup
    struct sensors_ sensors;
    reflectance_start();
    reflectance_set_threshold(9000, 11000, 11000, 11000, 11000, 9000);
    start();
    
    // Drive to first line.
    wait_SW1();
    reflectance_digital(&sensors);
    while (!on_line(&sensors)) 
    {
        motor_forward(SPEED, 0);
        reflectance_digital(&sensors);
    }
    motor_forward(0, 0);
    
    // Cross the first line.
    IR_wait();
    while (on_line(&sensors))
    {
        motor_forward(SPEED, 0);
        reflectance_digital(&sensors);
    }
    motor_forward(0, 0);
    
    // Drive to the end of the track.
    while (!on_line(&sensors))
    {
        if (sensors.R2)
        {
            SetMotors(0, 1, SPEED, SPEED, 0); // TURN RIGHT
        }
        else if (sensors.L2)
        {
            SetMotors(1, 0, SPEED, SPEED, 0); // TURN LEFT
        }
        else
        {
            motor_forward(SPEED, 0);
        }
        
        reflectance_digital(&sensors);
    }
    
    // End
    motor_forward(0, 0);
    motor_stop();
    while (FOREVER) {}
}

#endif

#if ASSIGNMENT_4_3
    
#define PRE_TURN_OFFSET 250

void zmain(void)
{
    // Setup
    int route[] = { LEFT, RIGHT, RIGHT };
    const int ROUTE_LEN = 3;
    struct sensors_ sensors;
    reflectance_start();
    reflectance_set_threshold(9000, 9000, 11000, 11000, 9000, 9000);
    start();
    
    // Drive to first line.
    wait_SW1();
    reflectance_digital(&sensors);
    while (!on_line(&sensors)) 
    {
        motor_forward(SPEED, 0);
        reflectance_digital(&sensors);
    }
    motor_forward(0, 0);
    
    // Cross the first line.
    IR_wait();
    while (on_line(&sensors))
    {
        motor_forward(SPEED, 0);
        reflectance_digital(&sensors);
    }
    motor_forward(0, 0);
    
    // Drive to the end of the track
    int current_line_state;
    int prev_line_state = on_line(&sensors);
    int turn = 0;
    while (FOREVER)
    {
        current_line_state = on_line(&sensors);
        if (!current_line_state && prev_line_state)
        {
            motor_forward(SPEED, PRE_TURN_OFFSET);
            smart_tank_turn(route[turn++], SPEED, &sensors);
        }
        // Check if robot is at the finish line
        else if (current_line_state && turn == ROUTE_LEN)
        {
            break; 
        }
        motor_forward(SPEED, 0);
        reflectance_digital(&sensors);
        prev_line_state = current_line_state;
    }
    
    // End
    motor_forward(0, 0);
    motor_stop();
    while (FOREVER) {};
}
    
#endif

/************************************************************************/
//                                  WEEK 5
/************************************************************************/

#define ASSIGNMENT_5_1 0
#define ASSIGNMENT_5_2 1
#define ASSIGNMENT_5_3 0

#if ASSIGNMENT_5_1

#define TOPIC "Zumo02/button"

void zmain(void)
{
    init(NULL, 0);
    
    uint64_t prevTicks = xTaskGetTickCount();
    uint64_t currentTicks;
    while(true)
    {
        io_wait_SW1();
        currentTicks = xTaskGetTickCount();
        print_mqtt(TOPIC, " %llu", currentTicks - prevTicks);
        prevTicks = currentTicks;
    }
}

#endif
#if ASSIGNMENT_5_2
    
#define DISTANCE_TRESHOLD 10
#define TURN_90_DEG 525 //ms
#define TOPIC "zumo02/turn"

const int ROBOT_SPEED = 50;
void zmain(void) 
{
    init(NULL, INIT_MOTOR | INIT_ULTRA);
    
    while (FOREVER) 
    {
        if (Ultra_GetDistance() < DISTANCE_TRESHOLD)
        {
            int direction = rand_range(RIGHT, LEFT);
            motor_backward(ROBOT_SPEED, 500);
            tank_turn(direction, ROBOT_SPEED, TURN_90_DEG);
            
            print_mqtt(TOPIC, " %s", direction == RIGHT ? "right" : "left");
        }
        else 
        {
            motor_forward(ROBOT_SPEED, 0);
        }
    }
}

#endif
#if ASSIGNMENT_5_3

#define TOPIC "zumo02/lap"
    
const int ROBOT_SPEED = 50;
void zmain(void)
{
    uint64_t startTicks, endTicks;
    struct sensors_ sensors;
    refl_conf_t confs = { 9000, 9000, 11000, 11000, 9000, 9000 };
    init(&confs, INIT_IR | INIT_MOTOR);
    
    reflectance_digital(&sensors);
    while (!on_line(&sensors)) 
    {
        motor_forward(ROBOT_SPEED, 0);
        reflectance_digital(&sensors);
    }
    motor_forward(0, 0);
    
    while (FOREVER)
    {
        IR_wait();
        startTicks = xTaskGetTickCount();
        reflectance_digital(&sensors);
        while (on_line(&sensors))
        {
            motor_forward(ROBOT_SPEED, 0);
            reflectance_digital(&sensors);
        }
        while (!on_line(&sensors)) 
        {
            motor_forward(ROBOT_SPEED, 0);
            reflectance_digital(&sensors);
        }
        endTicks = xTaskGetTickCount();
        print_mqtt(TOPIC, " %llu", endTicks - startTicks);
        motor_forward(0,0);
    }
}
#endif