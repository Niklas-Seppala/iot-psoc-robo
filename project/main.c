#include "botlib.h"

#define PROJECT_1 1
#define PROJECT_2 0
#define PROJECT_3 0

#if PROJECT_1

#define DISTANCE_TRESHOLD 2
#define SPEED 50
// These turn degrees are linked to constant speed
// defined earlier.
#define TURN_90_DEG 525
#define TURN_45_DEG 262
    
void navigate(struct sensors_ *sensors);
    
void zmain(void) 
{
    refl_conf_t confs = { 9000, 9000, 11000, 11000, 9000, 9000 };
    int init_flags = INIT_MOTOR | INIT_ULTRA | INIT_IR | INIT_RAND;
    struct sensors_ *sensors = init(&confs, init_flags);
    
    // Drive to the start line.
    io_wait_SW1();
    reflectance_digital(sensors);
    while (!on_line(sensors)) 
   {
        motor_forward(SPEED, 0);
        reflectance_digital(sensors);
    }
    motor_forward(0, 0);
    
    // Ready state. Wait for IR signal.
    print_mqtt("zumo02/ready", " %s", "zumo");
    IR_wait();
    
    // Enter the ring.
    uint32_t start_ticks = xTaskGetTickCount();
    print_mqtt("zumo02/start", " %lu", start_ticks);
    motor_forward(SPEED, 1500);
    
    // Execute sumo behaviour.
    while (!io_SW1_pressed())
    {
        navigate(sensors);
        motor_forward(SPEED, 0);
    }
    
    // Send run summary and hibernate.
    print_mqtt("zumo02/stop", " %lu", xTaskGetTickCount());
    print_mqtt("zumo02/time", " %lu", xTaskGetTickCount() - start_ticks);
    shutdown((void**)&sensors);
}

void navigate(struct sensors_ *sensors)
{
    if (Ultra_GetDistance() < DISTANCE_TRESHOLD)
    {
        print_mqtt("zumo02/obstacle", " %lu", xTaskGetTickCount());
        motor_backward(SPEED, 500);
        tank_turn(rand_range(RIGHT, LEFT), SPEED, TURN_90_DEG);
    }
    reflectance_digital(sensors);
    if (sensors->L3 && !sensors->R3)
    {
        // Left side is clipping -> turn right.
        motor_backward(SPEED, 200);
        tank_turn(RIGHT, SPEED, TURN_45_DEG);
    }
    else if (!sensors->L3 && sensors->R3)
    {
        // Right side is clipping -> turn left.
        motor_backward(SPEED, 200);
        tank_turn(LEFT, SPEED, TURN_45_DEG);
    }
    else if (on_line(sensors))
    {
        // Direct collision incoming.
        motor_backward(SPEED, 200);
        tank_turn(rand_range(RIGHT, LEFT), SPEED, TURN_90_DEG);
    }
}

#endif
#if PROJECT_2

void zmain(void) 
{
    
}

#endif    
#if PROJECT_3

void zmain(void) 
{
    
}

#endif
