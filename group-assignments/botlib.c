#include "botlib.h"

void io_wait_SW1() 
{
    while(FOREVER) 
    {
        if (SW1_Read() == BTN_PRESSED)
        {
            while (SW1_Read() != BTN_RELEASED);
            break;
        }
        vTaskDelay(50);
    }
}

void io_print_refl(struct sensors_ *s) 
{
    printf("%d%12d%12d%13d%15d%15d\n",
        s->L3, s->L2, s->L1,
        s->R1, s->R2, s->R3);     
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


void init(refl_conf_t *refl_confs, int flags) 
{
    printf("\n ******************\n         BOOT\n ******************\n");
    
    if (refl_confs) 
    {
        reflectance_start();
        reflectance_set_threshold(
            refl_confs->L3, refl_confs->L2,
            refl_confs->L1, refl_confs->R1,
            refl_confs->R2, refl_confs->R3);
    }
    
    if (flags & INIT_ULTRA)
    {
        Ultra_Start();
        vTaskDelay(30); // fix 0 read at the start
    }
    
    if (flags & INIT_IR)
    {
        IR_Start();
        IR_flush();
    }
    if (flags & INIT_MOTOR)
    {
        motor_start();
        motor_forward(0, 0);
    }
}

int on_line(struct sensors_ *refl_sensors) 
{
    return refl_sensors->L3 && refl_sensors->R3;
}

int rand_range(int min, int max)
{
    return (rand() % (max - min + 1)) + min;
}
