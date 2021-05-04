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

int io_SW1_pressed(void)
{
    if (SW1_Read() == BTN_PRESSED)
    {
        while (SW1_Read() != BTN_RELEASED);
        return 1;
    }
    
    return 0;
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
}

static int init_flags = -1;

void *init(refl_conf_t *refl_confs, int flags) 
{
    printf("\n ******************\n         BOOT\n ******************\n");
    
    void *r_val = NULL;
    if (refl_confs) 
    {
        r_val = calloc(1, sizeof(struct sensors_));
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
    if (flags & INIT_ULTRA)
    {
        srand((unsigned)time(NULL));
    }
    
    init_flags = flags;
    return r_val;
}

void shutdown(void *h_allocated)
{
    if (init_flags > 0) 
    {
        if (h_allocated) 
        {
            free(h_allocated);
        }    
        
        if (init_flags & INIT_MOTOR)
        {
            motor_forward(0,0);
            motor_stop();
        }
    }
    while(FOREVER);
}

int on_line(struct sensors_ *refl_sensors) 
{
    return refl_sensors->L3 && refl_sensors->R3;
}

int not_on_line(struct sensors_ *refl_sensors) 
{
    return !on_line(refl_sensors);
}

int negate_dir(int dir)
{
    return dir == LEFT ? RIGHT : LEFT;
}

int partially_on_line(struct sensors_ *sensors)
{
    return (sensors->L3 || sensors->R3);
}

void follow_line(struct sensors_ *sensors, int speed)
{
    if (!sensors->R1)
    {
        SetMotors(1, 0, speed, speed, 0);
    }
    else if (!sensors->L1)
    {
        SetMotors(0, 1, speed, speed, 0);
    }
    else
    {
        motor_forward(speed, 0);
    }
    reflectance_digital(sensors);
}

void update_pos(struct navigator *nav)
{
    if (!(nav->direction))
        nav->y++;
    else
        nav->x += nav->direction == LEFT ? -1 : 1;
}

int rand_range(int min, int max)
{
    return (rand() % (max - min + 1)) + min;
}
