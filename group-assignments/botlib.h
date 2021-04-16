#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Motor.h"
#include "Ultra.h"
#include "Reflectance.h"
#include "LSM303D.h"
#include "IR.h"
#include "mqtt_sender.h"

#define FOREVER 1

/************************************************************************/
/*                                    IO                                */
/************************************************************************/
#define BTN_PRESSED 0
#define BTN_RELEASED 1
/**
 * @brief 
 * 
 */
void io_wait_SW1();

/**
 * @brief 
 * 
 * @param s 
 */
void io_print_refl(struct sensors_ *s);


/************************************************************************/
/*                              MOVEMENT                                */
/************************************************************************/
#define RIGHT 1
#define LEFT 2
#define FORWARD 0
#define BACKWARD 1
extern const int ROBOT_SPEED;

/**
 * @brief 
 * 
 * @param direction 
 * @param speed 
 * @param duration 
 */
void tank_turn(int direction, int speed, int duration);

/**
 * @brief 
 * 
 * @param dir 
 * @param speed 
 * @param sensors 
 */
void smart_tank_turn(int dir, int speed, struct sensors_ * sensors);


/************************************************************************/
/*                             NAVIAGTION                               */
/************************************************************************/

/**
 * @brief 
 * 
 * @param refl_sensors 
 * @return int 
 */
int on_line(struct sensors_ *refl_sensors);


/************************************************************************/
/*                              UTILITY                                 */
/************************************************************************/
#define INIT_IR    0x1
#define INIT_MOTOR 0x2
#define INIT_ULTRA 0x4

/**
 * @brief Holds reflectance sensor's activation
 *        thresholds.
 */
typedef struct refl_configuration {
    uint16_t L3;
    uint16_t L2;
    uint16_t L1;
    uint16_t R1;
    uint16_t R2;
    uint16_t R3;
} refl_conf_t;

/**
 * @brief Initializes robot components.
 * 
 * @param refl_confs reflectance sensor configs.
 *          set to NULL if robot doesnt use reflectance.
 * @param flags init flags
 */
void init(refl_conf_t *refl_confs, int flags);

/**
 * @brief Gets a random integer inside set range.
 * 
 * @param min range minimun
 * @param max range maximum
 * @return int random integer
 */
int rand_range(int min, int max);
