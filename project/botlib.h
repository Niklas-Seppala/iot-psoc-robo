#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Motor.h"
#include "Ultra.h"
#include "Reflectance.h"
#include "LSM303D.h"
#include "IR.h"
#include "mqtt_sender.h"
    
#ifndef BOTLIB_H
#define BOTLIB_H

#define FOREVER 1

/***************************************/
/*                  IO                 */
/***************************************/

#define BTN_PRESSED 0
#define BTN_RELEASED 1

/**
 * @brief Waits for user button RELEASE.
 */
void io_wait_SW1(void);

/**
 * @brief Checks if the SW1 button was pressed
 *        (At that moment).
 * 
 * @return int boolean val
 */
int io_SW1_pressed(void);

/**
 * @brief prints reflectance sensor values.
 * 
 * @param s sensor value addr
 */
void io_print_refl(struct sensors_ *s);

/***************************************/
/*              MOVEMENT               */
/***************************************/
#define UP 0
#define RIGHT 1
#define LEFT 2
#define FORWARD 0
#define BACKWARD 1

/**
 * @brief Performs a tank turn based on provided
 *        direction, speed and duration (ms).
 * 
 * @param direction left/right
 * @param speed motor speed
 * @param duration duration in ms
 */
void tank_turn(int direction, int speed, int duration);

/**
 * @brief Performs a "smart" tank turn on grid
 *        crossroads.
 * 
 * @param dir left/right
 * @param speed motor speed
 * @param sensors reflectance sensors
 */
void smart_tank_turn(int dir, int speed, struct sensors_ *sensors);


/***************************************/
/*             NAVIGATION              */
/***************************************/

struct grid_map
{
    int x_min;
    int x_max;
    int y_max;
};

/**
 * @brief Navigator object, holds values
 *        related to navigation.
 */
struct navigator 
{
    int speed;
    int x;
    int y;
    int edge_flag;
    int direction;
    int ultra_treshold;
    struct sensors_ *sensors;
    struct grid_map *map;
};

/**
 * @brief Checks if the outermost sensors are
          on a line.
 * 
 * @param refl_sensors reflectance sensors
 * @return int boolean
 */
int on_line(struct sensors_ *refl_sensors);

/**
 * @brief negated version of "on_line"-function
 *        for function pointer usage.
 * 
 * @param sensors reflectance sensors
 * @return int boolean val
 */
int not_on_line(struct sensors_ *refl_sensors);

/**
 * @brief Checks if atleast other side of
 *        the robot's reflectance sensors
 *        are on a line.
 * 
 * @param sensors reflectance sensors
 * @return int boolean val
 */
int partially_on_line(struct sensors_ *sensors);

/**
 * @brief Left -> Right,
 *        Right -> Left
 * 
 * @param dir direction
 * @return int negated direction
 */
int negate_dir(int dir);

/**
 * @brief Follows the black line.
 * 
 * @param sensors reflectance sensors
 * @param speed desired speed
 */
void follow_line(struct sensors_ *sensors, int speed);

/***************************************/
/*               UTILITY               */
/***************************************/

#define INIT_IR    0x1
#define INIT_MOTOR 0x2
#define INIT_ULTRA 0x4
#define INIT_RAND  0x8

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
void *init(refl_conf_t *refl_confs, int flags);

/**
 * @brief Shut down the robot and release used
 *        resources.
 * 
 * @param h_allocated heap allocated
 */
void shutdown(void *h_allocated);

/**
 * @brief Gets a random integer inside set range.
 * 
 * @param min range minimun
 * @param max range maximum
 * @return int random integer
 */
int rand_range(int min, int max);

#endif // !BOTLIB_H