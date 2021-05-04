#include "botlib.h"
#include "gridai.h"

static void face_north(struct navigator *nav);
static void scan_xroads(struct navigator *nav);
static void scan_xroads(struct navigator *nav);
static int on_edge(struct navigator *nav);
static int side_bias_dir(struct navigator *nav);
static int finish_line(struct navigator *nav);


/***********************************************/
/*                  PUBLIC API                 */
/***********************************************/

int AI_grid_behaviour(struct navigator *nav)
{
    scan_xroads(nav);
    motor_forward(nav->speed, 200);
    
    // Stash the previous horizontal direction. 
    // If there was none, make one up.
    int horizontal_dir = nav->direction ? nav->direction : rand_range(RIGHT, LEFT);
    int sbias_dir = side_bias_dir(nav);
    
    face_north(nav);
    reflectance_digital(nav->sensors);
    if (!nav->sensors->R1 && !nav->sensors->L1)
    {
        // Robot is facing off the grid. Its at the top of
        // the map.
        // Turn to the middle of the map.
        nav->direction = sbias_dir;
        smart_tank_turn(nav->direction, nav->speed, nav->sensors);
        return 1;
    }
    
    if (Ultra_GetDistance() < nav->ultra_treshold)
    {
        // Obstacle detected on the north side.
        if (on_edge(nav))
        {
            // Handle possible edge of the map scenario.
            // Turn to the opposite side of the map is only
            // possible option
            nav->direction = sbias_dir;
            smart_tank_turn(nav->direction, nav->speed, nav->sensors);
            return 1;
        }
        // Follow the previous horizontal direction.
        nav->direction = horizontal_dir;
        smart_tank_turn(nav->direction, nav->speed, nav->sensors);
        if (Ultra_GetDistance() < nav->ultra_treshold)
        {
            // Obstacle detected on the dynamic turn decision side.
            // Negate current direction choice, LEFT => RIGHT etc.
            // Turn 180 degrees.
            nav->direction = negate_dir(nav->direction);
            smart_tank_turn(nav->direction, nav->speed, nav->sensors);
            smart_tank_turn(nav->direction, nav->speed, nav->sensors);
        }
    }
    return !finish_line(nav);
}

void AI_grid_next(struct navigator *nav)
{
    // Drive until R3 or L3 sensors detect a line (crossroads).
    while (!partially_on_line(nav->sensors))
    {
        follow_line(nav->sensors, nav->speed);
    }
}

void AI_grid_finish(struct navigator *nav)
{
    // Drive until R1 and R2 both lose the line.
    while (nav->sensors->L1 || nav->sensors->R1) 
    {
        motor_forward(nav->speed, 0);
        reflectance_digital(nav->sensors);
    }
}

void AI_grid_update_pos(struct navigator *nav)
{
    if (nav->direction == UP) 
    {
        nav->y++;
    }
    else 
    {
        nav->x += nav->direction == LEFT ? -1 : 1;
    }
}


/**********************************************/
/*                  INTERNALS                 */
/**********************************************/

static void face_north(struct navigator *nav)
{
    if (nav->direction == RIGHT)
    {
        smart_tank_turn(LEFT, nav->speed, nav->sensors);
    }
    else if (nav->direction == LEFT)
    {
        smart_tank_turn(RIGHT, nav->speed, nav->sensors);
    }
    nav->direction = UP;
}

static void scan_xroads(struct navigator *nav)
{
    motor_forward(nav->speed, 25);
    reflectance_digital(nav->sensors);
    if (nav->sensors->L3 && !nav->sensors->R3)
    {
        nav->edge_flag = EDGE_RIGHT;
    }
    else if (!nav->sensors->L3 && nav->sensors->R3)
    {
        nav->edge_flag = EDGE_LEFT;
    }
    else
    {
        nav->edge_flag = EDGE_NONE;
    }
}

static int finish_line(struct navigator *nav)
{
    return nav->y == nav->map->y_max && nav->x == 0;
}

static int on_edge(struct navigator *nav)
{
    if (nav->x < 0)
    {
        return nav->x == nav->map->x_min || nav->edge_flag == EDGE_LEFT;
    }
    else if (nav->x > 0)
    {
        return nav->x == nav->map->x_max || nav->edge_flag == EDGE_RIGHT;
    }
    return 0;
}

static int side_bias_dir(struct navigator *nav)
{
    if (nav->x < 0)
    {
        return RIGHT;
    }
    else if (nav->x > 0)
    {
        return LEFT;
    }
    else 
    {
        return UP;
    }
}
