#include "botlib.h"

#ifndef AI_H
#define AI_H

#define EDGE_NONE 0
#define EDGE_RIGHT 1
#define EDGE_LEFT -1

/**
 * @brief Moves to the next crossroads on the
          grid map.
 * 
 * @param nav robot's navigator
 */
void AI_grid_next(struct navigator *nav);

/**
 * @brief Drives to the finish "line".
 * 
 * @param nav robot's navigator
 */
void AI_grid_finish(struct navigator *nav);

/**
 * @brief Makes the direction decision on the grid
 *        crossroads.
 * 
 * @param nav robot's navigator
 * @return int 0 if grid map is solved.
 */
int AI_grid_behaviour(struct navigator *nav);

/**
 * @brief Updates navigator position based
 *        on previous turns.
 * 
 * @param nav navigator object
 */
void AI_grid_update_pos(struct navigator *nav);

#endif // !AI_H