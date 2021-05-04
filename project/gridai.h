#include "botlib.h"

#ifndef AI_H
#define AI_H
    
#define EDGE_NONE 0
#define EDGE_RIGHT 1
#define EDGE_LEFT -1

int grid_finish_line(struct navigator *nav);

void AI_grid_scan_xroads(struct navigator *nav);

void AI_grid_next(struct navigator *nav);

void AI_grid_finish(struct navigator *nav);

void AI_grid_face_north(struct navigator *nav);

int AI_grid_behaviour(struct navigator *nav);

int grid_side_bias_dir(struct navigator *nav);

int grid_on_edge(struct navigator *nav);

#endif // !AI_H