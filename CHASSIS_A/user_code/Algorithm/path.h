#ifndef PATH_H
#define PATH_H

#include <stdint.h>

/* x/y/s: mm, theta: rad, curvature: 1/m, velocity: m/s */
typedef struct
{
    float x;
    float y;
    float theta;
    float curvature;
    float s;
    float velocity;
    uint32_t sequence;
    uint32_t flags;
} PathPoint;

#define PATH_SIZE (161U)

extern const PathPoint path[PATH_SIZE];

#endif /* PATH_H */
