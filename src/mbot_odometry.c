#include "mbot_odometry.h"
#include <math.h>

void mbot_calculate_odometry(float vx, float vy, float wz, float dt, float* x, float* y, float* theta) {
    // Update pose
    *x += vx * dt * cos(*theta) - vy * dt * sin(*theta);
    *y += vx * dt * sin(*theta) + vy * dt * cos(*theta);
    *theta += wz * dt;

    // Normalize theta to [-pi, pi]
    while (*theta > M_PI) *theta -= 2.0 * M_PI;
    while (*theta <= -M_PI) *theta += 2.0 * M_PI;
}
