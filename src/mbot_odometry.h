#ifndef MBOT_ODOMETRY_H
#define MBOT_ODOMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Calculate odometry based on velocities
 * 
 * @param vx Forward velocity in m/s
 * @param vy Lateral velocity in m/s (always 0 for diff drive)
 * @param wz Angular velocity in rad/s
 * @param dt Time step in seconds
 * @param x Current x position (will be updated)
 * @param y Current y position (will be updated)
 * @param theta Current orientation (will be updated)
 */
void mbot_calculate_odometry(float vx, float vy, float wz, float dt, float* x, float* y, float* theta);

#ifdef __cplusplus
}
#endif

#endif // MBOT_ODOMETRY_H 