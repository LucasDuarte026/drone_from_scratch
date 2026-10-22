#ifndef IMU_ATTITUDE_H
#define IMU_ATTITUDE_H

#include <math.h>
#include "drone_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * First-order IIR low-pass filter on angular rates (Eq. 1-3)
 * p^{k+1} = 0.7 * p^k + 0.3 * p_new
 */
static inline double imu_filter_rate(double prev_rate, double new_sample) {
    return (prev_rate * LPF_GYRO_PREV) + (new_sample * LPF_GYRO_NEW);
}

/**
 * Accelerometer gravity projection tilt angles in degrees (Eq. 8-10)
 * phi_acc   = asin(Acc_y / Acc_Total)
 * theta_acc = -asin(Acc_x / Acc_Total)
 */
static inline void imu_calc_acc_angles(double ax, double ay, double az,
                                       float *angle_roll_acc, float *angle_pitch_acc) {
    double total = sqrt(ax * ax + ay * ay + az * az);
    if (total < 1e-4) {
        if (angle_roll_acc) *angle_roll_acc = 0.0f;
        if (angle_pitch_acc) *angle_pitch_acc = 0.0f;
        return;
    }

    double ratio_y = ay / total;
    double ratio_x = ax / total;

    // Numerical clamp to [-1.0, 1.0] to prevent NaN on asin()
    if (ratio_y > 1.0) ratio_y = 1.0;
    if (ratio_y < -1.0) ratio_y = -1.0;
    if (ratio_x > 1.0) ratio_x = 1.0;
    if (ratio_x < -1.0) ratio_x = -1.0;

    if (angle_roll_acc)  *angle_roll_acc  = (float)(asin(ratio_y) * RAD_TO_DEG);
    if (angle_pitch_acc) *angle_pitch_acc = (float)(-asin(ratio_x) * RAD_TO_DEG);
}

/**
 * Euler kinematic rates in deg/s (Eq. 4-7)
 * d_roll  = p + q * tan(theta) * sin(phi) + r * tan(theta) * cos(phi)
 * d_pitch = q * cos(phi) - r * sin(phi)
 * d_yaw   = (q * sin(phi) + r * cos(phi)) * sec(theta)
 */
static inline void imu_calc_euler_rates(double p, double q, double r,
                                        float roll_deg, float pitch_deg,
                                        double *d_roll, double *d_pitch, double *d_yaw) {
    double phi   = (double)roll_deg * DEG_TO_RAD;
    double theta = (double)pitch_deg * DEG_TO_RAD;

    double sin_phi   = sin(phi);
    double cos_phi   = cos(phi);
    double tan_theta = tan(theta);
    double cos_theta = cos(theta);
    double sec_theta = (fabs(cos_theta) > 1e-6) ? (1.0 / cos_theta) : 1e6;

    if (d_roll)  *d_roll  = p + q * tan_theta * sin_phi + r * tan_theta * cos_phi;
    if (d_pitch) *d_pitch = q * cos_phi - r * sin_phi;
    if (d_yaw)   *d_yaw   = q * sin_phi * sec_theta + r * cos_phi * sec_theta;
}

/**
 * Complementary filter fusion (Eq. 11-12)
 * angle = 0.9996 * gyro_integrated + 0.0004 * acc_angle
 */
static inline float imu_fuse_angle(float gyro_integrated_deg, float acc_deg) {
    return (float)(COMP_FILTER_GYRO * (double)gyro_integrated_deg + COMP_FILTER_ACC * (double)acc_deg);
}

#ifdef __cplusplus
}
#endif

#endif // IMU_ATTITUDE_H
