/**
 * ============================================================================
 * Drone Odometry & IMU Driver Tester (C Implementation)
 * 
 * Purpose:
 *   Communicates with Arduino over UART (/dev/ttyACM* or /dev/ttyUSB*) or
 *   runs a real-time synthetic mock simulation if hardware is not plugged in.
 *   Calculates / validates attitude estimation equations and streams telemetry
 *   into 'odometry/telemetry.log' for 3D visualization and real-time debugging.
 * ============================================================================
 */

#define _DEFAULT_SOURCE
#define _GNU_SOURCE
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <signal.h>


#define DEFAULT_OUTPUT_FILE "odometry/logs/telemetry.log"
#define DEFAULT_BAUDRATE    B57600
#define DT_SEC              0.004f   // 250 Hz loop period (4000 microseconds)
#define TELEMETRY_RATE_HZ   25       // Decimated 25 Hz telemetry stream


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)

// Canonical IMU State Variables (Appendix B Contract)
typedef struct {
    int16_t raw_acc_x;
    int16_t raw_acc_y;
    int16_t raw_acc_z;
    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;

    double gyro_axis_cal[3]; // 0=roll, 1=pitch, 2=yaw
    double gyro_roll_input;
    double gyro_pitch_input;
    double gyro_yaw_input;

    float angle_roll_acc;
    float angle_pitch_acc;

    float angle_roll;
    float angle_pitch;
    float angle_yaw;

    bool initialized;
} ImuDriverState;

static volatile bool g_running = true;

static void sigint_handler(int sig) {
    (void)sig;
    g_running = false;
}

// ----------------------------------------------------------------------------
// IMU Attitude Pipeline & Driver Functions
// ----------------------------------------------------------------------------

void imu_init(ImuDriverState *state) {
    memset(state, 0, sizeof(ImuDriverState));
    state->angle_roll = 0.0f;
    state->angle_pitch = 0.0f;
    state->angle_yaw = 0.0f;
    state->initialized = false;
}

/**
 * First-Order IIR Digital Low-Pass Filter:
 * p^{k+1} = 0.7 * p^k + 0.3 * p_new
 */
void imu_filter_gyro(ImuDriverState *state, double roll_dps, double pitch_dps, double yaw_dps) {
    state->gyro_roll_input  = (state->gyro_roll_input  * 0.7) + (roll_dps  * 0.3);
    state->gyro_pitch_input = (state->gyro_pitch_input * 0.7) + (pitch_dps * 0.3);
    state->gyro_yaw_input   = (state->gyro_yaw_input   * 0.7) + (yaw_dps   * 0.3);
}

/**
 * Accelerometer Gravity Projection (Eq. 8-10):
 * Acc_Total = sqrt(Acc_x^2 + Acc_y^2 + Acc_z^2)
 * phi_acc   = asin(Acc_y / Acc_Total) * (180 / pi)
 * theta_acc = -asin(Acc_x / Acc_Total) * (180 / pi)
 */
void imu_calc_acc_angles(ImuDriverState *state, double ax, double ay, double az) {
    double total = sqrt(ax * ax + ay * ay + az * az);
    if (total < 1e-4) {
        state->angle_roll_acc = 0.0f;
        state->angle_pitch_acc = 0.0f;
        return;
    }

    double ratio_y = ay / total;
    double ratio_x = ax / total;

    // Numerical clamp to [-1.0, 1.0] for domain safety
    if (ratio_y > 1.0) ratio_y = 1.0;
    if (ratio_y < -1.0) ratio_y = -1.0;
    if (ratio_x > 1.0) ratio_x = 1.0;
    if (ratio_x < -1.0) ratio_x = -1.0;

    state->angle_roll_acc  = (float)(asin(ratio_y) * RAD_TO_DEG);
    state->angle_pitch_acc = (float)(-asin(ratio_x) * RAD_TO_DEG);
}

/**
 * Euler Kinematic Rate Transformation & Complementary Filter Fusion (Eq. 4-7, 11-12)
 */
void imu_update_attitude(ImuDriverState *state, float dt) {
    double p = state->gyro_roll_input;
    double q = state->gyro_pitch_input;
    double r = state->gyro_yaw_input;

    double phi   = state->angle_roll * DEG_TO_RAD;
    double theta = state->angle_pitch * DEG_TO_RAD;

    double sin_phi = sin(phi);
    double cos_phi = cos(phi);
    double tan_theta = tan(theta);
    double sec_theta = 1.0 / (cos(theta) != 0.0 ? cos(theta) : 1e-6);

    // Euler angle rates in deg/s
    double d_roll  = p + q * tan_theta * sin_phi + r * tan_theta * cos_phi;
    double d_pitch = q * cos_phi - r * sin_phi;
    double d_yaw   = q * sin_phi * sec_theta + r * cos_phi * sec_theta;

    if (!state->initialized) {
        state->angle_roll  = state->angle_roll_acc;
        state->angle_pitch = state->angle_pitch_acc;
        state->angle_yaw   = 0.0f;
        state->initialized = true;
    } else {
        // Complementary filter: 0.9996 Gyro + 0.0004 Accel
        state->angle_roll  = (float)(0.9996 * (state->angle_roll  + d_roll  * dt) + 0.0004 * state->angle_roll_acc);
        state->angle_pitch = (float)(0.9996 * (state->angle_pitch + d_pitch * dt) + 0.0004 * state->angle_pitch_acc);
        state->angle_yaw   = (float)(state->angle_yaw + d_yaw * dt);

        // Normalize yaw to [-180, 180]
        while (state->angle_yaw > 180.0f)  state->angle_yaw -= 360.0f;
        while (state->angle_yaw < -180.0f) state->angle_yaw += 360.0f;
    }
}

// ----------------------------------------------------------------------------
// TODO: User Test Hooks (For real-time inspection and dimensioning verification)
// ----------------------------------------------------------------------------

void test_hook_gyro_sensitivity(const ImuDriverState *state) {
    // TODO: [Test Hook 1] Validação de sensibilidade do giroscópio (65.5 LSB/(deg/s))
    // Inspecione se os valores em dps estão compatíveis com rotações de bancada.
    (void)state;
}

void test_hook_iir_filter(const ImuDriverState *state) {
    // TODO: [Test Hook 2] Validação do filtro IIR (0.7 / 0.3) e tempo de resposta ao degrau
    // Meça a atenuação de ruído de alta frequência e o atraso de fase.
    (void)state;
}

void test_hook_asin_saturation(const ImuDriverState *state) {
    // TODO: [Test Hook 3] Teste de singularidade / gimbal lock e saturação de asin()
    // Verifique o comportamento sob aceleração centrípeta ou quedas livres.
    (void)state;
}

void test_hook_complementary_drift(const ImuDriverState *state) {
    // TODO: [Test Hook 4] Validação de drift do filtro complementar e convergência estática
    // Valide se os ângulos convergem exatamente para a gravidade após perturbações.
    (void)state;
}

// ----------------------------------------------------------------------------
// Serial Port Configuration
// ----------------------------------------------------------------------------

int open_serial_port(const char *device_path) {
    int fd = open(device_path, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        return -1;
    }

    struct termios options;
    if (tcgetattr(fd, &options) != 0) {
        close(fd);
        return -1;
    }

    cfsetispeed(&options, DEFAULT_BAUDRATE);
    cfsetospeed(&options, DEFAULT_BAUDRATE);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1; // 100ms timeout

    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

uint64_t get_time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}

// ----------------------------------------------------------------------------
// Main Loop: Serial Streaming vs Synthetic Mock
// ----------------------------------------------------------------------------

int main(int argc, char *argv[]) {
    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    const char *out_filepath = DEFAULT_OUTPUT_FILE;
    const char *serial_dev = NULL;
    bool force_mock = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--mock") == 0 || strcmp(argv[i], "-m") == 0) {
            force_mock = true;
        } else if ((strcmp(argv[i], "--port") == 0 || strcmp(argv[i], "-p") == 0) && i + 1 < argc) {
            serial_dev = argv[++i];
        } else if ((strcmp(argv[i], "--out") == 0 || strcmp(argv[i], "-o") == 0) && i + 1 < argc) {
            out_filepath = argv[++i];
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            printf("Usage: %s [options]\n", argv[0]);
            printf("Options:\n");
            printf("  --port, -p <dev>   Serial device port (e.g., /dev/ttyACM0)\n");
            printf("  --mock, -m         Force synthetic mock test pattern\n");
            printf("  --out,  -o <file>  Target log file (default: %s)\n", DEFAULT_OUTPUT_FILE);
            printf("  --help, -h         Show this help\n");
            return 0;
        }
    }

    // Try auto-detecting serial if not explicitly set and not mock
    int serial_fd = -1;
    if (!force_mock) {
        if (serial_dev) {
            serial_fd = open_serial_port(serial_dev);
        } else {
            const char *candidates[] = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1", NULL};
            for (int i = 0; candidates[i] != NULL; i++) {
                serial_fd = open_serial_port(candidates[i]);
                if (serial_fd >= 0) {
                    serial_dev = candidates[i];
                    break;
                }
            }
        }
    }

    FILE *log_fp = fopen(out_filepath, "w");
    if (!log_fp) {
        fprintf(stderr, "Error opening output file '%s': %s\n", out_filepath, strerror(errno));
        return 1;
    }

    printf("====================================================\n");
    printf(" Drone Odometry & Driver Tester Active\n");
    printf(" Target file: %s\n", out_filepath);
    if (serial_fd >= 0) {
        printf(" Mode: HARDWARE SERIAL (%s @ 57600 baud)\n", serial_dev);
    } else {
        printf(" Mode: SYNTHETIC MOCK (Harmonic 3D Drone Motion)\n");
        printf(" (Plug Arduino to /dev/ttyACM0 or use --port to switch)\n");
    }
    printf(" Press Ctrl+C to terminate cleanly.\n");
    printf("====================================================\n\n");

    ImuDriverState state;
    imu_init(&state);

    uint64_t start_time = get_time_ms();
    uint64_t last_stream_time = 0;
    double sim_t = 0.0;

    char serial_buffer[512];
    int serial_buf_len = 0;

    while (g_running) {
        uint64_t now = get_time_ms();

        if (serial_fd >= 0) {
            // Read from hardware UART
            char temp_buf[128];
            int n = read(serial_fd, temp_buf, sizeof(temp_buf) - 1);
            if (n > 0) {
                temp_buf[n] = '\0';
                for (int i = 0; i < n; i++) {
                    char c = temp_buf[i];
                    if (c == '\n' || c == '\r') {
                        if (serial_buf_len > 0) {
                            serial_buffer[serial_buf_len] = '\0';
                            // Parse serial line (format: Roll, Pitch, Yaw or raw values)
                            float r, p, y, ax = 0.0f, ay = 0.0f, az = 1.0f;
                            if (sscanf(serial_buffer, "%f %f %f", &r, &p, &y) == 3 ||
                                sscanf(serial_buffer, "%f,%f,%f", &r, &p, &y) == 3) {
                                state.angle_roll  = r;
                                state.angle_pitch = p;
                                state.angle_yaw   = y;
                                
                                // Write to telemetry log
                                fprintf(log_fp, "%llu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                                        (unsigned long long)(now - start_time),
                                        state.angle_roll, state.angle_pitch, state.angle_yaw,
                                        ax, ay, az);
                                fflush(log_fp);
                            }
                            serial_buf_len = 0;
                        }
                    } else if (serial_buf_len < (int)sizeof(serial_buffer) - 2) {
                        serial_buffer[serial_buf_len++] = c;
                    }
                }
            }
            usleep(2000); // 2ms polling
        } else {
            // Synthetic test flight: roll sinusoid, pitch gentle wobble, continuous yaw drift
            if (now - last_stream_time >= (1000 / TELEMETRY_RATE_HZ)) {
                last_stream_time = now;
                sim_t += 0.04; // 25 Hz steps

                // Simulated dynamic rates (deg/s)
                double roll_rate  = 18.0 * cos(sim_t * 1.2);
                double pitch_rate = 12.0 * -sin(sim_t * 0.8);
                double yaw_rate   = 15.0; // 15 deg/s turn

                // Pass through driver IIR filter
                imu_filter_gyro(&state, roll_rate, pitch_rate, yaw_rate);

                // Simulate accelerometer gravity projection
                double true_roll_rad  = (15.0 * sin(sim_t * 1.2)) * DEG_TO_RAD;
                double true_pitch_rad = (10.0 * cos(sim_t * 0.8)) * DEG_TO_RAD;
                double sim_ax = -sin(true_pitch_rad);
                double sim_ay = sin(true_roll_rad);
                double sim_az = cos(true_roll_rad) * cos(true_pitch_rad);

                imu_calc_acc_angles(&state, sim_ax, sim_ay, sim_az);
                imu_update_attitude(&state, 0.04f);

                // Execute User Test Hooks
                test_hook_gyro_sensitivity(&state);
                test_hook_iir_filter(&state);
                test_hook_asin_saturation(&state);
                test_hook_complementary_drift(&state);

                // Stream formatted telemetry line to file
                fprintf(log_fp, "%llu,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f\n",
                        (unsigned long long)(now - start_time),
                        state.angle_roll,
                        state.angle_pitch,
                        state.angle_yaw,
                        sim_ax, sim_ay, sim_az);
                fflush(log_fp);

                // Periodic diagnostic update on terminal
                static int count = 0;
                if (++count % 25 == 0) {
                    printf("[Streaming] t=%5.1fs | Roll=%+6.1f deg | Pitch=%+6.1f deg | Yaw=%+6.1f deg | AccZ=%.2fg\n",
                           (now - start_time) / 1000.0,
                           state.angle_roll, state.angle_pitch, state.angle_yaw, sim_az);
                }
            }
            usleep(5000); // 5ms sleep to yield CPU
        }
    }

    if (serial_fd >= 0) {
        close(serial_fd);
    }
    if (log_fp) {
        fclose(log_fp);
    }

    printf("\nTester stopped cleanly. Telemetry saved to '%s'.\n", out_filepath);
    return 0;
}
