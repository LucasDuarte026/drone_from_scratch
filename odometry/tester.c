/**
 * ============================================================================
 * Drone Odometry & IMU Attitude Tester (C Implementation)
 *
 * Purpose:
 *   Communicates with Arduino over UART (/dev/ttyACM* or /dev/ttyUSB*) or
 *   runs a real-time synthetic mock simulation using the canonical Stage 1
 *   math pipeline ('code/include/imu_attitude.h'). Streams telemetry into
 *   'odometry/logs/telemetry.log' for 3D visualization.
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

#include "imu_attitude.h"

#define DEFAULT_OUTPUT_FILE "odometry/logs/telemetry.log"
#define DEFAULT_BAUDRATE    B57600
#define TELEMETRY_RATE_HZ   25

typedef struct {
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

void imu_init(ImuDriverState *state) {
    memset(state, 0, sizeof(ImuDriverState));
}

void imu_step_mock(ImuDriverState *state, double roll_dps, double pitch_dps, double yaw_dps,
                   double ax, double ay, double az, float dt) {
    state->gyro_roll_input  = imu_filter_rate(state->gyro_roll_input,  roll_dps);
    state->gyro_pitch_input = imu_filter_rate(state->gyro_pitch_input, pitch_dps);
    state->gyro_yaw_input   = imu_filter_rate(state->gyro_yaw_input,   yaw_dps);

    imu_calc_acc_angles(ax, ay, az, &state->angle_roll_acc, &state->angle_pitch_acc);

    double d_roll = 0.0, d_pitch = 0.0, d_yaw = 0.0;
    imu_calc_euler_rates(state->gyro_roll_input, state->gyro_pitch_input, state->gyro_yaw_input,
                         state->angle_roll, state->angle_pitch,
                         &d_roll, &d_pitch, &d_yaw);

    if (!state->initialized) {
        state->angle_roll  = state->angle_roll_acc;
        state->angle_pitch = state->angle_pitch_acc;
        state->angle_yaw   = 0.0f;
        state->initialized = true;
    } else {
        float roll_gyro  = state->angle_roll  + (float)(d_roll  * dt);
        float pitch_gyro = state->angle_pitch + (float)(d_pitch * dt);
        state->angle_roll  = imu_fuse_angle(roll_gyro,  state->angle_roll_acc);
        state->angle_pitch = imu_fuse_angle(pitch_gyro, state->angle_pitch_acc);
        state->angle_yaw   = state->angle_yaw + (float)(d_yaw * dt);

        while (state->angle_yaw > 180.0f)  state->angle_yaw -= 360.0f;
        while (state->angle_yaw < -180.0f) state->angle_yaw += 360.0f;
    }
}

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
    options.c_cc[VTIME] = 1;

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
            char temp_buf[128];
            int n = read(serial_fd, temp_buf, sizeof(temp_buf) - 1);
            if (n > 0) {
                temp_buf[n] = '\0';
                for (int i = 0; i < n; i++) {
                    char c = temp_buf[i];
                    if (c == '\n' || c == '\r') {
                        if (serial_buf_len > 0) {
                            serial_buffer[serial_buf_len] = '\0';
                            float r, p, y, ax = 0.0f, ay = 0.0f, az = 1.0f;
                            if (sscanf(serial_buffer, "%f %f %f %f %f %f", &r, &p, &y, &ax, &ay, &az) == 6 ||
                                sscanf(serial_buffer, "%f,%f,%f,%f,%f,%f", &r, &p, &y, &ax, &ay, &az) == 6 ||
                                sscanf(serial_buffer, "%f %f %f", &r, &p, &y) == 3 ||
                                sscanf(serial_buffer, "%f,%f,%f", &r, &p, &y) == 3) {
                                state.angle_roll  = r;
                                state.angle_pitch = p;
                                state.angle_yaw   = y;

                                fprintf(log_fp, "%llu,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f\n",
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
            usleep(2000);
        } else {
            if (now - last_stream_time >= (1000 / TELEMETRY_RATE_HZ)) {
                last_stream_time = now;
                sim_t += 0.04;

                double roll_rate  = 18.0 * cos(sim_t * 1.2);
                double pitch_rate = 12.0 * -sin(sim_t * 0.8);
                double yaw_rate   = 15.0;

                double true_roll_rad  = (15.0 * sin(sim_t * 1.2)) * DEG_TO_RAD;
                double true_pitch_rad = (10.0 * cos(sim_t * 0.8)) * DEG_TO_RAD;
                double sim_ax = -sin(true_pitch_rad);
                double sim_ay = sin(true_roll_rad);
                double sim_az = cos(true_roll_rad) * cos(true_pitch_rad);

                imu_step_mock(&state, roll_rate, pitch_rate, yaw_rate,
                              sim_ax, sim_ay, sim_az, 0.04f);

                fprintf(log_fp, "%llu,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f\n",
                        (unsigned long long)(now - start_time),
                        state.angle_roll,
                        state.angle_pitch,
                        state.angle_yaw,
                        sim_ax, sim_ay, sim_az);
                fflush(log_fp);

                static int count = 0;
                if (++count % 25 == 0) {
                    printf("[Streaming] t=%5.1fs | Roll=%+6.1f deg | Pitch=%+6.1f deg | Yaw=%+6.1f deg | AccZ=%.2fg\n",
                           (now - start_time) / 1000.0,
                           state.angle_roll, state.angle_pitch, state.angle_yaw, sim_az);
                }
            }
            usleep(5000);
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
