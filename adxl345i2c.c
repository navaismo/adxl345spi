#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <signal.h>
#include <termios.h>
#include <getopt.h>
#include <time.h>

#define I2C_DEVICE "/dev/i2c-3" //change to match i2c interface
#define I2C_ADDR      0x53
#define DATA_FORMAT   0x31
#define DATA_FORMAT_B 0x0B
#define BW_RATE       0x2C
#define POWER_CTL     0x2D
#define DATAX0        0x32

#define CODE_VERSION "0.1"

const int freqDefault = 250;
const int freqMax = 3200;
const int coldStartSamples = 2;
const double coldStartDelay = 0.1;
const double accConversion = 2 * 16.0 / 8192.0;

volatile sig_atomic_t keepRunning = 1;

void intHandler(int dummy) {
    keepRunning = 0;
}

void printUsage() {
    printf("adxl345i2c (version %s)\n"
           "Usage: adxl345i2c [OPTION]...\n"
           "  -s, --save FILE     Save data to specified FILE\n"
           "  -f, --freq FREQ     Sampling rate in Hz (default: %d, max: %d)\n",
           CODE_VERSION, freqDefault, freqMax);
}

int kbhit() {
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

int i2c_fd = -1;

int adxl_init() {
    i2c_fd = open(I2C_DEVICE, O_RDWR);
    if (i2c_fd < 0) {
        perror("Opening I2C device");
        return -1;
    }
    if (ioctl(i2c_fd, I2C_SLAVE, I2C_ADDR) < 0) {
        perror("Selecting I2C device");
        close(i2c_fd);
        return -1;
    }
    return 0;
}

int adxl_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return write(i2c_fd, buf, 2);
}

int adxl_read(uint8_t reg, uint8_t *buf, int len) {
    if (write(i2c_fd, &reg, 1) != 1) return -1;
    return read(i2c_fd, buf, len);
}

void adxl_close() {
    if (i2c_fd >= 0) close(i2c_fd);
}

double time_now() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

int main(int argc, char *argv[]) {
    signal(SIGINT, intHandler);

    int bSave = 0;
    char vSave[256] = "";
    double vFreq = freqDefault;

    static struct option long_opts[] = {
        {"save", required_argument, 0, 's'},
        {"freq", required_argument, 0, 'f'},
        {"help", no_argument, 0, 'h'},
        {0,0,0,0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "s:f:h", long_opts, NULL)) != -1) {
        switch (opt) {
            case 's':
                bSave = 1;
                strncpy(vSave, optarg, sizeof(vSave)-1);
                break;
            case 'f':
                vFreq = atof(optarg);
                if (vFreq < 1) vFreq = 1;
                if (vFreq > freqMax) vFreq = freqMax;
                break;
            case 'h':
                printUsage();
                return 0;
            default:
                printUsage();
                return 1;
        }
    }

    if (adxl_init() < 0) return 1;

    // Verify device ID
    uint8_t id = 0;
    if (adxl_read(0x00, &id, 1) != 1 || id != 0xE5) {
        printf("ADXL345 not found (ID = 0x%02X)\n", id);
        adxl_close();
        return 1;
    }

    // Configure ADXL345
    adxl_write(BW_RATE, 0x0F);
    adxl_write(DATA_FORMAT, DATA_FORMAT_B);
    adxl_write(POWER_CTL, 0x08);

    // Cold start samples (discard)
    for (int i = 0; i < coldStartSamples; i++) {
        uint8_t data[6];
        adxl_read(DATAX0, data, 6);
        usleep((int)(coldStartDelay * 1e6));
    }

    double delay = 1.0 / vFreq;

    printf("Press Q to stop\n");

    if (!bSave) {
        double tStart = time_now();
        int samples = 0;
        while (keepRunning) {
            if (kbhit()) {
                int c = getchar();
                if (c == 'q' || c == 'Q') break;
            }
            uint8_t data[6];
            if (adxl_read(DATAX0, data, 6) != 6) {
                printf("I2C read error\n");
                break;
            }
            int16_t x_raw = (int16_t)(data[1] << 8 | data[0]);
            int16_t y_raw = (int16_t)(data[3] << 8 | data[2]);
            int16_t z_raw = (int16_t)(data[5] << 8 | data[4]);
            double t = time_now() - tStart;
            double x = x_raw * accConversion;
            double y = y_raw * accConversion;
            double z = z_raw * accConversion;
            printf("time = %.3f, x = %.3f, y = %.3f, z = %.3f\n", t, x, y, z);
            fflush(stdout);
            usleep((int)(delay * 1e6));
            samples++;
        }
        double tElapsed = time_now() - tStart;
        printf("Captured %d samples in %.2f seconds (%.1f Hz)\n", samples, tElapsed, samples / tElapsed);
    } else {
        FILE *f = fopen(vSave, "w");
        if (!f) {
            perror("Opening save file");
            adxl_close();
            return 1;
        }
        fprintf(f, "time,x,y,z\n");
        fflush(f);

        const int flushEvery = 1000;
        int totalSamples = 0;
        double tStart = time_now();

        while (keepRunning) {
            if (kbhit()) {
                int c = getchar();
                if (c == 'q' || c == 'Q') break;
            }
            uint8_t data[6];
            if (adxl_read(DATAX0, data, 6) != 6) {
                printf("I2C read error\n");
                break;
            }
            int16_t x_raw = (int16_t)(data[1] << 8 | data[0]);
            int16_t y_raw = (int16_t)(data[3] << 8 | data[2]);
            int16_t z_raw = (int16_t)(data[5] << 8 | data[4]);
            double t = time_now() - tStart;
            double x = x_raw * accConversion;
            double y = y_raw * accConversion;
            double z = z_raw * accConversion;
            fprintf(f, "%.6f,%.6f,%.6f,%.6f\n", t, x, y, z);
            totalSamples++;
            if (totalSamples % flushEvery == 0) fflush(f);
            usleep((int)(delay * 1e6));
        }
        fclose(f);
    }

    adxl_close();
    return 0;
}
