#include <stdio.h>
#include <pigpio.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#define DATA_FORMAT    0x31
#define DATA_FORMAT_B  0x0B
#define READ_BIT       0x80
#define MULTI_BIT      0x40
#define BW_RATE        0x2C
#define POWER_CTL      0x2D
#define DATAX0         0x32
#define DEVID          0x00
#define EXPECTED_DEVID 0xE5

#define CODE_VERSION "0.4"

const int freqDefault = 250;
const int freqMax = 3200;
const int speedSPI = 2000000;
const int coldStartSamples = 2;
const double coldStartDelay = 0.1;
const double accConversion = 2 * 16.0 / 8192.0;

volatile sig_atomic_t keepRunning = 1;
struct termios orig_term;

void intHandler(int dummy) {
    keepRunning = 0;
}

void printUsage() {
    printf("adxl345spi (version %s)\n"
           "Usage: adxl345spi [OPTION]...\n"
           "  -s, --save FILE     Save data to specified FILE\n"
           "  -f, --freq FREQ     Sampling rate in Hz (default: %d, max: %d)\n"
           "  -t, --time SECONDS  Stop after SECONDS seconds\n",
           CODE_VERSION, freqDefault, freqMax);
}

void cleanup(int h) {
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_term);
    if (h >= 0) spiClose(h);
    gpioTerminate();
}

int kbhit() {
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON);
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

int readBytes(int handle, char *data, int count) {
    char address = data[0];
    data[0] |= READ_BIT;
 
    // Set MUTLI_BIT only for sequential reads from DATAX0
    if (address == DATAX0) {
        data[0] |= MULTI_BIT;
    }

    return spiXfer(handle, data, data, count);
}

int writeBytes(int handle, char *data, int count) {
    // MULTI_BIT is only used for multi-byte READs.
    return spiWrite(handle, data, count);
}

int main(int argc, char *argv[]) {
    int i;
    signal(SIGINT, intHandler);  // handle Ctrl+C

    int bSave = 0;
    int duration = 0;
    char vSave[256] = "";
    double vFreq = freqDefault;

    for (i = 1; i < argc; i++) {
        if ((strcmp(argv[i], "-s") == 0) || (strcmp(argv[i], "--save") == 0)) {
            bSave = 1;
            if (i + 1 < argc) {
                strcpy(vSave, argv[++i]);
            } else {
                fprintf(stderr, "Error: Missing argument for -s/--save option.\n");
                printUsage();
                return 1;
            }
        } else if ((strcmp(argv[i], "-f") == 0) || (strcmp(argv[i], "--freq") == 0)) {
            if (i + 1 < argc) {
                vFreq = atof(argv[++i]);
                if (vFreq < 1 || vFreq > freqMax) {
                    fprintf(stderr, "Error: Invalid frequency. Must be between 1 and %d Hz.\n", freqMax);
                    return 1;
                }
            } else {
                fprintf(stderr, "Error: Missing argument for -f/--freq option.\n");
                printUsage();
                return 1;
            }
        } else if ((strcmp(argv[i], "-t") == 0) || (strcmp(argv[i], "--time") == 0)) {
            if (i + 1 < argc) {
                duration = atoi(argv[++i]);
                if (duration < 1) {
                    fprintf(stderr, "Error: Invalid duration.\n");
                    return 1;
                }
            } else {
                fprintf(stderr, "Error: Missing argument for -t/--time option.\n");
                printUsage();
                return 1;
            }
        } else {
            fprintf(stderr, "Error: Unknown option '%s'\n", argv[i]);
            printUsage();
            return 1;
        }
    }

    if (gpioInitialise() < 0) {
        fprintf(stderr, "Error: GPIO initialization failed.\n");
        return 1;
    }

    int h = spiOpen(0, speedSPI, 3);
    if (h < 0) {
        fprintf(stderr, "Error: Failed to open SPI device.\n");
        gpioTerminate();
        return 1;
    }

    tcgetattr(STDIN_FILENO, &orig_term);
    struct termios new_term = orig_term;
    new_term.c_lflag &= ~(ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_term);

    char devid_check[2] = { DEVID | READ_BIT, 0x00 };
    if (spiXfer(h, devid_check, devid_check, 2) != 2) {
        fprintf(stderr, "Error: Failed to read device ID from ADXL345.\n");
        cleanup(h);
        return 1;
    }
    if ((unsigned char)devid_check[1] != EXPECTED_DEVID) {
        fprintf(stderr, "Error: Unexpected device ID: 0x%02X (expected 0x%02X)\n",
                (unsigned char)devid_check[1], EXPECTED_DEVID);
        cleanup(h);
        return 1;
    }

    char data[7];

    // Configure ADXL345
    data[0] = BW_RATE;
    data[1] = 0x0F;
    if (writeBytes(h, data, 2) != 2) {
        fprintf(stderr, "Error: Failed to write BW_RATE register (0x%02X)\n", BW_RATE);
        cleanup(h);
        return 1;
    }

    data[0] = DATA_FORMAT;
    data[1] = DATA_FORMAT_B;
    if (writeBytes(h, data, 2) != 2) {
        fprintf(stderr, "Error: Failed to write DATA_FORMAT register (0x%02X)\n", DATA_FORMAT);
        cleanup(h);
        return 1;
    }

    data[0] = POWER_CTL;
    data[1] = 0x08;
    if (writeBytes(h, data, 2) != 2) {
        fprintf(stderr, "Error: Failed to write POWER_CTL register (0x%02X)\n", POWER_CTL);
        cleanup(h);
        return 1;
    }

    double delay = 1.0 / vFreq;

    if (!bSave) {
        for (i = 0; i < coldStartSamples; i++) {
            data[0] = DATAX0;
            readBytes(h, data, 7);
            time_sleep(coldStartDelay);
        }

        printf("Press Q to stop");
        printf("\n");
        double tStart = time_time();
        double tNow, tElapsed = 0;
        int samples = 0;
        double t = 0;
        int16_t x, y, z;

        while (keepRunning) {
            tNow = time_time();
            tElapsed = tNow - tStart;
            if (duration > 0 && tElapsed >= duration) break;
            if (kbhit()) {
                char ch = getchar();
                if (ch == 'q' || ch == 'Q') break;
            }

            data[0] = DATAX0;
            if (readBytes(h, data, 7) == 7) {
                x = (data[2] << 8) | data[1];
                y = (data[4] << 8) | data[3];
                z = (data[6] << 8) | data[5];
                t = time_time() - tStart;
                printf("time = %.3f, x = %.3f, y = %.3f, z = %.3f\n",
                       t, x * accConversion, y * accConversion, z * accConversion);
                samples++;
            }
            time_sleep(delay);
        }

        tElapsed = time_time() - tStart;
        printf("Captured %d samples in %.2f seconds (%.1f Hz)\n", samples, tElapsed, samples / tElapsed);
        cleanup(h);
        return 0;

    } else {
        printf("Press Q to stop\n");

        FILE *f = fopen(vSave, "w");
        if (!f) {
            fprintf(stderr, "Error: Failed to open file: %s\n", strerror(errno));
            cleanup(h);
            return 1;
        }
        fprintf(f, "time,x,y,z\n");

        const int flushEvery = 1000;
        int bufferIndex = 0;
        int totalSamples = 0;

        double *bt = malloc(sizeof(double) * flushEvery);
        double *bx = malloc(sizeof(double) * flushEvery);
        double *by = malloc(sizeof(double) * flushEvery);
        double *bz = malloc(sizeof(double) * flushEvery);

        if (!bt || !bx || !by || !bz) {
            fprintf(stderr, "Error: Failed to allocate buffer: %s\n", strerror(errno));
            fclose(f);
            cleanup(h);
            return 1;
        }

        int16_t x, y, z;
        double tStart = time_time();
        double tNow, tElapsed = 0, nextSample = 0;

        while (keepRunning) {
            tNow = time_time();
            tElapsed = tNow - tStart;
            if (duration > 0 && tElapsed >= duration) break;
            tNow = time_time() - tStart;
            if (tNow < nextSample)
                continue;

            if (kbhit()) {
                char ch = getchar();
                if (ch == 'q' || ch == 'Q') break;
            }

            nextSample += delay; // exact next sampling time

            // Read one sample
            data[0] = DATAX0;
            if (readBytes(h, data, 7) == 7) {
                x = (data[2] << 8) | data[1];
                y = (data[4] << 8) | data[3];
                z = (data[6] << 8) | data[5];

                double t = time_time() - tStart;
                bt[bufferIndex] = t;
                bx[bufferIndex] = x * accConversion;
                by[bufferIndex] = y * accConversion;
                bz[bufferIndex] = z * accConversion;
                bufferIndex++;
                totalSamples++;
            }

            if (bufferIndex >= flushEvery) {
                for (i = 0; i < flushEvery; i++) {
                    fprintf(f, "%.5f,%.5f,%.5f,%.5f\n", bt[i], bx[i], by[i], bz[i]);
                }
                fflush(f);
                bufferIndex = 0;
            }
        }

        // Write remaining samples
        for (i = 0; i < bufferIndex; i++) {
            fprintf(f, "%.5f,%.5f,%.5f,%.5f\n", bt[i], bx[i], by[i], bz[i]);
        }
        fflush(f);
        fclose(f);

        free(bt); free(bx); free(by); free(bz);
        tElapsed = time_time() - tStart;
        cleanup(h);
        printf("Saved %d samples in %.2f seconds (%.1f Hz) to %s\n",
               totalSamples, tElapsed, totalSamples / tElapsed, vSave);
    }

    return 0;
}