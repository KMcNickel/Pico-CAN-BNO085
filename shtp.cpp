#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "include/can.h"
#include "include/mcp2515.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "include/shtp.h"
#include "sh2.h"
#include "sh2_util.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"

uint txPin;
uint rxPin;
uint sckPin;
uint csPin;
uint rstPin;
uint intPin;
uint freq;
spi_inst_t * spiPort;
bool resetOccurred;
sh2_Hal_t * sh2HALInstance;

void eventHandler(void * cookie, sh2_AsyncEvent_t *pEvent)
{
    // If we see a reset, set a flag so that sensors will be reconfigured.
    if (pEvent->eventId == SH2_RESET) {
        printf("The IMU was reset");
        resetOccurred = true;
    }
}

void sensorHandler(void * cookie, sh2_SensorEvent_t *pEvent)
{
    int rc;
    sh2_SensorValue_t value;
    float scaleRadToDeg = 180.0 / 3.14159265358;
    float r, i, j, k, acc_deg, x, y, z;
    float t;
    static int skip = 0;

    rc = sh2_decodeSensorEvent(&value, pEvent);
    if (rc != SH2_OK) {
        printf("Error decoding sensor event: %d\n", rc);
        return;
    }

    t = value.timestamp / 1000000.0;  // time in seconds.
    switch (value.sensorId) {
        case SH2_RAW_ACCELEROMETER:
            printf("%8.4f Raw acc: %d %d %d\n",
                   t,
                   value.un.rawAccelerometer.x,
                   value.un.rawAccelerometer.y,
                   value.un.rawAccelerometer.z);
            break;

        case SH2_ACCELEROMETER:
            printf("%8.4f Acc: %f %f %f\n",
                   t,
                   value.un.accelerometer.x,
                   value.un.accelerometer.y,
                   value.un.accelerometer.z);
            break;
            
        case SH2_RAW_GYROSCOPE:
            printf("%8.4f Raw gyro: x:%d y:%d z:%d temp:%d time_us:%d\n",
                   t,
                   value.un.rawGyroscope.x,
                   value.un.rawGyroscope.y,
                   value.un.rawGyroscope.z,
                   value.un.rawGyroscope.temperature,
                   value.un.rawGyroscope.timestamp);
            break;
            
        case SH2_ROTATION_VECTOR:
            r = value.un.rotationVector.real;
            i = value.un.rotationVector.i;
            j = value.un.rotationVector.j;
            k = value.un.rotationVector.k;
            acc_deg = scaleRadToDeg * 
                value.un.rotationVector.accuracy;
            printf("%8.4f Rotation Vector: "
                   "r:%0.6f i:%0.6f j:%0.6f k:%0.6f (acc: %0.6f deg)\n",
                   t,
                   r, i, j, k, acc_deg);
            break;
        case SH2_GAME_ROTATION_VECTOR:
            r = value.un.gameRotationVector.real;
            i = value.un.gameRotationVector.i;
            j = value.un.gameRotationVector.j;
            k = value.un.gameRotationVector.k;
            printf("%8.4f GRV: "
                   "r:%0.6f i:%0.6f j:%0.6f k:%0.6f\n",
                   t,
                   r, i, j, k);
            break;
        case SH2_GYROSCOPE_CALIBRATED:
            x = value.un.gyroscope.x;
            y = value.un.gyroscope.y;
            z = value.un.gyroscope.z;
            printf("%8.4f GYRO: "
                   "x:%0.6f y:%0.6f z:%0.6f\n",
                   t,
                   x, y, z);
            break;
        case SH2_GYROSCOPE_UNCALIBRATED:
            x = value.un.gyroscopeUncal.x;
            y = value.un.gyroscopeUncal.y;
            z = value.un.gyroscopeUncal.z;
            printf("%8.4f GYRO_UNCAL: "
                   "x:%0.6f y:%0.6f z:%0.6f\n",
                   t,
                   x, y, z);
            break;
        case SH2_GYRO_INTEGRATED_RV:
            // These come at 1kHz, too fast to print all of them.
            // So only print every 10th one
            skip++;
            if (skip == 10) {
                skip = 0;
                r = value.un.gyroIntegratedRV.real;
                i = value.un.gyroIntegratedRV.i;
                j = value.un.gyroIntegratedRV.j;
                k = value.un.gyroIntegratedRV.k;
                x = value.un.gyroIntegratedRV.angVelX;
                y = value.un.gyroIntegratedRV.angVelY;
                z = value.un.gyroIntegratedRV.angVelZ;
                printf("%8.4f Gyro Integrated RV: "
                       "r:%0.6f i:%0.6f j:%0.6f k:%0.6f x:%0.6f y:%0.6f z:%0.6f\n",
                       t,
                       r, i, j, k,
                       x, y, z);
            }
            break;
        case SH2_IZRO_MOTION_REQUEST:
            printf("IZRO Request: intent:%d, request:%d\n",
                   value.un.izroRequest.intent,
                   value.un.izroRequest.request);
            break;
        case SH2_SHAKE_DETECTOR:
            printf("Shake Axis: %c%c%c\n", 
                   (value.un.shakeDetector.shake & SHAKE_X) ? 'X' : '.',
                   (value.un.shakeDetector.shake & SHAKE_Y) ? 'Y' : '.',
                   (value.un.shakeDetector.shake & SHAKE_Z) ? 'Z' : '.');

            break;
        default:
            printf("Unknown sensor: %d\n", value.sensorId);
            break;
    }
}

void reportProdIds()
{
    int status;
    sh2ProductIds_t prodIds;
    
    memset(&prodIds, 0, sizeof(prodIds));
    status = sh2_getProdIds(&prodIds);
    
    if (status < 0) {
        printf("Error %d from sh2_getProdIds.\n", status);
        return;
    }

    for (int n = 0; n < prodIds.numEntries; n++) {
        printf("Part %d : Version %d.%d.%d Build %d\n",
               prodIds.entry[n].swPartNumber,
               prodIds.entry[n].swVersionMajor, prodIds.entry[n].swVersionMinor, 
               prodIds.entry[n].swVersionPatch, prodIds.entry[n].swBuildNumber);
    }
}

void startReports()
{
    static sh2_SensorConfig_t config;
    int status;
    int sensorId;
    static const int enabledSensors[] =
    {
        SH2_GAME_ROTATION_VECTOR,
        // SH2_RAW_ACCELEROMETER,
        // SH2_RAW_GYROSCOPE,
        // SH2_ROTATION_VECTOR,
        // SH2_GYRO_INTEGRATED_RV,
        // SH2_IZRO_MOTION_REQUEST,
        // SH2_SHAKE_DETECTOR,
    };

    // These sensor options are disabled or not used in most cases
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.sniffEnabled = false;
    config.changeSensitivity = 0;
    config.batchInterval_us = 0;
    config.sensorSpecific = 0;

    // Select a report interval.
    // config.reportInterval_us = 100000;  // microseconds (10 Hz)
    // config.reportInterval_us = 40000;  // microseconds (25 Hz)
    config.reportInterval_us = 10000;  // microseconds (100 Hz)
    // config.reportInterval_us = 2500;   // microseconds (400 Hz)
    // config.reportInterval_us = 1000;   // microseconds (1000 Hz)

    for (int n = 0; n < ARRAY_LEN(enabledSensors); n++)
    {
        // Configure the sensor hub to produce these reports
        sensorId = enabledSensors[n];
        status = sh2_setSensorConfig(sensorId, &config);
        if (status != 0) {
            printf("Error while enabling sensor %d\n", sensorId);
        }
    }
    
}

int openSPI(sh2_Hal_t *self)
{
    gpio_init(csPin);
    gpio_set_dir(csPin, GPIO_OUT);
    gpio_put(csPin, 1);

    gpio_init(rstPin);
    gpio_set_dir(rstPin, GPIO_OUT);
    gpio_put(rstPin, 0);

    spi_init(spiPort, freq);

    spi_set_format(spiPort, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    
    gpio_set_function(sckPin, GPIO_FUNC_SPI);
    gpio_set_function(txPin, GPIO_FUNC_SPI);
    gpio_set_function(rxPin, GPIO_FUNC_SPI);
    
    sleep_ms(1);    //Allow the reset to take place

    gpio_put(rstPin, 1);
}

void closeSPI(sh2_Hal_t *self)
{
    spi_deinit(spiPort);
    gpio_put(csPin, 1);
    gpio_put(rstPin, 0);
}

int readSPI(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us)
{

}

int writeSPI(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{

}

uint32_t getTimeUsHAL(sh2_Hal_t *self)
{
    absolute_time_t now = get_absolute_time();
    return to_us_since_boot(now);   //This function actually returns a 64 bit number
}

bool startupIMU(spi_inst_t * spi, uint freq, uint sck, uint tx, uint rx, uint cs, uint rst, uint interrupt)
{
    spiPort = spi;
    txPin = tx;
    rxPin = rx;
    sckPin = sck;
    rstPin = rst;
    intPin = interrupt;
    freq = freq;
    csPin = cs;

    setupHALFunctions();

    int status = sh2_open(sh2HALInstance, eventHandler, NULL);
    if (status != SH2_OK) {
        printf("Error, %d, from sh2_open.\n", status);
        return false;
    }

    sh2_setSensorCallback(sensorHandler, NULL);
    if (status != SH2_OK) {
        printf("Error, %d, from sh2_setSensorCallback.\n", status);
        return false;
    }

    reportProdIds();
    resetOccurred = false;
    startReports();

    return true;
};

void setupHALFunctions()
{
    sh2HALInstance->open = openSPI;
    sh2HALInstance->close = closeSPI;
    sh2HALInstance->read = readSPI;
    sh2HALInstance->write = writeSPI;
    sh2HALInstance->getTimeUs = getTimeUsHAL;
}

void serviceIMU()
{
    if (resetOccurred) {
        resetOccurred = false;
        startReports();
    }
    
    sh2_service();
};