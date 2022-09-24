#ifndef SHTP_H_
#define SHTP_H_

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "can.h"
#include "mcp2515.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "sh2.h"
#include "sh2_util.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"

bool startupIMU(spi_inst_t * spi, uint freq, uint sck, uint tx, uint rx, int cs, uint rst, uint interrupt);
void serviceIMU();

#endif