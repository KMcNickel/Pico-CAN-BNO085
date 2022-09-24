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

#define VER_MAJOR       1
#define VER_MINOR       0
#define VER_REVISION    0
#define VER_BUILD       0

#define CAN_BAUD_RATE 500000    //500 kbaud
#define CAN_NODE_ID 0x6
//NOTE: There is a max of 32 messages per Node ID
#define CAN_RESET_ADDRESS       (CAN_NODE_ID << 5) | 0x0
#define CAN_VER_ADDRESS         (CAN_NODE_ID << 5) | 0x1

#define CAN_RECEIVE_MASK    0xE0
#define CAN_RECEIVE_FILTER  (CAN_NODE_ID << 5)

//BNO085 IMU Interface
#define BNO085_SCK_PIN 10
#define BNO085_TX_PIN  11
#define BNO085_RX_PIN  8
#define BNO085_CS_PIN  9
#define BNO085_INT_PIN 13
#define BNO085_RST_PIN 12
//CAN Interface
#define CAN_SCK_PIN 2
#define CAN_TX_PIN 3
#define CAN_RX_PIN 4
#define CAN_CS_PIN 5
//LEDs
#define GREEN_LED_PIN 25
#define RED_LED_PIN 25
#define LED_PWM_SLICE 0 //(PinNum >> 1) & 7 ... Can also use: uint pwm_gpio_to_slice_num(pinNum)
#define LED_PWM_COUNTER_MIN 0
#define LED_PWM_RED_ON_LVL  80
#define LED_PWM_GRN_ON_LVL  40
#define LED_PWM_COUNTER_MAX 100
#define GREEN_LED_PWM_CHAN PWM_CHAN_A
#define RED_LED_PWM_CHAN   PWM_CHAN_B

//Negative time means the timer will restart at the beginning of the callback (as opposed to the end)
#define CAN_SEND_TIMER_INTERVAL     -500
#define CAN_RECEIVE_TIMER_INTERVAL  -50

#define WATCHDOG_REBOOT_DELAY       50
#define BOOT_WAIT_TIME              1000

#define STDIO_UART_PERIPHERAL uart0
#define CANBUS_SPI_PERIPHERAL spi0
#define BNO085_SPI_PERIPHERAL spi1

can_frame frame;
uint8_t frameID = 0;
struct repeating_timer canSendTimer;
struct repeating_timer canReceiveTimer;

MCP2515 mcp2515(CANBUS_SPI_PERIPHERAL, CAN_CS_PIN, CAN_TX_PIN, CAN_RX_PIN, CAN_SCK_PIN, CAN_BAUD_RATE);

void stopAllTimers()
{
    cancel_repeating_timer(&canSendTimer);
    cancel_repeating_timer(&canReceiveTimer);
}

void rebootDevice()
{
    printf("A reboot has been triggered");
    stopAllTimers();
    watchdog_reboot(0, 0, WATCHDOG_REBOOT_DELAY);
    while(1);    
}

bool canSendTimerCallback(struct repeating_timer *t)
{
    can_frame frame;

    //Set up the CAN frame
    frame.can_dlc = 5;  //Values are 32 bit float plus frame ID
    //Add the frame ID so the other side can match all of the messages together
    frame.data[0] = frameID;

    /*frame.can_id = CAN_BAT_V_ADDRESS;
    memcpy(frame.data + 1, &batteryVoltage, sizeof(float));
    mcp2515.sendMessage(&frame);*/

    frameID++;

    return true;
}

bool canReceiveTimerCallback(struct repeating_timer *t)
{
    MCP2515::ERROR error;
    can_frame frame;

    error = mcp2515.readMessage(&frame);
        if(error == MCP2515::ERROR_OK)
            if(frame.can_id == CAN_RESET_ADDRESS)
                rebootDevice();

    return true;
}

void setupTimer()
{
    add_repeating_timer_ms(CAN_SEND_TIMER_INTERVAL, canSendTimerCallback, NULL, &canSendTimer);
    add_repeating_timer_ms(CAN_RECEIVE_TIMER_INTERVAL, canReceiveTimerCallback, NULL, &canReceiveTimer);
}

void startupStdio()
{
    stdio_init_all();

    printf("\r\n\r\nWait...\r\n");
    sleep_ms(BOOT_WAIT_TIME);

    printf("\r\n\r\nVersion: %d.%d.%d build %d\r\n", VER_MAJOR, VER_MINOR, VER_REVISION, VER_BUILD);
}

void startupCANBus()
{
    printf("Setting up MCP2515...\n");

    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ);
    mcp2515.setFilterMask(MCP2515::MASK0, false, CAN_RECEIVE_MASK);
    mcp2515.setFilter(MCP2515::RXF0, false, CAN_RECEIVE_FILTER);
    mcp2515.setNormalMode();

    can_frame frame;

    frame.can_id = CAN_VER_ADDRESS;
    frame.can_dlc = 4;
    frame.data[3] = VER_MAJOR;
    frame.data[2] = VER_MINOR;
    frame.data[1] = VER_REVISION;
    frame.data[0] = VER_BUILD;
    MCP2515::ERROR err = mcp2515.sendMessage(&frame);
    if(err != MCP2515::ERROR_OK) printf("MCP Error: %d\r\n", err);

    printf("MCP2515 setup complete\n");
}

void setRedLED(bool on)
{
    pwm_set_chan_level(LED_PWM_SLICE, RED_LED_PWM_CHAN, on ? LED_PWM_RED_ON_LVL : LED_PWM_COUNTER_MIN);
}

void setGreenLED(bool on)
{
    pwm_set_chan_level(LED_PWM_SLICE, GREEN_LED_PWM_CHAN, on ? LED_PWM_GRN_ON_LVL : LED_PWM_COUNTER_MIN);
}

void setupGPIO()
{
    gpio_set_function(GREEN_LED_PIN, GPIO_FUNC_PWM);
    gpio_set_function(RED_LED_PIN, GPIO_FUNC_PWM);
    pwm_set_wrap(LED_PWM_SLICE, LED_PWM_COUNTER_MAX);
    setRedLED(false);
    setGreenLED(false);
    pwm_set_enabled(LED_PWM_SLICE, true);

}

void peripheralStartup()
{
    setupGPIO();
    setRedLED(true);

    startupStdio();

    printf("Setting up Pins and Peripherals...\n");

    startupCANBus();
    startupIMU(BNO085_SPI_PERIPHERAL, BNO085_CS_PIN);
    //setupTimer();

    printf("Pins and Peripherals setup complete\n");

    setRedLED(false);
    setGreenLED(true);
}

int main ()
{
    peripheralStartup();

    while(true)
    { /*Don't do anything here, use timers instead*/ 
        //For now, we are ignoring ^ that line and disabling timers
        serviceIMU();
    }
}