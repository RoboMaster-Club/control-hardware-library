/**
 ******************************************************************************
 * @file    remote.h
 * @brief   This is the header file for Remote.h
 ******************************************************************************
 */
#ifndef __REMOTE_H
#define __REMOTE_H

#include <stdint.h>

#define REMOTE_STICK_MAX (660.0f)

enum SwitchPos {UP=1, DOWN, MID};

struct Joystick
{
    int16_t x;
    int16_t y;
};

struct Controller
{
    struct Joystick left_stick;
    struct Joystick right_stick;
    int32_t wheel;
    uint8_t left_switch; // 1 up; 2 down; 3 mid
    uint8_t right_switch;
};

struct Key
{
    uint8_t W;
    uint8_t S;
    uint8_t A;
    uint8_t D;
    uint8_t Q;
    uint8_t E;
    uint8_t Shift;
    uint8_t Ctrl;
};

struct Mouse
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
    uint8_t left;
    uint8_t right;
};

typedef struct
{
    struct Controller controller;
    struct Key key;
    struct Mouse mouse;
} Remote_t;


extern Remote_t g_remote;
extern void Remote_Init(void);
extern void Remote_BufferProcess(void);

#endif /* __REMOTE_H */
