/**
  ******************************************************************************
  * @file    remote.c
  * @brief   This file contains the driver functions for DR16 Remote From DJI
  ******************************************************************************
  */
#include "remote.h"
#include "usart.h"

Remote_t g_remote;
uint8_t remote_buffer[18];

/*
 * Remote_BufferProcess()
 * 
 * Decode the buffer received from DR16 receiver to g_remote. @ref Remote_t
 */
void Remote_BufferProcess()
{
	// controller decode
	g_remote.controller.right_stick.x = ((remote_buffer[0] | (remote_buffer[1] << 8)) & 0x07ff) - 1024;
	g_remote.controller.right_stick.y = (((remote_buffer[1] >> 3) | (remote_buffer[2] << 5)) & 0x07ff) - 1024;
	g_remote.controller.left_stick.x = (((remote_buffer[2] >> 6) | (remote_buffer[3] << 2) | (remote_buffer[4] << 10)) & 0x07ff) - 1024;
	g_remote.controller.left_stick.y = (((remote_buffer[4] >> 1) | (remote_buffer[5] << 7)) & 0x07ff) - 1024;
	g_remote.controller.wheel = ((remote_buffer[16] | (remote_buffer[17] << 8)) & 0x07FF) - 1024;
	g_remote.controller.left_switch = ((remote_buffer[5] >> 4) & 0x000C) >> 2;
	g_remote.controller.right_switch = ((remote_buffer[5] >> 4) & 0x0003);

	// mouse decode
	g_remote.mouse.x = remote_buffer[6] | (remote_buffer[7] << 8);
	g_remote.mouse.y = remote_buffer[8] | (remote_buffer[9] << 8);
	g_remote.mouse.z = remote_buffer[10] | (remote_buffer[11] << 8);
	g_remote.mouse.left = remote_buffer[12];
	g_remote.mouse.right = remote_buffer[13];

	// key decode
	uint16_t key_buffer = remote_buffer[14] | (remote_buffer[15] << 8);
	g_remote.key.W = (key_buffer >> 0) & 0x001;
	g_remote.key.S = (key_buffer >> 1) & 0x001;
	g_remote.key.A = (key_buffer >> 2) & 0x001;
	g_remote.key.D = (key_buffer >> 3) & 0x001;
	g_remote.key.Q = (key_buffer >> 4) & 0x001;
	g_remote.key.E = (key_buffer >> 5) & 0x001;
	g_remote.key.Shift = (key_buffer >> 6) & 0x001;
	g_remote.key.Ctrl = (key_buffer >> 7) & 0x001;

	// Miss Lock Protection
	if (g_remote.controller.left_stick.y == -660) 
	{
		g_remote.controller.left_stick.y = 0;
	}
}

void Remote_Init(void)
{
	HAL_UART_Receive_DMA(&huart3, remote_buffer, 18);
}
