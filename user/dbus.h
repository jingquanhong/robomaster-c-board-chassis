/**
  ******************************************************************************
  * @file    dbus.h
  * @brief   This file contains all the function prototypes for
  *          the dbus.c file
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DBUS_H__
#define __DBUS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define DBUS_BUFFER_SIZE 18
//#define DBUS_IWDG

void dbus_init(UART_HandleTypeDef* huart_ptr, DMA_HandleTypeDef* hdma_usart_rx_ptr);
void dbus_decode(void);
void dbus_callback(void);

typedef struct
{
	struct
	{
		uint16_t CH[4];
		uint8_t s1;
		uint8_t s2;
		uint16_t rolling_wheel;
	}channel;
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t left_key;
		uint8_t right_key;
	}mouse;
	struct
	{
		uint8_t w;
		uint8_t s;
		uint8_t a;
		uint8_t d;
		uint8_t q;
		uint8_t e;
		uint8_t shift;
		uint8_t ctrl;
	}keyboard;
}dbus_struct;

extern dbus_struct dbus_ctrl_data;


struct rc_info
{
  /* rocker channel information */
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;
  /* left and right lever information */
  uint8_t sw1;
  uint8_t sw2;
  /* mouse movement and button information */
  struct
  {
    int16_t x;
    int16_t y;
    int16_t z;

    uint8_t l;
    uint8_t r;
  } mouse;
  /* keyboard key information */
  union {
    uint16_t key_code;
    struct
    {
      uint16_t W : 1;
      uint16_t S : 1;
      uint16_t A : 1;
      uint16_t D : 1;
      uint16_t SHIFT : 1;
      uint16_t CTRL : 1;
      uint16_t Q : 1;
      uint16_t E : 1;
      uint16_t R : 1;
      uint16_t F : 1;
      uint16_t G : 1;
      uint16_t Z : 1;
      uint16_t X : 1;
      uint16_t C : 1;
      uint16_t V : 1;
      uint16_t B : 1;
    } bit;
  } kb;
  int16_t wheel;
};

extern struct rc_info rc;
#ifdef __cplusplus
}
#endif
#endif /*__ DBUS_H__ */