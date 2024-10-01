/**
  ******************************************************************************
  * @file		 pid.h
  * @author  Ginger
  * @version V1.0.0
  * @date    2015/11/14
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#ifndef _PID_H
#define _PID_H

#include "stdint.h"
typedef enum
{

	PID_Position,
	PID_Speed
	
}PID_ID;

typedef struct _PID_TypeDef
{
	PID_ID id;
	
	float target;							//Ä¿±êÖµ
	float lastNoneZeroTarget;
	float kp;
	float ki;
	float kd;
	
	float   measure;					//²âÁ¿Öµ
	float   err;							//Îó²î
	float   last_err;      		//ÉÏ´ÎÎó²î
	
	float pout;
	float iout;
	float dout;
	
	float output;						//±¾´ÎÊä³ö
	float last_output;			//ÉÏ´ÎÊä³ö
	
	float MaxOutput;				//Êä³öÏÞ·ù
	float IntegralLimit;		//»ý·ÖÏÞ·ù
	float DeadBand;			  //ËÀÇø£¨¾ø¶ÔÖµ£©
	float ControlPeriod;		//¿ØÖÆÖÜÆÚ
	float  Max_Err;					//×î´óÎó²î
	
					  uint32_t thistime;
					uint32_t lasttime;
						uint8_t dtime;	
	
	void (*f_param_init)(struct _PID_TypeDef *pid,  //PID²ÎÊý³õÊ¼»¯
				   PID_ID id,
				   uint16_t maxOutput,
				   uint16_t integralLimit,
				   float deadband,
				   uint16_t controlPeriod,
					int16_t max_err,     
					int16_t  target,
				   float kp,
				   float ki,
				   float kd);
				   
	void (*f_pid_reset)(struct _PID_TypeDef *pid, float kp,float ki, float kd);		//pidÈý¸ö²ÎÊýÐÞ¸Ä
	float (*f_cal_pid)(struct _PID_TypeDef *pid, float measure);   //pid¼ÆËã
}PID_TypeDef;

void pid_init(PID_TypeDef* pid);
#endif

//extern PID_TypeDef pid_pitch;    
extern PID_TypeDef motor_pid[4];
