/**
  ******************************************************************************
  * @file    pid.c
  * @author  Ginger
  * @version V1.0.0
  * @date    2015/11/14
  * @brief   ¶ÔÃ¿Ò»¸öpid½á¹¹Ìå¶¼ÒªÏÈ½øÐÐº¯ÊýµÄÁ¬½Ó£¬ÔÙ½øÐÐ³õÊ¼»¯
  ******************************************************************************
  * @attention Ó¦¸ÃÊÇÓÃ¶þ½×²î·Ö(d)ÔÆÌ¨»á¸ü¼ÓÎÈ¶¨
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "PID.h"
#include "stm32f4xx.h"

#define ABS(x)		((x>0)? x: -x) 

PID_TypeDef pid_pitch,pid_pithch_speed,pid_roll,pid_roll_speed,pid_yaw_speed;
extern int isMove;

/*²ÎÊý³õÊ¼»¯--------------------------------------------------------------*/
static void pid_param_init(
	PID_TypeDef * pid, 
	PID_ID   id,
	uint16_t maxout,
	uint16_t intergral_limit,
	float deadband,
	uint16_t period,
	int16_t  max_err,
	int16_t  target,

	float 	kp, 
	float 	ki, 
	float 	kd)
{
	pid->id = id;		
	
	pid->ControlPeriod = period;             //Ã»ÓÃµ½
	pid->DeadBand = deadband;
	pid->IntegralLimit = intergral_limit;
	pid->MaxOutput = maxout;
	pid->Max_Err = max_err;
	pid->target = target;
	
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	
	pid->output = 0;
}

/*ÖÐÍ¾¸ü¸Ä²ÎÊýÉè¶¨--------------------------------------------------------------*/
static void pid_reset(PID_TypeDef * pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}

/*pid¼ÆËã-----------------------------------------------------------------------*/

	
static float pid_calculate(PID_TypeDef* pid, float measure)//, int16_t target)
{
//	uint32_t time,lasttime;
	
	pid->lasttime = pid->thistime;
	pid->thistime = HAL_GetTick();
	pid->dtime = pid->thistime-pid->lasttime;
	pid->measure = measure;
  //	pid->target = target;
		
	pid->last_err  = pid->err;
	pid->last_output = pid->output;
	
	pid->err = pid->target - pid->measure;
	
	//ÊÇ·ñ½øÈëËÀÇø
	if((ABS(pid->err) > pid->DeadBand))
	{
		pid->pout = pid->kp * pid->err;
		pid->iout += (pid->ki * pid->err);
		

		pid->dout =  pid->kd * (pid->err - pid->last_err); 
		
		//»ý·ÖÊÇ·ñ³¬³öÏÞÖÆ
		if(pid->iout > pid->IntegralLimit)
			pid->iout = pid->IntegralLimit;
		if(pid->iout < - pid->IntegralLimit)
			pid->iout = - pid->IntegralLimit;
		
		//pidÊä³öºÍ
		pid->output = pid->pout + pid->iout + pid->dout;
		

		//pid->output = pid->output*0.7f + pid->last_output*0.3f;  //ÂË²¨
		if(pid->output>pid->MaxOutput)         
		{
			pid->output = pid->MaxOutput;
		}
		if(pid->output < -(pid->MaxOutput))
		{
			pid->output = -(pid->MaxOutput);
		}
	
	}


	return pid->output;
}

/*pid½á¹¹Ìå³õÊ¼»¯£¬Ã¿Ò»¸öpid²ÎÊýÐèÒªµ÷ÓÃÒ»´Î-----------------------------------------------------*/
void pid_init(PID_TypeDef* pid)
{
	pid->f_param_init = pid_param_init;
	pid->f_pid_reset = pid_reset;
	pid->f_cal_pid = pid_calculate;
}
