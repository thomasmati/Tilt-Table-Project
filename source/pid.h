/*
 * pid.h
 *
 *  Created on: Mar 2, 2023
 *      Author: Bob A
 */

#ifndef PID_H_
#define PID_H_

struct PIDStruct
{
	float	errSum;
	float	dError;
	float	lastInput;
	float	lastdError;
	float	lastError;
	float	Kp, Ki, Kd;
	float	upperLimit, lowerLimit;
	float	setpoint;
	int		startup;
};


// float version
typedef struct PIDStruct	PID;
PID* InitPID();
void ResetPID(PID* pPID);
float GetPIDOutput(PID* pPID, float input, float dTime);
void SetPIDGain(PID* pPID, float p, float i, float d);
void SetPIDLimits(PID* pPID, float lwrLmt, float uprLmt);
void SetPIDSetpoint(PID* pPID, float setPt);


#endif /* PID_H_ */
