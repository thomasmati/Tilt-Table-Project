/*
 * pid.c
 *
 *  Created on: Mar 2, 2023
 *
 */

#include "pid.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

void ResetPID(PID* pPID)
{
	pPID->startup = 1;
	pPID->errSum = 0;
	pPID->lastInput = 0;
}

PID* InitPID()
{
	PID*		pPID = malloc(sizeof(PID));

	if (pPID)
		memset(pPID, 0, sizeof(PID));

	pPID->startup = 1;

	return pPID;
}

void SetPIDSetpoint(PID* pPID, float setPt)
{
	if (! pPID)
		return;

	pPID->setpoint = setPt;
	pPID->errSum = 0;
}

void SetPIDLimits(PID* pPID, float lwrLmt, float uprLmt)
{
	if (! pPID)
		return;

	pPID->lowerLimit = lwrLmt;
	pPID->upperLimit = uprLmt;
}

void SetPIDGain(PID* pPID, float p, float i, float d)
{
	if (! pPID)
		return;

	pPID->Kp = p;
	pPID->Ki = i;
	pPID->Kd = d;
}

float GetPIDOutput(PID* pPID, float input, float dTime)
{
	float		output = 0;
	float		error;

	if (! pPID)
		return 0;

	if (pPID->startup)
	{
		pPID->startup = 0;
		pPID->lastInput = input;
		return pPID->setpoint;
	}

	error = pPID->setpoint - input;
	pPID->lastError = error;
	pPID->errSum += pPID->Ki*(error * dTime);

	if (pPID->Ki == 0.0)
		pPID->errSum = 0.0;
	else if (pPID->errSum > pPID->upperLimit)
		pPID->errSum = pPID->upperLimit;
	else if (pPID->errSum < pPID->lowerLimit)
		pPID->errSum = pPID->lowerLimit;

	pPID->dError = fabs(dTime) < 0.000001 ? 0 : (pPID->lastInput - input)/dTime;
	pPID->lastdError = pPID->dError;
		pPID->lastInput = input;
	// Compute PID Outpu
	output = pPID->Kp*error + pPID->errSum + pPID->Kd*pPID->dError;

	// check limits
	if (output > pPID->upperLimit)
		output = pPID->upperLimit;
	else if (output < pPID->lowerLimit)
		output = pPID->lowerLimit;

	return output;
}

