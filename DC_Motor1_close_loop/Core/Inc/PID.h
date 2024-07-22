#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct {
	/* Derivative low-pass filter time constant */
	float tau;
	/* Output limits */
	float limMin;
	float limMax;
	/* Integrator limits */
	float limMinInt;
	float limMaxInt;
	/* Sample time (in seconds) */
	float T;
	/* Controller "memory" */
	float integrator[6];
	float prevError[6];			/* Required for integrator */
	float differentiator[6];
	float prevMeasurement[6];		/* Required for differentiator */

	/* Controller output */
	float out[6];

} PIDController;

void  PID_Init(PIDController *pid, int nMotor);
float PID(PIDController *pid, float setpoint, float measurement,float Kp,float Ki,float Kd, int i);

#endif
