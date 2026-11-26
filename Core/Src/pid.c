#include "pid.h"
#include <math.h>
static float clamp(float v,float lo,float hi) { return v<lo?lo:(v>hi?hi:v); }
void PID_Reset(PID_t *p) { p->integral=0; p->prev_error=0; }
void PID_Init(PID_t *p,float kp,float ki,float kd,float lo,float hi,float ilim) {
    p->Kp=kp; p->Ki=ki; p->Kd=kd; p->out_min=lo; p->out_max=hi;
    p->integral_limit=ilim; p->setpoint=0; PID_Reset(p);
}
void PID_SetTarget(PID_t *p,float target) { p->setpoint=target; }
float PID_ComputeWithDerivative(PID_t *p,float measured,float derivative,float dt) {
    if(!isfinite(measured) || !isfinite(derivative) || !isfinite(dt) || dt<=0) {
        PID_Reset(p); return 0;
    }
    float error=p->setpoint-measured;
    if(fabsf(p->Ki)<=1e-6f) {
        p->integral=0.0f;
        p->prev_error=error;
        return clamp(p->Kp*error+p->Kd*derivative,p->out_min,p->out_max);
    }
    float next=clamp(p->integral+error*dt,-p->integral_limit,p->integral_limit);
    float output=p->Kp*error+p->Ki*next+p->Kd*derivative;
    /* Freeze integration when it would push an already saturated output further. */
    if(!((output>p->out_max && error>0) || (output<p->out_min && error<0))) p->integral=next;
    output=p->Kp*error+p->Ki*p->integral+p->Kd*derivative;
    p->prev_error=error;
    return clamp(output,p->out_min,p->out_max);
}
float PID_Compute(PID_t *p,float measured,float dt) {
    if(!isfinite(dt) || dt<=0) return 0;
    float derivative=(p->setpoint-measured-p->prev_error)/dt;
    return PID_ComputeWithDerivative(p,measured,derivative,dt);
}
