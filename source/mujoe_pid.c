/*
 * mujoe_pid.c
 *
 *      Author: Joe Corteo
 *      Source: https://github.com/muJoseph/mujoe_pid
 */

//////////////////////////////////////////////////////////////////////////////////////////////
//  INCLUDE
//////////////////////////////////////////////////////////////////////////////////////////////

#include "mujoe_pid.h"
#include <string.h>

//////////////////////////////////////////////////////////////////////////////////////////////
//  LOCAL FNC
//////////////////////////////////////////////////////////////////////////////////////////////

static inline void cycle_error( MUJOE_PID_CTX ctx, mujoe_pid_err_in_t err );
static inline void cycle_ctrl( MUJOE_PID_CTX ctx, mujoe_pid_ctrl_out_t ctrl );

//////////////////////////////////////////////////////////////////////////////////////////////
//  MACROS
//////////////////////////////////////////////////////////////////////////////////////////////

#define PI_VAL                              3.1416f

// Filter coefficient (N, [N] = rad/s)
#define MUJOE_PID_COMPUTE_N()               (mujoe_term_t)(2.0*PI_VAL*(float)ctx->cfg.lpf_f_Hz)

// P term, E(n-1) coefficient
#define MUJOE_PID_COMPUTE_A1()              (mujoe_coeff_t)(8.0f/(2.0f*ctx->irr_coeff.N*ctx->cfg.T_sec+4.0f))
// P term, E(n-2) coefficient
#define MUJOE_PID_COMPUTE_A2()              (mujoe_coeff_t)( (4.0f-2.0f*ctx->irr_coeff.N*ctx->cfg.T_sec)/(2.0f*ctx->irr_coeff.N*ctx->cfg.T_sec+4.0f) )

// I term, E(n) coefficient
#define MUJOE_PID_COMPUTE_B0()              (mujoe_coeff_t)(ctx->cfg.T_sec/2.0f)
// I term, E(n-1) coefficient
#define MUJOE_PID_COMPUTE_B1()              (mujoe_coeff_t)((ctx->irr_coeff.N*ctx->cfg.T_sec)/(ctx->irr_coeff.N + (2.0f/ctx->cfg.T_sec)))
// I term, E(n-2) coefficient
#define MUJOE_PID_COMPUTE_B2()              (mujoe_coeff_t)((ctx->cfg.T_sec/2.0f)*(ctx->irr_coeff.N-(2.0f/ctx->cfg.T_sec))/(ctx->irr_coeff.N+(2.0f/ctx->cfg.T_sec)))

// D term, E(n) coefficient
#define MUJOE_PID_COMPUTE_C0()              (mujoe_coeff_t)((4.0f*ctx->irr_coeff.N)/((2.0f*ctx->irr_coeff.N*ctx->cfg.T_sec)+4.0f))
// D term, E(n-1) coefficient
#define MUJOE_PID_COMPUTE_C1()              (mujoe_coeff_t)((8.0f*ctx->irr_coeff.N)/((2.0f*ctx->irr_coeff.N*ctx->cfg.T_sec)+4.0f))
// D term, E(n-2) coefficient
#define MUJOE_PID_COMPUTE_C2()              (mujoe_coeff_t)((4.0f*ctx->irr_coeff.N)/((2.0f*ctx->irr_coeff.N*ctx->cfg.T_sec)+4.0f))

// Output term, E(n-1) coefficient
#define MUJOE_PID_COMPUTE_D1()              (mujoe_coeff_t)((8.0f*ctx->irr_coeff.N)/((2.0f*ctx->irr_coeff.N*ctx->cfg.T_sec)+4.0f))
// Output term, E(n-2) coefficient
#define MUJOE_PID_COMPUTE_D2()              (mujoe_coeff_t)(((2.0f*ctx->irr_coeff.N*ctx->cfg.T_sec)-4.0f)/((2.0f*ctx->irr_coeff.N*ctx->cfg.T_sec)+4.0f))


//////////////////////////////////////////////////////////////////////////////////////////////
//  FUNCTIONS
//////////////////////////////////////////////////////////////////////////////////////////////

mujoe_pid_err_t mujoe_pid_initCtx( MUJOE_PID_CTX ctx, mujoe_pid_cfg_t *pCfg )
{
    memset(ctx,0,sizeof(mujoe_pid_t));
    ctx->cfg = *pCfg;

    if( (ctx->cfg.outputSaturation && (ctx->cfg.Umax <= ctx->cfg.Umin)) ||
        (ctx->cfg.antiWindup && (ctx->cfg.Imax <= ctx->cfg.Imin)) )
        return MUJOE_PID_ERR_INV_PARAM;

    // Generate IIR coefficients
    ctx->irr_coeff.N    = MUJOE_PID_COMPUTE_N();
    ctx->irr_coeff.a[0] = MUJOE_PID_COMPUTE_A1();
    ctx->irr_coeff.a[1] = MUJOE_PID_COMPUTE_A2();

    ctx->irr_coeff.b[0] = MUJOE_PID_COMPUTE_B0();
    ctx->irr_coeff.b[1] = MUJOE_PID_COMPUTE_B1();
    ctx->irr_coeff.b[2] = MUJOE_PID_COMPUTE_B2();

    ctx->irr_coeff.c[0] = MUJOE_PID_COMPUTE_C0();
    ctx->irr_coeff.c[1] = MUJOE_PID_COMPUTE_C1();
    ctx->irr_coeff.c[2] = MUJOE_PID_COMPUTE_C2();

    ctx->irr_coeff.d[0] = MUJOE_PID_COMPUTE_D1();
    ctx->irr_coeff.d[1] = MUJOE_PID_COMPUTE_D2();

    return MUJOE_PID_ERR_NONE;
}// mujoe_pid_initCtx

mujoe_pid_ctrl_out_t mujoe_pid_run( MUJOE_PID_CTX ctx, mujoe_pid_err_in_t err )
{
    mujoe_pid_ctrl_out_t out = 0;

    cycle_error( ctx, err );

    // Compute proportional term
    if( ctx->Kp != 0 )
    {
        ctx->rt.P = ctx->Kp * ( ctx->rt.E[0]                            // E(n)
                              - ctx->irr_coeff.a[0]*ctx->rt.E[1]        // E(n-1)
                              + ctx->irr_coeff.a[1]*ctx->rt.E[2] );     // E(n-2)
    }

    // Compute integral term
    if( ctx->Ki != 0 )
    {
        ctx->rt.I = ctx->Ki * ( ctx->irr_coeff.b[0]*ctx->rt.E[0]        // E(n)
                              + ctx->irr_coeff.b[1]*ctx->rt.E[1]        // E(n-1)
                              + ctx->irr_coeff.b[2]*ctx->rt.E[2] );     // E(n-2)

        // Clamp integral term magnitude if anti-windup is enabled
        if( ctx->cfg.antiWindup )
        {
            if( ctx->cfg.Imax < ctx->rt.I ){ ctx->rt.I = ctx->cfg.Imax;}
            else if( ctx->rt.I < ctx->cfg.Imin ){ ctx->rt.I = ctx->cfg.Imin;}
        }
    }

    // Compute derivative term
    if( ctx->Kd != 0 )
    {
        ctx->rt.D = ctx->Kd * ( ctx->irr_coeff.c[0]*ctx->rt.E[0]        // E(n)
                              - ctx->irr_coeff.c[1]*ctx->rt.E[1]        // E(n-1)
                              + ctx->irr_coeff.c[2]*ctx->rt.E[2] );     // E(n-2)
    }


    // Compute output, U(n)
    out = ctx->rt.P + ctx->rt.I + ctx->rt.D
                    + ctx->irr_coeff.d[0]*ctx->rt.U[0]   // U(n-1)
                    + ctx->irr_coeff.d[1]*ctx->rt.U[1];  // U(n-2), output 2T seconds ago

    // Clamp output magnitude if output saturation is enabled
    if( ctx->cfg.outputSaturation )
    {
        if( ctx->cfg.Umax < out ){ out = ctx->cfg.Umax;}
        else if( out < ctx->cfg.Umin ){ out = ctx->cfg.Umin;}
    }

    cycle_ctrl( ctx, out );

    return out;

} // mujoe_pid_run

void mujoe_pid_updateGains( MUJOE_PID_CTX ctx, mujoe_coeff_t kp, mujoe_coeff_t ki, mujoe_coeff_t kd)
{
    ctx->Kp = kp;
    ctx->Ki = ki;
    ctx->Kd = kd;
    mujoe_pid_reset(ctx);

}// mujoe_pid_updateGains

void mujoe_pid_reset( MUJOE_PID_CTX ctx )
{
    memset(&ctx->rt,0,sizeof(mujoe_pid_runtime_t));

}// mujoe_pid_reset
//////////////////////////////////////////////////////////////////////////////////////////////
//  STATIC FUNCTIONS
//////////////////////////////////////////////////////////////////////////////////////////////

static inline void cycle_error( MUJOE_PID_CTX ctx, mujoe_pid_err_in_t err )
{
    ctx->rt.E[2] = ctx->rt.E[1];
    ctx->rt.E[1] = ctx->rt.E[0];
    ctx->rt.E[0] = err;

} // cycle_error

static inline void cycle_ctrl( MUJOE_PID_CTX ctx, mujoe_pid_ctrl_out_t ctrl )
{
    ctx->rt.U[1] = ctx->rt.U[0];
    ctx->rt.U[0] = ctrl;

} // cycle_ctrl
