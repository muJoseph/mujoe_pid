/*
 * mujoe_pid.h
 *
 *      Author: Joe Corteo
 *      Source: https://github.com/muJoseph/mujoe_pid
 */

#ifndef MUJOE_PID_H_
#define MUJOE_PID_H_

//////////////////////////////////////////////////////////////////////////////////////////////
//  INCLUDE
//////////////////////////////////////////////////////////////////////////////////////////////

#include "mujoe_types.h"

//////////////////////////////////////////////////////////////////////////////////////////////
//  DEFINES
//////////////////////////////////////////////////////////////////////////////////////////////

typedef unsigned char mujoe_pid_err_t;
#define MUJOE_PID_ERR_NONE                  (mujoe_pid_err_t)0x00
#define MUJOE_PID_ERR_INV_PARAM             (mujoe_pid_err_t)0x01
#define MUJOE_PID_ERR_FAILURE               (mujoe_pid_err_t)0xFF

//////////////////////////////////////////////////////////////////////////////////////////////
//  TYPES
//////////////////////////////////////////////////////////////////////////////////////////////

// TODO: Host can change the data type of all coefficients and stored terms.
//       Of course, accuracy will be negatively impacted
typedef float mujoe_pid_err_in_t;
typedef float mujoe_pid_ctrl_out_t;
typedef float mujoe_coeff_t;
typedef float mujoe_term_t;


// Configuration presets
typedef enum
{
    MUJOE_PID_CFG_PRESET_ID_DEMO = 0,

}mujoe_pid_cfg_id_t;

typedef struct _mujoe_pid_cfg
{
    bool                antiWindup;         // If true, anti-windup is enabled. Integral term clamped to: Imin <= I <= Imax, where Imax > Imin
    mujoe_term_t        Imin;
    mujoe_term_t        Imax;
    bool                outputSaturation;   // If true, output clamped to : Umin <= U <= Umax, where Umax > Umin
    mujoe_term_t        Umax;
    mujoe_term_t        Umin;
    mujoe_term_t        T_sec;              // Sample period (seconds)
    mujoe_term_t        lpf_f_Hz;           // Derivative low pass filter pole location (Hz)
    mujoe_coeff_t       Kp;                 // Proportional gain
    mujoe_coeff_t       Ki;                 // Integral gain
    mujoe_coeff_t       Kd;                 // Derivative gain
}mujoe_pid_cfg_t;

#define MUJOE_PID_NUM_E_TERM        3
#define MUJOE_PID_NUM_U_TERM        2

typedef struct _mujoe_iir_coeff
{
    mujoe_coeff_t       a[MUJOE_PID_NUM_E_TERM-1];
    mujoe_coeff_t       b[MUJOE_PID_NUM_E_TERM];
    mujoe_coeff_t       c[MUJOE_PID_NUM_E_TERM];
    mujoe_coeff_t       d[MUJOE_PID_NUM_U_TERM];
    mujoe_term_t        N;

}mujoe_iir_coeff_t;

typedef struct _mujoe_pid_runtime
{
    mujoe_term_t              P;
    mujoe_term_t              I;
    mujoe_term_t              D;
    mujoe_pid_err_in_t        E[MUJOE_PID_NUM_E_TERM];
    mujoe_pid_ctrl_out_t      U[MUJOE_PID_NUM_U_TERM];
}mujoe_pid_runtime_t;

typedef struct _mujoe_pid
{
    mujoe_pid_cfg_t         cfg;            // Context configuration structure
    mujoe_pid_runtime_t     rt;             // Run-time variables
    mujoe_iir_coeff_t       irr_coeff;      // IIR Coefficients

}mujoe_pid_t, *MUJOE_PID_CTX;

//////////////////////////////////////////////////////////////////////////////////////////////
//  API
//////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief       Initializes PID context configuration parameters.
 *
 * @param       presetId: Configuration preset ID
 * @param       pCfg: Pointer to PID context configuration structure
 *
 */
extern void mujoe_pid_initConfigParams( mujoe_pid_cfg_id_t presetId, mujoe_pid_cfg_t *pCfg );

/**
 * @brief       Initializes PID context.
 *
 * @note        Must be called first!
 *
 * @param       ctx: Pointer to PID context data structure
 * @param       pCfg: Pointer to PID context configuration structure
 *
 * @return  MUJOE_PID_ERR_NONE
 * @return  MUJOE_PID_ERR_INV_PARAM
 */
extern mujoe_pid_err_t mujoe_pid_initCtx( MUJOE_PID_CTX ctx, mujoe_pid_cfg_t *pCfg );

/**
 * @brief       Run PID controller.
 *
 * @note        Call every T (seconds)
 *
 * @param       ctx: Pointer to PID context data structure
 * @param       err: Error signal
 *
 * @return  Control signal.
 */
extern mujoe_pid_ctrl_out_t mujoe_pid_run( MUJOE_PID_CTX ctx, mujoe_pid_err_in_t err );

/**
 * @brief       Updates the PID gains.
 *
 * @note        PID run-time variables are reset/cleared
 *
 * @param       ctx: Pointer to PID context data structure
 * @param       kp: Proportional gain
 * @param       ki: Integral gain
 * @param       kd: Derivative gain
 *
 */
extern void mujoe_pid_updateGains( MUJOE_PID_CTX ctx, mujoe_coeff_t kp, mujoe_coeff_t ki, mujoe_coeff_t kd);

/**
 * @brief       Enables anti-windup.
 *
 * @note        PID run-time variables are reset/cleared
 *
 * @param       ctx: Pointer to PID context data structure
 * @param       min: Minimum value that integral term will be clamped to (inclusive)
 * @param       max: Maximum value that integral term will be clamped to (inclusive)
 *
 * @return  MUJOE_PID_ERR_NONE
 * @return  MUJOE_PID_ERR_INV_PARAM
 *
 */
extern mujoe_pid_err_t mujoe_pid_enableAntWindup( MUJOE_PID_CTX ctx, float min, float max );

/**
 * @brief       Disables anti-windup.
 *
 * @note        PID run-time variables are reset/cleared
 *
 * @param       ctx: Pointer to PID context data structure
 *
 */
extern void mujoe_pid_disableAntiWindup( MUJOE_PID_CTX ctx );

/**
 * @brief       Resets/clears run-time variables.
 *
 * @param       ctx: Pointer to PID context data structure
 *
 */
extern void mujoe_pid_reset( MUJOE_PID_CTX ctx );

#endif /* MUJOE_PID_H_ */
