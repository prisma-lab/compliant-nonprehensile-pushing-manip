/*
 * Copyright (c) The acados authors.
 * 
 * This file is part of acados.
 * 
 * The 2-Clause BSD License
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

#define S_FUNCTION_NAME acados_solver_sfunction_pushing_reduced
#define S_FUNCTION_LEVEL 2

#define MDL_START

// acados
// #include "acados/utils/print.h"
#include "acados_c/sim_interface.h"
#include "acados_c/external_function_interface.h"

// example specific
#include "pushing_reduced_model/pushing_reduced_model.h"
#include "acados_solver_pushing_reduced.h"

#include "simstruc.h"

#define SAMPLINGTIME 0.001

static void mdlInitializeSizes (SimStruct *S)
{
    // specify the number of continuous and discrete states
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    int N = PUSHING_REDUCED_N;// specify the number of input ports
    if ( !ssSetNumInputPorts(S, 12) )
        return;

    // specify the number of output ports
    if ( !ssSetNumOutputPorts(S, 6) )
        return;

    // specify dimension information for the input ports
    // lbx_0
    ssSetInputPortVectorDimension(S, 0, 8);
    // ubx_0
    ssSetInputPortVectorDimension(S, 1, 8);
    // parameters
    ssSetInputPortVectorDimension(S, 2, (N+1) * 10);
    // y_ref_0
    ssSetInputPortVectorDimension(S, 3, 13);
    // y_ref
    ssSetInputPortVectorDimension(S, 4, 52);
    // y_ref_e
    ssSetInputPortVectorDimension(S, 5, 8);
    // lbx
    ssSetInputPortVectorDimension(S, 6, 32);
    // ubx
    ssSetInputPortVectorDimension(S, 7, 32);
    // lbu
    ssSetInputPortVectorDimension(S, 8, 25);
    // ubu
    ssSetInputPortVectorDimension(S, 9, 25);
    // lh
    ssSetInputPortVectorDimension(S, 10, 15);
    // uh
    ssSetInputPortVectorDimension(S, 11, 15);/* specify dimension information for the OUTPUT ports */
    ssSetOutputPortVectorDimension(S, 0, 5 );
    ssSetOutputPortVectorDimension(S, 1, 1 );
    ssSetOutputPortVectorDimension(S, 2, 1 );
    ssSetOutputPortVectorDimension(S, 3, 8 ); // state at shooting node 1
    ssSetOutputPortVectorDimension(S, 4, 1);
    ssSetOutputPortVectorDimension(S, 5, 1 );

    // specify the direct feedthrough status
    // should be set to 1 for all inputs used in mdlOutputs
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 2, 1);
    ssSetInputPortDirectFeedThrough(S, 3, 1);
    ssSetInputPortDirectFeedThrough(S, 4, 1);
    ssSetInputPortDirectFeedThrough(S, 5, 1);
    ssSetInputPortDirectFeedThrough(S, 6, 1);
    ssSetInputPortDirectFeedThrough(S, 7, 1);
    ssSetInputPortDirectFeedThrough(S, 8, 1);
    ssSetInputPortDirectFeedThrough(S, 9, 1);
    ssSetInputPortDirectFeedThrough(S, 10, 1);
    ssSetInputPortDirectFeedThrough(S, 11, 1);

    // one sample time
    ssSetNumSampleTimes(S, 1);
}


#if defined(MATLAB_MEX_FILE)

#define MDL_SET_INPUT_PORT_DIMENSION_INFO
#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO

static void mdlSetInputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
{
    if ( !ssSetInputPortDimensionInfo(S, port, dimsInfo) )
         return;
}

static void mdlSetOutputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
{
    if ( !ssSetOutputPortDimensionInfo(S, port, dimsInfo) )
         return;
}

#endif /* MATLAB_MEX_FILE */


static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, SAMPLINGTIME);
    ssSetOffsetTime(S, 0, 0.0);
}


static void mdlStart(SimStruct *S)
{
    pushing_reduced_solver_capsule *capsule = pushing_reduced_acados_create_capsule();
    pushing_reduced_acados_create(capsule);

    ssSetUserData(S, (void*)capsule);
}


static void mdlOutputs(SimStruct *S, int_T tid)
{
    pushing_reduced_solver_capsule *capsule = ssGetUserData(S);
    ocp_nlp_config *nlp_config = pushing_reduced_acados_get_nlp_config(capsule);
    ocp_nlp_dims *nlp_dims = pushing_reduced_acados_get_nlp_dims(capsule);
    ocp_nlp_in *nlp_in = pushing_reduced_acados_get_nlp_in(capsule);
    ocp_nlp_out *nlp_out = pushing_reduced_acados_get_nlp_out(capsule);

    InputRealPtrsType in_sign;

    int N = PUSHING_REDUCED_N;      

    // local buffer
    real_t buffer[13];

    /* go through inputs */
    // lbx_0
    in_sign = ssGetInputPortRealSignalPtrs(S, 0);
    for (int i = 0; i < 8; i++)
        buffer[i] = (double)(*in_sign[i]);

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", buffer);
    // ubx_0
    in_sign = ssGetInputPortRealSignalPtrs(S, 1);
    for (int i = 0; i < 8; i++)
        buffer[i] = (double)(*in_sign[i]);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", buffer);
    // parameters - stage-variant !!!
    in_sign = ssGetInputPortRealSignalPtrs(S, 2);

    // update value of parameters
    for (int ii = 0; ii <= N; ii++)
    {
        for (int jj = 0; jj < 10; jj++)
            buffer[jj] = (double)(*in_sign[ii*10+jj]);
        pushing_reduced_acados_update_params(capsule, ii, buffer, 10);
    }

  
    // y_ref_0
    in_sign = ssGetInputPortRealSignalPtrs(S, 3);

    for (int i = 0; i < 13; i++)
        buffer[i] = (double)(*in_sign[i]);

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "yref", (void *) buffer);

  
    // y_ref - for stages 1 to N-1
    in_sign = ssGetInputPortRealSignalPtrs(S, 4);

    for (int ii = 1; ii < N; ii++)
    {
        for (int jj = 0; jj < 13; jj++)
            buffer[jj] = (double)(*in_sign[(ii-1)*13+jj]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "yref", (void *) buffer);
    }

  
    // y_ref_e
    in_sign = ssGetInputPortRealSignalPtrs(S, 5);

    for (int i = 0; i < 8; i++)
        buffer[i] = (double)(*in_sign[i]);

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", (void *) buffer);
    // lbx
    in_sign = ssGetInputPortRealSignalPtrs(S, 6);
    for (int ii = 1; ii < N; ii++)
    {
        for (int jj = 0; jj < 8; jj++)
            buffer[jj] = (double)(*in_sign[(ii-1)*8+jj]);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "lbx", (void *) buffer);
    }
    // ubx
    in_sign = ssGetInputPortRealSignalPtrs(S, 7);
    for (int ii = 1; ii < N; ii++)
    {
        for (int jj = 0; jj < 8; jj++)
            buffer[jj] = (double)(*in_sign[(ii-1)*8+jj]);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "ubx", (void *) buffer);
    }
    // lbu
    in_sign = ssGetInputPortRealSignalPtrs(S, 8);
    for (int ii = 0; ii < N; ii++)
    {
        for (int jj = 0; jj < 5; jj++)
            buffer[jj] = (double)(*in_sign[ii*5+jj]);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "lbu", (void *) buffer);
    }
    // ubu
    in_sign = ssGetInputPortRealSignalPtrs(S, 9);
    for (int ii = 0; ii < N; ii++)
    {
        for (int jj = 0; jj < 5; jj++)
            buffer[jj] = (double)(*in_sign[ii*5+jj]);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "ubu", (void *) buffer);
    }
    // lh
    in_sign = ssGetInputPortRealSignalPtrs(S, 10);

    for (int ii = 0; ii < N; ii++)
    {
        for (int jj = 0; jj < 3; jj++)
            buffer[jj] = (double)(*in_sign[ii*3+jj]);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "lh", (void *) buffer);
    }
    // uh
    in_sign = ssGetInputPortRealSignalPtrs(S, 11);

    for (int ii = 0; ii < N; ii++)
    {
        for (int jj = 0; jj < 3; jj++)
            buffer[jj] = (double)(*in_sign[ii*3+jj]);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "uh", (void *) buffer);
    }

    /* call solver */
    int rti_phase = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "rti_phase", &rti_phase);
    int acados_status = pushing_reduced_acados_solve(capsule);


    /* set outputs */
    // assign pointers to output signals
    real_t *out_u0, *out_utraj, *out_xtraj, *out_status, *out_sqp_iter, *out_KKT_res, *out_KKT_residuals, *out_x1, *out_cpu_time, *out_cpu_time_sim, *out_cpu_time_qp, *out_cpu_time_lin, *out_cost_value;
    int tmp_int;
    out_u0 = ssGetOutputPortRealSignal(S, 0);
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", (void *) out_u0);

  
    out_status = ssGetOutputPortRealSignal(S, 1);
    *out_status = (real_t) acados_status;
    out_KKT_res = ssGetOutputPortRealSignal(S, 2);
    *out_KKT_res = (real_t) nlp_out->inf_norm_res;
    out_x1 = ssGetOutputPortRealSignal(S, 3);
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 1, "x", (void *) out_x1);
    out_cpu_time = ssGetOutputPortRealSignal(S, 4);
    // get solution time
    ocp_nlp_get(nlp_config, capsule->nlp_solver, "time_tot", (void *) out_cpu_time);
    out_sqp_iter = ssGetOutputPortRealSignal(S, 5);
    // get sqp iter
    ocp_nlp_get(nlp_config, capsule->nlp_solver, "sqp_iter", (void *) &tmp_int);
    *out_sqp_iter = (real_t) tmp_int;

}

static void mdlTerminate(SimStruct *S)
{
    pushing_reduced_solver_capsule *capsule = ssGetUserData(S);

    pushing_reduced_acados_free(capsule);
    pushing_reduced_acados_free_capsule(capsule);
}


#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
