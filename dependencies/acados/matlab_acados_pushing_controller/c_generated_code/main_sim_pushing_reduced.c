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


// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/sim_interface.h"
#include "acados_sim_solver_pushing_reduced.h"

#define NX     PUSHING_REDUCED_NX
#define NZ     PUSHING_REDUCED_NZ
#define NU     PUSHING_REDUCED_NU
#define NP     PUSHING_REDUCED_NP


int main()
{
    int status = 0;
    sim_solver_capsule *capsule = pushing_reduced_acados_sim_solver_create_capsule();
    status = pushing_reduced_acados_sim_create(capsule);

    if (status)
    {
        printf("acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    sim_config *acados_sim_config = pushing_reduced_acados_get_sim_config(capsule);
    sim_in *acados_sim_in = pushing_reduced_acados_get_sim_in(capsule);
    sim_out *acados_sim_out = pushing_reduced_acados_get_sim_out(capsule);
    void *acados_sim_dims = pushing_reduced_acados_get_sim_dims(capsule);

    // initial condition
    double x_current[NX];
    x_current[0] = 0.0;
    x_current[1] = 0.0;
    x_current[2] = 0.0;
    x_current[3] = 0.0;
    x_current[4] = 0.0;
    x_current[5] = 0.0;
    x_current[6] = 0.0;
    x_current[7] = 0.0;

  
    x_current[0] = 0;
    x_current[1] = 0.6;
    x_current[2] = 0.7853981634;
    x_current[3] = 3.141592654;
    x_current[4] = -0.05;
    x_current[5] = 0;
    x_current[6] = 0;
    x_current[7] = 0;
    
  


    // initial value for control input
    double u0[NU];
    u0[0] = 0.0;
    u0[1] = 0.0;
    u0[2] = 0.0;
    u0[3] = 0.0;
    u0[4] = 0.0;
    // set parameters
    double p[NP];
    p[0] = 1.962;
    p[1] = 1.962;
    p[2] = 0.0751;
    p[3] = 0.2;
    p[4] = 5;
    p[5] = 5;
    p[6] = 5;
    p[7] = 5;
    p[8] = 0.1;
    p[9] = 0.01;

    pushing_reduced_acados_sim_update_params(capsule, p, NP);
  

    int n_sim_steps = 3;
    // solve ocp in loop
    for (int ii = 0; ii < n_sim_steps; ii++)
    {
        sim_in_set(acados_sim_config, acados_sim_dims,
            acados_sim_in, "x", x_current);
        status = pushing_reduced_acados_sim_solve(capsule);

        if (status != ACADOS_SUCCESS)
        {
            printf("acados_solve() failed with status %d.\n", status);
        }

        sim_out_get(acados_sim_config, acados_sim_dims,
               acados_sim_out, "x", x_current);
        
        printf("\nx_current, %d\n", ii);
        for (int jj = 0; jj < NX; jj++)
        {
            printf("%e\n", x_current[jj]);
        }
    }

    printf("\nPerformed %d simulation steps with acados integrator successfully.\n\n", n_sim_steps);

    // free solver
    status = pushing_reduced_acados_sim_free(capsule);
    if (status) {
        printf("pushing_reduced_acados_sim_free() returned status %d. \n", status);
    }

    pushing_reduced_acados_sim_solver_free_capsule(capsule);

    return status;
}
