%
% Copyright (c) The acados authors.
%
% This file is part of acados.
%
% The 2-Clause BSD License
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%
% 1. Redistributions of source code must retain the above copyright notice,
% this list of conditions and the following disclaimer.
%
% 2. Redistributions in binary form must reproduce the above copyright notice,
% this list of conditions and the following disclaimer in the documentation
% and/or other materials provided with the distribution.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.;

%

%% test of native matlab interface
clear VARIABLES

% check that env.sh has been run
env_run = getenv('ENV_RUN');
if (~strcmp(env_run, 'true'))
	error('env.sh has not been sourced! Before executing this example, run: source env.sh');
end

%% arguments
compile_interface = 'auto';
codgen_model = 'true';
gnsf_detect_struct = 'true';
%method = 'erk';
% method = 'irk';
method = 'irk_gnsf';
sens_forw = 'true';
jac_reuse = 'true';
num_stages = 4;
num_steps = 4;
newton_iter = 5;
model_name = 'sim_pendulum';

h = 0.1;
x0 = [0; 1e-1; 0; 0e0];
u = 0;

%% model
model = pendulum_on_cart_model();

nx = model.nx;
nu = model.nu;

%% acados sim model
sim_model = acados_sim_model();
sim_model.set('name', model_name);
sim_model.set('T', h);

sim_model.set('sym_x', model.sym_x);
if isfield(model, 'sym_u')
    sim_model.set('sym_u', model.sym_u);
end
if isfield(model, 'sym_p')
    sim_model.set('sym_p', model.sym_p);
end

if (strcmp(method, 'erk'))
	sim_model.set('dyn_type', 'explicit');
	sim_model.set('dyn_expr_f', model.expr_f_expl);
else % irk irk_gnsf
	sim_model.set('dyn_type', 'implicit');
	sim_model.set('dyn_expr_f', model.expr_f_impl);
	sim_model.set('sym_xdot', model.sym_xdot);
%	if isfield(model, 'sym_z')
%		sim_model.set('sym_z', model.sym_z);
%	end
end


%% acados sim opts
sim_opts = acados_sim_opts();
sim_opts.set('compile_interface', compile_interface);
sim_opts.set('codgen_model', codgen_model);
sim_opts.set('num_stages', num_stages);
sim_opts.set('num_steps', num_steps);
sim_opts.set('newton_iter', newton_iter);
sim_opts.set('method', method);
sim_opts.set('sens_forw', sens_forw);
sim_opts.set('jac_reuse', jac_reuse);
if (strcmp(method, 'irk_gnsf'))
	sim_opts.set('gnsf_detect_struct', gnsf_detect_struct);
end


%% acados sim
% create sim
sim = acados_sim(sim_model, sim_opts);
% (re)set numerical part of model
%sim.set('T', 0.5);
%sim.C_sim
%sim.C_sim_ext_fun


N_sim = 100;

x_sim = zeros(nx, N_sim+1);
x_sim(:,1) = x0;

tic
for ii=1:N_sim
	
	% set initial state
	sim.set('x', x_sim(:,ii));
	sim.set('u', u);

    % initialize implicit integrator
    if (strcmp(method, 'irk'))
        sim.set('xdot', zeros(nx,1));
    elseif (strcmp(method, 'irk_gnsf'))
        n_out = sim.model_struct.dim_gnsf_nout;
        sim.set('phi_guess', zeros(n_out,1));
    end

	% solve
	sim.solve();


	% get simulated state
	x_sim(:,ii+1) = sim.get('xn');

end
simulation_time = toc


% xn
xn = sim.get('xn');
xn
% S_forw
S_forw = sim.get('S_forw')
Sx = sim.get('Sx');
Su = sim.get('Su');

%x_sim

% for ii=1:N_sim+1
% 	x_cur = x_sim(:,ii);
% 	visualize;
% end

figure;
plot(1:N_sim+1, x_sim);
legend('p', 'theta', 'v', 'omega');


fprintf('\nsuccess!\n\n');


if is_octave()
    waitforbuttonpress;
end
