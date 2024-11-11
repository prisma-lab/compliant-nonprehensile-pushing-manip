clear mex, clear all,  close all, clc
check_acados_requirements()

%% Parameters  
% (all except lb and ub will be updated at each step from 
% C++ code, so their value in this script is not relevant)

%Box dimension parameter
l = 0.1;

%Friction parameters
L_f_x = 1.9620;
L_f_y = 1.9620;
L_f_tau = 0.0751;
mu = 0.2;

%Impedance parameters
kc_x=5;
kc_y=5;
dc_x=5;
dc_y=5;

%End effector offset
r = 0.01;

%actual phi_dot
%phi_p_dot_0 = 0;
%phi_m_dot_0 = 0;

%parameters=[L_f_x; L_f_y; L_f_tau; mu; kc_x; kc_y; dc_x; dc_y; l; r; phi_p_dot_0; phi_m_dot_0]; 
parameters=[L_f_x; L_f_y; L_f_tau; mu; kc_x; kc_y; dc_x; dc_y; l; r];

%Initial conditions
xp_0 = 0;
yp_0 = 0.6;
theta_0 = pi/4;
phi_0 = pi;
xd_0 = -l/2;
yd_0 = 0;
f_x_0 = 0;
f_y_0 = 0;


x0 = [xp_0;yp_0;theta_0;phi_0;...
        xd_0;yd_0; f_x_0; f_y_0];

%State lower bound (lb) and upper bound (ub)
xp_lb = -10; xp_ub = 10;
yp_lb = -10; yp_ub = 10;
theta_lb = -4*pi; theta_ub = 4*pi;
phi_lb = 3*pi/4; phi_ub = 5*pi/4;
xd_lb = -10; xd_ub = 10;
yd_lb = -l/2+0.001; yd_ub = l/2-0.001;
f_x_lb = 0; f_x_ub = 100;
f_y_lb = -100; f_y_ub = 100;

%Control input lower bound (lb) and upper bound (ub)
phi_p_dot_lb = 0; phi_p_dot_ub = 10;
phi_m_dot_lb = 0; phi_m_dot_ub = 10;
f_x_dot_lb = -100; f_x_dot_ub = 100;
f_y_dot_lb = -100; f_y_dot_ub = 100;
epsilon_lb = -100; epsilon_ub = 100;
%xi_lb = 0; xi_ub = 100;

%Constr lower bound (lb) and upper bound (ub)
lambda_m_lb = 0; lambda_m_ub = 100;
lambda_p_lb = 0; lambda_p_ub = 100;
contact_constr_lb = 0; contact_constr_ub = 0;
%phi_dot_max_rate = 1000;
%phi_dot_rate_constr_lb = -phi_dot_max_rate; phi_dot_rate_constr_ub = phi_dot_max_rate;

%State weights
w_xp_err = 1;
w_yp_err = 1;
w_theta_err = 1;
w_phi_err = 1;
w_xd = 1;
w_yd = 1;
w_f_x = 1;
w_f_y = 1;


%Control weights
w_phi_p_dot = 1;
w_phi_m_dot = 1;
w_f_x_dot = 1;
w_f_y_dot = 1;
w_epsilon = 1;
%w_xi = 1;


%Constraints weights
w_push_terminal = 0.1;




%% optionsocp_nlp_solver
ocp_N = 5;
compile_interface = 'auto';
codgen_model = 'true';


% ocp
%ocp_nlp_solver = 'sqp';
ocp_nlp_solver = 'sqp_rti';
ocp_nlp_solver_max_iter = 5;
ocp_qp_solver = 'partial_condensing_hpipm';
%ocp_qp_solver = 'full_condensing_hpipm';
ocp_qp_solver_cond_N = 5;
ocp_sim_method = 'erk';
ocp_sim_method_num_stages = 4;
ocp_sim_method_num_steps = 1;

ocp_cost_type = 'linear_ls';
%ocp_cost_type = 'nonlinear_ls';
%ocp_cost_type = 'ext_cost';
ocp_levenberg_marquardt = 1e-1;
qp_solver_warm_start = 1;
qp_solver_iter_max = 100; % default is 50; OSQP needs a lot sometimes.
%% model
model = pushing_model_updated_reduced();


%% setup problem
%N = 25;
%T = 30; % time horizon length
dt = 0.001;
nx = model.nx;
nu = model.nu;
ny = nx+nu;
ny_e = nx; % number of outputs in mayer term
nbx = nx; % number of state bounds
nbu = nu; % number of input bounds

%cost
Vx = zeros(ny, nx); for ii=1:nx Vx(ii,ii)=1.0; end % state-to-output matrix in lagrange term
Vu = zeros(ny, nu); for ii=1:nu Vu(nx+ii,ii)=1.0; end % input-to-output matrix in lagrange term
Vx_e = zeros(ny_e, nx); for ii=1:nx Vx_e(ii,ii)=1.0; end % state-to-output matrix in mayer term
Q = diag([w_xp_err, w_yp_err, w_theta_err, w_phi_err,...
    w_xd, w_yd, w_f_x, w_f_y]);%, w_kv_x, w_kv_y]);
R = diag([w_phi_p_dot, w_phi_m_dot, w_f_x_dot, w_f_y_dot, w_epsilon]);%, w_xi]);
W = blkdiag(Q, R); % weight matrix in lagrange term
W_e = w_push_terminal*Q; % weight matrix in mayer term
%This will be updated at each step
%yref = zeros(ny, 1); % output reference in lagrange term
%yref_e = zeros(ny_e, 1); % output reference in mayer term
yref = vertcat([0;0;0;0], zeros(ny-4,1));
yref_e = vertcat([0;0;0;0], zeros(ny_e-4,1));

%constraints
Jbx = zeros(nbx, nx); for ii=1:nbx Jbx(ii,ii)=1.0; end
lbx = [xp_lb;yp_lb;theta_lb;phi_lb;...
    xd_lb;yd_lb; f_x_lb; f_y_lb];
ubx = [xp_ub;yp_ub;theta_ub;phi_ub;...
    xd_ub;yd_ub; f_x_ub; f_y_ub];

Jbu = zeros(nbu, nu); for ii=1:nbu Jbu(ii,ii)=1.0; end
lbu = [phi_p_dot_lb;phi_m_dot_lb;f_x_dot_lb;f_y_dot_lb;epsilon_lb];% xi_lb];
ubu = [phi_p_dot_ub;phi_m_dot_ub;f_x_dot_ub;f_y_dot_ub;epsilon_ub];%xi_ub];

%H_min = [lambda_m_lb;lambda_p_lb;contact_constr_lb;phi_dot_rate_constr_lb;phi_dot_rate_constr_lb];
%H_max = [lambda_m_ub;lambda_p_ub;contact_constr_ub;phi_dot_rate_constr_ub;phi_dot_rate_constr_ub];
H_min = [lambda_m_lb;lambda_p_lb;contact_constr_lb];
H_max = [lambda_m_ub;lambda_p_ub;contact_constr_ub];

%% acados ocp model
ocp_model = acados_ocp_model();
model_name = 'pushing_reduced';
ocp_model.set('name', model_name);
ocp_model.set('T', ocp_N*dt);

% symbolics
ocp_model.set('sym_x', model.sym_x);
ocp_model.set('sym_u', model.sym_u);
ocp_model.set('sym_p', model.sym_p);

% cost
ocp_model.set('cost_type', ocp_cost_type);
ocp_model.set('cost_type_e', ocp_cost_type);
ocp_model.set('cost_Vu', Vu);
ocp_model.set('cost_Vx', Vx);
ocp_model.set('cost_Vx_e', Vx_e);
ocp_model.set('cost_W', W);
ocp_model.set('cost_W_e', W_e);
ocp_model.set('cost_y_ref', yref);
ocp_model.set('cost_y_ref_e', yref_e);

% dynamics
ocp_model.set('dyn_type', 'explicit');
ocp_model.set('dyn_expr_f', model.expr_f_expl);

% constraints
ocp_model.set('constr_x0', x0);
ocp_model.set('constr_Jbx', Jbx);
ocp_model.set('constr_lbx', lbx);
ocp_model.set('constr_ubx', ubx);
ocp_model.set('constr_Jbu', Jbu);
ocp_model.set('constr_lbu', lbu);
ocp_model.set('constr_ubu', ubu);
ocp_model.set('constr_expr_h', model.expr_h);
ocp_model.set('constr_lh', H_min); % lower bound on h
ocp_model.set('constr_uh', H_max);  % upper bound on h
%ocp_model.set('constr_type', 'auto');

ocp_model.model_struct;

%% acados ocp opts

ocp_opts = acados_ocp_opts();
ocp_opts.set('parameter_values',parameters);
ocp_opts.set('compile_interface', compile_interface);
ocp_opts.set('codgen_model', codgen_model);
ocp_opts.set('param_scheme_N', ocp_N);
ocp_opts.set('nlp_solver', ocp_nlp_solver);
ocp_opts.set('nlp_solver_max_iter', ocp_nlp_solver_max_iter);
ocp_opts.set('qp_solver', ocp_qp_solver);
if (strcmp(ocp_qp_solver, 'partial_condensing_hpipm'))
    ocp_opts.set('qp_solver_cond_N', ocp_qp_solver_cond_N); %New horizon after partial condensing
end
ocp_opts.set('levenberg_marquardt', ocp_levenberg_marquardt);
ocp_opts.set('sim_method', ocp_sim_method);
ocp_opts.set('sim_method_num_stages', ocp_sim_method_num_stages);
ocp_opts.set('sim_method_num_steps', ocp_sim_method_num_steps);
ocp_opts.set('regularize_method', 'convexify');
ocp_opts.set('qp_solver_warm_start', qp_solver_warm_start);
ocp_opts.set('qp_solver_iter_max', qp_solver_iter_max);

ocp_opts.opts_struct;

%% acados ocp
ocp = acados_ocp(ocp_model, ocp_opts);
ocp_model.set('name', model_name);


%reference = importdata("eight.txt");
% x_traj_init = zeros(nx, ocp_N+1);
x_traj_init = repmat(x0, 1, ocp_N+1);
u_traj_init = zeros(nu, ocp_N);


% set trajectory initialization
ocp.set('init_x', x_traj_init);
ocp.set('init_u', u_traj_init);

%cost_val_ocp = zeros(n_sim, 1);

ocp.set('constr_x0', x0);
    
% solve OCP
ocp.solve();

% get cost value
%cost_val_ocp(ii) = ocp.get_cost();

% get solution
x_traj = ocp.get('x');
u_traj = ocp.get('u');


status = ocp.get('status');

if status==0
    %fprintf('\nsuccess!\n\n');
else
    fprintf('\nsolution failed with status %d!\n\n', status);
    %break;
end



%{
%% Plots
%ts = linspace(0, T, N+1);
States = {'xp';'yp';'theta';'phi';...
    'f_x';'f_y';'xd';'yd'};%;'kv_x';'kv_y'};
figure; hold on;
for i=1:4
    subplot(4, 1, i);
    plot(x_traj(i,:)); grid on;
    ylabel(States{i});
    xlabel('t [s]')
    hold on
    subplot(4, 1, i);
    %plot(reference(i,1:n_sim))
end

figure; hold on;
for i=5:8
    subplot(4, 1, i-4);
    plot(x_traj(i,:)); grid on;
    ylabel(States{i});
    xlabel('t [s]')
end

%ts = linspace(0, T, N);
figure; hold on;
ctrl = {'phi_p_dot', 'phi_m_dot', 'xd_dot', 'yd_dot', 'epsilon'};
for i=1:5
    subplot(5, 1, i);
    plot(u_traj(i,:)); grid on;
    ylabel(ctrl{i});
    xlabel('t [s]')
end
%}

%% go embedded
% to generate templated C code
ocp.generate_c_code;
