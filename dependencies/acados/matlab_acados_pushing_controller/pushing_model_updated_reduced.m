function model = pushing_model_updated_reduced()

import casadi.*

%% system dimensions
%state and control dimension
nx = 8;
nu = 5;
%np = 12;
np=10;
%% sample time
dt = 0.001;


%% named symbolic variables  
%Pusher-slider state
xp = SX.sym('xp'); % x position
yp = SX.sym('yp'); % y position
theta= SX.sym('theta');% slider theta
phi = SX.sym('phi');% pusher angle

%Admittance state
xd = SX.sym('xd');                   
yd = SX.sym('yd');
f_x = SX.sym('f_x');
f_y = SX.sym('f_y');



%Compact state variables
xs = vertcat(xp,yp,theta,phi);
x_ed = vertcat(xd, yd);
f = vertcat(f_x, f_y);

%Control inputs
phi_p_dot = SX.sym('phi_p_dot');
phi_m_dot = SX.sym('phi_m_dot');
f_x_dot = SX.sym('f_x_dot');                   
f_y_dot = SX.sym('f_y_dot');                   
epsilon = SX.sym('epsilon');
%xi = SX.sym('xi');


%Compact control variables

%% (unnamed) symbolic variables
sym_x = vertcat(xs, x_ed, f);
sym_u = vertcat(phi_p_dot, phi_m_dot, f_x_dot, f_y_dot, epsilon);%, xi);
sym_p = SX.sym('p', np, 1);
sym_xdot = SX.sym('xdot', nx, 1);


%% system parameters
%L=[0.2261,0,0;0,0.2220,0;0,0,0.4480];
%L=[0.1156,0,0;0,0.1154,0;0,0,0.2129];
L = diag([sym_p(1), sym_p(2), sym_p(3)]);
mu=sym_p(4);  %%friction coefficient btw pusher and slider


%Admittance parameters
%stiffness
kc_x=sym_p(5);
kc_y=sym_p(6);

%damping
dc_x=sym_p(7);
dc_y=sym_p(8);

%box dim
l = sym_p(9);

%end effector offset (sphere radius)
r = sym_p(10);

%actual phi_dot
%phi_p_dot_0 = sym_p(11);
%phi_m_dot_0 = sym_p(12);




%Impedance matrices
Kc = diag([kc_x, kc_y]);
Dc = diag([dc_x, dc_y]);



%% dynamics

%Contact point body frame
x_ec = [-l/2 - r; (l/2) * tan(pi - phi)];
x_ec_dot = [0;  -(l/2)*(phi_p_dot - phi_m_dot)/(cos(phi)^2)];

%Box orientation
R=[cos(theta),-sin(theta),0;sin(theta),cos(theta),0;0,0,1];

%Contact Jacobian
J=[1,0,-x_ec(2);0,1,x_ec(1)];


%Box dynamics
xs_dot = vertcat(R*L*(J')*f, phi_p_dot - phi_m_dot);

%force dynamics
f_dot = vertcat(f_x_dot, f_y_dot);

%Impedance dynamics
%x_ed_dot = Dc\(f - Kc*(x_ed - x_ec)) + x_ec_dot;
x_ed_dot = Kc\f_dot + x_ec_dot;


%Whole system dynamics
expr_f_expl = vertcat(xs_dot, x_ed_dot, f_dot);
expr_f_impl = expr_f_expl - sym_xdot;


%% constraints
expr_constraint_1=(mu*f_x-f_y);
expr_constraint_2=(mu*f_x+f_y);
expr_constraint_3=[mu*f_x-f_y;mu*f_x+f_y]'*[phi_p_dot;phi_m_dot] + epsilon;
%expr_constraint_4=phi_p_dot - phi_p_dot_0;
%expr_constraint_5=phi_m_dot - phi_m_dot_0;
%Bound constraints are defined from outside
%expr_h = vertcat(expr_constraint_1,expr_constraint_2, expr_constraint_3, expr_constraint_4,expr_constraint_5);
expr_h = vertcat(expr_constraint_1,expr_constraint_2, expr_constraint_3);

%% populate structure
model.nx = nx;
model.nu = nu;
model.sym_x = sym_x;
model.sym_p = sym_p;
model.sym_u = sym_u;
model.sym_xdot = sym_xdot;
model.expr_f_expl = expr_f_expl;
model.expr_f_impl = expr_f_impl;
model.expr_h = expr_h;
