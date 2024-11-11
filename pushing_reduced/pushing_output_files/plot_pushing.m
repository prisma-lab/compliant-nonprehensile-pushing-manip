clear all, clc, close all

folder_name = "sim_obstacle_tank/";

%File reading
des_trajectory=importdata(folder_name+"file_des_trajectory.csv");
current_state=importdata(folder_name+"file_current_state.csv");
current_control=importdata(folder_name+"file_current_control.csv");
reference_TCP_not_passive=importdata(folder_name+"file_reference_TCP_not_passive.csv");
reference_TCP_passive=importdata(folder_name+"file_reference_TCP_passive.csv");
actual_TCP=importdata(folder_name+"file_actual_TCP.csv");
passive_filter_state=importdata(folder_name+"file_passive_filter_state.csv");

%desired object trajectory
x_obj_des = des_trajectory(1,:);
y_obj_des = des_trajectory(2,:);
theta_obj_des = des_trajectory(3,:);

%current state
x_obj = current_state(1,:);
y_obj = current_state(2,:);
theta_obj = current_state(3,:);
phi = current_state(4,:);
x_MPC_des_body = current_state(5,:);
y_MPC_des_body = current_state(6,:);
f_x_MPC_body = current_state(7,:);
f_y_MPC_body = current_state(8,:);


%current control
phi_p_dot = current_control(1,:);
phi_m_dot = current_control(2,:);
phi_dot = phi_p_dot - phi_m_dot;
f_x_dot_MPC_body = current_control(3,:);
f_y_dot_MPC_body = current_control(4,:);
epsilon = current_control(5,:);

%reference TCP not passive
x_TCP_des_np_world = reference_TCP_not_passive(1,:);
y_TCP_des_np_world = reference_TCP_not_passive(2,:);
x_TCP_des_np_world_dot = reference_TCP_not_passive(3,:);
y_TCP_des_np_world_dot = reference_TCP_not_passive(4,:);

%reference TCP passive
%x_TCP_p_world = reference_TCP_passive(1,:);
%y_TCP_p_world = reference_TCP_passive(2,:);
%x_dot_TCP_p_world = reference_TCP_passive(3,:);
%y_dot_TCP_p_world = reference_TCP_passive(4,:);

%actual TCP
x_TCP_world = actual_TCP(1,:);
y_TCP_world = actual_TCP(2,:);
x_TCP_world_dot = actual_TCP(3,:);
y_TCP_world_dot = actual_TCP(4,:);

%passive filter state
x_TCP_des_p_world = passive_filter_state(1,:);
y_TCP_des_p_world = passive_filter_state(2,:);
x_TCP_des_p_world_dot = passive_filter_state(3,:);
y_TCP_des_p_world_dot = passive_filter_state(4,:);
x_MPC_des_body_dot = passive_filter_state(5,:);
y_MPC_des_body_dot = passive_filter_state(6,:);
z = passive_filter_state(7,:);
T = passive_filter_state(8,:);
alpha_p = passive_filter_state(9,:);
gamma_p = passive_filter_state(10,:);
phi_p = passive_filter_state(11,:);
V = passive_filter_state(12,:);
H = passive_filter_state(13,:);



%Time
%dim = size(des_trajectory);
dim = size(actual_TCP);
n_steps = dim(2);
dt = 0.001;
time = linspace(0, n_steps,n_steps+1)*dt;
%Remove last sample time because in all data last sample was not saved
%because of an error in code. Then adjust this
time(end)=[];
%x_obj_des(end)=[];
%y_obj_des(end)=[];
%theta_obj_des(end)=[];


%error
err_x_obj = x_obj_des - x_obj;
err_y_obj = y_obj_des - y_obj;
err_theta_obj = theta_obj_des - theta_obj;

%error TCP
err_x_TCP = x_TCP_des_np_world - x_TCP_world;
err_y_TCP = y_TCP_des_np_world - y_TCP_world;

%% PLOT

%current state
personal_plot2(time, x_obj, x_obj_des, '\(t\) [s]', '\(x_{b}\) [m]', '\(x_{b}\)', '\(x_{b}^*\)', 'plot_x_obj.pdf');
personal_plot2(time, y_obj, y_obj_des, '\(t\) [s]', '\(y_{b}\) [m]', '\(y_{b}\)', '\(y_{b}^*\)', 'plot_y_obj.pdf');
personal_plot2(time, theta_obj, theta_obj_des, '\(t\) [s]', '\(\theta_{b}\) [rad]', '\(\theta_{b}\)', '\(\theta_{b}^*\)', 'plot_theta_obj.pdf');
personal_plot1(time, phi, '\(t\) [s]', '\(\mathit{\Phi}_b\) [rad]', 'plot_phi.pdf');
%personal_plot2(time, x_MPC_des_body, y_MPC_des_body, '\(t\) [s]', '\(x_{d_{MPC}}^b\) [m]', '\(x_d^b\)', '\(y_d^b\)', 'plot_des_POS_body.pdf');
personal_plot2(time, f_x_MPC_body, f_y_MPC_body, '\(t\) [s]', '\(f_c^b\) [N]', '\(f_{c_x}^b\)', '\(f_{c_y}^b\)', 'plot_force_body.pdf');

%personal_plot2(time, x_MPC_des_body_dot, y_MPC_des_body_dot, '\(t\) [s]', '\(\dot{x}_{d_{MPC}}^b\) [m]', '\(\dot{x}_d^b\)', '\(\dot{y}_d^b\)', 'plot_des_POS_body.pdf');


%Trajectory in plane
personal_plot_plane_traj(x_obj, y_obj, x_obj_des, y_obj_des, '\(x_{obj}\) [m]', '\(y_{obj}\) [m]', '\(x_{b},y_{b}\)', '\(x_{b}^*,y_{b}^*\)', 'plot_plane.pdf');
%personal_plot_plane_traj(y_obj, x_obj, y_obj_des, x_obj_des, '\(y_{obj}\) [m]', '\(x_{obj}\) [m]', '\(y_{obj},x_{obj}\)', '\(y_{obj}^*,x_{obj}^*\)', 'plot_plane_inv.pdf');

%{
%current control
personal_plot2(time, phi_p_dot, phi_m_dot, '\(t\) [s]', '\(\dot{\mathit{\Phi}}\) [rad/s]', '\(\dot{\mathit{\Phi}}_+\)', '\(\dot{\mathit{\Phi}}_-\)', 'plot_phi_dot.pdf');
personal_plot1(time, phi_dot, '\(t\) [s]', '\(\dot{\mathit{\Phi}}\) [rad]', 'plot_phi_dot.pdf');
personal_plot2(time, f_x_dot_MPC_body, f_y_dot_MPC_body, '\(t\) [s]', '\(\dot{f}\) ', '\(\dot{f}_x^b\)', '\(\dot{f}_y^b\)', 'plot_force.pdf');
personal_plot1(time, epsilon, '\(t\) [s]', '\(\epsilon\)', 'F_plot_epsilon.pdf');
%}

%error
personal_plot3(time, err_x_obj, err_y_obj, err_theta_obj, '\(t\) [s]', '\(\mathcal{E}\) [m or rad]', '\(e_x\)', '\(e_y\)', '\(e_\theta\)', 'plot_error.pdf');
%{
%TCP
personal_plot3(time, x_TCP_des_np_world, x_TCP_des_p_world, x_TCP_world, '\(t\) [s]', '\(x_{TCP}\) [m]', '\(x_{{TCP}_{np}}^*\)', '\(x_{{TCP}_{p}}^*\)', '\(x_{TCP}\)', 'plot_x_TCP.pdf');
personal_plot3(time, y_TCP_des_np_world, y_TCP_des_p_world, y_TCP_world, '\(t\) [s]', '\(y_{TCP}\) [m]', '\(y_{{TCP}_{np}}^*\)', '\(y_{{TCP}_{p}}^*\)', '\(y_{TCP}\)', 'plot_y_TCP.pdf');
personal_plot3(time, x_TCP_des_np_world_dot, x_TCP_des_p_world_dot, x_TCP_world_dot, '\(t\) [s]', '\(\dot{x}_{TCP}\) [m/s]', '\(\dot{x}_{{TCP}_{np}}^*\)', '\(\dot{x}_{{TCP}_{p}}^*\)', '\(\dot{x}_{TCP}\)', 'plot_x_dot_TCP.pdf');
personal_plot3(time, y_TCP_des_np_world_dot, y_TCP_des_p_world_dot, y_TCP_world_dot, '\(t\) [s]', '\(\dot{y}_{TCP}\) [m/s]', '\(\dot{y}_{{TCP}_{np}}^*\)', '\(\dot{y}_{{TCP}_{p}}^*\)', '\(\dot{y}_{TCP}\)', 'plot_y_dot_TCP.pdf');
%}

personal_plot2(time, x_TCP_world, x_TCP_des_p_world,  '\(t\) [s]', '\(x_{e}\) [m]', '\(x_e\)', '\(x_{e}^*\)', 'plot_x_TCP.pdf');
personal_plot2(time, y_TCP_world, y_TCP_des_p_world,  '\(t\) [s]', '\(y_{e}\) [m]', '\(y_e\)', '\(y_{e}^*\)', 'plot_y_TCP.pdf');
personal_plot2(time, err_x_TCP, err_y_TCP,  '\(t\) [s]', '\(\mathcal{E}_{e}\) [m]', '\(e_x\)', '\(e_y\)', 'plot_err_TCP.pdf');


%TANK
personal_plot1(time, z, '\(t\) [s]', '\(z\)', 'plot_z.pdf');
personal_plot1(time, T, '\(t\) [s]', '\(T\)', 'plot_T.pdf');

personal_plot1(time, alpha_p, '\(t\) [s]', '\(\alpha\)', 'plot_alpha_p.pdf');
personal_plot1(time, gamma_p, '\(t\) [s]', '\(\gamma\)', 'plot_gamma_p.pdf');
personal_plot1(time, phi_p, '\(t\) [s]', '\(\varphi\)', 'plot_phi_p.pdf');

%{
personal_plot1(time, V, '\(t\) [s]', '\(V\)', 'F_plot_V.pdf');
personal_plot1(time, H, '\(t\) [s]', '\(H\)', 'F_plot_H.pdf');
personal_plot2(time, f_x_MPC_body, f_y_MPC_body, '\(t\) [s]', '\(f\) [N]', '\(f_x^b\)', '\(f_y^b\)', 'F_plot_force.pdf');
personal_plot2(time, f_p_x_world, f_p_y_world, '\(t\) [s]', '\(f_{comp}\) [N]', '\(f_x^b\)', '\(f_y^b\)', 'F_plot_force.pdf');
%}
