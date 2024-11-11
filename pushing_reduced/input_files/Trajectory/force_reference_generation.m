clear, clc, close all
traj=importdata('../eight_scaled_fast.txt');
dt = 0.001;

%Obtain velocity in body frame
%init
y_dot = zeros(1,size(traj,2));
x_dot = zeros(1,size(traj,2));
theta_dot = zeros(1,size(traj,2));
vel = zeros(3,1);
theta = traj(3,:);

%Compute vel in world fr and transform in body fr
for i=2:size(traj,2)
    R=[cos(theta(i)),-sin(theta(i)),0;sin(theta(i)),cos(theta(i)),0;0,0,1];
    vel = [vel, R' * [(traj(1,i) -traj(1, i-1))/dt; (traj(2,i) -traj(2, i-1))/dt; (traj(3,i) -traj(3, i-1))/dt]];	
end

subplot(3,1,1),plot(vel(1,:))
subplot(3,1,2),plot(vel(2,:))
subplot(3,1,3),plot(vel(3,:))


%Contact force at object com in body frame

L = diag([0.0427 0.0445 12.1273]);

force = L \  vel(:, 1);

for i=2:size(traj,2)
	force = [force, L \  vel(:, i)];
end
subplot(3,1,1),plot(force(1,:))
subplot(3,1,2),plot(force(2,:))
subplot(3,1,3),plot(force(3,:))

%{
traj(4,:) =[];
traj = [traj; force];
writematrix(traj,'eight_force_2.txt','Delimiter','tab');
type 'eight_force_2.txt';
%}