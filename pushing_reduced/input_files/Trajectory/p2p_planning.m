clear, clc, close all
%The code is similar to ex_2_path_planning.m, but now a time law of
%s defined in trapezoidal_vel_profile.m is considered

%Definition of initial time and final time
t_i=0;
t_f=20;

%Definition of sample time
dt=0.001;

%Definition of the initial and final configurations
x_i=0; y_i=0.6; theta_i=pi/4;
x_f=0.25; y_f=0.4; theta_f=-pi/2;

%Definition of parameters
k=0.5;
ax=k*cos(theta_f) - 3*x_f;
ay=k*sin(theta_f) - 3*y_f;
bx=k*cos(theta_i) + 3*x_i;
by=k*sin(theta_i) + 3*y_i;

%Initialization
t=t_i:dt:t_f;

s=zeros(1,length(t));
dot_s=zeros(1,length(t));
ddot_s=zeros(1,length(t));

x=zeros(1,length(t));
y=zeros(1,length(t));
theta=zeros(1,length(t));
phi=zeros(1,length(t));

x_p=zeros(1,length(t));
y_p=zeros(1,length(t));

wpts = [0 1];
tpts = [t_i t_f];
tvec = t;
[s, dot_s, ddot_s, pp]=quinticpolytraj(wpts, tpts, tvec);


%Path planning through Cartesian polynomial

for i=1:length(t)

%Determination of s in function of time using a trapezoidal velocity
%profile law
%(trapezoidal_vel_profile computes also ddot_s, but in this case it's not
%taken in account)

%[s(i), dot_s(i)]=trapezoidal_vel_profile(t(i), t_i, t_f, 1);

%Cartesian coordinates
x(i)=s(i)^3*x_f - (s(i)-1)^3*x_i + ax*s(i)^2*(s(i)-1) + bx*s(i)*(s(i)-1)^2;
y(i)=s(i)^3*y_f - (s(i)-1)^3*y_i + ay*s(i)^2*(s(i)-1) + by*s(i)*(s(i)-1)^2;

%First derivative
x_p(i)=3*s(i)^2*x_f - 3*(s(i)-1)^2*x_i + ax*(2*s(i)*(s(i)-1)+s(i)^2) + bx*((s(i)-1)^2+2*s(i)*(s(i)-1));
y_p(i)=3*s(i)^2*y_f - 3*(s(i)-1)^2*y_i + ay*(2*s(i)*(s(i)-1)+s(i)^2) + by*((s(i)-1)^2+2*s(i)*(s(i)-1));

%Orientation
theta(i)=atan2(y_p(i), x_p(i));

%Pusher angle
phi(i)=3.14;
end


figure 
hold on
subplot(3,1,1)
plot(t,x)
subplot(3,1,2)
plot(t,y)
subplot(3,1,3)
plot(t,theta)

figure
plot(x,y)

p2p=[x;y;theta;phi];
writematrix(p2p,'p2p_20s.txt','Delimiter','tab');
type 'p2p_20s.txt';

