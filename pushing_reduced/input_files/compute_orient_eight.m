clear, clc, close all
traj=importdata('eight_scaled_fast.txt');



for i=1:size(traj,2)
    if i<5
        traj(3,i) = pi/4;
    end
    if i > size(traj,2) - 5
        traj(3,i) = pi/4;
    end
end

plot(traj(3,:))

%{
dt = 0.001;
y_dot = zeros(1,size(traj,2));
x_dot = zeros(1,size(traj,2));
traj2 = zeros(1,size(traj,2));

for i=1:size(traj,2)
	if i>1
		y_dot(1,i) = (traj(2,i) -traj(2, i-1))/dt;
		x_dot(1,i) = (traj(1,i) -traj(1, i-1))/dt;
	end
end
traj2 = atan2(y_dot,  x_dot);

%adjust
for i=1:size(traj2,2)
    if i==1
        traj2(i) = traj2(i+1);
    end
    if traj2(i) > pi/2
        traj2(i) = traj2(i) - 2*pi;
    end
end


figure 
hold on
subplot(3,1,1)
plot(traj(1,:))
subplot(3,1,2)
plot(traj(2,:))
subplot(3,1,3)
plot(traj2)

figure
plot(traj(1,:),traj(2,:))

%}
eight_adjusted=traj;
writematrix(eight_adjusted,'eight_adjusted_40s.txt','Delimiter','tab');
type 'eight_adjusted_40s.txt';

