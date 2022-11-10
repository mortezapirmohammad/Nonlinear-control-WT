clc;
clear;
close all;
 
%% Create Simulation Time & Sampling Time
dt=0.001;
T=30;
t=0:dt:T-dt;
n=4;
X1(1) = 0;
X2(1) = 0;
X3(1) = 0;
X4(1) = 0;
 
 
%% desired path
x1d = 4*pi/180*sin(4*pi/30*t+3*pi/2)+4*pi/180*t;
x2d = 4*pi/180*sin(4*pi/30*t+3*pi/2)+4*pi/180;
x3d = (4*pi/30)*4*pi/180*cos(4*pi/30*t+3*pi/2);
x4d = -(4*pi/30)^2*4*pi/180*sin(4*pi/30*t+3*pi/2);
x4d_t = -(4*pi/30)^3*4*pi/180*cos(4*pi/30*t+3*pi/2);
% x1
x1=X1;
 
x1_t=X2;
x2_t=X3;
x3_t=X4;

 
%% Close Loop System
for i=2:30000-1
 
    u(i)=SMC(X1(i-1),X2(i-1),X3(i-1),X4(i-1),t(i-1),i);
    
    x4_t(i)=Plant(X1(i),X2(i),X3(i),X4(i),u(i));
    x3_t(i)=dt*x4_t(i)+x3_t(i-1);
    x2_t(i)=dt*x3_t(i)+x2_t(i-1);
    x1_t(i)=dt*x2_t(i)+x1_t(i-1);
    x1(i)=dt*x1_t(i)+x1(i-1);
    
    X1(i+1)=x1;
    X2(i+1)=x1_t;
    X3(i+1)=x2_t;
    X4(i+1)=x3_t;
    
end
u(n)=u(n-1);
%% Plot Result
figure;
plot(t,x2d,'linewidth',2);
hold on
plot(t,X2,'linewidth',2);
xlabel('time (sec)');
ylabel('\theta (rad)'); 
grid on;
title('sine tracking (Sliding Mode control)');
legend('simulation','desired');
ylim([-0.2 0.2]);
 
figure;
plot(t,x2d,'linewidth',2);
xlabel('time (sec)');
ylabel('Error (rad)'); 
grid on;
title('tracking error (Sliding Mode control)');
  
figure
plot(t,u)
xlabel('time (sec)');
ylabel('voltage (v)'); 
grid on;
title('control signal (Sliding Mode control)');
