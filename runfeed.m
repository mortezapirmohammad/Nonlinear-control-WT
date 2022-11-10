
function dy = feed_lin(t,y)
%% desired path
    yd = 4*pi/180*sin(4*pi/30*t+3*pi/2)+4*pi/180;
    yd_t = (4*pi/30)*4*pi/180*cos(4*pi/30*t+3*pi/2);
    yd_tt = -(4*pi/30)^2*4*pi/180*sin(4*pi/30*t+3*pi/2);
    yd_ttt = -(4*pi/30)^3*4*pi/180*cos(4*pi/30*t+3*pi/2);
%% system parameters
La= 0.8;
Lb=1.2;
R=1.13;
Lc=sqrt(La^2+Lb^2);
Phi1=atan(Lb/La);
J=1238;
D=1000;
Ae=pi/4*(0.05^2-0.025^2);
B=1.5e9;V=Ae*0.8;
Ce=1e-4+1e-5;De=1*1;
%% Design Feedback Linearization Controller 

    X = (La^2+Lc^2-2*La*Lc*cos(y(1)+phi1))^.5;
    phi2 = asin(Lc*sin(phi1+y(1))/X);
    X_dot = La*Lc*sin(phi1+y(1))*y(2)/sqrt(R^2+Lc^2-2*La*Lc*cos(phi1+y(1)));
 
    beta = -D/J*y(3)-C/J*y(2)+Ae*R/J*sin(phi2)*(-4*B*Ae/V*X_dot ...
        -4*B*Ce/V*(y(3)+D/J*y(2)+C/J*y(1)));
    alpha = Ae*R/J*sin(phi2)*4*B/V*De;
 
    
    u = 1/alpha*(-beta+v);
    dy = [y(2) ; y(3) ; beta+alpha*u ];
    %% Close Loop System
for 
    
    X = (La^2+Lc^2-2*La*Lc*cos(y+phi1))^.5;
    phi2 = asin(Lc*sin(phi1+y)/X);
    X_dot = La*Lc*sin(phi1+y)*y_t/sqrt(R^2+Lc^2-2*La*Lc*cos(phi1+y));
 
    beta = -D/J*y_tt-C/J*y_t+Ae*R/J*sin(phi2)*(-4*B*Ae/V*X_dot ...
        -4*B*Ce/V*(y_tt+D/J*y_t+C/J*y));
    alpha = Ae*R/J*sin(phi2)*4*B/V*De;
 
    
end

end

figure;
plot(t,x2d,t,X2,'linewidth',2);
xlabel('time (sec)');
ylabel('\theta (rad)'); 
grid on;
title('sine tracking (feedback linearization control)');
legend('simulation','desired');
ylim([-0.2 0.2]);
 
figure;
plot(t,x2d-X2,'linewidth',2);
xlabel('time (sec)');
ylabel('Error (rad)'); 
grid on;
title('tracking error (feedback linearization control)');
  
figure
plot(t,u)
xlabel('time (sec)');
ylabel('voltage (v)'); 
grid on;
title('control signal (feedback linearization control)');


