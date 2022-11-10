function x4_t=Plant(x1,x2,x3,x4,u)
 
    %% system parameters
    La = 0.8;
    Lb = 1.2;
    R = 1.13;
    Lc = sqrt(La^2+Lb^2);
    phi1 = atan(Lb/La);
    J = 1238;
    D = 1;
    C = 1000;
    Ae = pi/4*(0.05^2-0.025^2);
    B = 1.5e9;
    V = Ae*.8;
    Ce = 1e-4 + 1e-5;
    De = 1*1;
%% Create System
    z_tt=x4;
    z=x2;
    z_t=x3;
    X = (La^2+Lc^2-2*La*Lc*cos(z+phi1))^.5;
    X_dot = La*Lc*sin(phi1+z)*z_t/sqrt(R^2+Lc^2-2*La*Lc*cos(phi1+z));
    phi2 = asin(Lc*sin(phi1+z)/X);
    beta = -D/J*z_tt-C/J*z_t+Ae*R/J*sin(phi2)*(-4*B*Ae/V*X_dot ...
        -4*B*Ce/V*(z_tt+D/J*z_t+C/J*z));
    alpha = Ae*R/J*sin(phi2)*4*B/V*De;
 
    x4_t=beta+alpha*u;

end
