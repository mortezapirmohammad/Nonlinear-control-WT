function u = SMC(x1,x2,x3,x4,t,i)
    %% desired path
    x1d = -4*pi/(180*(4*pi/30))*cos(4*pi/30*t+3*pi/2)+4*pi/180*t;
    x2d = 4*pi/180*sin(4*pi/30*t+3*pi/2)+4*pi/180;
    x3d = (4*pi/30)*4*pi/180*cos(4*pi/30*t+3*pi/2);
    x4d = -(4*pi/30)^2*4*pi/180*sin(4*pi/30*t+3*pi/2);
    x4d_t = -(4*pi/30)^3*4*pi/180*cos(4*pi/30*t+3*pi/2);
    %% system parameters
    La = 0.8;
    Lb = 1.2;
    Lc = sqrt(La^2+Lb^2);
    R = 1.13;
    phi1 = atan(Lb/La);
    J = 1238;
    D = 1;
    C = 1000;
    Ae = pi/4*(0.05^2-0.025^2);
    B = 1.5e9;
    V = Ae*.8;
    Ce = 1e-4 + 1e-5;
    De = 1*1;
 
    %% Design Sliding Mode Controller 
    int_z=x1;
    z=x2;
    z_t=x3;
    z_tt=x4;
    X = (La^2+Lc^2-2*La*Lc*cos(z+phi1))^.5;
    phi2 = asin(Lc*sin(phi1+z)/X);
    X_dot = La*Lc*sin(phi1+z)*z_t/sqrt(R^2+Lc^2-2*La*Lc*cos(phi1+z));
 
    beta = -D/J*z_tt-C/J*z_t+Ae*R/J*sin(phi2)*(-4*B*Ae/V*X_dot ...
        -4*B*Ce/V*(z_tt+D/J*z_t+C/J*z));
    alpha = Ae*R/J*sin(phi2)*4*B/V*De;
    
    ueq=1/alpha*(x4d_t-beta-Ae);
    k=0.00001;
    s = 1;
    u(i) =ueq-k*sign(s) ;
    

end