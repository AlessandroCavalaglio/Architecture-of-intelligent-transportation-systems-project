function [T1, T2, T3, T4] = consmin(Mz, X, dX, vehicle_data)

Lf = vehicle_data.vehicle.Lf;  % [m] Distance between vehicle CoG and front wheels axle
    Lr = vehicle_data.vehicle.Lr;  % [m] Distance between vehicle CoG and front wheels axle
    L  = vehicle_data.vehicle.L;   % [m] Vehicle length
    Wf = vehicle_data.vehicle.Wf;  % [m] Width of front wheels axle 
    Wr = vehicle_data.vehicle.Wr;  % [m] Width of rear wheels axle                   
    m  = vehicle_data.vehicle.m;   % [kg] Vehicle Mass
    g  = vehicle_data.vehicle.g;   % [m/s^2] Gravitational acceleration
    hs = vehicle_data.vehicle.hGs;
    izz= vehicle_data.vehicle.i_zz;

u         = X(4);       % [m/s] vehicle longitudinal speed
v         = X(5);       % [m/s] vehicle lateral speed
Omega     = X(6);       % [rad/s] yaw rate
Fz__rr    = X(7);       % [N] vertical force for the rear right wheel
Fz__rl    = X(8);       % [N] vertical force for the rear left wheel
Fz__fr    = X(9);       % [N] vertical force for the front right wheel
Fz__fl    = X(10);      % [N] vertical force for the front left wheel
delta     = X(11);      % [rad] steering angle (at the wheel)

  

alpha__rr = X(16);      % [rad] side slip angle rear right wheel
alpha__rl = X(17);      % [rad] side slip angle rear left wheel
alpha__fr = X(18);      % [rad] side slip angle front right wheel
alpha__fl = X(19);      % [rad] side slip angle front left wheel


R = 0.3270;
u_dot = dX(4);
Tq = m*u_dot/R;
mu = 0.8;


B = [ 1,  1,  1,  1;
    -Wf/(2*R),  Wf/(2*R),  -Wr/(2*R),  Wr/(2*R) ];
%B = [];

v_c = [Tq; Mz];
% v_c = [10;10];

lb = [-mu*Fz__fl*R, -mu*Fz__fr*R ,-mu*Fz__rl*R, -mu*Fz__rr*R];

ub = [mu*Fz__fl*R, mu*Fz__fr*R, mu*Fz__rl*R, mu*Fz__rr*R];

options = optimoptions('fmincon','Display','iter','Algorithm','sqp');

fun = @(T)T(1)^2/(mu*Fz__fl*R)^2 + T(2)^2/(mu*Fz__fr*R)^2 + T(3)^2/(mu*Fz__rl*R)^2 + T(4)^2/(mu*Fz__rr*R)^2;

T = fmincon(fun, [1, 1, 1, 1], [], [], B, v_c, lb, ub , [], options);

T1 = T(1);
T2 = T(2);
T3 = T(3);
T4 = T(4);

% T1 = 0;
% T2 = 0;
% T3 = 0;
% T4 = 0; 
