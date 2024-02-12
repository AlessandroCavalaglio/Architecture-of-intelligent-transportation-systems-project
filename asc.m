function Mz = asc(delta, X, extra_params, beta_ref, gamma_ref, vehicle_data)

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
gamma     = X(6);       % [rad/s] yaw rate

Fx__fr = extra_params(7);
Fx__fl = extra_params(8);

beta = atan2(v, u);
beta_dot = 0.0;

gamma_dot_ref = 0.0;
beta_dot_ref = 0.0;

xi = 0.2;
S = gamma - gamma_ref + xi*(beta - beta_ref);

k_p = 8.0;
k_s = 0.5;

Mz = -Lf*(Fx__fr + Fx__fl)*sin(delta) - izz*(k_p*S + k_s*sign(S) - gamma_dot_ref + xi*(beta_dot - beta_dot_ref));
