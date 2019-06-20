function dx = BebopQuadcopterModel()
    
% global constants
g       = 9.81;


% model states
Cdrag   = 0.35;
OMEGA_0 = 7.8;
ETA     = 1.8;

% drone states
pos             = x(index.droneStates.pos); % position
vel             = x(index.droneStates.vel); % x y velocity
rp_del          = x(index.droneStates.rp_del);
tmp_angles =    [rp_del(2);-rp_del(1)];

omega           = x(index.droneStates.omega);

%% inputs
u_rp              = u(index.droneInputs.rp);
u_vel_z           = u(index.droneInputs.vel_z); % z velocity



%% defferental equation of the quadrotor model
u_acc =   tan(tmp_angles)*g; % helper function

% diff equations
dpos       = [vel;u_vel_z];
dvel       = u_acc-Cdrag*vel;
drp_del    = omega;
domega     = (OMEGA_0(1)^2*(u_rp - rp_del)   -  ETA(1)*OMEGA_0(1)*omega);

dx = [dpos;dvel;drp_del;domega];

end