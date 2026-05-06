%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autor: Manuel Eduardo Gantiva Osorio
% Proyecto: AQUATRONIC
% Escuela Técnica Superior de Ingeniería 
% Universidad Loyola Andalucía  
% Fecha: 06.01.2026
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;clc;clear;
%% Simulation configuration

% Simulation script for multi-ASV (surface vehicles) scenario.
% All added comments are in English. Units: SI (meters [m], seconds [s],
% kilograms [kg], meters/second [m/s], radians [rad], Newtons [N]).
%
% Reference frames / conventions:
% - Assume NED-like horizontal plane: x-forward, y-starboard/right.
% - Heading/angles in radians unless otherwise noted.
%
% Where to change parameters:
% - Vehicle properties are initialized in the functions: initCybership(), initOtter(), initAsvkic().
%   Edit those files (e.g., params inside) to change masses, inertia, geometry, actuator limits, etc.
% - Currents, wind and noise settings are controlled below (this script).
% - Control sample time T_s sets controller update frequency (seconds).

% 1. Simulation
no_vessels = 2;         % number of vessels to simulate [dimensionless]
Tsim = 100;             % total simulation time [s]
T_s = 0.1;              % control sample time / simulation step for control updates [s]

tm = 0:T_s:Tsim;        % time vector (from 0 to Tsim with step T_s) [s]

% 2. ASV parameters
% Initialize vehicle objects/structs. See each init* function for fields and units.
% Typical fields you will find there: mass [kg], Izz [kg*m^2], length [m], max_thrust [N], Cd [-], etc.
vessel_ciber = initCybership();
vessel_otter = initOtter();

% 3. Disturbances
% 3.1. Wind
activate_wind = 1;      % flag: 1 -> enable currents, 0 -> disable (dimensionless)
V_w = 2.5;              % wind speed [m/s]
beta_w = pi/2;         % wind direction [rad] 
wind = [activate_wind * V_w, beta_w];   % packaged wind vector: [speed, direction]

% 4. Ocean Currents
activate_currents = 1;  % flag: 1 -> enable currents, 0 -> disable (dimensionless)
V_c = 0.5;           % current speed [m/s]
beta_c = pi/2;         % current direction [rad] 
ocean_currents = Currents(beta_c, V_c, activate_currents); % packaged current vector: [speed, direction, flag]

% 5. Noise reference generated offline
paramsNoise = struct();
paramsNoise.flag_noise = 1;     % flag: 1 -> enable Noise, 0 -> disable (dimensionless)
paramsNoise.nxy_min  = -0.05;   % min position noise [m]
paramsNoise.nxy_max  = 0.05;    % max position noise [m]
paramsNoise.npsi_min = -0.01;   % min heading noise [rad]
paramsNoise.npsi_max = 0.01;    % max heading noise [rad]
paramsNoise.nr_min   = -0.001;  % min angular-rate noise [rad/s]
paramsNoise.nr_max   =  0.001;  % max angular-rate noise [rad/s]

% Returns noise signals for sensors: position [m], heading [rad], angular rate [rad/s]
noise = initNoise(no_vessels, paramsNoise, Tsim, T_s)';

disturbances_mod = struct();
disturbances_mod.V_w = V_w * activate_wind;
disturbances_mod.beta_w = beta_w * activate_wind;
disturbances_mod.V_c = V_c * activate_currents;
disturbances_mod.beta_c = beta_c * activate_currents;
disturbances_mod.V_c_dot = 0;
disturbances_mod.beta_c_dot = 0;

% Cleanup temporary workspace variables (keeps workspace tidy)
clear activate_currents beta_c V_c V_w activate_wind paramsNoise beta_w

%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creation of the ASV vector, definition of initial state and observers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ASVs = cell(1,no_vessels);

% State vector structure (per ASV)
% X = [ x y z phi theta psi u v w p q r su sv sw sp sq sr TL TR Fu tr ]'
% State vector (per ASV) with units:
%   x, y, z          - position [m]
%   phi, theta, psi  - Euler angles (roll, pitch, yaw) [rad]
%   u, v, w          - linear velocities in body frame (surge, sway, heave) [m/s]
%   p, q, r          - angular rates (roll, pitch, yaw rate) [rad/s]
%   su, sv, sw       - linear sigma-states / Disturbances accelerations [m/s^2] (start at 0)
%   sp, sq, sr       - angular sigma-states / Disturbances angular accelerations [rad/s^2] (start at 0)
%   TL, TR           - left/right thruster forces [N]
%   Fu               - forward propulsion force (surge) [N]
%   tr               - yaw torque / turning moment [N*m]

% Total items per ASV:
no_items = 22;    % length of state vector (22 states/variables)

% Initialize full-state vector for all vessels
X_0 = zeros(no_vessels * no_items, 1);

% History matrix: stores all vessel states over time
% Dimensions: (no_vessels*no_items) x length(time)
hist_X = repmat(X_0, 1, size(tm, 2));

for i = 1:no_vessels

    aux_skip_rows = (i-1)*no_items;  % index offset for vessel i

    switch i
            case 1
                % Position (important initial states)
                x0   = 3.535533906;    % initial x-position [m]
                y0   = 2.535533906;    % initial y-position [m]
                z0   = 0;              % surface level [m]
                % Orientation
                psi0 = pi/2-0.5;           % initial heading / yaw angle [rad]
                % 3-DOF velocities (surge, sway, yaw rate)
                u0 = 0;                % surge velocity [m/s]
                v0 = 0;                % sway velocity [m/s]
                r0 = 0;                % yaw rate [rad/s]
                % Initial control / actuation states
                Fu0 = 0;               % forward propulsion force (surge) [N]
                tr0 = 0;               % turning moment / yaw torque [N*m]
                % Pack initial state vector 
                X_0(1 + aux_skip_rows : no_items + aux_skip_rows) = [ x0; y0; z0; 0; 0; psi0; u0; v0; 0; 0; 0; r0; zeros(8,1); Fu0; tr0 ];
                % Create vessel object
                % initObserver(type, T_s) returns observer parameters:
                %   type = 0 -> ideal observer
                %   type = 1 -> Liu observer
                %   type = 2 -> Bejarano observer
                %   type = 3 -> Zonotope-based observer
                % ASVs{i} = Cybership(vessel_ciber, X_0(1 + aux_skip_rows : no_items + aux_skip_rows), initObserver(0, T_s), wind);
                % ASVs{i} = Asvkic(vessel_kinec, X_0(1 + aux_skip_rows: no_items + aux_skip_rows), initObserver(0,T_s));
                ASVs{i} = Otter(vessel_otter, X_0(1 + aux_skip_rows: no_items + aux_skip_rows), initObserver(0,T_s) ,wind);
            case 2
                % Position (important initial states)
                x0   = -2.535533906;   % initial x-position [m]
                y0   = 2.535533906;    % initial y-position [m]
                z0   = 0;              % surface level [m]
                % Orientation
                psi0 = pi/2;           % initial heading / yaw angle [rad]
                % 3-DOF velocities (surge, sway, yaw rate)
                u0 = 0;                % surge velocity [m/s]
                v0 = 0;                % sway velocity [m/s]
                r0 = 0;                % yaw rate [rad/s]
                % Initial control / actuation states
                Fu0 = 0;               % forward propulsion force (surge) [N]
                tr0 = 0;               % turning moment / yaw torque [N*m]
                X_0(1 + aux_skip_rows : no_items + aux_skip_rows) = [ x0; y0; z0; 0; 0; psi0; u0; v0; 0; 0; 0; r0; zeros(8,1); Fu0; tr0 ];
                ASVs{i} = Cybership(vessel_ciber,X_0(1 + aux_skip_rows: no_items + aux_skip_rows), initObserver(0,T_s) ,wind);
    end

end
hist_X(:, 1) = X_0;

clear x0 y0 z0 psi0 u0 v0 r0 Fu0 tr0
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creation of low-level controllers (IFAC) and definition of their parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

paramsIFACll = struct();
paramsIFACll.k_u   = 2.0;   % surge velocity gain     
paramsIFACll.k_r   = 0.4;   % yaw-rate gain          
paramsIFACll.k_psi = 0.1;   % heading (yaw) gain    
paramsIFACll.tau_d = 200;   % derivative constant for ref control [s]

LLCs = cell(1,no_vessels);

for i = 1:no_vessels
    switch i
            case 1
                LLCs{i} = IfacLowLevelController(vessel_otter,[paramsIFACll.k_u, paramsIFACll.k_r, paramsIFACll.k_psi], T_s, paramsIFACll.tau_d); 
            case 2
                LLCs{i} = IfacLowLevelController(vessel_ciber,[2.6, 3.0, 1.5], T_s, 200); 
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creation of low-level controllers (MPC) and definition of their parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

paramsMPCll = struct();
paramsMPCll.N_p = 20;    % prediction horizon (number of steps) [dimensionless]
paramsMPCll.N_c = 20;    % control horizon (number of steps) [dimensionless]
paramsMPCll.Q  = diag([200, 10, 200]);   % Q: state/output weighting matrix used in the stage cost 
paramsMPCll.R  = diag([0.00, 0.00]);     % R: control effort weighting matrix
paramsMPCll.Nu = diag([0, 0]);% Nu: weighting matrix for the control-reference tracking error
paramsMPCll.u_max = 2.0;      % max surge speed command [m/s]
paramsMPCll.u_min = 0;        % min surge speed command [m/s]
paramsMPCll.v_max = 1.25;     % max sway speed command [m/s]
paramsMPCll.v_min = -1.25;    % min sway speed command [m/s]
paramsMPCll.r_max = pi;       % max yaw rate command [rad/s]
paramsMPCll.r_min = -pi;      % min yaw rate command [rad/s]
paramsMPCll.Delta_F_min = -Inf;  % lower bound on delta forward force [N]
paramsMPCll.Delta_F_max =  Inf;  % upper bound on delta forward force [N]
paramsMPCll.Delta_T_min = -Inf;  % lower bound on delta turning torque [N*m]
paramsMPCll.Delta_T_max =  Inf;  % upper bound on delta turning torque [N*m]

mLLCs = cell(1,no_vessels);
for i = 1:no_vessels
    switch i
            case 1
                mLLCs{i} = MpcLowLevelController(vessel_otter, paramsMPCll, T_s); 
            case 2
                mLLCs{i} = MpcLowLevelController(vessel_ciber, paramsMPCll, T_s); 
    end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creation of low-level controllers (MPC) with Disturbaces and definition of their parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

paramsMPCllDis = struct();
paramsMPCllDis.N_p = 20;    % prediction horizon (number of steps) [dimensionless]
paramsMPCllDis.N_c = 20;    % control horizon (number of steps) [dimensionless]
% paramsMPCllDis.Q  = diag([200, 10, 10, 100]);   % Q: state/output weighting matrix used in the stage cost 
paramsMPCllDis.Q  = diag([0, 50, 10, 1000]);   % Q: state/output weighting matrix used in the stage cost 
paramsMPCllDis.R  = diag([0.00, 0.00]);     % R: control effort weighting matrix
paramsMPCllDis.Nu = diag([0, 0]);% Nu: weighting matrix for the control-reference tracking error
paramsMPCllDis.u_max = 2.0;      % max surge speed command [m/s]
paramsMPCllDis.u_min = 0;        % min surge speed command [m/s]
paramsMPCllDis.v_max = 1.25;     % max sway speed command [m/s]
paramsMPCllDis.v_min = -1.25;    % min sway speed command [m/s]
paramsMPCllDis.r_max = pi;       % max yaw rate command [rad/s]
paramsMPCllDis.r_min = -pi;      % min yaw rate command [rad/s]
paramsMPCllDis.Delta_F_min = -Inf;  % lower bound on delta forward force [N]
paramsMPCllDis.Delta_F_max =  Inf;  % upper bound on delta forward force [N]
paramsMPCllDis.Delta_T_min = -Inf;  % lower bound on delta turning torque [N*m]
paramsMPCllDis.Delta_T_max =  Inf;  % upper bound on delta turning torque [N*m]

mLLCs_dis = cell(1,no_vessels);
for i = 1:no_vessels
    switch i
            case 1
                mLLCs_dis{i} = MpcLlcDisController(vessel_otter, paramsMPCllDis, T_s); 
            case 2
                mLLCs_dis{i} = MpcLlcDisController(vessel_ciber, paramsMPCllDis, T_s); 
    end

end

%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creation of path following controllers vector (SGLOS) and definition of their parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

paramsWangml = struct();
paramsWangml.delta_sglos = 2.0;       % SGLOS lookahead distance [m]
paramsWangml.k_u_tar    = 1.0;       % surge-speed convergence gain [1/s]
paramsWangml.path_d     = pathScript(); % path definition (waypoints/trajectory) [m]
paramsWangml.tau_d      = 5;       % derivative constant for SGLOS control [s]


MLCs = cell(1,no_vessels);
for i = 1:no_vessels
    switch i
            case 1
                MLCs{i} = WangLosController([paramsWangml.delta_sglos, paramsWangml.k_u_tar], T_s, paramsWangml.tau_d, paramsWangml.path_d); 
            case 2
                MLCs{i} = WangLosController([paramsWangml.delta_sglos, paramsWangml.k_u_tar], T_s, paramsWangml.tau_d, paramsWangml.path_d); 
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creation of path following controllers vector (MPC) and definition of their parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

t_s_mlc  = 0.4;                    % MPC Band sampling time [s]
relacion_T = round(t_s_mlc/T_s);   % ratio between controller periods

paramsMPChl = struct();
paramsMPChl.N_p   = 40;                 % prediction horizon [steps]
paramsMPChl.N_c   = 40;                 % control horizon [steps]
paramsMPChl.Q     = diag([50,50,0,0,1]);% Q: state/output weighting matrix
paramsMPChl.R     = diag([0,0,100.0]);  % R: control effort weighting matrix
paramsMPChl.Nu    = diag([20,0,0]);     % Nu: control-reference tracking weighting
paramsMPChl.t_sim = T_s;                % internal MPC simulation time [s]

mMLCs = cell(1,no_vessels);
for i = 1:no_vessels
    switch i
            case 1
                mMLCs{i} = MpcPFController(vessel_otter, paramsMPChl, t_s_mlc); 
            case 2
                mMLCs{i} = MpcPFController(vessel_ciber, paramsMPChl, t_s_mlc); 
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creation of path following controllers vector (MPC) disturbances and definition of their parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

paramsMPChldis = struct();
paramsMPChldis.N_p   = 40;                 % prediction horizon [steps]
paramsMPChldis.N_c   = 40;                 % control horizon [steps]
paramsMPChldis.Q     = diag([50,50,0,0,1]);% Q: state/output weighting matrix
paramsMPChldis.R     = diag([0,0,100.0]);  % R: control effort weighting matrix
paramsMPChldis.Nu    = diag([20,0,0]);     % Nu: control-reference tracking weighting
paramsMPChldis.t_sim = T_s;                % internal MPC simulation time [s]
paramsMPChldis.path = 0;

mMLCs_dis = cell(1,no_vessels);
for i = 1:no_vessels
    switch i
            case 1
                mMLCs_dis{i} = MpcPFDisController(vessel_otter, paramsMPChldis, t_s_mlc); 
            case 2
                mMLCs_dis{i} = MpcPFDisController(vessel_ciber, paramsMPChldis, t_s_mlc); 
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creation of PF controllers vector (SGLOS) with model and definition of their parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

paramsSglosMp = struct();
paramsSglosMp.delta_sglos = 2.0;       % SGLOS lookahead distance [m]
paramsSglosMp.k_u_tar    = 1.0;       % surge-speed convergence gain [1/s]
paramsSglosMp.tau_d      = 5;         % derivative constant for SGLOS control [s]
paramsSglosMp.k_psi      = 0.8;         % heading (yaw) gain  
paramsSglosMp.N_p   = 30;             % prediction horizon [steps]
paramsSglosMp.N_c   = 30;             % control horizon [steps]
paramsSglosMp.t_sim = T_s;                % internal MPC simulation time [s]

mMLCs_sgmp = cell(1,no_vessels);
for i = 1:no_vessels
    switch i
            case 1
                mMLCs_sgmp{i} = SglosMpController(vessel_otter, paramsSglosMp, t_s_mlc); 
            case 2
                mMLCs_sgmp{i} = SglosMpController(vessel_ciber, paramsSglosMp, t_s_mlc); 
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creation and description of storage vectors
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Datos: X_hat = [ x y psi u v r su sv sr ]' 
% Datos: var_mlc = [ u_ref psi_ref r_ref x_e y_e w ]' 
% Datos: mpc_0 = [ j_min p_vec u_opt n_i t_sol succes ]' 149
% Datos: mpc_dis_0 = [ j_min p_vec u_opt n_i t_sol succes ]' 290
% Datos: mpc_hl_0 = [ j_min p_vec u_opt w xe ye vbar n_i t_sol succes ]' 456

X_hat = zeros(no_vessels*9,1);
var_mlc = zeros(no_vessels*6,1);
mpc_0 = zeros(no_vessels*(1 + (3+3*paramsMPCll.N_p+2*paramsMPCll.N_c+2)+ (2*paramsMPCll.N_c) + 3),1);
mpc_dis_0 = zeros(no_vessels*(1 + (4+4*paramsMPCllDis.N_p+2*paramsMPCllDis.N_c+2+6*paramsMPCllDis.N_p)+ (2*paramsMPCllDis.N_c) + 3),1);
mpc_hl_0 = zeros(no_vessels*(1 + (5+5*paramsMPChl.N_p+3*paramsMPChl.N_c+3)+ (3*paramsMPChl.N_c) + 7),1);
mpc_hl_dis_0 = zeros(no_vessels*(1 + (5+5*paramsMPChldis.N_p+3*paramsMPChldis.N_c+3+6*paramsMPChldis.N_p+5) + (3*paramsMPChldis.N_c) + 7),1);
sgmp_hl_0 = zeros(no_vessels*(1 + (5+5+6*paramsSglosMp.N_p+3) + (5*paramsSglosMp.N_c) + 4),1);

hist_X_hat        = repmat(X_hat, 1, size(tm, 2));                          % history of estimated ASV states
hist_mlc          = repmat(var_mlc, 1, size(tm, 2));                        % history of MLC controllers
hist_mpc_llc      = repmat(mpc_0, 1, size(tm, 2));                          % history of MPC low-level controllers
hist_mpc_llc_dis  = repmat(mpc_dis_0, 1, size(tm, 2));                          % history of MPC low-level with Disturbances controllers
hist_mpc_hlc      = repmat(mpc_hl_0, 1, ceil((size(tm, 2)-1)/relacion_T));      % history of MPC high-level controllers
hist_mpc_hlc_dis  = repmat(mpc_hl_dis_0, 1, ceil((size(tm, 2)-1)/relacion_T));  % history of MPC high-level controllers Disturbances
hist_sgmp_hlc_dis  = repmat(sgmp_hl_0, 1, ceil((size(tm, 2)-1)/relacion_T));  % history of MPC high-level controllers Disturbances

clear v aux_skip_rows i path_d mpc_0 mpc_hl_0 mpc_dis_0 mpc_hl_mod_0 mpc_hl_dis_0 paramsMPChl sgmp_hl_0...
      paramsMPChldis paramsMPCll paramsMPChlMod paramsMPCCoordinated var_mlc paramsIFACll paramsWangml

%% Simulation loop

t1 = datetime('now');
h = waitbar(0, 'Simulando...');     % Creation of progress bar
record_mlc = relacion_T;            % Variable for sampling time synchronization
for k=1:((Tsim-0)/T_s)
    hist_X_hat(:, k) = get_estimations(ASVs);               % Storage of the estimates
    hist_mlc(:, k) = WangLosController.get_mlc(MLCs);    % Storage wang MLC path following
    for v = 1:no_vessels
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Control phase per ship MPC Multilayer Path following
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        switch v
            case 1
                % ref = mMLCs{v}.step(1.0, ASVs{v}.observer_obj.getEstimations());
                ref = mMLCs_dis{v}.step(1.0, ASVs{v}.observer_obj.getEstimations(), disturbances_mod);
                % ref = mMLCs_sgmp{v}.step(1.0, ASVs{v}.observer_obj.getEstimations(), disturbances_mod);
                % ref = MLCs{v}.step(1.0, ASVs{v}.observer_obj.getEstimations());
            case 2
                % ref = mMLCs{v}.step(1.0, ASVs{v}.observer_obj.getEstimations());
                ref = mMLCs_dis{v}.step(1.0, ASVs{v}.observer_obj.getEstimations(), disturbances_mod);
                % ref = mMLCs_sgmp{v}.step(1.0, ASVs{v}.observer_obj.getEstimations(), disturbances_mod);
                % ref = MLCs{v}.step(1.0, ASVs{v}.observer_obj.getEstimations());
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Control phase per ship SGLOS Wang Path following
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % switch v
        %     case 1
        %         ref = MLCs{v}.step(1.0, ASVs{v}.observer_obj.getEstimations());
        %     case 2
        %         ref = MLCs{v}.step(1.0, ASVs{v}.observer_obj.getEstimations());
        % end

        % ASVs{v}.setTao(mLLCs_dis{v}.step([1.5; pi/2; 0.0], ASVs{v}.observer_obj.getEstimations(), disturbances_mod, T_s));
        ASVs{v}.setTao(mLLCs_dis{v}.step(ref, ASVs{v}.observer_obj.getEstimations(), disturbances_mod, T_s));
        % ASVs{v}.setTao(mLLCs{v}.step(ref, ASVs{v}.observer_obj.getEstimations(), T_s));
        % ASVs{v}.setTao(LLCs{v}.step(ref, ASVs{v}.observer_obj.getEstimations()));  

        % ASVs{v}.setTao([5.5; 0; 0.0]);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Data storage phase and evolve
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    X_0 = multi_asv_ode_int(tm(k), T_s, X_0, ASVs, ocean_currents, noise(:,k));
    hist_X(:, k+1) = X_0;
    hist_mpc_llc(:, k) = MpcLowLevelController.get_mpc_llc(mLLCs);
    hist_mpc_llc_dis(:, k) = MpcLlcDisController.get_mpc_llc(mLLCs_dis);

    % Store data at a different sampling rate
    if record_mlc == relacion_T
        hist_mpc_hlc(:, (k-1)/relacion_T +1) = MpcPFController.get_mpc_hlc(mMLCs);
        hist_mpc_hlc_dis(:, (k-1)/relacion_T +1) = MpcPFDisController.get_mpc_hlc(mMLCs_dis);
        hist_sgmp_hlc_dis(:, (k-1)/relacion_T +1) = SglosMpController.get_mpc_hlc(mMLCs_sgmp);
        record_mlc=1;
    else
        record_mlc = record_mlc + 1;
    end
    waitbar(k / ((Tsim-0)/T_s), h, sprintf('Simulando... %d%%', round(100 * k / ((Tsim-0)/T_s))));
end

hist_mlc(:, k+1) = WangLosController.get_mlc(MLCs);
hist_mpc_llc(:, k+1) = MpcLowLevelController.get_mpc_llc(mLLCs);
hist_mpc_llc_dis(:, k+1) = MpcLlcDisController.get_mpc_llc(mLLCs_dis);
% Store data at a different sampling rate
if record_mlc == relacion_T
    hist_mpc_hlc(:, (k)/relacion_T +1) = MpcPFController.get_mpc_hlc(mMLCs);
    hist_mpc_hlc_dis(:, (k)/relacion_T +1) = MpcPFDisController.get_mpc_hlc(mMLCs_dis);
    hist_sgmp_hlc_dis(:, (k)/relacion_T +1) = SglosMpController.get_mpc_hlc(mMLCs_sgmp);
end

hist_X_hat(:, k+1) = get_estimations(ASVs);
t2 = datetime('now');
disp(['Tiempo ejecucion: ', num2str(seconds(t2 - t1)), ' segundos']);
close(h); % Close the progress bar at the end

clear v X_0 k no_vessels X_hat t1 t2 ref Tsim h record_mlc relacion_T

%% Plot
% Parámetros de estilo
set(0,'DefaultFigureWindowStyle','docked')
% Figure_main: plots general parameters of the entire fleet (6 DOF states for all ASVs)
Figure_main(hist_X, tm, ASVs, no_items)

% Figure_Obs: compares real states, estimated states, and measured signals for a selected ASV
% (first argument is the vehicle index to visualize)
Figure_Obs(1, hist_X, hist_X_hat, noise, no_items, tm)
%% Figures for each implemented controller
% Figure_LLC: plots performance of the low-level backstepping controller for a selected ASV
% Figure_LLC(veh_idx, hist_X, hist_X_hat, hist_ref, no_items, tm)
% Figure_LLC(1, hist_X, hist_X_hat, hist_ref, no_items, tm)

% Figure_PF: plots performance of the path-following controller (MLC) for a selected ASV
% Figure_PF(veh_idx, hist_X, hist_X_hat, hist_mlc, tm)
% Figure_PF(1, hist_X, hist_X_hat, hist_mlc, tm)
% Figure_PF(2, hist_X, hist_X_hat, hist_mlc, tm)

% Figure_mpc_llc: plots performance of the MPC low-level controller for a selected ASV
% Figure_mpc_llc_dis(veh_idx, hist_X, hist_X_hat, hist_mpc_llc_dis, tm, offset, mLLCs_dis{veh_idx})
Figure_mpc_llc_dis(1, hist_X, hist_X_hat, hist_mpc_llc_dis, tm, 4.0, mLLCs_dis{1})
Figure_mpc_llc_dis(2, hist_X, hist_X_hat, hist_mpc_llc_dis, tm, 4.0, mLLCs_dis{2})

% Figure_mpc_llc: plots performance of the MPC low-level controller for a selected ASV
% Figure_mpc_llc(veh_idx, hist_X, hist_X_hat, hist_mpc_llc, tm, offset, mLLCs{veh_idx})
% Figure_mpc_llc(1, hist_X, hist_X_hat, hist_mpc_llc, tm, 0.0, mLLCs{1})

% Figure_mpc_hlc: plots performance of the MPC high-level controller for a selected ASV
% Figure_mpc_hlc(veh_idx, hist_X, hist_X_hat, hist_mpc_hlc, tm, offset, mMLCs{veh_idx})
% Figure_mpc_hlc(1, hist_X, hist_X_hat, hist_mpc_hlc, tm, 0.0, mMLCs{1})

% Figure_mpc_hlc: plots performance of the MPC high-level controller for a selected ASV
% Figure_mpc_hlc(veh_idx, hist_X, hist_X_hat, hist_mpc_hlc, tm, offset, mMLCs_dis{veh_idx})
Figure_mpc_hlc_dis(1, hist_X, hist_X_hat, hist_mpc_hlc_dis, tm, 0.0, mMLCs_dis{1})
Figure_mpc_hlc_dis(2, hist_X, hist_X_hat, hist_mpc_hlc_dis, tm, 0.0, mMLCs_dis{2})

% Figure_sgmp_hlc: plots performance of the SGLOS MP high-level controller for a selected ASV
% Figure_sgmp_hlc(veh_idx, hist_X, hist_X_hat, hist_sgmp_hlc_dis, tm, offset, mMLCs_sgmp{veh_idx})
% Figure_sgmp_hlc(1, hist_X, hist_X_hat, hist_sgmp_hlc_dis, tm, 4.0, mMLCs_sgmp{1})
% Figure_sgmp_hlc(2, hist_X, hist_X_hat, hist_sgmp_hlc_dis, tm, 4.0, mMLCs_sgmp{2})

% All these functions produce visualizations for ONE selected vehicle.
% The first argument is always the vehicle index to visualize.
%% Exercise animations by control type
set(0,'DefaultFigureWindowStyle','normal')
% animateFleetPF(hist_X, hist_mlc, tm, ASVs, MLCs, no_items)
% animateFleetMPCPF(hist_X, hist_mpc_hlc, tm, ASVs, mMLCs, no_items)
animateFleetMPCPFDis(hist_X, hist_mpc_hlc_dis, tm, ASVs, mMLCs_dis, no_items, disturbances_mod)
% animateFleetSGLOSMP(hist_X, hist_sgmp_hlc_dis, tm, ASVs, mMLCs_sgmp, no_items, disturbances_mod)
% animateFleet(hist_X, tm, ASVs, no_items)
% animateFleet3D(hist_X, tm, ASVs, no_items)