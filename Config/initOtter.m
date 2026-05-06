%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autor: Manuel Eduardo Gantiva Osorio
% Proyecto: AQUATRONIC
% Escuela Técnica Superior de Ingeniería 
% Universidad Loyola Andalucía
% Fecha: 24.04.2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script sets up the full parameter set for the Otter vessel model. It is a six
% degrees of freedom dynamic model that accounts for both kinematic and dynamic aspects,
% including inertia, damping, restoring forces, and added mass.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function vessel = initOtter()
%% 1. Parámetros físicos y geométricos
vessel.mass = 55.0;                                 % Masa total sin carga útil (kg)
vessel.rg = [0.2; 0; -0.2];                         % Centro de gravedad del casco (m)
vessel.L = 2.0;                                     % Longitud (m)
vessel.B = 1.08;                                    % Manga (ancho) (m)
vessel.T_sway = 1;                                  % Constante de tiempo en sway
vessel.T_yaw = 1;                                   % Constante de tiempo en yaw
vessel.Umax = 6 * 0.5144;                           % Velocidad máxima (m/s)

% Radio de giro
vessel.R44 = 0.4 * vessel.B;
vessel.R55 = 0.25 * vessel.L;
vessel.R66 = 0.25 * vessel.L;

%% 2. Parámetros del pontón (flotación)
vessel.B_pont = 0.25;
vessel.y_pont = 0.395;
vessel.Cw_pont = 0.75;
vessel.Cb_pont = 0.4;

%% 3. Carga útil (payload)
vessel.mp = 25;                            % Masa de la carga útil (kg)
vessel.rp = [0; 0; 0];                      % Posición respecto al centro del casco (m)

vessel.g = 9.81;
vessel.rho = 1025;

%% 4. Propulsión
vessel.k_pos = 0.02216 / 2;                         % Coeficiente propulsor positivo
vessel.k_neg = 0.01289 / 2;                         % Coeficiente propulsor negativo

vessel.maxRPM = sqrt((0.5 * 24.4 * 9.81) / vessel.k_pos);  % Máximo RPM
vessel.minRPM = -sqrt((0.5 * 13.6 * 9.81) / vessel.k_neg); % Mínimo RPM

vessel.maxThrust = vessel.k_pos * vessel.maxRPM * abs(vessel.maxRPM);    % positive thrust each actuator(N) 
vessel.minThrust = vessel.k_neg * vessel.minRPM * abs(vessel.minRPM);    % negative thrust each actuator(N) 

vessel.maxForce_u = 2*vessel.maxThrust;                             % Positive Force Surge (N)
vessel.minForce_u = 2*vessel.minThrust;                             % Negative Force Surge (N)
vessel.maxTorque_r =  vessel.y_pont * vessel.maxThrust - vessel.y_pont * vessel.minThrust; % Positive Torque (N m)

%% 5. Calculos

% === Centro de masas corregido ===
vessel.rg_corrected = (vessel.mass * vessel.rg + vessel.mp * vessel.rp) / (vessel.mass + vessel.mp);

% === Inercia ===
Ig_CG = vessel.mass * diag([vessel.R44^2, vessel.R55^2, vessel.R66^2]);
vessel.Ig = Ig_CG - vessel.mass * Smtrx(vessel.rg_corrected)^2 - vessel.mp * Smtrx(vessel.rp)^2;

% === Matrices base ===
I3 = eye(3); O3 = zeros(3,3);
H = Hmtrx(vessel.rg_corrected);
MRB_CG = [(vessel.mass + vessel.mp) * I3, O3;
           O3, vessel.Ig];
vessel.MRB = H' * MRB_CG * H;
vessel.H = H;

% === Masa añadida ===
Xudot = -addedMassSurge(vessel.mass, vessel.L, vessel.rho);
Yvdot = -1.5 * vessel.mass;
Zwdot = -1.0 * vessel.mass;
Kpdot = -0.2 * vessel.Ig(1,1);
Mqdot = -0.8 * vessel.Ig(2,2);
Nrdot = -1.7 * vessel.Ig(3,3);
vessel.MA = -diag([Xudot, Yvdot, Zwdot, Kpdot, Mqdot, Nrdot]);

% === Hidrostáticarg_corrected ===
nabla = (vessel.mass + vessel.mp) / vessel.rho;
T = nabla / (2 * vessel.Cb_pont * vessel.B_pont * vessel.L);
Aw_pont = vessel.Cw_pont * vessel.L * vessel.B_pont;

I_T = 2 * (1/12)*vessel.L*vessel.B_pont^3 * ...
      (6*vessel.Cw_pont^3 / ((1+vessel.Cw_pont)*(1+2*vessel.Cw_pont))) + ...
      2 * Aw_pont * vessel.y_pont^2;

I_L = 0.8 * 2 * (1/12) * vessel.B_pont * vessel.L^3;

KB = (1/3)*(5*T/2 - 0.5*nabla/(vessel.L*vessel.B_pont));
BM_T = I_T/nabla;
BM_L = I_L/nabla;
KM_T = KB + BM_T;
KM_L = KB + BM_L;
KG = T - vessel.rg_corrected(3);
GM_T = KM_T - KG;
GM_L = KM_L - KG;

G33 = vessel.rho * vessel.g * 2 * Aw_pont;
G44 = vessel.rho * vessel.g * nabla * GM_T;
G55 = vessel.rho * vessel.g * nabla * GM_L;
G_CF = diag([0 0 G33 G44 G55 0]);

H_g = Hmtrx([-0.2, 0, 0]);
vessel.G = H_g' * G_CF * H_g;

% === Parámetros de amortiguamiento (que dependen de M y G, se recalculan dentro de step) ===
vessel.T_sway = vessel.T_sway;
vessel.T_yaw = vessel.T_yaw;
vessel.Umax = vessel.Umax;

% === Datos geométricos útiles ===
vessel.L = vessel.L;
vessel.B = vessel.B_pont;
vessel.T = T;
vessel.Aw_pont = Aw_pont;

vessel.M = vessel.MRB + vessel.MA;

vessel.invM = inv([vessel.M(1,1) vessel.M(1,2) vessel.M(1,6); ...
                   vessel.M(2,1) vessel.M(2,2) vessel.M(2,6); ...
                   vessel.M(6,1) vessel.M(6,2) vessel.M(6,6)]);                                            % Es para el observador o controladores solo 3 DOF                   

vessel.g_payload = [0; 0; vessel.mp * vessel.g];
vessel.m_payload_base = Smtrx(vessel.rp);
vessel.invG_trim = inv(vessel.G(3:5,3:5));

vessel.X_u = -24.4 * vessel.g / vessel.Umax;
vessel.Y_v = -vessel.M(2,2) / vessel.T_sway;
vessel.Zw = -2 * 0.3 * sqrt(G33/vessel.M(3,3)) * vessel.M(3,3);
vessel.Kp = -2 * 0.2 * sqrt(G44/vessel.M(4,4)) * vessel.M(4,4);
vessel.Mq = -2 * 0.4 * sqrt(G55/vessel.M(5,5)) * vessel.M(5,5);
vessel.N_r = -vessel.M(6,6) / vessel.T_yaw;


vessel.X_u_abs_u = 0.0;                           % Parámetro hidrodinámico X_u_abs_u (kg/m)           
vessel.Y_v_abs_v = 0.0;                           % Parámetro hidrodinámico Y_v_abs_v (kg/m)           
vessel.Y_v_abs_r = 0.0;                           % Parámetro hidrodinámico Y_v_abs_r (kg)      
vessel.Y_r = 0.0;                                 % Parámetro hidrodinámico Y_r (kg m/s)           
vessel.Y_r_abs_v = 0.0;                           % Parámetro hidrodinámico Y_r_abs_v (kg)           
vessel.Y_r_abs_r = 0.0;                           % Parámetro hidrodinámico Y_r_abs_r (kg m)      
vessel.N_v = 0.0;                                 % Parámetro hidrodinámico N_v (kg m/s)           
vessel.N_v_abs_v = 0.0;                           % Parámetro hidrodinámico N_v_abs_v (kg)           
vessel.N_v_abs_r = 0.0;                           % Parámetro hidrodinámico N_v_abs_r (kg m)   
vessel.N_r_abs_v = 0.0;                           % Parámetro hidrodinámico N_r_abs_v (kg m)           
vessel.N_r_abs_r = 10*vessel.N_r;                 % Parámetro hidrodinámico N_r_abs_r (kg m2)   


%% 6. Otros parámetros útiles

vessel.totalLength = 2;                                                 % Longitud total del barco (m)
vessel.widthLength = 1.08;                                              % Ancho del barco (m)
vessel.hullmaximumHeight = 1.64;                                        % Altura máxima aproximada del barco con la estructura de la cámara (m)
vessel.hullmediumHeight = 0.45;                                         % Altura del casco del barco (m)
vessel.partialLength1 = 0.65;                                           % Longitud desde la proa hasta el inicio de la estructura de la cámara (m)
vessel.partialLength2 = 0.4;                                            % Longitud de la estructura de la cámara (m)
vessel.waterLine = 0.25;                                                % Línea de agua (m)

Loa = vessel.totalLength;
width = vessel.widthLength;
height = vessel.hullmaximumHeight;

A_F_w = width*height;
A_L_w = Loa*vessel.hullmediumHeight + vessel.partialLength2*(vessel.hullmaximumHeight - vessel.hullmediumHeight);

x1 = [0 ...
      0 ...
      vessel.partialLength1 ...
      vessel.partialLength1 ...
      vessel.partialLength1 + vessel.partialLength2 ...
      vessel.partialLength1 + vessel.partialLength2 ...
      vessel.totalLength ...
      vessel.totalLength ...
      ];
y1 = [0 ...
      vessel.hullmediumHeight ...
      vessel.hullmediumHeight ...
      vessel.hullmaximumHeight ...
      vessel.hullmaximumHeight ...
      vessel.hullmediumHeight ...
      vessel.hullmediumHeight ...
      0 ...
      ];
polyin = polyshape(x1,y1);
[sH,sL] = centroid(polyin);

vessel.frontalArea = A_F_w;                                             % Área frontal al viento (m)
vessel.lateralArea = A_L_w;                                             % Área lateral al viento (m)
vessel.lateralAreaCentroidX = sH;                                       % Posición X del centroide del área lateral al viento (m)
vessel.lateralAreaCentroidY = sL - vessel.waterLine;                    % Posición Y del centroide del área lateral al viento respecto a la línea de agua (m)

end