%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autor: Manuel Eduardo Gantiva Osorio
% Proyecto: AQUATRONIC
% Escuela Técnica Superior de Ingeniería 
% Universidad Loyola Andalucía
% Fecha: 24.04.2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script configures all parameters for the Cybership II vessel model. It is a
% dynamic model with three degrees of freedom—surge, sway, and yaw—and includes both
% kinematic and dynamic characteristics such as mass, damping, and added mass.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function vessel = initCybership()
%% 1. Parámetros másicos, inerciales y geométricos:

% Cybership II (Skjetne, 2004):

vessel.mass = 23.8;                                                     % Masa del barco (kg)
vessel.rotationalInertia = 1.76;                                        % Inercia respecto al eje de rotación (kg m2)
vessel.centerOfGravityDistanceX = 0.046;                                % Distancia en el eje X desde el centro de gravedad hasta el origen del sistema de referencia centrado en el cuerpo (m)
vessel.centerOfGravityDistanceY = 0;                                    % Distancia en el eje Y desde el centro de gravedad hasta el origen del sistema de referencia centrado en el cuerpo (m)

vessel.distanceThruster4 = 0.75;                                        % Distancia en el eje X del sistema de referencia centrado en el cuerpo desde el origen hasta la línea de actuación del propulsor 4 (m)
vessel.distanceThruster3 = 0.75;                                        % Distancia en el eje X del sistema de referencia centrado en el cuerpo desde el origen hasta la línea de actuación del propulsor 3 (m)
vessel.distanceThruster2 = 0.75;                                        % Distancia en el eje Y del sistema de referencia centrado en el cuerpo desde el origen hasta la línea de actuación del propulsor 2 (m)
vessel.distanceThruster1 = 0.75;                                        % Distancia en el eje Y del sistema de referencia centrado en el cuerpo desde el origen hasta la línea de actuación del propulsor 1 (m)

vessel.totalLength = 1.255;                                             % Longitud total del barco (m)
vessel.widthLength = 0.29;                                              % Ancho del barco (m)
vessel.hullmaximumHeight = 0.251;                                       % Altura máxima aproximada del casco del barco (m)

Loa = vessel.totalLength;
width = vessel.widthLength;
height = vessel.hullmaximumHeight;

A_F_w = width*height;
A_L_w = Loa*1/3*height + 1/3*Loa*2/3*height;

x1 = [0 0 1/3*Loa 1/3*Loa Loa Loa];
y1 = [0 height height 1/3*height 1/3*height 0];
polyin = polyshape(x1,y1);
[sH,sL] = centroid(polyin);

vessel.frontalArea = A_F_w;                                             % Área frontal al viento (m)
vessel.lateralArea = A_L_w;                                             % Área lateral al viento (m)
vessel.lateralAreaCentroidX = sH;                                       % Posición X del centroide del área lateral al viento (m)
vessel.lateralAreaCentroidY = sL;                                       % Posición Y del centroide del área lateral al viento (m)

clear Loa width height A_F_w A_L_w x1 y1 polyin sH sL

%% 2. Parámetros hidrodinámicos 

% Cybership II (Skjetne, 2004):

vessel.X_udot = -2;                                                 	% Parámetro hidrodinámico X_udot (kg)
vessel.Y_vdot = -10;                                                 	% Parámetro hidrodinámico Y_vdot (kg)
vessel.Y_rdot = -0;                                                 	% Parámetro hidrodinámico Y_rdot (kg m)
vessel.N_vdot = -0;                                                 	% Parámetro hidrodinámico N_vdot (kg m)
vessel.N_rdot = -1;                                                 	% Parámetro hidrodinámico N_rdot (kg m2)

vessel.X_u = -0.72253;                                                  % Parámetro hidrodinámico X_u (kg/s)           
vessel.X_u_abs_u = -1.32742;                                            % Parámetro hidrodinámico X_u_abs_u (kg/m)           

vessel.Y_v = -0.88965;                                                  % Parámetro hidrodinámico Y_v (kg/s)           
vessel.Y_v_abs_v = -36.47287;                                           % Parámetro hidrodinámico Y_v_abs_v (kg/m)           
vessel.Y_v_abs_r = -0.805;                                              % Parámetro hidrodinámico Y_v_abs_r (kg)      

vessel.Y_r = -7.25;                                                     % Parámetro hidrodinámico Y_r (kg m/s)           
vessel.Y_r_abs_v = -0.845;                                              % Parámetro hidrodinámico Y_r_abs_v (kg)           
vessel.Y_r_abs_r = -3.45;                                               % Parámetro hidrodinámico Y_r_abs_r (kg m)      

vessel.N_v = 0.0313;                                                    % Parámetro hidrodinámico N_v (kg m/s)           
vessel.N_v_abs_v = 3.95645;                                             % Parámetro hidrodinámico N_v_abs_v (kg)           
vessel.N_v_abs_r = 0.13;                                                % Parámetro hidrodinámico N_v_abs_r (kg m)   

vessel.N_r = -1.9;                                                      % Parámetro hidrodinámico N_r (kg m2/s)           
vessel.N_r_abs_v = 0.08;                                                % Parámetro hidrodinámico N_r_abs_v (kg m)           
vessel.N_r_abs_r = -0.75;                                               % Parámetro hidrodinámico N_r_abs_r (kg m2)    

vessel.xG= vessel.centerOfGravityDistanceX;
vessel.Iz = vessel.rotationalInertia;
vessel.M = [vessel.mass - vessel.X_udot     0    0;
                0   vessel.mass - vessel.Y_vdot  vessel.mass*vessel.xG - vessel.Y_rdot;
                0   vessel.mass*vessel.xG - vessel.N_vdot  vessel.Iz - vessel.N_rdot];
vessel.invM = inv(vessel.M);                                            % Matriz Invertida

%% 3. Límites de acciones de control:

vessel.maxForceThruster1 = 1;                                           % Máxima fuerza aplicable por parte del propulsor 1 (N)
vessel.minForceThruster1 = 0;                                           % Mínima fuerza aplicable por parte del propulsor 1 (N)

vessel.maxForceThruster2 = 1;                                           % Máxima fuerza aplicable por parte del propulsor 2 (N)
vessel.minForceThruster2 = 0;                                           % Mínima fuerza aplicable por parte del propulsor 2 (N)

vessel.maxForceThruster3 = 1;                                           % Máxima fuerza aplicable por parte del propulsor 3 (N)
vessel.minForceThruster3 = -1;                                          % Mínima fuerza aplicable por parte del propulsor 3 (N)

vessel.maxForceThruster4 = 1;                                           % Máxima fuerza aplicable por parte del propulsor 4 (N)
vessel.minForceThruster4 = -1;                                          % Mínima fuerza aplicable por parte del propulsor 4 (N)

%% 4. Límites de acciones de control (Otter):

vessel.y_pont = 0.6;
vessel.k_pos = 0.2216 / 2;                         % Coeficiente propulsor positivo
vessel.k_neg = 0.1289 / 2;                         % Coeficiente propulsor negativo
vessel.maxRPM = sqrt((0.5 * 24.4 * 9.81) / vessel.k_pos);  % Máximo RPM
vessel.minRPM = -sqrt((0.5 * 13.6 * 9.81) / vessel.k_neg); % Mínimo RPM

% vessel.maxThrust = vessel.k_pos * vessel.maxRPM * abs(vessel.maxRPM);    % positive thrust each actuator(N) 
% vessel.minThrust = vessel.k_neg * vessel.minRPM * abs(vessel.minRPM);    % negative thrust each actuator(N) 

vessel.maxThrust = 10;
vessel.minThrust = -6;

vessel.maxForce_u = 2*vessel.maxThrust;                             % Positive Force Surge (N)
vessel.minForce_u = 2*vessel.minThrust;                             % Negative Force Surge (N)
vessel.maxTorque_r =  vessel.y_pont * vessel.maxThrust - vessel.y_pont * vessel.minThrust; % Positive Torque (N m)

end