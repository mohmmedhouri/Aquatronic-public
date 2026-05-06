%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autor: Manuel Eduardo Gantiva Osorio
% Proyecto: AQUATRONIC
% Escuela Técnica Superior de Ingeniería 
% Universidad Loyola Andalucía
% Fecha: 24.04.2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script sets the parameters required to initialise a vehicle object. The model is
% purely kinematic, focusing solely on the vehicle's motion without accounting for
% dynamics such as mass, forces, or moments.
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function vessel = initAsvkic()
%% 1. Parámetros másicos, inerciales y geométricos:

vessel.invM = zeros(3,3);  

%% 2. Parámetros hidrodinámicos 

% ASV sin dinámica:
vessel.maxU = 5;                             % Positive Surge (m/s)
vessel.maxR = 3;                             % Positive yaw (rad/s)
vessel.T_s = 0.05;

vessel.X_udot = 0;                                                 	% Parámetro hidrodinámico X_udot (kg)
vessel.Y_vdot = 0;                                                 	% Parámetro hidrodinámico Y_vdot (kg)
vessel.Y_rdot = 0;                                                 	% Parámetro hidrodinámico Y_rdot (kg m)
vessel.N_vdot = 0;                                                 	% Parámetro hidrodinámico N_vdot (kg m)
vessel.N_rdot = 0;                                                 	% Parámetro hidrodinámico N_rdot (kg m2)

vessel.X_u = 0;                                                  % Parámetro hidrodinámico X_u (kg/s)           
vessel.X_u_abs_u = 0;                                            % Parámetro hidrodinámico X_u_abs_u (kg/m)           

vessel.Y_v = 0;                                                  % Parámetro hidrodinámico Y_v (kg/s)           
vessel.Y_v_abs_v = 0;                                           % Parámetro hidrodinámico Y_v_abs_v (kg/m)           
vessel.Y_v_abs_r = 0;                                              % Parámetro hidrodinámico Y_v_abs_r (kg)      

vessel.Y_r = 0;                                                     % Parámetro hidrodinámico Y_r (kg m/s)           
vessel.Y_r_abs_v = 0;                                              % Parámetro hidrodinámico Y_r_abs_v (kg)           
vessel.Y_r_abs_r = 0;                                               % Parámetro hidrodinámico Y_r_abs_r (kg m)      

vessel.N_v = 0;                                                    % Parámetro hidrodinámico N_v (kg m/s)           
vessel.N_v_abs_v = 0;                                             % Parámetro hidrodinámico N_v_abs_v (kg)           
vessel.N_v_abs_r = 0;                                                % Parámetro hidrodinámico N_v_abs_r (kg m)   

vessel.N_r = 0;                                                      % Parámetro hidrodinámico N_r (kg m2/s)           
vessel.N_r_abs_v = 0;                                                % Parámetro hidrodinámico N_r_abs_v (kg m)           
vessel.N_r_abs_r = 0;                                               % Parámetro hidrodinámico N_r_abs_r (kg m2)    

vessel.Iz= 0.1;
vessel.mass = 0.1;
vessel.M = [vessel.mass      0    0;
                0   vessel.mass   0;
                0   0    vessel.Iz];
vessel.invM = inv(vessel.M);                                            % Matriz Invertida

end