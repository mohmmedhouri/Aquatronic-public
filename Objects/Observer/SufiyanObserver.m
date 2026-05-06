classdef SufiyanObserver < matlab.System
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Autor: Manuel Eduardo Gantiva Osorio
    %        Guillermo Bejarano Pellicer
    %        Federico Peralta Samaniego
    % Proyecto: AQUATRONIC
    % Escuela Técnica Superior de Ingeniería 
    % Universidad Loyola Andalucía
    % Fecha: 24.04.2025
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % This class implements a state observer based on the method proposed by Sufiyan and 
    % Bejarano (2020). It is designed to operate under realistic conditions by accounting
    % for noise in position measurements, improving estimation accuracy in practical scenarios.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    properties (Access=private) 
        prev_estimations        %Estado Estimados del barco state = [ x y psi u v r sigma_u sigma_v sigma_r]' 
    end

    properties (Nontunable)
        type
        
        Lbar_psi
        PpWp
        invM
        Ts
    end

    methods
        function obj = SufiyanObserver(observer_param)

            obj = obj@matlab.System();
            
            obj.Lbar_psi = observer_param.Lbar_psi;
            obj.PpWp = observer_param.PpWp;

            obj.type = "SufiyanDecoupled";
            obj.Ts = observer_param.Ts;
            obj.invM = observer_param.invM;
            obj.prev_estimations = [observer_param.X_0;zeros(3,1)]; %Posiciones, velocidades y perturbaciones

        end

        function states_hat = getEstimations(obj)
            states_hat = obj.prev_estimations;

        end
    end


    methods(Access = protected)
        function stepImpl(obj, eta_noise, tau_n)
            %% 4. Implementación del observador de Liu:
            eta_noise = eta_noise(1:3);  %No usa la velocidad
            tau = [tau_n(1);0;tau_n(2)];
            psi = eta_noise(3);
            cos_psi = cos(psi);
            sin_psi = sin(psi);
            ROT = [cos_psi,    - sin_psi;
                   sin_psi,      cos_psi];

            A_p = zeros(6);
            A_p(1:2,3:4) = ROT;
            A_p(3:4,5:6) = eye(2);
            
            T_inv = eye(6);
            T_inv(1:2,1:2) = ROT;

            M_p = obj.invM(1:2,:);

            B_p = zeros(6,3);
            B_p(3:4,:) = M_p;

            A_psi = [0.0, 1.0, 0.0;
                    0.0, 0.0, 1.0;
                    0.0, 0.0, 0.0]; 
            
            M_psi = obj.invM(3,:);
            
            B_psi = zeros(3,3);
            B_psi(2,:) = M_psi;
            L_p_x = T_inv*obj.PpWp*ROT';

            X_est_p = [obj.prev_estimations(1:2);obj.prev_estimations(4:5);obj.prev_estimations(7:8)];
            X_est_psi = [obj.prev_estimations(3);obj.prev_estimations(6);obj.prev_estimations(9)];

            X_est_p_dot = A_p*X_est_p + B_p*tau + L_p_x*(eta_noise(1:2) - obj.prev_estimations(1:2));
            X_est_psi_dot = A_psi*X_est_psi + B_psi*tau + obj.Lbar_psi*(psi - obj.prev_estimations(3));

            eta_est_p_dot = X_est_p_dot(1:2,1);
            nu_est_p_dot = X_est_p_dot(3:4,1);
            sigma_est_p_dot = X_est_p_dot(5:6,1);

            psi_est_dot = X_est_psi_dot(1,1);
            r_est_dot = X_est_psi_dot(2,1);
            sigma3_est_dot = X_est_psi_dot(3,1);

            eta_est_dot = [eta_est_p_dot;psi_est_dot];
            nu_est_dot = [nu_est_p_dot;r_est_dot];
            sigma_est_dot = [sigma_est_p_dot;sigma3_est_dot];
            
            %% 5. Agrupación de variables de salida de la función
            obj.prev_estimations = [eta_est_dot;nu_est_dot;sigma_est_dot]* obj.Ts + obj.prev_estimations;
            % output = obj.prev_estimations;
        end
    end
end

