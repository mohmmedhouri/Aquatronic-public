classdef LiuConvergeObs < matlab.System
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Autor: Manuel Eduardo Gantiva Osorio
    %        Guillermo Bejarano Pellicer
    %        Federico Peralta Samaniego
    % Proyecto: AQUATRONIC
    % Escuela Técnica Superior de Ingeniería 
    % Universidad Loyola Andalucía
    % Fecha: 24.04.2025
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % This class implements a state observer for planar motion with three degrees of freedom,
    % based on the method proposed by Liu (2019). It assumes noise-free position and 
    % orientation measurements, providing state estimates under ideal sensor conditions.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    properties (Access=private)
        
        prev_estimations        %Estado Estimados del barco state = [ x y psi u v r sigma_u sigma_v sigma_r]' 
    end

    properties (Nontunable)
        type 

        epsilon
        alpha 
        invM
        Ts
    end

    methods
        function obj = LiuConvergeObs(observer_param)

            obj = obj@matlab.System();
            obj.epsilon = observer_param.epsilon;
            obj.alpha = observer_param.alpha;

            obj.type = "LiuConverger";
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
            ROT_discreto_est = [cos(psi),    - sin(psi),      0;
                                sin(psi),      cos(psi),      0;
                                       0,             0,      1];

            aux = ROT_discreto_est.'*(obj.prev_estimations(1:3) - eta_noise)/(obj.epsilon^2);
            bracket_aux_eta_est_dot = sign(aux).*(abs(aux).^obj.alpha);
            eta_est_dot = - 3*obj.epsilon*ROT_discreto_est*bracket_aux_eta_est_dot + ROT_discreto_est* obj.prev_estimations(4:6);

            bracket_aux_nu_est_dot = sign(aux).*(abs(aux).^(2*obj.alpha - 1));
            nu_est_dot = -3*bracket_aux_nu_est_dot +  obj.prev_estimations(7:9) + obj.invM*tau;

            bracket_aux_sigma_est_dot = sign(aux).*(abs(aux).^(3*obj.alpha - 2));
            sigma_est_dot = -1/(obj.epsilon)*bracket_aux_sigma_est_dot;

            %% 5. Agrupación de variables de salida de la función
            obj.prev_estimations = [eta_est_dot;nu_est_dot;sigma_est_dot]* obj.Ts + obj.prev_estimations;
            % output = obj.prev_estimations;
        end
    end
end

