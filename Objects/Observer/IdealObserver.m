classdef IdealObserver < matlab.System
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Autor: Manuel Eduardo Gantiva Osorio
    % Proyecto: AQUATRONIC
    % Escuela Técnica Superior de Ingeniería 
    % Universidad Loyola Andalucía
    % Fecha: 24.04.2025
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % This class implements an ideal state observer, which provides perfect state estimates
    % without any error. It is primarily used for testing purposes to eliminate uncertainty
    % and validate other components under ideal conditions.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    properties (Access=private)
        prev_estimations        %Estado Estimados del barco state = [ x y psi u v r sigma_u sigma_v sigma_r]' 
    end

    properties (Nontunable)
        type 
    end
    
    methods
        function obj = IdealObserver(observer_param)

            obj = obj@matlab.System();

            obj.type = "Ideal";
            obj.prev_estimations = [observer_param.X_0;zeros(3,1)]; %Posiciones, velocidades y perturbaciones

        end

        function states_hat = getEstimations(obj)
            states_hat = obj.prev_estimations;
        end

        function setEstimations(obj, real_states)
            obj.prev_estimations = real_states;
        end


    end

    methods(Access = protected)
        function stepImpl(obj, eta_noise, tau_n)
            
            obj.prev_estimations = zeros(9,1);
            % output = obj.prev_estimations;
        end
    end

end

