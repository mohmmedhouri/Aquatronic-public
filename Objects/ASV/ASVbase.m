classdef (Abstract) ASVbase < matlab.System
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Autor: Manuel Eduardo Gantiva Osorio
    % Proyecto: AQUATRONIC
    % Escuela Técnica Superior de Ingeniería 
    % Universidad Loyola Andalucía
    % Fecha: 24.04.2025
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % ASVbase is an abstract class that consolidates common attributes and methods shared
    % across different vessel models. It serves as a base structure to promote code
    % reusability and consistency in the implementation of various ship types.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

     properties(Nontunable)
       model
       observer_obj
    end
    
    properties (Access = protected)
        Tau                % Vector de fuerzas/momentos (3x1)
        Deltas           % Valores de propulsión (2x1)
        sig                % Señales internas (dimensión variable según modelo)
        nu_apli            % Velocidad aplicada (dimensión variable según modelo)

        state               %Estado actual del barco state = [ x y psi u v r u_dot v_dot]' 
    end
    
    methods
        function obj = ASVbase(model, X_0, observer_param)

            % Constructor de la clase base
            obj = obj@matlab.System();

            obj.model = model;

            obj.Tau = [X_0(21); 0; X_0(22)]; 
            obj.Deltas = [0 0]';            % n = [ n_left n_right ]'
            obj.sig = zeros(3,1);
            obj.nu_apli = zeros(3,1);  
            obj.state = [X_0(1:2); X_0(6:8); X_0(12); zeros(2,1)];              %Estado Inicial del barco state = [ x y psi u v r u_dot v_dot]'

            switch observer_param.obs
                case 0
                    obj.observer_obj = IdealObserver(observer_param);
                case 1
                    obj.observer_obj = LiuConvergeObs(observer_param);
                case 2
                    obj.observer_obj = SufiyanObserver(observer_param);
                case 3
                    obj.observer_obj = ZonotopicObserver(observer_param);
            end

        end

        function Estimate(obj, noise)
            measures =  obj.getMeasurements(noise); %[ x y psi r u_dot v_dot]'
            actions = obj.getActions();
            obj.observer_obj.step(measures(1:4),actions(3:4));
        end
        

        function act = getActions(obj)
            % Obtiene las acciones: estructura común
            act = [obj.Deltas(1); obj.Deltas(2); obj.Tau(1); obj.Tau(3)];
        end

        function sig = getSigmas(obj)
            sig = [obj.sig(1:2); zeros(3,1); obj.sig(3)];
        end

        function med = getMeasurements(obj, noise)
            if ~isequal(size(noise), [4, 1])
                error('El vector noise debe ser de 4x1');
            end
            noise = [noise; zeros(2,1)];   % Sin ruido en las Acceleraciones
            %Estado actual del barco state = [ x y psi u v r u_dot v_dot]'

            med = [obj.state(1:3)+noise(1:3);obj.state(6)+noise(4);obj.state(7:8)+noise(5:6)];
            % disp(med(4));
        end

        function setStates(obj, states)
            if ~isequal(size(states), [6, 1])
                error('El vector state debe ser de 4x1');
            end
            %Estado actual del barco state = [ x y psi u v r]'

            obj.state(1:6) = states;
            if(obj.observer_obj.type == "Ideal")
                obj.observer_obj.setEstimations([obj.state(1:6); obj.sig(1:2); obj.sig(3)])
            end

        end

        % Métodos por defecto que pueden ser sobrescritos en las subclases:
        
        function setThrust(obj, Deltas)
            warning('Esta clase no implementa el método "setThrust". Considere sobreescribir este método en la subclase.');
            obj.Deltas = [0 0]';            % n = [ n_left n_right ]'
        end
        
        function setTao(obj, tau)
            warning('Esta clase no implementa el método "setTao". Considere sobreescribir este método en la subclase.');
            obj.Tau = [0 0 0]';
        end
        
        function setVel(obj, nu_apli)
            warning('Esta clase no implementa el método "setVel". Considere sobreescribir este método en la subclase.');
            obj.nu_apli = [0 0 0]';
        end
    end
    
    % Se puede mantener abstracto el método stepImpl ya que es específico de cada modelo.
    methods (Abstract, Access = protected)
        xdot = stepImpl(obj, input)
    end
end