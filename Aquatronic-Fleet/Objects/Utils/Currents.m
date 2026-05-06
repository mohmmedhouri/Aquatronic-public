classdef Currents < matlab.System
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Autor: Manuel Eduardo Gantiva Osorio
    %        Federico Peralta Samaniego
    % Proyecto: AQUATRONIC
    % Escuela Técnica Superior de Ingeniería 
    % Universidad Loyola Andalucía
    % Fecha: 24.04.2025
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % This class models the behaviour of ocean currents and can be used as an external
    % disturbance affecting any of the vessels. It provides a realistic representation
    % of environmental conditions for simulation and control testing
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    properties
                beta_c
                V_c
                
                saved_state_variables
                enable_Vc
    end
    
    methods
        function obj = Currents(beta_c, V_c, enable_Vc)
            %OCEANCURRENTS Construct an instance of this class
            %   Detailed explanation goes here

            obj = obj@matlab.System();

            obj.beta_c = beta_c;
            obj.V_c = V_c;
%             obj.saved_state_variables = zeros(6, 1);
            obj.saved_state_variables = zeros(2, 1);
            obj.enable_Vc = enable_Vc;
        end

        function update_discrete_states(obj, state_variables)
            obj.saved_state_variables = state_variables;
        end
    end

    methods (Access = protected)
        
        function continuous_Vc = stepImpl(obj)
            if obj.enable_Vc ~= 0
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            x1 = obj.saved_state_variables(1);
%             x2 = state_variables(2);
%             x3 = state_variables(3);
            y1 = obj.saved_state_variables(2);
%             y1 = state_variables(4);
%             y2 = state_variables(5);
%             y3 = state_variables(6);

            % Aqui falta un update de V_c y beta_c
            u1 = obj.V_c * cos(obj.beta_c); % unfiltered_continuous_Vc_x
            u2 = obj.V_c * sin(obj.beta_c); % unfiltered_continuous_Vc_y

            x1_dot = -0.1*x1 + 0.25*u1;
            vc_x = 0.4*x1;
%             x2_dot = -x2 + vc_x;
%             vc_x_dot = -x2 + vc_x;
%             x3_dot = -x3 + vc_x_dot;
%             vc_x_dot2 = -x3 + vc_x_dot;

            y1_dot = -0.1*y1 + 0.25*u2;
            vc_y = 0.4*y1;
%             y2_dot = -y2 + vc_y;
%             vc_y_dot = -y2 + vc_y;
%             y3_dot = -y3 + vc_y_dot;
%             vc_y_dot2 = -y3 + vc_y_dot;
            
%             continuous_Vc = [vc_x; vc_y; 0;...
%                 vc_x_dot; vc_y_dot; 0; ...
%                 vc_x_dot2; vc_y_dot2; 0; ...
%                 x1_dot; x2_dot; x3_dot; ...
%                 y1_dot; y2_dot; y3_dot];

            continuous_Vc = [u1; u2; 0;...
                x1_dot;...
                y1_dot];
            else
            continuous_Vc = [0; 0; 0;...
                0;...
                0];
            end

        end
    end
end

