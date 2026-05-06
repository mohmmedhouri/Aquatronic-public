classdef Asvkic < ASVbase
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Autor: Manuel Eduardo Gantiva Osorio
    % Proyecto: AQUATRONIC
    % Escuela Técnica Superior de Ingeniería 
    % Universidad Loyola Andalucía
    % Fecha: 24.04.2025
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Asvkic is a subclass of ASVbase that implements a purely kinematic model of the
    % vessel. It excludes dynamic effects, focusing solely on the vessel's motion based on
    % its velocity and heading.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Public, tunable properties
    properties(Nontunable)
       
        %% 1.1. Límites en las acciones de control:
        maxU 
        maxR 
        T_s
        
    end

   
    methods
        function obj = Asvkic(vessel, X_0, observer_param)
            
            % Añadir informacion nesesaria para el observador
            observer_param.invM = vessel.invM;
            observer_param.X_0 = [X_0(1:2);X_0(6:8);X_0(12)];

            obj  = obj@ASVbase("ASVKic", X_0, observer_param); 
           

            obj.maxU  = vessel.maxU;
            obj.maxR = vessel.maxR;
            obj.T_s = vessel.T_s;
        end

    end

    methods(Access = public)

        function setVel(obj, nu_apli)
            
            if ~isequal(size(nu_apli), [3, 1])
                error('El vector nu_apli debe ser de 3x1');
            end

            if(nu_apli(1)>obj.maxU)
                nu_apli(1) = obj.maxU;
            end
            if(nu_apli(1)<0)
                nu_apli(1) = 0;
            end
            if(nu_apli(3)>obj.maxR)
                nu_apli(3) = obj.maxR;
            end
            if(nu_apli(3)<-obj.maxR)
                nu_apli(3) = -obj.maxR;
            end
            obj.nu_apli = nu_apli;        
        end

    end

    methods (Access = protected)

        function xdot = stepImpl(obj, X_r,Vc_params)
            % Dinámica del sistema
            % x = [ x y z phi theta psi u v w p q r]' 

            % === Extracción de corrientes en Magnitud y angulo ===
            V_c = sqrt(Vc_params(1)^2 + Vc_params(2)^2);
            beta_c = atan2(Vc_params(2), Vc_params(1));

            eta = X_r(1:6);
            nu = X_r(7:12);
            nu2 = nu(4:6);

            u_c = V_c * cos(beta_c - eta(6));
            v_c = V_c * sin(beta_c - eta(6));
            nu_c = [u_c; v_c; 0; 0; 0; 0];

            J = eulerang(eta(4), eta(5), eta(6));
            % === Extracción de estados ===
            nu_next = [obj.nu_apli(1:2); zeros(3,1); obj.nu_apli(3)];

            nu_c_dot = [-Smtrx(nu2)*nu_c(1:3); zeros(3,1)]; % zeros(6,1)
             
            nu_dot = ((nu_next - nu)/obj.T_s) + nu_c_dot;

            obj.state(7:8) = nu_dot(1:2); % Actualizo las acceleraciones

            % === Ecuaciones de movimiento ===
            xdot = [J * nu ;
                    nu_dot];
        end
    end
end
