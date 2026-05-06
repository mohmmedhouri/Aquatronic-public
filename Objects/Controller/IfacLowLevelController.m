classdef IfacLowLevelController < matlab.System
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Autor: Manuel Eduardo Gantiva Osorio
    % Proyecto: AQUATRONIC
    % Escuela Técnica Superior de Ingeniería 
    % Universidad Loyola Andalucía
    % Fecha: 24.04.2025
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % This class implements the low-level controller proposed by Sufiyan and Bejarano (2022).
    % It takes reference velocities as input and computes the corresponding control actions
    % required to drive the vessel, focusing on accurate tracking at the actuation level.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    properties (Nontunable)
        g_r_dot
        g_u_dot
        k_u
        k_psi
        k_r
        T_s
        a
        b

        Fu_max
        Taor_max
    end

    properties
        memU
        memR
    end

    methods
        function obj = IfacLowLevelController(vessel,k_vector, Ts, tau_d)

            obj = obj@matlab.System();
            invM = vessel.invM;
            %
            obj.g_u_dot = invM(1,1);
            obj.g_r_dot = invM(3,3);

            obj.k_u = k_vector(1);
            obj.k_r = k_vector(2);
            obj.k_psi = k_vector(3);
            obj.T_s = Ts;

            obj.a=(tau_d*Ts)/(tau_d*Ts+Ts);
            obj.b=1/(tau_d*Ts+Ts);

            obj.memU = [0; 0];
            obj.memR = [0; 0];

            obj.Fu_max = vessel.maxForce_u;
            obj.Taor_max = vessel.maxTorque_r;

        end
    end


    methods (Access = protected)

        function tau = stepImpl(obj, ref, estimation)
            u_ref = ref(1);
            r_ref = ref(3);
            psi_ref = ref(2);

            u_hat = estimation(4);
            r_hat = estimation(6);
            psi_hat = estimation(3);
            su_hat = estimation(7);
            sr_hat = estimation(9);

            % Aplicar el filtro derivativo avanzado para cada señal
            u_ref_dot = (obj.a * obj.memU(1)) + obj.b * (u_ref - obj.memU(2));
            r_ref_dot = (obj.a * obj.memR(1)) + obj.b * (r_ref - obj.memR(2));
            
            % Actualizar la memoria con la salida actual y la entrada actual
            obj.memU = [u_ref_dot; u_ref];
            obj.memR = [r_ref_dot; r_ref];

            c_ref = r_ref - obj.k_psi*(psi_hat-psi_ref);

            Fu = (1/obj.g_u_dot) * (u_ref_dot-su_hat-obj.k_u*(u_hat-u_ref));
            tau = (1/obj.g_r_dot) * (r_ref_dot-sr_hat-obj.k_r*(r_hat-c_ref)-obj.k_psi*(r_hat-r_ref));


            Fu_sat = saturation_control_GB(Fu,obj.Fu_max,0);
            tau_sat = saturation_control_GB(tau,obj.Taor_max,-obj.Taor_max);
        
            %% Salida
            tau = [Fu_sat; 0; tau_sat];
        end
    end
end
