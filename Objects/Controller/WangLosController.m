classdef WangLosController < matlab.System
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Autor: Manuel Eduardo Gantiva Osorio
    % Proyecto: AQUATRONIC
    % Escuela Técnica Superior de Ingeniería 
    % Universidad Loyola Andalucía
    % Fecha: 24.04.2025
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % This class implements a path-following controller for each vessel, based on the
    % approach proposed by Wang (2018). It receives the desired speed as input and calculates
    % the corresponding velocity and heading references required for accurate trajectory
    % tracking.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    properties (Nontunable)
        delta_sglos
        k_u_tar
        T_s

        a
        b
        u_max
        r_max

        path
    end

    properties(Access = private)
        memR
        w
        x_e
        y_e
        ref
        init
        psi_ant
        laps
    end

    methods
        function obj = WangLosController(k_vector, Ts, tau_d, path)

            obj = obj@matlab.System();

            obj.delta_sglos = k_vector(1);
            obj.k_u_tar = k_vector(2);
            obj.T_s = Ts;
            obj.path = path;

            obj.a=(tau_d*Ts)/(tau_d*Ts+Ts);
            obj.b=1/(tau_d*Ts+Ts);

            obj.memR = [0; 0];

            obj.w = 0;
            obj.x_e = 0;
            obj.y_e = 0;

            obj.ref = zeros(3,1);
            obj.init = 0;
            obj.psi_ant = 0;
            obj.laps = 0;
        end

        function value = getValuesMlc(obj)
            value = [ obj.ref; obj.x_e; obj.y_e; obj.w];
        end
    end

    methods (Static)
      function xk_hat = get_mlc(m_mlc)
            no_vessels = length(m_mlc);
            xk_hat = zeros(6*no_vessels,1);
            for v = 1:no_vessels
                xk_hat(1+(v-1)*6:6+(v-1)*6) = m_mlc{v}.getValuesMlc();
            end
      end
   end


    methods (Access = protected)

        function refs = stepImpl(obj, ud_min, estimation)
            x_hat = estimation(1);
            y_hat = estimation(2);
            psi_hat = estimation(3);
            v_hat = estimation(5);
            
            x_p = obj.path.x_p(obj.w);
            y_p = obj.path.y_p(obj.w);
            
            dx_p_dw = obj.path.dx_p_dw(obj.w);
            dy_p_dw = obj.path.dy_p_dw(obj.w);

            phi_p = atan2(dy_p_dw,dx_p_dw);
            
            obj.x_e =   (x_hat - x_p)*cos(phi_p) + (y_hat - y_p)*sin(phi_p);
            obj.y_e = - (x_hat - x_p)*sin(phi_p) + (y_hat - y_p)*cos(phi_p);
            
            k1 = ud_min/obj.delta_sglos;

            u_ref = k1*sqrt(obj.y_e^2 + obj.delta_sglos^2);
            
            beta_d = -atan2(v_hat,u_ref);
            
            psi_d = phi_p - beta_d - atan(obj.y_e/obj.delta_sglos);
            
            U_d = sqrt(u_ref^2 + v_hat^2);
            
            u_tar = obj.k_u_tar*obj.x_e + U_d*cos(psi_hat - phi_p + beta_d);
            
            w_dot = u_tar/(sqrt(dx_p_dw^2 + dy_p_dw^2));

            obj.w = w_dot*obj.T_s + obj.w; 

            % Aplicar el filtro derivativo avanzado para cada señal
            if(obj.init == 0)
                obj.memR = [0; psi_hat];
                obj.init = 1;
                obj.psi_ant = psi_d;
            end
            
            if (psi_d - obj.psi_ant) > pi
                obj.laps = obj.laps - 1;
            elseif (psi_d - obj.psi_ant) < -pi
                obj.laps = obj.laps + 1;
            end
            obj.psi_ant = psi_d;
            psi_d = psi_d + 2 * pi * obj.laps;

            r_ref = (obj.a * obj.memR(1)) + obj.b * (psi_d - obj.memR(2));
            obj.memR = [r_ref; psi_d];

            if(u_ref>2.0)
                u_ref=2.0;
            end

            c_ref = r_ref - 0.1 * atan2(sin(psi_hat - psi_d ), cos(psi_hat - psi_d ));

            refs= [u_ref; psi_d; c_ref];
            obj.ref = refs;

        end
    end
end
