classdef SglosMpController < matlab.System
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
        k_psi
        N_p
        N_c
        tau_d

        a
        b
       
        u_ref_max         
        u_ref_min           
        u_tar_max           
        u_tar_min        
        r_ref_max         
        r_ref_min  

        Delta_u_ref_min 
        Delta_u_ref_max 
        Delta_u_tar_min  
        Delta_u_tar_max   
        Delta_r_ref_min 
        Delta_r_ref_max 

        path

        % === Wind (for crossflow drag) ===
        sL
        Loa
        AFw
        ALw

        af_al
        sl_loa

        const_tauY

        n_controls
        n_states
        n_disturbances
        f_model
        ff_pre
        Param

        T_s
        R_T

        epsilon

    end

    properties
        
        w_ant
        u_tar_next
        x_e
        y_e
        v_bar_i
        computation_time
        u_nc
        U_ant
        p_vec
        init
        sincronizador
        u_nc_l

        psi_d_ant

    end

    methods
        function obj = SglosMpController(vessel, params, Ts)

            obj = obj@matlab.System();

            obj.delta_sglos = params.delta_sglos;
            obj.k_u_tar = params.k_u_tar;
            obj.k_psi = params.k_psi;
            obj.N_p = params.N_p;
            obj.N_c = params.N_c;
            obj.tau_d = params.tau_d;

            obj.T_s = Ts;

            obj.a=(obj.tau_d*obj.T_s)/(obj.tau_d*obj.T_s+obj.T_s);
            obj.b=1/(obj.tau_d*obj.T_s+obj.T_s);

            obj.init = 0;

            obj.sL = vessel.lateralAreaCentroidY;
            obj.Loa  = vessel.totalLength;
            obj.AFw = vessel.frontalArea;
            obj.ALw = vessel.lateralArea;
            obj.af_al = obj.AFw/obj.ALw;
            obj.sl_loa = obj.sL/obj.Loa;

            obj.const_tauY = 0.5*1.1960*obj.ALw;

            obj.u_ref_max = 2.0;                            % Límite superior para la acción de control u_ref (m/s)
            obj.u_ref_min = 0.0;                              % Límite inferior para la acción de control u_ref (m/s)
            obj.u_tar_max = inf;                              % Límite superior para la acción de control u_tar (m/s)
            obj.u_tar_min = -inf;                           % Límite inferior para la acción de control u_tar (m/s)
            obj.r_ref_max = pi/8;                           % Límite superior para la acción de control r_ref (rad/s)
            obj.r_ref_min = -pi/8;                          % Límite inferior para la acción de control r_ref (rad/s)

            obj.Delta_u_ref_min = -0.05;                     % Límite inferior para los incrementos de acción de control u (m/s)
            obj.Delta_u_ref_max = 0.05;                      % Límite superior para los incrementos de acción de control u (m/s)
            obj.Delta_u_tar_min = -inf;                     % Límite inferior para los incrementos de acción de control u_tar (m/s)
            obj.Delta_u_tar_max = inf;                      % Límite superior para los incrementos de acción de control u_tar (m/s)
            obj.Delta_r_ref_min = obj.r_ref_min/1;                 % Límite inferior para los incrementos de acción de control r (rad/s)
            obj.Delta_r_ref_max = obj.r_ref_max/1;                  % Límite superior para los incrementos de acción de control r (rad/s)

            obj.R_T = Ts/params.t_sim;
            if obj.R_T < 1 || abs(obj.R_T - round(obj.R_T)) > 1e-10
                error('Error in mid-level control timing');
            end
            obj.R_T = round(obj.R_T);
            obj.sincronizador = obj.R_T;
            obj.computation_time = 0.0;
            
            % Ecuaciones del modelo completas:
            [obj.f_model, obj.n_controls, obj.n_states, obj.n_disturbances, obj.epsilon] = obj.Dinamica_modelada(vessel);

            % Modelo sobre el horizonte de prediccion:
            [obj.ff_pre, obj.Param] = obj.Modelo_predictivo(obj.f_model, obj.T_s);
            
            % n_states inicial | n_controls Controles anteriores | n_disturbances*NP | ud_min | r_hat | psi_d anterior
            obj.p_vec = zeros(obj.n_states + obj.n_controls + obj.N_p*obj.n_disturbances + 3,1);

            obj.u_nc = zeros(obj.N_c,obj.n_controls); 
            obj.u_nc(:,1) = ones(size(obj.u_nc,1),1);
            obj.u_nc(:,2) = zeros(size(obj.u_nc,1),1);
            obj.u_nc(:,3) = zeros(size(obj.u_nc,1),1);

            obj.U_ant = zeros(obj.n_controls,1);
            obj.psi_d_ant = 0.0;
            obj.w_ant = 0;
            obj.u_tar_next = 0;
            obj.x_e = 0;
            obj.y_e = 0;
            obj.v_bar_i = 0;
        end

        function value = getValues(obj)
            u_opt = reshape(obj.u_nc',obj.n_controls*obj.N_c,1);
            % 1 T_solucion | p_vec | n_controls*NC U_opt | 1 w | 1 x_e | 1 y_e | 1 v_bar 
            value = [ obj.computation_time; obj.p_vec; u_opt; obj.w_ant; obj.x_e; obj.y_e; obj.v_bar_i ];
        end
    end

    methods (Static)
        function xk_hat = get_mpc_hlc(m_hlc)
            no_vessels = length(m_hlc);
            len_pvec = m_hlc{1}.n_states + m_hlc{1}.n_controls + m_hlc{1}.N_p*m_hlc{1}.n_disturbances + 3;
            len_u_opt = m_hlc{1}.N_c*m_hlc{1}.n_controls;
            lent_total = (1+len_pvec+len_u_opt+4);
            xk_hat = zeros(lent_total*no_vessels,1);
            for v = 1:no_vessels
                xk_hat(1+(v-1)*lent_total:lent_total+(v-1)*lent_total) = m_hlc{v}.getValues();
            end
        end
    end

    methods (Access = private)

        function [f_model, n_controls, n_states, n_disturbances, epsilon] = Dinamica_modelada(obj, vessel)
            import casadi.*; 
            
            x_e_bar = SX.sym('x_e_bar');                    % Along-track error modificado (m)
            y_e_bar = SX.sym('y_e_bar');                    % Cross-track error modificado (m)
            psi = SX.sym('psi');                            % Orientación respecto al Norte (rad)
            w = SX.sym('w');                                % Variable del punto objetivo sobre la trayectoria (p.u.)
            v_r_bar = SX.sym('v_r_bar');                    % Velocidad de sway modificada (m/s)
            
            states = [x_e_bar;y_e_bar;psi;w;v_r_bar]; 
            n_states = length(states);

            u_ref = SX.sym('u_ref');                        % Referencia para la velocidad de surge (m/s)
            u_tar = SX.sym('u_tar');                        % Velocidad de avance del punto objetivo sobre la trayectoria (m/s)
            r_ref = SX.sym('r_ref');                        % Referencia para la velocidad angular yaw (rad/s)
            psi_d = SX.sym('psi_d');                        % Referencia para la velocidad angular yaw (rad/s)
            psi_d_dot = SX.sym('psi_d_dot');                        % Referencia para la velocidad angular yaw (rad/s)
            
            controls = [u_ref;u_tar;r_ref;psi_d;psi_d_dot];
            n_controls = length(controls);

            V_w = SX.sym('V_w');              	    % Velocidad del viento (m/s)
            beta_w = SX.sym('beta_w');              % Dirección estándar del viento (rad)
            V_c = SX.sym('V_c');                    % Velocidad de la corriente (m/s)
            beta_c = SX.sym('beta_c');              % Dirección de la corriente (rad)
            V_c_dot = SX.sym('V_c_dot');            % Derivada temporal de la velocidad de la corriente (m/s2)
            beta_c_dot = SX.sym('beta_c_dot');      % Derivada temporal de la dirección de la corriente (rad/s)

            disturbances = [V_w;beta_w;V_c;beta_c;V_c_dot;beta_c_dot];
            n_disturbances = length(disturbances);

            if isequal(size(vessel.M), [6 6])
                vessel.M = [ ...
                    vessel.M(1,1) vessel.M(1,2) vessel.M(1,6); ...
                    vessel.M(2,1) vessel.M(2,2) vessel.M(2,6); ...
                    vessel.M(6,1) vessel.M(6,2) vessel.M(6,6)  ...
                ];
            end

            epsilon = vessel.M(2,3)/vessel.M(2,2);

            [spatial_path_CasADI, obj.path] = pathScriptCasADI(w);

            dx_p_dw = spatial_path_CasADI.dx_p_dw;          % Primera derivada de la posición Norte del punto objetivo sobre la trayectoria respecto a la variable del punto objetivo (m)
            dy_p_dw = spatial_path_CasADI.dy_p_dw;          % Primera derivada de la posición Este del punto objetivo sobre la trayectoria respecto a la variable del punto objetivo (m)
            dx_p_dw2 = spatial_path_CasADI.dx_p_dw2;        % Segunda derivada de la posición Norte del punto objetivo sobre la trayectoria respecto a la variable del punto objetivo (m)
            dy_p_dw2 = spatial_path_CasADI.dy_p_dw2;        % Segunda derivada de la posición Este del punto objetivo sobre la trayectoria respecto a la variable del punto objetivo (m)
            phi_p = spatial_path_CasADI.phi_p;              % Ángulo de la trayectoria respecto al Norte en el punto objetivo (rad)
            % tan_phi_p = dy_p_dw/dx_p_dw;                    % Tangente del ángulo de la trayectoria respecto al Norte en el punto objetivo (p.u.)
            F = sqrt(dx_p_dw^2 + dy_p_dw^2);                % Variable de curvatura de la trayectoria en el punto objetivo (m)
            % dphi_p_dw = 1/(1+tan_phi_p^2)*(dy_p_dw2*dx_p_dw - dx_p_dw2*dy_p_dw)/(dx_p_dw^2);        % Derivada del ángulo de la trayectoria respecto al Norte en el punto objetivo respecto a la variable del punto objetivo sobre la trayectoria (rad)
            dphi_p_dw = (dy_p_dw2*dx_p_dw - dx_p_dw2*dy_p_dw)/(dx_p_dw^2+dy_p_dw^2);

            V_c_vec = [V_c*cos(beta_c) V_c*sin(beta_c) 0]';
            
            ROT = [cos(psi),    - sin(psi),      0;
                   sin(psi),      cos(psi),      0;
                          0,             0,      1];
                      
            nu_c = ROT'*V_c_vec;

            u_r_ref = u_ref - nu_c(1);

            % 4.1. Cinemática del ASV:
            kinematics_ASV = [u_r_ref*cos(psi - phi_p) - v_r_bar*sin(psi - phi_p) + V_c*cos(beta_c - phi_p) + u_tar*(1/F*dphi_p_dw*y_e_bar - 1); ...
                              u_r_ref*sin(psi - phi_p) + v_r_bar*cos(psi - phi_p) + V_c*sin(beta_c - phi_p) - u_tar*1/F*dphi_p_dw*x_e_bar; ...
                              r_ref]; 

            kinematics_path = u_tar/F;
  
            v_r = v_r_bar - epsilon*r_ref;

            D = [-vessel.X_u-vessel.X_u_abs_u*abs(u_r_ref),                 0,                                      0;
                0,  -vessel.Y_v-vessel.Y_v_abs_v*abs(v_r)-vessel.Y_v_abs_r*abs(r_ref), -vessel.Y_r-vessel.Y_r_abs_v*abs(v_r)-vessel.Y_r_abs_r*abs(r_ref);
                0,  -vessel.N_v-vessel.N_v_abs_v*abs(v_r)-vessel.N_v_abs_r*abs(r_ref), -vessel.N_r-vessel.N_r_abs_v*abs(v_r)-vessel.N_r_abs_r*abs(r_ref)];
            
            alpha = vessel.M(1,1)/vessel.M(2,2);
            beta = D(2,2)/vessel.M(2,2);
            gamma = (epsilon)*( (D(2,2)/vessel.M(2,2)) - (D(2,3)/vessel.M(3,2)));
            delta = 1/vessel.M(2,2);

            % gamma = 1/vessel.M(2,2)*(epsilon*D(2,2) - D(2,3));
            
            % 5.2.2. Viento:
            v = v_r + nu_c(2);
           
            u_w = V_w*cos(beta_w - psi);
            v_w = V_w*sin(beta_w - psi);
            
            u_r_w = u_ref - u_w;
            v_r_w = v - v_w;
            
            V_r_w = sqrt(u_r_w^2 + v_r_w^2);
            gamma_r_w = - atan2(v_r_w,u_r_w);

            CD_l_AF = if_else(abs(gamma_r_w) <= pi/2, 0.55, 0.65);

            % wind coefficients
            CD_l = CD_l_AF*obj.af_al;
            den = 1-0.5*0.60*(1-CD_l/0.85)*sin(2*gamma_r_w)^2;

            CY =  0.85*sin(gamma_r_w)/den;

            tau_w_v = obj.const_tauY * CY*V_r_w^2;

            dot_v_r_bar = - alpha*u_r_ref*r_ref - beta*v_r_bar + gamma*r_ref + delta*tau_w_v;

            model = [kinematics_ASV;kinematics_path;dot_v_r_bar];
            
            f_model = Function('f',{states,controls,disturbances},{model});
        end

        function [ff_pred, Param] = Modelo_predictivo(obj, f_model, Ts)
            import casadi.*; 
            X_HL = SX.sym('X_HL',obj.n_states,(obj.N_p+1));% Estados actuales y a lo largo del horizonte
            U_HL = SX.sym('U_HL',obj.n_controls,obj.N_c); % Acciones de control a lo largo del horizonte

            % 6. Vector Parametros
            % - El estado inicial P_HL(1:obj.n_states)
            % - Los controles implementados en el instante anterior P_HL(obj.n_states+1:obj.n_states+obj.n_controls)
            % - Las perturbaciones estimadas a lo largo del horizonte P_HL(obj.n_states+obj.n_controls+1:obj.n_states+obj.n_controls+obj.N_p*obj.n_disturbances)
            % - ud_min, r_hat y psi_d del instante anterior P_HL(obj.n_states+obj.n_controls+obj.N_p*obj.n_disturbances+1:obj.n_states+obj.n_controls+obj.N_p*obj.n_disturbances+2)
            
            Param = SX.sym('P_HL',obj.n_states + obj.n_controls + obj.N_p*obj.n_disturbances + 3);
            
            % 7. Solución simbólica:

            idx_dis0 = obj.n_states + obj.n_controls; 

            X_HL(:,1) = Param(1:obj.n_states);            % Estado inicial
            ud_min = Param(obj.n_states+obj.n_controls+obj.N_p*obj.n_disturbances+1);                     % Velocida minima deciada (m)
            r_act = Param(obj.n_states+obj.n_controls+obj.N_p*obj.n_disturbances+2);                 % Velocida minima deciada (m)
            % psi_d_prev = Param(obj.n_states+obj.n_controls+obj.N_p*obj.n_disturbances+3);                 % Velocida minima deciada (m)
            con = Param(obj.n_states+1:obj.n_states+obj.n_controls);                                      % Acciones del estado anterior

            for k = 1:obj.N_p
                st       = X_HL(:,k);                   % Estados
                dis = Param(idx_dis0 + (k-1)*obj.n_disturbances + 1: idx_dis0 + k*obj.n_disturbances); %disturbances

                % states = [x_e_bar;y_e_bar;psi;w;v_r_bar]; 

                [spatial_path_CasADI,] = pathScriptCasADI(st(4));
                phi_p = spatial_path_CasADI.phi_p;
                ROT = [cos(st(3)) -sin(st(3)) 0;sin(st(3)) cos(st(3)) 0; 0 0 1];
                nu_c = ROT'*[dis(3)*cos(dis(4)) dis(3)*sin(dis(4)) 0]';
                % zt = [x_e;y_e;psi;w;v]; 
                zt = [st(1) - obj.epsilon*cos(st(3)-phi_p); ...
                      st(2) - obj.epsilon*sin(st(3)-phi_p); ...
                      st(3); ...
                      st(4); ...
                      ((st(5) + nu_c(2)) - obj.epsilon*r_act)];

                if k <= obj.N_c
                    k1 = ud_min/obj.delta_sglos;
        
                    u_ref = k1*sqrt(zt(2)^2 + obj.delta_sglos^2);

                    beta_d = -atan2(zt(5),u_ref);
    
                    U_d = sqrt(u_ref^2 + zt(5)^2);
                    u_tar = obj.k_u_tar*zt(1) + U_d*cos(zt(3) - phi_p + beta_d); 
                    
                    psi_d = phi_p - beta_d - atan(zt(2)/obj.delta_sglos);
                    psi_dot_ref = (obj.a * con(5)) + obj.b * atan2(sin(psi_d - con(4)), cos(psi_d - con(4)));
    
                    r_ref = psi_dot_ref - obj.k_psi * atan2(sin(zt(3) - psi_d ), cos(zt(3) - psi_d ));
                    % r_ref = fmin(obj.r_ref_max, fmax(obj.r_ref_min, r_ref));
                    % dr_ref = r_ref - con(3);
                    % dr_ref = fmin(obj.Delta_r_ref_max, fmax(obj.Delta_r_ref_min, dr_ref));
                    % r_ref  = con(3) + dr_ref;

                    % u_tar = fmin(obj.u_tar_max, fmax(obj.u_tar_min, u_tar));
                    % du_tar = u_tar - con(2);
                    % du_tar = fmin(obj.Delta_u_tar_max, fmax(obj.Delta_u_tar_min, du_tar));
                    % u_tar  = con(2) + du_tar;

                    % u_ref = fmin(obj.u_ref_max, fmax(obj.u_ref_min, u_ref));
                    % 
                    % du_ref = u_ref - con(1);
                    % du_ref = fmin(obj.Delta_u_ref_max, fmax(obj.Delta_u_ref_min, du_ref));
                    % u_ref  = con(1) + du_ref;
                    
                    con = [u_ref;u_tar;r_ref;psi_d;psi_dot_ref];
                    r_act = r_ref;
                end

                U_HL(:,k) = con;

                f_value  = f_model(st,con,dis);          % Ecuaciones del modelo continuo
                st_next  = st + Ts*f_value;          % Ecuaciones del modelo discretizado
                X_HL(:,k+1) = st_next;                  % Shift temporal
            end
            
            % 8. Función para obtener la trayectoria óptima de los estados sabiendo la solución óptima:
            
            ff_pred = Function('ff',{Param},{X_HL, U_HL});
        end

        function [xe, ye, v_r_bar] =  get_inicial_values(obj, x, y, psi, v, r, Vc, Beta_c)
            obj.evolve_w(obj.u_tar_next);
            x_p = obj.path.x_p(obj.w_ant);
            y_p = obj.path.y_p(obj.w_ant);
            phi_p = obj.path.phi_p(obj.w_ant);
            R2 = [cos(phi_p) -sin(phi_p);
                  sin(phi_p) cos(phi_p)];
            aux = R2'*[x + obj.epsilon*cos(psi) - x_p;
                       y + obj.epsilon*sin(psi) - y_p];

            cen = R2'*[x - x_p;
                       y - y_p];

            ROT = [cos(psi) -sin(psi) 0;
                   sin(psi) cos(psi) 0;
                   0 0 1];
            nu_c = ROT'*[Vc*cos(Beta_c) Vc*sin(Beta_c) 0]';

            xe = aux(1); 
            ye = aux(2);
            v_bar = v + obj.epsilon*r;
            v_r_bar = v_bar - nu_c(2);

            obj.x_e = cen(1);
            obj.y_e = cen(2);
            obj.v_bar_i = v_bar;
        end

        function evolve_w(obj, u_tar)
            dx_p_dw = obj.path.dx_p_dw(obj.w_ant);          % Primera derivada de la posición Norte del punto objetivo sobre la trayectoria respecto a la variable del punto objetivo (m)
            dy_p_dw = obj.path.dy_p_dw(obj.w_ant);          % Primera derivada de la posición Este del punto objetivo sobre la trayectoria respecto a la variable del punto objetivo (m)
            F = sqrt(dx_p_dw^2 + dy_p_dw^2);                % Variable de curvatura de la trayectoria en el punto objetivo (m)
            w_dot = u_tar/F;
            obj.w_ant = obj.w_ant + obj.T_s*w_dot;
        end
    end


    methods (Access = protected)

        function refs = stepImpl(obj, ud_min, estimation, disturbances_mod)
            import casadi.*; 
            if obj.sincronizador == obj.R_T
                x_hat = estimation(1);
                y_hat = estimation(2);
                psi_hat = estimation(3);
                v_hat = estimation(5);
                r_hat = estimation(6);

                V_w_0_mod = disturbances_mod.V_w;
                beta_w_0_mod = disturbances_mod.beta_w;
                V_c_0_mod = disturbances_mod.V_c;
                beta_c_0_mod = disturbances_mod.beta_c;
                V_c_dot_0_mod = disturbances_mod.V_c_dot;
                beta_c_dot_0_mod = disturbances_mod.beta_c_dot;
 
                [xe_bar, ye_bar, v_r_bar] = obj.get_inicial_values(x_hat, y_hat, psi_hat, v_hat, r_hat, V_c_0_mod, beta_c_0_mod);
    
                p_vector = zeros(obj.n_states + obj.n_controls + obj.N_p*obj.n_disturbances + 2,1);
    
                p_vector(1:obj.n_states,1) = [xe_bar ye_bar psi_hat, obj.w_ant, v_r_bar];

                p_vector(obj.n_states +1:obj.n_states+obj.n_controls,1) = obj.U_ant;
    
                idx_dis0 = obj.n_states + obj.n_controls; 
            
                for k=1:obj.N_p
                    p_vector(idx_dis0 + (k-1)*obj.n_disturbances+1: idx_dis0 + k*obj.n_disturbances,1) = [V_w_0_mod beta_w_0_mod V_c_0_mod beta_c_0_mod V_c_dot_0_mod beta_c_dot_0_mod]';
                end

                p_vector(obj.n_states + obj.n_controls + obj.N_p*obj.n_disturbances + 1,1) = ud_min;

                p_vector(obj.n_states + obj.n_controls + obj.N_p*obj.n_disturbances + 2,1) = r_hat;

                p_vector(obj.n_states + obj.n_controls + obj.N_p*obj.n_disturbances + 3,1) = obj.psi_d_ant;

                obj.p_vec = p_vector;
                
                tic
                [x_pre, u_pre] = obj.ff_pre(p_vector);
                obj.computation_time = toc;
                x_pre = full(x_pre)';
                obj.u_nc = full(u_pre)';

                obj.u_tar_next= obj.u_nc(1,2);
                obj.U_ant = obj.u_nc(1,:);

                % obj.u_nc_l = [obj.u_nc(:,1) x_pre(2:end,3) obj.u_nc(:,3)];

                obj.u_nc_l = [obj.u_nc(:,1) obj.u_nc(:,4) obj.u_nc(:,3)];
                
                %Ajustar salida a la frecunecia del simulador
                u_nc_llc = kron(obj.u_nc_l(:,1:3), ones(obj.R_T, 1));
    
                % Salida
                refs= reshape(u_nc_llc.', [], 1);
                obj.sincronizador = 1;

            else
                u_nc_llc = kron(obj.u_nc_l(:,1:3), ones(obj.R_T, 1));
                shift = 3 * obj.sincronizador;
                % Salida
                ref_last= reshape(u_nc_llc.', [], 1);
                refs = [ref_last(shift+1:end); repmat(ref_last(end), shift, 1)];
                obj.sincronizador = obj.sincronizador + 1;
                % refs= reshape(obj.u_nc_l.', [], 1);
            end
        end
    end
end
