classdef MpcLlcDisController < matlab.System
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Autor: Manuel Eduardo Gantiva Osorio
    %         Guillermo Bejarano
    % Proyecto: AQUATRONIC
    % Escuela Técnica Superior de Ingeniería 
    % Universidad Loyola Andalucía
    % Fecha: 5.04.2025
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % This class implements the low-level MPC.
    % It takes reference velocities as input and computes the corresponding control actions
    % required to drive the vessel, focusing on accurate tracking at the actuation level.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    properties (Nontunable)
        N_p
        N_c
        Q
        R
        Nu
        n_controls
        n_states
        n_disturbances
        f_model
        ff_opt
        Param
        obj_fc
        solver
        args
        
        U0_LL
        

        Ts_LL
        

        u_max
        u_min
        v_max
        v_min
        r_max
        r_min
        Delta_F_min
        Delta_F_max
        Delta_T_min
        Delta_T_max
        % Fu_max
        % Taor_max
        F_max
        F_min
        d

        % === Wind (for crossflow drag) ===
        sL
        Loa
        AFw
        ALw

        af_al
        sl_loa

        const_tauX
        const_tauY
        const_tauN

    end

    properties 
        computation_time
        success
        u_nc
        U_ant
        j_min
        p_vec
        n_ite
    end

    methods
        function obj = MpcLlcDisController(vessel, params, Ts)
            import casadi.*; 
            obj = obj@matlab.System();
            obj.N_p = params.N_p;
            obj.N_c = params.N_c;
            obj.Q = params.Q;
            obj.R = params.R;
            obj.Nu = params.Nu;
            
            obj.u_max = params.u_max;
            obj.u_min = params.u_min;
            obj.v_max = params.v_max;
            obj.v_min = params.v_min;
            obj.r_max = params.r_max;
            obj.r_min = params.r_min;
            
            obj.Delta_F_min = params.Delta_F_min;
            obj.Delta_F_max = params.Delta_F_max;
            obj.Delta_T_min = params.Delta_T_min;
            obj.Delta_T_max = params.Delta_T_max;
            obj.F_max = vessel.maxThrust;      %Fuerza Maxima por propulsor
            obj.F_min = vessel.minThrust;      %Fuerza Minima por propulsor
            % obj.Fu_max = vessel.maxForce_u;
            % obj.Taor_max = vessel.maxTorque_r;
            obj.d  = vessel.y_pont;
            obj.Ts_LL = Ts;
            obj.computation_time = 0.0;
            obj.success = 0;

            obj.sL = vessel.lateralAreaCentroidY;
            obj.Loa  = vessel.totalLength;
            obj.AFw = vessel.frontalArea;
            obj.ALw = vessel.lateralArea;
            obj.af_al = obj.AFw/obj.ALw;
            obj.sl_loa = obj.sL/obj.Loa;

            obj.const_tauX = 0.5*1.1960*obj.AFw;
            obj.const_tauY = 0.5*1.1960*obj.ALw;
            obj.const_tauN = 0.5*1.1960*obj.ALw*obj.Loa;
            
            % Ecuaciones del modelo completas:
            [obj.f_model, obj.n_controls, obj.n_states, obj.n_disturbances] = obj.Dinamica_modelada(vessel);   
            
            % Modelo sobre el horizonte de prediccion:
            [obj.ff_opt, X_LL, U_LL, obj.Param] = obj.Modelo_predictivo(obj.f_model, Ts);
            
            %  Construcción de la función objetivo a minimizar:
            [obj.obj_fc] = obj.funcion_objetivo(X_LL, U_LL, obj.Param);
            
            [obj.solver, obj.args] = obj.create_solver(obj.obj_fc, X_LL, U_LL, obj.Param);


            obj.u_nc = zeros(obj.N_c,obj.n_controls); 
            obj.u_nc(:,1) = 2*obj.F_max*ones(size(obj.u_nc,1),1);
            obj.u_nc(:,2) = zeros(size(obj.u_nc,1),1);
            obj.U0_LL = obj.u_nc;  % la inicial por fecto todo al maximo

            obj.U_ant = zeros(obj.n_controls,1);

            obj.j_min = 0;
            % 3 inicial | 60 referencias | 40 U_ref | 2 Controles anteriores 
            obj.p_vec = zeros(obj.n_states + obj.N_p*obj.n_states + obj.N_c*obj.n_controls + obj.n_controls + obj.N_p*obj.n_disturbances,1);
            obj.n_ite = 0;
        end
        function value = getValues(obj)
            u_opt = reshape(obj.u_nc',obj.n_controls*obj.N_c,1);
            % 1 jm | p_vec | n_controls*NC U_opt | 1 N_iteraciones | 1 T_solucion | 1 Succesful 
            value = [ obj.j_min; obj.p_vec; u_opt; obj.n_ite; obj.computation_time; obj.success];
        end
    end

    methods (Static)
      function xk_hat = get_mpc_llc(m_llc)
            no_vessels = length(m_llc);
            len_pvec = m_llc{1}.n_states + m_llc{1}.N_p*m_llc{1}.n_states + m_llc{1}.N_c*m_llc{1}.n_controls + m_llc{1}.n_controls+ m_llc{1}.N_p*m_llc{1}.n_disturbances;
            len_u_opt = m_llc{1}.N_c*m_llc{1}.n_controls;
            lent_total = (1+len_pvec+len_u_opt+3);
            xk_hat = zeros(lent_total*no_vessels,1);
            for v = 1:no_vessels
                xk_hat(1+(v-1)*lent_total:lent_total+(v-1)*lent_total) = m_llc{v}.getValues();
            end
      end
   end

    methods (Access = private)

        function [f_model, n_controls, n_states, n_disturbances] = Dinamica_modelada(obj, vessel)
            import casadi.*; 

            psi = SX.sym('psi');                % Orientación respecto al Norte (rad)            
            u_r = SX.sym('u_r');                % Velocidad de surge relativo (m/s)
            v_r = SX.sym('v_r');                % Velocidad de sway relativo (m/s)
            r = SX.sym('r');                    % Velocidad angular yaw (rad/s)

            F_U = SX.sym('F_U');              	% Fuerza de avance (N)
            T_R = SX.sym('T_R');              	% Par de giro (N*m)

            V_w = SX.sym('V_w');              	    % Velocidad del viento (m/s)
            beta_w = SX.sym('beta_w');              % Dirección estándar del viento (rad)
            V_c = SX.sym('V_c');                    % Velocidad de la corriente (m/s)
            beta_c = SX.sym('beta_c');              % Dirección de la corriente (rad)
            V_c_dot = SX.sym('V_c_dot');            % Derivada temporal de la velocidad de la corriente (m/s2)
            beta_c_dot = SX.sym('beta_c_dot');      % Derivada temporal de la dirección de la corriente (rad/s)
            
            states = [psi;u_r;v_r;r]; 
            n_states = length(states);
            
            if isequal(size(vessel.M), [6 6])
                vessel.M = [ ...
                    vessel.M(1,1) vessel.M(1,2) vessel.M(1,6); ...
                    vessel.M(2,1) vessel.M(2,2) vessel.M(2,6); ...
                    vessel.M(6,1) vessel.M(6,2) vessel.M(6,6)  ...
                ];
            end

            tau = [F_U 0 T_R]';
            IG = vessel.invM*tau;
            controls = [F_U;T_R];
            n_controls = length(controls);

            disturbances = [V_w;beta_w;V_c;beta_c;V_c_dot;beta_c_dot];
            n_disturbances = length(disturbances);
          
            D = [-vessel.X_u-vessel.X_u_abs_u*abs(u_r),                 0,                                      0;
                0,  -vessel.Y_v-vessel.Y_v_abs_v*abs(v_r)-vessel.Y_v_abs_r*abs(r), -vessel.Y_r-vessel.Y_r_abs_v*abs(v_r)-vessel.Y_r_abs_r*abs(r);
                0,  -vessel.N_v-vessel.N_v_abs_v*abs(v_r)-vessel.N_v_abs_r*abs(r), -vessel.N_r-vessel.N_r_abs_v*abs(v_r)-vessel.N_r_abs_r*abs(r)];
            
            C = [0,         0,          -vessel.M(2,2)*v_r-vessel.M(2,3)*r;
                 0,         0,                    vessel.M(1,1)*u_r;
                vessel.M(2,2)*v_r+vessel.M(2,3)*r,      -vessel.M(1,1)*u_r,      0];

            V_c_vec = [V_c*cos(beta_c) V_c*sin(beta_c) 0]';
            
            ROT = [cos(psi),    - sin(psi),      0;
                   sin(psi),      cos(psi),      0;
                          0,             0,      1];
                      
            nu_c = ROT'*V_c_vec;
            
            % 5.2.2. Viento:
            
            u = u_r + nu_c(1);
            v = v_r + nu_c(2);
           
            u_w = V_w*cos(beta_w - psi);
            v_w = V_w*sin(beta_w - psi);
            
            u_r_w = u - u_w;
            v_r_w = v - v_w;
            
            V_r_w = sqrt(u_r_w^2 + v_r_w^2);
            gamma_r_w = - atan2(v_r_w,u_r_w);

            CD_l_AF = if_else(abs(gamma_r_w) <= pi/2, 0.55, 0.65);

            % wind coefficients
            CD_l = CD_l_AF*obj.af_al;
            den = 1-0.5*0.60*(1-CD_l/0.85)*sin(2*gamma_r_w)^2;

            CX = -CD_l_AF*cos(gamma_r_w)/den;
            CY =  0.85*sin(gamma_r_w)/den;
            CN =  (obj.sl_loa - 0.18*(gamma_r_w - pi/2))*CY;

            tau_w = [obj.const_tauX * CX*V_r_w^2;
                     obj.const_tauY * CY*V_r_w^2;
                     obj.const_tauN * CN*V_r_w^2 ];
            
            sigma = -vessel.invM*(C*[u_r;v_r;r] + D*[u_r;v_r;r]-tau_w);
            dynamics = IG + sigma; 

            psi_dot = r; 

            model = [psi_dot;dynamics];

            % Función no lineal de mapeo x_dot = f(x,u) del modelo:
            f_model = Function('f',{states,controls,disturbances},{model});
        end
        
        function [ff_opt, X_LL, U_LL, Param] = Modelo_predictivo(obj, f_model, Ts)
            import casadi.*; 
            U_LL = SX.sym('U_LL',obj.n_controls,obj.N_c); 
            X_LL = SX.sym('X_LL',obj.n_states,(obj.N_p+1));
            
            % 6. Vector Parametros
            % - El estado inicial P_LL(1:obj.n_states)
            % - Los estados deseados a lo largo del horizonte P_LL(obj.n_states+1:obj.n_states+obj.n_states*obj.N_p)
            % - Los controles deseados a lo largo del horizonte P_LL(obj.n_states+obj.n_states*obj.N_p+1:obj.n_states+obj.n_states*obj.N_p+obj.n_controls*obj.N_c)
            % - Los controles implementados en el instante anterior P_LL(obj.n_states+obj.n_states*obj.N_p+obj.n_controls*obj.N_c+1:obj.n_states+obj.n_states*obj.N_p+obj.n_controls*obj.N_c+obj.n_controls)
            % - Las perturbaciones estimadas a lo largo del horizonte P_LL(obj.n_states+obj.n_states*obj.N_p+obj.n_controls*obj.N_c+1:obj.n_states+obj.n_states*obj.N_p+obj.n_controls*obj.N_c+obj.n_controls+n_disturbances_LL*N_p_LL)
            
            Param = SX.sym('P_LL',obj.n_states + obj.N_p*obj.n_states + obj.N_c*obj.n_controls + obj.n_controls + obj.N_p*obj.n_disturbances);
            
            % 7. Solución simbólica:

            idx_dis0 = obj.n_states + obj.N_p*obj.n_states + obj.N_c*obj.n_controls + obj.n_controls; 
            
            X_LL(:,1) = Param(1:obj.n_states);            % Estado inicial
            for k = 1:obj.N_p
                st       = X_LL(:,k);                   % Estados
                dis = Param(idx_dis0 + (k-1)*obj.n_disturbances + 1: idx_dis0 + k*obj.n_disturbances); %disturbances
                if k <= obj.N_c                          % Acciones de control
                    con      = U_LL(:,k);                   
                else
                    con      = U_LL(:,obj.N_c);
                end
                f_value  = f_model(st,con,dis);          % Ecuaciones del modelo continuo
                st_next  = st + Ts*f_value;          % Ecuaciones del modelo discretizado
                X_LL(:,k+1) = st_next;                  % Shift temporal
            end
            
            % 8. Función para obtener la trayectoria óptima de los estados sabiendo la solución óptima:
            
            ff_opt = Function('ff',{U_LL,Param},{X_LL});
        end
    
        function [obj_LL] = funcion_objetivo(obj, X_LL, U_LL, P_LL)
            import casadi.*; 
            
                                   
            % 10. Construcción de la función objetivo a minimizar:
            
            obj_LL = 0; 
            for k=2:obj.N_p+1
                
               cost_con = SX(0);  % Crea un 1x1 casadi.SX con valor cero
               if k <= obj.N_c+1                                                                                   
                    con      = U_LL(:,k-1);                                                                     % Acciones de control
                    cost_con = con'*obj.R*con;                                                          % Coste de etapa de las acciones de control
                % else
                %     con      = U_LL(:,obj.N_c);                                                                  % Acciones de control
                %     cost_con = con'*obj.R*con;                                                       % Coste de etapa de las acciones de control
                end
            
                st = X_LL(:,k);                                                                                 % Estados
                ref_st = P_LL(obj.n_states*(k-1)+1:obj.n_states*k);                                             % Referencias para los estados

                idx_dis0 = obj.n_states + obj.N_p*obj.n_states + obj.N_c*obj.n_controls + obj.n_controls; 
                dis = P_LL(idx_dis0 + (k-2)*obj.n_disturbances + 1: idx_dis0 + (k-1)*obj.n_disturbances); %disturbances
                ROT = [cos(st(1)) -sin(st(1)) 0;sin(st(1)) cos(st(1)) 0; 0 0 1];
                nu_c = ROT'*[dis(3)*cos(dis(4)) dis(3)*sin(dis(4)) 0]';
                zt = [atan2(sin(st(1)-ref_st(1)), cos(st(1)-ref_st(1))) st(2)+nu_c(1)-ref_st(2) st(3)+nu_c(2)-ref_st(3) st(4)-ref_st(4)]';

                cost_st = (zt)'*obj.Q*(zt);                                             % Coste de etapa de los errores de seguimiento de las referencias de los estados
                
                cost_ref_con = SX(0);  % Crea un 1x1 casadi.SX con valor cero
                if k <= obj.N_c+1                                                                                   
                    ref_con  = P_LL(obj.n_states+obj.n_states*obj.N_p+obj.n_controls*(k-2)+1:obj.n_states+obj.n_states*obj.N_p+obj.n_controls*(k-1));     % Referencias para las acciones de control
                    cost_ref_con = (con - ref_con)'*obj.Nu*(con - ref_con);                                 % Coste de etapa de las desviaciones de las acciones de control respecto a los valores deseados    
                % else
                %     ref_con  = P_LL(obj.n_states+obj.n_states*obj.N_p+obj.n_controls*(obj.N_c-1)+1:obj.n_states+obj.n_states*obj.N_p+obj.n_controls*obj.N_c);     % Referencias para las acciones de control
                %     cost_ref_con = (con - ref_con)'*obj.Nu*(con - ref_con);                                 % Coste de etapa de las desviaciones de las acciones de control respecto a los valores deseados    
                end
                    
                obj_LL = obj_LL + cost_st + cost_con + cost_ref_con;                                            % Función objetivo 
            end
        end
        
        function [solver, args] = create_solver(obj, obj_fc, X_LL, U_LL, P_LL)
            % 11. Restricciones:
            import casadi.*; 
            % 11.1: Variables de estado:

            idx_dis0 = obj.n_states + obj.N_p*obj.n_states + obj.N_c*obj.n_controls + obj.n_controls; 

            g_LL = [];  
            for k = 2:obj.N_p+1  
                g_LL = [g_LL ; X_LL(1,k)];   %#ok<*AGROW> % Variable de estado psi

                dis = P_LL(idx_dis0 + (k-2)*obj.n_disturbances + 1: idx_dis0 + (k-1)*obj.n_disturbances); %disturbances
                ROT = [cos(X_LL(1,k)) -sin(X_LL(1,k)) 0;sin(X_LL(1,k)) cos(X_LL(1,k)) 0; 0 0 1];
                nu_c = ROT'*[dis(3)*cos(dis(4)) dis(3)*sin(dis(4)) 0]';
                
                g_LL = [g_LL ; X_LL(2,k) + nu_c(1)];   % Variable de estado u
                g_LL = [g_LL ; X_LL(3,k) + nu_c(2)];   % Variable de estado v
                g_LL = [g_LL ; X_LL(4,k)];   % Variable de estado r
            end
            
            args = struct;
            
            args.lbg(obj.n_states-3:obj.n_states:obj.n_states*obj.N_p-3,1) = -Inf;              % Límite inferior para la variable de estado psi (rad)
            args.lbg(obj.n_states-2:obj.n_states:obj.n_states*obj.N_p-2,1) = obj.u_min;         % Límite inferior para la variable de estado u (m/s)
            args.lbg(obj.n_states-1:obj.n_states:obj.n_states*obj.N_p-1,1) = obj.v_min;         % Límite inferior para la variable de estado v (m/s)
            args.lbg(obj.n_states:obj.n_states:obj.n_states*obj.N_p,1) = obj.r_min;             % Límite inferior para la variable de estado r (rad/s)
            
            args.ubg(obj.n_states-3:obj.n_states:obj.n_states*obj.N_p-3,1) = Inf;               % Límite superior para la variable de estado psi (rad)
            args.ubg(obj.n_states-2:obj.n_states:obj.n_states*obj.N_p-2,1) = obj.u_max;         % Límite superior para la variable de estado u (m/s)
            args.ubg(obj.n_states-1:obj.n_states:obj.n_states*obj.N_p-1,1) = obj.v_max;         % Límite superior para la variable de estado v (m/s)
            args.ubg(obj.n_states:obj.n_states:obj.n_states*obj.N_p,1) = obj.r_max;             % Límite superior para la variable de estado r (rad/s)
            
            % 11.2: Delta Acciones de control:
            
            for i=1:obj.n_controls
                g_LL = [g_LL ; U_LL(i,1) - P_LL(obj.n_states + obj.N_p*obj.n_states + obj.N_c*obj.n_controls + i,1)];   %#ok<*AGROW> % Incremento de las primeras variables de entrada
            end
            
            for k = 2:obj.N_c 
                g_LL = [g_LL ; U_LL(1,k) - U_LL(1,k-1)];   %#ok<*AGROW> % Incrementos de variables de entrada: Delta_F_min <= (F_u(k) - F_u(k-1)) <= Delta_F_max
                g_LL = [g_LL ; U_LL(2,k) - U_LL(2,k-1)];   %#ok<*AGROW> % Incrementos de variables de entrada: Delta_T_min <= (T_r(k) - T_r(k-1)) <= Delta_T_max
            end
            
            args.lbg(obj.n_states*obj.N_p+obj.n_controls-1:obj.n_controls:obj.n_states*obj.N_p+obj.n_controls*obj.N_c-1,1) = obj.Delta_F_min;             % Límite inferior para el incremento de la acción de control F_u (N/s)
            args.lbg(obj.n_states*obj.N_p+obj.n_controls:obj.n_controls:obj.n_states*obj.N_p+obj.n_controls*obj.N_c,1) = obj.Delta_T_min;                 % Límite inferior para el incremento de la acción de control T_r (N m/s)
            args.ubg(obj.n_states*obj.N_p+obj.n_controls-1:obj.n_controls:obj.n_states*obj.N_p+obj.n_controls*obj.N_c-1,1) = obj.Delta_F_max;             % Límite superior para el incremento de la acción de control F_u (N/s)
            args.ubg(obj.n_states*obj.N_p+obj.n_controls:obj.n_controls:obj.n_states*obj.N_p+obj.n_controls*obj.N_c,1) = obj.Delta_T_max;                 % Límite superior para el incremento de la acción de control T_r (N m/s)
            
            args.lbx(obj.n_controls-1:obj.n_controls:obj.n_controls*obj.N_c-1,1)  = -Inf; % 0;              % Límite inferior para F_u (N)
            args.lbx(obj.n_controls:obj.n_controls:obj.n_controls*obj.N_c,1)      = -Inf; % -obj.Taor_max;  % Límite inferior para T_r (N m)
            args.ubx(obj.n_controls-1:obj.n_controls:obj.n_controls*obj.N_c-1,1)  = Inf;  % obj.Fu_max;     % Límite superior para F_u (N)
            args.ubx(obj.n_controls:obj.n_controls:obj.n_controls*obj.N_c,1)      = Inf;  % obj.Taor_max;   % Límite superior para T_r (N m)
            
            for k = 1:obj.N_c 
                g_LL = [g_LL ; U_LL(1,k) - (U_LL(2,k)/obj.d)];   %#ok<*AGROW> % Saturacion: 2*F_min <= (F_u(k) - T_r(k)/d) <= 2*F_max
                g_LL = [g_LL ; U_LL(1,k) + (U_LL(2,k)/obj.d)];   %#ok<*AGROW> % Saturacion: 2*F_min <= (F_u(k) + T_r(k)/d) <= 2*F_max
                g_LL = [g_LL ; U_LL(1,k) + (abs(U_LL(2,k))/obj.d)];   %#ok<*AGROW> % Saturacion: 0 <= (F_u(k) + |T_r(k)|/d) <= Inf
            end

            idx_1 = obj.n_states*obj.N_p+obj.n_controls*obj.N_c; % Indice donde inicia estas renstricciones

            args.lbg(idx_1+1:3:idx_1+obj.N_c*3-2,1) = 2*obj.F_min;     % Límite inferior para 
            args.lbg(idx_1+2:3:idx_1+obj.N_c*3-1,1) = 2*obj.F_min;     % Límite inferior para 
            args.lbg(idx_1+3:3:idx_1+obj.N_c*3,1) = 0;            % Límite inferior para 
            
            args.ubg(idx_1+1:3:idx_1+obj.N_c*3-2,1) = 2*obj.F_max;     % Límite superior para 
            args.ubg(idx_1+2:3:idx_1+obj.N_c*3-1,1) = 2*obj.F_max;     % Límite superior para 
            args.ubg(idx_1+3:3:idx_1+obj.N_c*3,1) = Inf;          % Límite superior para 

            %12. Conversión del vector de variables de decisión en un vector de CasADi:
            
            OPT_variables_LL = reshape(U_LL,obj.n_controls*obj.N_c,1);
            nlp_prob_LL = struct('f', obj_fc, 'x', OPT_variables_LL, 'g', g_LL, 'p', P_LL);
            
            % 13. Opciones de la optimización:
            
            N_max_iter = 1000;
            
            opts_LL = struct;
            opts_LL.ipopt.max_iter = N_max_iter;
            opts_LL.ipopt.print_level = 0;  % 3
            opts_LL.print_time = false;  %0
            opts_LL.ipopt.sb = 'yes';               % Silence banner (extra)
            opts_LL.ipopt.acceptable_tol = 1e-6;
            opts_LL.ipopt.acceptable_obj_change_tol = 1e-4;
            
            % 14. Creación del objeto de solver:
            
            solver = nlpsol('solver', 'ipopt', nlp_prob_LL, opts_LL);
        end
     
    end


    methods (Access = protected)

        function tau = stepImpl(obj, ref, estimation, disturbances_mod, Tm_HL)
            import casadi.*; 
            % u_ref = ref(1);
            % r_ref = ref(3);
            % v_ref = ref(2);
            
            N_c_HL = length(ref)/3.0;

            psi_hat = estimation(3);
            u_hat = estimation(4);
            v_hat = estimation(5);
            r_hat = estimation(6);

            V_w_0_mod = disturbances_mod.V_w;
            beta_w_0_mod = disturbances_mod.beta_w;
            V_c_0_mod = disturbances_mod.V_c;
            beta_c_0_mod = disturbances_mod.beta_c;
            V_c_dot_0_mod = disturbances_mod.V_c_dot;
            beta_c_dot_0_mod = disturbances_mod.beta_c_dot;

            ROT = [cos(psi_hat) -sin(psi_hat) 0;sin(psi_hat) cos(psi_hat) 0; 0 0 1];
            nu_c = ROT'*[V_c_0_mod*cos(beta_c_0_mod) V_c_0_mod*sin(beta_c_0_mod) 0]';
            u_c = nu_c(1);
            v_c = nu_c(2);

            % u_c = V_c_0_mod * cos(beta_c_0_mod - psi_hat);
            % v_c = V_c_0_mod * sin(beta_c_0_mod - psi_hat);

            u_r_hat = u_hat - u_c;
            v_r_hat = v_hat - v_c;

            p_vector = zeros(obj.n_states + obj.N_p*obj.n_states + obj.N_c*obj.n_controls + obj.n_controls + obj.N_p*obj.n_disturbances,1);

            p_vector(1:obj.n_states,1) = [psi_hat u_r_hat v_r_hat r_hat];
            for k=2:obj.N_p+1
                if k<=N_c_HL*Tm_HL/obj.Ts_LL
                    ind_k_HL = floor((k-2)*obj.Ts_LL/Tm_HL)+1;
                    xs_LL = [ref((ind_k_HL-1)*3+2) ref((ind_k_HL-1)*3+1) 0 ref((ind_k_HL-1)*3+3)]';
                else
                    xs_LL = [ref((N_c_HL-1)*3+2) ref((N_c_HL-1)*3+1) 0 ref((N_c_HL-1)*3+3)]';
                end
                p_vector(obj.n_states*(k-1)+1:obj.n_states*k,1) = xs_LL;
            end

            % No es nesesario ya que es 0
            % for k=1:N_c_LL
            %     us_LL = [0 0]';
            %     p_vector_LL(n_states_LL+n_states_LL*N_p_LL+n_controls_LL*(k-1)+1:n_states_LL+n_states_LL*N_p_LL+n_controls_LL*k,1) = us_LL;
            % end

            p_vector(obj.n_states*(obj.N_p+1)+obj.n_controls*obj.N_c+1:obj.n_states*(obj.N_p+1)+obj.n_controls*obj.N_c+obj.n_controls,1) = obj.U_ant;

            idx_dis0 = obj.n_states + obj.N_p*obj.n_states + obj.N_c*obj.n_controls + obj.n_controls; 
            
            for k=1:obj.N_p
                p_vector(idx_dis0 + (k-1)*obj.n_disturbances+1: idx_dis0 + k*obj.n_disturbances,1) = [V_w_0_mod beta_w_0_mod V_c_0_mod beta_c_0_mod V_c_dot_0_mod beta_c_dot_0_mod]';
            end

            obj.p_vec = p_vector;
            x0 = reshape(obj.u_nc',obj.n_controls*obj.N_c,1);

            tic
            sol = obj.solver('x0', x0, ...                            % Solución inicial propuesta
                               'lbx', obj.args.lbx, 'ubx', obj.args.ubx, ...      % Límites de las acciones de control
                               'lbg', obj.args.lbg, 'ubg', obj.args.ubg, ...      % Límites de las variables de estado (restricciones g) 
                               'p', p_vector);                                  % Vector de parámetros de la optimización
            obj.computation_time = toc;
            stat = obj.solver.stats();                         
            obj.success = stat.success;
            obj.n_ite = stat.iter_count;
        
            % Repetición de la optimización con solución inicial por defecto, en caso de fracaso:
        
            if obj.success == 0
                tic
                x0 = reshape(obj.U0_LL',obj.n_controls*obj.N_c,1); % initial value of the optimization variables
                sol = obj.solver('x0', x0, ...
                                   'lbx', obj.args.lbx, 'ubx', obj.args.ubx, ...      % Límites de las acciones de control
                                   'lbg', obj.args.lbg, 'ubg', obj.args.ubg, ...      % Límites de las variables de estado (restricciones g) 
                                   'p', p_vector);                                  % Vector de parámetros de la optimización
                obj.computation_time = toc;
                stat = obj.solver.stats();                         
                obj.success = stat.success;
                obj.n_ite = stat.iter_count;
            end

            if obj.success == 0
                obj.u_nc = [obj.u_nc(2:end, :); obj.u_nc(end, :)];
                obj.U_ant = obj.u_nc(1,:);
                obj.j_min = -1;
            else
                obj.u_nc = reshape(full(sol.x)',obj.n_controls,obj.N_c)';
                obj.U_ant = obj.u_nc(1,:);
                obj.j_min = full(sol.f);
            end
            
            
            % Controlador de bajo nivel: Extracción de las acciones de control óptimas:
            % ff_value_LL = obj.ff_opt(u_LL',p_vector);

            %% Salida
            tau = [obj.U_ant(1); 0; obj.U_ant(2)];
        end
    end
end
