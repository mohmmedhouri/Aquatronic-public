classdef MpcPFController < matlab.System
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
        f_model
        ff_opt
        Param
        obj_fc
        solver
        args

        T_s
        R_T

        x_e_bar_max
        x_e_bar_min 
        y_e_bar_max
        y_e_bar_min 
        psi_max 
        psi_min
        w_max 
        w_min 
        v_bar_max 
        v_bar_min

        banda_error_y_e_bar
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


        epsilon
        path

    end

    properties 
        w_ant
        u_tar_next
        x_e
        y_e
        v_bar_i
        computation_time
        success
        u_nc
        U0_HL
        U_ant
        j_min
        p_vec
        n_ite
        init
        sincronizador
    end

    methods
        function obj = MpcPFController(vessel, params, Ts)
            import casadi.*; 
            obj = obj@matlab.System();
            obj.N_p = params.N_p;
            obj.N_c = params.N_c;
            obj.Q = params.Q;
            obj.R = params.R;
            obj.Nu = params.Nu;

            obj.x_e_bar_max = 10;           % Límite superior para la variable de estado x_e_bar (m)
            obj.x_e_bar_min = -10;          % Límite inferior para la variable de estado x_e_bar (m)
            obj.y_e_bar_max = 10;           % Límite superior para la variable de estado y_e_bar (m)
            obj.y_e_bar_min = -10;          % Límite inferior para la variable de estado y_e_bar (m)
            obj.psi_max = Inf;               % Límite superior para la variable de estado psi (rad)
            obj.psi_min = -Inf;              % Límite inferior para la variable de estado psi (rad)
            obj.w_max = Inf;                % Límite superior para la variable de estado w (p.u.)       
            obj.w_min = 0;                  % Límite inferior para la variable de estado w (p.u.)       
            obj.v_bar_max = 1.5;              % Límite superior para la variable de estado v_bar (m/s)       
            obj.v_bar_min = -1.5;             % Límite inferior para la variable de estado v_bar (m/s) 

            obj.u_ref_max = 1.8;                            % Límite superior para la acción de control u_ref (m/s)
            obj.u_ref_min = 0.4;                              % Límite inferior para la acción de control u_ref (m/s)
            obj.u_tar_max = 3;                              % Límite superior para la acción de control u_tar (m/s)
            obj.u_tar_min = 0.01;                           % Límite inferior para la acción de control u_tar (m/s)
            obj.r_ref_max = pi/4;                           % Límite superior para la acción de control r_ref (rad/s)
            obj.r_ref_min = -pi/4;                          % Límite inferior para la acción de control r_ref (rad/s)
            
            obj.Delta_u_ref_min = -0.1;                     % Límite inferior para los incrementos de acción de control u (m/s)
            obj.Delta_u_ref_max = 0.1;                      % Límite superior para los incrementos de acción de control u (m/s)
            obj.Delta_u_tar_min = -Inf;                     % Límite inferior para los incrementos de acción de control u_tar (m/s)
            obj.Delta_u_tar_max = Inf;                      % Límite superior para los incrementos de acción de control u_tar (m/s)
            obj.Delta_r_ref_min = -pi/4/10;                 % Límite inferior para los incrementos de acción de control r (rad/s)
            obj.Delta_r_ref_max = pi/4/10;                  % Límite superior para los incrementos de acción de control r (rad/s)
     
            obj.T_s = Ts;
            obj.R_T = Ts/params.t_sim;
            if obj.R_T < 1 || abs(obj.R_T - round(obj.R_T)) > 1e-10
                error('Error in mid-level control timing');
            end
            obj.R_T = round(obj.R_T);
            obj.sincronizador = obj.R_T;
            obj.computation_time = 0.0;
            obj.success = 0;
            
            % Ecuaciones del modelo completas:
            [obj.f_model, obj.n_controls, obj.n_states, obj.epsilon] = obj.Dinamica_modelada(vessel);
            
            % Modelo sobre el horizonte de prediccion:
            [obj.ff_opt, X_HL, U_HL, obj.Param] = obj.Modelo_predictivo(obj.f_model, Ts);
            
            %  Construcción de la función objetivo a minimizar:
            [obj.obj_fc] = obj.funcion_objetivo(X_HL, U_HL, obj.Param);
            
            [obj.solver, obj.args] = obj.create_solver(obj.obj_fc, X_HL, U_HL, obj.Param);

            obj.u_nc = zeros(obj.N_c,obj.n_controls); 
            obj.u_nc(:,1) = ones(size(obj.u_nc,1),1);
            obj.u_nc(:,2) = zeros(size(obj.u_nc,1),1);
            obj.u_nc(:,3) = zeros(size(obj.u_nc,1),1);
            obj.U0_HL = obj.u_nc;  % la inicial por fecto todo al maximo
            obj.U_ant = zeros(obj.n_controls,1);

            obj.w_ant = 0;
            obj.u_tar_next = 0;
            obj.x_e = 0;
            obj.y_e = 0;
            obj.v_bar_i = 0;

            obj.j_min = 0;
            % n_states inicial | n_states*NP referencias | n_controls*NC U_opt | n_controls Controles anteriores 
            obj.p_vec = zeros(obj.n_states + obj.N_p*obj.n_states + obj.N_c*obj.n_controls + obj.n_controls,1);
            obj.n_ite = 0;
            obj.init = 0;
        end
        function value = getValues(obj)
            u_opt = reshape(obj.u_nc',obj.n_controls*obj.N_c,1);
            % 1 jm | p_vec | n_controls*NC U_opt | 1 w | 1 x_e | 1 y_e | 1 v_bar | 1 N_iteraciones | 1 T_solucion | 1 Succesful 
            value = [ obj.j_min; obj.p_vec; u_opt; obj.w_ant; obj.x_e; obj.y_e; obj.v_bar_i; obj.n_ite; obj.computation_time; obj.success];
        end
    end

    methods (Static)
        function xk_hat = get_mpc_hlc(m_hlc)
            no_vessels = length(m_hlc);
            len_pvec = m_hlc{1}.n_states + m_hlc{1}.N_p*m_hlc{1}.n_states + m_hlc{1}.N_c*m_hlc{1}.n_controls + m_hlc{1}.n_controls;
            len_u_opt = m_hlc{1}.N_c*m_hlc{1}.n_controls;
            lent_total = (1+len_pvec+len_u_opt+4+3);
            xk_hat = zeros(lent_total*no_vessels,1);
            for v = 1:no_vessels
                xk_hat(1+(v-1)*lent_total:lent_total+(v-1)*lent_total) = m_hlc{v}.getValues();
            end
      end
   end

    methods (Access = private)

        function [f_model, n_controls, n_states, epsilon] = Dinamica_modelada(obj, vessel)
            import casadi.*; 
            
            x_e_bar = SX.sym('x_e_bar');                    % Along-track error modificado (m)
            y_e_bar = SX.sym('y_e_bar');                    % Cross-track error modificado (m)
            psi = SX.sym('psi');                            % Orientación respecto al Norte (rad)
            w = SX.sym('w');                                % Variable del punto objetivo sobre la trayectoria (p.u.)
            v_bar = SX.sym('v_bar');                        % Velocidad de sway modificada (m/s)
            
            states = [x_e_bar;y_e_bar;psi;w;v_bar]; 
            n_states = length(states);

            u_ref = SX.sym('u_ref');                        % Referencia para la velocidad de surge (m/s)
            u_tar = SX.sym('u_tar');                        % Velocidad de avance del punto objetivo sobre la trayectoria (m/s)
            r_ref = SX.sym('r_ref');                        % Referencia para la velocidad angular yaw (rad/s)
            
            controls = [u_ref;u_tar;r_ref];
            n_controls = length(controls);

            [spatial_path_CasADI, obj.path] = pathScriptCasADI(w);

            % x_p = spatial_path_CasADI.x_p;                  % Posición Norte del punto objetivo sobre la trayectoria (m)
            % y_p = spatial_path_CasADI.y_p;                  % Posición Este del punto objetivo sobre la trayectoria (m)
            dx_p_dw = spatial_path_CasADI.dx_p_dw;          % Primera derivada de la posición Norte del punto objetivo sobre la trayectoria respecto a la variable del punto objetivo (m)
            dy_p_dw = spatial_path_CasADI.dy_p_dw;          % Primera derivada de la posición Este del punto objetivo sobre la trayectoria respecto a la variable del punto objetivo (m)
            dx_p_dw2 = spatial_path_CasADI.dx_p_dw2;        % Segunda derivada de la posición Norte del punto objetivo sobre la trayectoria respecto a la variable del punto objetivo (m)
            dy_p_dw2 = spatial_path_CasADI.dy_p_dw2;        % Segunda derivada de la posición Este del punto objetivo sobre la trayectoria respecto a la variable del punto objetivo (m)
            phi_p = spatial_path_CasADI.phi_p;              % Ángulo de la trayectoria respecto al Norte en el punto objetivo (rad)
            % tan_phi_p = dy_p_dw/dx_p_dw;                    % Tangente del ángulo de la trayectoria respecto al Norte en el punto objetivo (p.u.)
            F = sqrt(dx_p_dw^2 + dy_p_dw^2);                % Variable de curvatura de la trayectoria en el punto objetivo (m)
            % dphi_p_dw = 1/(1+tan_phi_p^2)*(dy_p_dw2*dx_p_dw - dx_p_dw2*dy_p_dw)/(dx_p_dw^2);        % Derivada del ángulo de la trayectoria respecto al Norte en el punto objetivo respecto a la variable del punto objetivo sobre la trayectoria (rad)
            dphi_p_dw = (dy_p_dw2*dx_p_dw - dx_p_dw2*dy_p_dw)/(dx_p_dw^2+dy_p_dw^2);
            % 4.1. Cinemática del ASV:
            kinematics_ASV = [u_ref*cos(psi - phi_p) - v_bar*sin(psi - phi_p) + u_tar*(1/F*dphi_p_dw*y_e_bar - 1); ...
                              u_ref*sin(psi - phi_p) + v_bar*cos(psi - phi_p) - u_tar*1/F*dphi_p_dw*x_e_bar; ...
                              r_ref]; 

            kinematics_path = u_tar/F;
            
            epsilon = vessel.M(2,3)/vessel.M(2,2);
            v = v_bar - epsilon*r_ref;

            D = [-vessel.X_u-vessel.X_u_abs_u*abs(u_ref),                 0,                                      0;
                0,  -vessel.Y_v-vessel.Y_v_abs_v*abs(v)-vessel.Y_v_abs_r*abs(r_ref), -vessel.Y_r-vessel.Y_r_abs_v*abs(v)-vessel.Y_r_abs_r*abs(r_ref);
                0,  -vessel.N_v-vessel.N_v_abs_v*abs(v)-vessel.N_v_abs_r*abs(r_ref), -vessel.N_r-vessel.N_r_abs_v*abs(v)-vessel.N_r_abs_r*abs(r_ref)];
            
            alpha = vessel.M(1,1)/vessel.M(2,2);
            beta = D(2,2)/vessel.M(2,2);
            gamma = (epsilon)*( (D(2,2)/vessel.M(2,2)) - (D(2,3)/vessel.M(3,2)) );

            % gamma = 1/vessel.M(2,2)*(epsilon*D(2,2) - D(2,3));

            dot_v_bar = - alpha*u_ref*r_ref - beta*v_bar + gamma*r_ref;

            model = [kinematics_ASV;kinematics_path;dot_v_bar];
            
            f_model = Function('f',{states,controls},{model});
        end
        
        function [ff_opt, X_HL, U_HL, Param] = Modelo_predictivo(obj, f_model, Ts)
            import casadi.*; 
            U_HL = SX.sym('U_HL',obj.n_controls,obj.N_c); % Acciones de control a lo largo del horizonte
            X_HL = SX.sym('X_HL',obj.n_states,(obj.N_p+1));% Estados actuales y a lo largo del horizonte
            
            % 6. Vector Parametros
            % - El estado inicial P_LL(1:obj.n_states)
            % - Los estados deseados a lo largo del horizonte P_LL(obj.n_states+1:obj.n_states+obj.n_states*obj.N_p)
            % - Los controles deseados a lo largo del horizonte P_LL(obj.n_states+obj.n_states*obj.N_p+1:obj.n_states+obj.n_states*obj.N_p+obj.n_controls*obj.N_c)
            % - Los controles implementados en el instante anterior P_LL(obj.n_states+obj.n_states*obj.N_p+obj.n_controls*obj.N_c+1:obj.n_states+obj.n_states*obj.N_p+obj.n_controls*obj.N_c+obj.n_controls)
            
            Param = SX.sym('P_HL',obj.n_states + obj.N_p*obj.n_states + obj.N_c*obj.n_controls + obj.n_controls);
            
            % 7. Solución simbólica:
            
            X_HL(:,1) = Param(1:obj.n_states);            % Estado inicial
            for k = 1:obj.N_p
                st       = X_HL(:,k);                   % Estados
                if k <= obj.N_c                          % Acciones de control
                    con      = U_HL(:,k);                   
                else
                    con      = U_HL(:,obj.N_c);
                end
                f_value  = f_model(st,con);          % Ecuaciones del modelo continuo
                st_next  = st + Ts*f_value;          % Ecuaciones del modelo discretizado
                X_HL(:,k+1) = st_next;                  % Shift temporal
            end
            
            % 8. Función para obtener la trayectoria óptima de los estados sabiendo la solución óptima:
            
            ff_opt = Function('ff',{U_HL,Param},{X_HL});
        end
    
        function [obj_HL] = funcion_objetivo(obj, X_HL, U_HL, P_HL)
            import casadi.*; 
           
            % 10. Construcción de la función objetivo a minimizar:
            
            obj_HL = 0; 
            for k=2:obj.N_p+1
                cost_con = SX(0);  % Crea un 1x1 casadi.SX con valor cero
                if k <= obj.N_c+1                                                                                   
                    con      = U_HL(:,k-1);                                                                     % Acciones de control
                    cost_con = con'*obj.R*con;                                                          % Coste de etapa de las acciones de control
                % else
                %     con      = U_HL(:,obj.N_c);                                                                  % Acciones de control
                %     cost_con = con'*obj.R*con;                                                       % Coste de etapa de las acciones de control
                end
            
                st = X_HL(:,k);                                                                                 % Estados
                ref_st = P_HL(obj.n_states*(k-1)+1:obj.n_states*k);                                               % Referencias para los estados
                

                cost_st = (st - ref_st)'*obj.Q*(st - ref_st);                                             % Coste de etapa de los errores de seguimiento de las referencias de los estados
            
                cost_ref_con = SX(0);  % Crea un 1x1 casadi.SX con valor cero
                if k <= obj.N_c+1                                                                                   
                    ref_con  = P_HL(obj.n_states+obj.n_states*obj.N_p+obj.n_controls*(k-2)+1:obj.n_states+obj.n_states*obj.N_p+obj.n_controls*(k-1));     % Referencias para las acciones de control
                    cost_ref_con = (con - ref_con)'*obj.Nu*(con - ref_con);                                 % Coste de etapa de las desviaciones de las acciones de control respecto a los valores deseados    
                % else
                %     ref_con  = P_HL(obj.n_states+obj.n_states*obj.N_p+obj.n_controls*(obj.N_c-1)+1:obj.n_states+obj.n_states*obj.N_p+obj.n_controls*obj.N_c);     % Referencias para las acciones de control
                %     cost_ref_con = (con - ref_con)'*obj.Nu*(con - ref_con);                                 % Coste de etapa de las desviaciones de las acciones de control respecto a los valores deseados    
                end
                    
                obj_HL = obj_HL + cost_st + cost_con + cost_ref_con;                                            % Función objetivo 
            end
        end
        
        function [solver, args] = create_solver(obj, obj_fc, X_HL, U_HL, P_HL)
            % 11. Restricciones:
            import casadi.*; 
            % 11.1: Variables de estado:          
            g_HL = [];  
            for k = 2:obj.N_p+1  
                g_HL = [g_HL ; X_HL(1,k)];   %#ok<*AGROW> % Variable de estado x_e_bar
                g_HL = [g_HL ; X_HL(2,k)];   % Variable de estado y_e_bar
                g_HL = [g_HL ; X_HL(3,k)];   % Variable de estado psi
                g_HL = [g_HL ; X_HL(4,k)];   % Variable de estado w
                g_HL = [g_HL ; X_HL(5,k)];   % Variable de estado v_bar
            end
            
            args = struct;
            
            args.lbg(obj.n_states-4:obj.n_states:obj.n_states*obj.N_p-4,1) = obj.x_e_bar_min;     % Límite inferior para la variable de estado x_e_bar (m)
            args.lbg(obj.n_states-3:obj.n_states:obj.n_states*obj.N_p-3,1) = obj.y_e_bar_min;     % Límite inferior para la variable de estado y_e_bar (m)
            args.lbg(obj.n_states-2:obj.n_states:obj.n_states*obj.N_p-2,1) = obj.psi_min;         % Límite inferior para la variable de estado psi (rad)
            args.lbg(obj.n_states-1:obj.n_states:obj.n_states*obj.N_p-1,1) = obj.w_min;           % Límite inferior para la variable de estado w (p.u.)
            args.lbg(obj.n_states:obj.n_states:obj.n_states*obj.N_p,1) = obj.v_bar_min;           % Límite inferior para la variable de estado v_bar (m/s)
            
            args.ubg(obj.n_states-4:obj.n_states:obj.n_states*obj.N_p-4,1) = obj.x_e_bar_max;     % Límite superior para la variable de estado x_e_bar (m)
            args.ubg(obj.n_states-3:obj.n_states:obj.n_states*obj.N_p-3,1) = obj.y_e_bar_max;     % Límite superior para la variable de estado y_e_bar (m)
            args.ubg(obj.n_states-2:obj.n_states:obj.n_states*obj.N_p-2,1) = obj.psi_max;         % Límite superior para la variable de estado psi (rad)
            args.ubg(obj.n_states-1:obj.n_states:obj.n_states*obj.N_p-1,1) = obj.w_max;           % Límite superior para la variable de estado w (p.u.)
            args.ubg(obj.n_states:obj.n_states:obj.n_states*obj.N_p,1) = obj.v_bar_max;           % Límite superior para la variable de estado v_bar (m/s)
            
            % 11.2: Delta Acciones de control:
            
            for i=1:obj.n_controls
                g_HL = [g_HL ; U_HL(i,1) - P_HL(obj.n_states + obj.N_p*obj.n_states + obj.N_c*obj.n_controls + i,1)];   %#ok<*AGROW> % Incremento de las primeras variables de entrada
            end
            
            for k = 2:obj.N_c 
                g_HL = [g_HL ; U_HL(1,k) - U_HL(1,k-1)];   %#ok<*AGROW> % Incrementos de variables de entrada: Delta_u_ref_min <= (u_ref(k) - u_ref(k-1)) <= Delta_u_ref_max
                g_HL = [g_HL ; U_HL(2,k) - U_HL(2,k-1)];   %#ok<*AGROW> % Incrementos de variables de entrada: Delta_u_tar_min <= (u_tar(k) - u_tar(k-1)) <= Delta_u_tar_max
                g_HL = [g_HL ; U_HL(3,k) - U_HL(3,k-1)];   %#ok<*AGROW> % Incrementos de variables de entrada: Delta_r_ref_min <= (r_ref(k) - r_ref(k-1)) <= Delta_r_ref_max
            end
            
            args.lbg(obj.n_states*obj.N_p+obj.n_controls-2:obj.n_controls:obj.n_states*obj.N_p+obj.n_controls*obj.N_c-2,1) = obj.Delta_u_ref_min;               % Límite inferior para el incremento de la acción de control u_ref (m/s)
            args.lbg(obj.n_states*obj.N_p+obj.n_controls-1:obj.n_controls:obj.n_states*obj.N_p+obj.n_controls*obj.N_c-1,1) = obj.Delta_u_tar_min;               % Límite inferior para el incremento de la acción de control u_tar (m/s)
            args.lbg(obj.n_states*obj.N_p+obj.n_controls:obj.n_controls:obj.n_states*obj.N_p+obj.n_controls*obj.N_c,1) = obj.Delta_r_ref_min;               % Límite inferior para el incremento de la acción de control r_ref (rad/s)
             
            args.ubg(obj.n_states*obj.N_p+obj.n_controls-2:obj.n_controls:obj.n_states*obj.N_p+obj.n_controls*obj.N_c-2,1) = obj.Delta_u_ref_max;               % Límite superior para el incremento de la acción de control u_ref (m/s)
            args.ubg(obj.n_states*obj.N_p+obj.n_controls-1:obj.n_controls:obj.n_states*obj.N_p+obj.n_controls*obj.N_c-1,1) = obj.Delta_u_tar_max;               % Límite superior para el incremento de la acción de control u_tar (m/s)
            args.ubg(obj.n_states*obj.N_p+obj.n_controls:obj.n_controls:obj.n_states*obj.N_p+obj.n_controls*obj.N_c,1) = obj.Delta_r_ref_max;               % Límite superior para el incremento de la acción de control r_ref (rad/s)
            
            args.lbx(obj.n_controls-2:obj.n_controls:obj.n_controls*obj.N_c-2,1)  = obj.u_ref_min;               % Límite inferior para la entrada u_ref (m/s)
            args.lbx(obj.n_controls-1:obj.n_controls:obj.n_controls*obj.N_c-1,1)  = obj.u_tar_min;               % Límite inferior para la entrada u_tar (m/s)
            args.lbx(obj.n_controls:obj.n_controls:obj.n_controls*obj.N_c,1)  = obj.r_ref_min;               % Límite inferior para la entrada r_ref (rad/s)
            
            args.ubx(obj.n_controls-2:obj.n_controls:obj.n_controls*obj.N_c-2,1)  = obj.u_ref_max;               % Límite superior para la entrada u_ref (m/s)
            args.ubx(obj.n_controls-1:obj.n_controls:obj.n_controls*obj.N_c-1,1)  = obj.u_tar_max;               % Límite superior para la entrada u_tar (m/s)
            args.ubx(obj.n_controls:obj.n_controls:obj.n_controls*obj.N_c,1)  = obj.r_ref_max;               % Límite superior para la entrada r_ref (rad/s)
            
            %12. Conversión del vector de variables de decisión en un vector de CasADi:
            
            OPT_variables_HL = reshape(U_HL,obj.n_controls*obj.N_c,1);
            nlp_prob_HL = struct('f', obj_fc, 'x', OPT_variables_HL, 'g', g_HL, 'p', P_HL);
            
            % 13. Opciones de la optimización:
            
            N_max_iter = 1000;
            
            opts_HL = struct;
            opts_HL.ipopt.max_iter = N_max_iter;
            opts_HL.ipopt.print_level = 0;  % 3
            opts_HL.print_time = false;  %0 false
            opts_HL.ipopt.sb = 'yes';               % Silence banner (extra)
            opts_HL.ipopt.acceptable_tol = 1e-6;
            opts_HL.ipopt.acceptable_obj_change_tol = 1e-4;
           
            % 14. Creación del objeto de solver:
            
            solver = nlpsol('solver', 'ipopt', nlp_prob_HL, opts_HL);
        end
     
        function [xe, ye, v_bar] =  get_inicial_values(obj, x, y, psi, v, r)
            obj.evolve_w(obj.u_tar_next);
            x_p = obj.path.x_p(obj.w_ant);
            y_p = obj.path.y_p(obj.w_ant);
            phi_p = obj.path.phi_p(obj.w_ant);
            R2 = [cos(phi_p) -sin(phi_p);
                  sin(phi_p) cos(phi_p)];
            aux = R2'*[x + obj.epsilon*cos(psi) - x_p;
                       y + obj.epsilon*sin(psi) - y_p];

            xe = aux(1); 
            ye = aux(2);
            v_bar = v + obj.epsilon*r;

            obj.x_e = xe;
            obj.y_e = ye;
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

        function refs = stepImpl(obj, u_d, estimation)
            import casadi.*; 
            if obj.sincronizador == obj.R_T
                x_hat = estimation(1);
                y_hat = estimation(2);
                psi_hat = estimation(3);
                % u_hat = estimation(4);
                v_hat = estimation(5);
                r_hat = estimation(6);
    
                if (obj.init == 0)
                    obj.u_nc(:,1) = u_d*obj.u_nc(:,1);
                    obj.u_nc(:,2) = u_d*obj.u_nc(:,2);
                    obj.U0_HL = obj.u_nc;  % la inicial por fecto todo al maximo
                    obj.init = 1;
                end
    
                [xe, ye, v_bar] = obj.get_inicial_values(x_hat, y_hat, psi_hat, v_hat, r_hat);
    
                p_vector = zeros(obj.n_states + obj.N_p*obj.n_states + obj.N_c*obj.n_controls + obj.n_controls,1);
    
                p_vector(1:obj.n_states,1) = [xe ye psi_hat, obj.w_ant, v_bar];
                
                % No es nesesario ya que es 0
                % for k=2:N_p_HL+1
                %     xs_HL = [0 0 0 0 0]';
                %     p_vector_HL(n_states_HL*(k-1)+1:n_states_HL*k,1) = xs_HL;
                % end
    
    
                for k=1:obj.N_c
                    % controls = [u_ref;u_tar;r_ref];
                    us_HL = [u_d u_d 0]';
                    p_vector(obj.n_states+obj.n_states*obj.N_p+obj.n_controls*(k-1)+1:obj.n_states+obj.n_states*obj.N_p+obj.n_controls*k,1) = us_HL;
                end
    
                p_vector(obj.n_states*(obj.N_p+1)+obj.n_controls*obj.N_c+1:obj.n_states*(obj.N_p+1)+obj.n_controls*obj.N_c+obj.n_controls,1) = obj.U_ant;
    
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
    
                % g_val = full(sol.g);  % convertir a vector numérico si está en formato SX o MX
                % for i = 1:length(g_val)
                %     if g_val(i) < obj.args.lbg(i)  || g_val(i) > obj.args.ubg(i)
                %         fprintf('⚠️ Restricción %d violada: valor = %.6f, rango = [%.6f, %.6f]\n', ...
                %             i, g_val(i), obj.args.lbg(i), obj.args.ubg(i));
                %     end
                % end
                % 
                % lam_g_val = full(sol.lam_g);
                % 
                % for i = 1:length(lam_g_val)
                %     if abs(lam_g_val(i)) > 1  % puedes ajustar el umbral
                %         fprintf('Restricción %d tiene multiplicador λ = %.4f\n', i, lam_g_val(i));
                %     end
                % end
                % Repetición de la optimización con solución inicial por defecto, en caso de fracaso:
            
                if obj.success == 0
                    tic
                    x0 = reshape(obj.U0_HL',obj.n_controls*obj.N_c,1); % initial value of the optimization variables
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
                    % controls = [u_ref;u_tar;r_ref];
                    obj.U_ant = obj.u_nc(1,:);
                    obj.j_min = -1;
                else
                    obj.u_nc = reshape(full(sol.x)',obj.n_controls,obj.N_c)';
                    % obj.u_nc(:, 1) = [obj.u_nc(4:end, 1); obj.u_nc(end, 1); obj.u_nc(end, 1); obj.u_nc(end, 1)];
                    % obj.u_nc(:, 3) = [obj.u_nc(4:end, 3); obj.u_nc(end, 3); obj.u_nc(end, 3); obj.u_nc(end, 3)];
                    obj.U_ant = obj.u_nc(1,:);
                    obj.j_min = full(sol.f);
                end
                obj. u_tar_next = obj.u_nc(1,2);
    
                % Controlador de bajo nivel: Extracción de las acciones de control óptimas:
                % ff_value_LL = obj.ff_opt(U_HL',p_vector);
                
                %Ajustar salida a la frecunecia del simulador
                u_nc_llc = kron(obj.u_nc(:,1:3), ones(obj.R_T, 1));
    
                % Salida
                refs= reshape(u_nc_llc.', [], 1);
                obj.sincronizador = 1;
            else
                %Ajustar salida a la frecunecia del simulador
                u_nc_llc = kron(obj.u_nc(:,1:3), ones(obj.R_T, 1));
                shift = 3 * obj.sincronizador;
                % Salida
                ref_last= reshape(u_nc_llc.', [], 1);
                refs = [ref_last(shift+1:end); repmat(ref_last(end), shift, 1)];
                obj.sincronizador = obj.sincronizador + 1;
            end
        end
    end
end
