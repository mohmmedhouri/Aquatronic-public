classdef Otter < ASVbase
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Autor: Manuel Eduardo Gantiva Osorio
    % Proyecto: AQUATRONIC
    % Escuela Técnica Superior de Ingeniería 
    % Universidad Loyola Andalucía
    % Fecha: 24.04.2025
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Otter is a subclass of ASVbase that implements the Otter vessel model based on the
    % formulation proposed by Fossen. It is a full six degrees of freedom dynamic model.
    % For correct functionality, it requires the MSS (Marine Systems Simulator) library
    % to be installed.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    properties(Nontunable)
        % Parámetros físicos y geométricos

        m
        mp
        Ig

        % === Matrices dinámicas ===
        MA         % Added mass matrix
        H
        M          % Total mass matrix (MRB + MA)
        G          % Hydrostatic stiffness matrix
        invM

        % === Trim parameters ===
        g_payload        % Vector [0; 0; mp*g]
        m_payload_base   % Precomputed moment: Smtrx(rp) * g_payload
        invG_trim        % Inverse of G(3:5,3:5)
        
        % === Linear damping coefficients ===
        Xu    % Surge damping
        Yv    % Sway damping
        Zw    % Heave damping
        Kp    % Roll damping
        Mq    % Pitch damping
        Nr    % Yaw damping

        X_u_abs_u   % Parámetro hidrodinámico X_u_abs_u (kg/m)           
      
        Y_v_abs_v  % Parámetro hidrodinámico Y_v_abs_v (kg/m)           
        Y_v_abs_r  % Parámetro hidrodinámico Y_v_abs_r (kg)      
        
        Y_r        % Parámetro hidrodinámico Y_r (kg m/s)           
        Y_r_abs_v  % Parámetro hidrodinámico Y_r_abs_v (kg)           
        Y_r_abs_r  % Parámetro hidrodinámico Y_r_abs_r (kg m)      
        
        N_v        % Parámetro hidrodinámico N_v (kg m/s)           
        N_v_abs_v  % Parámetro hidrodinámico N_v_abs_v (kg)           
        N_v_abs_r  % Parámetro hidrodinámico N_v_abs_r (kg m)   
         
        N_r_abs_v  % Parámetro hidrodinámico N_r_abs_v (kg m)           
        N_r_abs_r  % Parámetro hidrodinámico N_r_abs_r (kg m2)
        
        
        % === Geometry (for crossflow drag) ===
        L         % Length of the vessel
        B_pont    % Pontoon beam width
        T         % Draft
        
        y_pont
        
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

        wind_wave_params   % Vector de parámetros de viento y oleaje (2x1)
        

        % Propulsión
        k_pos                        
        k_neg 
        maxRPM
        minRPM
        
        maxThrust 
        minThrust 
        
        maxForce_u 
        minForce_u 
        maxTorque_r
    end  

    methods
        function obj = Otter(vessel, X_0, observer_param, wind_params)

            % Añadir informacion nesesaria para el observador
            observer_param.invM = vessel.invM;
            observer_param.X_0 = [X_0(1:2);X_0(6:8);X_0(12)];

            obj  = obj@ASVbase("Otter", X_0, observer_param);  
            
            obj.m = vessel.mass;
            obj.mp = vessel.mp;
            obj.Ig = vessel.Ig;

            % === Dynamic matrices ===
            obj.MA  = vessel.MA;
            obj.H = vessel.H;
            obj.M   = vessel.M;
            obj.G   = vessel.G;
            obj.invM = inv(obj.M);
        
            % === Trim compensation ===
            obj.g_payload        = vessel.g_payload;
            obj.m_payload_base   = vessel.m_payload_base;
            obj.invG_trim        = vessel.invG_trim;
        
            % === Linear damping coefficients ===
            obj.Xu = vessel.X_u;
            obj.Yv = vessel.Y_v;
            obj.Zw = vessel.Zw;
            obj.Kp = vessel.Kp;
            obj.Mq = vessel.Mq;
            obj.Nr = vessel.N_r;

            % === Non-Linear damping coefficients ===
            
            obj.X_u_abs_u  = vessel.X_u_abs_u;     
            obj.Y_v_abs_v =  vessel.Y_v_abs_v;        
            obj.Y_v_abs_r =  vessel.Y_v_abs_r; 
            obj.Y_r       =  vessel.Y_r;          
            obj.Y_r_abs_v =  vessel.Y_r_abs_v;      
            obj.Y_r_abs_r =  vessel.Y_r_abs_r;    
            obj.N_v       =  vessel.N_v;           
            obj.N_v_abs_v =  vessel.N_v_abs_v;       
            obj.N_v_abs_r =  vessel.N_v_abs_r;    
            obj.N_r_abs_v =  vessel.N_r_abs_v;         
            obj.N_r_abs_r =  vessel.N_r_abs_r;

        
            % === Geometry (used in crossFlowDrag) ===
            obj.L       = vessel.L;
            obj.B_pont  = vessel.B;
            obj.T       = vessel.T;

            obj.y_pont = vessel.y_pont;
            
            obj.k_pos = vessel.k_pos;
            obj.k_neg  = vessel.k_neg; 
            obj.maxRPM = vessel.maxRPM;
            obj.minRPM = vessel.minRPM;
            obj.maxThrust  = vessel.maxThrust;
            obj.minThrust = vessel.minThrust;
            obj.maxForce_u  = vessel.maxForce_u;
            obj.minForce_u = vessel.minForce_u;
            obj.maxTorque_r = vessel.maxTorque_r;


             % === Geometry (used in Wind Disturbances) ===

            obj.sL = vessel.lateralAreaCentroidY;
            obj.Loa  = vessel.totalLength;
            obj.AFw = vessel.frontalArea;
            obj.ALw = vessel.lateralArea;
            obj.af_al = obj.AFw/obj.ALw;
            obj.sl_loa = obj.sL/obj.Loa;

            obj.const_tauX = 0.5*1.1960*obj.AFw;
            obj.const_tauY = 0.5*1.1960*obj.ALw;
            obj.const_tauN = 0.5*1.1960*obj.ALw*obj.Loa;

            % Inicializamos con valores por defecto.
            obj.wind_wave_params(1) = wind_params(1);
            obj.wind_wave_params(2) = wind_params(2);

        end
    end

    methods(Static, Access = private)
        
        function e = wind_disturbance(psi, u, v, V_w, beta_w, af_al_, sl_loa_,const_tauX_, const_tauY_, const_tauN_)
            %% 3. Cálculo de las fuerzas y momentos debido al viento:
            u_r_w = u - V_w*cos(beta_w - psi);
            v_r_w = v - V_w*sin(beta_w - psi);
            V_r = sqrt(u_r_w^2 + v_r_w^2);
            gamma_r = - atan2(v_r_w,u_r_w);


            if(abs(gamma_r)<= pi/2)
                CD_l_AF = 0.55;
            else
                CD_l_AF = 0.65;
            end

            % wind coefficients
            CD_l = CD_l_AF*af_al_;
            den = 1-0.5*0.60*(1-CD_l/0.85)*sin(2*gamma_r)^2;

            CX = -CD_l_AF*cos(gamma_r)/den;
            CY =  0.85*sin(gamma_r)/den;
            CN =  (sl_loa_ - 0.18*(gamma_r - pi/2))*CY;

            e = [const_tauX_ * CX*V_r^2;
                const_tauY_ * CY*V_r^2;
                0;
                0;
                0;
                const_tauN_ * CN*V_r^2 ];

        end

    end

    methods(Access = public)
    
        function setThrust(obj, delta)
            if ~isequal(size(delta), [2, 1])
                error('El vector delta debe ser de 2x1');
            end
            Trusters = zeros(2,1);
            % Recorremos cada elemento de delta
            for i = 1:length(delta)
                if delta(i) > obj.maxThrust
                    Trusters(i) = obj.maxThrust;
                elseif delta(i) < obj.minThrust
                    Trusters(i) = obj.minThrust;
                else
                    Trusters(i) = delta(i) ;
                end
            end
            obj.Deltas = Trusters;
            
            obj.Tau = [Trusters(1) + Trusters(2) 0 obj.y_pont * Trusters(1) - obj.y_pont * Trusters(2) ]';
        end

        function setTao(obj, tau)
            if ~isequal(size(tau), [3, 1])
                error('El vector tau debe ser de 3x1');
            end
            obj.Tau = tau;
            if (obj.Tau(1) > obj.maxForce_u)
                obj.Tau(1) = obj.maxForce_u;
            end
            if (obj.Tau(1) < obj.minForce_u)
                obj.Tau(1) = obj.minForce_u;
            end
            if (obj.Tau(3) > obj.maxTorque_r)
                obj.Tau(3) = obj.maxTorque_r;
            end
            if (obj.Tau(3) < -obj.maxTorque_r)
                obj.Tau(3) = -obj.maxTorque_r;
            end

            f_u = obj.Tau(1);
            t_r = obj.Tau(3);
            Thrust = zeros(2,1);
            Thrust(1) = (f_u*obj.y_pont + t_r)/(2*obj.y_pont); % Force Left
            Thrust(2) = (f_u*obj.y_pont - t_r)/(2*obj.y_pont); % Force Right
            obj.Deltas = Thrust;
            
        end

    end
        

    methods (Access = protected)
        function xdot = stepImpl(obj, x, Vc_params)
            % Dinámica del sistema
            % x = [ u v w p q r x y z phi theta psi ]' 
            % x = [ x y z phi theta psi u v w p q r]' 

            % === Extracción de corrientes en Magnitud y angulo ===
            V_c = sqrt(Vc_params(1)^2 + Vc_params(2)^2);
            beta_c = atan2(Vc_params(2), Vc_params(1));
            psi = x(6);

            % === Extracción de estados ===
            nu = x(7:12);
            eta = x(1:6);
            nu1 = nu(1:3); 
            nu2 = nu(4:6);

            U = norm(nu);
            u_c = V_c * cos(beta_c - eta(6));
            v_c = V_c * sin(beta_c - eta(6));
            nu_c = [u_c; v_c; 0; 0; 0; 0];
            nu_r = nu - nu_c;

            % === Masa añadida ===
            
            CRB_CG = [ (obj.m+obj.mp) * Smtrx(nu2)         zeros(3,3)
                        zeros(3,3)               -Smtrx(obj.Ig*nu2)  ];

            CRB = obj.H' * CRB_CG * obj.H;

            CA  = m2c(obj.MA, nu_r);
            
            % Uncomment to cancel the Munk moment in yaw, if stability problems
            % CA(6,1) = 0; 
            % CA(6,2) = 0; 
            % CA(1,6) = 0;
            % CA(2,6) = 0;

            C = CRB + CA;

            % === Wind ===

            if obj.wind_wave_params(1) ~= 0
                u = nu(1);
                v = nu(2);
                beta_w = obj.wind_wave_params(2);

                e = obj.wind_disturbance(psi, u, v, obj.wind_wave_params(1),...
                    beta_w, obj.af_al, obj.sl_loa, obj.const_tauX,...
                    obj.const_tauY, obj.const_tauN);
                %                 end
            else
                e = [0;0;0;0;0;0];
            end
                  

            % === Trim ===
            
            f_payload = Rzyx(eta(4), eta(5), eta(6))' * obj.g_payload;
            m_payload = obj.m_payload_base * f_payload;
            g_0 = [f_payload; m_payload];
            eta_0 = [0; 0; obj.invG_trim * g_0(3:5); 0];
            eta = eta - eta_0;

            % === Amortiguamiento lineal ===
            % tau_damp = [obj.Xu*nu_r(1); 
            %             obj.Yv*nu_r(2);
            %             obj.Zw*nu_r(3);
            %             obj.Kp*nu_r(4); 
            %             obj.Mq*nu_r(5);
            %             obj.Nr*(1 + 10*abs(nu_r(6))) * nu_r(6)];

            % === Amortiguamiento no-lineal ===
            D = [obj.Xu, 0, 0, 0, 0, 0; 
                 0, obj.Yv + obj.Y_v_abs_v*abs(nu_r(2)) + obj.Y_v_abs_r*abs(nu_r(6)), 0, 0, 0, obj.Y_r + obj.Y_r_abs_v*abs(nu_r(2)) + obj.Y_r_abs_r*abs(nu_r(6));
                 0, 0, obj.Zw, 0, 0, 0;
                 0, 0, 0, obj.Kp, 0, 0; 
                 0, 0, 0, 0, obj.Mq, 0;
                 0, obj.N_v + obj.N_v_abs_v*abs(nu_r(2)) + obj.N_v_abs_r*abs(nu_r(6)), 0, 0, 0, obj.Nr + obj.N_r_abs_v*abs(nu_r(2)) + obj.N_r_abs_r*abs(nu_r(6))];
            tau_damp = D*nu_r;
     
            tau = [obj.Tau(1); 0; 0; 0; 0; obj.Tau(3)];

            % === Otras fuerzas ===
            tau_crossflow = crossFlowDrag(obj.L, obj.B_pont, obj.T, nu_r);
            nu_c_dot = [-Smtrx(nu2)*nu_c(1:3); zeros(3,1)];
            J = eulerang(eta(4), eta(5), eta(6));

            sigmas6 = nu_c_dot + obj.invM * (tau_damp + tau_crossflow - C*nu_r - obj.G*eta + e); 

            nu_dot = obj.invM * (tau)+ sigmas6;
            obj.sig = [sigmas6(1:2);sigmas6(6)];

            obj.state(7:8) = nu_dot(1:2); % Actualizo las acceleraciones

            % === Ecuaciones de movimiento ===
            xdot = [J * nu ;
                    nu_dot];
        end
    end
end