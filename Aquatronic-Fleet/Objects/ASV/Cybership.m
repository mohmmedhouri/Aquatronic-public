classdef Cybership < ASVbase
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Autor: Manuel Eduardo Gantiva Osorio
    %        Guillermo Bejarano Pellicer
    %        Federico Peralta Samaniego
    % Proyecto: AQUATRONIC
    % Escuela Técnica Superior de Ingeniería 
    % Universidad Loyola Andalucía
    % Fecha: 24.04.2025
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Cybership is a subclass of ASVbase that iimplements the Cybership II vessel model with 
    % full dynamic behaviour. It includes three degrees of freedom—surge, sway, and yaw—and
    % incorporates mass properties, damping, and added mass to simulate realistic vessel
    % dynamics.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Public, tunable properties
    properties(Nontunable)
        %% 1.1. Parámetros másicos, inerciales y geométricos:
        mass
        Iz
        xG

        %% 1.2. Parámetros hidrodinámicos:
        X_udot
        Y_vdot
        Y_rdot
        N_vdot
        N_rdot
        X_u
        X_u_abs_u
        Y_v
        Y_v_abs_v
        Y_v_abs_r
        Y_r
        Y_r_abs_v
        Y_r_abs_r
        N_v
        N_v_abs_v
        N_v_abs_r
        N_r
        N_r_abs_v
        N_r_abs_r

        %% 1.3. Límites en las acciones de control:

        y_pont 
        k_pos                        
        k_neg 
        maxRPM
        minRPM
        
        maxThrust 
        minThrust 
        
        maxForce_u 
        minForce_u 
        maxTorque_r

        %% 1.4. Matrices y elementos derivados de los parámetros
        % 1.4.1 Matriz de inercia:
        M
        invM

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
        
    end

   
    methods
        function obj = Cybership(vessel, X_0, observer_param, wind_params)
            
            % Añadir informacion nesesaria para el observador
            observer_param.invM = vessel.invM;
            observer_param.X_0 = [X_0(1:2);X_0(6:8);X_0(12)];

            obj  = obj@ASVbase("Cybership", X_0, observer_param);

            obj.mass = vessel.mass;
            obj.Iz = vessel.rotationalInertia;
            obj.xG= vessel.centerOfGravityDistanceX;
            obj.X_udot = vessel.X_udot;
            obj.Y_vdot = vessel.Y_vdot;
            obj.Y_rdot = vessel.Y_rdot;
            obj.N_vdot = vessel.N_vdot;
            obj.N_rdot = vessel.N_rdot;
            obj.X_u = vessel.X_u;
            obj.X_u_abs_u = vessel.X_u_abs_u;
            obj.Y_v = vessel.Y_v;
            obj.Y_v_abs_v = vessel.Y_v_abs_v;
            obj.Y_v_abs_r = vessel.Y_v_abs_r;
            obj.Y_r = vessel.Y_r;
            obj.Y_r_abs_v = vessel.Y_r_abs_v;
            obj.Y_r_abs_r = vessel.Y_r_abs_r;
            obj.N_v = vessel.N_v;
            obj.N_v_abs_v = vessel.N_v_abs_v;
            obj.N_v_abs_r = vessel.N_v_abs_r;
            obj.N_r =vessel.N_r;
            obj.N_r_abs_v = vessel.N_r_abs_v;
            obj.N_r_abs_r = vessel.N_r_abs_r;

            obj.M = [obj.mass - obj.X_udot     0    0;
                0   obj.mass - obj.Y_vdot  obj.mass*obj.xG - obj.Y_rdot;
                0   obj.mass*obj.xG - obj.N_vdot  obj.Iz - obj.N_rdot];
            obj.invM = inv(obj.M);

            % Initial condition

            obj.sL = vessel.lateralAreaCentroidY;
            obj.Loa  = vessel.totalLength;
            obj.AFw = vessel.frontalArea;
            obj.ALw = vessel.lateralArea;
            obj.af_al = obj.AFw/obj.ALw;
            obj.sl_loa = obj.sL/obj.Loa;

            obj.const_tauX = 0.5*1.1960*obj.AFw;
            obj.const_tauY = 0.5*1.1960*obj.ALw;
            obj.const_tauN = 0.5*1.1960*obj.ALw*obj.Loa;
                      
            obj.y_pont  = vessel.y_pont;
            obj.k_pos = vessel.k_pos;
            obj.k_neg  = vessel.k_neg; 
            obj.maxRPM = vessel.maxRPM;
            obj.minRPM = vessel.minRPM;
            obj.maxThrust  = vessel.maxThrust;
            obj.minThrust = vessel.minThrust;
            obj.maxForce_u  = vessel.maxForce_u;
            obj.minForce_u = vessel.minForce_u;
            obj.maxTorque_r = vessel.maxTorque_r;

            % Inicializamos con valores por defecto.
            obj.wind_wave_params(1) = wind_params(1);
            obj.wind_wave_params(2) = wind_params(2);
   
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
            Thrust(1) = (f_u*obj.y_pont + t_r)/(2*obj.y_pont);
            Thrust(2) = (f_u*obj.y_pont - t_r)/(2*obj.y_pont);
            
            obj.Deltas = Thrust;
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
                const_tauN_ * CN*V_r^2 ];

        end

    end

    methods (Access = protected)

        %         function
        %         [xdot,beta_r,sigma_r,nu_c,nu_c_dot,beta_c,nu,nu_dot,beta,sigma,sigma_r_dot,sigma_dot]
        %         = stepImpl(obj, X_r,tau,e,e_dot,Vc,Vc_dot,Vc_dot2)
        %         function xdot = stepImpl(obj, X_r,tau,e,e_dot,Vc,Vc_dot,Vc_dot2)
        function xdot = stepImpl(obj, X_r,Vc_params)
            % x = [ x y z phi theta psi u v w p q r]' 

            % === Extracción de corrientes en Magnitud y angulo ===
            % V_c = sqrt(Vc_params(1)^2 + Vc_params(2)^2);
            % beta_c = atan2(Vc_params(2), Vc_params(1));

            % 2. Detalle del estado inicial del sistema:
            %             x = X_r(1);
            %             y = X_r(2);
            psi = X_r(6);
            %             eta = [x;y;psi];

            u = X_r(7);
            v = X_r(8);
            r = X_r(12);
            nu = [u;v;r];

            nu2 = [0; 0; r];

            V_u_c = Vc_params(1); % V_c * cos(beta_c);
            V_v_c = Vc_params(2); % V_c * sin(beta_c);
            V_n_c = [V_u_c; V_v_c; 0];
            


            % 3. Cálculo de matrices dinámicas de movimiento del barco:
            ROT = [cos(psi),    - sin(psi),      0;
                sin(psi),      cos(psi),      0;
                0,             0,      1];
            nu_c = ROT'*V_n_c;
            nu_r = nu - nu_c;
            u_r = nu_r(1);
            v_r = nu_r(2);

            c13 = - obj.M(2,2)* v_r - obj.M(2,3)*r;
            c23 = obj.M(1,1)*u_r;

            C = [   0,       0,     c13;
                0,       0,     c23;
                -c13,    -c23,       0];


            D = [- obj.X_u - obj.X_u_abs_u*abs(u_r),        0,       0;
                0,      - obj.Y_v - obj.Y_v_abs_v*abs(v_r) - obj.Y_v_abs_r*abs(r),     - obj.Y_r - obj.Y_r_abs_v*abs(v_r) - obj.Y_r_abs_r*abs(r);
                0,      - obj.N_v - obj.N_v_abs_v*abs(v_r) - obj.N_v_abs_r*abs(r),     - obj.N_r - obj.N_r_abs_v*abs(v_r) - obj.N_r_abs_r*abs(r)];

            % 4. Implementación del modelo dinámico:
            if obj.wind_wave_params(1) ~= 0
                % nu = nu_r + nu_c;
                % u = nu(1);
                % v = nu(2);
                beta_w = obj.wind_wave_params(2);

                e = obj.wind_disturbance(psi, u, v, obj.wind_wave_params(1),...
                    beta_w, obj.af_al, obj.sl_loa, obj.const_tauX,...
                    obj.const_tauY, obj.const_tauN);
                %                 end
            else
                e = [0;0;0];
            end

            eta_dot = ROT*(nu);

            % S = [0 -1 0;
            %      1  0 0;
            %      0  0 0];
            % nu_c_dot = r*S'*nu_c;
            nu_c_dot = [-Smtrx(nu2)*nu_c(1:3)];

            obj.sig = obj.invM*(-(C+D)*nu_r + e) + nu_c_dot;

            nu_dot = obj.invM*(obj.Tau) + obj.sig;

            obj.state(7:8) = nu_dot(1:2); % Actualizo las acceleraciones
            xdot = [eta_dot(1:2); zeros(3,1) ; eta_dot(3); ...
                    nu_dot(1:2); zeros(3,1); nu_dot(3)];
        end
    end
end
