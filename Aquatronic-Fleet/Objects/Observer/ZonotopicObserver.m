classdef ZonotopicObserver < matlab.System
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Autor: Manuel Eduardo Gantiva Osorio
    %        Federico Peralta Samaniego
    % Proyecto: AQUATRONIC
    % Escuela Técnica Superior de Ingeniería 
    % Universidad Loyola Andalucía
    % Fecha: 24.04.2025
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % This class implements the zonotopic state observer proposed by Orihuela (2022), with
    % a modification that includes angular velocity measurements. It considers noise in all
    % measurements, providing robust state estimation under uncertain and noisy conditions.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    properties (Access=private) 
        zr
        zp

        Cr
        Cp
        Rr
        Rp

        orden_est_r
        orden_est_p

        Qr
        Qp
       
        Ar
        Bwr
        Br
        Ap
        Bwp
        Bp

        Wp      %Diagonal
        Wr    %Diagonal

        prev_estimations        %Estado Estimados del barco state = [ x y psi u v r sigma_u sigma_v sigma_r]' 
    end

    properties (Nontunable)
        type
        
        Lbar_psi
        PpWp
        invM
        Ts
    end

    methods
        function obj = ZonotopicObserver(observer_param)

            obj.type = "DualZonotopic";
            obj.Ts = observer_param.Ts;
            obj.invM = observer_param.invM;
            obj.prev_estimations = [observer_param.X_0;zeros(3,1)]; %Posiciones, velocidades y perturbaciones

            cr0 = [observer_param.X_0(3); 0; 0];
            Hr0 = 1*eye(3);
            cp0 = [observer_param.X_0(1:2); zeros(4,1)];
            Hp0 = 1*eye(6);

            Max_n_r = observer_param.Max_n_r;
            Max_n_p = observer_param.Max_n_p ;
            orden_est_r = observer_param.orden_est_r;
            orden_est_p = observer_param.orden_est_p;
            Max_w_r = observer_param.Max_w_r;
            Max_w_p = observer_param.Max_w_p;
            obj.Wr = observer_param.Wr;
            obj.Wp = observer_param.Wp;
            
            obj.zr = Zonotopo(cr0, Hr0);
            obj.zp = Zonotopo(cp0, Hp0);

            obj.Cr = [eye(2) zeros(2,1)];
            obj.Cp = [eye(2) zeros(2) zeros(2)];
            obj.Rr = diag(Max_n_r);
            obj.Rp = diag(Max_n_p);
            obj.orden_est_r = orden_est_r;
            obj.orden_est_p = orden_est_p;

            obj.Qr = Max_w_r*eye(1);
            obj.Qp = Max_w_p*eye(2);
        

            obj.Ar = [1 obj.Ts 0;
                      0 1  obj.Ts;
                      0 0  1];
            obj.Br = [zeros(1,3);obj.Ts*obj.invM(3,:);zeros(1,3)];
            obj.Bwr = [0;0;obj.Ts];

            obj.Ap = [eye(2)   obj.Ts*eye(2) zeros(2);
                      zeros(2) eye(2)    obj.Ts*eye(2);
                     zeros(2) zeros(2)  eye(2)];
            obj.Bp = [zeros(2,3);obj.Ts*obj.invM(1:2,:);zeros(2,3)];
            obj.Bwp = [zeros(2);zeros(2); obj.Ts*eye(2)];

        end

        function states_hat = getEstimations(obj)
            states_hat = obj.prev_estimations;

        end
    end
    methods(Access = protected)
        function stepImpl(obj, eta_noise, tau_n)

            % Variables
            x_y = eta_noise(1:2);
            r_p = eta_noise(3:4);
            tau = [tau_n(1);0;tau_n(2)];

            % Filter
            obj.zr.filtering(r_p,obj.Cr,obj.Rr);
            obj.zp.filtering(x_y,obj.Cp,obj.Rp);


            % Order Reduction
            obj.zr.reduccion_orden(obj.orden_est_r,diag(obj.Wr))
            obj.zp.reduccion_orden(obj.orden_est_p,diag(obj.Wp));

            % Prediction
            Ar_ = obj.Ar;
            qr = Ar_*obj.zr.c + obj.Br*tau;
            Hr = [Ar_*obj.zr.H  obj.Bwr*obj.Qr];
            obj.zr.update_c_H(qr, Hr);

            c = obj.zp.c;
            H = obj.zp.H;
            [c1, s1] = obj.max_min_trig(r_p(1)-obj.Rr(1,1),r_p(1)+obj.Rr(1,1));
            [rA,mA] = obj.rad_mid_1(obj.Ap,c1, s1);
            c_pre = mA*c + obj.Bp*tau;
            H1 = mA*H;
            H2 = obj.rs(rA*abs(H));
            H3 = obj.rs(rA*abs(c));
            HZ = [H1 H2 H3];
            num_ceros = 0;
            Hn = zeros(size(HZ,1),size(HZ,2));
            l = 1;
            for k =1:size(HZ,2)
                if HZ(:,k)==0
                    num_ceros = num_ceros +1;
                else
                    Hn(:,l)=HZ(:,k);
                    l = l+1;
                end
            end
            Hn = Hn(:,1:end-num_ceros);
            H_pre = cat(2,Hn,obj.Bwp*obj.Qp);
            obj.zp.update_c_H(c_pre, H_pre);

            % Output Estimation
            eta = [obj.zp.c(1:2);obj.zr.c(1)]; % eta estimada
            nu = [obj.zp.c(3:4);obj.zr.c(2)]; % nu estimada
            sigma = [obj.zp.c(5:6);obj.zr.c(3)]; % sigma estimada
            obj.prev_estimations = [eta;nu;sigma];
        end
    end
    methods (Static, Access=private)

        function [rA,mA] = rad_mid_1(Ap,c,s)
            Rmin = [c(1) -s(2);
                s(1) c(1)];

            Rmax = [c(2) -s(1);
                s(2) c(2)];

            ARmin = Ap;
            ARmin(1:2,3:4) = ARmin(1:2,3:4)*Rmin;

            ARmax = Ap;
            ARmax(1:2,3:4) = ARmax(1:2,3:4)*Rmax;

            mA = (ARmin+ARmax)/2;
            rA = (ARmax-ARmin)/2;

        end

        function M = rs(H)
            [fil,col] = size(H);
            M = zeros(fil);
            for i=1:fil
                for j=1:col
                    M(i,i) = M(i,i) + abs(H(i,j));
                end
            end
        end

        function [c,s] = max_min_trig(min_psi,max_psi)
            
            while min_psi>0
                min_psi = min_psi - 2*pi;
                max_psi = max_psi - 2*pi;
            end
            while min_psi<0
                min_psi = min_psi + 2*pi;
                max_psi = max_psi + 2*pi;
            end

            if min_psi < pi/2 && max_psi > pi/2
                c_min = cos(max_psi);
                c_max = cos(min_psi);
                s_min = min(sin(min_psi),sin(max_psi));
                s_max = 1;
            elseif min_psi < pi && max_psi > pi
                c_min = -1;
                c_max = max(cos(min_psi),cos(max_psi));
                s_min = sin(max_psi);
                s_max = sin(min_psi);
            elseif min_psi < 3*pi/2 && max_psi > 3*pi/2
                c_min = cos(min_psi);
                c_max = cos(max_psi);
                s_min = -1;
                s_max = max(sin(min_psi),sin(max_psi));
            elseif min_psi < 2*pi && max_psi > 2*pi
                c_min = min(cos(min_psi),cos(max_psi));
                c_max = 1;
                s_min = sin(min_psi);
                s_max = sin(max_psi);
            else
                c_min = min(cos(min_psi),cos(max_psi));
                c_max = max(cos(min_psi),cos(max_psi));
                s_min = min(sin(min_psi),sin(max_psi));
                s_max = max(sin(min_psi),sin(max_psi));
            end

            c = zeros(1,2);
            s = zeros(1,2);
            c(1) = c_min;
            c(2) = c_max;
            s(1) = s_min;
            s(2) = s_max;

        end
    end
end

