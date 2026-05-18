classdef FleetController < handle
    % FleetController
    %
    % W-based fleet controller.
    %
    % This controller follows the paper idea:
    %
    %   s = [s_m; s_R; s_L]
    %
    % In this code:
    %
    %   s_R = w_1
    %   s_L = w_2
    %
    % It coordinates the virtual target path variables w of both ASVs and
    % outputs desired surge commands:
    %
    %   fleet_ref.u_d(1) -> ASV 1
    %   fleet_ref.u_d(2) -> ASV 2
    %
    % These commands are passed to:
    %
    %   MpcPFDisController.step(u_d_v, estimation, disturbances_mod)

    properties
        no_vessels

        % Nominal fleet speed [m/s]
        u_fleet

        % Speed limits [m/s]
        u_min
        u_max

        % Synchronization gain for w error
        k_w

        % Closed path settings
        closed_path
        w_period

        % Desired phase/progress offset
        % [0;0] means w1 and w2 should be equal
        w_offset

        % Smoothing
        alpha_u
        u_last
    end

    methods
        function obj = FleetController(no_vessels)

            if no_vessels ~= 2
                error('FleetController currently supports exactly two ASVs.');
            end

            obj.no_vessels = no_vessels;

            % Nominal fleet speed
            obj.u_fleet = 1.0;

            % Compatible with MpcPFDisController limits
            obj.u_min = 0.4;
            obj.u_max = 1.8;

            % Synchronization gain
            obj.k_w = 0.25;

            % For circular/concentric paths use true
            obj.closed_path = true;
            obj.w_period = 2*pi;

            % Same path progress / same angular value if w is angular
            obj.w_offset = [0; 0];

            % Command smoothing
            obj.alpha_u = 0.4;
            obj.u_last = obj.u_fleet * ones(no_vessels,1);
        end

        function fleet_ref = step(obj, mMLCs_dis)

            % -------------------------------------------------------------
            % 1. Read current virtual-target path variables w
            % -------------------------------------------------------------
            w = zeros(obj.no_vessels,1);

            for i = 1:obj.no_vessels
                w(i) = mMLCs_dis{i}.getCurrent_w();
            end

            % -------------------------------------------------------------
            % 2. Read local path geometry f_i = sqrt(dx^2 + dy^2)
            % -------------------------------------------------------------
            f = zeros(obj.no_vessels,1);

            for i = 1:obj.no_vessels

                path_i = mMLCs_dis{i}.path;

                if obj.closed_path
                    w_eval = mod(w(i), obj.w_period);
                else
                    w_eval = w(i);
                end

                dx = path_i.dx_p_dw(w_eval);
                dy = path_i.dy_p_dw(w_eval);

                f(i) = sqrt(dx^2 + dy^2);

                % Numerical protection
                f(i) = max(f(i), 1e-6);
            end

            % Meta-vehicle path metric
            f_m = mean(f);

            % -------------------------------------------------------------
            % 3. Geometry-scaled base speeds
            %
            % Paper idea:
            %   u_ref_delta = f_delta / f_m * u_fleet
            %
            % This makes the outer/larger path faster.
            % -------------------------------------------------------------
            u_base = (f / f_m) * obj.u_fleet;

            % -------------------------------------------------------------
            % 4. Synchronization using w
            % -------------------------------------------------------------
            w_sync = w - obj.w_offset;

            if obj.closed_path

                w_phase = mod(w_sync, obj.w_period);

                % For two vehicles, use circular mean of the two path phases
                w_ref = obj.circularMean(w_phase, obj.w_period);

                w_error = zeros(obj.no_vessels,1);

                for i = 1:obj.no_vessels
                    w_error(i) = obj.wrapToPeriod(w_ref - w_phase(i), obj.w_period);
                end

            else

                w_ref = mean(w_sync);
                w_error = w_ref - w_sync;

            end

            % -------------------------------------------------------------
            % 5. Convert w-error into speed correction
            %
            % If ASV is behind: w_error > 0 -> speed increases
            % If ASV is ahead:  w_error < 0 -> speed decreases
            % -------------------------------------------------------------
            u_cmd = u_base + obj.k_w * w_error;

            % -------------------------------------------------------------
            % 6. Saturation
            % -------------------------------------------------------------
            u_cmd = min(max(u_cmd, obj.u_min), obj.u_max);

            % -------------------------------------------------------------
            % 7. Smoothing
            % -------------------------------------------------------------
            u_cmd = obj.alpha_u * obj.u_last + (1 - obj.alpha_u) * u_cmd;
            obj.u_last = u_cmd;

            % -------------------------------------------------------------
            % 8. Output
            % -------------------------------------------------------------
            fleet_ref = struct();

            % Main output to MpcPFDisController
            fleet_ref.u_d = u_cmd;

            % Diagnostics
            fleet_ref.w = w;
            fleet_ref.w_sync = w_sync;
            fleet_ref.w_ref = w_ref;
            fleet_ref.w_error = w_error;

            fleet_ref.f = f;
            fleet_ref.f_m = f_m;
            fleet_ref.u_base = u_base;

            % For future boom/towing correction. Keep zero for now.
            fleet_ref.tao_e = zeros(3, obj.no_vessels);
        end
    end

    methods (Static, Access = private)

        function e = wrapToPeriod(e, period)
            e = mod(e + period/2, period) - period/2;
        end

        function w_mean = circularMean(w, period)

            angle = 2*pi*w/period;

            s = mean(sin(angle));
            c = mean(cos(angle));

            w_mean = atan2(s,c) * period/(2*pi);

            if w_mean < 0
                w_mean = w_mean + period;
            end
        end
    end
end