%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autor: Manuel Eduardo Gantiva Osorio
% Proyecto: AQUATRONIC
% Escuela Técnica Superior de Ingeniería 
% Universidad Loyola Andalucía
% Fecha: 24.04.2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function generates a main overview plot showing the behaviour of all vehicles
% using only their state information. It does not include any control or estimation
% data, making it suitable for general visualisation regardless of the control strategy
% used.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Figure_main(hist_X,tm, m_asv, no_items)
    FontSize = 20;
    LineWidth = 3;

    no_vessels = length(m_asv);
    
    f = figure('Name', ('Fleet performances'), ...
             'NumberTitle','off');
    tg = uitabgroup(f);
    
    % ---------------------------
    % 01 - 2D Trajectory (NED Frame)
    % ---------------------------
    t_traj = uitab(tg, 'Title', 'Trayectory 2D');
    ax_traj = axes(t_traj); hold(ax_traj, 'on'); grid(ax_traj, 'on');
    
    for j = 1:no_vessels
        idx_x = (j-1)*no_items + 1;
        idx_y = (j-1)*no_items + 2;
        plot(ax_traj, hist_X(idx_y, :), hist_X(idx_x, :), 'LineWidth', LineWidth);
    end
    
    xlabel(ax_traj, '$y$ [m]', 'Interpreter', 'latex', 'FontSize', FontSize);
    ylabel(ax_traj, '$x$ [m]', 'Interpreter', 'latex', 'FontSize', FontSize);
    title(ax_traj, '\textbf{Trayectory NED}', 'Interpreter', 'latex', 'FontSize', FontSize);
    legend(ax_traj, compose('ASV %d', 1:no_vessels), 'Location', 'best', 'FontSize', FontSize-2, 'Interpreter', 'latex');
    ax_traj.FontSize = FontSize;
    axis(ax_traj, 'equal');
    
    % ---------------------------
    % 02 - Position Components (x, y)
    % ---------------------------
    t_pos = uitab(tg, 'Title', 'Positions');
    axs_pos = gobjects(2,1);
    
    for k = 1:2
        axs_pos(k) = subplot(2,1,k,'Parent',t_pos); hold(axs_pos(k), 'on'); grid(axs_pos(k), 'on');
    end
    
    pos_labels = {'$x$ [m]', '$y$ [m]'};
    pos_titles = {'{x position}', '{y position}'};
    pos_indices = [1, 2];
    
    for k = 1:2
        for j = 1:no_vessels
            idx = (j-1)*no_items + pos_indices(k);
            plot(axs_pos(k), tm, hist_X(idx, :), 'LineWidth', LineWidth);
        end
        xlabel(axs_pos(k), 'Time [s]', 'Interpreter', 'latex', 'FontSize', FontSize);
        ylabel(axs_pos(k), pos_labels{k}, 'Interpreter', 'latex', 'FontSize', FontSize);
        title(axs_pos(k), ['\textbf' pos_titles{k}], 'Interpreter', 'latex', 'FontSize', FontSize);
        legend(axs_pos(k), compose('ASV %d', 1:no_vessels), 'Location', 'best', 'FontSize', FontSize-2, 'Interpreter', 'latex');
        axs_pos(k).FontSize = FontSize;
    end
    
    % ---------------------------
    % 03 - Heading (psi)
    % ---------------------------
    t_heading = uitab(tg, 'Title', 'Heading');
    ax_h = axes(t_heading); hold(ax_h, 'on'); grid(ax_h, 'on');
    
    for j = 1:no_vessels
        idx = (j-1)*no_items + 6;
        plot(ax_h, tm, hist_X(idx, :), 'LineWidth', LineWidth);
    end
    
    xlabel(ax_h, 'Time [s]', 'Interpreter', 'latex', 'FontSize', FontSize);
    ylabel(ax_h, '$\psi$ [rad]', 'Interpreter', 'latex', 'FontSize', FontSize);
    title(ax_h, '\textbf{Heading ($\psi$)}', 'Interpreter', 'latex', 'FontSize', FontSize);
    legend(ax_h, compose('ASV %d', 1:no_vessels), 'Location', 'best', 'FontSize', FontSize-2, 'Interpreter', 'latex');
    ax_h.FontSize = FontSize;
    
    % ---------------------------
    % 04 - Altitude and Angles (z, phi, theta)
    % ---------------------------
    t_att = uitab(tg, 'Title', 'Height and slopes');
    axs = gobjects(3,1);
    
    for k = 1:3
        axs(k) = subplot(3,1,k,'Parent',t_att); hold(axs(k), 'on'); grid(axs(k), 'on');
    end
    
    names_att = {'$z$ [m]', '$\phi$ [rad]', '$\theta$ [rad]'};
    idx_att = [3, 4, 5];
    
    for k = 1:3
        for j = 1:no_vessels
            idx = (j-1)*no_items + idx_att(k);
            plot(axs(k), tm, hist_X(idx, :), 'LineWidth', LineWidth);
        end
        xlabel(axs(k), 'Time [s]', 'Interpreter', 'latex', 'FontSize', FontSize);
        ylabel(axs(k), names_att{k}, 'Interpreter', 'latex', 'FontSize', FontSize);
        legend(axs(k), compose('ASV %d', 1:no_vessels), 'Location', 'best', 'FontSize', FontSize-2, 'Interpreter', 'latex');
        axs(k).FontSize = FontSize;
    end
    
    % ---------------------------
    % 05 - Linear and Angular Velocities
    % ---------------------------
    
    % --- Surge (u)
    t_u = uitab(tg, 'Title', 'Surge');
    ax_u = axes(t_u); hold(ax_u, 'on'); grid(ax_u, 'on');
    for j = 1:no_vessels
        idx = (j-1)*no_items + 7;
        plot(ax_u, tm, hist_X(idx, :), 'LineWidth', LineWidth);
    end
    xlabel(ax_u, 'Time [s]', 'Interpreter', 'latex', 'FontSize', FontSize);
    ylabel(ax_u, '$u$ [m s$^{-1}$]', 'Interpreter', 'latex', 'FontSize', FontSize);
    title(ax_u, '\textbf{Surge}', 'Interpreter', 'latex', 'FontSize', FontSize);
    legend(ax_u, compose('ASV %d', 1:no_vessels), 'Location', 'best', 'FontSize', FontSize-2, 'Interpreter', 'latex');
    ax_u.FontSize = FontSize;
    
    % --- Sway (v)
    t_v = uitab(tg, 'Title', 'Sway');
    ax_v = axes(t_v); hold(ax_v, 'on'); grid(ax_v, 'on');
    for j = 1:no_vessels
        idx = (j-1)*no_items + 8;
        plot(ax_v, tm, hist_X(idx, :), 'LineWidth', LineWidth);
    end
    xlabel(ax_v, 'Time [s]', 'Interpreter', 'latex', 'FontSize', FontSize);
    ylabel(ax_v, '$v$ [m s$^{-1}$]', 'Interpreter', 'latex', 'FontSize', FontSize);
    title(ax_v, '\textbf{Sway}', 'Interpreter', 'latex', 'FontSize', FontSize);
    legend(ax_v, compose('ASV %d', 1:no_vessels), 'Location', 'best', 'FontSize', FontSize-2, 'Interpreter', 'latex');
    ax_v.FontSize = FontSize;
    
    % --- Yaw rate (r)
    t_r = uitab(tg, 'Title', 'Yaw');
    ax_r = axes(t_r); hold(ax_r, 'on'); grid(ax_r, 'on');
    for j = 1:no_vessels
        idx = (j-1)*no_items + 12;
        plot(ax_r, tm, hist_X(idx, :), 'LineWidth', LineWidth);
    end
    xlabel(ax_r, 'Time [s]', 'Interpreter', 'latex', 'FontSize', FontSize);
    ylabel(ax_r, '$r$ [rad s$^{-1}$]', 'Interpreter', 'latex', 'FontSize', FontSize);
    title(ax_r, '\textbf{Yaw}', 'Interpreter', 'latex', 'FontSize', FontSize);
    legend(ax_r, compose('ASV %d', 1:no_vessels), 'Location', 'best', 'FontSize', FontSize-2, 'Interpreter', 'latex');
    ax_r.FontSize = FontSize;
    
    % --- Heave, Roll, Pitch (w, p, q)
    t_rot = uitab(tg, 'Title', 'Heave, Roll, Pitch');
    axs_rot = gobjects(3,1);
    for k = 1:3
        axs_rot(k) = subplot(3,1,k, 'Parent', t_rot); hold(axs_rot(k), 'on'); grid(axs_rot(k), 'on');
    end
    
    rot_labels = {'$w$ [m s$^{-1}$]', '$p$ [rad s$^{-1}$]', '$q$ [rad s$^{-1}$]'};
    rot_indices = [9, 10, 11];
    
    for k = 1:3
        for j = 1:no_vessels
            idx = (j-1)*no_items + rot_indices(k);
            plot(axs_rot(k), tm, hist_X(idx, :), 'LineWidth', LineWidth);
        end
        xlabel(axs_rot(k), 'Time [s]', 'Interpreter', 'latex', 'FontSize', FontSize);
        ylabel(axs_rot(k), rot_labels{k}, 'Interpreter', 'latex', 'FontSize', FontSize);
        legend(axs_rot(k), compose('ASV %d', 1:no_vessels), 'Location', 'best', 'FontSize', FontSize-2, 'Interpreter', 'latex');
        axs_rot(k).FontSize = FontSize;
    end
    
    % ---------------------------
    % 06 - Disturbances (Sigma)
    % ---------------------------
    
    % --- Sigma u
    t_su = uitab(tg, 'Title', 'Sigma u');
    ax_su = axes(t_su); hold(ax_su, 'on'); grid(ax_su, 'on');
    for j = 1:no_vessels
        idx = (j-1)*no_items + 13;
        plot(ax_su, tm, hist_X(idx, :), 'LineWidth', LineWidth);
    end
    xlabel(ax_su, 'Time [s]', 'Interpreter', 'latex', 'FontSize', FontSize);
    ylabel(ax_su, '$\sigma_{u}$ [m s$^{-2}$]', 'Interpreter', 'latex', 'FontSize', FontSize);
    title(ax_su, '\textbf{Surge disturbance}', 'Interpreter', 'latex', 'FontSize', FontSize);
    legend(ax_su, compose('ASV %d', 1:no_vessels), 'Location', 'best', 'FontSize', FontSize-2, 'Interpreter', 'latex');
    ax_su.FontSize = FontSize;
    
    % --- Sigma v
    t_sv = uitab(tg, 'Title', 'Sigma v');
    ax_sv = axes(t_sv); hold(ax_sv, 'on'); grid(ax_sv, 'on');
    for j = 1:no_vessels
        idx = (j-1)*no_items + 14;
        plot(ax_sv, tm, hist_X(idx, :), 'LineWidth', LineWidth);
    end
    xlabel(ax_sv, 'Time [s]', 'Interpreter', 'latex', 'FontSize', FontSize);
    ylabel(ax_sv, '$\sigma_{v}$ [m s$^{-2}$]', 'Interpreter', 'latex', 'FontSize', FontSize);
    title(ax_sv, '\textbf{Sway disturbance}', 'Interpreter', 'latex', 'FontSize', FontSize);
    legend(ax_sv, compose('ASV %d', 1:no_vessels), 'Location', 'best', 'FontSize', FontSize-2, 'Interpreter', 'latex');
    ax_sv.FontSize = FontSize;
    
    % --- Sigma r
    t_sr = uitab(tg, 'Title', 'Sigma r');
    ax_sr = axes(t_sr); hold(ax_sr, 'on'); grid(ax_sr, 'on');
    for j = 1:no_vessels
        idx = (j-1)*no_items + 18;
        plot(ax_sr, tm, hist_X(idx, :), 'LineWidth', LineWidth);
    end
    xlabel(ax_sr, 'Time [s]', 'Interpreter', 'latex', 'FontSize', FontSize);
    ylabel(ax_sr, '$\sigma_{r}$ [rad s$^{-2}$]', 'Interpreter', 'latex', 'FontSize', FontSize);
    title(ax_sr, '\textbf{Yaw disturbance}', 'Interpreter', 'latex', 'FontSize', FontSize);
    legend(ax_sr, compose('ASV %d', 1:no_vessels), 'Location', 'best', 'FontSize', FontSize-2, 'Interpreter', 'latex');
    ax_sr.FontSize = FontSize;
    
    % ---------------------------
    % 07 - Control Actions
    % ---------------------------
    
    % --- Thrusters (T_L and T_R)
    t_act = uitab(tg, 'Title', 'Actuators Force');
    axs_act = gobjects(2,1);
    for k = 1:2
        axs_act(k) = subplot(2,1,k, 'Parent', t_act); hold(axs_act(k), 'on'); grid(axs_act(k), 'on');
    end
    
    act_labels = {'$F_L$ [N]', '$F_R$ [N]'};
    act_titles = {'\textbf{Force Left Thruster}', '\textbf{Force Right Thruster}'};
    act_indices = [19, 20];
    
    
    for k = 1:2
        for j = 1:no_vessels
            idx = (j-1)*no_items + act_indices(k);
            plot(axs_act(k), tm, hist_X(idx, :), 'LineWidth', LineWidth);
        end
        xlabel(axs_act(k), 'Time [s]', 'Interpreter', 'latex', 'FontSize', FontSize);
        ylabel(axs_act(k), act_labels{k}, 'Interpreter', 'latex', 'FontSize', FontSize);
        title(axs_act(k), act_titles{k}, 'Interpreter', 'latex', 'FontSize', FontSize);
        legend(axs_act(k), compose('ASV %d', 1:no_vessels), 'Location', 'best', 'FontSize', FontSize-2, 'Interpreter', 'latex');
        axs_act(k).FontSize = FontSize;
        % ylim(axs_act(k), [-1.0, 1.0]);     % Límite fijo en y
    end
    
    % --- Total Force (Fu)
    t_fu = uitab(tg, 'Title', 'Forces');
    ax_fu = axes(t_fu); hold(ax_fu, 'on'); grid(ax_fu, 'on');
    for j = 1:no_vessels
        idx = (j-1)*no_items + 21;
        plot(ax_fu, tm, hist_X(idx, :), 'LineWidth', LineWidth);
    end
    xlabel(ax_fu, 'Time [s]', 'Interpreter', 'latex', 'FontSize', FontSize);
    ylabel(ax_fu, '$F_u$ [N]', 'Interpreter', 'latex', 'FontSize', FontSize);
    title(ax_fu, '\textbf{Total Force}', 'Interpreter', 'latex', 'FontSize', FontSize);
    legend(ax_fu, compose('ASV %d', 1:no_vessels), 'Location', 'best', 'FontSize', FontSize-2, 'Interpreter', 'latex');
    ax_fu.FontSize = FontSize;
    
    % --- Yaw Torque (tr)
    t_tr = uitab(tg, 'Title', 'Torque');
    ax_tr = axes(t_tr); hold(ax_tr, 'on'); grid(ax_tr, 'on');
    for j = 1:no_vessels
        idx = (j-1)*no_items + 22;
        plot(ax_tr, tm, hist_X(idx, :), 'LineWidth', LineWidth);
    end
    xlabel(ax_tr, 'Time [s]', 'Interpreter', 'latex', 'FontSize', FontSize);
    ylabel(ax_tr, '$\tau_r$ [N m]', 'Interpreter', 'latex', 'FontSize', FontSize);
    title(ax_tr, '\textbf{Torques}', 'Interpreter', 'latex', 'FontSize', FontSize);
    legend(ax_tr, compose('ASV %d', 1:no_vessels), 'Location', 'best', 'FontSize', FontSize-2, 'Interpreter', 'latex');
    ax_tr.FontSize = FontSize;


end

