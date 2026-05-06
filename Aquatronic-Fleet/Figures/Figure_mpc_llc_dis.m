%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autor: Manuel Eduardo Gantiva Osorio
% Proyecto: AQUATRONIC
% Escuela Técnica Superior de Ingeniería 
% Universidad Loyola Andalucía
% Fecha: 24.04.2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script creates a static multi-tabbed figure using both vessel state history and
% path-following control data. Each tab plots a different variable relevant to the
% control system. It is intended for use when the path-following controller is active,
% enabling detailed analysis of both guidance and low-level control performance.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Figure_mpc_llc_dis(Vessel, hist_X, hist_X_hat, hist_mpc_llc, tm, t_s, LLC)
% Figure_PF  Grafica en 5 pestañas las señales de tu controlador PF
% Inputs:
%   Vessel      - índice del vehículo a graficar (1…N)
%   hist_X      - (no_items·N)×T matriz de estados reales
%   hist_X_hat  - (9·N)×T matriz de estados estimados
%   hist_mpc_llc- (149·N)×T matriz de salidas del controlador [ j_min p_vec u_opt n_i t_sol succes ]
%   no_items    - número de filas por vehículo en hist_X (22)
%   tm          - [1×T] vector de Times
%   ts          - Time especifico para evaluar
%   LLC         - Control MPC para obtener f_opt y np

% Dimensiones internas
nx = 22;            % filas por vehículo en hist_X
nh = 9;             % filas por vehículo en hist_X_hat
nm = 1 + (LLC.n_states+LLC.n_states*LLC.N_p+LLC.n_controls*LLC.N_c+LLC.n_controls+LLC.n_disturbances*LLC.N_p)+ (LLC.n_controls*LLC.N_c) + 3;             % filas por vehículo en hist_mlc

% Índices de filas para el vehículo seleccionado
idxX  = (Vessel-1)*nx  + (1:nx);
idxXh = (Vessel-1)*nh  + (1:nh);
idxM  = (Vessel-1)*nm  + (1:nm);

% Extraer datos
real_X  = hist_X(idxX, :);
est_X   = hist_X_hat(idxXh, :);
mpc_llc = hist_mpc_llc(idxM, :);

% Configuración de estilo
FontSize  = 16;
LineWidth = 2;

% Crear figura y grupo de pestañas
fig = figure('Name', sprintf('Controlador MPC -LLC - Vehículo %d', Vessel), ...
             'NumberTitle','off');
tg = uitabgroup(fig);

%% 0) Heading reference (psi_ref) vs real psi y estimado psi
tab0 = uitab(tg, 'Title', 'Heading');
ax0  = axes('Parent', tab0); hold(ax0,'on'); grid(ax0,'on');
plot(ax0, tm, mpc_llc(6,:), 'LineWidth',LineWidth, 'DisplayName','$\psi_{ref}$');
plot(ax0, tm, real_X(6,:), 'LineWidth',LineWidth, 'DisplayName','$\psi$');
plot(ax0, tm, est_X(3,:), 'LineWidth',LineWidth, 'DisplayName','$\hat{\psi}$');
xlabel(ax0, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax0, '$\psi$ [rad]','Interpreter','latex','FontSize',FontSize);
title(ax0, '\textbf{Heading Performance}','Interpreter','latex','FontSize',FontSize);
legend(ax0, 'Location','best','Interpreter','latex', 'FontSize', FontSize);

%% 1) Surge reference (u_ref) vs real u y estimado u
tab1 = uitab(tg, 'Title', 'Surge');
ax1  = axes('Parent', tab1); hold(ax1,'on'); grid(ax1,'on');
plot(ax1, tm, mpc_llc(7,:), 'LineWidth',LineWidth, 'DisplayName','$u_{ref}$');
plot(ax1, tm, real_X(7,:), 'LineWidth',LineWidth, 'DisplayName','$u$');
plot(ax1, tm, est_X(4,:), 'LineWidth',LineWidth, 'DisplayName','$\hat{u}$');
xlabel(ax1, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax1, '$u$ [m s$^{-1}$]','Interpreter','latex','FontSize',FontSize);
title(ax1, '\textbf{Surge Performance}','Interpreter','latex','FontSize',FontSize);
legend(ax1, 'Location','best','Interpreter','latex', 'FontSize', FontSize);

%% 2) Yaw‑rate reference (r_ref)
tab2 = uitab(tg, 'Title', 'Yaw');
ax2  = axes('Parent', tab2); hold(ax2,'on'); grid(ax2,'on');
plot(ax2, tm, mpc_llc(9,:), 'LineWidth',LineWidth, 'DisplayName','$r_{ref}$');
plot(ax2, tm, real_X(12,:), 'LineWidth',LineWidth, 'DisplayName','$r$');
plot(ax2, tm, est_X(6,:), 'LineWidth',LineWidth, 'DisplayName','$\hat{r}$');
xlabel(ax2, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax2, '$r$ [rad s$^{-1}$]','Interpreter','latex','FontSize',FontSize);
title(ax2, '\textbf{Yaw Performance}','Interpreter','latex','FontSize',FontSize);
legend(ax2, 'Location','best','Interpreter','latex', 'FontSize', FontSize);

%% 3) Sway reference (sway_ref)
tab3 = uitab(tg, 'Title', 'Sway');
ax3  = axes('Parent', tab3); hold(ax3,'on'); grid(ax3,'on');
plot(ax3, tm, mpc_llc(8,:), 'LineWidth',LineWidth, 'DisplayName','$v_{ref}$');
plot(ax3, tm, real_X(8,:), 'LineWidth',LineWidth, 'DisplayName','$v$');
plot(ax3, tm, est_X(5,:), 'LineWidth',LineWidth, 'DisplayName','$\hat{v}$');
xlabel(ax3, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax3, '$v$ [m s$^{-1}$]','Interpreter','latex','FontSize',FontSize);
title(ax3, '\textbf{Sway Performance}','Interpreter','latex','FontSize',FontSize);
legend(ax3, 'Location','best','Interpreter','latex', 'FontSize', FontSize);
%% 4) Predicion con entradas optimas

NP = LLC.N_p;
NC = LLC.N_c;
indt = find(abs(tm - t_s) < 1e-6, 1);
idx_base_dis = LLC.n_states+LLC.n_states*LLC.N_p+LLC.n_controls*LLC.N_c+LLC.n_controls;

P_LL = mpc_llc(2:LLC.n_states+LLC.n_states*LLC.N_p+LLC.n_controls*LLC.N_c+LLC.n_controls+LLC.n_disturbances*LLC.N_p+1,indt);
U_opt = reshape((mpc_llc(LLC.n_states+LLC.n_states*LLC.N_p+LLC.n_controls*LLC.N_c+LLC.n_controls+LLC.n_disturbances*LLC.N_p+2:LLC.n_states+LLC.n_states*LLC.N_p+LLC.n_controls*LLC.N_c+LLC.n_controls+LLC.n_disturbances*LLC.N_p+LLC.n_controls*LLC.N_c+1,indt))',LLC.n_controls,LLC.N_c)';
x_opt = full(LLC.ff_opt(U_opt',P_LL));
u_opt = full(U_opt);

dis = reshape((P_LL(idx_base_dis+1:idx_base_dis+LLC.n_disturbances*NP))',LLC.n_disturbances,NP);
dis = [dis, dis(:,end)]; % add 1 
z_opt = zeros(size(x_opt)); % misma dimensión

for k = 1:NP+1
    theta = x_opt(1,k);

    ROT = [ cos(theta) -sin(theta)  0
            sin(theta)  cos(theta)  0
            0           0           1 ];

    nu_c = ROT' * [ dis(3,k)*cos(dis(4,k))
                    dis(3,k)*sin(dis(4,k))
                    0 ];

    z_opt(:,k) = [ x_opt(1,k)
                   x_opt(2,k) + nu_c(1)
                   x_opt(3,k) + nu_c(2)
                   x_opt(4,k) ];
end 
%% 5) Heading ref por ventana de prediccion
tab3a = uitab(tg, 'Title', 'Heading NP');
ax3a  = axes('Parent', tab3a); hold(ax3a,'on'); grid(ax3a,'on');
plot(ax3a, tm(indt:indt+NP), mpc_llc(6,indt:indt+NP), 'LineWidth',LineWidth, 'DisplayName','$\psi_{ref}$');
plot(ax3a, tm(indt:indt+NP), real_X(6,indt:indt+NP), 'LineWidth',LineWidth, 'DisplayName','$\psi$');
plot(ax3a, tm(indt:indt+NP), est_X(3,indt:indt+NP), 'LineWidth',LineWidth, 'DisplayName','$\hat{\psi}$');
plot(ax3a, tm(indt:indt+NP), z_opt(1,:), 'LineWidth',LineWidth, 'DisplayName','${\psi}_{p}$');
xlim([tm(indt) tm(indt+NP)]);
xlabel(ax3a, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax3a, '$\psi$ [rad]','Interpreter','latex','FontSize',FontSize);
title(ax3a, '\textbf{Heading Performance NP}','Interpreter','latex','FontSize',FontSize);
legend(ax3a, 'Location','best','Interpreter','latex', 'FontSize', FontSize);

%% 6) Surge ref por ventana de prediccion
tab4 = uitab(tg, 'Title', 'Surge NP');
ax4  = axes('Parent', tab4); hold(ax4,'on'); grid(ax4,'on');
plot(ax4, tm(indt:indt+NP), mpc_llc(7,indt:indt+NP), 'LineWidth',LineWidth, 'DisplayName','$u_{ref}$');
plot(ax4, tm(indt:indt+NP), real_X(7,indt:indt+NP), 'LineWidth',LineWidth, 'DisplayName','$u$');
plot(ax4, tm(indt:indt+NP), est_X(4,indt:indt+NP), 'LineWidth',LineWidth, 'DisplayName','$\hat{u}$');
plot(ax4, tm(indt:indt+NP), z_opt(2,:), 'LineWidth',LineWidth, 'DisplayName','${u}_{p}$');
xlim([tm(indt) tm(indt+NP)]);
xlabel(ax4, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax4, '$u$ [m s$^{-1}$]','Interpreter','latex','FontSize',FontSize);
title(ax4, '\textbf{Surge Performance NP}','Interpreter','latex','FontSize',FontSize);
legend(ax4, 'Location','best','Interpreter','latex', 'FontSize', FontSize);

%% 7) Yaw ref por ventana de prediccion
tab5 = uitab(tg, 'Title', 'Yaw NP');
ax5  = axes('Parent', tab5); hold(ax5,'on'); grid(ax5,'on');
plot(ax5, tm(indt:indt+NP), mpc_llc(9,indt:indt+NP), 'LineWidth',LineWidth, 'DisplayName','$r_{ref}$');
plot(ax5, tm(indt:indt+NP), real_X(12,indt:indt+NP), 'LineWidth',LineWidth, 'DisplayName','$r$');
plot(ax5, tm(indt:indt+NP), est_X(6,indt:indt+NP), 'LineWidth',LineWidth, 'DisplayName','$\hat{r}$');
plot(ax5, tm(indt:indt+NP), z_opt(4,:), 'LineWidth',LineWidth, 'DisplayName','${r}_p$');
xlim([tm(indt) tm(indt+NP)]);
xlabel(ax5, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax5, '$r$ [rad s$^{-1}$]','Interpreter','latex','FontSize',FontSize);
title(ax5, '\textbf{Yaw Performance NP}','Interpreter','latex','FontSize',FontSize);
legend(ax5, 'Location','best','Interpreter','latex', 'FontSize', FontSize);

%% 8) Sway por ventana de prediccion
tab6 = uitab(tg, 'Title', 'Sway NP');
ax6  = axes('Parent', tab6); hold(ax6,'on'); grid(ax6,'on');
plot(ax6, tm(indt:indt+NP), mpc_llc(8,indt:indt+NP), 'LineWidth',LineWidth, 'DisplayName','$v_{ref}$');
plot(ax6, tm(indt:indt+NP), real_X(8,indt:indt+NP), 'LineWidth',LineWidth, 'DisplayName','$v$');
plot(ax6, tm(indt:indt+NP), est_X(5,indt:indt+NP), 'LineWidth',LineWidth, 'DisplayName','$\hat{v}$');
plot(ax6, tm(indt:indt+NP), z_opt(3,:), 'LineWidth',LineWidth, 'DisplayName','${v}_p$');
xlabel(ax6, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax6, '$v$ [m s$^{-1}$]','Interpreter','latex','FontSize',FontSize);
title(ax6, '\textbf{Sway Performance NP}','Interpreter','latex','FontSize',FontSize);
legend(ax6, 'Location','best','Interpreter','latex', 'FontSize', FontSize);

%% 9) Acciones
tab7 = uitab(tg, 'Title', 'Acciones');
ax7  = subplot(2,1,1,'Parent', tab7); hold(ax7,'on'); grid(ax7,'on');
plot(ax7, tm, real_X(21,:), 'LineWidth',LineWidth);
xlabel(ax7, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax7, '$F_u$ [N]','Interpreter','latex','FontSize',FontSize);
title(ax7, '\textbf{Force u}','Interpreter','latex','FontSize',FontSize);

ax8  = subplot(2,1,2,'Parent', tab7); hold(ax8,'on'); grid(ax8,'on');
plot(ax8, tm, real_X(22,:), 'LineWidth',LineWidth);
xlabel(ax8, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax8, '$\tau_r$ [N m]','Interpreter','latex','FontSize',FontSize);
title(ax8, '\textbf{Tao r}','Interpreter','latex','FontSize',FontSize);

%% 10) Acciones por ventana de prediccion
tab9 = uitab(tg, 'Title', 'Acciones NP');
ax9  = subplot(2,1,1,'Parent', tab9); hold(ax9,'on'); grid(ax9,'on');
plot(ax9, tm(indt:indt+NP), real_X(21,indt:indt+NP), 'LineWidth',LineWidth, 'DisplayName','$F_u$');
plot(ax9, tm(indt+1:indt+NC), u_opt(:,1), 'LineWidth',LineWidth, 'DisplayName','${F_u}_p$');
xlabel(ax9, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax9, '$F_u$ [N]','Interpreter','latex','FontSize',FontSize);
title(ax9, '\textbf{Force u}','Interpreter','latex','FontSize',FontSize);
legend(ax9, 'Location','best','Interpreter','latex', 'FontSize', FontSize);

ax10  = subplot(2,1,2,'Parent', tab9); hold(ax10,'on'); grid(ax10,'on');
plot(ax10, tm(indt:indt+NP), real_X(22,indt:indt+NP), 'LineWidth',LineWidth, 'DisplayName','$\tau_r$');
plot(ax10, tm(indt+1:indt+NC), u_opt(:,2), 'LineWidth',LineWidth, 'DisplayName','${\tau_r}_p$');
xlabel(ax10, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax10, '$\tau_r$ [N m]','Interpreter','latex','FontSize',FontSize);
title(ax10, '\textbf{Torque r}','Interpreter','latex','FontSize',FontSize);
legend(ax10, 'Location','best','Interpreter','latex', 'FontSize', FontSize);

%% 11) Actuators Force
tab15 = uitab(tg, 'Title', 'Actuators Force');
ax7  = subplot(2,1,1,'Parent', tab15); hold(ax7,'on'); grid(ax7,'on');
plot(ax7, tm, real_X(19,:), 'LineWidth',LineWidth);
xlabel(ax7, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax7, '$F_L$ [N]','Interpreter','latex','FontSize',FontSize);
title(ax7, '\textbf{Force Left Thruster}','Interpreter','latex','FontSize',FontSize);

ax8  = subplot(2,1,2,'Parent', tab15); hold(ax8,'on'); grid(ax8,'on');
plot(ax8, tm, real_X(20,:), 'LineWidth',LineWidth);
xlabel(ax8, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax8, '$F_R$ [N]','Interpreter','latex','FontSize',FontSize);
title(ax8, '\textbf{Force Right Thruster}','Interpreter','latex','FontSize',FontSize);
%% 12) Variables solver
tab10 = uitab(tg, 'Title', 'Solver 1');
ax11  = subplot(2,1,1,'Parent', tab10); hold(ax11,'on'); grid(ax11,'on');
plot(ax11, tm, mpc_llc(1,:), 'LineWidth',LineWidth);
xlabel(ax11, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax11, '$J$','Interpreter','latex','FontSize',FontSize);
title(ax11, '\textbf{Minimum Cost Function}','Interpreter','latex','FontSize',FontSize);

ax12  = subplot(2,1,2,'Parent', tab10); hold(ax12,'on'); grid(ax12,'on');
stem(ax12, tm(), mpc_llc(end,:), 'LineWidth',LineWidth);
xlabel(ax12, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax12, 'successful','Interpreter','latex','FontSize',FontSize);
title(ax12, '\textbf{Solver successful}','Interpreter','latex','FontSize',FontSize);
%% 13) Variables solver
tab11 = uitab(tg, 'Title', 'Solver 2');
ax13  = subplot(2,1,1,'Parent', tab11); hold(ax13,'on'); grid(ax13,'on');
plot(ax13, tm, mpc_llc(end-1,:), 'LineWidth',LineWidth);
xlabel(ax13, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax13, 'Time c [s]','Interpreter','latex','FontSize',FontSize);
title(ax13, '\textbf{Computing time}','Interpreter','latex','FontSize',FontSize);

ax14  = subplot(2,1,2,'Parent', tab11); hold(ax14,'on'); grid(ax14,'on');
stem(ax14, tm(), mpc_llc(end-2,:), 'LineWidth',LineWidth);
xlabel(ax14, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax14, 'Iterations','Interpreter','latex','FontSize',FontSize);
title(ax14, '\textbf{Number of iterations}','Interpreter','latex','FontSize',FontSize);

end
