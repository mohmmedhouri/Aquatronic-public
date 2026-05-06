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

function Figure_sgmp_hlc(Vessel, hist_X, hist_X_hat, hist_sgmp_hlc, tm, t_s, MLC)
% Figure_PF  Grafica en 5 pestañas las señales de tu controlador PF
% Inputs:
%   Vessel      - índice del vehículo a graficar (1…N)
%   hist_X      - (no_items·N)×T matriz de estados reales
%   hist_X_hat  - matriz de estados estimados
%   hist_sgmp_hlc- matriz de salidas del controlador [T_solucion | p_vec | n_controls*NC U_opt | 1 w | 1 x_e | 1 y_e | 1 v_bar  ]
%   no_items    - número de filas por vehículo en hist_X (22)
%   tm          - [1×T] vector de Times
%   ts          - Time especifico para evaluar
%   MLC         - Control MPC para obtener f_opt y np

% Dimensiones internas
nx = 22;            % filas por vehículo en hist_X
nh = 9;             % filas por vehículo en hist_X_hat
p_vec_size = MLC.n_states+MLC.n_controls+MLC.n_disturbances*MLC.N_p+3;
p_vec_0 = MLC.n_states+MLC.n_controls+MLC.n_disturbances*MLC.N_p;                                        %Tamaño para sacar las referencias actuales
nm = 1 + (MLC.n_states+MLC.n_controls+MLC.n_disturbances*MLC.N_p+3)+ (MLC.n_controls*MLC.N_c) + 4;  % filas por vehículo en hist_mlc

% Índices de filas para el vehículo seleccionado
idxX  = (Vessel-1)*nx  + (1:nx);
idxXh = (Vessel-1)*nh  + (1:nh);
idxM  = (Vessel-1)*nm  + (1:nm);

% Extraer datos
real_X  = hist_X(idxX, :);
est_X   = hist_X_hat(idxXh, :);
mpc_hlc = hist_sgmp_hlc(idxM, :);

% Vector de tiempo para MLC
Tsam = tm(2);
t_mlc = tm(1):MLC.T_s:tm(end); % time vector

% Configuración de estilo
FontSize  = 16;
LineWidth = 2;

% Crear figura y grupo de pestañas
fig = figure('Name', sprintf('Controlador SGLOS -MP - Vehículo %d', Vessel), ...
             'NumberTitle','off');
tg = uitabgroup(fig);

%% 1) Tracking errors and w
tab1 = uitab(tg, 'Title', 'Tracking errors');
ax1  = subplot(2,1,1,'Parent', tab1); hold(ax1,'on'); grid(ax1,'on');
plot(ax1, t_mlc, mpc_hlc(end-2,:), 'LineWidth',LineWidth, 'DisplayName','$x_{e}$');
plot(ax1, t_mlc, mpc_hlc(end-1,:), 'LineWidth',LineWidth, 'DisplayName','$y_{e}$');
xlabel(ax1, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax1, 'Error [m]','Interpreter','latex','FontSize',FontSize);
title(ax1, '\textbf{Tracking errors}','Interpreter','latex','FontSize',FontSize);
legend(ax1, 'Location','best','Interpreter','latex', 'FontSize', FontSize);

ax2  = subplot(2,1,2,'Parent', tab1); hold(ax2,'on'); grid(ax2,'on');
plot(ax2, t_mlc(), mpc_hlc(end-3,:), 'LineWidth',LineWidth);
xlabel(ax2, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax2, '$w$','Interpreter','latex','FontSize',FontSize);
title(ax2, '\textbf{Parameterized Variable }$w$','Interpreter','latex','FontSize',FontSize);

%% 2) Heading and Sway modified
tab2 = uitab(tg, 'Title', 'psi y v_bar');
ax2  = subplot(2,1,1,'Parent', tab2); hold(ax2,'on'); grid(ax2,'on');
plot(ax2, tm, real_X(6,:), 'LineWidth',LineWidth, 'DisplayName','$\psi$');
plot(ax2, tm, est_X(3,:), 'LineWidth',LineWidth, 'DisplayName','$\hat{\psi}$');
xlabel(ax2, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax2, '$\psi$ [rad]','Interpreter','latex','FontSize',FontSize);
title(ax2, '\textbf{Heading Performance}','Interpreter','latex','FontSize',FontSize);
legend(ax2, 'Location','best','Interpreter','latex', 'FontSize', FontSize);

ax3  = subplot(2,1,2,'Parent', tab2); hold(ax3,'on'); grid(ax3,'on');
plot(ax3, t_mlc(), mpc_hlc(end,:), 'LineWidth',LineWidth);
xlabel(ax3, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax3, '$\bar{v}$ [m s$^{-1}$]','Interpreter','latex','FontSize',FontSize);
title(ax3, '\textbf{Modified Sway }$\bar{v}$','Interpreter','latex','FontSize',FontSize);

%% 3) Surge and Yaw reference (surge_ref y yaw_ref)
tab3 = uitab(tg, 'Title', 'u y r');
ax3  = subplot(2,1,1,'Parent', tab3); hold(ax3,'on'); grid(ax3,'on');
plot(ax3, t_mlc(1:end-1) + Tsam, mpc_hlc(1+p_vec_size+1,1:end-1), 'LineWidth',LineWidth, 'DisplayName','$u_{ref}$');
plot(ax3, t_mlc(1:end), mpc_hlc(1+p_vec_0+1,1:end), '--', 'LineWidth',LineWidth, 'DisplayName','$u_{d}$');
plot(ax3, tm, real_X(7,:), 'LineWidth',LineWidth, 'DisplayName','$u$');
plot(ax3, tm, est_X(4,:), 'LineWidth',LineWidth, 'DisplayName','$\hat{u}$');
xlabel(ax3, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax3, '$u$ [m s$^{-1}$]','Interpreter','latex','FontSize',FontSize);
title(ax3, '\textbf{Surge Performance}','Interpreter','latex','FontSize',FontSize);
legend(ax3, 'Location','best','Interpreter','latex', 'FontSize', FontSize);

ax4  = subplot(2,1,2,'Parent', tab3); hold(ax4,'on'); grid(ax4,'on');
plot(ax4, t_mlc(1:end-1) + Tsam, mpc_hlc(1+p_vec_size+3,1:end-1), 'LineWidth',LineWidth, 'DisplayName','$r_{ref}$');
plot(ax4, tm, real_X(12,:), 'LineWidth',LineWidth, 'DisplayName','$r$');
plot(ax4, tm, est_X(6,:), 'LineWidth',LineWidth, 'DisplayName','$\hat{r}$');
xlabel(ax4, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax4, '$r$ [rad s$^{-1}$]','Interpreter','latex','FontSize',FontSize);
title(ax4, '\textbf{Yaw Performance}','Interpreter','latex','FontSize',FontSize);
legend(ax4, 'Location','best','Interpreter','latex', 'FontSize', FontSize);

%% 4) U tar y norma de la tangente

tab4 = uitab(tg, 'Title', 'U_tar');
ax3  = subplot(2,1,1,'Parent', tab4); hold(ax3,'on'); grid(ax3,'on');
plot(ax3, t_mlc(1:end-1) + Tsam, mpc_hlc(1+p_vec_size+2,1:end-1), 'LineWidth',LineWidth, 'DisplayName','$u_{tar}$');
plot(ax3, t_mlc(1:end), mpc_hlc(1+p_vec_0+1,1:end), '--', 'LineWidth',LineWidth, 'DisplayName','$u_{d}$');
xlabel(ax3, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax3, '$u$ [m s$^{-1}$]','Interpreter','latex','FontSize',FontSize);
title(ax3, '\textbf{Target speed}','Interpreter','latex','FontSize',FontSize);

w = mpc_hlc(end-3, :);  % Vector fila
F = zeros(1,length(w));        % Prealoca el vector F
for i = 1:length(w)
    dx = MLC.path.dx_p_dw(w(i));
    dy = MLC.path.dy_p_dw(w(i));
    F(i) = sqrt(dx^2 + dy^2);
end

ax4  = subplot(2,1,2,'Parent', tab4); hold(ax4,'on'); grid(ax4,'on');
plot(ax4, t_mlc, F, 'LineWidth',LineWidth);
xlabel(ax4, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax4, '$F$ [m s$^{-1}$]','Interpreter','latex','FontSize',FontSize);
title(ax4, '\textbf{Tangent vector norm}','Interpreter','latex','FontSize',FontSize);

%% 9) Variables solver 1
tab10 = uitab(tg, 'Title', 'Solver 1');
ax11  = subplot(1,1,1,'Parent', tab10); hold(ax11,'on'); grid(ax11,'on');
plot(ax11, t_mlc, mpc_hlc(1,:), 'LineWidth',LineWidth);
xlabel(ax11, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax11, '$J$','Interpreter','latex','FontSize',FontSize);
title(ax11, '\textbf{Computing time}','Interpreter','latex','FontSize',FontSize);

%% Predicion con entradas optimas

NP = MLC.N_p;
NC = MLC.N_c;
indt = find(abs(t_mlc - t_s) < 1e-6, 1);
idx_base_dis = MLC.n_states+MLC.n_controls;

P_LL = mpc_hlc(2:MLC.n_states+MLC.n_controls+MLC.n_disturbances*MLC.N_p+3+1,indt);
U_opt = reshape((mpc_hlc(1+MLC.n_states+MLC.n_controls+MLC.n_disturbances*MLC.N_p+3+1:1+MLC.n_states+MLC.n_controls+MLC.n_disturbances*MLC.N_p+3+MLC.n_controls*MLC.N_c,indt))',MLC.n_controls,MLC.N_c)';
[st_pre, ]= MLC.ff_pre(P_LL);
st_pre = full(st_pre);
u_opt = full(U_opt);
if NP > NC
    ultima_fila = u_opt(end, :);
    filas_extra = repmat(ultima_fila, NP - NC, 1);
    u_opt = [u_opt; filas_extra];
end

dis = reshape((P_LL(idx_base_dis+1:idx_base_dis+MLC.n_disturbances*NP))',MLC.n_disturbances,NP);
r_act = P_LL(MLC.n_states+MLC.n_controls+NP*MLC.n_disturbances+2);
dis = [dis, dis(:,end)]; % add 1 
z_opt = zeros(size(st_pre)); % misma dimensión

for k = 1:NP+1
    theta = st_pre(3,k);

    [j , spatial_path]  = pathScriptCasADI(st_pre(4,k));
    phi_p = spatial_path.phi_p(st_pre(4,k));

    ROT = [ cos(theta) -sin(theta)  0
            sin(theta)  cos(theta)  0
            0           0           1 ];

    nu_c = ROT' * [ dis(3,k)*cos(dis(4,k))
                    dis(3,k)*sin(dis(4,k))
                    0 ];

    z_opt(:,k) = [st_pre(1,k) - MLC.epsilon*cos(st_pre(3,k)-phi_p); ...
                  st_pre(2,k) - MLC.epsilon*sin(st_pre(3,k)-phi_p); ...
                  st_pre(3,k); ...
                  st_pre(4,k); ...
                  (st_pre(5,k) + nu_c(2)) - MLC.epsilon*r_act];
    if(k<NP+1)
        r_act = u_opt(k,3);
    end 
end 


indt_s0= find(abs(tm - t_s) < 1e-6, 1);            %Posicion inicial tm
indt_s1= find(abs(tm - t_mlc(indt+NP)) < 1e-6, 1); %Posicion final tm

clear filas_extra ultima_fila
%% 6) Tracking errors and w
tab6 = uitab(tg, 'Title', 'Tracking errors NP');
ax1  = subplot(2,1,1,'Parent', tab6); hold(ax1,'on'); grid(ax1,'on');
plot(ax1, t_mlc(indt:indt+NP), mpc_hlc(end-2,indt:indt+NP), 'LineWidth',LineWidth, 'DisplayName','$x_{e}$');
plot(ax1, t_mlc(indt:indt+NP), mpc_hlc(end-1,indt:indt+NP), 'LineWidth',LineWidth, 'DisplayName','$y_{e}$');
plot(ax1, t_mlc(indt:indt+NP), z_opt(1,:), 'LineWidth',LineWidth, 'DisplayName','$x_{e_p}$');
plot(ax1, t_mlc(indt:indt+NP), z_opt(2,:), 'LineWidth',LineWidth, 'DisplayName','$y_{e_p}$');
xlabel(ax1, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax1, 'Error [m]','Interpreter','latex','FontSize',FontSize);
title(ax1, '\textbf{Tracking errors}','Interpreter','latex','FontSize',FontSize);
legend(ax1, 'Location','best','Interpreter','latex', 'FontSize', FontSize);
xlim([t_mlc(indt) t_mlc(indt+NP)]);
ax2  = subplot(2,1,2,'Parent', tab6); hold(ax2,'on'); grid(ax2,'on');
plot(ax2, t_mlc(indt:indt+NP), mpc_hlc(end-3,indt:indt+NP), 'LineWidth',LineWidth, 'DisplayName','$w$');
plot(ax2, t_mlc(indt:indt+NP), z_opt(4,:), 'LineWidth',LineWidth, 'DisplayName','$w_{p}$');
xlabel(ax2, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax2, '$w$','Interpreter','latex','FontSize',FontSize);
title(ax2, '\textbf{Parameterized Variable }$w$','Interpreter','latex','FontSize',FontSize);
legend(ax2, 'Location','best','Interpreter','latex', 'FontSize', FontSize);
xlim([t_mlc(indt) t_mlc(indt+NP)]);

%% 7) Heading and Sway modified
tab7 = uitab(tg, 'Title', 'psi y v_bar NP');
ax2  = subplot(2,1,1,'Parent', tab7); hold(ax2,'on'); grid(ax2,'on');
plot(ax2, tm(indt_s0:indt_s1), real_X(6,indt_s0:indt_s1), 'LineWidth',LineWidth, 'DisplayName','$\psi$');
plot(ax2, tm(indt_s0:indt_s1), est_X(3,indt_s0:indt_s1), 'LineWidth',LineWidth, 'DisplayName','$\hat{\psi}$');
plot(ax2, t_mlc(indt:indt+NP), z_opt(3,:), 'LineWidth',LineWidth, 'DisplayName','$\psi_{p}$');
xlabel(ax2, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax2, '$\psi$ [rad]','Interpreter','latex','FontSize',FontSize);
title(ax2, '\textbf{Heading Performance}','Interpreter','latex','FontSize',FontSize);
legend(ax2, 'Location','best','Interpreter','latex', 'FontSize', FontSize);
xlim([t_mlc(indt) t_mlc(indt+NP)]);
ax3  = subplot(2,1,2,'Parent', tab7); hold(ax3,'on'); grid(ax3,'on');
plot(ax3, t_mlc(indt:indt+NP), mpc_hlc(end,indt:indt+NP), 'LineWidth',LineWidth, 'DisplayName','$\bar{v}$');
plot(ax3, t_mlc(indt:indt+NP), z_opt(5,:), 'LineWidth',LineWidth, 'DisplayName','$\bar{v_p}$');
xlabel(ax3, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax3, '$\bar{v}$ [m s$^{-1}$]','Interpreter','latex','FontSize',FontSize);
title(ax3, '\textbf{Modified Sway }$\bar{v}$','Interpreter','latex','FontSize',FontSize);
legend(ax3, 'Location','best','Interpreter','latex', 'FontSize', FontSize);
xlim([t_mlc(indt) t_mlc(indt+NP)]);

%% 8) Surge and Yaw reference (surge_ref y yaw_ref) NP
tab8 = uitab(tg, 'Title', 'u y r NP');
ax3  = subplot(2,1,1,'Parent', tab8); hold(ax3,'on'); grid(ax3,'on');
plot(ax3, t_mlc(indt:indt+NP) + Tsam, mpc_hlc(1+p_vec_size+1,indt:indt+NP), 'LineWidth',LineWidth, 'DisplayName','$u_{ref}$');
plot(ax3, tm(indt_s0:indt_s1), real_X(7,indt_s0:indt_s1), 'LineWidth',LineWidth, 'DisplayName','$u$');
plot(ax3, tm(indt_s0:indt_s1), est_X(4,indt_s0:indt_s1), 'LineWidth',LineWidth, 'DisplayName','$\hat{u}$');
plot(ax3, t_mlc(indt:indt+NP-1) + Tsam, u_opt(:,1), 'LineWidth',LineWidth, 'DisplayName','${u}_p$');
xlabel(ax3, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax3, '$u$ [m s$^{-1}$]','Interpreter','latex','FontSize',FontSize);
title(ax3, '\textbf{Surge Performance}','Interpreter','latex','FontSize',FontSize);
legend(ax3, 'Location','best','Interpreter','latex', 'FontSize', FontSize);
xlim([t_mlc(indt) t_mlc(indt+NP)]);
ax4  = subplot(2,1,2,'Parent', tab8); hold(ax4,'on'); grid(ax4,'on');
plot(ax4, t_mlc(indt:indt+NP) + Tsam, mpc_hlc(1+p_vec_size+3,indt:indt+NP), 'LineWidth',LineWidth, 'DisplayName','$r_{ref}$');
plot(ax4, tm(indt_s0:indt_s1), real_X(12,indt_s0:indt_s1), 'LineWidth',LineWidth, 'DisplayName','$r$');
plot(ax4, tm(indt_s0:indt_s1), est_X(6,indt_s0:indt_s1), 'LineWidth',LineWidth, 'DisplayName','$\hat{r}$');
plot(ax4, t_mlc(indt:indt+NP-1) + Tsam, u_opt(:,3), 'LineWidth',LineWidth, 'DisplayName','${r}_p$');
xlabel(ax4, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax4, '$r$ [rad s$^{-1}$]','Interpreter','latex','FontSize',FontSize);
title(ax4, '\textbf{Yaw Performance}','Interpreter','latex','FontSize',FontSize);
legend(ax4, 'Location','best','Interpreter','latex', 'FontSize', FontSize);
xlim([t_mlc(indt) t_mlc(indt+NP)]);

%% 9) U tar y norma de la tangente NP

tab9 = uitab(tg, 'Title', 'U_tar NP');
ax3  = subplot(2,1,1,'Parent', tab9); hold(ax3,'on'); grid(ax3,'on');
plot(ax3, t_mlc(indt:indt+NP) + Tsam, mpc_hlc(1+p_vec_size+2,indt:indt+NP), 'LineWidth',LineWidth, 'DisplayName','$u_{tar}$');
plot(ax3, t_mlc(indt:indt+NP-1) + Tsam, u_opt(:,2), 'LineWidth',LineWidth, 'DisplayName','$u_{tar_p}$');
xlabel(ax3, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax3, '$u$ [m s$^{-1}$]','Interpreter','latex','FontSize',FontSize);
title(ax3, '\textbf{Target speed}','Interpreter','latex','FontSize',FontSize);
legend(ax3, 'Location','best','Interpreter','latex', 'FontSize', FontSize);
xlim([t_mlc(indt) t_mlc(indt+NP)]);

w = mpc_hlc(end-6, indt:indt+NP);  % Vector fila
F = zeros(1,length(w));        % Prealoca el vector F
for i = 1:length(w)
    dx = MLC.path.dx_p_dw(w(i));
    dy = MLC.path.dy_p_dw(w(i));
    F(i) = sqrt(dx^2 + dy^2);
end

w = z_opt(4,:);  % Vector fila
Fp = zeros(1,length(w));        % Prealoca el vector F
for i = 1:length(w)
    dx = MLC.path.dx_p_dw(w(i));
    dy = MLC.path.dy_p_dw(w(i));
    Fp(i) = sqrt(dx^2 + dy^2);
end

ax4  = subplot(2,1,2,'Parent', tab9); hold(ax4,'on'); grid(ax4,'on');
plot(ax4, t_mlc(indt:indt+NP), F, 'LineWidth',LineWidth, 'DisplayName','$F$');
plot(ax4, t_mlc(indt:indt+NP), F, 'LineWidth',LineWidth, 'DisplayName','$F_{p}$');
xlabel(ax4, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax4, '$F$ [m s$^{-1}$]','Interpreter','latex','FontSize',FontSize);
title(ax4, '\textbf{Tangent vector norm}','Interpreter','latex','FontSize',FontSize);
legend(ax4, 'Location','best','Interpreter','latex', 'FontSize', FontSize);
xlim([t_mlc(indt) t_mlc(indt+NP)]);

end
