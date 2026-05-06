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

function Figure_PF(Vessel, hist_X, hist_X_hat, hist_mlc, tm)
% Figure_PF  Grafica en 5 pestañas las señales de tu controlador PF
% Inputs:
%   Vessel      - índice del vehículo a graficar (1…N)
%   hist_X      - (no_items·N)×T matriz de estados reales
%   hist_X_hat  - (9·N)×T matriz de estados estimados
%   hist_mlc    - (6·N)×T matriz de salidas del controlador [u_ref; r_ref; psi_ref; x_e; y_e; w]
%   no_items    - número de filas por vehículo en hist_X (22)
%   tm          - [1×T] vector de Times

% Dimensiones internas
nx = 22;      % filas por vehículo en hist_X
nh = 9;             % filas por vehículo en hist_X_hat
nm = 6;             % filas por vehículo en hist_mlc

% Índices de filas para el vehículo seleccionado
idxX  = (Vessel-1)*nx  + (1:nx);
idxXh = (Vessel-1)*nh  + (1:nh);
idxM  = (Vessel-1)*nm  + (1:nm);

% Extraer datos
real_X  = hist_X(idxX, :);
est_X   = hist_X_hat(idxXh, :);
mlc     = hist_mlc(idxM, :);

% Configuración de estilo
FontSize  = 16;
LineWidth = 2;

% Crear figura y grupo de pestañas
fig = figure('Name', sprintf('Controlador PF - Vehículo %d', Vessel), ...
             'NumberTitle','off');
tg = uitabgroup(fig);

%% 1) Surge reference (u_ref) vs real u y estimado u
tab1 = uitab(tg, 'Title', 'Surge');
ax1  = axes('Parent', tab1); hold(ax1,'on'); grid(ax1,'on');
plot(ax1, tm, mlc(1,:), 'LineWidth',LineWidth, 'DisplayName','$u_{ref}$');
plot(ax1, tm, real_X(7,:), 'LineWidth',LineWidth, 'DisplayName','$u$');
plot(ax1, tm, est_X(4,:), 'LineWidth',LineWidth, 'DisplayName','$\hat{u}$');
xlabel(ax1, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax1, '$u$ [m s$^{-1}$]','Interpreter','latex','FontSize',FontSize);
title(ax1, '\textbf{Surge Performance}','Interpreter','latex','FontSize',FontSize);
legend(ax1, 'Location','best','Interpreter','latex', 'FontSize', FontSize);

%% 2) Yaw‑rate reference (r_ref)
tab2 = uitab(tg, 'Title', 'Yaw');
ax2  = axes('Parent', tab2); hold(ax2,'on'); grid(ax2,'on');
plot(ax2, tm, mlc(3,:), 'LineWidth',LineWidth, 'DisplayName','$r_{ref}$');
plot(ax2, tm, real_X(12,:), 'LineWidth',LineWidth, 'DisplayName','$r$');
plot(ax2, tm, est_X(6,:), 'LineWidth',LineWidth, 'DisplayName','$\hat{r}$');
xlabel(ax2, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax2, '$r$ [rad s$^{-1}$]','Interpreter','latex','FontSize',FontSize);
title(ax2, '\textbf{Yaw Performance}','Interpreter','latex','FontSize',FontSize);
legend(ax2, 'Location','best','Interpreter','latex', 'FontSize', FontSize);

%% 3) Heading reference (psi_ref)
tab3 = uitab(tg, 'Title', 'Heading');
ax3  = axes('Parent', tab3); hold(ax3,'on'); grid(ax3,'on');
plot(ax3, tm, mlc(2,:), 'LineWidth',LineWidth, 'DisplayName','$\psi_{ref}$');
plot(ax3, tm, real_X(6,:), 'LineWidth',LineWidth, 'DisplayName','$\psi$');
plot(ax3, tm, est_X(3,:), 'LineWidth',LineWidth, 'DisplayName','$\hat{\psi}$');
xlabel(ax3, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax3, '$\psi$ [rad]','Interpreter','latex','FontSize',FontSize);
title(ax3, '\textbf{Heading Performance}','Interpreter','latex','FontSize',FontSize);
legend(ax3, 'Location','best','Interpreter','latex', 'FontSize', FontSize);

%% 4) Cross‑ & along‑track errors (x_e, y_e)
tab4 = uitab(tg, 'Title', 'Errors');
ax4  = axes('Parent', tab4); hold(ax4,'on'); grid(ax4,'on');
plot(ax4, tm, mlc(4,:), 'LineWidth',LineWidth, 'DisplayName','$x_e$');
plot(ax4, tm, mlc(5,:), 'LineWidth',LineWidth, 'DisplayName','$y_e$');
xlabel(ax4, 'Time [s]','Interpreter','latex','FontSize',FontSize);
ylabel(ax4, 'Error [m]','Interpreter','latex','FontSize',FontSize);
title(ax4, '\textbf{Tracking errors}','Interpreter','latex','FontSize',FontSize);
legend(ax4, 'Location','best','Interpreter','latex', 'FontSize', FontSize);

%% 5) w de todos los vehículos
tab5 = uitab(tg, 'Title', 'w (all vessels)');
ax5  = axes('Parent', tab5); 
hold(ax5,'on'); 
grid(ax5,'on');

% Número de vehículos
numV = size(hist_mlc,1) / nm;

% Para cada vehículo, extraer su fila de w (6ª fila de cada bloque) y graficar
for j = 1:numV
    idx_w = (j-1)*nm + 6;
    wj    = hist_mlc(idx_w, :);
    plot(ax5, tm, wj, 'LineWidth', LineWidth, ...
         'DisplayName', sprintf('$ASV%d$', j));
end

xlabel(ax5, 'Time [s]',    'Interpreter','latex','FontSize',FontSize);
ylabel(ax5, '$w$',            'Interpreter','latex','FontSize',FontSize);
title(ax5, '\textbf{Parameterized Variable }w', ...
      'Interpreter','latex','FontSize',FontSize);
legend(ax5, 'Location','best','Interpreter','latex', 'FontSize', FontSize);

%% 6) Tasa de cambio de w (\dot w) de todos los vehículos
tab6 = uitab(tg, 'Title', '\dot w');
ax6  = axes('Parent', tab6);
hold(ax6,'on');
grid(ax6,'on');

for j = 1:numV
    idx_w   = (j-1)*nm + 6;
    wj      = hist_mlc(idx_w, :);
    dwj_dt  = gradient(wj, tm);  % derivada numérica de w respecto al Time
    plot(ax6, tm, dwj_dt, 'LineWidth', LineWidth, ...
         'DisplayName', sprintf('$ASV%d$', j));
end

xlabel(ax6, 'Time [s]',       'Interpreter','latex','FontSize',FontSize);
ylabel(ax6, '$\dot w$',         'Interpreter','latex','FontSize',FontSize);
title(ax6, '\textbf{Rate of Change of }w', ...
      'Interpreter','latex','FontSize',FontSize);
legend(ax6, 'Location','best','Interpreter','latex','FontSize',FontSize);

end
