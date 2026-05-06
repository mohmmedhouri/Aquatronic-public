%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autor: Manuel Eduardo Gantiva Osorio
% Proyecto: AQUATRONIC
% Escuela Técnica Superior de Ingeniería 
% Universidad Loyola Andalucía
% Fecha: 24.04.2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script generates a static multi-tabbed figure, with each tab displaying a plot
% of variables involved in low-level control. It is specifically used when only the
% low-level controller is active, without any path-following guidance, allowing for
% detailed analysis of control behaviour.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Figure_LLC(Vessel, hist_X, hist_X_hat, hist_ref, no_items, tm)
    FontSize = 16;
    LineWidth = 2;
    fig = figure;
    tg = uitabgroup(fig);
    
    row_ref_v = (Vessel-1)*3 + 1;
    row_ref_omega = (Vessel-1)*3 + 2;
    row_ref_phi = (Vessel-1)*3 + 3;
    ref_v = hist_ref(row_ref_v, :);
    ref_omega = hist_ref(row_ref_omega, :);
    ref_phi = hist_ref(row_ref_phi, :);
    
    row_real_v = (Vessel-1)*no_items + 7;
    row_real_omega = (Vessel-1)*no_items + 12;
    row_real_phi = (Vessel-1)*no_items + 3;
    real_v = hist_X(row_real_v, :);
    real_omega = hist_X(row_real_omega, :);
    real_phi = hist_X(row_real_phi, :);
    
    row_est_v = (Vessel-1)*9 + 4;
    row_est_omega = (Vessel-1)*9 + 6;
    row_est_phi = (Vessel-1)*9 + 3;
    est_v = hist_X_hat(row_est_v, :);
    est_omega = hist_X_hat(row_est_omega, :);
    est_phi = hist_X_hat(row_est_phi, :);
    
    ref_data = {ref_v, ref_omega, ref_phi};
    real_data = {real_v, real_omega, real_phi};
    est_data = {est_v, est_omega, est_phi};
    error_data = {ref_v - est_v, ref_omega - est_omega, ref_phi - est_phi};
    
    tab_titles = {'Velocidad de avance', 'Velocidad de giro', 'Ángulo'};
    y_labels = {'v [m/s]', '\omega [rad/s]', '\phi [rad]'};
    tab_titles_error = {'Error: Velocidad de avance', 'Error: Velocidad de giro', 'Error: Ángulo'};
    y_labels_error = {'Error v [m/s]', 'Error \omega [rad/s]', 'Error \phi [rad]'};
    
    for i = 1:3
        tab = uitab(tg, 'Title', tab_titles{i});
        ax = axes('Parent', tab);
        hold(ax, 'on');
        grid(ax, 'on');
        plot(ax, tm, ref_data{i}, 'LineWidth', LineWidth, 'DisplayName', 'Referencia');
        plot(ax, tm, real_data{i}, 'LineWidth', LineWidth, 'DisplayName', 'Real');
        plot(ax, tm, est_data{i}, 'LineWidth', LineWidth, 'DisplayName', 'Estimado');
        xlabel(ax, 'Time [s]', 'FontSize', FontSize);
        ylabel(ax, y_labels{i}, 'FontSize', FontSize);
        title(ax, tab_titles{i}, 'FontSize', FontSize);
        legend(ax, 'Location', 'best');
        ax.FontSize = FontSize;
    end
    
    for i = 1:3
        tab = uitab(tg, 'Title', tab_titles_error{i});
        ax = axes('Parent', tab);
        hold(ax, 'on');
        grid(ax, 'on');
        plot(ax, tm, error_data{i}, 'LineWidth', LineWidth, 'DisplayName', 'Error (Ref - Est)');
        xlabel(ax, 'Time [s]', 'FontSize', FontSize);
        ylabel(ax, y_labels_error{i}, 'FontSize', FontSize);
        title(ax, tab_titles_error{i}, 'FontSize', FontSize);
        legend(ax, 'Location', 'best');
        ax.FontSize = FontSize;
    end
end
