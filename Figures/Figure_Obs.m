%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autor: Manuel Eduardo Gantiva Osorio
% Proyecto: AQUATRONIC
% Escuela Técnica Superior de Ingeniería 
% Universidad Loyola Andalucía
% Fecha: 24.04.2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function generates a detailed plot for a single vessel, displaying real, measured,
% and estimated states. Each estimated state is shown in a separate tab, allowing for 
% clear comparison and evaluation of estimation accuracy.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Figure_Obs(Vessel, hist_X, hist_X_hat, noise, no_items, tm)
% Figure_Obs generates 9 tabs with plots for estimated, real, and measured values (where applicable)
%
% Parameters:
%   Vessel      - vehicle number to plot
%   hist_X      - matrix with the real history, with order:
%                 [ x y z phi theta ψ u v w p q r su sv sw sp sq sr TL TR Fu tr ]'
%                 (each vehicle occupies a block of "no_items" rows, typically 22)
%   hist_X_hat  - matrix with the estimated history, with 9 items per vehicle:
%                 [ x; y; ψ; u; v; r; σ_u; σ_v; σ_r ]
%   noise       - matrix with sensor noise (for [ x, y, ψ, r ]), same for all vessels (4 rows)
%   no_items    - number of rows (items) per vehicle in hist_X
%   tm          - time vector

    % Create figure and tab group
    fig = figure('Name', sprintf('Estimaciones - Vehículo %d', Vessel), ...
             'NumberTitle','off');
    tg = uitabgroup(fig);
    
    % Extract data for the selected vessel
    idx_real = (Vessel-1)*no_items + (1:no_items);      % indices for hist_X
    idx_est  = (Vessel-1)*9 + (1:9);                      % indices for hist_X_hat
    real_data = hist_X(idx_real, :);
    est_data = hist_X_hat(idx_est, :);
    % noise is used directly since it's the same for all vessels
    noise_data = noise;

    % Define mapping for the 9 estimated variables:
    mapping_est = {'Position x', 'Position y', 'Heading', 'Surge', 'Sway', 'Yaw', 'Sigmas_u', 'Sigmas_v', 'Sigmas_r'};
    % mapping_real_indices: index in hist_X for the real value of each variable.
    % hist_X order: [x, y, z, phi, theta, ψ, u, v, w, p, q, r, su, sv, sw, sp, sq, sr, TL, TR, Fu, tr]'
    mapping_real_indices = [1, 2, 6, 7, 8, 12, 13, 14, 18];
    % noise_mapping: for sensor variables (x, y, ψ, r)
    % corresponding indices: 1->x, 2->y, 3->ψ, 4->r; others do not have direct measurement.
    noise_mapping = [1, 2, 3, NaN, NaN, 4, NaN, NaN, NaN];

    % Y-axis labels with English units (using LaTeX formatting)
    y_labels = {'$x$ [m]', '$y$ [m]', '$\psi$ [rad]', '$u$ m s$^{-1}$]', '$v$ [m s$^{-1}$]', '$r$ [rad s$^{-1}$]', ... 
                '$\sigma_u$ [m s$^{-2}$]', '$\sigma_v$ [m s$^{-2}$]', '$\sigma_r$ [rad s$^{-2}$]'};
    % Tab titles in English
    tab_titles = {'\textbf{Position $x$}', '\textbf{Position $y$}', '\textbf{Heading}', '\textbf{Surge}', '\textbf{Sway}', '\textbf{Yaw}', ...
                  '\textbf{Sigma u}', '\textbf{Sigma v}', '\textbf{Sigma r}'};
    
    % Format configurations
    FontSize = 20;
    LineWidth = 3;

    % Loop over each of the 9 variables (tabs)
    for k = 1:9
        % Create tab for variable k
        tab = uitab(tg, 'Title', mapping_est{k});
        % Create axes in the tab
        ax = axes('Parent', tab);
        hold(ax, 'on');
        grid(ax, 'on');
        
        % Plot estimated value
        plot(ax, tm, est_data(k, :), 'LineWidth', LineWidth, 'DisplayName', 'Estimated');
        
        % Plot real value if available
        if ~isnan(mapping_real_indices(k))
            plot(ax, tm, real_data(mapping_real_indices(k), :), 'LineWidth', LineWidth, 'DisplayName', 'Real');
            
            % Plot measurement if the sensor exists (x, y, ψ, r)
            if ~isnan(noise_mapping(k))
                plot(ax, tm, real_data(mapping_real_indices(k), :) + noise_data(noise_mapping(k), :), ...
                    'LineWidth', LineWidth, 'DisplayName', 'Measured');
            end
        end
        
        % Configure labels and title
        xlabel(ax, 'Time [s]','Interpreter', 'latex', 'FontSize', FontSize);
        ylabel(ax, y_labels{k}, 'Interpreter', 'latex', 'FontSize', FontSize);
        title(ax, tab_titles{k}, 'Interpreter', 'latex', 'FontSize', FontSize);
        legend(ax, 'Location', 'best', 'FontSize', FontSize-2, 'Interpreter', 'latex');
        ax.FontSize = FontSize;
    end
end
