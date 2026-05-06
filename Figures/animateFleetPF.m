%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autor: Manuel Eduardo Gantiva Osorio
% Proyecto: AQUATRONIC
% Escuela Técnica Superior de Ingeniería 
% Universidad Loyola Andalucía
% Fecha: 24.04.2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script produces a 2D animation similar to the basic trajectory visualisation, but
% it also incorporates path-following control data. In addition to the travelled paths,
% it plots the desired trajectories and target points for each vessel, providing insight
% into path-following performance.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function animateFleetPF(hist_X, hist_mlc, tm, m_asv, m_mlc, no_items)
    no_vessels = length(m_asv);

    hist_tar = repmat(zeros(no_vessels*2,1),1,size(tm, 2));
    hist_tar_c = repmat(zeros(no_vessels*2,1),1,size(tm, 2));
    for t = 1:length(tm)
        for j = 1:no_vessels
            idx = (j-1)*6;
            idx2 = (j-1)*2;

            xl = 0;
            yl = 0;
                       
            hist_tar_c(idx2+1, t) = m_mlc{j}.path.x_p(hist_mlc(idx+6, t));
            hist_tar_c(idx2+2, t) = m_mlc{j}.path.y_p(hist_mlc(idx+6, t));
            
            dx_c_dw = m_mlc{j}.path.dx_p_dw(hist_mlc(idx+6, t));
            dy_c_dw = m_mlc{j}.path.dy_p_dw(hist_mlc(idx+6, t));

            phi_c = atan2(dy_c_dw,dx_c_dw);
            
            hist_tar(idx2+1, t) = hist_tar_c(idx2+1, t) + xl *cos(phi_c) - yl*sin(phi_c);
            hist_tar(idx2+2, t) = hist_tar_c(idx2+2, t) + xl *sin(phi_c) + yl*cos(phi_c);
        end
    end    

    figure('Name', 'Fleet Animation');
    ax = axes; hold(ax, 'on'); grid(ax, 'on'); axis equal;

    % Parámetros base
    L = 2;
    W = 1;
    
    %% Definir figuras locales para cada modelo

    shape_kic = [ L,   -L/2,   -L/2;
                    0,    -W,      W ];
                
    shape_cyber = [ -L/2,   L/2,   L,   L/2,   -L/2;
                  W,      W,     0,   -W,    -W ];

    shape_otro = [ -L/2, -L/2,  L/6,  L/6,  L,  L/6,  L/6;
                   -2*W/3,    2*W/3,   2*W/3,    4*W/3,  0,  -4*W/3, -2*W/3 ];

    W = 0.8;         % Ancho del casco
    H = 2.0;         % Altura del casco (rectángulo)
    T = 0.6;         % Altura de la punta triangular
    S = 0.5;         % Desplazamiento lateral (mitad de la distancia entre cascos)

    shape_oto = [-H/2,   H,    H+T,   H, H-H/2, H-H/2, H,  H+T,   H, -H/2, -H/2, -H/3.5, -H/3.5, -H/2;
                 -S-W, -S-W, -S-W/2, -S,   -S,    S,   S, S+W/2, S+W, S+W,  S,     S,    -S,    -S];

    
    % Colores para cada vessel
    cmap = lines(no_vessels);

    % Prealocar objetos gráficos
    vessel_shapes = gobjects(1, no_vessels);
    center_dots = gobjects(1, no_vessels);
    target_dots = gobjects(1, no_vessels);
    target_c_dots = gobjects(1, no_vessels);

    % Dibujar trayectorias completas (líneas grises discontinuas)
    for j = 1:no_vessels
        idx_x = (j-1)*no_items + 1;
        idx_y = (j-1)*no_items + 2;
        % Plot de la trayectoria
        plot(hist_X(idx_y, :), hist_X(idx_x, :), '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 2.0);
        
        % Extraer la posición inicial (primer tiempo de la trayectoria)
        initX = hist_X(idx_y, 1);  % En el plot, X corresponde a la fila idx_y.
        initY = hist_X(idx_x, 1);  % Y corresponde a la fila idx_x.
        
        % Agregar el número identificador cerca del inicio del dron
        % Se utiliza un pequeño offset para que el número no se superponga exactamente a la trayectoria.
        text(initX + 0.8, initY + 0.8, num2str(j), ...
             'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
    end
    
     % Dibujar trayectorias deseadas
    for j = 1:no_vessels
        idx_x = (j-1)*2 + 1;
        idx_y = (j-1)*2 + 2;
        % Plot de la trayectoria
        plot(hist_tar(idx_y, :), hist_tar(idx_x, :), ':', 'Color', cmap(j,:), 'LineWidth', 2.5);
        plot(hist_tar_c(idx_y, :), hist_tar_c(idx_x, :), ':', 'Color', [0.2 0.2 0.2], 'LineWidth', 1.5);
       
    end

    % Establecer límites del gráfico
    x_all = hist_X(1:no_items:end, :);
    y_all = hist_X(2:no_items:end, :);
    margin = 5;
    xlim([min(y_all(:))-margin, max(y_all(:))+margin]);
    ylim([min(x_all(:))-margin, max(x_all(:))+margin]);

    % Inicializar barcos y puntos centrales
    for j = 1:no_vessels
        vessel_shapes(j) = fill(NaN, NaN, cmap(j,:), 'FaceAlpha', 0.3, 'EdgeColor', 'k');
        center_dots(j) = plot(NaN, NaN, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 4);
        target_dots(j) =  plot(NaN, NaN, 'bo', 'MarkerFaceColor', cmap(j,:), 'MarkerSize', 4);
        target_c_dots(j) =  plot(NaN, NaN, 'ro', 'MarkerFaceColor', cmap(j,:), 'MarkerSize', 4);
    end

    % Bucle de animación
    for t = 1:length(tm)
        for j = 1:no_vessels
            idx = (j-1)*no_items;
            idx2 = (j-1)*2;
            x = hist_X(idx + 1, t);     % Coordenada Norte
            y = hist_X(idx + 2, t);     % Coordenada Este
            psi = hist_X(idx + 6, t);   % Heading
            y_t = hist_tar(idx2 + 2, t);
            x_t = hist_tar(idx2 + 1, t);
            y_c = hist_tar_c(idx2 + 2, t);
            x_c = hist_tar_c(idx2 + 1, t);
            psi = -psi + pi/2;          % Ajuste del ángulo

            % Seleccionar la figura local según el modelo del barco
            model = m_asv{j}.model;
            switch model
                case 'Cybership'
                    shape_local = shape_cyber;
                case 'Otter'
                    shape_local = shape_oto;
                case 'ASVKic'
                    shape_local = shape_kic;
                otherwise
                    shape_local = shape_otro; % Por defecto
            end

            % Rotar la figura según el heading
            R = [cos(psi), -sin(psi); sin(psi), cos(psi)];
            rotated_shape = R * shape_local;

            shape_global = rotated_shape + [y; x];

            % Actualizar la figura y el punto central
            set(vessel_shapes(j), 'XData', shape_global(1,:), 'YData', shape_global(2,:));
            set(center_dots(j), 'XData', y, 'YData', x);
            set(target_dots(j), 'XData', y_t, 'YData', x_t);
            set(target_c_dots(j), 'XData', y_c, 'YData', x_c);
        end

        title(ax, sprintf('Time: %.2f s', tm(t)), 'FontSize', 16);
        xlabel('$y$ [m]', 'Interpreter', 'latex', 'FontSize', 20);
        ylabel('$x$ [m]', 'Interpreter', 'latex', 'FontSize', 20);
        drawnow;
        pause(0.01);
    end
end