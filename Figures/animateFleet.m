%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autor: Manuel Eduardo Gantiva Osorio
% Proyecto: AQUATRONIC
% Escuela Técnica Superior de Ingeniería 
% Universidad Loyola Andalucía
% Fecha: 24.04.2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script takes the historical state data of each vessel along with relevant objects
% to produce a basic animated plot. It visualises the movement of the entire fleet over
% time, displaying only the travelled trajectories without including any
% controller-related information.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function animateFleet(hist_X, tm, m_asv, no_items)
    no_vessels = length(m_asv);
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
    end

    % Bucle de animación
    for t = 1:length(tm)
        for j = 1:no_vessels
            idx = (j-1)*no_items;
            x = hist_X(idx + 1, t);     % Coordenada Norte
            y = hist_X(idx + 2, t);     % Coordenada Este
            psi = hist_X(idx + 6, t);   % Heading
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
        end

        title(ax, sprintf('Time: %.2f s', tm(t)), 'FontSize', 16);
        xlabel('$y$ [m]', 'Interpreter', 'latex', 'FontSize', 20);
        ylabel('$x$ [m]', 'Interpreter', 'latex', 'FontSize', 20);
        drawnow;
        pause(0.01);
    end
end