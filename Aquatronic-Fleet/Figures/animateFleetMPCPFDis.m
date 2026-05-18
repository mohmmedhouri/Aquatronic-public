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

function Video_animateFleetMPCPFDis(hist_X, hist_mlc, tm, m_asv, m_mlc, no_items, disturbances)
    no_vessels = length(m_asv);

    V_w = disturbances.V_w;
    beta_w = disturbances.beta_w;
    V_c = disturbances.V_c;
    beta_c = disturbances.beta_c;
    
    % Vector de tiempo para MLC
    t_mlc = tm(1):m_mlc{1}.T_s:tm(end); % time vector

    p_vec_size = m_mlc{1}.n_states+m_mlc{1}.n_states*m_mlc{1}.N_p+m_mlc{1}.n_controls*m_mlc{1}.N_c+m_mlc{1}.n_controls+m_mlc{1}.n_disturbances*m_mlc{1}.N_p+m_mlc{1}.n_states;
    u_opt_size = m_mlc{1}.n_controls*m_mlc{1}.N_c;
    hist_tar_hl = repmat(zeros(no_vessels*2,1),1,size(t_mlc, 2));

    
    for t = 1:length(t_mlc)
        for j = 1:no_vessels
            idx = (j-1)*(1+p_vec_size+u_opt_size+7);
            idx2 = (j-1)*2;
      
            hist_tar_hl(idx2+1, t) = m_mlc{j}.path.x_p(hist_mlc(idx+1+p_vec_size+u_opt_size+1, t));
            hist_tar_hl(idx2+2, t) = m_mlc{j}.path.y_p(hist_mlc(idx+1+p_vec_size+u_opt_size+1, t));
            
        end
    end    

    % Inicializa matriz de salida
    hist_tar = zeros(size(hist_tar_hl, 1) , length(tm));
    
    % Índice para recorrer t_mlc
    j = 1;
    
    for i = 1:length(tm)
        % Avanza en t_mlc hasta que el tiempo sea mayor al tiempo actual de tm
        while j < length(t_mlc) && t_mlc(j+1) <= tm(i)
            j = j + 1;
        end
        hist_tar(:, i) = hist_tar_hl(:, j);
    end
    
    clear hist_tar_hl t_mlc j i t
    figure('Name', 'Fleet Animation');
    ax = axes; hold(ax, 'on'); grid(ax, 'on'); axis equal;
   
    %% Definir figuras locales para cada modelo

    % Parámetros base
    L = 2;
    W = 1;
    
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
    end

    % Establecer límites del gráfico
    x_all = hist_X(1:no_items:end, :);
    y_all = hist_X(2:no_items:end, :);
    margin = 5;
    xlim([min(y_all(:))-margin, max(y_all(:))+margin]);
    ylim([min(x_all(:))-margin, max(x_all(:))+margin]);

    %% ==== Campos vectoriales de viento y corriente ====

    % Malla base del campo
    nx = 6; ny = 6;
    xl = xlim(ax);
    yl = ylim(ax);
    
    [yg, xg] = meshgrid( ...
        linspace(xl(1), xl(2), nx), ...
        linspace(yl(1), yl(2), ny) );
    
    % Separación vertical para evitar superposición
    offset_w = +1.5;   % viento
    offset_c = -1.5;   % corriente
    
    % Tamaño fijo de flechas (NO depende de la magnitud)
    arrow_len = 1.5;
    
    % Inicializar objetos quiver vacíos
    q_w = gobjects(1);
    q_c = gobjects(1);

    % Inicializar barcos y puntos centrales
    for j = 1:no_vessels
        vessel_shapes(j) = fill(NaN, NaN, cmap(j,:), 'FaceAlpha', 0.3, 'EdgeColor', 'k');
        center_dots(j) = plot(NaN, NaN, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 4);
        target_dots(j) =  plot(NaN, NaN, 'bo', 'MarkerFaceColor', cmap(j,:), 'MarkerSize', 4);
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
        end

        % ===== Campo de viento =====
        if V_w > 0
            uw = arrow_len * sin(beta_w);   % beta respecto al norte
            vw = arrow_len * cos(beta_w);
        
            if ~isgraphics(q_w)
                q_w = quiver( ...
                    yg + offset_w, xg, ...
                    uw*ones(size(xg)), vw*ones(size(xg)), ...
                    0, 'Color', [0.1 0.8 0.1], 'LineWidth', 2.0, 'MaxHeadSize', 2.0);
            else
                set(q_w, ...
                    'UData', uw*ones(size(xg)), ...
                    'VData', vw*ones(size(xg)));
            end
        else
            if isgraphics(q_w); delete(q_w); end
        end

        % ===== Campo de corriente =====
        if V_c > 0
            uc = arrow_len * sin(beta_c);
            vc = arrow_len * cos(beta_c);
        
            if ~isgraphics(q_c)
                q_c = quiver( ...
                    yg + offset_c, xg, ...
                    uc*ones(size(xg)), vc*ones(size(xg)), ...
                    0, 'Color', [0 0.6 1], 'LineWidth', 2.0, 'MaxHeadSize', 2.0);
            else
                set(q_c, ...
                    'UData', uc*ones(size(xg)), ...
                    'VData', vc*ones(size(xg)));
            end
        else
            if isgraphics(q_c); delete(q_c); end
        end

        title(ax, sprintf('Time: %.2f s', tm(t)), 'FontSize', 16);
        xlabel('$y$ [m]', 'Interpreter', 'latex', 'FontSize', 20);
        ylabel('$x$ [m]', 'Interpreter', 'latex', 'FontSize', 20);
        
        drawnow;
        pause(0.01);
    end
end