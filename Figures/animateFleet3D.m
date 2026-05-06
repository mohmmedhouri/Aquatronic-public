%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autor: Manuel Eduardo Gantiva Osorio
% Proyecto: AQUATRONIC
% Escuela Técnica Superior de Ingeniería 
% Universidad Loyola Andalucía
% Fecha: 24.04.2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script generates a 3D animated visualisation of the fleet using the historical
% state data of each vessel. It captures the full six degrees of freedom motion,
% providing a more comprehensive view of each vessel’s dynamic behaviour throughout
% the simulation.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function animateFleet3D(hist_X, tm, m_asv, no_items)
    % hist_X: matriz de estados. Se asume que cada barco tiene al menos:
    %         [ North, East, Down, phi, theta, psi, ... ] en ese orden.
    % tm: vector de tiempos.
    % m_asv: cell array donde cada elemento es un struct con el campo .model.
    % no_items: número de elementos en el vector de estado de cada barco.
    
    no_vessels = length(m_asv);
    
    % Crear figura 3D y configurar ejes
    figure('Name', 'Fleet 3D Animation');
    ax = axes;
    hold(ax, 'on');
    grid(ax, 'on');
    axis equal;
    xlabel('East'); ylabel('North'); zlabel('Down');
    view(45,30);         % Ajusta el ángulo de vista
    xlim([-inf, inf]);
    ylim([-inf, inf]);
    zlim([-1.0, 3]);     % Límite fijo en Z
    
    % Parámetros para definir las figuras 2D
    L = 2;     % Longitud característica
    W = 1;     % Ancho (la base se extiende de -W a W)
    thickness = 1.0; % Grosor para la extrusión 3D
    
    % Definir las figuras 2D base (cada columna es un vértice [x; y])
    shape_kic = [ L,   -L/2,   -L/2;
                    0,    -W,      W ];
                
    shape_cyber = [ -L/2,   L/2,   L,   L/2,   -L/2;
                  W,      W,     0,   -W,    -W ];
    
    shape_otro = [ -L/2, -L/2,  L/6,  L/6,  L,  L/6,  L/6;
                 -2*W/3, 2*W/3, 2*W/3, 4*W/3, 0, -4*W/3, -2*W/3 ];

    W = 0.8;         % Ancho del casco
    H = 2.0;         % Altura del casco (rectángulo)
    T = 0.6;         % Altura de la punta triangular
    S = 0.5;         % Desplazamiento lateral (mitad de la distancia entre cascos)

    shape_oto = [-H/2,   H,    H+T,   H, H-H/2, H-H/2, H,  H+T,   H, -H/2, -H/2, -H/3.5, -H/3.5, -H/2;
                 -S-W, -S-W, -S-W/2, -S,   -S,    S,   S, S+W/2, S+W, S+W,  S,     S,    -S,    -S];
    
    % Precomputar la figura extruida de cada barco según su modelo.
    extrudedShapes = cell(1, no_vessels);
    facesCell = cell(1, no_vessels);
    for j = 1:no_vessels
        model = m_asv{j}.model;
        switch model
            case 'Cybership'
                poly2d = shape_cyber;
            case 'Otter'
                poly2d = shape_oto;
            case 'ASVKic'
                poly2d = shape_kic;
            otherwise
                poly2d = shape_otro;  % valor por defecto
        end
        [vertices_extruded, faces_extruded] = extrudePolygon(poly2d, thickness);
        extrudedShapes{j} = vertices_extruded;
        facesCell{j} = faces_extruded;
    end

    % Dibujar las trayectorias completas para cada barco en 3D
    % En el marco NED:
    %   - North se toma de hist_X(idx+1,:), se usará como Y.
    %   - East se toma de hist_X(idx+2,:), se usará como X.
    %   - Down se toma de hist_X(idx+3,:), se usará como Z.
    for j = 1:no_vessels
        idx_north = (j-1)*no_items + 1;
        idx_east  = (j-1)*no_items + 2;
        idx_down  = (j-1)*no_items + 3;
        plot3(hist_X(idx_east, :), hist_X(idx_north, :), hist_X(idx_down, :), ...
              '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5);
    end

    % Inicializar los objetos gráficos (patch) para cada barco usando la figura extruida
    vessel_patches = gobjects(1, no_vessels);
    for j = 1:no_vessels
        vessel_patches(j) = patch('Faces', facesCell{j}, 'Vertices', extrudedShapes{j}, ...
            'FaceColor', rand(1,3), 'FaceAlpha', 0.7, 'EdgeColor', 'k');
    end

    % Bucle de animación
    for t = 1:length(tm)
        for j = 1:no_vessels
            idx = (j-1)*no_items;
            % Extraer posición en el marco NED
            north = hist_X(idx + 1, t);
            east  = hist_X(idx + 2, t);
            down  = hist_X(idx + 3, t);
            
            % Extraer orientación (ángulos de Euler)
            phi   = hist_X(idx + 4, t);   % roll
            theta = hist_X(idx + 5, t);   % pitch
            psi   = hist_X(idx + 6, t);   % yaw
            psi = -psi + pi/2;            % Ajuste de ángulo (como en 2D)
            
            % Calcular la matriz de rotación 3D
            R = eul2rotMat(phi, theta, psi);
            
            % Obtener la figura local extruida precomputada para este barco
            localVertices = extrudedShapes{j};  % Tamaño: [numVertices x 3]
            % Aplicar la rotación
            rotatedVertices = (R * localVertices')';
            % Trasladar usando [east, north, down] (según el marco NED)
            transformedVertices = rotatedVertices + repmat([east, north, down], size(rotatedVertices, 1), 1);
            
            % Actualizar la posición del objeto patch
            set(vessel_patches(j), 'Vertices', transformedVertices);
        end
        drawnow;
        pause(0.02);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Función auxiliar: extrudePolygon
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [vertices, faces] = extrudePolygon(poly2d, d)
    % poly2d: matriz 2 x n, donde cada columna es un vértice (en el plano XY)
    % d: grosor de extrusión.
    % Devuelve:
    %   vertices: matriz (2*n)x3, cada fila es un vértice en 3D.
    %   faces: matriz de caras con padding de NaN (cada fila es una cara).
    
    n = size(poly2d, 2);
    
    % Crear vértices para la cara superior (z = d/2) y la inferior (z = -d/2)
    top = [poly2d; (d/2)*ones(1, n)];    % 3 x n
    bottom = [poly2d; (-d/2)*ones(1, n)];  % 3 x n
    
    % Combinar vértices: cada fila es un vértice (total 2*n vértices)
    vertices = [top'; bottom'];
    
    % Definir la cara superior: usar el orden dado
    topFace = 1:n;
    % Definir la cara inferior: usar los índices n+1:2*n en orden inverso
    bottomFace = (n+1):(2*n);
    bottomFace = bottomFace(end:-1:1);
    
    % Definir las caras laterales: cada arista del polígono 2D genera un cuadrilátero.
    sideFaces = zeros(n, 4);
    for i = 1:n
        i_next = mod(i, n) + 1;  % siguiente vértice (cierra el polígono)
        sideFaces(i, :) = [i, i_next, i_next + n, i + n];
    end
    
    % Unir todas las caras en un cell array
    facesCell = { topFace, bottomFace };
    for i = 1:n
         facesCell{end+1} = sideFaces(i, :);
    end
    
    % Convertir el cell array a una matriz numérica con padding (NaN)
    maxVerts = max(cellfun(@length, facesCell));
    faces = NaN(length(facesCell), maxVerts);
    for i = 1:length(facesCell)
         currentFace = facesCell{i};
         faces(i, 1:length(currentFace)) = currentFace;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Función auxiliar: eul2rotMat
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function R = eul2rotMat(phi, theta, psi)
    % Convierte ángulos de Euler (phi, theta, psi) a una matriz de rotación 3x3.
    % Se utiliza la secuencia R = Rz(psi)*Ry(theta)*Rx(phi)
    
    Rx = [1,        0,         0;
          0, cos(phi), -sin(phi);
          0, sin(phi),  cos(phi)];
      
    Ry = [ cos(theta), 0, sin(theta);
                0,    1,      0;
          -sin(theta), 0, cos(theta)];
      
    Rz = [ cos(psi), -sin(psi), 0;
           sin(psi),  cos(psi), 0;
             0,          0,     1];
         
    R = Rz * Ry * Rx;
end
