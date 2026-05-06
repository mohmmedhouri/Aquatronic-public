%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autor: Manuel Eduardo Gantiva Osorio
% Proyecto: AQUATRONIC
% Escuela Técnica Superior de Ingeniería 
% Universidad Loyola Andalucía
% Fecha: 24.04.2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script generates a control action vector over a 200-second simulation period.
% Instead of providing reference values, it defines manual control inputs that can be
% used to simulate operator behaviour and evaluate the performance of the observer.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function hist_ref = genHistTao(no_vessels, tm)
N = length(tm);       % Número de muestras
    % Prealocar vectores para un solo vehículo
Fu     = zeros(1, N);
Fv = zeros(1, N);
Tao   = zeros(1, N);

%% Segmento 1: 0 <= t <= 50 s
idx1 = (tm >= 0) & (tm <= 50);
Fu(idx1)  = 3.0 + 1.0 * sin(0.125663706 * tm(idx1));         
Tao(idx1) = 0.0;        

%% Segmento 2: 50 < t <= 100 s
idx2 = (tm > 50) & (tm <= 100);
Fu(idx2)  = 3.0;  % Velocidad de avance variable
Tao(idx2) = 0.5 * sin(0.125663706 * tm(idx2));                           % Velocidad angular constante

%% Segmento 3: 100 < t <= 150 s
idx3 = (tm > 100) & (tm <= 150);
Fu(idx3)  = 3.0 + 0.5 * sin(0.25132741228 * tm(idx3)); % Variación en la velocidad de avance
Tao(idx3) = 0.0;       % Variación en la velocidad angular

%% Segmento 4: 150 < t <= 200 s
idx4 = (tm > 150) & (tm <= 200);
% La velocidad disminuye linealmente de 1 (en t=150 s) a 0.2 (en t=200 s)
Fu(idx4)  = 3 - ((3 - 1) / (200 - 150)) .* (tm(idx4) - 150);
Tao(idx4) = 0 - ((0 - 1) / (200 - 150)) .* (tm(idx4) - 150);  % Velocidad angular constante


%% Crear la matriz de referencias para un solo vehículo
% Cada vehículo tiene:
%   Fila 1: Force u
%   Fila 2: Force v = 0
%   Fila 3: Torque r
singleRef = [Fu; Fv; Tao];  % Tamaño 3 x N

% figure;
% % Fuerza u
% subplot(3,1,1);
% plot(tm, Fu, 'LineWidth', 1.5);
% grid on;
% ylabel('F_u');
% title('F_u vs. tiempo');
% % Fuerza v (siempre cero, determinación rápida)
% subplot(3,1,2);
% plot(tm, Fv, 'LineWidth', 1.5);
% grid on;
% ylabel('F_v');
% title('F_v vs. tiempo (debería ser cero)');
% % Torque r (Tao)
% subplot(3,1,3);
% plot(tm, Tao, 'LineWidth', 1.5);
% grid on;
% xlabel('Tiempo (s)');
% ylabel('\tau');
% title('\tau vs. tiempo');

%% Replicar el vector de referencia para todos los vehículos
% La matriz final tendrá dimensión (no_vessels*3) x N.
hist_ref = repmat(singleRef, no_vessels, 1);

    
end
