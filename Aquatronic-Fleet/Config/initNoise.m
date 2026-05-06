%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autor: Manuel Eduardo Gantiva Osorio
% Proyecto: AQUATRONIC
% Escuela Técnica Superior de Ingeniería 
% Universidad Loyola Andalucía
% Fecha: 24.04.2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script initialises all parameters related to measurement noise vectors. It
% generates and returns the noise vectors for each time step, which are later used in
% the simulation to model sensor measurement uncertainty.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function noise_reference = initNoise(no_vessels,paramsNoise, Tsim, T_s)
% Noises for ETA
nxy_min = paramsNoise.nxy_min*paramsNoise.flag_noise;
nxy_max = paramsNoise.nxy_max*paramsNoise.flag_noise;
npsi_min = paramsNoise.npsi_min*paramsNoise.flag_noise;
npsi_max = paramsNoise.npsi_max*paramsNoise.flag_noise;
nr_min = paramsNoise.nr_min*paramsNoise.flag_noise;
nr_max = paramsNoise.nr_max*paramsNoise.flag_noise;

nx_seeds = repmat(10, 1, no_vessels);
ny_seeds = repmat(90, 1, no_vessels);
npsi_seeds = repmat(50, 1, no_vessels);
nr_seeds = repmat(70, 1, no_vessels);

t_ref_n = (0:T_s*2:Tsim)';
t_ref_n_h = (0:T_s:Tsim)';

% 3. Número de instantes del perfil temporal:

NP_ref_n = size(t_ref_n,1);
noise_reference = zeros(size(t_ref_n_h,1), 4);
% 4. Definición del perfil temporal del ruido en la componente X del vector de posiciones de los barcos:

for v=1:no_vessels
    ref_nx = zeros(NP_ref_n,1);
    ref_ny = zeros(NP_ref_n,1);
    ref_npsi = zeros(NP_ref_n,1);
    ref_nr = zeros(NP_ref_n,1);
    rng(nx_seeds(v));
    for i=1:NP_ref_n
        ref_nx(i,1) = nxy_min + (nxy_max - nxy_min)*rand(1);
    end

    rng(ny_seeds(v));
    for i=1:NP_ref_n
        ref_ny(i,1) = nxy_min + (nxy_max - nxy_min)*rand(1);
    end

    rng(npsi_seeds(v));
    for i=1:NP_ref_n
        ref_npsi(i,1) = npsi_min + (npsi_max - npsi_min)*rand(1);
    end

    rng(nr_seeds(v));
    for i=1:NP_ref_n
        ref_nr(i,1) = nr_min + (nr_max - nr_min)*rand(1);
    end
end

noise_reference(:, 1: 4) = [pchip(t_ref_n,ref_nx,t_ref_n_h) ...
                            pchip(t_ref_n,ref_ny,t_ref_n_h) ...
                            pchip(t_ref_n,ref_npsi,t_ref_n_h)...
                            pchip(t_ref_n,ref_nr,t_ref_n_h)];

clear i NP_ref_n npsi_min npsi_max npsi_seeds nxy_min nxy_max nx_seeds ny_seeds ref_ny ref_nx t_ref_n ref_npsi sample_time_noise ref_nx_h ref_ny_h ref_npsi_h 

% figure;
% % Ruido en X
% subplot(4,1,1);
% plot(t_ref_n_h, noise_reference(:,1), 'LineWidth', 1.5);
% grid on;
% ylabel('n_x');
% title('Ruido en componente X');
% 
% % Ruido en Y
% subplot(4,1,2);
% plot(t_ref_n_h, noise_reference(:,2), 'LineWidth', 1.5);
% grid on;
% ylabel('n_y');
% title('Ruido en componente Y');
% 
% % Ruido en psi (orientación)
% subplot(4,1,3);
% plot(t_ref_n_h, noise_reference(:,3), 'LineWidth', 1.5);
% grid on;
% ylabel('n_\psi');
% title('Ruido en psi');
% 
% % Ruido en r (velocidad angular)
% subplot(4,1,4);
% plot(t_ref_n_h, noise_reference(:,4), 'LineWidth', 1.5);
% grid on;
% xlabel('Tiempo (s)');
% ylabel('n_r');
% title('Ruido en r');

end