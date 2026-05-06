%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autor: Manuel Eduardo Gantiva Osorio
% Proyecto: AQUATRONIC
% Escuela Técnica Superior de Ingeniería 
% Universidad Loyola Andalucía
% Fecha: 24.04.2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function performs discrete-time integration of a multi-vessel system over a given
% time step. It extracts and integrates the vessel states using a continuous ODE solver,
% updates each vessel's states, and compiles the results—including real. It also updates
% the internal state of the ocean current model to reflect environmental changes over time.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function xk_1 = multi_asv_ode_int(t_0, t_s, xk, m_asv, ocean_currents,noise)
    no_vessels = length(m_asv);
    xk_1 = zeros(no_vessels*22,1);
    no_items = 12;  %Items Derivativos
    xk_0 = [];
    
    for i = 0:no_vessels-1
        start_idx = i * 22 + 1;
        block = xk(start_idx : start_idx + no_items - 1);   % tomar solo las filas 1:12 del bloque
        xk_0 = [xk_0; block];
    end
    
    
    % Aplicar tau en tiempo continuo pero solo considerando los primeros 12 valores de X de cada vehiculo
    opts = odeset('InitialStep',1e-3, 'MaxStep',1e-2,'RelTol',1e-3,'AbsTol',1e-6);
    [~, x_s] = ode45(@(t, X_r) continuousode(t, X_r, m_asv, no_vessels, ocean_currents),...
        [t_0 t_0+t_s], [xk_0; ocean_currents.saved_state_variables], opts);
    
    ocean_currents.saved_state_variables = x_s(end, end-1:end)';


    for v = 1:no_vessels
        m_asv{v}.Estimate(noise);
        aux_skip_rows_1 = ((v-1)*22);
        aux_skip_rows = ((v-1)*12);
    
        x_f =  x_s(end, 1 + aux_skip_rows:12+ aux_skip_rows )' ;
        m_asv{v}.setStates([x_f(1:2);x_f(6:8);x_f(12)]);   %Actualiuzo el estado del Barco [ x y psi u v r]'
        xk_1(1 + aux_skip_rows_1:22 + aux_skip_rows_1) = [x_f; m_asv{v}.getSigmas();  m_asv{v}.getActions()];
    end
end


%% Integrar: [eta_r nu_r state_variables]
function dydt = continuousode(~, X_r, m_asv, no_vessels, ocean_c)

dydt = zeros(no_vessels*12 + 2,1);

all_Vc = ocean_c.step();
Vc = all_Vc(1:3);

% aplicar tau
for v = 1:no_vessels
    
    dydt(1 + ((v-1)*12):12 + ((v-1)*12))  = m_asv{v} ( X_r(1 + ((v-1)*12):12 + ((v-1)*12)), Vc);
end

dydt(end-1:end) = all_Vc(4:5);

end