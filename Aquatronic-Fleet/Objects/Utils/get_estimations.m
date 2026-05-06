%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autor: Manuel Eduardo Gantiva Osorio
%        Guillermo Bejarano Pellicer
% Proyecto: AQUATRONIC
% Escuela Técnica Superior de Ingeniería 
% Universidad Loyola Andalucía
% Fecha: 24.04.2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function serves as a utility to collect and store data from all state estimators.
% It compiles the estimation results into a history vector, functioning solely as a data
% storage tool for later analysis.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function xk_hat = get_estimations(m_asv)
    no_vessels = length(m_asv);
    xk_hat = zeros(9*no_vessels,1);
    for v = 1:no_vessels
        hats =  m_asv{v}.observer_obj.getEstimations();
        xk_hat(1+(v-1)*9:9+(v-1)*9) = hats;
    end
end