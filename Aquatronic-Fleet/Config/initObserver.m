%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autor: Manuel Eduardo Gantiva Osorio
% Proyecto: AQUATRONIC
% Escuela Técnica Superior de Ingeniería 
% Universidad Loyola Andalucía
% Fecha: 24.04.2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script initialises all necessary parameters for creating an observer object. The
% parameters are assigned based on the selected observer type, allowing for flexible
% configuration of different estimation strategies within the simulation.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function observer_param = initObserver(observer, Ts)
    observer_param.obs = observer;
    switch observer
        case 1
            observer_param.epsilon = 0.1 ;
            observer_param.alpha = 1.0;
            observer_param.Ts = Ts;
        case 2
            observer_param.Lbar_psi = [10.3703722483136; 42.7624634190640; 76.0083203318361];
            observer_param.PpWp = [6.7189695799299,0.0;
                                   0.0,6.7189695799299;
                                   5.10273077523022,0.0;
                                   0.0,5.10273077523022;
                                   1.56374424601122,0.0;
                                   0.0,1.56374424601122];
            observer_param.Ts = Ts;
        case 3
            observer_param.Ts = Ts;

            observer_param.Max_n_r = [0.01 0.001];
            observer_param.Max_n_p = [0.05 0.05];
            observer_param.orden_est_r = 300;
            observer_param.orden_est_p = observer_param.orden_est_r*2;

            % observer_param.Max_w_r = 0.03;
            % observer_param.Max_w_p = 0.001;

            observer_param.Max_w_r = 0.03;
            observer_param.Max_w_p = 0.8;
            observer_param.Wp = [1 1 100 100 1 1];
            observer_param.Wr = [1 100 1];
        otherwise
            observer_param.obs = observer;
    end
end

