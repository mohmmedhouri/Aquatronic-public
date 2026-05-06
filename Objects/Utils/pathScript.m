%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autor: Manuel Eduardo Gantiva Osorio
% Proyecto: AQUATRONIC
% Escuela Técnica Superior de Ingeniería 
% Universidad Loyola Andalucía
% Fecha: 24.04.2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script creates an object that defines a desired path based on a parametrised
% curve. It returns a single object that can be used to simulate and reference the
% intended trajectory during vessel navigation or control tasks.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function spatial_path = pathScript()

    spatial_path.x_p = @(w) 40-40*cos(w);
    spatial_path.y_p = @(w) 40*sin(w); 
    spatial_path.dx_p_dw = @(w) 40*sin(w);
    spatial_path.dy_p_dw = @(w) 40*cos(w);
    spatial_path.dx_p_dw2 = @(w) 40*cos(w); 
    spatial_path.dy_p_dw2 = @(w) -40*sin(w); 
    spatial_path.phi_p = @(w) atan2(spatial_path.dy_p_dw(w),spatial_path.dx_p_dw(w)); 

    % spatial_path.x_p = @(w) w;
    % spatial_path.y_p = @(w) 10; 
    % spatial_path.dx_p_dw = @(w) 1;
    % spatial_path.dy_p_dw = @(w) 0;
    % spatial_path.dx_p_dw2 = @(w) 0; 
    % spatial_path.dy_p_dw2 = @(w) 0; 
    % spatial_path.phi_p = @(w) atan2(spatial_path.dx_p_dw(w),spatial_path.dy_p_dw(w)); 

    % spatial_path.x_p = @(w) 100*w*w;
    % spatial_path.y_p = @(w) 100*w; 
    % spatial_path.dx_p_dw = @(w) 200*w;
    % spatial_path.dy_p_dw = @(w) 100;
    % spatial_path.dx_p_dw2 = @(w) 200; 
    % spatial_path.dy_p_dw2 = @(w) 0; 
    % spatial_path.phi_p = @(w) atan2(spatial_path.dx_p_dw(w),spatial_path.dy_p_dw(w)); 

end
