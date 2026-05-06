%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autor: Guillermo Bejarano Pellicer
% Proyecto Ypacaraí
% Escuela Técnica Superior de Ingeniería 
% Universidad Loyola Andalucía
% Fecha: 27.05.2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function x_sat = saturation_control_GB(x,xmax,xmin)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% FUNCIÓN QUE IMPLEMENTA LA FUNCIÓN SATURACIÓN PARA LIMITAR LA ACCIÓN DE CONTROL ENTRE 
% DOS VALORES LÍMITE

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_sat = x;

if x_sat > xmax
    x_sat = xmax;
elseif x_sat < xmin
    x_sat = xmin;
end
