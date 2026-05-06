
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autor: Guillermo Bejarano Pellicer
% Proyecto: AQUATRONIC
% Escuela Técnica Superior de Ingeniería 
% Universidad Loyola Andalucía
% Fecha: 28.09.2022
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Pyy = Slin_GB(lambda,w,wo,sigma)
% Pyy = Slin(lambda,w) 2nd-order linear power spectral density (PSD) function
%
%   w       = wave spectrum frequency (rad/s)
%   lambda  = relative damping factor
%
%   See ExLinspec.m
%
% Author:   Thor I. Fossen
% Date:     15th August 2001
% Revisions: 

Pyy = 4*(lambda*wo*sigma)^2*w.^2 ./ ( (wo^2-w.^2).^2 + 4*(lambda*wo.*w).^2 );
