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
function [spatial_path_CasADI, spatial_path] = pathScriptCasADI(w, asv)

    if nargin < 2 || isempty(asv)
        asv = 0;   % o el valor por defecto que quieras
    end

    % spatial_path_CasADI.x_p = 30*cos(w);
    % spatial_path_CasADI.y_p = 20*sin(2*w); 
    % spatial_path_CasADI.dx_p_dw = -30*sin(w);
    % spatial_path_CasADI.dy_p_dw = 40*cos(2*w);
    % spatial_path_CasADI.dx_p_dw2 = -30*cos(w); 
    % spatial_path_CasADI.dy_p_dw2 = -80*sin(2*w); 
    % spatial_path_CasADI.dx_p_dw3 = 30*sin(w); 
    % spatial_path_CasADI.dy_p_dw3 = -160*cos(2*w); 
    % spatial_path_CasADI.phi_p = atan2(spatial_path_CasADI.dy_p_dw,spatial_path_CasADI.dx_p_dw); 
    % spatial_path.x_p = @(w) 30*cos(w);
    % spatial_path.y_p = @(w) 20*sin(2*w); 
    % spatial_path.dx_p_dw = @(w) -30*sin(w);
    % spatial_path.dy_p_dw = @(w) 40*cos(2*w);
    % spatial_path.dx_p_dw2 = @(w) -30*cos(w);
    % spatial_path.dy_p_dw2 = @(w) -80*sin(2*w);
    % spatial_path.phi_p = @(w) atan2(spatial_path.dy_p_dw(w),spatial_path.dx_p_dw(w)); 
    % spatial_path.dphi_p_dw = @(w) (spatial_path.dy_p_dw2(w).*spatial_path.dx_p_dw(w) - spatial_path.dx_p_dw2(w).*spatial_path.dy_p_dw(w)) ./ ...
    %                               (spatial_path.dx_p_dw(w).^2 + spatial_path.dy_p_dw(w).^2);

    % spatial_path_CasADI.x_p = 30-30*cos(w);
    % spatial_path_CasADI.y_p = 30*sin(w); 
    % spatial_path_CasADI.dx_p_dw = 30*sin(w);
    % spatial_path_CasADI.dy_p_dw = 30*cos(w);
    % spatial_path_CasADI.dx_p_dw2 = 30*cos(w); 
    % spatial_path_CasADI.dy_p_dw2 = -30*sin(w); 
    % spatial_path_CasADI.dx_p_dw3 = -30*sin(w); 
    % spatial_path_CasADI.dy_p_dw3 = -30*cos(w); 
    % spatial_path_CasADI.phi_p = atan2(spatial_path_CasADI.dy_p_dw,spatial_path_CasADI.dx_p_dw); 
    % spatial_path.x_p = @(w) 30-30*cos(w);
    % spatial_path.y_p = @(w) 30*sin(w); 
    % spatial_path.dx_p_dw = @(w) 30*sin(w);
    % spatial_path.dy_p_dw = @(w) 30*cos(w);
    % spatial_path.dx_p_dw2 = @(w) 30*cos(w);
    % spatial_path.dy_p_dw2 = @(w) -30*sin(w);
    % spatial_path.phi_p = @(w) atan2(spatial_path.dy_p_dw(w),spatial_path.dx_p_dw(w)); 
    % spatial_path.dphi_p_dw = @(w) (spatial_path.dy_p_dw2(w).*spatial_path.dx_p_dw(w) - spatial_path.dx_p_dw2(w).*spatial_path.dy_p_dw(w)) ./ ...
    %                               (spatial_path.dx_p_dw(w).^2 + spatial_path.dy_p_dw(w).^2);

    % spatial_path_CasADI.x_p = w;
    % spatial_path_CasADI.y_p = w;
    % spatial_path_CasADI.dx_p_dw = 1;
    % spatial_path_CasADI.dy_p_dw = 1;
    % spatial_path_CasADI.dx_p_dw2 = 0;
    % spatial_path_CasADI.dy_p_dw2 = 0;
    % spatial_path_CasADI.dx_p_dw3 = 0;
    % spatial_path_CasADI.dy_p_dw3 = 0;
    % spatial_path_CasADI.phi_p = atan2(spatial_path_CasADI.dy_p_dw,spatial_path_CasADI.dx_p_dw); 
    % spatial_path.x_p = @(w) w;
    % spatial_path.y_p = @(w) w; 
    % spatial_path.dx_p_dw = @(w) 1;
    % spatial_path.dy_p_dw = @(w) 1;
    % spatial_path.dx_p_dw2 = @(w) 0;
    % spatial_path.dy_p_dw2 = @(w) 0;
    % spatial_path.phi_p = @(w) atan2(spatial_path.dy_p_dw(w),spatial_path.dx_p_dw(w)); 
    % spatial_path.dphi_p_dw = @(w) (spatial_path.dy_p_dw2(w).*spatial_path.dx_p_dw(w) - spatial_path.dx_p_dw2(w).*spatial_path.dy_p_dw(w)) ./ ...
    %                               (spatial_path.dx_p_dw(w).^2 + spatial_path.dy_p_dw(w).^2);
    

    % spatial_path_CasADI.x_p = w;
    % spatial_path_CasADI.y_p = 10.0;
    % spatial_path_CasADI.dx_p_dw = 1;
    % spatial_path_CasADI.dy_p_dw = 0;
    % spatial_path_CasADI.dx_p_dw2 = 0;
    % spatial_path_CasADI.dy_p_dw2 = 0;
    % spatial_path_CasADI.dx_p_dw3 = 0;
    % spatial_path_CasADI.dy_p_dw3 = 0;
    % spatial_path_CasADI.phi_p = atan2(spatial_path_CasADI.dy_p_dw,spatial_path_CasADI.dx_p_dw); 
    % spatial_path.x_p = @(w) w;
    % spatial_path.y_p = @(w) 10.0; 
    % spatial_path.dx_p_dw = @(w) 1;
    % spatial_path.dy_p_dw = @(w) 0;
    % spatial_path.dx_p_dw2 = @(w) 0;
    % spatial_path.dy_p_dw2 = @(w) 0;
    % spatial_path.phi_p = @(w) atan2(spatial_path.dy_p_dw(w),spatial_path.dx_p_dw(w)); 
    % spatial_path.dphi_p_dw = @(w) (spatial_path.dy_p_dw2(w).*spatial_path.dx_p_dw(w) - spatial_path.dx_p_dw2(w).*spatial_path.dy_p_dw(w)) ./ ...
    %                               (spatial_path.dx_p_dw(w).^2 + spatial_path.dy_p_dw(w).^2);

    if(asv == 0)
        spatial_path_CasADI.x_p = 10-10*cos(w);
        spatial_path_CasADI.y_p = 10*sin(w); 
        spatial_path_CasADI.dx_p_dw = 10*sin(w);
        spatial_path_CasADI.dy_p_dw = 10*cos(w);
        spatial_path_CasADI.dx_p_dw2 = 10*cos(w); 
        spatial_path_CasADI.dy_p_dw2 = -10*sin(w); 
        spatial_path_CasADI.dx_p_dw3 = -10*sin(w); 
        spatial_path_CasADI.dy_p_dw3 = -10*cos(w); 
        spatial_path_CasADI.phi_p = atan2(spatial_path_CasADI.dy_p_dw,spatial_path_CasADI.dx_p_dw); 
        spatial_path.x_p = @(w) 10-10*cos(w);
        spatial_path.y_p = @(w) 10*sin(w); 
        spatial_path.dx_p_dw = @(w) 10*sin(w);
        spatial_path.dy_p_dw = @(w) 10*cos(w);
        spatial_path.dx_p_dw2 = @(w) 10*cos(w);
        spatial_path.dy_p_dw2 = @(w) -10*sin(w);
        spatial_path.phi_p = @(w) atan2(spatial_path.dy_p_dw(w),spatial_path.dx_p_dw(w)); 
        spatial_path.dphi_p_dw = @(w) (spatial_path.dy_p_dw2(w).*spatial_path.dx_p_dw(w) - spatial_path.dx_p_dw2(w).*spatial_path.dy_p_dw(w)) ./ ...
                                      (spatial_path.dx_p_dw(w).^2 + spatial_path.dy_p_dw(w).^2);
    elseif(asv == 1)
        spatial_path_CasADI.x_p = 10-8*cos(w);
        spatial_path_CasADI.y_p = 8*sin(w); 
        spatial_path_CasADI.dx_p_dw = 8*sin(w);
        spatial_path_CasADI.dy_p_dw = 8*cos(w);
        spatial_path_CasADI.dx_p_dw2 = 8*cos(w); 
        spatial_path_CasADI.dy_p_dw2 = -8*sin(w); 
        spatial_path_CasADI.dx_p_dw3 = -8*sin(w); 
        spatial_path_CasADI.dy_p_dw3 = -8*cos(w); 
        spatial_path_CasADI.phi_p = atan2(spatial_path_CasADI.dy_p_dw,spatial_path_CasADI.dx_p_dw); 
        spatial_path.x_p = @(w) 10-8*cos(w);
        spatial_path.y_p = @(w) 8*sin(w); 
        spatial_path.dx_p_dw = @(w) 8*sin(w);
        spatial_path.dy_p_dw = @(w) 8*cos(w);
        spatial_path.dx_p_dw2 = @(w) 8*cos(w);
        spatial_path.dy_p_dw2 = @(w) -8*sin(w);
        spatial_path.phi_p = @(w) atan2(spatial_path.dy_p_dw(w),spatial_path.dx_p_dw(w)); 
        spatial_path.dphi_p_dw = @(w) (spatial_path.dy_p_dw2(w).*spatial_path.dx_p_dw(w) - spatial_path.dx_p_dw2(w).*spatial_path.dy_p_dw(w)) ./ ...
                                      (spatial_path.dx_p_dw(w).^2 + spatial_path.dy_p_dw(w).^2);
    elseif(asv == 2)
        spatial_path_CasADI.x_p = 10-12*cos(w);
        spatial_path_CasADI.y_p = 12*sin(w); 
        spatial_path_CasADI.dx_p_dw = 12*sin(w);
        spatial_path_CasADI.dy_p_dw = 12*cos(w);
        spatial_path_CasADI.dx_p_dw2 = 12*cos(w); 
        spatial_path_CasADI.dy_p_dw2 = -12*sin(w); 
        spatial_path_CasADI.dx_p_dw3 = -12*sin(w); 
        spatial_path_CasADI.dy_p_dw3 = 1210*cos(w); 
        spatial_path_CasADI.phi_p = atan2(spatial_path_CasADI.dy_p_dw,spatial_path_CasADI.dx_p_dw); 
        spatial_path.x_p = @(w) 10-12*cos(w);
        spatial_path.y_p = @(w) 12*sin(w); 
        spatial_path.dx_p_dw = @(w) 12*sin(w);
        spatial_path.dy_p_dw = @(w) 12*cos(w);
        spatial_path.dx_p_dw2 = @(w) 12*cos(w);
        spatial_path.dy_p_dw2 = @(w) -12*sin(w);
        spatial_path.phi_p = @(w) atan2(spatial_path.dy_p_dw(w),spatial_path.dx_p_dw(w)); 
        spatial_path.dphi_p_dw = @(w) (spatial_path.dy_p_dw2(w).*spatial_path.dx_p_dw(w) - spatial_path.dx_p_dw2(w).*spatial_path.dy_p_dw(w)) ./ ...
                                      (spatial_path.dx_p_dw(w).^2 + spatial_path.dy_p_dw(w).^2);
    end    
    % 
end
