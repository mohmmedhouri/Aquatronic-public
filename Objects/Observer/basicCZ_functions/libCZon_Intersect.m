%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function:    libCZon_Intersect(Z,Y);                                    %
% Description: Computes intersection of constrainted zonotopes.           %
% Input:       Z      - constrained zonotope                              %
%              Y      - constrained zonotope                              %
% Output:      ZcY    - Intersection                                      %
%              flag   - status flag. Zero if successful.                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ZcY, flag] = libCZon_Intersect(Z,Y)

    %---------------------------------------------------------%
    %Initialize
    flag  = 0; ZcY=[];
    %---------------------------------------------------------%


    
    %---------------------------------------------------------%
    %Check inputs
    %Check inputs
    nz  = size(Z{1},1);
    ngz = size(Z{2},2);
    ncz = size(Z{3},1);
    ny  = size(Y{1},1);
    ngy = size(Y{2},2);
    ncy = size(Y{3},1);
    
    if ( nz<1 || ny<1 ) flag=-1; return; end;
    if ( nz ~= ny     ) flag=-1; return; end;
    %---------------------------------------------------------%
    
    
    
    %---------------------------------------------------------%
    %Intersection
    ZcY    = cell(4,1);
    ZcY{1} = Z{1};
    ZcY{2} = [Z{2} zeros(nz,ngy)];
    
    if (ncz~=0 && ncy~=0)
        %Both zonotopes are constrained
        ZcY{3} = [Z{3}           zeros(ncz,ngy) ;
                  zeros(ncy,ngz)  Y{3}           ;
                  Z{2}           -Y{2}          ];
        ZcY{4} = [Z{4} ; Y{4}; Y{1}-Z{1}];
    else
        %At most one zonotope is constrained
        if (ncz~=0)
            ZcY{3} = [Z{3}           zeros(ncz,ngy) ;
                      Z{2}           -Y{2}          ];
            ZcY{4} = [Z{4} ; Y{1}-Z{1}];
        elseif (ncy~=0)
            ZcY{3} = [zeros(ncy,ngz)  Y{3}  ;
                      Z{2}           -Y{2} ];
            ZcY{4} = [Y{4}; Y{1}-Z{1}];
        else
            %Niether zonotope is constrained
            ZcY{3} = [Z{2} -Y{2}];
            ZcY{4} = [Y{1}-Z{1}];
        end
    end
   
    %---------------------------------------------------------%
  
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End libCZon_Intersect                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%