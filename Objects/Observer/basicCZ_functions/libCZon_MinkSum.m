%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function:    libCZon_MinkSum(Z,Y);                                      %
% Description: Computes minkowski sum of constrainted zonotopes.          %
% Input:       Z      - constrained zonotope                              %
%              Y      - constrained zonotope                              %
% Output:      ZpY    - Minkowski sum                                     %
%              flag   - status flag. Zero if successful.                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ZpY, flag] = libCZon_MinkSum(Z,Y)

    %---------------------------------------------------------%
    %Initialize
    flag  = 0; ZpY=[];
    %---------------------------------------------------------%


    
    %---------------------------------------------------------%
    %Check inputs
    [nz ngz] = size(Z{2});
    ncz = size(Z{3},1);
    [ny ngy] = size(Y{2});
    ncy = size(Y{3},1);
    
    if (nz ~= ny); flag=-1; return; end;
    %---------------------------------------------------------%
    
    
    
    %---------------------------------------------------------%
    %Sum
    ZpY    = cell(4,1);
    ZpY{1} = Z{1}+Y{1};
    ZpY{2} = [Z{2} Y{2}];
    
    if (ncz~=0 && ncy~=0)
        %Both zonotopes are constrained
        ZpY{3} = [Z{3}           zeros(ncz,ngy) ;
                  zeros(ncy,ngz) Y{3}          ];
        ZpY{4} = [Z{4} ; Y{4}];
    else
        %At most one zonotope is constrained
        if (ncz~=0)
            ZpY{3} = [Z{3} zeros(ncz,ngy)];
            ZpY{4} = Z{4};
        elseif (ncy~=0)
            ZpY{3} = [zeros(ncy,ngz) Y{3}];
            ZpY{4} = Y{4};
        else
            %Niether zonotope is constrained
            ZpY{3} = [];
            ZpY{4} = [];
        end
    end
    %---------------------------------------------------------%
  
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End libCZon_MinkSum                                                     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%