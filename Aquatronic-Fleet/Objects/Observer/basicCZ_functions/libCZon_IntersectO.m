%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function:    libCZon_IntersectO(Z,C,Y);                                 %
% Description: Computes intersection of constrained zonotope Z with the   %
%              set {z: Cz \in Y}, where Y is a constrained zonotope.      %
% Input:       Z      - constrained zonotope                              %
%              C      - ny-by-nz matrix                                   %
%              Y      - constrained zonotope                              %
% Output:      O      - Intersection                                      %
%              flag   - status flag. Zero if successful.                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [O, flag] = libCZon_IntersectO(Z,C,Y)

    %---------------------------------------------------------%
    %Initialize
    flag  = 0; O=[];
    %---------------------------------------------------------%


    
    %---------------------------------------------------------%
    %Check inputs
    nz  = size(Z{1},1);
    ngz = size(Z{2},2);
    ncz = size(Z{3},1);
    ny  = size(Y{1},1);
    ngy = size(Y{2},2);
    ncy = size(Y{3},1);
    if ( nz<1   || ny<1        ); flag=-1; return; end;
    if ( any(size(C)~=[ny,nz]) ); flag=-1; return; end;
    %---------------------------------------------------------%
    
    
    
    %---------------------------------------------------------%
    %Intersection
    O    = cell(4,1);
    O{1} = Z{1};
    O{2} = [Z{2} zeros(nz,ngy)];
    
    if (ncz~=0 && ncy~=0)
        %Both zonotopes are constrained
        O{3} = [Z{3}           zeros(ncz,ngy)  ;
                zeros(ncy,ngz) Y{3}            ;
                C*Z{2}         -Y{2}          ];
        O{4} = [Z{4} ; Y{4}; Y{1}-C*Z{1}];
    else
        %At most one zonotope is constrained
        if (ncz~=0)
            O{3} = [Z{3}           zeros(ncz,ngy)  ;
                    C*Z{2}         -Y{2}          ];
            O{4} = [Z{4} ; Y{1}-C*Z{1}];
        elseif (ncy~=0)
            O{3} = [zeros(ncy,ngz) Y{3}            ;
                    C*Z{2}         -Y{2}          ];
            O{4} = [Y{4}; Y{1}-C*Z{1}];
        else
            %Niether zonotope is constrained
            O{3} = [C*Z{2} -Y{2}];
            O{4} = Y{1}-C*Z{1};
        end
    end
    %---------------------------------------------------------%
  
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End libCZon_IntersectO                                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%