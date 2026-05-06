    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function:    libCZon_RmZeroConstr(Z)                                    %
% Description: Removes zero constraints.                                  %
% Input:       Z      - constrained zonotope                              %
% Output:      Z      - modified constrained zonotope                     %
%              flag   - status flag. Zero if successful.                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Modified by Brenner in 06-02-2019: added ZeroInd as output variable

function [Z, nc, flag, ZeroInd] = libCZon_RmZeroConstr(Z)

    %---------------------------------------------------------%
    %Initialize
    flag  = 0;
    %---------------------------------------------------------%


    
    %---------------------------------------------------------%
    %Check inputs
    [n  ng] = size( Z{2} );
    [nc ~ ] = size( Z{3} );
    %if (nc<1); return; end;
    if (nc<1); ZeroInd = []; return; end; % MODIFIED BY BRENNER
    %---------------------------------------------------------%
    
    
    
    %---------------------------------------------------------%
    %Remove trivial constraints
    tol = max(nc,ng)*eps('double')*norm(Z{3},'inf');
    RowMaxs = max(abs(Z{3}),[],2);
    ZeroInd = find(RowMaxs<=ones(nc,1)*tol);
    
    for i=1:length(ZeroInd)
        if abs(Z{4}(ZeroInd(i)))>tol
            %The set is empty
            flag=-4;
            Z{1}=[];
            Z{2}=[];
            Z{3}=[];
            Z{4}=[];
            return;
        end
    end
    Z{3}(ZeroInd,:)=[];
    %Z{4}(ZeroInd  )=[];
    Z{4}(ZeroInd,:)=[]; % MODIFIED BY BRENNER (avoids an empty column vector to be mistakely turned into a empty row vector by MATLAB)
    
    %Update nc
    nc = size(Z{3},1);
    %---------------------------------------------------------%
  
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End libCZon_RmZeroConstr                                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%