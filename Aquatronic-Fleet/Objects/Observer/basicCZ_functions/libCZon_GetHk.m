%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function:    libCZon_GetHk(Z);                                          %
% Description: Computes (H,k) representation of a constrained zonotope.   %
%              If it is known that the lifted zonotope is full            %
%              dimensional (e.g., [Z{2}; Z{3}] has full row rank), use    %
%              libCZon_GetHk_FullRank for improved efficiency.            %
% Input:       Z      - constrained zonotope                              %
% Output:      H      - H matrix                                          %
%              k      - k vector                                          %
%              flag   - status flag. Zero if successful.                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H, k, flag] = libCZon_GetHk(Z)

    %---------------------------------------------------------%
    %Initialize
    flag  = 0; H = []; k = [];
    %---------------------------------------------------------%


    
    %---------------------------------------------------------%
    %Check inputs
    n  = size(Z{1},1);
    ng = size(Z{2},2);
    %---------------------------------------------------------%
    
    
    
    %---------------------------------------------------------%
    %Eliminate zero constraints
    [Z, nc, flag] = libCZon_RmZeroConstr(Z);
    if (flag); return; end;
    
    %Construct Lifted Zonotope
    [ZL, flag] = libCZon_Lift(Z);
    if (flag); return; end;
       
    %Get (HL,k) Representation of ZL
    [HL,k,flag] = libZon_GetHk(ZL);
    if (flag); return; end;
    
    %Get H
    if (size(HL,1)>0)
      H = HL(:,1:n);
    else
      H = [];
    end
    %---------------------------------------------------------%
  
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End libCZon_GetHk                                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 