%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function:    libZon_ReduceOrderSimple                                   %
% Description: Reduces the order of a zonotope cheaply by aggregating     %
%              some generators into an interval                           %
% Input:       Z      - initial zonotope                                  %
%              zo     - desired order                                     %
% Output:      Z      - reduced zonotope                                  %
%              ng     - number of generators in reduced zonotope          %
%              flag   - status flag. Zero if successful.                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Z,ng,flag] = libZon_ReduceOrderSimple( Z , zo )

    %---------------------------------------------------------%
    %Initialize
    flag = 0;
    %---------------------------------------------------------%


    
    %---------------------------------------------------------%
    %Check inputs
    [n ng] = size(Z{2});
    if (n <= 0 || ng <= 0) ;   flag=-1; return; end;
    if (zo<1);                 flag=-1; return; end;
    %---------------------------------------------------------%
    
    
    
    %---------------------------------------------------------%
    %Number of generators to aggregate into n generators
    N_Agg = floor(ng+(1-zo)*n);
    if (N_Agg<=n); return; end;
    
    %Order generators by nearness to a scaled unit vector
    L = abs(Z{2});
    norm_diff = (sum(L)-max(L))';
    [norm_diff, index] = sort(norm_diff);
    
    %Aggregate N_Agg generators into n generators
    Ordered_Gen = Z{2}(:,index(1:ng));
    L2 = abs( Ordered_Gen(:,1:N_Agg) );
    mag = sum(L2,2);
    Z{2} = [diag(mag) Ordered_Gen(:,N_Agg+1:ng) ];
    
    %Update ng
    [~, ng] = size(Z{2});
    %---------------------------------------------------------%
  
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End libZon_ReduceOrderSimple                                            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%