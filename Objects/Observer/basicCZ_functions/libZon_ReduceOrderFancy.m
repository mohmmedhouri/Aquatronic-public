%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function:    libZon_ReduceOrderFancy                                    %
% Description: Reduceds the order of a zonotope by hueristically          %
%              searching for an optimal set of generators to aggregate    %
%              into a parallelatope.                                      %
% Input:       Z      - initial zonotope                                  %
%              zo     - desired order                                     %
% Output:      Zp     - reduced zonotope                                  %
%              ng     - reduced zonotope order                            %
%              flag   - status flag. Zero if successful.                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Zp,ng,flag] = libZon_ReduceOrderFancy( Z , zo )

    %---------------------------------------------------------%
    %Initialize
    FuncID = 'libZon_ReduceOrderFancy';
    flag = 0; Zp=Z;
    %---------------------------------------------------------%


    
    %---------------------------------------------------------%
    %Check inputs
    [n  ng] = size(Z{2});
    if (n <= 0 || ng <= 0)
        disp(strcat('Input error in ',FuncID));
        flag=-1; return;
    end
    if (zo<1)
        disp(strcat('Input error in ',FuncID));
        flag=-1; return;
    end
    %---------------------------------------------------------%
    
    
    
    %---------------------------------------------------------%
    %Compute order
    N_Agg = floor(ng+(1-zo)*n);
    if (N_Agg<=n); return; end;
    %---------------------------------------------------------%
    
    
    
    %---------------------------------------------------------%
    %Split zonotope into Z=Z1+Z2.
    %  -Z2 will be overapproximated by a parallelotope.
    %  -The split is determined by ordering the generators by
    %   2-norm.
    
    %Order generators by 2-norm
    L = Z{2}.*Z{2};
    two_norm = sum(L,1);
    [~,index] = sort(two_norm);
    Ordered_Gen = Z{2}(:,index(1:ng));
    
    %Select several candidate generators
    et = min(n+8,ng);                   %<-- This number of candidates is 
                                        %    suggested by Althoff
    etG = Ordered_Gen(:,ng-et+1:ng); %<-- Select longest et generators
    
    %Columns of Pi are chosen as some combination of n generators
    %out of the et selected above.
    
    %Compute all combinations
    Combs = combnk(1:et,n);
    nc = size(Combs,1);
    
    %Compute cheap volume measure for every combination
    for i=1:nc
        G = etG( : , Combs(i,:) );
        vt(i) = 1/abs( det(G) );   
    end
    
    %Sort nc combinations by vt
    [~, index2] = sort(vt);
    
    %Final Pi Matrix
    Pi = etG( : , Combs(index2(1),:) );
    if (cond(Pi)>1e14)
       Pi=eye(n); 
    end
    %---------------------------------------------------------%
    
    
    
    %---------------------------------------------------------%
    %Reduce
    Zp{1} = Pi\Z{1};
    Zp{2} = Pi\Z{2};
%     try
    [Zp,ng,flag] = libZon_ReduceOrder(Zp,zo,'Chisci'); % Prova MIA
    if (flag); return; end;
%     catch
%         keyboard
%     end
    
    [Zp, flag] = libZon_MatMult(Pi,Zp);
    if (flag); return; end;
    %---------------------------------------------------------%
  
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End libZon_ReduceOrderFancy                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%