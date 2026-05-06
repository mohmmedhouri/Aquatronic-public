%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function:    libCZon_ReduceDOFOrder                                     %
% Description: Reduces the dof order of a constrained zonotope using one  %
%              of three methods.                                          %
% Input:       Z      - constrained zonotope                              %
%              ddofo  - desired degrees of freedom order                  %
%              meth   - method ('Simple','Althoff','Chisci')              %
% Output:      Z      - reduced constrained zonotope                      %
%              ng     - number of generators in reduced zonotope          %
%              flag   - status flag. Zero if successful.                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Z,ng,flag] = libCZon_ReduceDOFOrder( Z , ddofo , meth )

    %---------------------------------------------------------%
    %Initialize
    flag = 0;
    %---------------------------------------------------------%


    
    %---------------------------------------------------------%
    %Check inputs
    FuncID = 'libCZon_ReduceDOFOrder';
    [n,  ng] = size(Z{2});
    [nc, ~ ] = size(Z{3});
    if ( n <= 0 || ng <= 0 )
        disp(strcat('Empty c. zonotope on input to  ',FuncID));
        flag=-1; return;
    end
    if ( ddofo<1 )
        disp(strcat('Invalid order on input to  ',FuncID));
        flag=-1; return;
    end
    %---------------------------------------------------------%
    
    
    
    %---------------------------------------------------------%
    %Compute desired lifted zonotope order
    delta_dof=ng-nc-ddofo*n;
    if (delta_dof<=0); return; end;
    dlzo=(ng-delta_dof)/(n+nc);
    
    lastwarn('')
    %Compute lifted zonotope
    
    % THIS IS TO STOP WARNINGS!!!!
    if min(size(Z{3}))==0
        Z{3}=[];
    end
    
    ZL=cell(4,1);
    ZL{1}=[Z{1}; -Z{4}];
    ZL{2}=[Z{2};  Z{3}];
    
   
    %Reduce lifted zonotope
    [ZL,ng,flag] = libZon_ReduceOrder(ZL,dlzo,meth);
    if (flag); return; end;
    
    %Return to original space
    Z{2}=ZL{2}(  1:n   ,:);
    Z{3}=ZL{2}(n+1:n+nc,:);
    %---------------------------------------------------------%
  
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End libCZon_ReduceDOFOrder                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%