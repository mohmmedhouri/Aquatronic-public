%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function:    libCZon_Scale(Z);                                          %
% Description: Tightens the fundamental hypercube based on constraints    %
%              and rescales Z accordingly. Procedure does not change the  %
%              set, but makes dualization less conservative. For best     %
%              results, execute libCZon_PartialSolve prior to this        %
%              function.                                                  %
% Input:       Z      - constrained zonotope                              %
% Output:      Z      - constrained zonotope                              %
%              flag   - status flag. Zero if successful.                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [Z,xiL_Score,xiU_Score,flag] = libCZon_Scale(Z)
    %---------------------------------------------------------%
    %Initialize
    FuncID = 'libCZon_Scale';
    flag=0; xiL_Score=[]; xiU_Score=[];
    %---------------------------------------------------------%


    
    %---------------------------------------------------------%
    %Check inputs
    [n  ng] = size(Z{2});
    [nc ~ ] = size(Z{3});
    
    if (nc==0)
        disp(strcat('No constraints on input to ',FuncID));
        return;
    end
    %---------------------------------------------------------%
    
    
    
    %---------------------------------------------------------%
    %Scale
    tol = max(nc,ng)*eps('double')*norm(Z{3},'inf');
    xiL_Score=ones(1,ng)*Inf;
    xiU_Score=xiL_Score;
    for k=1:2
    for row=1:nc
    
        %Tighten xi_i\in [-1,1]
        aT = Z{3}(row,:);
        b  = Z{4}(row)  ;
        aT_Norm=norm(aT,1);
        for col=1:ng

            if ( abs(aT(col))>tol)
                
                aT_NormM=aT_Norm-abs(aT(col));
                xiL_Update = (b/aT(col))-abs(aT_NormM/aT(col));
                xiU_Update = (b/aT(col))+abs(aT_NormM/aT(col));
                
                xi_L=max(-1,xiL_Update);
                xi_U=min( 1,xiU_Update);
                
                xi_r = 0.5*(xi_U-xi_L);
                xi_m = 0.5*(xi_U+xi_L);
                
                if (xiL_Update>=-1)
                    xiL_Score(col) = 0;
                else
                    if (xi_r>tol)
                        xiL_Score(col) = min(xiL_Score(col),(abs(xiL_Update)-1)/xi_r);
                    end
                end
                    
                if (xiU_Update<=1)
                    xiU_Score(col) = 0;
                else
                    if (xi_r>tol)
                        xiU_Score(col) = min(xiU_Score(col),(abs(xiU_Update)-1)/xi_r);
                    end             
                end

                if ( abs(xi_r-1)>tol )

                    %Check feasibility here
                    if (xi_r<-tol)
                       disp(strcat('Empty set generated in ',FuncID))
                       Z{1}=[];
                       Z{2}=[];
                       Z{3}=[];
                       Z{4}=[];
                       return;
                    else
                       xi_r=max(0,xi_r); 
                    end

                    Z{1}=Z{1}+Z{2}(:,col)*xi_m;
                    Z{4}=Z{4}-Z{3}(:,col)*xi_m;
                    Z{2}(:,col)=Z{2}(:,col)*xi_r;
                    Z{3}(:,col)=Z{3}(:,col)*xi_r;

                end
                
            end
            
        end
        
        [Max,maxi] = max(abs(Z{3}(row,:)));
        if (Max>tol)
            Z{4}(row  ) = Z{4}(row  )/Z{3}(row,maxi); 
            Z{3}(row,:) = Z{3}(row,:)/Z{3}(row,maxi); 
        end
        
    end
    end
    %---------------------------------------------------------%
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End libCZon_Scale                                                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%