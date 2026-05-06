%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function:    libZon_GetHk(Z);                                           %
% Description: Computes (H,k) representation of a zonotope. If it is      %
%              known that the input zonotope is full dimensional (e.g.,   %
%              Z{2} has full row rank), use libZon_GetHk_FullRank for     %
%              improved efficiency. This function uses a modification of  %
%              the method of Althoff to account for possibly non-full     %
%              dimensional zonotopes.                                     %
% Input:       Z      - zonotope                                          %
% Output:      H      - H matrix                                          %
%              k      - k vector                                          %
%              flag   - status flag. Zero if successful.                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H, k, flag] = libZon_GetHk(Z)

    %---------------------------------------------------------%
    %Initialize
    FuncID = 'libZon_GetHk';
    flag  = 0; H = []; k = [];
    %---------------------------------------------------------%


    
    %---------------------------------------------------------%
    %Check inputs
    c=Z{1};
    G=Z{2};
    [n,ng]=size(G);

    if (n==0 || ng==0);
        disp(['Improper inputs to ',FuncID]);
        flag=-1;
        return;
    end
    %---------------------------------------------------------%
    
    
    
    %---------------------------------------------------------%
    %Get number of possible facets
    GF=[G eye(n)];
    [~,ngF]=size(GF);
    comb=combnk(1:ngF,n-1);
    NumCombs = size(comb,1);
    %comb=combinator(nrOfGeneratorsF,dim-1,'c');
    %---------------------------------------------------------%
    
    
    
    %---------------------------------------------------------%
    %Build H matrix
    H1=[];
    nStrips=0;

    if (n==1)
        H1=1;
        k=norm(G,1); % +c; see below
    else
        for i=1:NumCombs
            indices=comb(i,:);
            Q=GF(:,indices);
            v=ndimCross(Q);
            nv = norm(v);
            if nv>1e-10;
                
                nStrips = nStrips+1;
                H1(nStrips,:)=v'/nv;
                
            end
        end
        k = sum( abs( H1*G ) ,2);
    end
    
    if (size(k,1)>0)
        H=[H1;-H1];
        k=H*c+[k;k];
    end
    %---------------------------------------------------------%
    
    if (0)
        xwidth = norm(G(1,:),1);
        ywidth = norm(G(2,:),1);
        m = (size(k,1)/2);
        for i=1:m
            if ( abs(H(i,2))>1e-8 )
                x1=linspace(c(1)-2*xwidth,c(1)+2*xwidth,20);
                x2=linspace(c(1)-2*xwidth,c(1)+2*xwidth,20);
                y1=(k(i)-H(i,1)*x1)/H(i,2);
                y2=(k(m+i)-H(m+i,1)*x2)/H(m+i,2);
            else
                if ( abs(H(i,1))>1e-8 )
                    color='m';
                    y1=linspace(c(2)-2*ywidth,c(2)+2*ywidth,20);
                    y2=linspace(c(2)-2*ywidth,c(2)+2*ywidth,20);
                    x1 = k(i)/H(i,1);
                    x2 = k(m+i)/H(m+i,1);
                end
            end
            plot(x1,y1,'b');
            plot(x2,y2,'r');
        end
    end
  
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End libZon_GetHk                                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 