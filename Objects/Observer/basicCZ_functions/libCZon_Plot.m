%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function:    libCZon_Plot(Z);                                           %
% Description: Plots a constrained zonotope.                              %
% Input:       Z      - constrained zonotope                              %
%              options - structure with elements:                         %
%              .color  - string indicating plot color; e.g., 'k'          %
% Output:      flag   - status flag. Zero if successful.                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function flag = libCZon_Plot(Z,options)

    %---------------------------------------------------------%
    %Initialize
    flag  = 0;
    %---------------------------------------------------------%


    
    %---------------------------------------------------------%
    %Check inputs
    %---------------------------------------------------------%
    
    
    
    %---------------------------------------------------------%
    %Plot
    [H, k, flag] = libCZon_GetHk(Z);
    if (flag); return; end;
    
    if (size(H,1)>0)
        P = Polyhedron('A',H,'b',k);
        plot(P,'color',options.color);
    end
    
    %V = extreme(P);
    %indices = convhull(V(:,1),V(:,2));
    %Vt = V(indices,:);
    %plot(Vt(:,1),Vt(:,2),'b-');
    %---------------------------------------------------------%
  
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End libCZon_Plot                                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 