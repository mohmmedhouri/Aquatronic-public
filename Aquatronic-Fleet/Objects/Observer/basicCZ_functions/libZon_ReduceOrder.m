%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function:    libZon_ReduceOrder                                         %
% Description: Reduces the order of a zonotope using one of three methods.%
% Input:       Z      - initial zonotope                                  %
%              zo     - desired order                                     %
%              meth   - method ( 'Simple' , 'Althoff' , 'Chisci' )        %
% Output:      Z      - reduced zonotope                                  %
%              ng     - number of generators in reduced zonotope          %
%              flag   - status flag. Zero if successful.                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Z,ng,flag] = libZon_ReduceOrder( Z , zo , meth )

    %---------------------------------------------------------%
    %Initialize
    flag = 0;
    %---------------------------------------------------------%


    
    %---------------------------------------------------------%
    %Choose Method
    if ( strcmp(meth,'Simple') )
        [Z,ng,flag] = libZon_ReduceOrderSimple( Z , zo );
    end
    
    if ( strcmp(meth,'Althoff') )
        [Z,ng,flag] = libZon_ReduceOrderFancy( Z , zo );
    end
    
    if ( strcmp(meth,'Chisci') )
        [Z,ng,flag] = libZon_ReduceOrderChisci( Z , zo );
    end
    
    if ( ~strcmp(meth,'Simple') && ~strcmp(meth,'Althoff') && ~strcmp(meth,'Chisci') )
        disp('Invalid method in libZon_ReduceOrder');
        flag=-1;
    end
    %---------------------------------------------------------%
  
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End libZon_ReduceOrder                                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%