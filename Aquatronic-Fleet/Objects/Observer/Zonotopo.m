classdef Zonotopo < matlab.System
    %ZONOTOPICOBSERVER Summary of this class goes here
    %   Detailed explanation goes here


    properties
        c
        H
    end

    methods (Static)
        function col_mayor = mayor(vec,N,i)

            max = -1;
            col_mayor = 1;
            for k = i:N
                if (vec(1,k)>max)
                    col_mayor = k;
                    max = vec(1,k);
                end
            end
        end
    end

    methods

        function obj = Zonotopo(c, H)
            obj.c = c;
            obj.H = H;
        end

        function L = filtering(obj,y,C,R)
            % Se obtiene el zonotopo de filtrado a partir del zonotopo de
            % predicción y de la medida
            c_ = obj.c;
            H_ = obj.H;

            n = size(c_,1);
            I = eye(n);

            PZ = H_*H_';
            PV = R*R';


            % Ganancia local
            L = (PZ*C')/(C*PZ*C'+PV);

            % Filtrado

            c_fil= c_ + L*(y-C*c_);
            H_fil= [(I-L*C)*H_  -L*R];


            % Elimina columnas de ceros
            num_ceros = 0;
            Hn = zeros(n,size(H_fil,2));
            l = 1;
            for k =1:size(H_fil,2)
                if H_fil(:,k)==0
                    num_ceros = num_ceros +1;
                else
                    Hn(:,l)=H_fil(:,k);
                    l = l+1;
                end
            end
            Hn = Hn(:,1:end-num_ceros);

            update_c_H(obj, c_fil, Hn);
        end

        function update_c_H(obj, newc, newH)
            obj.c = newc;
            obj.H = newH;
        end

        function reduccion_orden(obj, s, W)
            H1 = obj.H;
            c1 = obj.c;
            r = size(H1,2);

            n = size(c1,1);
            Hred = zeros(n,s);
            %Comprueba que el orden que se pide es correcto
            if s>=r
                return
            end

            % Creación de vector de normas euclídeas de las columnas de H1:
            norma = zeros(1,r);

            for j=1:r
                norma(1,j) = sqrt(H1(:,j)'*W*H1(:,j));
            end

            % Ordenación del vector norma y de las columnas de la matriz H1 en orden decreciente de su norma euclídea:

            vec_aux = zeros(n,1);

            for j=1:r-1
                col_mayor = obj.mayor(norma,r,j);
                aux = norma(1,j);
                for i=1:n
                    vec_aux(i,1) = H1(i,j);
                end
                norma(1,j) = norma(1,col_mayor);
                for i=1:n
                    H1(i,j) = H1(i,col_mayor);
                end
                norma(1,col_mayor) = aux;
                for i=1:n
                    H1(i,col_mayor) = vec_aux(i,1);
                end
            end

            Q = zeros(n,n);

            for i=1:n
                for j=1:n
                    aux = 0;
                    if (i==j)
                        for k=(s-n+1):r
                            aux = aux + abs(H1(i,k));
                        end
                        Q(i,j) = aux;
                    end
                end
            end

            % Cálculo de la matriz Hred:
            Hred(:,1:(s-n)) = H1(:,1:(s-n));
            Hred(:,s-n+1:s) = Q;
            update_c_H(obj, c1, Hred);
        end

    end

end

