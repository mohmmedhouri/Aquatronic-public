clear all
close all

options.color='brgymck';
options.irnc=1;
options.irdofo=1;
options.gmeth='Chisci';

% Z=libCZon_Rand(n,nc,ng);

Z{1}=[0;0];
Z{2}=[eye(2) [1;-1] [1;-2] [1;-3] [1;-4]];
Z{3}=[0 0 0 -1 4 -3;-2 6 -4 0 0 0;0 1 0 -2 0 3;2 0 -2 0 0 -4];
Z{4}=[1;-1;3;-3];
n=2;nc=4;ng=6;


Z2{1}=0;
Z2{2}=1;
Z2{3}=[];
Z2{4}=[];
O=libCZon_IntersectO(Z,[1 1],Z2);
options1.color=options.color(1);
libCZon_Plot(Z,options1);
options1.color=options.color(2);
hold on
libCZon_Plot(O,options1);


i=1;
Zloop{i}.Z=Z;
Zloop{i}.ng=ng;
Zloop{i}.nc=nc;

while Zloop{i}.nc>options.irnc
    i=i+1;
    [Zloop{i}.Z, Zloop{i}.ng, Zloop{i}.nc, flag, xi_elim] = libCZon_ScaleDualize3(Zloop{i-1}.Z);
    if (flag); disp(['libCZon_ScaleDualize3 failed in ',FuncID]); return; end;
end


[Zloop{i+1}.Z,~,flag] = libCZon_ReduceDOFOrder(Zloop{i}.Z,options.irdofo,options.gmeth);
if (flag); disp(['libCZon_ReduceDOFOrder failed in ',FuncID]); return; end;


for j=i+1:-1:1
    options1.color=options.color(j);
    libCZon_Plot(Zloop{j}.Z,options1);
    hold on
end
