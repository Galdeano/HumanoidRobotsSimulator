
clear all;
close all;

%%
%com y=-0.04
%com z= 0.24
%f2f y=-0.08

t0=0.0;
t1=2.0;
t2=4.0;
t3=6.0;
t4=10.0;
t5=12.0;
t6=14.0;
t7=18.0;
t8=20.0;
t9=22.0;

t10=24.0;
t11=26.0;


t=t0:0.005:t9;
t_v= [   t0    t1    t2    t3    t4    t5    t6    t7    t8    t9];

comX=[ 0.02  0.02  0.02  0.02  0.12  0.12  0.12  0.22  0.22  0.22]
comY=[-0.04 -0.08 -0.08 -0.08  0.00  0.00  0.00 -0.08 -0.08 -0.08]
comZ=[ 0.24  0.24  0.24  0.24  0.24  0.24  0.24  0.24  0.24  0.24]
pp=csape(t_v,[0 comX 0],[1 1])
Com_x0=fnval(pp,t);

pp=csape(t_v,[0 comY 0],[1 1])
Com_y0=fnval(pp,t);

pp=csape(t_v,[0 comZ 0],[1 1])
Com_z0=fnval(pp,t);


%

rfX= [ 0.00  0.00  0.05  0.10  0.10  0.10  0.10  0.10  0.20  0.30]
rfY= [ 0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00  0.00]
rfZ= [ 0.00  0.00  0.04  0.00  0.00  0.00  0.00  0.00  0.04  0.00]
pp=csape(t_v,[0 rfX 0],[1 1])
rf_x0=fnval(pp,t);

pp=csape(t_v,[0 rfY 0],[1 1])
rf_y0=fnval(pp,t);

pp=csape(t_v,[0 rfZ 0],[1 1])
rf_z0=fnval(pp,t);

%

lfX= [ 0.00  0.00  0.00  0.00  0.00  0.10  0.20  0.20  0.20  0.20]
lfY= [-0.08 -0.08 -0.08 -0.08 -0.08 -0.08 -0.08 -0.08 -0.08 -0.08]
lfZ= [ 0.00  0.00  0.00  0.00  0.00  0.04  0.00  0.00  0.00  0.00]

pp=csape(t_v,[0 lfX 0],[1 1])
lf_x0=fnval(pp,t);

pp=csape(t_v,[0 lfY 0],[1 1])
lf_y0=fnval(pp,t);

pp=csape(t_v,[0 lfZ 0],[1 1])
lf_z0=fnval(pp,t);

hold on
plot3(Com_x0,Com_y0,Com_z0)
plot3(rf_x0,rf_y0,rf_z0)
plot3(lf_x0,lf_y0,lf_z0)

fid = fopen('./Human_op3.txt', 'w');
for i=1:size(t,2)
fprintf(fid, '%2.10f,%2.10f,%2.10f,%2.10f,%2.10f,%2.10f,%2.10f,%2.10f,%2.10f\n', Com_x0(1,i),Com_y0(1,i),Com_z0(1,i),lf_x0(1,i),lf_y0(1,i),lf_z0(1,i),rf_x0(1,i),rf_y0(1,i),rf_z0(1,i));
end
fclose(fid)

