clc
clear all 
close all


%% Params generaux
global Zc g
Zc=0.8;
g=9.81;


%% paramètres de marche

deg2rad=pi/180;
rad2deg=180/pi;

% Sx=[0 0.3 0.3 0.3 0 0];
% Sy=[0.2 0.2 0.2 0.2 0.2 0.2];
% St=[0 0 0 0 0 0].*deg2rad;

% Sx=[0 0.2 0.2 0.2 0 0];
% Sy=[0.2 0.3 0.1 0.3 0.2 0.2];
%St=[0 0 0 0 0 0].*deg2rad;

% Sx=[0 0.25 0.25 0.25 0 0];
% Sy=[0.2 0.2 0.2 0.2 0.2 0.2];
% St=[0 10 20 30 30 30].*deg2rad;

Zc=0.884101;
Sx=[0 0.2 0.2 0.2 0 0];
Sy=[0.1595 0.1595 0.1595 0.1595 0.1595 0.1595];
St=[0 0 0 0 0 0].*deg2rad;

Tsup=2.0;
Start_y=-0.079750*0.9975;

% Tsup=0.8;
% Start_y=-0.079750*0.9;
%Tdbl=0.1;
a=10;
b=1;

Tc=sqrt(Zc/g);
C=cosh(Tsup/Tc);
S=sinh(Tsup/Tc);

length(Sx)

%% positionnement de pied

Px(1)=0;
Py(1)=-0.079750;


% for n=2:(length(Sx)+1)
%     Px(n)=Px(n-1)+Sx(n-1);
%     Py(n)=Py(n-1)+((-1)^n)*Sy(n-1);
% end

for n=2:(length(Sx)+1)
    Px(n)=Px(n-1)+ cos(St(n-1))*Sx(n-1) -sin(St(n-1))*((-1)^n)*Sy(n-1);
    Py(n)=Py(n-1)+ sin(St(n-1))*Sx(n-1) +cos(St(n-1))*((-1)^n)*Sy(n-1);
end

Px
Py

plot(Px,Py)


%% primitive de marche

% for n=1:(length(Sx))
%     x_bar(n)=Sx(n)/2;
%     y_bar(n)=-((-1)^n)*Sy(n)/2;
% end

for n=1:(length(Sx))
    x_bar(n)=cos(St(n))*Sx(n)/2 -sin(St(n))*(-(-1)^n)*Sy(n)/2;
    y_bar(n)=sin(St(n))*Sx(n)/2 +cos(St(n))*(-(-1)^n)*Sy(n)/2;
end

x_bar
y_bar

%% vitesse finale

% for n=1:(length(Sx))
%     vx_bar(n)=((C+1)/(Tc*S))*x_bar(n);
%     vy_bar(n)=((C-1)/(Tc*S))*y_bar(n);
% end

for n=1:(length(Sx))
    vx_bar(n)=cos(St(n))*((C+1)/(Tc*S))*x_bar(n) -sin(St(n))*((C-1)/(Tc*S))*y_bar(n);
    vy_bar(n)=sin(St(n))*((C+1)/(Tc*S))*x_bar(n) +cos(St(n))*((C-1)/(Tc*S))*y_bar(n);
end

vx_bar
vy_bar

%% état cible

for n=1:(length(Sx))
    x_d(n)=Px(n)+x_bar(n);
    dx_d(n)=vx_bar(n);
    y_d(n)=Py(n)+y_bar(n);
    dy_d(n)=vy_bar(n);
end

x_d
dx_d
y_d
dy_d

%% placement de pied modifié

D=a*(C-1)^2+b*(S/Tc)^2


%% com
global Pxs Pys
T=0
n=0

Pxs=0;
%Pys=-0.1;
Pys=-0.079750;
Px_mem=[Pxs Pxs];
Py_mem=[-Pys Pys];



comx=0;
%comy=-0.09;
comy=Start_y;
vcomx=0;
vcomy=0;

tx_mem=0;
x_mem=[comx vcomx];
ty_mem=0;
y_mem=[comy vcomy];

figure
hold on

fld=[Pxs;-Pys;0];
frd=[Pxs;Pys;0];

k=1;
for i=1:(length(Sx)-1)

    [tx,x] = ode45('PenduleODEx',[T:0.005:(T+Tsup)],[comx; vcomx]); 
    [ty,y] = ode45('PenduleODEy',[T:0.005:(T+Tsup)],[comy; vcomy]); 
    T=T+Tsup;
    n=n+1;
    tx_mem=[tx_mem;tx];
    x_mem=[x_mem;x];
    ty_mem=[ty_mem;ty];
    y_mem=[y_mem;y];
    
    for j=1:(length(x)/10)
        line([x(j*10,1) Pxs],[y(j*10,1) Pys],[Zc 0])
    end
    
    Pxm=-a*((C-1)/D)*(x_d(n+1)-C*x(end,1)-Tc*S*x(end,2))-b*(S/(Tc*D))*(dx_d(n+1)-(S/Tc)*x(end,1)-C*x(end,2));
    Pym=-a*((C-1)/D)*(y_d(n+1)-C*y(end,1)-Tc*S*y(end,2))-b*(S/(Tc*D))*(dy_d(n+1)-(S/Tc)*y(end,1)-C*y(end,2));  
    Pxs=Pxm;
    Pys=Pym;
    
    Px_mem=[Px_mem Pxs];
    Py_mem=[Py_mem Pys];

    comx=x(end,1);
    vcomx=x(end,2);
    comy=y(end,1);
    vcomy=y(end,2);
    
    if (k==1)
        x_sp=csape([T-Tsup T],[0 Px_mem(n) Px_mem(n+2) 0],[1,1]);
        y_sp=csape([T-Tsup T],[0 Py_mem(n) Py_mem(n+2) 0],[1,1]);
        z_sp=csape([T-Tsup T-Tsup/2 T],[0 0 0.05 0 0],[2,2]);
        frd=[frd,[Px_mem(n+1)+0*tx';Py_mem(n+1)+0*tx';0*tx']];
        fld=[fld,[fnval(x_sp,tx)';fnval(y_sp,ty)';fnval(z_sp,tx)']];
        k=2;
    else
        x_sp=csape([T-Tsup T],[0 Px_mem(n) Px_mem(n+2) 0],[1,1]);
        y_sp=csape([T-Tsup T],[0 Py_mem(n) Py_mem(n+2) 0],[1,1]);
        z_sp=csape([T-Tsup T-Tsup/2 T],[0 0 0.05 0 0],[2,2]);
        frd=[frd,[fnval(x_sp,tx)';fnval(y_sp,ty)';fnval(z_sp,tx)']];
        fld=[fld,[Px_mem(n+1)+0*tx';Py_mem(n+1)+0*tx';0*tx']];
        k=1;
    end
    
end


Azimut =45;
Elevation =30;

grid on
set(gcf,'Renderer','OpenGL'); 
view([Azimut,Elevation])
plot3(fld(1,:),fld(2,:),fld(3,:))
plot3(frd(1,:),frd(2,:),frd(3,:))

figure
plot(tx_mem,x_mem(:,1),ty_mem,y_mem(:,1))

figure
hold on

for i=2:length(Px_mem)
    p=patch([-0.05+Px_mem(i),-0.05+Px_mem(i),0.1+Px_mem(i),0.1+Px_mem(i)],[0.05+Py_mem(i),-0.05+Py_mem(i),-0.05+Py_mem(i),0.05+Py_mem(i)],[1,1,1]);
    set(p,'FaceAlpha',0.5);
    try
        rotate(p,[0,0,1],St(i-1)*rad2deg,[Px_mem(i),Py_mem(i),0])
    catch
        
    end
end

for i=1:length(Px)
    p=patch([-0.05+Px(i),-0.05+Px(i),0.1+Px(i),0.1+Px(i)],[0.05+Py(i),-0.05+Py(i),-0.05+Py(i),0.05+Py(i)],[1,1,1]);
    set(p,'FaceAlpha',0.5);
    set(p,'EdgeColor','r');
    set(p,'LineStyle',':');
    try
        rotate(p,[0,0,1],St(i-1)*rad2deg,[Px(i),Py(i),0])
    catch
        
    end
end

plot(x_mem(:,1),y_mem(:,1))
plot(Px,Py,'+')
plot(Px_mem,Py_mem,'*')
plot(x_d,y_d,'g+')
% plot(frd(1,:),frd(2,:))
% plot(fld(1,:),fld(2,:))

axis equal


%% starter
tstart=5;

x_sp=csape([0 tstart],[0 0.048516 0 0],[1,1]);
x_st=fnval(x_sp,0:0.005:tstart);

y_sp=csape([0 tstart],[0 0 Start_y 0],[1,1]);
y_st=fnval(y_sp,0:0.005:tstart);

x_mem=[[0*x_st',0*x_st'];x_mem];
y_mem=[[y_st',0*y_st'];y_mem];
z_mem=[Zc+0*y_st';Zc+0*tx_mem];


fld=[[0*y_st',0.079750+0*y_st',0*y_st']',fld];
frd=[[0*y_st',-0.079750+0*y_st',0*y_st']',frd];

figure
hold on
Azimut =45;
Elevation =30;
grid on
set(gcf,'Renderer','OpenGL'); 
view([Azimut,Elevation]);
plot3(x_mem(:,1),y_mem(:,1),z_mem);
plot3(fld(1,:),fld(2,:),fld(3,:));
plot3(frd(1,:),frd(2,:),frd(3,:));

for k=1:(length(z_mem)/20)
    j=20*k;
    line([x_mem(j,1) fld(1,j)],[y_mem(j,1) fld(2,j)],[z_mem(j,1) fld(3,j)]);
    line([x_mem(j,1) frd(1,j)],[y_mem(j,1) frd(2,j)],[z_mem(j,1) frd(3,j)]);
end


%%

data_LIPM=[[x_mem(:,1)+0.048516 y_mem(:,1) z_mem(:,1)] frd' fld'];
dlmwrite('LIPM_op.txt', data_LIPM,  'precision', '%.10f','newline', 'pc');




