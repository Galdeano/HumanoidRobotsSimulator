clc,
clear all,
close all, 
%% Dans ce code on a utilisé un algorithme developpé par SHUUJI KAJITA 
%% declaration des constantes
global z g
%z=0.8;
g=9.8;%gravité
hfeet=0.04;
z=0.24+hfeet;%la hauteur du centre de masse restant constante
pas=0.001;
%% paramètres de marche
%% 
%%avec largeur du pas constante et aucune orientation

steps=12;
xsize=0.03;
sx=[0];
sy=[0.082];
st=[0];
for i=1:steps
sx=[sx xsize];
sy=[sy 0.082];
st=[st 0.*pi/180];
end
sx=[sx 0 0];
sy=[sy 0.082 0.082];
st=[st 0.*pi/180 0.*pi/180];




% sx=[0 0.03 0.03 0.03 0.03 0.03 0.03 0.03 0 0];
% sy=[0.082 0.082 0.082 0.082 0.082 0.082 0.082 0.082 0.082 0.082];
% st=[0 0 0 0 0 0 0 0 0 0].*pi/180;

%marche diagonale 
% sx=[0 0.2 0.2 0.2 0 0];
% sy=[0.2 0.3 0.1 0.3 0.2 0.2];
% st=[0 0 0 0 0 0].*pi/180;

%marche selon un arc de cercle
% sx=[0 0.25 0.25 0.25 0 0];
% sy=[0.2 0.2 0.2 0.2 0.2 0.2];
% st=[0 10 20 30 30 30].*pi/180;

%% parametres de la marche pour six pas successives 
%%
% sx=[0 0.2 0.2 0.2 0 0];%selon l'axe x
% sy=[0.1595 0.1595 0.1595 0.1595 0.1595 0.1595];%selon l'axe y
% st=[0 0 0 0 0 0].*pi/180;%angle de rotation 

%Tsup=1.64;%temps de support
%Tsup=1.64/2;%temps de support
Tsup=1.50/2;%temps de support
%Start_y=-0.03;

%Tsup=0.8;
%Start_y=-0.03;
%Tdbl=0.1;
a=10;
b=1;

Tc=sqrt(z/g);%constante de temps 
C=cosh(Tsup/Tc);
S=sinh(Tsup/Tc);

Azimut =45;
Elevation =30;

%%

Tpas=int32(Tsup/pas);
inter=80;%pour 1ms
%inter=10;%pour 5ms
Tinter=inter*pas;

%% ETAPE 2:calcul de la position du pied: ici le pied droit
%% 

%% remarque:%si on choisit le pied gauche comme point de départ de la marche
%% on remplace -(-1)^n par +(-1)^n

%% position du pied droit
Px(1)=0;%position initiale du pied en x
Py(1)=-0.082/2;%position initiale du pied en y

%% calcul de la position du pied pour n pas
%% 
for n=2:(length(sx)+1)
    Px(n)=Px(n-1)+ cos(st(n-1))*sx(n-1) -sin(st(n-1))*((-1)^n)*sy(n-1);
    Py(n)=Py(n-1)+ sin(st(n-1))*sx(n-1) +cos(st(n-1))*((-1)^n)*sy(n-1);
    
    %plot(Px,Py),title('position du pied droit'),grid on;  
end

% plot(Px,Py),
% %title('position du pied droit'),
% grid on;  

%% position du pied gauche
% Pxg(1)=0;%position initiale du pied en x
% Pyg(1)=0.078/2;%position initiale du pied en y

% for n=2:(length(sx)+1)
%     Pxg(n)=Pxg(n-1)+ cos(st(n-1))*sx(n-1) +sin(st(n-1))*((-1)^n)*sy(n-1);
%     Pyg(n)=Pyg(n-1)+ sin(st(n-1))*sx(n-1) -cos(st(n-1))*((-1)^n)*sy(n-1);
%     
%     %plot(Px,Py),title('position du pied droit'),grid on;  
% end
% hold on
% plot(Pxg,Pyg,'r'),title('position des deux pieds: le pied droit en bleu et le pied gauche en rouge'),grid on;  


%% Etape6:les caracteristiques de la primitive de marche suivante 
%% position finale 
for n=1:(length(sx))
    xbar(n)=cos(st(n))*sx(n)/2 -sin(st(n))*(-(-1)^n)*sy(n)/2;
    ybar(n)=sin(st(n))*sx(n)/2 +cos(st(n))*(-(-1)^n)*sy(n)/2;
end

%% vitesse finale
for n=1:(length(sx))
    vxbar(n)=cos(st(n))*((C+1)/(Tc*S))*xbar(n) -sin(st(n))*((C-1)/(Tc*S))*ybar(n);
    vybar(n)=sin(st(n))*((C+1)/(Tc*S))*xbar(n) +cos(st(n))*((C-1)/(Tc*S))*ybar(n);
end

%% Etape7:calcul de l'etat cible (désiré)
%% 
for n=1:(length(sx))
    xd(n)=Px(n)+xbar(n);
    vxd(n)=vxbar(n);
    yd(n)=Py(n)+ybar(n);
    vyd(n)=vybar(n);
end

%% Etape8:calcul de la position modifiée du pied
%% 
D=a*(C-1)^2+b*(S/Tc)^2;%constante permettant de calculer la position modifiée du pied

%% com
global Pxm Pym %position modifiée du pied
T=0;%temps 
n=0;%numéro du pas
%% initialisation de la position modifiée du pied: identique à celle non
%% modifiée 
Pxm=0;
Pym=-0.082/2;
%Pym=-0.017;
%Pym=-Pxm;
Pxmem=[Pxm Pxm];
Pymem=[-Pym Pym];
%% initialisation de la position initiale du centre de masse
%% 
xcom=0;
%ycom=-(0.082/2)*0.999944; %pour t=1.64
%ycom=-(0.082/2)*0.9895; %pour t=1.64/2
%ycom=-(0.082/2)*0.9835; %pour t=1.50/2

ylauch=-(0.082/2)*0.9766;
ycom=ylauch;

%% initialisation de la vitesse initiale du centre de masse
%% 
vxcom=0;
vycom=0;

txmem=0;
xmem=[xcom vxcom];%propriétés du centre de masse en x
tymem=0;
ymem=[ycom vycom];%propriétés du centre de masse en y

% figure
% hold on,grid on,

fld=[Pxm;-Pym;0+hfeet];
frd=[Pxm;Pym;0+hfeet];

k=1;
%% ETAPE 3
%% integrer le modele dynamique décrivant  le centre de masse en x et en y
%%
for i=1:(length(sx)-1)

    [tx,x] = ode45('PenduleODEx',[T:pas:(T+Tsup)],[xcom; vxcom]); 
    [ty,y] = ode45('PenduleODEy',[T:pas:(T+Tsup)],[ycom; vycom]); 
    
    %% affecter les nouvelles propriétés (position et vitesse) calculées au
    %% centre de masse aprés integration sur une periode de support Tsup
    xcom=x(end,1);
    vxcom=x(end,2);
    ycom=y(end,1);
    vycom=y(end,2);
    
    T=T+Tsup;%temps de passage au pas suivant 
    n=n+1;%passage au pas suivant
    txmem=[txmem;tx];%memorisation de la periode d'integrantion et celle la precedant suivant l'axe x
    xmem=[xmem;x];%memorisation de la position du COM respectivement avant et apres integration suivant l'axe x
    tymem=[tymem;ty];%memorisation de la periode d'integrantion et celle la precedant suivant l'axe y
    ymem=[ymem;y];%memorisation de la position du COM respectivement avant et apres integration suivant l'axe y
    
    %% primitives de marche en modele LIP-3D elementaire
%  for j=1:(length(x)/10)
%         line([x(j*10,1) Pxm],[y(j*10,1) Pym],[z 0])   %%
% 
%     end
    %% calcul de la position modifiée du pied
    %%
    Pxms=-a*((C-1)/D)*(xd(n+1)-C*x(end,1)-Tc*S*x(end,2))-b*(S/(Tc*D))*(vxd(n+1)-(S/Tc)*x(end,1)-C*x(end,2));
    Pyms=-a*((C-1)/D)*(yd(n+1)-C*y(end,1)-Tc*S*y(end,2))-b*(S/(Tc*D))*(vyd(n+1)-(S/Tc)*y(end,1)-C*y(end,2));  
    %% affecter les nouvelles valeurs de la position modifiée du pied
    %%
    Pxm=Pxms;
    Pym=Pyms;
    
    Pxmem=[Pxmem Pxm];%position calculée et elle modifiée du pien en x
    Pymem=[Pymem Pym];%position calculée et elle modifiée du pien en y
   
    %% Inerpolation polynomiale des differentes positions du pied
    %% la fonction csape permet de préciser de nombreux types de conditions
    %% extrêmes, y compris les conditions de fin périodiques%%
    if (k==1)
        x_sp=csape([T-Tsup+Tinter T-Tinter],[0 Pxmem(n) Pxmem(n+2) 0],[1,1]);
        y_sp=csape([T-Tsup+Tinter T-Tinter],[0 Pymem(n) Pymem(n+2) 0],[1,1]);
        z_sp=csape([T-Tsup+Tinter T-Tsup/2 T-Tinter],[0 0 0.02 0 0],[2,2]);
        frd=[frd,[Pxmem(n+1)+0*tx';Pymem(n+1)+0*tx';0*tx'+hfeet]];
        fld=[fld,...
            [[Pxmem(n)+0*(T-Tsup:pas:T-Tsup+Tinter-pas) fnval(x_sp,[T-Tsup+Tinter:pas:T-Tinter]) Pxmem(n+2)+0*(T-Tsup+pas:pas:T-Tsup+Tinter)];...
            [Pymem(n)+0*(T-Tsup:pas:T-Tsup+Tinter-pas) fnval(y_sp,[T-Tsup+Tinter:pas:T-Tinter]) Pymem(n+2)+0*(T-Tsup+pas:pas:T-Tsup+Tinter)];...
            [0*(T-Tsup:pas:T-Tsup+Tinter-pas) fnval(z_sp,[T-Tsup+Tinter:pas:T-Tinter]) 0*(T-Tsup+pas:pas:T-Tsup+Tinter)]+hfeet]];
        k=2;
    else
        x_sp=csape([T-Tsup+Tinter T-Tinter],[0 Pxmem(n) Pxmem(n+2) 0],[1,1]);
        y_sp=csape([T-Tsup+Tinter T-Tinter],[0 Pymem(n) Pymem(n+2) 0],[1,1]);
        z_sp=csape([T-Tsup+Tinter T-Tsup/2 T-Tinter],[0 0 0.02 0 0],[2,2]);
        %frd=[frd,[fnval(x_sp,tx)';fnval(y_sp,ty)';fnval(z_sp,tx)']];
        frd=[frd,...
            [[Pxmem(n)+0*(T-Tsup:pas:T-Tsup+Tinter-pas) fnval(x_sp,[T-Tsup+Tinter:pas:T-Tinter]) Pxmem(n+2)+0*(T-Tsup+pas:pas:T-Tsup+Tinter)];...
            [Pymem(n)+0*(T-Tsup:pas:T-Tsup+Tinter-pas) fnval(y_sp,[T-Tsup+Tinter:pas:T-Tinter]) Pymem(n+2)+0*(T-Tsup+pas:pas:T-Tsup+Tinter)];...
            [0*(T-Tsup:pas:T-Tsup+Tinter-pas) fnval(z_sp,[T-Tsup+Tinter:pas:T-Tinter]) 0*(T-Tsup+pas:pas:T-Tsup+Tinter)]+hfeet]];
        fld=[fld,[Pxmem(n+1)+0*tx';Pymem(n+1)+0*tx';0*tx'+hfeet]];
        k=1;
    end
    
%     if (k==1)
%         x_sp=csape([T-Tsup T-Tsup+Tinter T-Tinter T],[0 Pxmem(n) Pxmem(n) Pxmem(n+2) Pxmem(n+2) 0],[1,1]);
%         y_sp=csape([T-Tsup T-Tsup+Tinter T-Tinter T],[0 Pymem(n) Pymem(n) Pymem(n+2) Pymem(n+2) 0],[1,1]);
%         z_sp=csape([T-Tsup T-Tsup+Tinter T-Tsup/2 T-Tinter T],[0 0 0 0.02 0 0 0],[2,2]);
%         frd=[frd,[Pxmem(n+1)+0*tx';Pymem(n+1)+0*tx';0*tx']];
%         fld=[fld,[fnval(x_sp,tx)';fnval(y_sp,ty)';fnval(z_sp,tx)']];
%         k=2;
%     else
%         x_sp=csape([T-Tsup T-Tsup+Tinter T-Tinter T],[0 Pxmem(n) Pxmem(n) Pxmem(n+2) Pxmem(n+2) 0],[1,1]);
%         y_sp=csape([T-Tsup T-Tsup+Tinter T-Tinter T],[0 Pymem(n) Pymem(n) Pymem(n+2) Pymem(n+2) 0],[1,1]);
%         z_sp=csape([T-Tsup T-Tsup+Tinter T-Tsup/2 T-Tinter T],[0 0 0 0.02 0 0 0],[2,2]);
%         frd=[frd,[fnval(x_sp,tx)';fnval(y_sp,ty)';fnval(z_sp,tx)']];
%         fld=[fld,[Pxmem(n+1)+0*tx';Pymem(n+1)+0*tx';0*tx']];
%         k=1;
%     end

%     if (k==1)
%         x_sp=csape([T-Tsup T],[0 Pxmem(n) Pxmem(n+2) 0],[1,1]);
%         y_sp=csape([T-Tsup T],[0 Pymem(n) Pymem(n+2) 0],[1,1]);
%         z_sp=csape([T-Tsup T-Tsup/2 T],[0 0 0.02 0 0],[2,2]);
%         frd=[frd,[Pxmem(n+1)+0*tx';Pymem(n+1)+0*tx';0*tx']];
%         fld=[fld,[fnval(x_sp,tx)';fnval(y_sp,ty)';fnval(z_sp,tx)']];
%         k=2;
%     else
%         x_sp=csape([T-Tsup T],[0 Pxmem(n) Pxmem(n+2) 0],[1,1]);
%         y_sp=csape([T-Tsup T],[0 Pymem(n) Pymem(n+2) 0],[1,1]);
%         z_sp=csape([T-Tsup T-Tsup/2 T],[0 0 0.02 0 0],[2,2]);
%         frd=[frd,[fnval(x_sp,tx)';fnval(y_sp,ty)';fnval(z_sp,tx)']];
%         fld=[fld,[Pxmem(n+1)+0*tx';Pymem(n+1)+0*tx';0*tx']];
%         k=1;
%     end
end

%% trajectoires 3D des deux pieds
%%
% grid on,title('Trajectoires 3D des deux pieds');
% view([Azimut,Elevation])
% %% Dessiner les trajectoires polynomiales entre les differentes positions
% %% (interpolation)
% % des pieds
% plot3(fld(1,:),fld(2,:),fld(3,:));
% plot3(frd(1,:),frd(2,:),frd(3,:));
% 
% figure
% plot(txmem,xmem(:,1),tymem,ymem(:,1)),title('Trajectoire du centre de masse'),grid on;
% figure
% plot(txmem,xmem(:,2),tymem,ymem(:,2)),title('Trajectoire vitesse du centre de masse'),grid on;
% figure
% hold on
% 
% plot(xmem(:,1),ymem(:,1));
% plot(Px,Py,'+');
% plot(Pxmem,Pymem,'*');
% plot(xd,yd,'r+');

%% dble



for i=1:(length(sx)-2)

xp=xmem(Tpas-inter+(i-1)*Tpas,1);%selection d'un point en x precedent le milieu 
yp=ymem(Tpas-inter+(i-1)*Tpas,1);%selection d'un point en y precedent le milieu 

xi=xmem(Tpas+(i-1)*Tpas,1);
yi=ymem(Tpas+(i-1)*Tpas,1);

xn=xmem(Tpas+inter+(i-1)*Tpas,1);%selection d'un point en x suivant le milieu 
yn=ymem(Tpas+inter+(i-1)*Tpas,1);%selection d'un point en y suivant le milieu 


vx=diff(xmem)/pas;
vy=diff(ymem)/pas;

vxp=(vx(Tpas-inter-1+(i-1)*Tpas,1)+vx(Tpas-inter+(i-1)*Tpas,1))/2;
vyp=(vy(Tpas-inter-1+(i-1)*Tpas,1)+vy(Tpas-inter+(i-1)*Tpas,1))/2;
vxn=(vx(Tpas+inter-1+(i-1)*Tpas,1)+vx(Tpas+inter+(i-1)*Tpas,1))/2;
vyn=(vy(Tpas+inter-1+(i-1)*Tpas,1)+vy(Tpas+inter+(i-1)*Tpas,1))/2;


ax=diff(diff(xmem))/(pas^2);
ay=diff(diff(ymem))/(pas^2);

axp=ax(Tpas-inter-1+(i-1)*Tpas,1);
ayp=ay(Tpas-inter-1+(i-1)*Tpas,1);
axn=ax(Tpas+inter+1+(i-1)*Tpas,1);
ayn=ay(Tpas+inter+1+(i-1)*Tpas,1);



% xsp=csape([txmem(Tpas-inter+(i-1)*Tpas,1),txmem(Tpas+inter+1+(i-1)*Tpas,1)],[axp,xp,xn,axn],[2,2])%% interpolation of curve csape 
% ysp=csape([tymem(Tpas-inter+(i-1)*Tpas,1),tymem(Tpas+inter+1+(i-1)*Tpas,1)],[ayp,yp,yn,ayn],[2,2])%% interpolation of curve csape 
% 
% xst=fnval(xsp,txmem(Tpas-inter+0+(i-1)*Tpas,1):pas:txmem(Tpas+inter+1+(i-1)*Tpas,1));
% yst=fnval(ysp,tymem(Tpas-inter+0+(i-1)*Tpas,1):pas:tymem(Tpas+inter+1+(i-1)*Tpas,1));

% figure
% plot(xsp)
%%% 


[ xst ] = spline_pva(txmem(Tpas-inter+(i-1)*Tpas,1):pas:txmem(Tpas+inter+1+(i-1)*Tpas,1),xp,vxp,axp,xn,vxn,axn );
[ yst ] = spline_pva(tymem(Tpas-inter+(i-1)*Tpas,1):pas:tymem(Tpas+inter+1+(i-1)*Tpas,1),yp,vyp,ayp,yn,vyn,ayn );


xmem2=[xmem(1:Tpas-inter-1+(i-1)*Tpas,1);xst';xmem(Tpas+inter+1+(i-1)*Tpas:end,1)];
xmem3=[xmem2(:,1) xmem(:,2)];
xmem=xmem3;

ymem2=[ymem(1:Tpas-inter-1+(i-1)*Tpas,1);yst';ymem(Tpas+inter+1+(i-1)*Tpas:end,1)];
ymem3=[ymem2(:,1) ymem(:,2)];
ymem=ymem3;
% 
end

%% zmp
zmem=0*xmem(:,1)+z;
px=xmem(:,1)-(z/g)*[0; diff(diff(xmem(:,1)))/(pas^2) ; 0] ;
py=ymem(:,1)-(z/g)*[0; diff(diff(ymem(:,1)))/(pas^2) ; 0] ;

figure
hold on
plot(xmem(:,1)')
plot(px,'r')

figure
hold on
plot(ymem(:,1)')
plot(py,'r')



% figure
% hold on
% plot(diff(xmem(:,1))')
% plot(px(2,end),'r')
% 
% figure
% hold on
% plot(diff(ymem(:,1))')
% plot(py(2,end),'r')



%% starter
tstart=1;


%Start_y=-(0.082/2)*0.999944;%pour t=1.64
%Start_y=-(0.082/2)*0.9895;%pour t=1.64/2
%Start_y=-(0.082/2)*0.9835; 

Start_y=ylauch; 

xsp=csape([0 tstart],[0 0.0 0 0],[1,1]);
xst=fnval(xsp,0:pas:tstart);

ysp=csape([0 tstart],[0 0 Start_y 0],[1,1]);
yst=fnval(ysp,0:pas:tstart);



xmem=[[0*xst',0*xst'];xmem];
ymem=[[yst',0*yst'];ymem];
zmem=[z+0*yst';z+0*txmem];


fld=[[0*yst',0.082/2+0*yst',0*yst'+hfeet]',fld];
frd=[[0*yst',-0.082/2+0*yst',0*yst'+hfeet]',frd];

%% ender
% tend=1;
% 
% te=0:pas:tend;
% 
% indend=size(xmem,1);
% 
% xe=xmem(indend,1);
% ye=ymem(indend,1);
% 
% vx=diff(xmem)/pas;
% vy=diff(ymem)/pas;
% 
% vxe=vx(indend-1,1);
% vye=vy(indend-1,1);
% 
% ax=diff(diff(xmem))/(pas^2);
% ay=diff(diff(ymem))/(pas^2);
% 
% axe=ax(indend-2,1);
% aye=ay(indend-2,1);
% 
% 
% 
% 
% [ xste ] = spline_pva(te,xe,vxe,axe,xe,0.0,0.0 );
% [ yste ] = spline_pva(te,ye,vye,aye,ye,0.0,0.0 );
% 
% xmem=[xmem;[xste',0*xste']];
% ymem=[ymem;[yste',0*yste']];
% zmem=[zmem;z+0*te'];
% 
% 
% fld=[fld,[fld(1,end)+0*te',fld(2,end)+0*te',fld(3,end)+0*te']'];
% frd=[frd,[frd(1,end)+0*te',frd(2,end)+0*te',frd(3,end)+0*te']'];
% 


%% visu

figure
hold on
Azimut =45;
Elevation =30;
grid on

view([Azimut,Elevation]);
plot3(xmem(:,1),ymem(:,1),zmem,'r');
plot3(fld(1,:),fld(2,:),fld(3,:),'g');
plot3(frd(1,:),frd(2,:),frd(3,:),'k');

for k=1:(length(zmem)/35)
    j=35*k;
    line([xmem(j,1) fld(1,j)],[ymem(j,1) fld(2,j)],[zmem(j,1) fld(3,j)]);
    line([xmem(j,1) frd(1,j)],[ymem(j,1) frd(2,j)],[zmem(j,1) frd(3,j)]);
end

% figure
% hold on
% plot(xmem(:,1)')
% 
% 
% figure
% hold on
% plot(ymem(:,1)')


%% zmp
zmem=0*xmem(:,1)+z;
px=xmem(:,1)-(z/g)*[0; diff(diff(xmem(:,1)))/(pas^2) ; 0] ;
py=ymem(:,1)-(z/g)*[0; diff(diff(ymem(:,1)))/(pas^2) ; 0] ;

figure
hold on
plot(xmem(:,1)')
plot(px,'r')

figure
hold on
plot(ymem(:,1)')
plot(py,'r')


%%

traj=[xmem(:,1)';ymem(:,1)';zmem';fld(:,:);frd(:,:)];
save('traj.mat','traj')


tcoms=[xmem(:,1)+0.014,ymem(:,1),zmem(:,1)]-fld(:,:)';
 
tf2fs=(frd(:,:)-fld(:,:))';

oris=0*tf2fs;

fid = fopen('../Hoap_Walk4.txt', 'w');
for i=1:(size(xmem,1)-1)
fprintf(fid, '%2.10f,%2.10f,%2.10f,%2.10f,%2.10f,%2.10f,%2.10f,%2.10f,%2.10f\n',tcoms(i,1),tcoms(i,2),tcoms(i,3),...
    tf2fs(i,1),tf2fs(i,2),tf2fs(i,3),...
    oris(i,1),oris(i,2),oris(i,3));
end
fclose(fid)


