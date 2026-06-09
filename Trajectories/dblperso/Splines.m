
clear all
close all

%Cubic Hermite splines
t=0:0.01:1;
%t2=1:0.01:2;

%Hermite basis functions
h1 = 2*t.^3 - 3*t.^2 + 1;
h2 = -2*t.^3 +3*t.^2;
h3 = t.^3 - 2*t.^2 + t;
h4 = t.^3 -t.^2;

figure(1);
plot(t,h1,t,h2,t,h3,t,h4);
axis equal


%positions et vitesses désirées
p1=0;
v1=1;
p2=0.5;
v2=1;

S1=v1*h3+p1*h1+p2*h2+v2*h4;

figure(2);
plot(t,S1);
axis equal

%%

%Bornes 
x0=0.5;
x1=2.5;

x=x0:0.01:x1;
t= (x-x0)/(x1-x0);

h1 = 2*t.^3 - 3*t.^2 + 1;
h2 = -2*t.^3 +3*t.^2;
h3 = t.^3 - 2*t.^2 + t;
h4 = t.^3 -t.^2;

%positions et vitesses désirées
p1=0;
v1=-1;
p2=1;
v2=-0.5;

S2=(x1-x0)*v1*h3+p1*h1+p2*h2+(x1-x0)*v2*h4;

figure(2);
plot(x,S2);
axis equal

figure(3);
plot(x(1:(length(x)-1)),diff(S2)/0.01);
axis equal

figure(4);
plot(x(1:(length(x)-2)),diff(diff(S2))/(0.01*0.01));
axis equal

%verif
ape = csape([0.5 2.5],[-1 0 1 -0.5],[1 1]);
values = fnval(ape,x)
figure(5);
plot(x,values);
axis equal

% p1=1;
% v1=-1;
% p2=1;
% v2=0;
% 
% S2=v1*h3+p1*h1+p2*h2+v2*h4;
% 
% figure(3);
% plot([t,t2],[S1,S2]);
% axis equal

%%

%Bornes 
x0=0.5;
x1=2.5;
x2=3.5;

xf=x0:0.01:x1;
tf= (xf-x0)/(x1-x0);
xs=x1:0.01:x2;
ts= (xs-x1)/(x2-x1);

h11 = 2*tf.^3 - 3*tf.^2 + 1;
h12 = -2*tf.^3 +3*tf.^2;
h13 = tf.^3 - 2*tf.^2 + tf;
h14 = tf.^3 -tf.^2;

h21 = 2*ts.^3 - 3*ts.^2 + 1;
h22 = -2*ts.^3 +3*ts.^2;
h23 = ts.^3 - 2*ts.^2 + ts;
h24 = ts.^3 -ts.^2;

%positions et vitesses désirées
p1=0;
v1=-1;
p2=0.7;
p3=1;
v3=-0.5;


%v2=(p3-p2)/(2*(x2-x1))+(p2-p1)/(2*(x1-x0));

v2=((-6*p1-2*(x1-x0)*v1+6*p2)/((x1-x0)*(x1-x0))+(-6*p2+6*p3-2*(x2-x1)*v3)/((x2-x1)*(x2-x1)))/(4/(x1-x0)+4/(x2-x1));


S1=(x1-x0)*v1*h13+p1*h11+p2*h12+(x1-x0)*v2*h14;

S2=(x2-x1)*v2*h23+p2*h21+p3*h22+(x2-x1)*v3*h24;

xp=[xf(1:(length(xf)-1)) xs];
Sp=[S1(1:(length(S1)-1)) S2];

figure(2);
plot(xp,Sp);
axis equal

figure(3);
plot(xp(1:(length(xp)-1)),diff(Sp)/0.01);
axis equal

figure(4);
plot(xp(1:(length(xp)-2)),diff(diff(Sp))/(0.01*0.01));
axis equal


test= 6*tf*(2*p1+(x1-x0)*v1-2*p2+(x1-x0)*v2)/((x1-x0)*(x1-x0))+2*(-3*p1-2*(x1-x0)*v1+3*p2-(x1-x0)*v2)/((x1-x0)*(x1-x0));

test2= 6*ts*(2*p2+(x2-x1)*v2-2*p3+(x2-x1)*v3)/((x2-x1)*(x2-x1))+2*(-3*p2-2*(x2-x1)*v2+3*p3-(x2-x1)*v3)/((x2-x1)*(x2-x1));


v2=(((-6*(2*p1+(x1-x0)*v1-2*p2)-2*(-3*p1-2*(x1-x0)*v1+3*p2))/((x1-x0)*(x1-x0)))+2*(-3*p2+3*p3-(x2-x1)*v3)/((x2-x1)*(x2-x1)))/(4/(x1-x0)+4/(x2-x1));



figure(5);
plot(xp,[test(1:(length(xf)-1)) test2]);
axis equal



%%
%verif
x=0.5:0.01:3.5;
ape = csape([0.5 2.5 3.5],[-1 0 0.7 1 -0.5],[1 1]);
values = fnval(ape,x)
figure(5);
plot(x,values);
axis equal

% p1=1;
% v1=-1;
% p2=1;
% v2=0;
% 
% S2=v1*h3+p1*h1+p2*h2+v2*h4;
% 
% figure(3);
% plot([t,t2],[S1,S2]);
% axis equal


%% Splines du 6 eme ordre

clear all 
close all

A=[[0,0,0,0,0,1];[0,0,0,0,1,0];[0,0,0,2,0,0];[1,1,1,1,1,1];[5,4,3,2,1,0];[20,12,6,2,0,0]];

r=[[1,0,0,0,0,0];[0,1,0,0,0,0];[0,0,1,0,0,0];[0,0,0,1,0,0];[0,0,0,0,1,0];[0,0,0,0,0,1]];

coeff=inv(A)*r;
t=0:0.01:1;

coeff(6,1);

for i=1:6
    h(i,:)=coeff(1,i)*t(:).*t(:).*t(:).*t(:).*t(:)+coeff(2,i)*t(:).*t(:).*t(:).*t(:)+coeff(3,i)*t(:).*t(:).*t(:)+coeff(4,i)*t(:).*t(:)+coeff(5,i)*t(:)+coeff(6,i);
end

figure(6);
hold on
for i=1:6
plot(t,h(i,:));
end
axis equal
hold off

%%
p1=0;
v1=-1;
a1=0.5;
p2=1;
v2=-2;
a2=-1;

S1(1,:)=p1*h(1,:)+v1*h(2,:)+a1*h(3,:)+p2*h(4,:)+v2*h(5,:)+a2*h(6,:);

figure(7);
plot(t,S1);
axis equal


figure(8);
plot(t(1:length(t)-1),diff(S1)/0.01);



figure(9);
plot(t(1:length(t)-2),diff(diff(S1))/(0.01*0.01));


