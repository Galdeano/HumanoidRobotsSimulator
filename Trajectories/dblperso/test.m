clear all 
close all

t=3:0.01:9;
pp=2;
vp=0;
ap=0;
pn=1;
vn=-2;
an=5;


[ S ] = spline_pva( t,pp,vp,ap,pn,vn,an )

plot(t,S)

figure(1);
plot(t,S);
axis equal

figure(2);
plot(t(1:length(t)-1),diff(S)/0.01);


figure(3);
plot(t(1:length(t)-2),diff(diff(S))/(0.01*0.01));
