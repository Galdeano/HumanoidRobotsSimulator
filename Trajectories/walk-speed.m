
clear all;
close all;

%% init

importdata('./Hoap_Walk.txt');

ti=0.6;
step=4;

t=0:0.005:(step*2+3)*ti;


taille=size(t,2);


tcoms=zeros(taille,3);
tf2fs=zeros(taille,3);
oris=zeros(taille,3);


ssods=0.55; % single support over double support
ssodsf=ssods; % single support over double support

xcoff=0.014;
ycamp=0.016;
%ycamp=0.046;
%ycamp=0.030;
xfamp=0.03;%0.02
zfamp=0.01;

%% x


comp=0.005;
s=0.0*(0:0.005:((1.5+(1-ssods)/2)*ti-0.005));
s2=fnval(csape([0 ti*ssods],[0 0.0 -xfamp comp],[1 1]), 0:0.005:(ti*ssods-0.005));
wd=-xfamp+0*(0:0.005:(ti*(1-ssods)-0.005));
wu=xfamp+0*(0:0.005:(ti*(1-ssods)-0.005));
d=fnval(csape([0 ti*ssods],[0 xfamp -xfamp comp],[1 1]), 0:0.005:(ti*ssods-0.005));
u=fnval(csape([0 ti*ssods],[0 -xfamp xfamp 0],[1 1]), 0:0.005:(ti*ssods-0.005));
f=fnval(csape([0 ti*ssods],[0 xfamp 0.0 comp],[1 1]), 0:0.005:ti*ssods);
f2=0.0*(0:0.005:((0.5+(1-ssods)/2)*ti-0.005));

% p=[s s2 wd u wu d wd u wu d wd u wu d wd u wu f f2];
% plot(p)
% break
x=[s s2 wd u];
for i=0:(step-2)
    x=[x wu d wd u];
end
x=[x wu f f2];

%tcoms(:,1)=0.005+0.5*x;
%tcoms(:,1)=0.009+0.5*x;
tf2fs(:,1)=x;
%plot(diff(tf2fs(:,1),2))

xco=xfamp*0.5;

dv=2.3*xfamp/ti;

s=0.0*(0:0.005:((1.5+(1-ssodsf)/2)*ti-0.005));
s2=fnval(csape([0 ti*ssodsf],[0.0 0.0 -xco dv],[1 1]), 0:0.005:(ti*ssodsf-0.005));
d=fnval(csape([0 ti*ssodsf],[dv xco -xco dv],[1 1]), 0:0.005:(ti*ssodsf-0.005));
u2=fnval(csape([0 (ti*(1-ssodsf))],[dv -xco xco dv],[1 1]), 0:0.005:(ti*(1-ssodsf)-0.005));
f=fnval(csape([0 ti*ssodsf],[dv xco 0.0 0],[1 1]), 0:0.005:ti*ssodsf);
f2=0.0*(0:0.005:((0.5+(1-ssodsf)/2)*ti-0.005));

% p=[s s2 wd u wu d wd u wu d wd u wu d wd u wu f f2];
% plot(p)
% break

x2=[s s2 u2 d];
for i=0:(step-2)
    x2=[x2 u2 d u2 d];
end
x2=[x2 u2 f f2];

tcoms(:,1)=xcoff+0.5*x+0.5*x2;
%tcoms(:,1)=0.009+0.5*x2;
%tcoms(:,1)=0.009+0.5*x2-0.5*x;
%tcoms(:,1)=0.009+0.5*x2-1*x;

% figure(20)
% plot(diff(tcoms(:,1)))
% 
% figure(21)
% plot(diff(tcoms(:,1),2))

% figure(8)
% plot(tcoms(:,1))

% xco=xfamp*0.75
% s=0*(0:0.005:((1.5+(1-ssods)/2)*ti-0.005));
% s2=fnval(csape([0 ti*ssods],[0 0.0 -xco 0],[1 1]), 0:0.005:(ti*ssods-0.005));
% d2=fnval(csape([0 (ti*(1-ssods))],[0 xco -xco 0],[1 1]), 0:0.005:(ti*(1-ssods)-0.005));
% u3=fnval(csape([0 ti+(ti*(ssods))],[0 -xco xco 0],[1 1]), 0:0.005:(ti+(ti*(ssods))-0.005));
% f=fnval(csape([0 ti*(1-ssods)],[0 xco 0.0 0],[1 1]), 0:0.005:ti*(1-ssods));
% f2=0*(0:0.005:((0.5+(1-ssods)/2)*ti-0.005));
% 
% x3=[s s2];
% for i=0:(step-2)
%     x3=[x3 u3 d2];
% end
% x3=[x3 u3 f f2];
% 
% tcoms(:,1)=x3+0.009


%break

%% y
clear s s2 wu d wd u f f2
g1=0.74;
g2=0.73;
s=fnval(csape([0 ti/2 3*ti/4 ti],[0 0.0 0.0 g2*ycamp ycamp 0],[1 1]), 0:0.005:(ti-0.005));
d=fnval(csape([0 ti/4 3*ti/4 ti],[0 ycamp g1*ycamp -g1*ycamp -ycamp 0],[1 1]), 0:0.005:(ti-0.005));
u=fnval(csape([0 ti/4 3*ti/4 ti],[0 -ycamp -g1*ycamp g1*ycamp ycamp 0],[1 1]), 0:0.005:(ti-0.005));
f=fnval(csape([0 ti/4 ti/2 ti],[0 -ycamp -g2*ycamp 0.0 0.0 0],[1 1]), 0:0.005:ti);

% s=fnval(csape([0 ti/2 ti],[0 0.0 0.0 ycamp 0],[1 1]), 0:0.005:(ti-0.005));
% d=fnval(csape([0 ti],[0 ycamp -ycamp 0],[1 1]), 0:0.005:(ti-0.005));
% u=fnval(csape([0 ti],[0 -ycamp ycamp 0],[1 1]), 0:0.005:(ti-0.005));
% f=fnval(csape([0 ti/2 ti],[0 -ycamp 0.0 0.0 0],[1 1]), 0:0.005:ti);

y=[s d];
for i=0:(step-1)
    y=[y u d];
end
y=[y f];
tcoms(:,2)=-0.038+y;
tf2fs(:,2)=-0.082+0*t;

%tcoms(:,2)=-0.0368+[s d u d u d u d u d f];
% tcoms(:,2)=-0.04+[s d u d u d u d u d f];
% tf2fs(:,2)=-0.082+0*t;

%% z
clear s d u f
s=0*(0:0.005:((1.5+(1-ssods)/2)*ti-0.005));
w=0*(0:0.005:(ti*(1-ssods)-0.005));
d=fnval(csape([0 ti*ssods/2 ti*ssods],[0 0.0 -zfamp 0.0 0],[1 1]), 0:0.005:(ti*ssods-0.005));
u=fnval(csape([0 ti*ssods/2 ti*ssods],[0 0.0 0.0 0.0 0],[1 1]), 0:0.005:(ti*ssods-0.005));
f=0*(0:0.005:((0.5+(1-ssods)/2)*ti));
% p=[s d w u w d w u w d w u w d w u w d f];
% plot(p)
% break
z=[s d];
for i=0:(step-1)
    z=[z w u w d];
end
z=[z f];

tcoms(:,3)=0.24+z;

s=0*(0:0.005:((1.5+(1-ssodsf)/2)*ti-0.005));
w=0*(0:0.005:(ti*(1-ssodsf)-0.005));
d=fnval(csape([0 ti*ssodsf/2 ti*ssodsf],[0 0.0 -zfamp 0.0 0],[1 1]), 0:0.005:(ti*ssodsf-0.005));
u=fnval(csape([0 ti*ssodsf/2 ti*ssodsf],[0 0.0 zfamp 0.0 0],[1 1]), 0:0.005:(ti*ssodsf-0.005));
f=0*(0:0.005:((0.5+(1-ssodsf)/2)*ti));


z=[s d];
for i=0:(step-1)
    z=[z w u w d];
end
z=[z f];

tf2fs(:,3)=z;

% tcoms(:,3)=0.24+[s d w u w d w u w d w u w d w u w d f];
% u=fnval(csape([0 ti*ssods/2 ti*ssods],[0 0.0 zfamp 0.0 0],[1 1]), 0:0.005:(ti*ssods-0.005));
% tf2fs(:,3)=[s d w u w d w u w d w u w d w u w d f];

%% visu zmp

% figure(1)
% plot(tcoms)
% figure(2)
% plot(tf2fs)

% figure(1)
% plot(diff(tcoms,2))
% figure(2)
% plot(diff(tf2fs,2))

% figure(3)
% hold on 
% plot(tcoms(:,2))
% plot(tcoms(:,2)-(0.24/9.81)*[0;diff(tcoms(:,2),2)./(0.005*0.005);0])

v=0
for i=1:length(t)
    if tf2fs(i,3)<0
        v(i+1)=v(i)-(tf2fs(i,1)-tf2fs((i-1),1));
    else
        v(i+1)=v(i);
    end
    tx(i)=tcoms(i,1)+v(i+1);
end

figure(4)
plot(tx)

% figure(5)
% plot(diff(tx))
%     
% figure(6)
% plot(tx,tcoms(:,2))

figure(7)
hold on 
plot(tx,tcoms(:,2))
%tx-(0.24/9.81)*[0;diff(tx,2)./(0.005*0.005);0]
plot(tx-(0.24/9.81)*[0,diff(tx,2)./(0.005*0.005),0],tcoms(:,2)-(0.24/9.81)*[0;diff(tcoms(:,2),2)./(0.005*0.005);0])


% figure(5)
% plot(v)
% 
% figure(6)
% plot(x2)


% break

%% rec

fid = fopen('./Hoap_Walk2.txt', 'w');
for i=1:size(t,2)
fprintf(fid, '%2.10f,%2.10f,%2.10f,%2.10f,%2.10f,%2.10f,%2.10f,%2.10f,%2.10f\n', tcoms(i,1),tcoms(i,2),tcoms(i,3),tf2fs(i,1),tf2fs(i,2),tf2fs(i,3),oris(i,1),oris(i,2),oris(i,3));
end
fclose(fid)

%close all

%%
% tk=0:0.01:2;
% test=csape([0 2],[0 1 0 0],[1 1])
% 
% a=(test.coefs(1))*tk.^3 + (test.coefs(2))*tk.^2 +(test.coefs(3))*tk +(test.coefs(4));
% plot(tk,a)

