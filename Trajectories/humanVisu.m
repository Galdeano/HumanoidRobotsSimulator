
clear all;
close all;

%%

Human_op = importdata('./Human_op.txt');

t=0:0.005:(size(Human_op,1)-1)*0.005;

figure
hold on
for i=1:9
plot(t,Human_op(:,i))
end



scale_task_F2F=0.353461;
scale_task_CoM=0.305784;

Human_op(:,1)=Human_op(:,1).*0.5;
Human_op(:,4)=Human_op(:,4).*0.5;
Human_op(:,7)=Human_op(:,7).*0.5;


%%
figure
plot3((Human_op(:,4)-Human_op(:,7)).*scale_task_F2F,(Human_op(:,5)-Human_op(:,8)).*scale_task_F2F,(Human_op(:,6)-Human_op(:,9)).*scale_task_F2F)
axis square
xlabel('x')
ylabel('y')
zlabel('z')


figure
plot3((Human_op(:,1)-Human_op(:,7)).*scale_task_CoM,(Human_op(:,2)-Human_op(:,8)).*scale_task_CoM,(Human_op(:,3)-Human_op(:,9)-0.1+0.1).*scale_task_CoM)
axis square
xlabel('x')
ylabel('y')
zlabel('z')

% 
% figure
% plot3(Human_op(:,1).*scale_task_CoM,Human_op(:,2).*scale_task_CoM,(Human_op(:,3)-0.1).*scale_task_CoM)
% axis square
% xlabel('x')
% ylabel('y')
% zlabel('z')
% 
% 
% figure
% plot((Human_op(:,5)-Human_op(:,8)).*scale_task_F2F)

figure
plot(Human_op(:,2).*scale_task_F2F)

figure
hold on
plot3(Human_op(:,1),Human_op(:,2),Human_op(:,3))
plot3(Human_op(:,4),Human_op(:,5),Human_op(:,6))
plot3(Human_op(:,7),Human_op(:,8),Human_op(:,9))
axis square
axis equal
xlabel('x')
ylabel('y')
zlabel('z')

%% 

close all

comy=0.0045*sin(5.7*t+4.4)-0.061;
comy2=0.0105*sin(5.7*t+4.4)-0.0625;
comy3=0.0085*sin(5.7*t+4.4)-0.063;
comy4=0.0145*sin(5.7*t+4.4)-0.062;

% t1=0.25;
% t2=0.92;
% t3=1.6;
% t4=2.2;
% t5=2.8;
t1=0.25;
t2=0.92;
t3=1.71;
t4=2.27;
t5=2.7;

p1=-0.066;
p2=-0.066;
p3=-0.085;
p4=-0.0566;
p5=-0.0655;
p6=-0.059;

ap1=-0.066;
ap2=-0.066;
ap3=-0.128;
ap4=-0.030;
%ap5=-0.100;
ap5=-0.090;
%ap5=-0.085;
ap6=-0.035;

ap23=(ap2+ap3)/2;
ap34=(ap3+ap4)/2;
ap45=(ap4+ap5)/2;
ap56=(ap5+ap6)/2;

% ap33=(15*ap3+ap34)/16;
% ap344=(ap34+15*ap4)/16;
% ap44=(15*ap4+ap45)/16;
% ap455=(ap45+15*ap5)/16;
% ap55=(15*ap5+ap56)/16;
% ap566=(ap56+15*ap6)/16;

ap33=(7*ap3+ap34)/8;
ap344=(ap34+7*ap4)/8;
ap44=(7*ap4+ap45)/8;
ap455=(ap45+7*ap5)/8;
ap55=(7*ap5+ap56)/8;
ap566=(ap56+7*ap6)/8;

pp=csape([0 t1],[0 p1 p2 0],[1 1])
val1=fnval(pp,t);
pp=csape([t1 t2],[0 p2 p3 0],[1 1])
val2=fnval(pp,t);
pp=csape([t2 t3],[0 p3 p4 0],[1 1])
val3=fnval(pp,t);
pp=csape([t3 t4],[0 p4 p5 0],[1 1])
val4=fnval(pp,t);
pp=csape([t4 t5],[0 p5 p6 0],[1 1])
val5=fnval(pp,t);

pp=csape([0 t1],[0 ap1 ap2 0],[1 1])
val1a=fnval(pp,t);
pp=csape([t1 (t1+t2)/2 t2],[0 ap2 ap23 ap3 0],[1 1])
val2a=fnval(pp,t);
pp=csape([t2 (3*t2+t3)/4 (t2+t3)/2 (t2+3*t3)/4 t3],[0 ap3 ap33 ap34 ap344 ap4 0],[1 1])
val3a=fnval(pp,t);
pp=csape([t3 (3*t3+t4)/4 (t3+t4)/2 (t3+3*t4)/4 t4],[0 ap4 ap44 ap45 ap455 ap5 0],[1 1])
val4a=fnval(pp,t);
pp=csape([t4 (3*t4+t5)/4 (t4+t5)/2 (t4+3*t5)/4 t5],[0 ap5 ap55 ap56 ap566 ap6 0],[1 1])
val5a=fnval(pp,t);

figure
hold on
plot(t,Human_op(:,2).*scale_task_F2F)
%plot(t,comy,'r')
%plot(t,comy2,'g')
%plot(t,comy3,'m')
%plot(t,comy4,'y')
plot(t(1:(t1/0.005)),val1(1:(t1/0.005)),'r')
plot(t((t1/0.005):(t2/0.005)),val2((t1/0.005):(t2/0.005)),'r')
plot(t((t2/0.005):(t3/0.005)),val3((t2/0.005):(t3/0.005)),'r')
plot(t((t3/0.005):(t4/0.005)),val4((t3/0.005):(t4/0.005)),'r')
plot(t((t4/0.005):end),val5((t4/0.005):end),'r')

plot(t(1:(t1/0.005)),val1a(1:(t1/0.005)),'g')
plot(t((t1/0.005):(t2/0.005)),val2a((t1/0.005):(t2/0.005)),'g')
plot(t((t2/0.005):(t3/0.005)),val3a((t2/0.005):(t3/0.005)),'g')
plot(t((t3/0.005):(t4/0.005)),val4a((t3/0.005):(t4/0.005)),'g')
plot(t((t4/0.005):end),val5a((t4/0.005):end),'g')

valy=[val1a(1:(t1/0.005)-1),val2a((t1/0.005):(t2/0.005)-1),val3a((t2/0.005):(t3/0.005)-1),val4a((t3/0.005):(t4/0.005)-1),val5a((t4/0.005):end)];


% 
% p1=-0.073;
% p2=-0.071;
% p3=-0.043;
% p4=0.058+0.00;
% p5=0.173+0.00;
% p6=0.292+0.00;
% 
% d=0.1;
% 
% pp=csape([0 t1],[0 p1 p2 0],[1 1])
% val1=fnval(pp,t);
% pp=csape([t1 t2],[0 p2 p3 0.05],[1 1])
% val2=fnval(pp,t);
% pp=csape([t2 t3],[0.05 p3 p4 d],[1 1])
% val3=fnval(pp,t);
% pp=csape([t3 t4],[d p4 p5 d],[1 1])
% val4=fnval(pp,t);
% pp=csape([t4 t5],[d p5 p6 d],[1 1])
% val5=fnval(pp,t);
% 
% figure
% hold on
% plot(t,Human_op(:,1).*scale_task_F2F)
% plot(t(1:(t1/0.005)),val1(1:(t1/0.005)),'r')
% plot(t((t1/0.005):(t2/0.005)),val2((t1/0.005):(t2/0.005)),'r')
% plot(t((t2/0.005):(t3/0.005)),val3((t2/0.005):(t3/0.005)),'r')
% plot(t((t3/0.005):(t4/0.005)),val4((t3/0.005):(t4/0.005)),'r')
% plot(t((t4/0.005):end),val5((t4/0.005):end),'r')
% 
% valx=[val1(1:(t1/0.005)),val2((t1/0.005):(t2/0.005)-1),val3((t2/0.005):(t3/0.005)-1),val4((t3/0.005):(t4/0.005)-1),val5((t4/0.005):end)];





Human_op2=Human_op;

Human_op2(:,1)=Human_op2(:,1)./0.5;
Human_op2(:,4)=Human_op2(:,4)./0.5;
Human_op2(:,7)=Human_op2(:,7)./0.5;
%Human_op2(270:541,2)=(comy4(270:541))./scale_task_F2F;
Human_op2(:,2)=(valy(1,:)')./scale_task_F2F;
%Human_op2(:,1)=(valx(1,:)')./(scale_task_F2F*0.5);

fid = fopen('./Human_op2.txt', 'w');
for i=1:541
fprintf(fid, '%2.10f,%2.10f,%2.10f,%2.10f,%2.10f,%2.10f,%2.10f,%2.10f,%2.10f\n', Human_op2(i,1),Human_op2(i,2),Human_op2(i,3),Human_op2(i,4),Human_op2(i,5),Human_op2(i,6),Human_op2(i,7),Human_op2(i,8),Human_op2(i,9));
end
fclose(fid)

%%

figure
hold on
plot3(Human_op(:,1)./0.5,Human_op(:,2),Human_op(:,3))
plot3(Human_op(:,4)./0.5,Human_op(:,5),Human_op(:,6))
plot3(Human_op(:,7)./0.5,Human_op(:,8),Human_op(:,9))
plot3(Human_op2(:,1),Human_op2(:,2),Human_op2(:,3))
plot3(Human_op2(:,4),Human_op2(:,5),Human_op2(:,6))
plot3(Human_op2(:,7),Human_op2(:,8),Human_op2(:,9))

axis square
axis equal
xlabel('x')
ylabel('y')
zlabel('z')


