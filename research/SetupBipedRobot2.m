%%% SetupBipedRobot2.m

ToDeg = 180/pi;
ToRad = pi/180;
UX = [1 0 0]';
UY = [0 1 0]';
UZ = [0 0 1]';

% H=0.31;
% Lc=0.5033;
% Lt=0.51;
% Lp=0.28;
Hy=0.155*2;
Hx=0.0468;
Lc=0.5073;
Lt=0.510;
Lp=0.1104;

color=['b','b','b','r','g','g'];

uLINK    = struct('name','BODY'    , 'm',   6.1, 'sister', 0, 'child', 2, 'b',[0  0  0]'    ,'a',UZ,'q',0,'color',126);

uLINK(2) = struct('name','RLEG_J0' , 'm',     0, 'sister', 8, 'child', 3, 'b',[Hx -Hy/2 0]'   ,'a',UX,'q',0,'color',120);
uLINK(3) = struct('name','RLEG_J1' , 'm',     0, 'sister', 0, 'child', 4, 'b',[0  0   0]'   ,'a',UY,'q',0,'color',120);
uLINK(4) = struct('name','RLEG_J2' , 'm',     8, 'sister', 0, 'child', 5, 'b',[0  0   0]'   ,'a',UZ,'q',0,'color',120);
uLINK(5) = struct('name','RLEG_J3' , 'm',  10.9, 'sister', 0, 'child', 6, 'b',[0  0  -Lc]'  ,'a',UY,'q',0,'color',112);
uLINK(6) = struct('name','RLEG_J4' , 'm',     0, 'sister', 0, 'child', 7, 'b',[0  0  -Lt]'  ,'a',UY,'q',0,'color',116);
uLINK(7) = struct('name','RLEG_J5' , 'm',   1.6, 'sister', 0, 'child', 0, 'b',[0  0   0  ]' ,'a',UX,'q',0,'color',116);

uLINK(8) = struct('name','LLEG_J0' , 'm',     0, 'sister', 0, 'child', 9, 'b',[Hx Hy/2 0]'   ,'a',UX,'q',0,'color',120);
uLINK(9) = struct('name','LLEG_J1' , 'm',     0, 'sister', 0, 'child',10, 'b',[0  0   0]'   ,'a',UY,'q',0,'color',120);
uLINK(10)= struct('name','LLEG_J2' , 'm',     8, 'sister', 0, 'child',11, 'b',[0  0   0]'   ,'a',UZ,'q',0,'color',120);
uLINK(11)= struct('name','LLEG_J3' , 'm',  10.9, 'sister', 0, 'child',12, 'b',[0  0  -Lc]'  ,'a',UY,'q',0,'color',112);
uLINK(12)= struct('name','LLEG_J4' , 'm',     0, 'sister', 0, 'child',13, 'b',[0  0  -Lt]'  ,'a',UY,'q',0,'color',116);
uLINK(13)= struct('name','LLEG_J5' , 'm',   1.6, 'sister', 0, 'child', 0, 'b',[0  0   0  ]' ,'a',UX,'q',0,'color',116);

FindMother(1);   

% static const float buteePlus[12]  = {  1.07f ,  0.30f ,  1.02f ,  1.85f ,  1.06f ,  1.06f , // rad
%                                        1.16f ,  0.29f ,  1.02f ,  1.85f ,  1.09f ,  1.07f } ; // 8&9 copy de 2&3
% static const float buteeMoins[12] = { -1.06f , -1.91f , -1.01f , -0.10f , -1.06f , -1.03f , // rad
%                                       -1.04f , -1.70f , -1.01f , -0.10f , -1.03f , -1.08f } ; // 8&9 copy de 2&3



for n=1:length(uLINK)
    uLINK(n).dq     = 0;            % joint velocity [rad/s]
    uLINK(n).ddq    = 0;            % joint acceleration [rad/s^2]
    %uLINK(n).c      = [0 0 0]';     % center of gravity [m]
    uLINK(n).c      = uLINK(n).b/2;     % center of gravity [m]
    uLINK(n).I      = zeros(3,3);   % around the center of gravity inertia tensor [kg.m^2]
    uLINK(n).Ir     = 0.0;          % child electric motor moment of inertia [kg.m^2]
    uLINK(n).gr     = 0.0;          % reduction ratio of the motor [-]
    uLINK(n).u      = 0.0;          % joint torque [Nm]
end

%Mass=TotalMass(1)	 


%%
for n=1:length(uLINK)
    eval([uLINK(n).name,'=',num2str(n),';']);
end

%%%%%%%%%visualisation %%%%%%%%%
shape = [0.1 0.3 0.04];
com   = [0 0 0.05]';
SetupRigidBody(BODY, shape,com);

shape = [0.28 0.08 0.04];
com   = [0.0853  0 -0.08]';
SetupRigidBody(RLEG_J5, shape,com);

shape = [0.28 0.08 0.04];
com   = [0.0853  0 -0.08]';
SetupRigidBody(LLEG_J5, shape,com);


uLINK(RLEG_J1).q = -10.0*ToRad;
uLINK(RLEG_J3).q = 20.0*ToRad;
uLINK(RLEG_J4).q = -10.0*ToRad;

uLINK(LLEG_J1).q = -10.0*ToRad;
uLINK(LLEG_J3).q = 20.0*ToRad;
uLINK(LLEG_J4).q = -10.0*ToRad;

uLINK(BODY).p = [0.0, 0.0, 1.0]';
uLINK(BODY).R = eye(3);


for n=1:length(uLINK)
    uLINK(n).c      = uLINK(n).b/2;     % center of gravity [m]
    uLINK(n).I      = zeros(3,3);   % around the center of gravity inertia tensor [kg.m^2]
end


% Eléments d'inertie du robot Sherpa - 20091021SK
mPelvis=3.2+1.9+1; % [kg]
mUpperLimb=5+3; % [kg]
mLowerLimb=9.9+1; % [kg]
mFoot=0.8+0.8; % [kg]
%Total=mPelvis+2*mUpperLimb+2*mLowerLimb+2*mFoot


uLINK(BODY).m = mPelvis;
uLINK(BODY).c = [-5.26, 0, 90.54]'*10^(-3);
uLINK(BODY).I = [67142085.09,216.58,-574455.03;216.58,19827233.11,911.80;-574455.03,911.80,55544704.20]*10^(-9);

uLINK(RLEG_J0).m = 0;
uLINK(RLEG_J0).c = [0, 0, 0]';
uLINK(RLEG_J0).I = [0 0 0;0 0 0;0 0 0];

uLINK(RLEG_J1).m = 0;
uLINK(RLEG_J1).c = [6.8 -2.5 0.76]'*10^(-3);
uLINK(RLEG_J1).I = [41333.63,-2250.09,-322.24;-2250.09,34045.60,253.84;-322.24,253.84,63665.84]*10^(-9);


uLINK(RLEG_J2).m  = mUpperLimb;
uLINK(RLEG_J2).c  = [32, 0 , -204]'*10^(-3);
uLINK(RLEG_J2).I = [86258877.92,-98067.36,874868.98;-98067.36,83201177.34,548297.57;874868.98,548297.57,12112937.91]*10^(-9);
    

uLINK(RLEG_J3).m  = mLowerLimb;
uLINK(RLEG_J3).c  = [0, 0, -255]'*10^(-3);
uLINK(RLEG_J3).I = [193722580.67,9634.89,28785499.93;9634.89,199833828.82,-1258803.65;28785499.93,-1258803.65,35402462.45]*10^(-9);


uLINK(RLEG_J5).m  = mFoot;
uLINK(RLEG_J5).c  =  [85.9, 0, 67.1]'*10^(-3);
uLINK(RLEG_J5).I =  [3547351.01,-13737.57,-1352828.32;-13737.57,7363368.32,1769.65;-1352828.32,1769.65,5552347.77]*10^(-9);

uLINK(LLEG_J0).m  = 0;
uLINK(LLEG_J0).c  =  [0, 0, 0]';
uLINK(LLEG_J0).I =  [0,0,0;0,0,0;0,0,0];

uLINK(LLEG_J1).m  = 0; %0.129;
uLINK(LLEG_J1).c  =  [6.8, 2.5, 0.76]'*10^(-3);
uLINK(LLEG_J1).I =  [41333.63,-2250.09,-322.24;-2250.09,34045.60,253.84;-322.24,253.84,63665.84]*10^(-9);

uLINK(LLEG_J2).m  = mUpperLimb; %5.792;
uLINK(LLEG_J2).c  =  [32, 0 , -204]'*10^(-3);
uLINK(LLEG_J2).I =  [86258877.92,-98067.36,874868.98;-98067.36,83201177.34,548297.57;874868.98,548297.57,12112937.91]*10^(-9);

uLINK(LLEG_J3).m  = mLowerLimb; %10.447;
uLINK(LLEG_J3).c  =  [0, 0, -255]'*10^(-3);
uLINK(LLEG_J3).I =  [193722580.67,9634.89,28785499.93;9634.89,199833828.82,-1258803.65;28785499.93,-1258803.65,35402462.45]*10^(-9);

uLINK(LLEG_J5).m  = mFoot; %1.614;
uLINK(LLEG_J5).c  =  [85.9, 0, 67.1]'*10^(-3);
uLINK(LLEG_J5).I =  [3547351.01,-13737.57,-1352828.32;-13737.57,7363368.32,1769.65;-1352828.32,1769.65,5552347.77]*10^(-9);



ForwardKinematics(1);


%DrawAllJoints(1);
%    axis equal
%    set(gca,...
%        'CameraPositionMode','manual',...
%        'CameraPosition',[4,4,1],...
%        'CameraViewAngleMode','manual',....
%        'CameraViewAngle',15,...
%        'Projection','perspective',... 
%        'XLimMode','manual',...
%        'XLim',[-0.5 0.5],...
%        'YLimMode','manual',...
%        'YLim',[-0.5 0.5],...
%        'ZLimMode','manual',...
%        'ZLim',[0 1.5])
%    grid on
%    drawnow;

% for i=1:13
%     disp(['gsl_vector_set (uLINK[',num2str(i),'].c, 0, ',num2str(uLINK(i).c(1)),');']);
%     disp(['gsl_vector_set (uLINK[',num2str(i),'].c, 1, ',num2str(uLINK(i).c(2)),');']);
%     disp(['gsl_vector_set (uLINK[',num2str(i),'].c, 2, ',num2str(uLINK(i).c(3)),');']);
%     disp([' ']);
% end    

for i=1:13
    disp(['gsl_matrix_set (uLINK[',num2str(i),'].I, 0, 0, ',num2str(uLINK(i).I(1,1), '%2.11f'),');']);
    disp(['gsl_matrix_set (uLINK[',num2str(i),'].I, 0, 1, ',num2str(uLINK(i).I(1,2), '%2.11f'),');']);
    disp(['gsl_matrix_set (uLINK[',num2str(i),'].I, 0, 2, ',num2str(uLINK(i).I(1,3), '%2.11f'),');']);
    disp(['gsl_matrix_set (uLINK[',num2str(i),'].I, 1, 0, ',num2str(uLINK(i).I(2,1), '%2.11f'),');']);
    disp(['gsl_matrix_set (uLINK[',num2str(i),'].I, 1, 1, ',num2str(uLINK(i).I(2,2), '%2.11f'),');']);
    disp(['gsl_matrix_set (uLINK[',num2str(i),'].I, 1, 2, ',num2str(uLINK(i).I(2,3), '%2.11f'),');']);
    disp(['gsl_matrix_set (uLINK[',num2str(i),'].I, 2, 0, ',num2str(uLINK(i).I(3,1), '%2.11f'),');']);
    disp(['gsl_matrix_set (uLINK[',num2str(i),'].I, 2, 1, ',num2str(uLINK(i).I(3,2), '%2.11f'),');']);
    disp(['gsl_matrix_set (uLINK[',num2str(i),'].I, 2, 2, ',num2str(uLINK(i).I(3,3), '%2.11f'),');']);
    disp([' ']);
end  
