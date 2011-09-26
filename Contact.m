
clear all
close all

Kf = 1000000; %1.0E+4 Rigidite (N/m),
Df = 2000;  %1.0E+3 viscosite (N/(m/s)) du sol

g=9.81;
Mass=47.1;

y1=0.1; %initial position

v1 = 0; 
y = y1;  v = v1;
accel=0;

%* Loop until ball hits ground or max steps completed
tau = 0.0001;  % (sec)
t = 1; 
for istep=1:(t/tau)

  %* Record position (computed and theoretical) for plotting
  yplot(istep) = y;   % Record trajectory for plot
  t = (istep-1)*tau;     % Current time

  %* Calculate the acceleration of the ball 
  if( y < 0 )  
  accel = -g;
  f=(-y*Kf-v*Df)/(Mass);  
    if f>0
        accel = -g+f;
    end
  else 
  accel = -g;      % Gravity
  end
  %* Calculate the new position and velocity using Euler method
  y = y + tau*v;                 % Euler step
  v = v + tau*accel;     
  accelplot(istep) = accel;
end

figure(1)
plot(yplot);
%figure(2)
%plot(accelplot);























