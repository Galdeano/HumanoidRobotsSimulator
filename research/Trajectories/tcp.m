
%% initializations
clc
clear all 
close all
pnet('closeall')

%% Data import
% m03 = importdata('./m03.csv');
fid = fopen('./m03.csv');
m=zeros(21,17902);
for i=1:17902
    tline = fgetl(fid);
    m(:,i)=sscanf(tline, '%*hd,%*hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%*hd,%*hd,%*c,%*c,%*c,%*c', [1, inf]);
    %disp(tline)
end
fclose(fid);

data_t=zeros(17902,21);


%% Initialization of ip communication
%Use TCP/UDP/IP Toolbox 2.0.6
%http://www.mathworks.com/matlabcentral/fileexchange/345
host='10.59.145.197';
port=55000;
con=pnet('tcpconnect',host,port)
if con==-1, error 'Bad url or server down.....'; end
disp(['Connected to: ' host]);

pause(1);


%% Control Loop
  for i=1:17902
%sensor read
  data=pnet(con,'read',21+21+4+4+6,'int16','intel');
%data contain
%     short q[21];
%     short dq[21];
%     short fsr_right[4];
%     short fsr_left[4];
%     short acc_gyro[6];
%
%to have articular positions in radians
%do q_rad=(deg2rad/209)*q_encoders;
  try
  data_t(i,:)=data;%data storage
  catch ME
  end

%motor write
  pnet(con,'write', int16(m(:,i)),'intel');
  end

%% Close communications 
pnet(con,'close');
pnet('closeall')





