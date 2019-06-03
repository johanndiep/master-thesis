% Johann Diep (jdiep@student.ethz.ch) - April 2019

% This program measures the data rate. 

clear 
clc

%% Closing and deleting ports

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);   
end

%% Range and pressure aquisition form each anchor via TWR

port = seriallist;
serial = serial(port);
fopen(serial); % run sudo chmod 666 /dev/ttyACM* on console first

tictoc_variable = 1;

tic;
for i = 1:100
   if tictoc_variable == 0
       tic;
       tictoc_variable = 1;
   end
    
   line = fgetl(serial);
   line = fgetl(serial);
   line = fgetl(serial);
   line = fgetl(serial);
   line = fgetl(serial);
   line = fgetl(serial);
   line = fgetl(serial);
   line = fgetl(serial);
   
   if tictoc_variable == 1
       time(i) = toc;
       tictoc_variable = 0;
   end
end
t = mean(time)
freq = 1/mean(time)

%% GetRangeMeasurement function

tic;
getRangeMeasurement(serial);
time = toc;

freq = 1/time