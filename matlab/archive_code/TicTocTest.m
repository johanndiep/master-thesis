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
line = fgetl(serial);
line = fgetl(serial);
line = fgetl(serial);
line = fgetl(serial);
line = fgetl(serial);
line = fgetl(serial);
time = toc;
   
t = mean(time);
freq = 1/mean(time)

%% GetRangeMeasurement function

tic;
getRangeMeasurement(serial);
time = toc;

freq = 1/time