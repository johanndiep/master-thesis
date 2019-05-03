% Johann Diep (jdiep@student.ethz.ch) - May 2019

% This program reads and controls the Sniffer module.

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

while true
    command = input("User Input", 's');
    fwrite(serial, command);
end