% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% This function sets up the serial port communication.
%
% Input:
%   - SerialObject (optional): An object of class serial
%
% Output:
%   - SerialObject: An object of class serial

function SerialObject = SerialPortSetup(SerialObject)
    if isempty(SerialObject)
        delete(SerialObject);
        clear serial;
    end
        
    if ~isempty(instrfind)
        fclose(instrfind);
        delete(instrfind);
    end

    PortAddress = seriallist;
    SerialObject = serial(PortAddress);
end