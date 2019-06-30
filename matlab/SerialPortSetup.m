% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% This function sets up the serial port communication.
%
% Output:
%   - SerialObject: An object of class serial

function SerialObject = SerialPortSetup()
    if ~isempty(instrfind)
        fclose(instrfind);
        delete(instrfind);
    end

    PortAddress = seriallist;
    SerialObject = serial(PortAddress);
end