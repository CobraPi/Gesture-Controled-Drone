function [s, flag] = setupSerial(comPort)
% Initialize the serial port communication between Arduino and MATLAB
% We ensure that the arduino is also communicating with MATLAB at this
% time. A predefined code on the Arduino acknowledges this.
% If setup is complete then value of setup is returned as 1 else 0
flag = 1;

% If connection open already, close it...
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end
s = serial(comPort);
set(s, 'DataBits', 8);
set(s, 'StopBits', 1);
set(s, 'BaudRate', 115200);
set(s, 'Parity', 'none');
fopen(s);

a = 'b';
while (a ~= 'a')
    a = fread(s, 1, 'uchar');
end

fprintf(s, '%c', 'a');
fscanf(s, '%u');

end