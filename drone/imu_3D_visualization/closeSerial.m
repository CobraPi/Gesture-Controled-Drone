function closeSerial()
clc
clear all;
if ~isempty(instrfind)
   fclose(instrfind);
   delete(instrfind);
end
clc
end