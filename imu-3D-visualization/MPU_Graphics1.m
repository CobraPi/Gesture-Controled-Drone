% MPU6050 Function
function MPU_Graphics()
% COM Port Name
comPort = '/dev/tty.usbmodem1A1211';

% GUI Figure
fig = figure;
set(fig, 'Position', [250 300 1200 600], 'Color', [0.941 0.941 0.941]);
% Position: [left bottom width height]

% Axes for Accelerometer Vectors
hAxesAcc = axes(...
    'Units','normalized',...
    'Position', [0.1 0.2 0.25 0.5], ...
    'Tag','hAxesAcc', ...
    'Parent', fig);
zlabel('Acceleration (g)');
ylabel('Acceleration (g)');
xlabel('Acceleration (g)');

% Axes for Gyroscope Vectors
hAxesGyr = axes(...
    'Units','normalized',...
    'Position', [0.4 0.2 0.25 0.5], ...
    'Tag','hAxesGyr', ...
    'Parent', fig);
zlabel('Angular Velocity (rad/s)');
ylabel('Angular Velocity (rad/s)');
xlabel('Angular Velocity (rad/s)');

% Axes for Magnetometer Vectors
hAxesGyr = axes(...
    'Units','normalized',...
    'Position', [0.7 0.2 0.25 0.5], ...
    'Tag','hAxesMag', ...
    'Parent', fig);
zlabel('Heading (Gauss)');
ylabel('Heading (Gauss)');
xlabel('Heading (Gauss)');

% Connect/Disconnect Button
hConnectBtn = uicontrol(...
    'Style', 'pushbutton', ...
    'String', 'Connect', ...
    'Units', 'normalized', ...
    'Callback', {@connectArduino,comPort}, ...
    'Position', [0.1 0.93 0.1 0.05], ...
    'Tag', 'hConnectBtn', ...
    'Parent', fig);

% Start/Stop Button
hStopBtn = uicontrol(...
    'Style', 'pushbutton', ...
    'String', 'Start', ...
    'Units', 'normalized', ...
    'Enable', 'off', ...
    'Callback', {@stopBtnCallback}, ...
    'Position', [0.1 0.88 0.1 0.05], ...
    'Tag', 'hStopBtn', ...
    'Parent', fig);

% Calibrate Accelerometer Button
hCaliBtn = uicontrol(...
    'Style', 'pushbutton', ...
    'String', 'Calibrate', ...
    'Units', 'normalized', ...
    'Enable', 'off', ...
    'Callback', {@calibrateAcc}, ...
    'Position', [0.1 0.83 0.1 0.05], ...
    'Tag', 'hCaliBtn', ...
    'Parent', fig);

% Get All Handles in Figure
myhandles = guihandles(fig);

% Set Poll Data Flag
data.stop = false;

% Set Relevant User Data
set(myhandles.hStopBtn, 'UserData', data);
calCo.offset = [0,0,0];
calCo.g = [1,1,1];
set(myhandles.hCaliBtn, 'UserData', calCo);

% Store UI Data
guidata(fig, myhandles);

end




function pollData(myhandles)
   
    
    % Get Serial Port Instance
    s = get(myhandles.hConnectBtn, 'UserData');
    % Initialized Accelerometer and Gyroscope Max Values
    xmaxacc = 0;
    ymaxacc = 0;
    zmaxacc = 0;
    xmaxgyr = 0;
    ymaxgyr = 0;
    zmaxgyr = 0;
    xmaxmag = 0;
    ymaxmag = 0;
    zmaxmag = 0;
    
    % Read the CAD data file:
    [F, V, C] = rndread(filename);
    clf;
    p = patch('faces', F, 'vertices' ,V);
    %set(p, 'facec', 'b');              % Set the face color (force it)
    set(p, 'facec', 'flat');            % Set the face color flat
    set(p, 'FaceVertexCData', C);       % Set the color (from file)
    %set(p, 'facealpha',.4)             % Use for transparency
    set(p, 'EdgeColor','none');         % Set the edge color
    %set(p, 'EdgeColor',[1 0 0 ]);      % Use to see triangles, if needed.
    light                               % add a default light
    daspect([1 1 1])                    % Setting the aspect ratio
    view(3)                             % Isometric view
    xlabel('X'),ylabel('Y'),zlabel('Z')
    title(['Imported CAD data from ' filename])
    drawnow                             %, axis manual
    %
    disp(['CAD file ' filename ' data is read, will now show object rotating'])
    pause(1) 
    
    
    % Start Loop to Poll Data From Arduino
    keepPolling = true;
    while keepPolling
        % Check State of Start/Stop Button UserData
        userData = get(myhandles.hStopBtn, 'UserData');             
        % Ask for Accerlometer Data and Read from the Arduino
        fprintf(s, '%c', 'a');
        accX = fscanf(s, '%e');
        accY = fscanf(s, '%e');
        accZ = fscanf(s, '%e');
        % Get calibration data
        calCo = get(myhandles.hCaliBtn, 'UserData');
        accX = (accX - calCo.offset(1)) / calCo.g(1);
        accY = (accY - calCo.offset(2)) / calCo.g(2);
        accZ = (accZ - calCo.offset(3)) / calCo.g(3);        
        % Ask for Gyroscope Data and Read from the Arduino
        fprintf(s, '%c', 'g');
        gyrX = fscanf(s, '%e');
        gyrY = fscanf(s, '%e');
        gyrZ = fscanf(s, '%e');
        % Ask for Magnetometer Data and Read from the Arduino
        fprintf(s, '%c', 'm');
        magX = fscanf(s, '%e');
        magY = fscanf(s, '%e');
        magZ = fscanf(s, '%e');
        
           
        % Move it around.
        % To use homogenous transforms, the n by 3 Vertices will be turned to 
        % n by 4 vertices, then back to 3 for the set command.
        % Note: n by 4 needed for translations, not used here, but could, using tl(x,y,z)
        V = V';
        V = [V(1,:); V(2,:); V(3,:); ones(1,length(V))];
        %
        vsize = maxv(V); %attempt to determine the maximum xyz vertex. 
        axis([-vsize vsize -vsize vsize -vsize vsize]);
        %
        for ang = 0:1:45
            nv = rx(ang)*V;
            set(p,'Vertices',nv(1:3,:)')       
            drawnow
        end
        
        % If Start/Stop Button UserData Shows Button Pressed
        if userData.stop == true
            % Update Start/Stop Button UserData and Change Polling Loop
            % Flag
            userData.stop = false;
            set(myhandles.hStopBtn, 'UserData', userData);
            keepPolling = false;
            disp('stopped');
        end
    end
end


function calibrateAcc(hObject, ~)
    % Bring Command Window to the Front
    commandwindow;
    % Get UI Handles
    myhandles = guidata(gcbo);
    % Get Serial Instance
    s = get(myhandles.hConnectBtn, 'UserData');
    % Prompt User to Orient The MPU6050
    disp('Hold the Accelerometer so that the X-Axis is facing upwards and press any key to continue');
    pause;
    % Read Accelerometer Values
    [accx_x, accx_y, accx_z] = readAcc(s);
    % Prompt User to Orient The MPU6050
    disp('Hold the Accelerometer so that the Y-Axis is facing upwards and press any key to continue');
    pause;
    % Read Accelerometer Values
    [accy_x, accy_y, accy_z] = readAcc(s);
    % Prompt User to Orient The MPU6050
    disp('Hold the Accelerometer so that the Z-Axis is facing upwards and press any key to continue');
    pause;
    % Read Accelerometer Values
    [accz_x, accz_y, accz_z] = readAcc(s);
    % Calculate Offsets
    offsetX = (accx_z + accx_y) / 2;
    offsetY = (accy_x + accy_z) / 2;
    offsetZ = (accz_x + accz_y) / 2;
    % Calculate Gains
    gainX = accx_x - offsetX;
    gainY = accy_y - offsetY;
    gainZ = accz_z - offsetZ;
    % Store Calibration Data in Struct
    calCo.offset = [offsetX, offsetY, offsetZ];
    calCo.g = [gainX, gainY, gainZ];
    % Store Calibration Data Struct in UserData
    set(myhandles.hCaliBtn, 'UserData', calCo);
    % Set UI Handles
    guidata(hObject, myhandles);
end


function [acc_x, acc_y, acc_z] = readAcc(s)
    % Read X, Y, Z Accelerometer Values
    fprintf(s, '%c', 'a');
    acc_x = fscanf(s, '%e');
    acc_y = fscanf(s, '%e');
    acc_z = fscanf(s, '%e');
    
end

% Start Polling Data When Start Pressed; Stop Polling Data When Stop Pressed
function stopBtnCallback(hObject, ~)
    myhandles = guidata(gcbo);
    tmpStr = get(hObject, 'String');
    if strcmp(tmpStr, 'Start')
        set(hObject, 'String', 'Stop');
        pollData(myhandles);
    else
        userData = get(myhandles.hStopBtn, 'UserData');
        userData.stop = true;
        set(myhandles.hStopBtn, 'UserData', userData);
        set(hObject, 'String', 'Start');
        guidata(hObject, myhandles);       
    end  
end

% Connect to Arduino When Connect Pressed; Sever Connection When Disconnect
% Pressed
function connectArduino(hObject, ~, comPort)
    tmpStr = get(hObject, 'String');
    myhandles = guidata(gcbo);
    if strcmp(tmpStr, 'Connect')
        set(myhandles.hStopBtn, 'Enable', 'on');
        set(myhandles.hCaliBtn, 'Enable', 'on');
        set(hObject, 'String', 'Disconnect');
        if (~exist('serialFlag', 'var'))
            [s, ~] = setupSerial(comPort);
        else
            s = [];
        end
    else
        set(hObject, 'String', 'Connect');
        set(myhandles.hStopBtn, 'Enable', 'off');
        set(myhandles.hCaliBtn, 'Enable', 'off');
        closeSerial();
        s = [];
    end
    set(myhandles.hConnectBtn, 'UserData', s);
    guidata(hObject, myhandles);
end

function Rx = rx(THETA)
% ROTATION ABOUT THE X-AXIS
%
% Rx = rx(THETA)
%
% This is the homogeneous transformation for
% rotation about the X-axis.
%
%	    NOTE:  The angle THETA must be in DEGREES.
%
THETA = THETA*pi/180;  % Note: THETA in radians.
c = cos(THETA);
s = sin(THETA);
Rx = [1 0 0 0; 0 c -s 0; 0 s c 0; 0 0 0 1];
%
end
function Ry = ry(THETA)
% ROTATION ABOUT THE Y-AXIS
%
% Ry = ry(THETA)
%
% This is the homogeneous transformation for
% rotation about the Y-axis.
%
%		NOTE: The angel THETA must be in DEGREES.
%
THETA = THETA*pi/180;  %Note: THETA is in radians.
c = cos(THETA);
s = sin(THETA);
Ry = [c 0 s 0; 0 1 0 0; -s 0 c 0; 0 0 0 1];
%
end
function Rz = rz(THETA)
% ROTATION ABOUT THE Z-AXIS
%
% Rz = rz(THETA)
%
% This is the homogeneous transformation for
% rotation about the Z-axis.
%
%		NOTE:  The angle THETA must be in DEGREES.
%
THETA = THETA*pi/180;  %Note: THETA is in radians.
c = cos(THETA);
s = sin(THETA);
Rz = [c -s 0 0; s c 0 0; 0 0 1 0; 0 0 0 1];
%
end
function T = tl(x,y,z)
% TRANSLATION ALONG THE X, Y, AND Z AXES
%
% T = tl(x,y,z)
%
% This is the homogeneous transformation for
% translation along the X, Y, and Z axes.
%
T = [1 0 0 x; 0 1 0 y; 0 0 1 z; 0 0 0 1];
%
end
function vsize = maxv(V)
%
% Look at the xyz elements of V, and determine the maximum
% values during some simple rotations.
    vsize = max(max(V));
    % Rotate it a bit, and check for max and min vertex for viewing.
    for ang = 0:10:360
        vsizex = rx(ang)*V;
        maxv = max(max(vsizex));
        if maxv > vsize, vsize = maxv; end
        vsizey = ry(ang)*V;
        maxv = max(max(vsizey));
        if maxv > vsize, vsize = maxv; end
        vsizez = rz(ang)*V;
        maxv = max(max(vsizez));
        if maxv > vsize, vsize = maxv; end
        vsizev = rx(ang)*ry(ang)*rz(ang)*V;
        maxv = max(max(vsizev));
        if maxv > vsize, vsize = maxv; end
    end
    %
end
function [fout, vout, cout] = rndread(filename)
% Reads CAD STL ASCII files, which most CAD programs can export.
% Used to create Matlab patches of CAD 3D data.
% Returns a vertex list and face list, for Matlab patch command.
% 
% filename = 'hook.stl';  % Example file.
%
fid=fopen(filename, 'r'); %Open the file, assumes STL ASCII format.
if fid == -1 
    error('File could not be opened, check name or path.')
end
%
% Render files take the form:
%   
%solid BLOCK
%  color 1.000 1.000 1.000
%  facet
%      normal 0.000000e+00 0.000000e+00 -1.000000e+00
%      normal 0.000000e+00 0.000000e+00 -1.000000e+00
%      normal 0.000000e+00 0.000000e+00 -1.000000e+00
%    outer loop
%      vertex 5.000000e-01 -5.000000e-01 -5.000000e-01
%      vertex -5.000000e-01 -5.000000e-01 -5.000000e-01
%      vertex -5.000000e-01 5.000000e-01 -5.000000e-01
%    endloop
% endfacet
%
% The first line is object name, then comes multiple facet and vertex lines.
% A color specifier is next, followed by those faces of that color, until
% next color line.
%
CAD_object_name = sscanf(fgetl(fid), '%*s %s');  %CAD object name, if needed.
%                                                %Some STLs have it, some don't.   
vnum=0;       %Vertex number counter.
report_num=0; %Report the status as we go.
VColor = 0;
%
while feof(fid) == 0                    % test for end of file, if not then do stuff
    tline = fgetl(fid);                 % reads a line of data from file.
    fword = sscanf(tline, '%s ');       % make the line a character string
% Check for color
    if strncmpi(fword, 'c',1) == 1;    % Checking if a "C"olor line, as "C" is 1st char.
       VColor = sscanf(tline, '%*s %f %f %f'); % & if a C, get the RGB color data of the face.
    end                                % Keep this color, until the next color is used.
    if strncmpi(fword, 'v',1) == 1;    % Checking if a "V"ertex line, as "V" is 1st char.
       vnum = vnum + 1;                % If a V we count the # of V's
       report_num = report_num + 1;    % Report a counter, so long files show status
       if report_num > 249;
           disp(sprintf('Reading vertix num: %d.',vnum));
           report_num = 0;
       end
       v(:,vnum) = sscanf(tline, '%*s %f %f %f'); % & if a V, get the XYZ data of it.
       c(:,vnum) = VColor;              % A color for each vertex, which will color the faces.
    end                                 % we "*s" skip the name "color" and get the data.                                          
end
%   Build face list; The vertices are in order, so just number them.
%
fnum = vnum/3;      %Number of faces, vnum is number of vertices.  STL is triangles.
flist = 1:vnum;     %Face list of vertices, all in order.
F = reshape(flist, 3,fnum); %Make a "3 by fnum" matrix of face list data.
%
%   Return the faces and vertexs.
%
fout = F';  %Orients the array for direct use in patch.
vout = v';  % "
cout = c';
%
fclose(fid);
end