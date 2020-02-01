function cad2matdemo(filename)
% CAD2MATDEMO, a demonstration of importing 3D CAD data into Matlab.
% To get CAD data into Matlab, the process is:
%
% 1) Export the 3D CAD data as an ASCII STL (or Pro/E render SLP) file.
% 2) This Matlab routine reads the CAD data
% 3) Once read, the CAD data is rotated around a bit.
%
% Program has been tested with: AutoCAD, Cadkey, and Pro/Engineer.
% Should work with most any CAD programs that can export STL.
% 
% Format Details:  STL is supported, and the color version of STL
% that Pro/E exports, called 'render.'  The render (SLP) is just 
% like STL but with color added.
% 
% Note: This routine has both the import function and some basic
% manipulation for testing.  The actual reading mechanism is located
% at the end of this file. 

% COM Port Name
comPort = '/dev/tty.usbmodem1411521';
[s, ~] = setupSerial(comPort);
pause(2);

if nargin == 0    
   filename = 'top_plate.stl'; % a simple demo part
   warning(['No file specified, using demo file: ' filename]);
end
%
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
    %title(['Imported CAD data from ' filename])
    drawnow                             %, axis manual
    %
    disp(['CAD file ' filename ' data is read, will now show object rotating'])
    pause(1) 
    
    %
    % Move it around.
    % To use homogenous transforms, the n by 3 Vertices will be turned to 
    % n by 4 vertices, then back to 3 for the set command.
    % Note: n by 4 needed for translations, not used here, but could, using tl(x,y,z)
    
    V = V'; % Transpose V Matrix
    V = [V(1,:); V(2,:); V(3,:); ones(1,length(V))]; % Add row of 1's
    
    vsize = maxv(V); %attempt to determine the maximum xyz vertex. 
    axis([-vsize vsize -vsize vsize -vsize vsize]);
    
    nv = rx(90) * V; %70 = 0
    set(p,'Vertices',nv(1:3,:)')
    drawnow;
    nv = rz(-10) * nv; %70 = 0
    set(p,'Vertices',nv(1:3,:)')
    drawnow;
    V = nv;
   
    ax = 0;
    ay = 0;
    az = 0;
    gx = 0;
    gy = 0;
    gz = 0;
    mx = 0;
    my = 0;
    mz = 0;
    
    yaw = 0;
    pitch = 0;
    roll = 0;
    heading = 0;
    
    accAlpha = 0.35;
    magAlpha = 0.042;
   
    while(1)
        % Ask for Accerlometer Data and Read from the Arduino
        %fprintf(s, '%c', 'p');
%         pitchRaw = fscanf(s, '%e') * -1;
%         rollRaw = fscanf(s, '%e');
%         headingRaw = fscanf(s, '%e') - 65; 
%         pitch = ((1 - alpha) * pitch) + (alpha * pitchRaw);
%         roll = ((1 - alpha) * roll) + (alpha * rollRaw);
%         heading = ((1 - alpha) * heading) + (alpha * headingRaw);
        
        % Accelerometer data requested
        fprintf(s, '%c', 'a');
        axRaw = fscanf(s, '%e');
        ayRaw = fscanf(s, '%e');
        azRaw = fscanf(s, '%e');
        % Gyroscope data requested 
        fprintf(s, '%c', 'g');
        gxRaw = fscanf(s, '%e');
        gyRaw = fscanf(s, '%e');
        gzRaw = fscanf(s, '%e');
        % Magnetometer data requested
        fprintf(s, '%c', 'm');
        mxRaw = fscanf(s, '%e');
        myRaw = fscanf(s, '%e');
        mzRaw = fscanf(s, '%e');
        % Filter values
        ax = ((1 - accAlpha) * ax) + (accAlpha * axRaw);
        ay = ((1 - accAlpha) * ay) + (accAlpha * ayRaw);
        az = ((1 - accAlpha) * az) + (accAlpha * azRaw);
        gx = ((1 - accAlpha) * gx) + (accAlpha * gxRaw);
        gy = ((1 - accAlpha) * gy) + (accAlpha * gyRaw);
        gz = ((1 - accAlpha) * gz) + (accAlpha * gzRaw);
        mx = ((1 - magAlpha) * mx) + (magAlpha * mxRaw);
        my = ((1 - magAlpha) * my) + (magAlpha * myRaw);
        mz = ((1 - magAlpha) * mz) + (magAlpha * mzRaw);
        
        % Normalize Vectors
%        accNorm = sqrt(ax*ax + ay*ay + az*az);
%        ax = ax/accNorm;
%        ay = ay/accNorm;
%        az = az/accNorm;
%        magNorm = sqrt(mx*mx + my*my + mz*mz);
%        mx = mx/magNorm;
%        my = my/magNorm;
%        mz = mz/magNorm;
        
        % Calculate Yaw 
%         if (my == 0)
%             if (mx < 0)
%                 yaw = 180;
%             else 
%                 yaw = 0;
%             end
%         elseif (my > 0)
%             yaw = (pi/2) - atan(mx/my);
%         elseif (my < 0)
%             yaw = (3*pi/2) - atan(mx/my);       
%         end

       
        pitch = atan2(ax, sqrt(ay * ay + az * az));
        roll = atan2(ay, sqrt(ax * ax + az * az));
        yaw = atan2((-my*cos(roll) + mz*sin(roll)),(mx*cos(pitch) + my*sin(pitch)*sin(roll) + mz*sin(pitch)*cos(roll)));
           
     
        yaw = (yaw * 180/pi - 12.7);
        pitch = pitch * 180/pi;
        roll = roll * 180/pi;
        
        %disp(yaw);
        %disp(pitch);
        %disp(roll);
        
        cy = cosd(yaw);
        sy = sind(yaw);
        
        r3_yaw = [cy -sy 0 0;
                  sy cy 0 0;
                  0 0 1 0;
                  0 0 0 1];
        
        cr = cosd(roll);
        sr = sind(roll);
              
        r2_roll = [cr 0 sr 0;
                    0 1 0 0;
                    -sr 0 cr 0;
                    0 0 0 1];
        
        cp = cosd(pitch);
        sp = sind(pitch);
        
        r1_pitch = [1 0 0 0;
                   0 cp -sp 0;
                   0 sp cp 0;
                   0 0 0 1];
                   
        %nv = r3_yaw * r1_pitch * r2_roll*V;       
        nv = r1_pitch * r2_roll * V;
        
        set(p,'Vertices',nv(1:3,:)');
        
        grid on;
        drawnow;
        
        %pause(0.1);
        
        
        
    end

    
%
% End of main routine, here are the functions used:
% Homogeneous manipulation functions follow:
%
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
    vout = v';  
    cout = c';
    %
    fclose(fid);
