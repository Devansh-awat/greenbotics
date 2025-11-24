clear; clc; close all;

%% --- Arena geometry ---
plane_dims       = [3, 3];   % arena size (meters)
plane_thickness  = 0.01;

% outer walls (3D only)
border_height       = 0.10;
border_thickness    = 0.02;
border_color        = [0 0 0];

% inner square (central obstacle, visual reference)
inner_square_dims   = [1, 1];   % central square size (meters)

%% --- Main front-view figure (robot camera style) ---
figFront = figure('Name','Greenbotics WRO Arena - Front View',...
                  'Color',[0.05 0.05 0.1],'Position',[50 100 900 650]);
axFront = axes('Parent',figFront); 
hold(axFront,'on'); 
axis(axFront,'equal');
axis(axFront,[0 plane_dims(1) 0 plane_dims(2) 0 0.4]);
axFront.Color = [0.2 0.2 0.25];
xlabel(axFront,'X (m)');
ylabel(axFront,'Y (m)');
zlabel(axFront,'Z (m)');
title(axFront,'Front / Chase View');

%% --- Top-down debug view ---
figTop = figure('Name','Greenbotics WRO Arena - Top Debug View',...
                'Color',[0.05 0.05 0.1],'Position',[1000 100 600 600]);
axTop = axes('Parent',figTop); 
hold(axTop,'on'); 
axis(axTop,'equal');
axis(axTop,[0 plane_dims(1) 0 plane_dims(2)]);
axTop.Color = [0.15 0.15 0.18];
xlabel(axTop,'X (m)');
ylabel(axTop,'Y (m)');
title(axTop,'Top View (see robot path & turns)');

%% --- Floor with WRO texture (used in both views) ---
mat_img      = [];
pxPerMeterX  = [];
pxPerMeterY  = [];
image_file   = 'wro.png';

try
    tex = im2double(imread(image_file));
catch
    try
        image_file = 'wro.jpeg';
        tex = im2double(imread(image_file));
    catch
        tex = [];
    end
end

if ~isempty(tex)
    [imgH,imgW,~] = size(tex);
    pxPerMeterX = (imgW-1) / plane_dims(1);
    pxPerMeterY = (imgH-1) / plane_dims(2);
    mat_img     = tex;

    % Front view floor
    [X,Y] = meshgrid(linspace(0,plane_dims(1),imgW), ...
                     linspace(0,plane_dims(2),imgH));
    Z = ones(size(X))*(plane_thickness + 0.0001);
    surface(axFront,X,Y,Z,'CData',tex,'FaceColor','texturemap','EdgeColor','none');

    % Top view floor (2D image)
    image(axTop,linspace(0,plane_dims(1),imgW), ...
                linspace(0,plane_dims(2),imgH),flipud(tex));
    set(axTop,'YDir','normal');
    disp(['Using mat texture from ', image_file]);
else
    [X,Y] = meshgrid(linspace(0,plane_dims(1),50), ...
                     linspace(0,plane_dims(2),50));
    Z = ones(size(X))*plane_thickness;
    surf(axFront,X,Y,Z,'FaceColor',[0.3 0.6 0.3],'EdgeColor','none');

    imagesc(axTop,[0 plane_dims(1)],[0 plane_dims(2)],0.3*ones(50,50,3));
    set(axTop,'YDir','normal');

    disp('Could not load wro.png or wro.jpeg, using plain green mat.');
end

%% --- Outer walls (3D in front view + 2D outline in top view) ---
b  = border_thickness;
h  = border_height;
z0 = plane_thickness;

% 3D outer walls
drawCuboid(axFront,[0,0,z0],                  [plane_dims(1),b,h],       border_color);
drawCuboid(axFront,[0,plane_dims(2)-b,z0],    [plane_dims(1),b,h],       border_color);
drawCuboid(axFront,[0,0,z0],                  [b,plane_dims(2),h],       border_color);
drawCuboid(axFront,[plane_dims(1)-b,0,z0],    [b,plane_dims(2),h],       border_color);

% 2D outline in top view
plot(axTop,[0 plane_dims(1) plane_dims(1) 0 0], ...
          [0 0 plane_dims(2) plane_dims(2) 0],'k-','LineWidth',2);

%% --- Inner square walls (3D + 2D outline) ---
offset   = (plane_dims - inner_square_dims)/2;
inner_min = offset;                       % [1,1]
inner_max = offset + inner_square_dims;   % [2,2]
inner_wall_thickness = 0.08;
inner_wall_height    = 0.15;
inner_wall_color     = [0 0 0];

t_in = inner_wall_thickness;
h_in = inner_wall_height;
c_in = inner_wall_color;

% 3D inner walls
drawCuboid(axFront, [inner_min(1),                inner_min(2),                z0], ...
                    [inner_square_dims(1), t_in,               h_in], c_in);
drawCuboid(axFront, [inner_min(1),                inner_max(2)-t_in, z0], ...
                    [inner_square_dims(1), t_in,   h_in], c_in);
drawCuboid(axFront, [inner_min(1),                inner_min(2),                z0], ...
                    [t_in,                 inner_square_dims(2), h_in],    c_in);
drawCuboid(axFront, [inner_max(1)-t_in, inner_min(2),     z0], ...
                    [t_in,                 inner_square_dims(2), h_in],    c_in);

% 2D outline inner square
plot(axTop,[inner_min(1) inner_max(1) inner_max(1) inner_min(1) inner_min(1)], ...
          [inner_min(2) inner_min(2) inner_max(2) inner_max(2) inner_min(2)], ...
          'k-','LineWidth',2);

%% --- Lighting (front view) ---
lighting(axFront,'gouraud'); material(axFront,'dull');
light('Parent',axFront,'Position',[1.5,1.5,5],'Style','local');
camproj(axFront,'perspective');

%% --- Path around inner walls (anticlockwise rectangle) ---
follow_offset = 0.30;   % distance from inner walls

% Corners of the rectangle we want to follow (outside inner square)
c1 = [inner_min(1) - follow_offset, inner_min(2) - follow_offset]; % bottom-left
c2 = [inner_max(1) + follow_offset, inner_min(2) - follow_offset]; % bottom-right
c3 = [inner_max(1) + follow_offset, inner_max(2) + follow_offset]; % top-right
c4 = [inner_min(1) - follow_offset, inner_max(2) + follow_offset]; % top-left

waypoints = [c1; c2; c3; c4];      % anticlockwise loop
num_wp    = size(waypoints,1);
wp_idx    = 1;                     % <<< this was missing in the other version

% Show the path in top view
plot(axTop,waypoints([1:4 1],1),waypoints([1:4 1],2),'c--','LineWidth',1.5);

%% --- Robot setup: start at first waypoint, facing next ---
robot.pos  = [waypoints(1,:), 0.015];  % x,y,z
next_wp    = waypoints(2,:);
hd0        = atan2d(next_wp(2)-robot.pos(2), next_wp(1)-robot.pos(1));
robot.heading = hd0;
robot.size    = 0.15;
robot.color   = [1 0 0];

[XrF,YrF,ZrF] = makeCube(robot.size,robot.pos,robot.heading);
robot.patchFront = surf(axFront,XrF,YrF,ZrF,'FaceColor',robot.color,'EdgeColor','none');

% In top view, draw robot as a small dot
robot_top = plot(axTop,robot.pos(1),robot.pos(2),'ro','MarkerSize',6,'MarkerFaceColor','r');

%% --- Virtual front camera parameters (view only) ---
cam_height   = 0.25;   % camera height
cam_forward  = -0.08;  % negative = camera slightly behind robot centre
cam_fov      = 70;     % visual field of view

%% --- Motion & steering parameters ---
dt            = 0.05;
speed         = 0.12;      % m/s
max_time      = 400;
corner_tol    = 0.03;      % distance to waypoint to switch
k_turn        = 4.0;       % steering gain
max_turn_rate = 140;       % deg/s max steering rate

%% --- Wiggle parameters (whole robot shaking) ---
wiggle_xy_amp  = 0.01;   % meters (side-to-side & forward-back wobble)
wiggle_rot_amp = 2.0;    % degrees (small yaw wobble)

disp('Starting WRO simulation: rectangular path with car-like turns + wiggle (ACW).');

%% --- Main loop ---
for t = 0:dt:max_time

    %% --- Path following: target current waypoint ---
    target = waypoints(wp_idx,:);
    vec    = target - robot.pos(1:2);
    dist   = norm(vec);

    % if close to current waypoint, go to next one
    if dist < corner_tol
        wp_idx = wp_idx + 1;
        if wp_idx > num_wp
            wp_idx = 1;
        end
        target = waypoints(wp_idx,:);
        vec    = target - robot.pos(1:2);
        dist   = norm(vec);
    end

    desired_heading = atan2d(vec(2), vec(1));

    % --- Car-like steering: limited turn rate ---
    heading_error = wrapTo180(desired_heading - robot.heading);
    turn_cmd      = k_turn * heading_error;            % deg/s "command"
    turn_cmd      = max(-max_turn_rate, min(max_turn_rate, turn_cmd));
    robot.heading = robot.heading + turn_cmd * dt;

    %% --- Move robot forward ---
    move_dist = speed * dt;
    if move_dist > dist
        move_dist = dist;  % avoid huge overshoot at corners
    end

    robot.pos(1) = robot.pos(1) + move_dist*cosd(robot.heading);
    robot.pos(2) = robot.pos(2) + move_dist*sind(robot.heading);

    %% --- Safety: keep inside outer arena ---
    margin = 0.05;
    if robot.pos(1) < margin
        robot.pos(1) = margin;
    elseif robot.pos(1) > plane_dims(1)-margin
        robot.pos(1) = plane_dims(1)-margin;
    end
    if robot.pos(2) < margin
        robot.pos(2) = margin;
    elseif robot.pos(2) > plane_dims(2)-margin
        robot.pos(2) = plane_dims(2)-margin;
    end

    %% --- Wiggle: apply small random offset to pos + heading for rendering ---
    wig_dx   = wiggle_xy_amp  * randn();
    wig_dy   = wiggle_xy_amp  * randn();
    wig_rot  = wiggle_rot_amp * randn();

    render_pos     = [robot.pos(1) + wig_dx, robot.pos(2) + wig_dy, robot.pos(3)];
    render_heading = robot.heading + wig_rot;

    %% --- Update robot meshes (front + top views) using render pose ---
    [XrF,YrF,ZrF] = makeCube(robot.size,render_pos,render_heading);
    set(robot.patchFront,'XData',XrF,'YData',YrF,'ZData',ZrF);
    set(robot_top,'XData',render_pos(1),'YData',render_pos(2));

    %% --- Update MATLAB camera to robot FRONT / CHASE view (also wiggled) ---
    fwd2D = [cosd(render_heading), sind(render_heading)];

    camPosFront = [ ...
        render_pos(1) + cam_forward * fwd2D(1), ...
        render_pos(2) + cam_forward * fwd2D(2), ...
        cam_height];

    look_dist = 1.2;  % look ahead
    target_xy = [ ...
        render_pos(1) + look_dist * fwd2D(1), ...
        render_pos(2) + look_dist * fwd2D(2)];
    camTargetFront = [target_xy(1), target_xy(2), 0];

    campos(axFront, camPosFront);
    camtarget(axFront, camTargetFront);
    camup(axFront, [0 0 1]);
    camproj(axFront,'perspective');
    camva(axFront, cam_fov);

    drawnow;
end

fprintf('Simulation finished at t = %.1f s.\n', t);
fprintf('Robot final position: (%.3f, %.3f)\n', robot.pos(1), robot.pos(2));

%% ================= LOCAL FUNCTIONS =================

function a = wrapTo180(a)
    a = mod(a+180,360) - 180;
end

function drawCuboid(ax,pos,sz,col)
    x = pos(1); y = pos(2); z = pos(3);
    w = sz(1);  l = sz(2);  h = sz(3);
    v = [x,y,z;
         x+w,y,z;
         x+w,y+l,z;
         x,y+l,z;
         x,y,z+h;
         x+w,y,z+h;
         x+w,y+l,z+h;
         x,y+l,z+h];
    f = [1 2 3 4;
         5 6 7 8;
         1 2 6 5;
         2 3 7 6;
         3 4 8 7;
         4 1 5 8];
    patch(ax,'Vertices',v,'Faces',f,'FaceColor',col,'EdgeColor','none');
end

function [X,Y,Z] = makeCube(sz,pos,hd)
    L = sz/2;
    pts = [-L -L 0;
            L -L 0;
            L  L 0;
           -L  L 0;
           -L -L L;
            L -L L;
            L  L L;
           -L  L L];
    R = [cosd(hd*pi/180) -sind(hd*pi/180) 0;
         sind(hd*pi/180)  cosd(hd*pi/180) 0;
         0                0               1];
    pts = (R * pts')';
    pts = pts + pos;
    f = [1 2 3 4;
         5 6 7 8;
         1 2 6 5;
         2 3 7 6;
         3 4 8 7;
         4 1 5 8];
    X = reshape(pts(f',1),4,6);
    Y = reshape(pts(f',2),4,6);
    Z = reshape(pts(f',3),4,6);
end
