%% ====================================================================
%% PART 2 — Map & Geometry Generation
%% ====================================================================
fprintf('--- PART 2: GEOMETRY GENERATION ---\n');

ds_map = 0.01; % Resolution

% --- 1. DEFINE DETAILED MAP (2x2m Total Area) ---
% 1. Outer Walls (2m x 2m box: -1 to +1)
x_out = -1:ds_map:1; 
y_out = -1:ds_map:1;

OuterTop    = [x_out',  1*ones(length(x_out),1)];
OuterBottom = [x_out', -1*ones(length(x_out),1)];
OuterLeft   = [-1*ones(length(y_out),1), y_out'];
OuterRight  = [ 1*ones(length(y_out),1), y_out'];

% 2. Inner Walls Calculation
% Goal: Create specific corridor widths
% Top/Bottom Corridor Width target: 0.47m -> Inner Y = 1.0 - 0.47 = 0.53
% Left/Right Corridor Width target: 0.43m -> Inner X = 1.0 - 0.43 = 0.57
inner_y_limit = 0.53; 
inner_x_limit = 0.57; 

x_in = -inner_x_limit:ds_map:inner_x_limit;

InnerTop    = [x_in',  inner_y_limit*ones(length(x_in),1)];
InnerBottom = [x_in', -inner_y_limit*ones(length(x_in),1)];

% 3. Vertical Walls (Left/Right internal structures)
% Notches scaled for 2x2 map
y_seg1 = inner_y_limit:-ds_map:0.3;
y_seg2 = 0.2:-ds_map:-0.2;
y_seg3 = -0.3:-ds_map:-inner_y_limit;

LeftWall = [
    [-inner_x_limit*ones(length(y_seg1),1), y_seg1'];           
    make_notch_geometry(-inner_x_limit, -0.4, 0.3, 0.1, ds_map); % Scaled notch
    [-inner_x_limit*ones(length(y_seg2),1), y_seg2'];           
    make_notch_geometry(-inner_x_limit, -0.4, -0.2, 0.1, ds_map); % Scaled notch
    [-inner_x_limit*ones(length(y_seg3),1), y_seg3'];           
];

RightWall = [
    [inner_x_limit*ones(length(y_seg1),1), y_seg1'];            
    make_notch_geometry(inner_x_limit, 0.4, 0.3, 0.1, ds_map);  
    [inner_x_limit*ones(length(y_seg2),1), y_seg2'];            
    make_notch_geometry(inner_x_limit, 0.4, -0.2, 0.1, ds_map); 
    [inner_x_limit*ones(length(y_seg3),1), y_seg3'];            
];

MapPoints = [OuterTop; OuterBottom; OuterLeft; OuterRight; ...
             InnerTop; InnerBottom; LeftWall; RightWall];

% Plot to verify
figure;
plot(MapPoints(:,1), MapPoints(:,2), '.k');
axis equal;
title('2x2m Environment Map');
grid on;

% ---------------- GEOMETRY DEFINITIONS (SCALED) ----------------
% Path magnitude (must be inside the corridor, approx 0.8m)
path_X_mag = 0.8; 
path_Y_mag = 0.75; 

% Corridor Widths (Outer - Inner) for calc
width_H = 1.0 - inner_y_limit; % 0.47m
width_V = 1.0 - inner_x_limit; % 0.43m

% --- CONSTRAINT CALCULATION FOR JUNCTIONS ---
opt_dist_H = width_H / (2 * tan(hfov/2));
opt_dist_V = width_V / (2 * tan(hfov/2));

target_outer_H = opt_dist_H + 0.1; % Reduced offset for smaller map
target_outer_V = opt_dist_V + 0.1;
target_inner_H = target_outer_H * 0.6; 
target_inner_V = target_outer_V * 0.6; 

% --- 2. DEFINE ROBOT PATH (Red Centerline) ---
ds_path = 0.05; % Finer step for smaller map
P_Top = [(-path_X_mag:ds_path:path_X_mag)', path_Y_mag*ones(length(-path_X_mag:ds_path:path_X_mag),1)];
P_Right = [path_X_mag*ones(length(-path_Y_mag:ds_path:path_Y_mag),1), (path_Y_mag:-ds_path:-path_Y_mag)'];
P_Bot = [(path_X_mag:-ds_path:-path_X_mag)', -path_Y_mag*ones(length(path_X_mag:-ds_path:-path_X_mag),1)];
P_Left = [-path_X_mag*ones(length(-path_Y_mag:ds_path:path_Y_mag),1), (-path_Y_mag:ds_path:path_Y_mag)'];

poses = [P_Top; P_Right; P_Bot; P_Left];
Np = size(poses,1);

% Path Distance
distVals = zeros(Np,1); currentD = 0;
for i = 2:Np, step=norm(poses(i,:)-poses(i-1,:)); if step>ds_path*2, step=0; end; currentD=currentD+step; distVals(i)=currentD; end

% --- 3. GENERATE CANDIDATES (Gap-Aware) ---
candidateSpacing = 0.10; % Finer spacing for small map
C_list = []; O_list = []; W_type = [];

% A. Outer Walls (Type 1) - Coords +/- 1.0
xs = -1.0:candidateSpacing:1.0;
C_list = [C_list; [xs', 1.0*ones(length(xs),1)]]; O_list = [O_list; -90*ones(length(xs),1)]; W_type=[W_type; ones(length(xs),1)];
C_list = [C_list; [xs', -1.0*ones(length(xs),1)]]; O_list = [O_list; 90*ones(length(xs),1)]; W_type=[W_type; ones(length(xs),1)];

ys = -1.0:candidateSpacing:1.0;
C_list = [C_list; [-1.0*ones(length(ys),1), ys']]; O_list = [O_list; 0*ones(length(ys),1)]; W_type=[W_type; ones(length(ys),1)];
C_list = [C_list; [1.0*ones(length(ys),1), ys']]; O_list = [O_list; 180*ones(length(ys),1)]; W_type=[W_type; ones(length(ys),1)];

% B. Inner Top/Bot (Type 2) - Coords +/- inner_y_limit
xs = -inner_x_limit:candidateSpacing:inner_x_limit;
C_list = [C_list; [xs', inner_y_limit*ones(length(xs),1)]]; O_list = [O_list; 90*ones(length(xs),1)]; W_type=[W_type; 2*ones(length(xs),1)];
C_list = [C_list; [xs', -inner_y_limit*ones(length(xs),1)]]; O_list = [O_list; -90*ones(length(xs),1)]; W_type=[W_type; 2*ones(length(xs),1)];

% C. Inner Side Walls (Skip Door Gaps) (Type 2)
% NOTE: Adjusted gap logic to match new map notches
% Notches are roughly at Y = 0.3 and Y = -0.2 with width 0.1
notch_width = 0.15; % buffer
door_gap_1 = [0.3 - notch_width, 0.3 + notch_width]; 
door_gap_2 = [-0.2 - notch_width, -0.2 + notch_width];

ys = -inner_y_limit:candidateSpacing:inner_y_limit;
valid_y = [];
for y = ys
    if ~((y > door_gap_1(1) && y < door_gap_1(2)) || (y > door_gap_2(1) && y < door_gap_2(2)))
        valid_y = [valid_y; y];
    end
end
C_list = [C_list; [inner_x_limit*ones(length(valid_y),1), valid_y]]; O_list = [O_list; 0*ones(length(valid_y),1)]; W_type=[W_type; 2*ones(length(valid_y),1)];
C_list = [C_list; [-inner_x_limit*ones(length(valid_y),1), valid_y]]; O_list = [O_list; 180*ones(length(valid_y),1)]; W_type=[W_type; 2*ones(length(valid_y),1)];

C = C_list; Orientations = O_list; WallTypes = W_type; Nc = size(C,1);
InitialCount = Nc;
fprintf('Initial Candidates Generated: %d\n', InitialCount);

% --- VISIBILITY MATRIX ---
fprintf('Calculating Visibility Matrix... ');
V = zeros(Np, Nc);
for i = 1:Np
    robot_pos = poses(i,:); 
    % Adjust Yaw logic for 2x2 map (turn happens around 0.8)
    if abs(robot_pos(1)) > (path_X_mag - 0.1), robotYaw = pi/2; else, robotYaw = 0; end 
    
    for j = 1:Nc
        cj = C(j,:); 
        v_vec = cj - robot_pos; 
        dist = norm(v_vec);
        if dist <= dmax && dist >= 0.05
            angleGlobal = atan2(v_vec(2), v_vec(1));
            bearing = mod(angleGlobal - robotYaw + pi, 2*pi) - pi;
            
            nx = cosd(Orientations(j)); ny = sind(Orientations(j));
            v_tag_to_bot = robot_pos - cj;
            dotProd = nx*v_tag_to_bot(1) + ny*v_tag_to_bot(2);
            
            if abs(bearing) <= hfov/2 && dotProd > 0, V(i,j) = 1; end
        end
    end
end
fprintf('Done.\n');
