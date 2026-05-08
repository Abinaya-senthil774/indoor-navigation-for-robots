%% FINAL INTEGRATED SYSTEM: Feasibility Analysis + Multi-Stage Optimization + Analytics
% Features:
% 1. PRE-PROCESS: Tag Feasibility Algorithm (Selects Size & Thresh).
% 2. MAP: Functional Doors, Junctions (Same-Side), Straight (Zig-Zag).
% 3. ANALYTICS: Intermediate Plots & Metrics after EVERY stage.
% 4. FINAL OUTPUT: Efficiency Plot, Blackout Plot, CSV Export.

clear; clc; close all;

%% ====================================================================
%% PART 1 — Tag Feasibility Analysis (Algorithm 1)
%% ====================================================================
fprintf('--- PART 1: TAG FEASIBILITY ANALYSIS ---\n');

% ---------------- INPUTS ----------------
fx = 219.6585; fy = 219.6585; cx = 120; cy = 170;
f = min(fx, fy);
K = [fx, fy, cx, cy];

% Candidate configurations
tagSizes = [0.10, 0.15, 0.20];       % Meters
pixelThreshList = [30, 40, 50];      % Pixels
hfov = deg2rad(82);

% Environment Constraints
maxCorridorWidth = 2.0;              % Max width between walls
maxWallOffset = 1.0;                 % Approx max dist from robot centerline to wall

feasibleConfigs = [];

% --- PRINT TABLE HEADER ---
fprintf('\n%s\n', repmat('=',1,70));
fprintf('| %-8s | %-8s | %-10s | %-15s | %-8s |\n', 'Tag(m)', 'PxThr', 'MaxDist(m)', 'Status', 'Score');
fprintf('%s\n', repmat('-',1,70));

bestScore = -inf;
bestConfig = [];

for s = tagSizes
    for pmin = pixelThreshList
        
        % Step 1: Compute max detection distance
        d_calc = (f * s) / pmin;
        
        % Status Check
        status = 'OK';
        isFeasible = true;
        
        % Step 2: Check lateral visibility constraint (Wall Offset)
        if maxWallOffset > d_calc
            status = 'Fail:TooShort';
            isFeasible = false;
        end
        
        % Step 3: Check corridor coverage (Corridor Width)
        if maxCorridorWidth > 2 * d_calc
            status = 'Fail:Width';
            isFeasible = false;
        end
        	
        % Step 4: Scoring (Heuristic)
        score = 0;
        if isFeasible
             % Penalize distance from 40px threshold (sweet spot)
            score = score - abs(pmin - 40) * 10; 
            % Reward Distance
            score = score + d_calc * 5;
            % Penalize large tags slightly (cost/aesthetics)
            score = score - s * 2;
            
            feasibleConfigs = [feasibleConfigs; s, pmin, d_calc, score];
            
            if score > bestScore
                bestScore = score;
                bestConfig = [s, pmin, d_calc];
            end
        else
            score = NaN; % No score for failed configs
        end
        
        % Print Row
        if isFeasible
             fprintf('| %-8.2f | %-8d | %-10.2f | %-15s | %-8.1f |\n', s, pmin, d_calc, status, score);
        else
             fprintf('| %-8.2f | %-8d | %-10.2f | %-15s | %-8s |\n', s, pmin, d_calc, status, 'N/A');
        end
    end
end
fprintf('%s\n', repmat('=',1,70));

if isempty(bestConfig)
    error('No feasible tag configurations found!');
end

% Set Global Parameters based on Selection
tagSize = bestConfig(1);
pixelThresh = bestConfig(2);
dmax = bestConfig(3);

fprintf('\n>> OPTIMAL CONFIG SELECTED (Highest Score):\n');
fprintf('   Tag Size:     %.2f m\n', tagSize);
fprintf('   Pixel Thresh: %d px\n', pixelThresh);
fprintf('   Max Dist:     %.2f m\n\n', dmax);

% ---------------- GEOMETRY DEFINITIONS ----------------
path_X_mag = 1.75;
path_Y_mag = 1.625;

% Corridor Widths (Outer - Inner)
width_H = 2.0 - 1.25; % 0.75m
width_V = 2.0 - 1.50; % 0.50m

% --- CONSTRAINT CALCULATION FOR JUNCTIONS ---
opt_dist_H = width_H / (2 * tan(hfov/2));
opt_dist_V = width_V / (2 * tan(hfov/2));
target_outer_H = opt_dist_H + 0.2;
target_outer_V = opt_dist_V + 0.2;
target_inner_H = target_outer_H * 0.6; 
target_inner_V = target_outer_V * 0.6; 

%% ====================================================================
%% PART 2 — Map & Geometry Generation
%% ====================================================================
fprintf('--- PART 2: GEOMETRY GENERATION ---\n');
ds_map = 0.01; % Resolution

% --- 1. DEFINE DETAILED MAP ---
x_out = -2:ds_map:2; y_out = -2:ds_map:2;
OuterTop    = [x_out',  2*ones(length(x_out),1)];
OuterBottom = [x_out', -2*ones(length(x_out),1)];
OuterLeft   = [-2*ones(length(y_out),1), y_out'];
OuterRight  = [ 2*ones(length(y_out),1), y_out'];

x_in = -1.5:ds_map:1.5;
InnerTop    = [x_in',  1.25*ones(length(x_in),1)];
InnerBottom = [x_in', -1.25*ones(length(x_in),1)];

y_seg1 = 1.25:-ds_map:0.615;
y_seg2 = 0.385:-ds_map:-0.385;
y_seg3 = -0.615:-ds_map:-1.25;

LeftWall = [
    [-1.5*ones(length(y_seg1),1), y_seg1'];           
    make_notch_geometry(-1.5, -1.0, 0.615, 0.23, ds_map);
    [-1.5*ones(length(y_seg2),1), y_seg2'];           
    make_notch_geometry(-1.5, -1.0, -0.385, 0.23, ds_map);
    [-1.5*ones(length(y_seg3),1), y_seg3'];           
];

RightWall = [
    [1.5*ones(length(y_seg1),1), y_seg1'];            
    make_notch_geometry(1.5, 1.0, 0.615, 0.23, ds_map);  
    [1.5*ones(length(y_seg2),1), y_seg2'];            
    make_notch_geometry(1.5, 1.0, -0.385, 0.23, ds_map); 
    [1.5*ones(length(y_seg3),1), y_seg3'];            
];

MapPoints = [OuterTop; OuterBottom; OuterLeft; OuterRight; ...
             InnerTop; InnerBottom; LeftWall; RightWall];

% --- 2. DEFINE ROBOT PATH (Red Centerline) ---
ds_path = 0.1;
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
candidateSpacing = 0.15; 
C_list = []; O_list = []; W_type = [];

% A. Outer Walls (Type 1)
xs = -2:candidateSpacing:2;
C_list = [C_list; [xs', 2.0*ones(length(xs),1)]]; O_list = [O_list; -90*ones(length(xs),1)]; W_type=[W_type; ones(length(xs),1)];
C_list = [C_list; [xs', -2.0*ones(length(xs),1)]]; O_list = [O_list; 90*ones(length(xs),1)]; W_type=[W_type; ones(length(xs),1)];

ys = -2:candidateSpacing:2;
C_list = [C_list; [-2.0*ones(length(ys),1), ys']]; O_list = [O_list; 0*ones(length(ys),1)]; W_type=[W_type; ones(length(ys),1)];
C_list = [C_list; [2.0*ones(length(ys),1), ys']]; O_list = [O_list; 180*ones(length(ys),1)]; W_type=[W_type; ones(length(ys),1)];

% B. Inner Top/Bot (Type 2)
xs = -1.5:candidateSpacing:1.5;
C_list = [C_list; [xs', 1.25*ones(length(xs),1)]]; O_list = [O_list; 90*ones(length(xs),1)]; W_type=[W_type; 2*ones(length(xs),1)];
C_list = [C_list; [xs', -1.25*ones(length(xs),1)]]; O_list = [O_list; -90*ones(length(xs),1)]; W_type=[W_type; 2*ones(length(xs),1)];

% C. Inner Side Walls (Skip Door Gaps) (Type 2)
door_gap_1 = [0.385, 0.615]; door_gap_2 = [-0.615, -0.385];
ys = -1.25:candidateSpacing:1.25;
valid_y = [];
for y = ys
    if ~((y > door_gap_1(1) && y < door_gap_1(2)) || (y > door_gap_2(1) && y < door_gap_2(2)))
        valid_y = [valid_y; y];
    end
end
C_list = [C_list; [1.5*ones(length(valid_y),1), valid_y]]; O_list = [O_list; 0*ones(length(valid_y),1)]; W_type=[W_type; 2*ones(length(valid_y),1)];
C_list = [C_list; [-1.5*ones(length(valid_y),1), valid_y]]; O_list = [O_list; 180*ones(length(valid_y),1)]; W_type=[W_type; 2*ones(length(valid_y),1)];

C = C_list; Orientations = O_list; WallTypes = W_type; Nc = size(C,1);
InitialCount = Nc;
fprintf('Initial Candidates Generated: %d\n', InitialCount);

% --- VISIBILITY MATRIX ---
fprintf('Calculating Visibility Matrix... ');
V = zeros(Np, Nc);
for i = 1:Np
    robot_pos = poses(i,:); 
    if abs(robot_pos(1)) > 1.65, robotYaw = pi/2; else, robotYaw = 0; end 
    
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

%% ====================================================================
%% PART 3 — Optimization Stages with INTERMEDIATE PLOTS
%% ====================================================================
fprintf('\n--- PART 3: OPTIMIZATION ---\n');
selectedIndices = [];
coveredPoses = zeros(Np,1);
plotColors = {'b', 'm', 'c'}; 
StageData = []; % For Efficiency Plot later [MeanVisible, Availability]

% =================================================================
% STAGE 1: FUNCTIONAL (Near Doors)
% =================================================================
fprintf('\n>>> STAGE 1: FUNCTIONAL TAGS <<<\n');
DoorTargets = [-1.5, 0.615; 1.5, 0.615; -1.5, -0.385; 1.5, -0.385]; 
Tfunc = [];
for d = 1:size(DoorTargets,1)
    dists = vecnorm(C - DoorTargets(d,:), 2, 2);
    [minD, idx] = min(dists);
    if minD < 0.6, Tfunc = [Tfunc; idx]; end
end
Tfunc = unique(Tfunc);
selectedIndices = [selectedIndices; Tfunc];

% --- METRICS & PLOT STAGE 1 ---
[avail, reduction, blackout, meanTags] = print_segment_metrics(V, selectedIndices, Nc, Np, ds_path);
StageData = [StageData; meanTags, avail];

figure('Name', 'Stage 1: Functional');
plot(MapPoints(:,1), MapPoints(:,2), 'k.', 'MarkerSize', 1); hold on;
plot(poses(:,1), poses(:,2), 'Color', [0.8 0.8 0.8]);
plot(C(Tfunc,1), C(Tfunc,2), 'bo', 'MarkerFaceColor','b', 'MarkerSize', 8);
axis equal; grid on; xlim([-2.5 2.5]); ylim([-2.5 2.5]);
title(['Stage 1: Functional (Avail: ' num2str(avail, '%.1f') '%)']);

% =================================================================
% STAGE 2: JUNCTIONS (2 Before + 2 After)
% =================================================================
fprintf('\n>>> STAGE 2: JUNCTION TAGS <<<\n');
Corners = [1.75, 1.625; -1.75, 1.625; 1.75, -1.625; -1.75, -1.625];
Tjunc = [];
for c = 1:size(Corners,1)
    cx = Corners(c,1); cy = Corners(c,2);
    
    % --- HORIZONTAL LEG ---
    idxH = find( (abs(Orientations) == 90) & abs(C(:,2)-cy) < 0.8 & abs(C(:,1)-cx) < 2.0 );
    distsH = vecnorm(C(idxH,:) - [cx, cy], 2, 2);
    [~, bestOuterH_idx] = min(abs(distsH - target_outer_H));
    bestH_Tag = idxH(bestOuterH_idx);
    
    sameSideH = find(Orientations(idxH) == Orientations(bestH_Tag));
    subsetH = idxH(sameSideH);
    distsH_sub = vecnorm(C(subsetH,:) - [cx, cy], 2, 2);
    
    [~, i1] = min(abs(distsH_sub - target_outer_H));
    [~, i2] = min(abs(distsH_sub - target_inner_H));
    Tjunc = [Tjunc; subsetH(i1); subsetH(i2)];
    
    % --- VERTICAL LEG ---
    idxV = find( (abs(Orientations) == 0 | abs(Orientations) == 180) & abs(C(:,1)-cx) < 0.8 & abs(C(:,2)-cy) < 2.0 );
    distsV = vecnorm(C(idxV,:) - [cx, cy], 2, 2);
    [~, bestOuterV_idx] = min(abs(distsV - target_outer_V));
    bestV_Tag = idxV(bestOuterV_idx);
    
    sameSideV = find(Orientations(idxV) == Orientations(bestV_Tag));
    subsetV = idxV(sameSideV);
    distsV_sub = vecnorm(C(subsetV,:) - [cx, cy], 2, 2);
    
    [~, i3] = min(abs(distsV_sub - target_outer_V));
    [~, i4] = min(abs(distsV_sub - target_inner_V));
    Tjunc = [Tjunc; subsetV(i3); subsetV(i4)];
end
Tjunc = unique(Tjunc);
Tjunc = setdiff(Tjunc, selectedIndices); 
selectedIndices = [selectedIndices; Tjunc];

% --- METRICS & PLOT STAGE 2 ---
[avail, reduction, blackout, meanTags] = print_segment_metrics(V, selectedIndices, Nc, Np, ds_path);
StageData = [StageData; meanTags, avail];

figure('Name', 'Stage 2: Junctions');
plot(MapPoints(:,1), MapPoints(:,2), 'k.', 'MarkerSize', 1); hold on;
plot(poses(:,1), poses(:,2), 'Color', [0.8 0.8 0.8]);
plot(C(Tfunc,1), C(Tfunc,2), 'bo', 'MarkerFaceColor','b');
plot(C(Tjunc,1), C(Tjunc,2), 'ms', 'MarkerFaceColor','m', 'MarkerSize', 8);
axis equal; grid on; xlim([-2.5 2.5]); ylim([-2.5 2.5]);
title(['Stage 2: Junctions (Avail: ' num2str(avail, '%.1f') '%)']);

% =================================================================
% STAGE 3: STRAIGHT (Zig-Zag Optimized)
% =================================================================
% =================================================================
% STAGE 3: STRAIGHT (Zig-Zag Optimized) + ITERATION ANALYTICS
% =================================================================
fprintf('\n>>> STAGE 3: STRAIGHT/ZIG-ZAG TAGS <<<\n');

Tstraight = [];
candLeft = setdiff(1:Nc, selectedIndices);
SameSidePenaltyWeight = 100;

% --- INITIAL COVERAGE ---
visCounts = sum(V(:,selectedIndices), 2); 
coveredPoses = visCounts > 0;

% --- ITERATION LOGGING ---
iter = 0;
coverageHistory = [];
candidateCountHistory = [];
tagCountHistory = [];

fprintf('\nIter | Coverage(%%) | Uncovered | Candidates | Total Tags\n');
fprintf('----------------------------------------------------------\n');

while any(~coveredPoses)
    iter = iter + 1;

    bestScore = -inf;
    bestTag = -1;

    % --- LOG NUMBER OF CANDIDATES ANALYZED THIS ITERATION ---
    candidateCountHistory(iter) = length(candLeft);

    for k = 1:length(candLeft)
        t = candLeft(k);

        % --- NEW COVERAGE CONTRIBUTION ---
        newCov = sum((1-coveredPoses) .* V(:,t));

        % --- SAME-SIDE PENALTY ---
        dists = vecnorm(C(selectedIndices,:) - C(t,:), 2, 2);
        [minDist, closestIdx] = min(dists);
        neighborTag = selectedIndices(closestIdx);

        penalty = 0;
        if minDist < 3.0 && WallTypes(t) == WallTypes(neighborTag)
            penalty = SameSidePenaltyWeight;
        end

        score = newCov - penalty;

        if score > bestScore
            bestScore = score;
            bestTag = t;
        end
    end

    % --- TERMINATION CONDITIONS ---
    if bestScore <= -50
        fprintf('Stopping: score too low (no useful candidates left)\n');
        break;
    end

    if sum((1-coveredPoses) .* V(:,bestTag)) == 0
        fprintf('Stopping: no candidate adds new coverage\n');
        break;
    end

    % --- ACCEPT BEST TAG ---
    Tstraight = [Tstraight; bestTag];
    selectedIndices = [selectedIndices; bestTag];
    coveredPoses = coveredPoses | V(:,bestTag);
    candLeft = candLeft(candLeft ~= bestTag);

    % --- LOG METRICS ---
    coveragePct = sum(coveredPoses) / Np * 100;
    coverageHistory(iter) = coveragePct;
    tagCountHistory(iter) = length(selectedIndices);

    % --- PRINT ITERATION SUMMARY ---
    fprintf('%4d | %10.2f | %9d | %10d | %10d\n', ...
            iter, ...
            coveragePct, ...
            sum(~coveredPoses), ...
            candidateCountHistory(iter), ...
            tagCountHistory(iter));
end

fprintf('\nStage 3 completed in %d iterations.\n', iter);
fprintf('Straight tags added: %d\n', length(Tstraight));
fprintf('Final coverage: %.2f %%\n', coverageHistory(end));
figure('Name','Stage 3 Coverage Progress');
plot(coverageHistory,'-o','LineWidth',1.5,'MarkerSize',6);
xlabel('Iteration');
ylabel('Localization Availability (%)');
title('Coverage Improvement During Stage 3');
grid on;
ylim([0 105]);
figure('Name','Stage 3 Candidate Analysis');
plot(candidateCountHistory,'-s','LineWidth',1.5,'MarkerSize',6);
xlabel('Iteration');
ylabel('Candidates Evaluated');
title('Candidate Pool Size per Iteration');
grid on;
figure('Name','Stage 3 Optimization Dynamics');
yyaxis left
plot(coverageHistory,'-o','LineWidth',1.5);
ylabel('Coverage (%)');
ylim([0 105]);

yyaxis right
plot(candidateCountHistory,'-s','LineWidth',1.5);
ylabel('Candidates Evaluated');

xlabel('Iteration');
title('Stage 3: Coverage vs Candidate Pool');
grid on;


% --- METRICS & PLOT STAGE 3 ---
[avail, reduction, blackout, meanTags] = print_segment_metrics(V, selectedIndices, Nc, Np, ds_path);
StageData = [StageData; meanTags, avail];

figure('Name', 'Stage 3: Straight (Final)');
plot(MapPoints(:,1), MapPoints(:,2), 'k.', 'MarkerSize', 1); hold on;
plot(poses(:,1), poses(:,2), 'Color', [0.8 0.8 0.8]);
plot(C(Tfunc,1), C(Tfunc,2), 'bo', 'MarkerFaceColor','b');
plot(C(Tjunc,1), C(Tjunc,2), 'ms', 'MarkerFaceColor','m');
plot(C(Tstraight,1), C(Tstraight,2), 'c^', 'MarkerFaceColor','c', 'MarkerSize', 8);
axis equal; grid on; xlim([-2.5 2.5]); ylim([-2.5 2.5]);
title(['Stage 3: Final (Avail: ' num2str(avail, '%.1f') '%)']);


%% ====================================================================
%% PART 4 — FINAL METRICS & EXPORT
%% ====================================================================
fprintf('\n=== FINAL SUMMARY METRICS ===\n');
[final_avail, final_red, final_blackout, final_mean] = print_segment_metrics(V, selectedIndices, Nc, Np, ds_path);

% --- PLOT 4: EFFICIENCY ANALYSIS (Bubble Plot) ---
% Matches the logic: X=Mean Visible Tags, Y=Localization Availability
figure('Name', 'Efficiency of Tag Placement');
plot(StageData(:,1), StageData(:,2), 'b.', 'MarkerSize', 25); hold on;
grid on;
labels = {'Optimal_Case1 (Func)', 'Optimal_Case2 (Junc)', 'Optimal_Case3 (Straight)'};
text(StageData(:,1)-0.05, StageData(:,2)+1.5, labels, 'FontSize', 9);
xlabel('Mean visible tags per frame');
ylabel('Localization availability (%)');
title('Efficiency of tag placement');
xlim([0, max(StageData(:,1))+0.5]); ylim([0, 105]);

% --- PLOT 5: BLACKOUT ANALYSIS ---
% Visualizes where the robot has 0 tags vs >0 tags
visCountsFinal = sum(V(:,selectedIndices), 2);
figure('Name', 'Localization Availability & Blackouts');
subplot(2,1,1);
plot(distVals, visCountsFinal, 'LineWidth', 1.5);
xlabel('Path Distance (m)'); ylabel('Visible Tags');
title('Number of Visible Tags along Path'); grid on;
subplot(2,1,2);
area(distVals, double(visCountsFinal>0), 'FaceColor', 'g', 'FaceAlpha', 0.3);
xlabel('Path Distance (m)'); ylabel('Available? (0/1)');
title('Binary Localization Availability'); ylim([-0.2 1.2]); grid on;

% --- EXPORT ---
FinalC = C(selectedIndices, :); 
FinalO = Orientations(selectedIndices); 

% CRITICAL FIX: Webots "Box" + "Image Texture" orientation fix
% 0 deg in Math = East (+X).
% 0 deg in Webots (with 90deg X-tilt) = South (-Y).
% Correction: Add 90 degrees to Math Angle to get Webots Angle.
FinalO_Webots = FinalO + 90;
% Normalize to [-180, 180]
FinalO_Webots = mod(FinalO_Webots + 180, 360) - 180;

FinalType = cell(length(selectedIndices),1);
for i=1:length(selectedIndices)
    idx = selectedIndices(i);
    if ismember(idx, Tfunc), FinalType{i}='Functional';
    elseif ismember(idx, Tjunc), FinalType{i}='Junction';
    else, FinalType{i}='Straight';
    end
end

T = table((1:length(selectedIndices))', FinalType, FinalC(:,1), FinalC(:,2), ...
          repmat(1.5, length(selectedIndices),1), FinalO_Webots, ...
          'VariableNames', {'ID','Type','X','Y','Z','Yaw_deg'});
          
writetable(T, 'aruco_tags.csv');
fprintf('\nData exported to aruco_tags.csv with +90 deg Webots correction.\n');

%% ====================================================================
%% FUNCTIONS
%% ====================================================================
function P = make_notch_geometry(x_wall, x_depth, y_top, width, ds)
    y_bot = y_top - width;
    if x_wall > x_depth, x_h = x_wall:-ds:x_depth; else, x_h = x_wall:ds:x_depth; end
    y_v = y_top:-ds:y_bot;
    TopEdge = [x_h', y_top * ones(length(x_h),1)];
    BackEdge = [x_depth * ones(length(y_v),1), y_v'];
    BotEdge = [flip(x_h)', y_bot * ones(length(x_h),1)];
    P = [TopEdge; BackEdge; BotEdge];
end

function [pct_avail, pct_reduction, max_blackout, mean_vis_all] = print_segment_metrics(V, selectedIdx, totalCand, Np, ds_path)
    % 1. Vis Counts
    if isempty(selectedIdx)
        visCounts = zeros(Np,1);
    else
        visCounts = sum(V(:,selectedIdx), 2);
    end
    
    coveredPoses = visCounts > 0;
    
    % 2. Calculate Metrics
    pct_used = (length(selectedIdx) / totalCand) * 100;
    pct_reduction = (1 - length(selectedIdx) / totalCand) * 100;
    pct_avail = sum(coveredPoses) / Np * 100;
    
    % 3. Blackout Calculation
    final_blackout_status = double(~coveredPoses);
    gaps = []; currGap = 0;
    for i = 1:Np
        if final_blackout_status(i)
            currGap = currGap + 1;
        elseif currGap > 0
            gaps = [gaps; currGap]; 
            currGap = 0; 
        end
    end
    if currGap > 0, gaps = [gaps; currGap]; end
    if ~isempty(gaps)
        max_blackout = max(gaps) * ds_path;
    else
        max_blackout = 0;
    end
    
    % 4. Mean Visible Tags (All Frames)
    mean_vis_all = mean(visCounts);
    
    % PRINT
    fprintf('   > Percentage Used:        %.1f%% (%d of %d)\n', pct_used, length(selectedIdx), totalCand);
    fprintf('   > Percentage Reduction:   %.1f%%\n', pct_reduction);
    fprintf('   > Localization Avail:     %.1f%%\n', pct_avail);
    fprintf('   > Max Blackout Length:    %.2f m\n', max_blackout);
end
