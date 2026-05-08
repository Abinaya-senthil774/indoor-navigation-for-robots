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
