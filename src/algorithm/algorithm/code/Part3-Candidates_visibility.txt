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
