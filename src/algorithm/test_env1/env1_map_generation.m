%% ====================================================================
%% PART 2 — Map Generation & Tag Placement
%% ====================================================================
fprintf('\n--- PART 2: MAP & PLACEMENT ---\n');

% ---------------- MAP & POSE GENERATION ----------------
ds = 0.4; 
y_vert = -2:ds:2;
P1 = [zeros(length(y_vert),1), y_vert']; % Vertical corridor (Main)
x_horz = -1.5:ds:1.5;
P2 = [x_horz', 1.25*ones(length(x_horz),1)];  % Top horizontal (Inner)
P3 = [x_horz', -1.25*ones(length(x_horz),1)]; % Bottom horizontal (Inner)
poses = [P1; P2; P3];
Np = size(poses,1);

% ---------------- TAG CANDIDATE GENERATION ----------------
% Wall offset uses the constant defined in Part 1
C = generateTagCandidates(dmax, wallOffset);
Nc = size(C,1);

