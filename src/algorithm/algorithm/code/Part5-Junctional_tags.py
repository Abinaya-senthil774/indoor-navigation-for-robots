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
