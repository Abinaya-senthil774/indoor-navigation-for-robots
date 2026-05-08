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
