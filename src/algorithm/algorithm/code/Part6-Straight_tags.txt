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

