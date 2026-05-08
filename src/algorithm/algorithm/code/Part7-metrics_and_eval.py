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
