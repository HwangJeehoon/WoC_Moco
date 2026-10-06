%% Baseline vs propulsion comparison -- Run Current Section only
% 기존 최적화 결과만 읽어 figure를 생성한다. 최적화는 실행하지 않는다.
%
% Compared cases (all use effort=1, final-time=0):
%   1) propulsion=0: free-speed baseline
%   2) propulsion=1: free-speed propulsion solution (s=0.2)
%   3) propulsion=10: free-speed propulsion solution (s=0.2)
%   4) propulsion=10: s=0.2, time/distance locked to case 2

clc; close all;

thisScriptDir = fileparts(mfilename('fullpath'));
if isempty(thisScriptDir)
    if isfile(fullfile(pwd, 'plot_propulsion_baseline_w1_w10_comparison.m'))
        thisScriptDir = pwd;
    elseif isfile(fullfile(pwd, 'script', ...
            'plot_propulsion_baseline_w1_w10_comparison.m'))
        thisScriptDir = fullfile(pwd, 'script');
    else
        error(['Script 위치를 찾지 못했습니다. 작업 폴더를 repository root ', ...
               '또는 script 폴더로 설정하세요.']);
    end
end
resultsDir = fullfile(thisScriptDir, '..', 'results');

caseSpecs = struct( ...
    'resultName', { ...
        'propulsion_ft0_w0_baseline', ...
        'propulsion_ft0_w1', ...
        'propulsion_ft0_w10', ...
        'propulsion_ft0_w10_speedLockedToW1'}, ...
    'label', { ...
        'Baseline: propulsion=0, free speed', ...
        'Propulsion=1, free speed, s=0.2', ...
        'Propulsion=10, free speed, s=0.2', ...
        'Propulsion=10, speed locked to w=1, s=0.2'}, ...
    'shortLabel', {'w=0', 'w=1', 'w=10 free', 'w=10 locked'}, ...
    'color', { ...
        [0.35, 0.35, 0.35], ...
        [0.00, 0.45, 0.74], ...
        [0.85, 0.33, 0.10], ...
        [0.49, 0.18, 0.56]}, ...
    'lineStyle', {'--', '-', '-.', '-'});

numCases = numel(caseSpecs);
caseData = cell(numCases, 1);
for iCase = 1:numCases
    caseData{iCase} = readCaseResult(resultsDir, caseSpecs(iCase).resultName);
end

jointVariables = { ...
    '/jointset/hip_r/hip_flexion_r/value', ...
    '/jointset/knee_r/knee_angle_r/value', ...
    '/jointset/ankle_r/ankle_angle_r/value'};
jointLabels = {'Right hip angle', 'Right knee angle', ...
    'Right ankle angle'};
jointSigns = [1, -1, 1];

figure('Name', 'Baseline vs free/locked propulsion comparison', ...
    'Color', 'w', 'Units', 'normalized', 'Position', [0.03, 0.06, 0.94, 0.84]);
% 'loose' spacing keeps y-axis labels from being clipped between columns.
layout = tiledlayout(3, 3, 'TileSpacing', 'loose', 'Padding', 'compact');

% Column 1: right AP-GRF, right vGRF, and positive AP-GRF impulse.
axRightAP = nexttile(layout, 1); hold(axRightAP, 'on');
legendHandles = gobjects(numCases, 1);
for iCase = 1:numCases
    data = caseData{iCase};
    legendHandles(iCase) = plot(axRightAP, normalizeGaitTime(data.grf.time), ...
        data.grf.ground_force_r_vx, 'Color', caseSpecs(iCase).color, ...
        'LineStyle', caseSpecs(iCase).lineStyle, 'LineWidth', 1.8);
end
formatGrfAxis(axRightAP, 'Right AP-GRF', 'AP-GRF (N)');

axRightV = nexttile(layout, 4); hold(axRightV, 'on');
for iCase = 1:numCases
    data = caseData{iCase};
    plot(axRightV, normalizeGaitTime(data.grf.time), data.grf.ground_force_r_vy, ...
        'Color', caseSpecs(iCase).color, ...
        'LineStyle', caseSpecs(iCase).lineStyle, 'LineWidth', 1.8);
end
formatGrfAxis(axRightV, 'Right vGRF', 'vGRF (N)');

axImpulse = nexttile(layout, 7);
positiveImpulse = cellfun(@(x) x.rightPositiveApImpulse, caseData);
impulseBars = bar(axImpulse, categorical({caseSpecs.shortLabel}), positiveImpulse);
impulseBars.FaceColor = 'flat';
impulseBars.CData = vertcat(caseSpecs.color);
formatMetricAxis(axImpulse, 'Right positive AP-GRF impulse', ...
    'Propulsion impulse (N s)');
addBarLabels(axImpulse, positiveImpulse, '%.1f');

% Column 2: average speed, metabolic cost, and objectives.
axSpeed = nexttile(layout, 2);
averageSpeed = cellfun(@(x) x.averageSpeed, caseData);
speedBars = bar(axSpeed, categorical({caseSpecs.shortLabel}), averageSpeed);
speedBars.FaceColor = 'flat';
speedBars.CData = vertcat(caseSpecs.color);
formatMetricAxis(axSpeed, 'Average walking speed', 'Speed (m/s)');
axSpeed.YLabel.FontSize = 12;
addBarLabels(axSpeed, averageSpeed, '%.3f');

axMetabolic = nexttile(layout, 5);
metabolicCost = cellfun(@(x) x.metabolicEnergyPerDistance, caseData);
metabolicBars = bar(axMetabolic, categorical({caseSpecs.shortLabel}), metabolicCost);
metabolicBars.FaceColor = 'flat';
metabolicBars.CData = vertcat(caseSpecs.color);
formatMetricAxis(axMetabolic, 'Metabolic cost per distance', 'Metabolic cost (J m^{-1})');
addBarLabels(axMetabolic, metabolicCost, '%.1f');

% Objective contributions.
axObjective = nexttile(layout, 8);
objectiveData = zeros(3, numCases);
for iCase = 1:numCases
    objectiveData(:, iCase) = [caseData{iCase}.totalObjective; ...
        caseData{iCase}.effortObjective; caseData{iCase}.propulsionObjective];
end
objectiveBars = bar(axObjective, categorical({'Total', 'Effort', 'Propulsion'}), ...
    objectiveData, 'grouped');
for iCase = 1:numCases
    objectiveBars(iCase).FaceColor = caseSpecs(iCase).color;
end
yline(axObjective, 0, 'k-', 'HandleVisibility', 'off');
grid(axObjective, 'on');
ylabel(axObjective, 'Objective contribution');
title(axObjective, 'Moco objectives', 'FontSize', 14);

% Column 3: right hip, knee (sign reversed), and ankle angles.
jointTileIndices = [3, 6, 9];
for iJoint = 1:numel(jointVariables)
    axJoint = nexttile(layout, jointTileIndices(iJoint)); hold(axJoint, 'on');
    for iCase = 1:numCases
        data = caseData{iCase};
        plot(axJoint, normalizeGaitTime(data.kin.time), ...
            jointSigns(iJoint) * rad2deg(data.kin{:, jointVariables{iJoint}}), ...
            'Color', caseSpecs(iCase).color, ...
            'LineStyle', caseSpecs(iCase).lineStyle, 'LineWidth', 1.8);
    end
    grid(axJoint, 'on'); xlim(axJoint, [0, 100]);
    xlabel(axJoint, 'Gait cycle (%)'); ylabel(axJoint, 'Angle (deg)');
    title(axJoint, jointLabels{iJoint}, 'FontSize', 14);
end

legendText = string({caseSpecs.label});
legendFigure = legend(legendHandles, legendText, 'Interpreter', 'none', ...
    'FontSize', 12, 'Orientation', 'horizontal', 'NumColumns', 2);
try
    legendFigure.Layout.Tile = 'north';
catch
    legend(legendHandles, legendText, 'Interpreter', 'none', ...
        'Location', 'southoutside', 'FontSize', 12, 'NumColumns', 2);
end
sgtitle('Baseline vs propulsion-goal comparison', ...
    'FontWeight', 'bold', 'FontSize', 20);


function data = readCaseResult(resultsDir, resultName)
    resultRoot = fullfile(resultsDir, resultName, 'result_1', 'moco_result');
    assert(isfolder(resultRoot), '결과 폴더가 없습니다: %s', resultRoot);

    data.grf = readStoTable(fullfile(resultRoot, ...
        'moco_WoC_Solution_iter01_GRF.sto'));
    data.kin = readStoTable(fullfile(resultRoot, ...
        'moco_WoC_Solution_iter01_kinematics.sto'));

    halfKinText = fileread(fullfile(resultRoot, ...
        'moco_WoC_Solution_iter01_kinematics_half.sto'));
    metabolicText = fileread(fullfile(resultRoot, ...
        'moco_WoC_Solution_iter01_metabolic.sto'));

    data.totalObjective = headerValue(halfKinText, 'objective', NaN);
    data.effortObjective = headerValue(halfKinText, 'objective_effort', NaN);
    data.propulsionObjective = ...
        headerValue(halfKinText, 'objective_propulsion_r', 0) + ...
        headerValue(halfKinText, 'objective_propulsion_l', 0);
    data.metabolicEnergy = headerValue(metabolicText, ...
        'Total metabolic energy(J)', NaN);
    data.rightPositiveApImpulse = trapz(data.grf.time, ...
        max(data.grf.ground_force_r_vx, 0));
    pelvisTxName = '/jointset/groundPelvis/pelvis_tx/value';
    distance = data.kin{end, pelvisTxName} - data.kin{1, pelvisTxName};
    duration = data.kin.time(end) - data.kin.time(1);
    data.averageSpeed = distance / duration;
    data.metabolicEnergyPerDistance = data.metabolicEnergy / distance;
end


function tableData = readStoTable(filePath)
    headerLines = find(strcmp(strtrim(readlines(filePath)), "endheader"), ...
        1, 'first');
    assert(~isempty(headerLines), 'endheader를 찾지 못했습니다: %s', filePath);
    tableData = readtable(filePath, 'FileType', 'text', ...
        'NumHeaderLines', headerLines, 'VariableNamingRule', 'preserve');
end


function value = headerValue(text, fieldName, defaultValue)
    token = regexp(text, ['(?m)^' regexptranslate('escape', fieldName) ...
        '=([^\r\n]+)'], 'tokens', 'once');
    if isempty(token)
        value = defaultValue;
    else
        value = str2double(token{1});
    end
end


function timePercent = normalizeGaitTime(time)
    timePercent = 100 * (time - time(1)) / (time(end) - time(1));
end


function formatGrfAxis(ax, panelTitle, yAxisLabel)
    yline(ax, 0, 'k-', 'HandleVisibility', 'off');
    grid(ax, 'on'); xlim(ax, [0, 100]);
    xlabel(ax, 'Gait cycle (%)'); ylabel(ax, yAxisLabel);
    title(ax, panelTitle, 'FontSize', 14);
end


function formatMetricAxis(ax, panelTitle, yAxisLabel)
    grid(ax, 'on');
    ylabel(ax, yAxisLabel);
    title(ax, panelTitle, 'FontSize', 14);
end


function addBarLabels(ax, values, labelFormat)
    for iValue = 1:numel(values)
        text(ax, iValue, values(iValue), sprintf(labelFormat, values(iValue)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
            'FontSize', 10, 'FontWeight', 'bold');
    end
end
