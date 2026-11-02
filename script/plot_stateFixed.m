% plot_stateFixed.m
%
% test_stateFixed 결과를 x = 평균속도 v_avg, y = effort target E0로 그린다.
% 한 bound 세트(boundSet)의 case만 쓰고, E0마다 J1-J2-J3 점을 찍어 선으로 잇는다.
% 점 색은 J1 -> J3로 갈수록 진해진다 (J 값 오름차순, 한 hue).
% Verdict가 OK가 아닌 case는 속이 빈 마커로 그린다 (수렴 여부와 상관없이 데이터는 그린다).
%
% 데이터: <resultsDir>/<runTag>_bounds_summary.csv (test_stateFixed가 쓰는 E0 x 세트 전체 요약)
% 그림:   <resultsDir>/stateFixed_plots/<시각>/ (png, fig, 그린 데이터 csv; 기존 폴더는 덮어쓰지 않는다)
clc; clear; close all;
thisScriptDir = fileparts(mfilename('fullpath'));

%% ===== 사용자 입력 =====
resultsDir = fullfile(thisScriptDir, '..', 'results');
% 그릴 실행 (test_stateFixed의 baseTag). 한 그림 안의 실행은 같은 objectiveMode, 같은 J 지표여야 한다.
% J = P_gate 최대화: 20261101_160827 (E0 = 0.10), 20261102_002630 (E0 = 0.08, 0.12)
runTags = {'stateFixed_max_20261101_160827', 'stateFixed_max_20261102_002630'};
boundSet = 'X3';
connectBy = 'E0';   % 'E0': 같은 E0의 J1-J2-J3를 잇는다 | 'J': 같은 J의 점을 E0 순서로 잇는다
modelLabel = '2D_gait';   % 제목의 모델 이름 (gait mode는 결과의 GaitMode에서 붙인다)

%% ===== 데이터 읽기 =====
cols = {'Case', 'ObjectiveMode', 'PropulsionDefinition', 'GaitMode', 'Verdict', 'BoundSet', ...
    'E_target', 'RunPropulsionTarget', 'v_avg_mps', 'E_actual', 'P_gate_actual', ...
    'MeanPropulsiveForce_N', 'PhaseDuration_s', 'PhaseDistance_m', 'BoundHits'};
textCols = {'Case', 'ObjectiveMode', 'PropulsionDefinition', 'GaitMode', 'Verdict', 'BoundSet', 'BoundHits'};
parts = cell(numel(runTags), 1);
for r = 1:numel(runTags)
    f = fullfile(resultsDir, [runTags{r} '_bounds_summary.csv']);
    if ~isfile(f), error('plot_stateFixed:NoSummary', '요약 파일이 없습니다: %s', f); end
    opts = detectImportOptions(f, 'TextType', 'string');
    opts = setvartype(opts, intersect(textCols, opts.VariableNames), 'string');   % 빈 열(BoundHits 등)도 string으로 읽는다
    t = readtable(f, opts);
    missing = setdiff(cols, t.Properties.VariableNames);
    if ~isempty(missing)
        error('plot_stateFixed:MissingColumn', '%s에 열이 없습니다: %s', f, strjoin(missing, ', '));
    end
    parts{r} = t(:, cols);
end
data = vertcat(parts{:});
data.BoundHits = fillmissing(data.BoundHits, 'constant', "");
data = data(data.BoundSet == boundSet, :);
if isempty(data), error('plot_stateFixed:NoRows', 'bound 세트 %s의 case가 없습니다.', boundSet); end

objectiveMode = unique(data.ObjectiveMode);
gaitMode = unique(data.GaitMode);
isMeanForce = startsWith(data.PropulsionDefinition, 'gate_mean_force');
if numel(objectiveMode) > 1 || numel(gaitMode) > 1 || any(isMeanForce) ~= all(isMeanForce)
    error('plot_stateFixed:MixedRuns', 'objectiveMode, gaitMode나 J 지표가 다른 실행이 섞여 있습니다.');
end
if all(isMeanForce)
    jName = 'Mean Propulsive Force'; jUnit = 'N'; metricTag = 'mpf';
else
    jName = 'Propulsion'; jUnit = 'N\cdots'; metricTag = 'pgate';
end

% 같은 (E0, J)가 두 번 나오면 어느 해를 그릴지 정할 수 없으므로 멈춘다.
data.E0 = round(data.E_target, 10);
data.J = data.RunPropulsionTarget;
[~, firstIdx, grp] = unique([data.E0, data.J], 'rows');
if numel(firstIdx) < height(data)
    dup = data(ismember(grp, find(accumarray(grp, 1) > 1)), :);
    disp(dup(:, {'Case', 'E0', 'J'}));
    error('plot_stateFixed:Duplicate', '같은 E0, J의 case가 여러 실행에 있습니다. runTags를 줄여주세요.');
end
data = sortrows(data, {'E0', 'J'});

%% ===== 그림 =====
ink = hex2rgb({'#0b0b0b', '#52514e', '#a3a29e'});
% J 색: 파랑 ordinal ramp (흰 배경에서 가장 옅은 점도 보이도록 step 250부터 650까지)
ramp = hex2rgb({'#86b6ef', '#6da7ec', '#5598e7', '#3987e5', '#2a78d6', ...
                '#256abf', '#1c5cab', '#184f95', '#104281'});
J_levels = unique(data.J);
nJ = numel(J_levels);
jColor = ramp(round(linspace(1, size(ramp, 1), nJ)), :);
if nJ == 1, jColor = ramp(5, :); end
E0_levels = unique(data.E0);
ok = data.Verdict == "OK";

fig = figure('Color', 'w', 'Position', [100 100 680 440]);
ax = axes(fig); hold(ax, 'on');
set(ax, 'Box', 'off', 'TickDir', 'out', 'XColor', ink(2, :), 'YColor', ink(2, :), ...
    'GridColor', [0.85 0.85 0.85], 'GridAlpha', 1, 'FontSize', 10);
grid(ax, 'on');

% 선을 먼저 그리고 점을 위에 올린다.
switch connectBy
    case 'E0'
        for e = 1:numel(E0_levels)
            rows = data.E0 == E0_levels(e);
            plot(ax, data.v_avg_mps(rows), data.E0(rows), '-', 'Color', ink(3, :), ...
                'LineWidth', 2, 'HandleVisibility', 'off');
        end
    case 'J'
        for j = 1:nJ
            rows = data.J == J_levels(j);
            plot(ax, data.v_avg_mps(rows), data.E0(rows), '-', 'Color', jColor(j, :), ...
                'LineWidth', 2, 'HandleVisibility', 'off');
        end
    otherwise
        error('plot_stateFixed:ConnectBy', 'connectBy는 ''E0'' 또는 ''J''입니다: %s', connectBy);
end

hJ = gobjects(nJ, 1);
for j = 1:nJ
    rows = data.J == J_levels(j);
    hJ(j) = plot(ax, data.v_avg_mps(rows & ok), data.E0(rows & ok), 'o', 'MarkerSize', 9, ...
        'LineWidth', 1.5, 'MarkerEdgeColor', jColor(j, :), 'MarkerFaceColor', jColor(j, :), ...
        'DisplayName', sprintf('J%d = %g %s', j, J_levels(j), jUnit));
    plot(ax, data.v_avg_mps(rows & ~ok), data.E0(rows & ~ok), 'o', 'MarkerSize', 9, ...
        'LineWidth', 1.5, 'MarkerEdgeColor', jColor(j, :), 'MarkerFaceColor', 'w', ...
        'HandleVisibility', 'off');
end
hLegend = hJ;
if any(~ok)
    hLegend(end + 1) = plot(ax, NaN, NaN, 'o', 'MarkerSize', 9, 'LineWidth', 1.5, ...
        'MarkerEdgeColor', ink(2, :), 'MarkerFaceColor', 'w', 'DisplayName', 'Verdict not OK');
end
legend(ax, hLegend, 'Location', 'best', 'Box', 'off');

xlabel(ax, 'Average Velocity (m/s)');
ylabel(ax, 'Fixed Effort');
yticks(ax, E0_levels);
vPad = max(0.1 * (max(data.v_avg_mps) - min(data.v_avg_mps)), 0.05);
xlim(ax, [min(data.v_avg_mps) - vPad, max(data.v_avg_mps) + vPad]);
ePad = max(0.15 * (max(E0_levels) - min(E0_levels)), 0.005);
ylim(ax, [min(E0_levels) - ePad, max(E0_levels) + ePad]);
% 예: 'Target (J) : Propulsion - 2D_gait, Sym' ('_'가 아래첨자가 되지 않도록 Interpreter none)
title(ax, sprintf('Target (J) : %s - %s, %s', jName, modelLabel, erase(gaitMode, 'mode')), ...
    'Color', ink(1, :), 'FontSize', 14, 'Interpreter', 'none');

%% ===== 저장 =====
outDir = fullfile(resultsDir, 'stateFixed_plots', char(datetime('now', 'Format', 'yyyyMMdd_HHmmss')));
if exist(outDir, 'dir'), error('plot_stateFixed:Exists', '이미 있는 폴더입니다: %s', outDir); end
mkdir(outDir);
name = sprintf('E0_vs_speed_%s_%s_%s', char(erase(objectiveMode, 'SpeedFixedEP')), metricTag, boundSet);
exportgraphics(fig, fullfile(outDir, [name '.png']), 'Resolution', 150);
savefig(fig, fullfile(outDir, [name '.fig']));
writetable(data, fullfile(outDir, [name '_data.csv']));
fprintf('저장: %s\n', outDir);
disp(data(:, {'E0', 'J', 'Verdict', 'v_avg_mps', 'E_actual', 'P_gate_actual', ...
    'MeanPropulsiveForce_N', 'PhaseDuration_s', 'PhaseDistance_m', 'BoundHits'}));


function rgb = hex2rgb(hex)
    rgb = cell2mat(cellfun(@(h) sscanf(h(2:end), '%2x%2x%2x')' / 255, hex(:), ...
        'UniformOutput', false));
end
