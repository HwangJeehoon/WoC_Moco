% 호출 규약:
%   WoC_moco_main(model, iter, optMode, result_name)
%   WoC_moco_main(model, iter, optMode, result_name, opts)
%   WoC_moco_main(model, iter, optMode, result_name, opts, optResume)
%
%   optMode: 'modeWoC'    - WoC QP 기반 최적 보조 토크
%            'modeOff'    - 보조력 없음 (AFO=0, QP 생략)
%            struct       - modeSpline (optMode.type = 'modeSpline', + 파라미터 필드)
%
%   opts (선택 struct):
%     .alpha    : QP 비용 계수 (default = 0.01)
%     .beta     : QP 스무딩 계수 (default = 0)
%     .cost     : Main cost 종류 'et'|'etw'|'etv' (default = 'et')
%     .effort   : Moco effort goal weight (default = 1)
%     .finalTime: Moco final time goal weight (default = 0.03)
%     .gaitMode : 'modeSym' | 'modeAsym' (default = 'modeSym')
%
%   Resume 예시 (optResume.resume_name 이 있으면 자동 resume):
%     optResume.resume_name = 'et_a001b0_iter600\result_600';
%     WoC_moco_main('2D_gait_AFO_pc.osim', 400, 'modeWoC', 'et_a001b0_iter600to1000', opts, optResume)

%% Propulsion-weight sweep and speed-locked comparison -- Run Current Section only
% `Run`으로 전체 파일을 실행하지 말고 이 section만 실행한다.
%
% Case 1--4: (mocoEffort, mocoPropulsionWeight) =
%            (1, 5), (1, 1), (1, 0.3), (1, 0.03)
% Case 5:    w=1 해의 half-gait time/distance를 exact bound로 고정한
%            (1, 10) 해. 따라서 w=1 해와 같은 평균 속도를 강제한다.
% Smoothing: w=1에서 mocoPropulsionSmoothing = 0.2, 0.6, 1.0을 비교한다.
%            w=1, s=0.2는 Case 2를 재사용하며 나머지 case는 s=0.2이다.
%
% Solver mesh(300), convergence/constraint tolerance(1e-3)는
% moco_WoC_loop.m의 기존 설정을 그대로 사용한다.
clc; clear; close all;
thisScriptDir = fileparts(mfilename('fullpath'));
if isempty(thisScriptDir)
    % Current Section 실행 시 mfilename이 비어도, repository root와
    % script 폴더 중 현재 작업 폴더에 맞는 위치를 선택한다.
    if isfile(fullfile(pwd, 'WoC_moco_main_loop.m'))
        thisScriptDir = pwd;
    elseif isfile(fullfile(pwd, 'script', 'WoC_moco_main_loop.m'))
        thisScriptDir = fullfile(pwd, 'script');
    else
        error(['WoC_moco_main_loop.m 위치를 찾지 못했습니다. 작업 폴더를 ', ...
               'repository root 또는 script 폴더로 설정하세요.']);
    end
end
addpath(genpath(fullfile(thisScriptDir, '..', 'src')));

% true: 새 최적화를 실행한다. Plot/table만 다시 만들려면 false로 둔다.
runOptimizations = true;

modelName = '2D_gait_AFO_pc.osim';
resultsDir = fullfile(thisScriptDir, '..', 'results');
sweepWeights = [5, 1, 0.3, 0.03];
sweepResultNames = ["propulsion_ft0_w5", "propulsion_ft0_w1", ...
    "propulsion_ft0_w0p3", "propulsion_ft0_w0p03"];
speedLockedResultName = "propulsion_ft0_w10_speedLockedToW1";
primaryCaseLabels = ["w = 5", "w = 1", "w = 0.3", "w = 0.03", ...
    "w = 10 (w=1 speed locked)"];
smoothingValues = [0.2, 0.6, 1.0];
smoothingResultNames = [sweepResultNames(2), ...
    "propulsion_ft0_w1_s0p6", "propulsion_ft0_w1_s1p0"];
smoothingCaseLabels = ["s = 0.2", "s = 0.6", "s = 1.0"];
primaryResultNames = [sweepResultNames, speedLockedResultName];
% w=1, s=0.2는 weight sweep 결과를 재사용하므로 중복 실행하지 않는다.
allResultNames = [primaryResultNames, smoothingResultNames(2:end)];
allCaseLabels = [primaryCaseLabels, "w = 1, s = 0.6", "w = 1, s = 1.0"];
primaryCaseIndices = 1:numel(primaryResultNames);
smoothingCaseIndices = [2, numel(primaryResultNames) + 1, ...
    numel(primaryResultNames) + 2];

% 모든 sweep case에 modeSym 기본 guess와 기존 mesh/tolerance를 사용한다.
optsBase = struct();
optsBase.gaitMode = 'modeSym';
optsBase.mocoEffort = 1;
optsBase.mocoFinalTime = 0;
optsBase.mocoPropulsionSmoothing = 0.2;
% Time bound가 speed-locked case에서 바뀌어도 weight 정의가 변하지 않도록
% propulsion 정규화 기준 시간은 기존 default mean([0.4, 0.8]) = 0.6 s로 고정한다.
optsBase.mocoPropulsionReferenceDuration = 0.6;
pelvisTxName = '/jointset/groundPelvis/pelvis_tx/value';

if runOptimizations
    % 1) 동일 조건에서 propulsion weight sweep. 이미 완료된 case는 재사용한다.
    for iWeight = 1:numel(sweepWeights)
        opts = optsBase;
        opts.mocoPropulsionWeight = sweepWeights(iWeight);
        resultName = sweepResultNames(iWeight);
        completeKinPath = fullfile(resultsDir, resultName, 'result_1', ...
            'moco_result', 'moco_WoC_Solution_iter01_kinematics.sto');
        if isfile(completeKinPath)
            fprintf('완료된 case 재사용: %s\n', resultName);
        else
            assert(~isfolder(fullfile(resultsDir, resultName)), ...
                '불완전한 결과 폴더가 있습니다: %s. 삭제하거나 새 결과 이름을 사용하세요.', ...
                resultName);
            WoC_moco_main(modelName, 1, 'modeOff', char(resultName), opts);
        end
    end

    % 2) w=1 해에서 full gait의 time/distance를 읽어 half-gait bound로 변환.
    w1ResultRoot = fullfile(resultsDir, sweepResultNames(2), 'result_1', ...
        'moco_result');
    w1KinematicsFile = fullfile(w1ResultRoot, ...
        'moco_WoC_Solution_iter01_kinematics.sto');
    w1KinematicsHeaderLines = find(strcmp( ...
        strtrim(readlines(w1KinematicsFile)), "endheader"), 1, 'first');
    assert(~isempty(w1KinematicsHeaderLines), ...
        'endheader를 찾지 못했습니다: %s', w1KinematicsFile);
    w1Kinematics = readtable(w1KinematicsFile, ...
        'FileType', 'text', 'NumHeaderLines', w1KinematicsHeaderLines, ...
        'VariableNamingRule', 'preserve');
    w1FullTime = w1Kinematics.time(end) - w1Kinematics.time(1);
    w1FullDistance = w1Kinematics{end, pelvisTxName} - ...
        w1Kinematics{1, pelvisTxName};
    w1HalfTime = w1FullTime / 2;
    w1HalfDistance = w1FullDistance / 2;
    fprintf(['w=1 reference: full time = %.6f s, full distance = %.6f m, ', ...
             'speed = %.6f m/s\n'], w1FullTime, w1FullDistance, ...
             w1FullDistance / w1FullTime);

    % 3) w=10: w=1 해와 동일한 half-gait time/distance를 exact bound로 강제.
    optsSpeedLocked = optsBase;
    optsSpeedLocked.mocoPropulsionWeight = 10;
    optsSpeedLocked.mocoTimeBound = [w1HalfTime, w1HalfTime];
    optsSpeedLocked.mocoDistBound = [w1HalfDistance, w1HalfDistance];
    speedLockedKinPath = fullfile(resultsDir, speedLockedResultName, 'result_1', ...
        'moco_result', 'moco_WoC_Solution_iter01_kinematics.sto');
    if isfile(speedLockedKinPath)
        fprintf('완료된 case 재사용: %s\n', speedLockedResultName);
    else
        assert(~isfolder(fullfile(resultsDir, speedLockedResultName)), ...
            '불완전한 결과 폴더가 있습니다: %s. 삭제하거나 새 결과 이름을 사용하세요.', ...
            speedLockedResultName);
        WoC_moco_main(modelName, 1, 'modeOff', ...
            char(speedLockedResultName), optsSpeedLocked);
    end

    % 4) w=1에서 propulsion smoothing만 0.6, 1.0으로 변경해 비교.
    % s=0.2 결과는 위 weight sweep의 w=1 결과를 그대로 사용한다.
    for iSmoothing = 2:numel(smoothingValues)
        optsSmoothing = optsBase;
        optsSmoothing.mocoPropulsionWeight = 1;
        optsSmoothing.mocoPropulsionSmoothing = smoothingValues(iSmoothing);
        resultName = smoothingResultNames(iSmoothing);
        completeKinPath = fullfile(resultsDir, resultName, 'result_1', ...
            'moco_result', 'moco_WoC_Solution_iter01_kinematics.sto');
        if isfile(completeKinPath)
            fprintf('완료된 case 재사용: %s\n', resultName);
        else
            assert(~isfolder(fullfile(resultsDir, resultName)), ...
                '불완전한 결과 폴더가 있습니다: %s. 삭제하거나 새 결과 이름을 사용하세요.', ...
                resultName);
            WoC_moco_main(modelName, 1, 'modeOff', char(resultName), optsSmoothing);
        end
    end
end

% 5) 모든 case의 AP-GRF, kinematics, metabolic 및 objective를 읽는다.
numCases = numel(allResultNames);
positiveImpulse = zeros(numCases, 2);
positiveImpulseTotal = zeros(numCases, 1);
peakPropulsion = zeros(numCases, 2);
peakBraking = zeros(numCases, 2);
duration = zeros(numCases, 1);
distance = zeros(numCases, 1);
speed = zeros(numCases, 1);
effortObjective = zeros(numCases, 1);
propulsionObjective = zeros(numCases, 1);
metabolicEnergy = zeros(numCases, 1);
metabolicRate = zeros(numCases, 1);
metabolicEnergyPerDistance = zeros(numCases, 1);
grfTime = cell(numCases, 1);
grfRightAP = cell(numCases, 1);
grfLeftAP = cell(numCases, 1);

for iCase = 1:numCases
    resultRoot = fullfile(resultsDir, allResultNames(iCase), 'result_1', ...
        'moco_result');
    assert(isfolder(resultRoot), '결과 폴더가 없습니다: %s', resultRoot);

    grfFile = fullfile(resultRoot, 'moco_WoC_Solution_iter01_GRF.sto');
    grfHeaderLines = find(strcmp(strtrim(readlines(grfFile)), ...
        "endheader"), 1, 'first');
    assert(~isempty(grfHeaderLines), 'endheader를 찾지 못했습니다: %s', grfFile);
    grf = readtable(grfFile, 'FileType', 'text', ...
        'NumHeaderLines', grfHeaderLines, 'VariableNamingRule', 'preserve');

    kinFile = fullfile(resultRoot, ...
        'moco_WoC_Solution_iter01_kinematics.sto');
    kinHeaderLines = find(strcmp(strtrim(readlines(kinFile)), ...
        "endheader"), 1, 'first');
    assert(~isempty(kinHeaderLines), 'endheader를 찾지 못했습니다: %s', kinFile);
    kin = readtable(kinFile, 'FileType', 'text', ...
        'NumHeaderLines', kinHeaderLines, 'VariableNamingRule', 'preserve');
    halfKinText = fileread(fullfile(resultRoot, ...
        'moco_WoC_Solution_iter01_kinematics_half.sto'));
    metabolicText = fileread(fullfile(resultRoot, ...
        'moco_WoC_Solution_iter01_metabolic.sto'));

    grfTime{iCase} = grf.time;
    grfRightAP{iCase} = grf.ground_force_r_vx;
    grfLeftAP{iCase} = grf.ground_force_l_vx;
    positiveImpulse(iCase, :) = [ ...
        trapz(grf.time, max(grf.ground_force_r_vx, 0)), ...
        trapz(grf.time, max(grf.ground_force_l_vx, 0))];
    positiveImpulseTotal(iCase) = sum(positiveImpulse(iCase, :));
    peakPropulsion(iCase, :) = [max(grf.ground_force_r_vx), ...
        max(grf.ground_force_l_vx)];
    peakBraking(iCase, :) = [min(grf.ground_force_r_vx), ...
        min(grf.ground_force_l_vx)];
    duration(iCase) = kin.time(end) - kin.time(1);
    distance(iCase) = kin{end, pelvisTxName} - kin{1, pelvisTxName};
    speed(iCase) = distance(iCase) / duration(iCase);
    effortObjective(iCase) = str2double(regexp(halfKinText, ...
        'objective_effort=([^\r\n]+)', 'tokens', 'once'));
    propulsionR = str2double(regexp(halfKinText, ...
        'objective_propulsion_r=([^\r\n]+)', 'tokens', 'once'));
    propulsionL = str2double(regexp(halfKinText, ...
        'objective_propulsion_l=([^\r\n]+)', 'tokens', 'once'));
    propulsionObjective(iCase) = propulsionR + propulsionL;
    metabolicEnergy(iCase) = str2double(regexp(metabolicText, ...
        'Total metabolic energy\(J\)=([^\r\n]+)', 'tokens', 'once'));
    metabolicRate(iCase) = str2double(regexp(metabolicText, ...
        'Avg metabolic rate\(W\)=([^\r\n]+)', 'tokens', 'once'));
    metabolicEnergyPerDistance(iCase) = metabolicEnergy(iCase) / distance(iCase);
end

comparison = table(allCaseLabels(:), positiveImpulse(:, 1), ...
    positiveImpulse(:, 2), positiveImpulseTotal, peakPropulsion(:, 1), ...
    peakBraking(:, 1), duration, distance, speed, effortObjective, ...
    propulsionObjective, metabolicEnergy, metabolicRate, ...
    metabolicEnergyPerDistance, 'VariableNames', ...
    {'Case', 'PositiveImpulseRight_Ns', 'PositiveImpulseLeft_Ns', ...
    'PositiveImpulseTotal_Ns', 'PropulsivePeakRight_N', ...
    'BrakingPeakRight_N', 'Duration_s', 'Distance_m', 'Speed_mps', ...
    'EffortObjective', 'PropulsionObjective', 'MetabolicEnergy_J', ...
    'MetabolicRate_W', 'MetabolicEnergyPerDistance_Jpm'});
disp(comparison);
smoothingComparison = comparison(smoothingCaseIndices, :);
smoothingComparison.Case = smoothingCaseLabels(:);
disp(smoothingComparison);

% Plot 1: gait-cycle normalized AP-GRF 파형 (좌/우).
colors = lines(numCases);
figure('Name', 'Propulsion sweep: AP-GRF', 'Color', 'w');
tiledlayout(2, 1, 'TileSpacing', 'compact');
nexttile; hold on;
for iCase = primaryCaseIndices
    timePercent = 100 * (grfTime{iCase} - grfTime{iCase}(1)) / ...
        (grfTime{iCase}(end) - grfTime{iCase}(1));
    plot(timePercent, grfRightAP{iCase}, 'LineWidth', 1.2, ...
        'Color', colors(iCase, :));
end
yline(0, 'k:'); grid on; ylabel('Right AP-GRF (N)');
legend(primaryCaseLabels, 'Location', 'best');
nexttile; hold on;
for iCase = primaryCaseIndices
    timePercent = 100 * (grfTime{iCase} - grfTime{iCase}(1)) / ...
        (grfTime{iCase}(end) - grfTime{iCase}(1));
    plot(timePercent, grfLeftAP{iCase}, 'LineWidth', 1.2, ...
        'Color', colors(iCase, :));
end
yline(0, 'k:'); grid on;
xlabel('Gait cycle (%)'); ylabel('Left AP-GRF (N)');

% Plot 2: propulsion, speed 및 effort의 case별 변화.
figure('Name', 'Propulsion sweep: summary', 'Color', 'w');
tiledlayout(1, 3, 'TileSpacing', 'compact');
nexttile;
bar(categorical(primaryCaseLabels), positiveImpulseTotal(primaryCaseIndices));
ylabel('Total positive AP-GRF impulse (N s)'); grid on; xtickangle(25);
nexttile;
bar(categorical(primaryCaseLabels), speed(primaryCaseIndices));
ylabel('Average speed (m s^{-1})'); grid on; xtickangle(25);
nexttile;
bar(categorical(primaryCaseLabels), effortObjective(primaryCaseIndices));
ylabel('Moco effort objective'); grid on; xtickangle(25);

% Plot 3: propulsion-비용 trade-off. 마지막 점은 w=1 speed-locked w=10이다.
figure('Name', 'Propulsion sweep: trade-off', 'Color', 'w');
tiledlayout(1, 2, 'TileSpacing', 'compact');
nexttile; hold on;
scatter(positiveImpulseTotal(primaryCaseIndices), ...
    metabolicEnergyPerDistance(primaryCaseIndices), 90, ...
    colors(primaryCaseIndices, :), 'filled');
for iCase = primaryCaseIndices
    text(positiveImpulseTotal(iCase), metabolicEnergyPerDistance(iCase), ...
        ['  ' char(primaryCaseLabels(iCase))], 'VerticalAlignment', 'bottom');
end
grid on; xlabel('Total positive AP-GRF impulse (N s)');
ylabel('Metabolic energy per distance (J m^{-1})');
nexttile; hold on;
scatter(positiveImpulseTotal(primaryCaseIndices), effortObjective(primaryCaseIndices), ...
    90, colors(primaryCaseIndices, :), 'filled');
for iCase = primaryCaseIndices
    text(positiveImpulseTotal(iCase), effortObjective(iCase), ...
        ['  ' char(primaryCaseLabels(iCase))], 'VerticalAlignment', 'bottom');
end
grid on; xlabel('Total positive AP-GRF impulse (N s)');
ylabel('Moco effort objective');

% Plot 4: w=1에서 smoothing 변화만 비교한다.
figure('Name', 'Propulsion smoothing comparison (w=1)', 'Color', 'w');
tiledlayout(2, 3, 'TileSpacing', 'compact');
nexttile; hold on;
for iSmooth = 1:numel(smoothingCaseIndices)
    iCase = smoothingCaseIndices(iSmooth);
    timePercent = 100 * (grfTime{iCase} - grfTime{iCase}(1)) / ...
        (grfTime{iCase}(end) - grfTime{iCase}(1));
    plot(timePercent, grfRightAP{iCase}, 'LineWidth', 1.2, ...
        'Color', colors(iCase, :));
end
yline(0, 'k:'); grid on; ylabel('Right AP-GRF (N)');
legend(smoothingCaseLabels, 'Location', 'best');
nexttile; hold on;
for iSmooth = 1:numel(smoothingCaseIndices)
    iCase = smoothingCaseIndices(iSmooth);
    timePercent = 100 * (grfTime{iCase} - grfTime{iCase}(1)) / ...
        (grfTime{iCase}(end) - grfTime{iCase}(1));
    plot(timePercent, grfLeftAP{iCase}, 'LineWidth', 1.2, ...
        'Color', colors(iCase, :));
end
yline(0, 'k:'); grid on;
xlabel('Gait cycle (%)'); ylabel('Left AP-GRF (N)');
nexttile;
bar(categorical(smoothingCaseLabels), positiveImpulseTotal(smoothingCaseIndices));
ylabel('Total positive AP-GRF impulse (N s)'); grid on; xtickangle(25);
nexttile;
bar(categorical(smoothingCaseLabels), effortObjective(smoothingCaseIndices));
ylabel('Moco effort objective'); grid on; xtickangle(25);
nexttile;
bar(categorical(smoothingCaseLabels), metabolicEnergyPerDistance(smoothingCaseIndices));
ylabel('Metabolic energy per distance (J m^{-1})'); grid on; xtickangle(25);
nexttile;
bar(categorical(smoothingCaseLabels), speed(smoothingCaseIndices));
ylabel('Average speed (m s^{-1})'); grid on; xtickangle(25);

%% test
clc; clear; close all;
opts.QP_effort = 0.01;
WoC_moco_main('2D_gait_AFO_pc.osim', 2, 'modeWoC', 'test1', opts)

clc; clear; close all;
WoC_moco_main('2D_gait_AFO_pc_20BW.osim', 2, 'modeOff', 'test2')

clc; clear; close all;
optMode.type    = 'modeSpline';
optMode.trigger = 0.1;
optMode.rise    = 0.1;
optMode.flat    = 0.2;
optMode.fall    = 0.1;
optMode.maxVal  = 1.0;
WoC_moco_main('2D_gait_AFO_pc_30BW.osim', 2, optMode, 'test3')

clc; clear; close all;
optResume.resume_name = 'test3\result_1';
WoC_moco_main('2D_gait_AFO_pc_20BW.osim', 2, 'modeOff', 'test4', struct(), optResume)

clc; clear; close all;
optMode.type    = 'modeSpline';
optMode.trigger = 0.1;
optMode.rise    = 0.1;
optMode.flat    = 0.0;
optMode.fall    = 0.1;
optMode.maxVal  = 0.8;
WoC_moco_main('2D_gait_AFO_pc_30BW.osim', 2, optMode, 'test5')


clc; clear; close all;
optMode.type    = 'modeSpline';
optMode.trigger = 0.1;
optMode.rise    = 0.1;
optMode.flat    = 0.0;
optMode.fall    = 0.1;
optMode.maxVal  = 0.8;
opts.QP_effort = 0.03;
opts.QP_smooth = 0.1;
opts.gaitMode = 'modeSym';
WoC_moco_main('2D_gait_AFO_pc.osim', 2, optMode, 'test6', opts)

clc; clear; close all;
optMode.type    = 'modeSpline';
optMode.trigger = 0.1;
optMode.rise    = 0.1;
optMode.flat    = 0.0;
optMode.fall    = 0.1;
optMode.maxVal  = 0.8;
opts.QP_effort = 0.03;
opts.QP_smooth = 0.1;
opts.gaitMode = 'modeAsym';
WoC_moco_main('2D_gait_AFO_pc.osim', 2, optMode, 'test7', opts)

clc; clear; close all;
optMode.type    = 'modeSpline';
optMode.trigger = 0.25;
optMode.rise    = 0.25;
optMode.flat    = 0.0;
optMode.fall    = 0.1;
optMode.maxVal  = 0.8;
opts.gaitMode = 'modeSym';
WoC_moco_main('2D_gait_AFO_pc.osim', 2, optMode, 'test_modeS2', opts)

clc; clear; close all;
optMode.type    = 'modeTorqAmp';
optMode.maxVal  = 0.8;
opts.gaitMode = 'modeSym';
opts.guessInitSto = 'C:\Users\Jeehoon_Hwang\Desktop\Sim\WoC_Moco\results_initial\SF681-1\result_1\moco_result\moco_WoC_Solution_iter01_kinematics_half.sto';
WoC_moco_main('2D_gait_AFO_pc.osim', 2, optMode, 'test_torqA2', opts)

clc; clear; close all;
optMode.type    = 'modeTorqAmp';
optMode.maxVal  = 0.8;
opts.gaitMode = 'modeSym';
opts.guessInitSto = 'C:\Users\Jeehoon_Hwang\Desktop\Sim\WoC_Moco\results_initial\SF682-1\result_1\moco_result\moco_WoC_Solution_iter01_kinematics_half.sto';
WoC_moco_main('2D_gait_AFO_pc.osim', 2, optMode, 'test_torqA3', opts)

%% gait2354 example
clc; clear; close all;
optMode.type    = 'modeOff';
opts.gaitMode = 'modeAsym';
if isempty(mfilename)
    thisFile = matlab.desktop.editor.getActiveFilename;
else
    thisFile = mfilename("fullpath");
end
opts.guessInitSto = fullfile(fileparts(thisFile), '..', 'inputs', 'guess_init_full_gait2354.sto');
WoC_moco_main('gait2354_afo.osim', 1, optMode, 'test_gait2354', opts)

%% modeWoC 예시

clear; close all;
opts.QP_effort = 0.01;
opts.QP_smooth = 0;
opts.gaitMode = 'modeSym';
WoC_moco_main('2D_gait_AFO_pc.osim', 1, 'modeWoC', 'woc_test1', opts)

clear; close all;
opts.QP_effort = 0.01;
opts.QP_smooth = 0;
opts.gaitMode = 'modeAsym';
WoC_moco_main('2D_gait_AFO_pc.osim', 1, 'modeWoC', 'woc_test2', opts)

clear; close all;
opts.QP_effort = 0.01;
opts.QP_smooth = 1;
opts.gaitMode = 'modeSym';
WoC_moco_main('2D_gait_AFO_pc.osim', 1, 'modeWoC', 'woc_test3', opts)

clear; close all;
opts.QP_effort = 0.01;
opts.QP_smooth = 1;
opts.gaitMode = 'modeAsym';
WoC_moco_main('2D_gait_AFO_pc.osim', 1, 'modeWoC', 'woc_test4', opts)

% clc; clear; close all;
% opts.QP_effort = 0.01;
% optResume.resume_name = 'et_a001b0_iter600\result_600';
% WoC_moco_main('2D_gait_AFO_pc.osim', 400, 'modeWoC', 'et_a001b0_iter600to1000', opts, optResume)

% clc; clear; close all;
% opts.QP_effort = 0.3;
% WoC_moco_main('2D_gait_AFO_pc.osim', 300, 'modeWoC', 'et_a03b0_iter300', opts)

% clc; clear; close all;
% opts.QP_effort = 0.01;
% WoC_moco_main('2D_gait_AFO_pc_30BW.osim', 300, 'modeWoC', 'et_a001b0_iter300_30BW', opts)

% clc; clear; close all;
% opts.QP_effort = 0.05;
% WoC_moco_main('2D_gait_AFO_pc_30BW.osim', 300, 'modeWoC', 'et_a005b0_iter300_30BW', opts)

% clc; clear; close all;
% opts.QP_effort = 0.1;
% WoC_moco_main('2D_gait_AFO_pc_30BW.osim', 300, 'modeWoC', 'et_a01b0_iter300_30BW', opts)

% clc; clear; close all;
% opts.QP_effort = 0.3;
% WoC_moco_main('2D_gait_AFO_pc_30BW.osim', 300, 'modeWoC', 'et_a03b0_iter300_30BW', opts)

% clc; clear; close all;
% opts.QP_effort = 0.5;
% WoC_moco_main('2D_gait_AFO_pc_30BW.osim', 300, 'modeWoC', 'et_a05b0_iter300_30BW', opts)

% clc; clear; close all;
% opts.QP_effort = 0.01;
% optResume.resume_name = 'et_a001b0_iter1000\result_1000';
% WoC_moco_main('2D_gait_AFO_pc.osim', 500, 'modeWoC', 'et_a001b0_iter1000to1500', opts, optResume)

% clc; clear; close all;
% opts.QP_effort = 0.01;
% WoC_moco_main('2D_gait_AFO_pc_40BW.osim', 300, 'modeWoC', 'et_a001b0_iter300_40BW', opts)

% clc; clear; close all;
% opts.QP_effort = 0.05;
% WoC_moco_main('2D_gait_AFO_pc_40BW.osim', 300, 'modeWoC', 'et_a005b0_iter300_40BW', opts)

% clc; clear; close all;
% opts.QP_effort = 0.1;
% WoC_moco_main('2D_gait_AFO_pc_40BW.osim', 300, 'modeWoC', 'et_a01b0_iter300_40BW', opts)

% clc; clear; close all;
% opts.QP_effort = 0.3;
% WoC_moco_main('2D_gait_AFO_pc_40BW.osim', 300, 'modeWoC', 'et_a03b0_iter300_40BW', opts)

% clc; clear; close all;
% opts.QP_effort = 0.5;
% WoC_moco_main('2D_gait_AFO_pc_40BW.osim', 300, 'modeWoC', 'et_a05b0_iter300_40BW', opts)

% clc; clear; close all;
% opts.QP_effort = 0.01;
% WoC_moco_main('2D_gait_AFO_pc_20BW.osim', 300, 'modeWoC', 'et_a001b0_iter300_20BW', opts)

% clc; clear; close all;
% opts.QP_effort = 0.05;
% WoC_moco_main('2D_gait_AFO_pc_20BW.osim', 300, 'modeWoC', 'et_a005b0_iter300_20BW', opts)  % 258번에서 오류뜸

% clc; clear; close all;
% opts.QP_effort = 0.01;
% optResume.resume_name = 'et_a005b0_iter300_20BW\result_258';
% WoC_moco_main('2D_gait_AFO_pc_20BW.osim', 45, 'modeWoC', 'et_a005b0_iter258to300_20BW', opts, optResume)

% clc; clear; close all;
% opts.QP_effort = 0.1;
% WoC_moco_main('2D_gait_AFO_pc_20BW.osim', 300, 'modeWoC', 'et_a01b0_iter300_20BW', opts)

% clc; clear; close all;
% opts.QP_effort = 0.3;
% WoC_moco_main('2D_gait_AFO_pc_20BW.osim', 300, 'modeWoC', 'et_a03b0_iter300_20BW', opts)  % 36번까지 돌리고 끊음

% clc; clear; close all;
% opts.QP_effort = 0.3;
% optResume.resume_name = 'et_a03b0_iter300_20BW\result_36';
% WoC_moco_main('2D_gait_AFO_pc_20BW.osim', 270, 'modeWoC', 'et_a005b0_iter36to300_20BW', opts, optResume)

% clc; clear; close all;
% opts.QP_effort = 0.5;
% WoC_moco_main('2D_gait_AFO_pc_20BW.osim', 300, 'modeWoC', 'et_a05b0_iter300_20BW', opts)

clc; clear; close all;
opts.QP_effort = 0.01;
WoC_moco_main('2D_gait_AFO_pc_20BW.osim', 300, 'modeWoC', 'test2', opts)

%% modeOff 예시

% clc; clear; close all;
% WoC_moco_main('2D_gait_AFO_pc_20BW.osim', 1, 'modeOff', 'off_20BW')

%% modeSpline 예시
%   optMode를 struct로 전달. trigger+rise+flat+fall <= 0.6, maxVal in [0,1]
%
%   정규화 도메인 [0, 0.6] 기준 파형 예시:
%     (trigger=0.1, rise=0.1, flat=0.2, fall=0.1, max=0.8)
%     → 0.1에서 시작해 0.1 동안 상승, 0.2 동안 0.8 유지, 0.1 동안 하강

% clc; clear; close all;
% optMode.type    = 'modeSpline';
% optMode.trigger = 0.1;
% optMode.rise    = 0.1;
% optMode.flat    = 0.2;
% optMode.fall    = 0.1;
% optMode.maxVal  = 0.8;
% WoC_moco_main('2D_gait_AFO_pc_20BW.osim', 300, optMode, 'spline_t01r01f02d01m08_20BW')

%% 근육 장애 (modeWoC)

% clc; clear; close all;
% opts.QP_effort = 0.01;
% WoC_moco_main('2D_gait_AFO_pc_sol25.osim', 300, 'modeWoC', 'et_a001b0_iter300_50BW_sol25', opts)

% clc; clear; close all;
% opts.QP_effort = 0.01;
% WoC_moco_main('2D_gait_AFO_pc_gastr25.osim', 300, 'modeWoC', 'et_a001b0_iter300_50BW_gastr25', opts)

%% Off 추출 (moco_WoC_loop_extractOff 직접 사용 - 단발성 추출용)
clc; clear; close all;

if isempty(mfilename)
    thisFile = matlab.desktop.editor.getActiveFilename;
else
    thisFile = mfilename("fullpath");
end
baseFolder = fileparts(thisFile);

Model = '2D_gait_AFO_pc_short.osim';
ModelDir = fullfile(baseFolder,'..','models', Model);
resultDir = fullfile(baseFolder,'..','results', "short_test");
guess ='guess_init_half_short.sto';
guessDir = fullfile(baseFolder,'..','inputs', guess);
sol = moco_WoC_loop_extractOff(guessDir,ModelDir);
resOpts.modelPath = ModelDir;
moco_WoC_getResult(sol,resultDir,resOpts);

Model = '2D_gait_AFO_pc_long.osim';
ModelDir = fullfile(baseFolder,'..','models', Model);
resultDir = fullfile(baseFolder,'..','results', "long_test");
guess ='guess_init_half_long.sto';
guessDir = fullfile(baseFolder,'..','inputs', guess);
sol = moco_WoC_loop_extractOff(guessDir,ModelDir);
resOpts.modelPath = ModelDir;
moco_WoC_getResult(sol,resultDir,resOpts);

%% Asym test
clc; clear; close all;

if isempty(mfilename)
    thisFile = matlab.desktop.editor.getActiveFilename;
else
    thisFile = mfilename("fullpath");
end
baseFolder = fileparts(thisFile);

Model = '2D_gait_AFO_pc_off.osim';
ModelPath = fullfile(baseFolder,'..','models',Model);
guessInit ='guess_init_full.sto';
guessPath = fullfile(baseFolder,'..','inputs',guessInit);
sol = moco_WoC_loop_asym(guessPath,ModelPath);
resultDir = fullfile(baseFolder,'..','results','Compare_Asym');
resOpts.modelPath = ModelPath;
moco_WoC_getResult(sol,resultDir,resOpts);

Model = '2D_gait_AFO_pc_off.osim';
ModelPath = fullfile(baseFolder,'..','models',Model);
guessInit ='guess_init_half.sto';
guessPath = fullfile(baseFolder,'..','inputs',guessInit);
sol = moco_WoC_loop_extractOff(guessPath,ModelPath);
resultDir = fullfile(baseFolder,'..','results','Compare_Sym');
resOpts.modelPath = ModelPath;
moco_WoC_getResult(sol,resultDir,resOpts);

dataAsym = fullfile(baseFolder,'..','results','Compare_Asym','moco_WoC_Solution_kinematics_half.sto');
dataAsym = get_opensim_STO2(dataAsym);
grfAsym = fullfile(baseFolder,'..','results','Compare_Asym','moco_WoC_Solution_GRF.sto');
grfAsym = get_opensim_STO2(grfAsym);
dataSym  = fullfile(baseFolder,'..','results','Compare_Sym','moco_WoC_Solution_kinematics.sto');
dataSym = get_opensim_STO2(dataSym);
grfSym = fullfile(baseFolder,'..','results','Compare_Sym','moco_WoC_Solution_GRF.sto');
grfSym = get_opensim_STO2(grfSym);

figure; hold on
plot(dataAsym.time, dataAsym.x_jointset_ankle_l_ankle_angle_l_value)
plot(dataSym.time, dataSym.x_jointset_ankle_l_ankle_angle_l_value)
ylabel('Angle(rad)')
xlabel('time')
title('Ankle(L)')
legend('Full', 'Half')

figure; hold on
plot(dataAsym.time, dataAsym.x_jointset_knee_l_knee_angle_l_value)
plot(dataSym.time, dataSym.x_jointset_knee_l_knee_angle_l_value)
ylabel('Angle(rad)')
xlabel('time')
legend('Full', 'Half')
title('Knee(L)')

figure; hold on
plot(dataAsym.time, dataAsym.x_jointset_hip_l_hip_flexion_l_value)
plot(dataSym.time, dataSym.x_jointset_hip_l_hip_flexion_l_value)
ylabel('Angle(rad)')
xlabel('time')
legend('Full', 'Half')
title('Hip(L)')

figure; hold on
plot(grfAsym.time(1:101), grfAsym.ground_force_l_vx(1:101))
plot(grfSym.time, grfSym.ground_force_l_vx)
ylabel('Force(N)')
xlabel('time')
title('apGRF')
legend('Full', 'Half')


clc; clear; close all;
opts.gaitMode = 'modeAsym';
WoC_moco_main('2D_gait_AFO_pc_off.osim', 1, 'modeOff', 'Compare_Asym2', opts)

clc; clear; close all;
opts.gaitMode = 'modeSym';
WoC_moco_main('2D_gait_AFO_pc_off.osim', 1, 'modeOff', 'Compare_Sym2', opts)
