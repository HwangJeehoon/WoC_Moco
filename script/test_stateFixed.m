% test_stateFixed.m
%
% 같은 E0에 대해 propulsion target J_k마다 독립적으로 푼다.
%
%   minimize (cfg.objectiveMode = 'minSpeedFixedEP') 또는
%   maximize (cfg.objectiveMode = 'maxSpeedFixedEP')
%               w_v * v_avg,  v_avg = (pelvis_tx(tf) - pelvis_tx(t0)) / (tf - t0)
%   subject to  E = E0, P_gate = J_k, 기존 동역학/주기성/대칭성/상태·제어 bounds
%
%   E      : weighted 모드 effort goal의 weight 곱하기 전 값
%            (1/d_COM) * int sum_c u_c^2 dt, c = 근육 excitation 18개 + lumbarAct
%            (+ AFO_r/AFO_l: PrescribedController 값, modeOff에서는 0)
%   P_gate : 한 보행주기 양발 gated propulsive impulse [N*s] (meanPropPerVel의 Iplus)
%            int [F_r g(F_r) + F_l g(F_l)] dt, F_foot = heel + front x-force,
%            g(F) = 1/(1 + exp(-(F - F0)/sigma))
%            (modeSym은 반주기 적분의 2배; max(F,0) impulse, Tplus, P_gate/Tplus는
%             비교용으로 함께 보고)
%   w_v    : 평균속도 cost weight (목적함수의 유일한 항)
%
% bound 세트(boundSets)마다 J1..Jn을 차례로 푼다. bound 세트와 initial guess는
% objectiveMode별로 정한다. 한 J나 한 세트가 수렴하지 않아도 기록만 하고 다음을
% 계속 푼다 (target, bound, equality는 바꾸지 않는다).
%
% baseTag = stateFixed_<min|max>_<시각>
% 결과: <resultsDir>/<baseTag>_<세트>_<sym|asym>_J<k>/ (case별),
%       <resultsDir>/<baseTag>_<세트>_<mode>_summary/<시각>/ (세트별 summary.csv, 그림),
%       <resultsDir>/<baseTag>_bounds_summary.csv (전체 세트 비교).
%       기존 결과 폴더는 덮어쓰지 않는다.
% 플러그인이 없으면 먼저 빌드:
%   run(fullfile('..', 'plugins', 'osimFixedEffortPropulsion', 'build_osimFixedEffortPropulsion.m'))
clc; clear; close all;
thisScriptDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(thisScriptDir, '..', 'src')));

%% ===== 사용자 입력 =====
E0 = 0.1;                       % effort target (weight 곱하기 전 effort metric, 무차원)
J_targets = [30, 40, 50];       % [J1, J2, J3] [N*s], 한 보행주기 양발 P_gate

cfg.objectiveMode = 'maxSpeedFixedEP';   % 'minSpeedFixedEP' | 'maxSpeedFixedEP'

% bound 세트 (반 걸음 기준, modeAsym은 x2)와 initial guess.
%   최저 가능 속도 = distBound(1) / timeBound(2), 최고 가능 속도 = distBound(2) / timeBound(1)
if strcmp(cfg.objectiveMode, 'maxSpeedFixedEP')
    % 속도를 밀어붙인 이전 해들은 반 걸음 시간 하한 0.4 s에 붙었다. 시간 하한과 거리
    % 상한을 넓혀 가며 최고 속도가 bound에서 떨어지는지 본다 (X1 = 기존 기본 bound).
    boundSets = struct( ...
        'name',      {'X1',        'X2',         'X3'}, ...
        'timeBound', {[0.4, 0.8],  [0.33, 0.8],  [0.25, 0.8]}, ...   % 최고 2.50 / 3.64 / 6.00 m/s
        'distBound', {[0.4, 1.0],  [0.4, 1.2],   [0.4, 1.5]});
    % Initial guess (모든 J 공통): propulsion_compare_no_goal (effort 1 + final time 0.03,
    % modeOff, modeSym, mesh 300; E = 0.0986, P_gate = 31.7 N*s, v_avg = 1.33 m/s,
    % 반 걸음 0.448 s / 0.594 m; 모든 세트의 bound 안)
    cfg.guessSto = fullfile(thisScriptDir, '..', 'results', 'propulsion_compare_no_goal', ...
        'result_1', 'moco_result', 'moco_WoC_Solution_iter01_kinematics_half.sto');
else
    % 이전 실행 [0.4 0.8] s / [0.4 1.0] m은 세 J 모두 0.5 m/s 모서리(time_ub;dist_lb)에 걸렸다.
    boundSets = struct( ...
        'name',      {'B1',        'B2',        'B3'}, ...
        'timeBound', {[0.4, 1.0],  [0.4, 1.2],  [0.4, 1.5]}, ...    % 최저 0.300 / 0.208 / 0.133 m/s
        'distBound', {[0.30, 1.0], [0.25, 1.0], [0.20, 1.0]});
    % Initial guess (J별): stateFixed_20261030_221717의 같은 J 해
    % (bound [0.4 0.8] s / [0.4 1.0] m, v_avg = 0.5 m/s, E = 0.1, P_gate = J; 모든 세트의 bound 안)
    % (sprintf에는 폴더 이름만 넘긴다: Windows 경로의 '\'를 escape 문자로 읽기 때문)
    cfg.guessSto = arrayfun(@(k) fullfile(thisScriptDir, '..', 'results', ...
        sprintf('stateFixed_20261030_221717_sym_J%d', k), 'result_1', 'moco_result', ...
        'moco_WoC_Solution_iter01_kinematics_half.sto'), 1:numel(J_targets), 'UniformOutput', false);
end

cfg.speedWeight = 1;            % 평균속도 cost weight 크기 w_v (방향은 objectiveMode)
cfg.gateSigma = 1.0;            % propulsion gate 폭 sigma [N] (meanPropPerVel 기본값)
cfg.gateThreshold = 5.0;        % propulsion gate 중심 F0 [N] (meanPropPerVel 기본값)
cfg.model = '2D_gait_AFO_pc.osim';
cfg.optMode = 'modeOff';        % 보조력 모드 (modeOff | modeWoC | modeSpline 등 string)
cfg.gaitMode = 'modeSym';       % 'modeSym' | 'modeAsym'
cfg.meshIntervals = 300;
cfg.convergenceTolerance = 5e-5;
cfg.constraintTolerance = 5e-5;
cfg.maxIterations = 5000;
cfg.effortTol = 1e-3;           % |E - E0| 검증 허용오차
cfg.propulsionTol = 1e-2;       % |P_gate - J| 검증 허용오차 [N*s]
cfg.resultsDir = fullfile(thisScriptDir, '..', 'results');   % 실제 저장 루트
cfg.continuation = false;       % true: case k가 직전 수렴 case의 해에서 시작 (CSV Guess 열에 기록)
cfg.runOptimizations = true;    % false: 같은 runTag의 기존 결과만 요약

cfg.E0 = E0;
cfg.J_targets = J_targets;
cfg.targetSource = 'experiment (user input)';

%% ===== bound 세트별 실행 =====
baseTag = sprintf('stateFixed_%s_%s', erase(cfg.objectiveMode, 'SpeedFixedEP'), ...
    char(datetime('now', 'Format', 'yyyyMMdd_HHmmss')));
summaries = cell(numel(boundSets), 1);
for b = 1:numel(boundSets)
    cfg.timeBound = boundSets(b).timeBound;
    cfg.distBound = boundSets(b).distBound;
    cfg.runTag = [baseTag '_' boundSets(b).name];
    vFloor = cfg.distBound(1) / cfg.timeBound(2);
    vCeil = cfg.distBound(2) / cfg.timeBound(1);
    fprintf('\n##### %s bound set %s: time %s s, dist %s m (v range %.3f-%.3f m/s) #####\n', ...
        cfg.objectiveMode, boundSets(b).name, mat2str(cfg.timeBound), ...
        mat2str(cfg.distBound), vFloor, vCeil);
    try
        s = WoC_moco_runMinSpeedCases(cfg);
        s.BoundSet = repmat(string(boundSets(b).name), height(s), 1);
        s.VFloor_mps = repmat(vFloor, height(s), 1);
        s.VCeil_mps = repmat(vCeil, height(s), 1);
        summaries{b} = s;
    catch ME
        % 한 세트가 실패해도 다음 세트를 계속 돌린다.
        warning('test_stateFixed:BoundSetFailed', 'Bound set %s failed: %s', ...
            boundSets(b).name, ME.message);
    end
end

summaries = summaries(~cellfun(@isempty, summaries));
if isempty(summaries)
    warning('test_stateFixed:NoSummary', '요약할 bound 세트 결과가 없습니다.');
else
    allSummary = vertcat(summaries{:});
    writetable(allSummary, fullfile(cfg.resultsDir, [baseTag '_bounds_summary.csv']));
    disp(allSummary(:, {'BoundSet', 'Case', 'Verdict', 'v_avg_mps', 'VFloor_mps', ...
        'VCeil_mps', 'PhaseDuration_s', 'PhaseDistance_m', 'BoundHits'}));
end
