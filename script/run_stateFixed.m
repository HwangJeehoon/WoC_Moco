% run_stateFixed.m
%
% E0_list의 각 E0와 propulsion target J_k마다 독립적으로 푼다.
%
%   minimize (cfg.objectiveMode = 'minSpeedStateFixed') 또는
%   maximize (cfg.objectiveMode = 'maxSpeedStateFixed')
%               w_v * v_avg,  v_avg = (pelvis_tx(tf) - pelvis_tx(t0)) / (tf - t0)
%   subject to  E = E0, J 지표 = J_k, 기존 동역학/주기성/대칭성/상태·제어 bounds
%
%   E (cfg.energyMetric):
%     'effort'       : weighted 모드 effort goal의 weight 곱하기 전 값 [-]
%                      (1/d_COM) * int sum_c u_c^2 dt, c = 근육 excitation 18개 + lumbarAct
%                      (AFO_r/AFO_l control은 명시적으로 제외)
%     'metabolicCoT' : metabolic cost of transport [J/kg/m]
%                      int P_met dt / (mass * d_COM), P_met = Bhargava2004SmoothedMuscleMetabolics
%                      total_metabolic_rate [W] (basal 포함, use_smoothing, 모든 근육 등록),
%                      mass = 모델 총질량. modeSym은 반주기 에너지 / 반주기 COM 변위.
%                      E0_CoT는 직접 입력한다 (effort 목표값을 쓰지 않는다). 비워 두면 공통
%                      initial guess의 CoT만 출력하고 멈춘다.
%   J 지표 (cfg.propulsionMetric):
%     'impulse'   : P_gate, 한 보행주기 양발 gated propulsive impulse [N*s] (meanPropPerVel의 Iplus)
%                   int [F_r g(F_r) + F_l g(F_l)] dt, F_foot = heel + front x-force,
%                   g(F) = 1/(1 + exp(-(F - F0)/sigma)) (modeSym은 반주기 적분의 2배)
%     'meanForce' : 평균 추진력 P_gate / Tplus [N], Tplus = int [g(F_r) + g(F_l)] dt
%                   (meanPropPerVel의 MeanPropulsiveForce m; 반주기와 한 주기가 같다)
%            (P_gate, max(F,0) impulse, Tplus, 평균 추진력은 지표와 상관없이 함께 보고)
%   w_v    : 평균속도 cost weight (목적함수의 유일한 항)
%
% E0마다 bound 세트(boundSets)를 돌고, 세트마다 J1..Jn을 차례로 푼다. bound 세트는
% objectiveMode별로 정하고, 모든 case는 inputs의 공통 initial guess를 쓴다.
% 한 J나 한 세트가 수렴하지 않아도 기록만 하고 다음을 계속 푼다
% (target, bound, equality는 바꾸지 않는다).
%
% baseTag = stateFixed_<min|max>[_mpf][_cot]_<시각> (_mpf: meanForce 지표, _cot: metabolicCoT 지표),
% E0 태그 = E<E0의 '.'을 'p'로> (0.12 -> E0p12)
% 결과: <resultsDir>/<Index><번호>_<baseTag>_<E0 태그>_<세트>_<sym|asym>_J<k>/ (case별),
%       <resultsDir>/<baseTag>_<E0 태그>_<세트>_<mode>_summary/<시각>/ (세트별 summary.csv, 그림),
%       <resultsDir>/<baseTag>_bounds_summary.csv (전체 E0 x 세트 비교).
%       번호는 같은 Index의 기존 결과 폴더 중 최대 번호 다음부터 case마다 증가한다.
%       최소 세 자리로 표시하며 (H001, H002, ...), 기존 폴더명은 접두어 뒤에 유지한다.
%       기존 결과 폴더는 덮어쓰지 않는다.
% 플러그인이 없거나 C++ 소스가 바뀌었으면 먼저 빌드 (빌드 후 MATLAB을 새로 시작):
%   run(fullfile('..', 'plugins', 'osimFixedEffortPropulsion', 'build_osimFixedEffortPropulsion.m'))
clc; clear; close all;
thisScriptDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(thisScriptDir, '..', 'src')));

%% ===== 사용자 입력 ===== %%
Index = 'H';                   % 결과 폴더 접두어. 같은 Index의 마지막 번호 다음부터 이어간다
cfg.objectiveMode = 'maxSpeedStateFixed';   % 'minSpeedStateFixed' | 'maxSpeedStateFixed'

% State Fixed 설정
cfg.energyMetric = 'metabolicCoT';  % 'effort' | 'metabolicCoT'
E0_effort = [0.08, 0.10, 0.12];     % effort target들 (weight 곱하기 전 effort metric, 무차원)
E0_CoT = [3.2, 3.35, 3.5];          % metabolic CoT target들 [J/kg/m]: 낮음 / 중간 / 높음
if strcmp(cfg.energyMetric, 'metabolicCoT')
    E0_list = E0_CoT;               % 적힌 순서로 푼다
    cfg.energyTol = 1e-3;           % |CoT - E0| 검증 허용오차 [J/kg/m]
else
    E0_list = E0_effort;
    cfg.energyTol = 1e-3;           % |E - E0| 검증 허용오차 [-]
end
cfg.propulsionMetric = 'meanForce';   % 'impulse' (J = inte(positive apGRF)) | 'meanForce' (J = impulse를 propulsive time으로 나눈 것)
J_targets = [50, 65, 80];       % [J1, J2, J3]

% Bound 및 Initial guess 설정
if strcmp(cfg.objectiveMode, 'maxSpeedStateFixed')
    boundSets = struct( ...
        'name',      {'X3'}, ...
        'timeBound', {[0.25, 0.8]}, ...   % 최고 6.00 m/s
        'distBound', {[0.4, 1.5]});
else
    boundSets = struct( ...
        'name',      {'B3'}, ...
        'timeBound', {[0.4, 1.5]}, ...    % 최저 0.133 m/s
        'distBound', {[0.20, 1.0]});
end
stateFixedGuess = fullfile(thisScriptDir, '..', 'inputs', 'guess_init_stateFixed.sto');
guessFor = @(E0, setName) repmat({stateFixedGuess}, 1, numel(J_targets));

% 기타 설정
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
cfg.propulsionTol = 1e-2; % |J 지표 - J| 검증 허용오차
if strcmp(cfg.propulsionMetric, 'meanForce'), cfg.propulsionTol = 0.1; end % |J 지표 - J| 검증 허용오차
cfg.resultsDir = fullfile(thisScriptDir, '..', 'results');   % 실제 저장 루트
cfg.Index = Index;
cfg.continuation = false;       % true: case k가 직전 수렴 case의 해에서 시작 (CSV Guess 열에 기록)
cfg.runOptimizations = true;    % false: 같은 runTag의 기존 결과만 요약
cfg.J_targets = J_targets;
cfg.targetSource = 'experiment (user input)';

%% ===== 공통 initial guess의 에너지 지표 (target 규모 확인) =====
% 실행과 같은 대사 모델·정규화로 guess 자체의 effort E와 metabolic CoT를 계산한다.
guessMetrics = moco_WoC_evalGaitMetrics(fullfile(thisScriptDir, '..', 'models', cfg.model), ...
    stateFixedGuess, 1 + strcmp(cfg.gaitMode, 'modeSym'), cfg.gateSigma, cfg.gateThreshold);
fprintf(['\n공통 initial guess (%s): effort E = %.6g, metabolic CoT = %.6g J/kg/m ', ...
         '(E_met %.4g J, mass %.4g kg, d_COM %.4g m, v_avg %.4g m/s, basal %.4g W)\n'], ...
    stateFixedGuess, guessMetrics.E_simpson, guessMetrics.CoT_simpson, ...
    guessMetrics.metabolic_energy_simpson, guessMetrics.mass_kg, guessMetrics.com_displacement, ...
    guessMetrics.v_avg, guessMetrics.basal_rate_W);
if strcmp(cfg.energyMetric, 'metabolicCoT')
    if isempty(E0_list)
        error('run_stateFixed:NoCoTTarget', ['cfg.energyMetric = ''metabolicCoT'' 인데 E0_CoT가 비어 있습니다.\n', ...
            '공통 guess의 CoT는 %.4g J/kg/m 입니다. E0_CoT에 CoT target [J/kg/m]을 직접 입력한 뒤 ', ...
            '다시 실행하세요 (effort target E0_effort는 CoT target으로 쓰지 않습니다).'], ...
            guessMetrics.CoT_simpson);
    end
    % basal 항만으로도 CoT >= basal / (mass * v_avg) 이므로 속도 상한에서의 이 값보다
    % 작은 target은 풀 수 없다 (effort 값 0.08-0.12 등을 잘못 넣은 경우).
    for b = 1:numel(boundSets)
        vCeil = boundSets(b).distBound(2) / boundSets(b).timeBound(1);
        cotFloor = guessMetrics.basal_rate_W / (guessMetrics.mass_kg * vCeil);
        if any(E0_list <= cotFloor)
            error('run_stateFixed:CoTTargetTooLow', ['E0_CoT %s 중 bound 세트 %s의 basal-only CoT ', ...
                '하한 %.4g J/kg/m (v <= %.3g m/s) 이하인 값이 있습니다.'], mat2str(E0_list), ...
                boundSets(b).name, cotFloor, vCeil);
        end
    end
end

%% ===== E0 x bound 세트별 실행 =====
metricTag = '';
if strcmp(cfg.propulsionMetric, 'meanForce'), metricTag = '_mpf'; end
energyTag = '';
if strcmp(cfg.energyMetric, 'metabolicCoT'), energyTag = '_cot'; end
baseTag = sprintf('stateFixed_%s%s%s_%s', erase(cfg.objectiveMode, 'SpeedStateFixed'), metricTag, ...
    energyTag, char(datetime('now', 'Format', 'yyyyMMdd_HHmmss')));
summaries = cell(numel(E0_list), numel(boundSets));
for e = 1:numel(E0_list)
    cfg.E0 = E0_list(e);
    e0Tag = ['E' strrep(sprintf('%g', cfg.E0), '.', 'p')];
    for b = 1:numel(boundSets)
        cfg.timeBound = boundSets(b).timeBound;
        cfg.distBound = boundSets(b).distBound;
        cfg.guessSto = guessFor(cfg.E0, boundSets(b).name);
        cfg.runTag = sprintf('%s_%s_%s', baseTag, e0Tag, boundSets(b).name);
        vFloor = cfg.distBound(1) / cfg.timeBound(2);
        vCeil = cfg.distBound(2) / cfg.timeBound(1);
        fprintf('\n##### %s E0 = %g (%s), bound set %s: time %s s, dist %s m (v range %.3f-%.3f m/s) #####\n', ...
            cfg.objectiveMode, cfg.E0, cfg.energyMetric, boundSets(b).name, mat2str(cfg.timeBound), ...
            mat2str(cfg.distBound), vFloor, vCeil);
        for k = 1:numel(J_targets)
            [~, guessName, guessExt] = fileparts(cfg.guessSto{k});
            fprintf('  J%d = %g (%s): guess %s\n', k, J_targets(k), cfg.propulsionMetric, [guessName guessExt]);
        end
        try
            s = WoC_moco_runStateFixedCases(cfg);
            s.BoundSet = repmat(string(boundSets(b).name), height(s), 1);
            s.VFloor_mps = repmat(vFloor, height(s), 1);
            s.VCeil_mps = repmat(vCeil, height(s), 1);
            summaries{e, b} = s;
        catch ME
            % 한 세트가 실패해도 다음 세트를 계속 돌린다.
            warning('run_stateFixed:BoundSetFailed', 'E0 = %g, bound set %s failed: %s', ...
                cfg.E0, boundSets(b).name, ME.message);
        end
    end
end

summaries = summaries(~cellfun(@isempty, summaries));
if isempty(summaries)
    warning('run_stateFixed:NoSummary', '요약할 bound 세트 결과가 없습니다.');
else
    allSummary = vertcat(summaries{:});
    writetable(allSummary, fullfile(cfg.resultsDir, [baseTag '_bounds_summary.csv']));
    disp(allSummary(:, {'EnergyMetric', 'E_target', 'E_actual', 'BoundSet', 'Case', 'Verdict', 'RunPropulsionTarget', ...
        'P_gate_actual', 'MeanPropulsiveForce_N', 'v_avg_mps', 'VFloor_mps', 'VCeil_mps', ...
        'PhaseDuration_s', 'PhaseDistance_m', 'BoundHits'}));
end
