% run_stateFixed_rajagopal.m
%
% rajagopal_AFO.osim (subject19, 3D)으로 stateFixed 최고속도 문제를 푼다. 수학적 정의와
% 실행·결과 정리 흐름은 run_stateFixed.m과 같고, 모델별 경로·접촉력·bound는
% moco_WoC_modelProfile('rajagopal_AFO.osim')이 정한다. E0와 J_k마다 독립적으로 푼다.
%
%   maximize    v_avg = (pelvis_tx(tf) - pelvis_tx(t0)) / (tf - t0)
%               (Moco cost = -speedWeight * v_avg, 유일한 cost)
%   subject to  metabolic CoT = E0      (endpoint equality, J/kg/m)
%               P_gate = J_k            (endpoint equality, N*s, 한 주기 양발 합)
%               또는 cfg.propulsionConstraintMode = 'range'이면
%               lower_k <= P_gate <= upper_k (endpoint bounds, cfg.J_bounds 행마다 한 case)
%               동역학, 슬개골 coupler, modeAsym 주기성, 상태·제어 bounds,
%               발 사이·팔-몸통 MocoFrameDistanceConstraint (path constraint)
%
%   CoT    = int P_met dt / (mass * d_COM), P_met = Bhargava2004SmoothedMuscleMetabolics
%            total_metabolic_rate (basal 포함, 근육 80개 모두 등록, 토크 구동기 대사 없음),
%            mass = 모델 총질량, d_COM = 전체 모델 질량중심 변위.
%   P_gate = int [F_r g(F_r) + F_l g(F_l)] dt, F_foot = 그 발 6개 접촉력 x-force 합
%            (합산한 뒤 gate), g(F) = 1/(1+exp(-(F-F0)/sigma)), modeAsym impulseScale = 1.
%   AFO    = AFO_controller가 전 구간 0 (modeOff), optimization control이 아니다.
%
% Bound 입력은 기존 인터페이스대로 반주기 기준이며, modeAsym에서 moco_WoC_loop가 한 번만
% 2배를 적용한다: X3 time [0.25, 0.8] -> 최종 시간 [0.5, 1.6] s, dist [0.4, 1.5] ->
% pelvis_tx(tf) [0.8, 3.0] m, t0 = 0, pelvis_tx(t0) = 0. 보행속도·pelvis 속도 bound는 없다.
%
% 초기값: inputs/guess_init_rajagopal.sto와 GRF_init_rajagopal.sto (수정하지 않는다).
% pelvis_tx/pelvis_tz의 world offset을 뺀 작업용 사본을 결과 폴더에 만들고
% (GRF CoP x/z도 같은 값만큼 이동), 세 case 모두 같은 사본에서 독립적으로 시작한다.
%
% 결과 (기존 결과를 덮어쓰지 않는다):
%   <resultsDir>/<baseTag>_inputs/                     정렬한 초기값 사본, run_config.txt
%   <resultsDir>/<Index><번호>_<baseTag>_<E0 태그>_X3_asym_J<값>/      case별 결과
%   <resultsDir>/<baseTag>_<E0 태그>_X3_asym_summary/<시각>/           summary.csv, 그림
%   <resultsDir>/<baseTag>_bounds_summary.csv
%   baseTag = stateFixed_max_cot_rajagopalAFO_off_<시각>
% 한 case가 실패해도 기록하고 다음 case를 푼다 (target, bound, equality는 바꾸지 않는다).
%
% 플러그인이 없거나 C++ 소스가 바뀌었으면 먼저 빌드 (빌드 후 MATLAB을 새로 시작):
%   run(fullfile('..', 'plugins', 'osimFixedEffortPropulsion', 'build_osimFixedEffortPropulsion.m'))
clc; clear; close all;
thisScriptDir = fileparts(mfilename('fullpath'));
repoRoot = fileparts(thisScriptDir);
addpath(genpath(fullfile(repoRoot, 'src')));

%% ===== 사용자 입력 ===== %%
Index = 'R';                         % 결과 폴더 접두어. 같은 Index의 마지막 번호 다음부터 이어간다
cfg.model = 'rajagopal_AFO.osim';
cfg.objectiveMode = 'maxSpeedStateFixed';
cfg.energyMetric = 'metabolicCoT';
E0_CoT = [5.6];                      % metabolic CoT target [J/kg/m]
cfg.propulsionMetric = 'impulse';
% J 제약 형태 (impulse: 한 주기 양발 합산 gated propulsive impulse [N*s])
%   'equality' : P_gate = J_k, cfg.J_targets의 각 값이 독립 case (cfg.J_bounds = [])
%   'range'    : lower_k <= P_gate <= upper_k, cfg.J_bounds의 각 행이 독립 case (cfg.J_targets = [])
%                예) cfg.J_bounds = [30, 34; 34, 38];   (사용법 예시, 실제 값은 직접 입력)
cfg.propulsionConstraintMode = 'equality';
cfg.J_targets = [32, 36, 40];
cfg.J_bounds = [];
cfg.gaitMode = 'modeAsym';           % Rajagopal profile은 modeAsym만 지원
cfg.optMode = 'modeOff';             % AFO = 0 (Rajagopal profile은 modeOff만 지원)

% Bound 세트 (반주기 기준 입력. modeAsym에서 x2 한 번 적용)
boundSets = struct( ...
    'name',      {'X3'}, ...
    'timeBound', {[0.25, 0.8]}, ...  % -> 최종 시간 [0.5, 1.6] s
    'distBound', {[0.4, 1.5]});      % -> pelvis_tx(tf) [0.8, 3.0] m

% 초기 trajectory와 대응 GRF (원본, 수정하지 않음)
guessSource = fullfile(repoRoot, 'inputs', 'guess_init_rajagopal.sto');
grfSource = fullfile(repoRoot, 'inputs', 'GRF_init_rajagopal.sto');

% 기타 설정 (기존 stateFixed와 같다)
cfg.speedWeight = 1;                 % 평균속도 cost weight 크기 (방향은 objectiveMode)
cfg.gateSigma = 1.0;                 % propulsion gate 폭 sigma [N]
cfg.gateThreshold = 5.0;             % propulsion gate 중심 F0 [N]
cfg.meshIntervals = 300;
cfg.convergenceTolerance = 5e-5;
cfg.constraintTolerance = 5e-5;
cfg.maxIterations = 5000;
cfg.energyTol = 1e-5;                % |CoT - E0| 결과 검증 허용오차 [J/kg/m] (제약 폭이 아님)
cfg.propulsionTol = 1e-2;            % propulsion 위반량 max(lower - J, J - upper, 0) 결과 검증 허용오차 [N*s]
                                     % (equality: |P_gate - J|; solver tolerance·구간 폭과 별개)
cfg.propulsionBoundTol = 1e-2;       % 결과가 lower/upper(또는 target)에 붙었다고 표시하는 허용오차 [N*s]
cfg.continuation = false;            % false: 모든 case가 같은 정렬 초기값에서 시작
cfg.runOptimizations = true;         % false: 같은 runTag의 기존 결과만 요약 (baseTagOverride 지정)
baseTagOverride = '';                % 재요약 시 기존 baseTag (예: 'stateFixed_max_cot_rajagopalAFO_off_20261111_190000')
cfg.skipSolve = false;               % true: 문제 구성·검사·후처리만 점검 (solve 없음, Verdict "Setup check")
cfg.resultsDir = fullfile(repoRoot, 'results');
cfg.Index = Index;
cfg.caseNameUsesTargetValue = true;  % case 이름에 J 값 (equality ..._asym_J32, range는 항상 ..._asym_JR30to34)
cfg.targetSource = 'user input (rajagopal_AFO maxSpeed CoT sweep)';
% (cfg.lumbarControlBound는 2D /lumbarAct 전용이므로 설정하지 않는다)

E0_list = E0_CoT;
profile = moco_WoC_modelProfile(cfg.model);
modelPath = fullfile(repoRoot, 'models', cfg.model);
% propulsion 입력을 실행 전에 검사하고 case 목록을 만든다 (equality: J_targets 값, range: J_bounds 행)
propCases = WoC_moco_propulsionCases(cfg);
propUnit = 'N*s';
if strcmp(cfg.propulsionMetric, 'meanForce'), propUnit = 'N'; end

%% ===== 실행 태그와 작업용 초기값 사본 =====
metricTag = '';
if strcmp(cfg.propulsionMetric, 'meanForce'), metricTag = '_mpf'; end
if strcmp(propCases(1).mode, 'range'), metricTag = [metricTag '_jrange']; end
energyTag = '';
if strcmp(cfg.energyMetric, 'metabolicCoT'), energyTag = '_cot'; end
if isempty(baseTagOverride)
    baseTag = sprintf('stateFixed_%s%s%s_rajagopalAFO_%s_%s', erase(cfg.objectiveMode, 'SpeedStateFixed'), ...
        metricTag, energyTag, lower(erase(cfg.optMode, 'mode')), ...
        char(datetime('now', 'Format', 'yyyyMMdd_HHmmss')));
else
    baseTag = baseTagOverride;
end
inputsDir = fullfile(cfg.resultsDir, [baseTag '_inputs']);
if isfolder(inputsDir)
    if cfg.runOptimizations
        error('run_stateFixed_rajagopal:Exists', '작업 폴더가 이미 있습니다: %s', inputsDir);
    end
    % 재요약: 기존 정렬 사본을 그대로 쓰고 offset만 다시 계산한다 (기존 파일을 덮어쓰지 않음).
    align = moco_WoC_alignGaitStart(guessSource, grfSource, fullfile(tempdir, [baseTag '_inputs_check']), ...
        profile, 'rajagopal_init_aligned');
    align.kinematicsPath = fullfile(inputsDir, 'rajagopal_init_aligned_kinematics.sto');
    align.grfPath = fullfile(inputsDir, 'rajagopal_init_aligned_GRF.sto');
else
    align = moco_WoC_alignGaitStart(guessSource, grfSource, inputsDir, profile, 'rajagopal_init_aligned');
end
fprintf(['\n정렬한 초기값 사본: %s\n  pelvis_tx offset %.6g m, pelvis_tz offset %.6g m 제거 ', ...
         '(GRF CoP x/z 동일 이동), t = %.4g-%.4g s, %d points\n'], align.kinematicsPath, ...
         align.offsetX, align.offsetZ, align.time, align.numTimes);
cfg.guessSto = repmat({align.kinematicsPath}, 1, numel(propCases));

%% ===== 초기값 지표 (target 규모 확인, 실패로 처리하지 않음) =====
% 실행과 같은 대사 모델·접촉력 목록으로 정렬된 초기값의 CoT와 P_gate를 계산하고,
% P_gate는 정렬된 GRF 파일에서도 독립적으로 다시 적분한다.
guessMetrics = moco_WoC_evalGaitMetrics(modelPath, align.kinematicsPath, 1, ...
    cfg.gateSigma, cfg.gateThreshold);
grfIndep = gatedImpulseFromGrf(align.grfPath, cfg.gateSigma, cfg.gateThreshold);
fprintf(['\n초기값 (%s, model %s):\n  T = %.4g s, pelvis_tx 변위 = %.4g m, v_avg = %.4g m/s\n', ...
         '  metabolic CoT = %.6g J/kg/m (Simpson; trapz %.6g; E_met %.5g J, mass %.4g kg, ', ...
         'd_COM %.5g m, basal %.4g W, %d muscles)\n', ...
         '  P_gate = %.6g N*s (Simpson; trapz %.6g; GRF 파일 독립 재계산 %.6g), ', ...
         'max(F,0) impulse %.5g N*s, Tplus %.4g s\n'], ...
    guessSource, cfg.model, guessMetrics.duration, guessMetrics.distance, guessMetrics.v_avg, ...
    guessMetrics.CoT_simpson, guessMetrics.CoT_trapz, guessMetrics.metabolic_energy_simpson, ...
    guessMetrics.mass_kg, guessMetrics.com_displacement, guessMetrics.basal_rate_W, ...
    guessMetrics.n_metabolic_muscles, guessMetrics.P_gate_simpson, guessMetrics.P_gate_trapz, ...
    grfIndep.P_gate, guessMetrics.P_physical_trapz, guessMetrics.Tplus_simpson);
guessJ = guessMetrics.P_gate_simpson;
if strcmp(cfg.propulsionMetric, 'meanForce'), guessJ = guessMetrics.mean_propulsive_force; end
guessCheck = WoC_moco_propulsionCheck(guessJ * ones(1, numel(propCases)), [propCases.lower], ...
    [propCases.upper], cfg.propulsionTol, cfg.propulsionBoundTol);
fprintf('  target residual (초기값 - target): CoT %s J/kg/m\n', mat2str(guessMetrics.CoT_simpson - E0_list, 5));
for k = 1:numel(propCases)
    fprintf('  propulsion case %d (%s %s): 초기값 J %.6g, 위반량 %.4g %s\n', k, propCases(k).label, ...
        propUnit, guessJ, guessCheck.violation(k), propUnit);
end

% basal 항만으로도 CoT >= basal / (mass * v_avg) 이므로 속도 상한에서의 이 값보다
% 작은 target은 풀 수 없다.
for b = 1:numel(boundSets)
    vCeil = boundSets(b).distBound(2) / boundSets(b).timeBound(1);
    cotFloor = guessMetrics.basal_rate_W / (guessMetrics.mass_kg * vCeil);
    if any(E0_list <= cotFloor)
        error('run_stateFixed_rajagopal:CoTTargetTooLow', ['E0_CoT %s 중 bound 세트 %s의 ', ...
            'basal-only CoT 하한 %.4g J/kg/m (v <= %.3g m/s) 이하인 값이 있습니다.'], ...
            mat2str(E0_list), boundSets(b).name, cotFloor, vCeil);
    end
end
if cfg.runOptimizations
    writeRunConfig(fullfile(inputsDir, 'run_config.txt'), cfg, E0_list, boundSets, align, ...
        guessMetrics, grfIndep, profile);
end

%% ===== E0 x bound 세트별 실행 =====
summaries = cell(numel(E0_list), numel(boundSets));
for e = 1:numel(E0_list)
    cfg.E0 = E0_list(e);
    e0Tag = ['E' strrep(sprintf('%g', cfg.E0), '.', 'p')];
    for b = 1:numel(boundSets)
        cfg.timeBound = boundSets(b).timeBound;
        cfg.distBound = boundSets(b).distBound;
        cfg.runTag = sprintf('%s_%s_%s', baseTag, e0Tag, boundSets(b).name);
        boundScale = 1 + strcmp(cfg.gaitMode, 'modeAsym');
        vFloor = cfg.distBound(1) / cfg.timeBound(2);
        vCeil = cfg.distBound(2) / cfg.timeBound(1);
        fprintf(['\n##### %s %s, E0 = %g J/kg/m, bound set %s: input time %s s, dist %s m ', ...
                 '-> applied final time %s s, pelvis_tx(tf) %s m (v range %.3f-%.3f m/s) #####\n'], ...
            cfg.model, cfg.objectiveMode, cfg.E0, boundSets(b).name, mat2str(cfg.timeBound), ...
            mat2str(cfg.distBound), mat2str(cfg.timeBound * boundScale), ...
            mat2str(cfg.distBound * boundScale), vFloor, vCeil);
        for k = 1:numel(propCases)
            fprintf('  case %d: %s %s (%s, %s, %s): guess %s\n', k, propCases(k).label, propUnit, ...
                cfg.propulsionMetric, cfg.gaitMode, cfg.optMode, align.kinematicsPath);
        end
        try
            s = WoC_moco_runStateFixedCases(cfg);
            s.BoundSet = repmat(string(boundSets(b).name), height(s), 1);
            s.VFloor_mps = repmat(vFloor, height(s), 1);
            s.VCeil_mps = repmat(vCeil, height(s), 1);
            summaries{e, b} = s;
        catch ME
            % 한 세트가 실패해도 다음 세트를 계속 돌린다.
            warning('run_stateFixed_rajagopal:BoundSetFailed', 'E0 = %g, bound set %s failed: %s', ...
                cfg.E0, boundSets(b).name, ME.message);
        end
    end
end

summaries = summaries(~cellfun(@isempty, summaries));
if isempty(summaries)
    warning('run_stateFixed_rajagopal:NoSummary', '요약할 bound 세트 결과가 없습니다.');
else
    allSummary = vertcat(summaries{:});
    writetable(allSummary, fullfile(cfg.resultsDir, [baseTag '_bounds_summary.csv']));
    disp(allSummary(:, {'Model', 'EnergyMetric', 'E_target', 'E_actual', 'BoundSet', 'Case', ...
        'Verdict', 'PropulsionConstraintMode', 'PropulsionLower', 'PropulsionUpper', ...
        'PropulsionActual', 'PropulsionPass', 'PropulsionBoundHits', 'v_avg_mps', 'TimeBoundInput_s', ...
        'TimeBound_s', 'DistBoundInput_m', 'DistBound_m', 'PhaseDuration_s', 'PhaseDistance_m', ...
        'BoundHits'}));
end


%% ===== local functions =====
function out = gatedImpulseFromGrf(grfPath, sigma, F0)
% GRF 파일의 발별 AP force(ground_force_*_vx)로 P_gate를 사다리꼴 적분 (OpenSim 미사용).
    fid = fopen(grfPath);
    cleaner = onCleanup(@() fclose(fid));
    line = '';
    while ~strcmp(strtrim(line), 'endheader'), line = fgetl(fid); end
    labels = strsplit(strtrim(fgetl(fid)), '\t');
    data = textscan(fid, repmat('%f', 1, numel(labels)), 'CollectOutput', true);
    data = data{1};
    t = data(:, 1);
    F = [data(:, strcmp(labels, 'ground_force_r_vx')), data(:, strcmp(labels, 'ground_force_l_vx'))];
    g = 1 ./ (1 + exp(-(F - F0) / sigma));
    out.P_gate = sum(trapz(t, F .* g));
    out.P_physical = sum(trapz(t, max(F, 0)));
    out.Tplus = sum(trapz(t, g));
end


function writeRunConfig(path, cfg, E0_list, boundSets, align, gm, grfIndep, profile)
    fid = fopen(path, 'w');
    cleaner = onCleanup(@() fclose(fid));
    boundScale = 1 + strcmp(cfg.gaitMode, 'modeAsym');
    fprintf(fid, 'model: %s (profile %s)\n', cfg.model, profile.name);
    fprintf(fid, 'objective: %s (cost -%g * v_avg only), optMode %s, gaitMode %s\n', ...
        cfg.objectiveMode, cfg.speedWeight, cfg.optMode, cfg.gaitMode);
    propCases = WoC_moco_propulsionCases(cfg);
    fprintf(fid, 'energy: %s, E0 = %s J/kg/m (endpoint equality); propulsion: %s, %s constraint, cases: %s\n', ...
        cfg.energyMetric, mat2str(E0_list), cfg.propulsionMetric, propCases(1).mode, ...
        strjoin({propCases.label}, '; '));
    for b = 1:numel(boundSets)
        fprintf(fid, ['bound set %s: input time %s s, dist %s m -> applied final time %s s, ', ...
                      'pelvis_tx(tf) %s m, t0 = 0\n'], boundSets(b).name, mat2str(boundSets(b).timeBound), ...
            mat2str(boundSets(b).distBound), mat2str(boundSets(b).timeBound * boundScale), ...
            mat2str(boundSets(b).distBound * boundScale));
    end
    fprintf(fid, 'gate: sigma %g N, F0 %g N; contacts per foot: %s\n', cfg.gateSigma, ...
        cfg.gateThreshold, profile.contactDescription);
    fprintf(fid, 'solver: mesh %d, convergence tol %g, constraint tol %g, max iterations %d\n', ...
        cfg.meshIntervals, cfg.convergenceTolerance, cfg.constraintTolerance, cfg.maxIterations);
    fprintf(fid, ['verification tolerances: energy %g J/kg/m, propulsion violation %g N*s; ', ...
                  'propulsion bound-hit tolerance %g N*s\n'], cfg.energyTol, cfg.propulsionTol, cfg.propulsionBoundTol);
    fprintf(fid, 'initial guess (aligned copy): %s\n  source %s, offsets x %.17g m, z %.17g m\n', ...
        align.kinematicsPath, align.sourceGuess, align.offsetX, align.offsetZ);
    fprintf(fid, ['initial guess metrics: T %.6g s, distance %.6g m, v_avg %.6g m/s, CoT %.6g J/kg/m, ', ...
                  'P_gate %.6g N*s (GRF file %.6g N*s)\n'], gm.duration, gm.distance, gm.v_avg, ...
        gm.CoT_simpson, gm.P_gate_simpson, grfIndep.P_gate);
end
