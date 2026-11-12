function summary = WoC_moco_runStateFixedCases(cfg)
% WoC_moco_runStateFixedCases
%
%   stateFixed 실행기 (min/maxSpeedStateFixed: E와 propulsion을 고정하고 평균속도를 최소/최대화).
%   같은 E0에 대해 propulsion target J_k마다 하나씩 다음 문제를 독립적으로 푼다
%   (opts.mocoObjectiveMode = cfg.objectiveMode):
%
%     minimize (minSpeedStateFixed) 또는 maximize (maxSpeedStateFixed)
%                 w_v * v_avg,  v_avg = (pelvis_tx(tf) - pelvis_tx(t0)) / (tf - t0)
%     subject to  E = E0, P_gate = J_k (또는 평균 추진력 = J_k), 기존 동역학/주기성/대칭성/bounds
%
%   각 case 결과를 저장된 궤적에서 control과 접촉력으로 다시 적분해 표/CSV와
%   그림으로 저장한다. solver 수렴 여부와 equality 충족 여부를 따로 판정하며,
%   실패 case에서 target·bound·제약을 바꿔 다시 풀지 않는다. 한 case의 실행이
%   수렴하지 않거나 실패해도 기록만 하고 다음 case를 계속 푼다.
%
%   propulsion 제약은 cfg.propulsionConstraintMode로 고른다 (WoC_moco_propulsionCases):
%     'equality' (기본값, 기존 방식): J_targets의 각 값 J_k가 독립 case, J_actual = J_k
%     'range'    : J_bounds의 각 행 [lower, upper]가 독립 case, lower <= J_actual <= upper
%                  (닫힌 구간, 구간 중심을 target으로 쓰지 않는다)
%   판정은 두 모드 모두 violation = max([lower - J, J - upper, 0]) <= propulsionTol
%   (equality는 lower = upper = J_k)이며, 경계 접촉은 propulsionBoundTol로 따로 표시한다.
%
%   cfg 필드:
%     E0                 : energy target (지표는 energyMetric)
%     propulsionConstraintMode, J_targets (equality), J_bounds (range, n x 2)
%                        : propulsion 제약 (지표는 propulsionMetric). 실행 전에 검사한다.
%     propulsionBoundTol : 결과가 lower/upper(또는 equality target)에 붙었다고 표시하는
%                          허용오차 (없으면 propulsionTol). 판정 허용오차와 별개다.
%     energyMetric       : 'effort' (없으면 이 값, E = (1/d_COM) int sum_c u_c^2 dt [-])
%                          | 'metabolicCoT' (E = int P_met dt / (mass * d_COM) [J/kg/m],
%                          P_met = Bhargava2004 total_metabolic_rate, basal 포함)
%     propulsionMetric   : 'impulse' (없으면 이 값, J = P_gate [N*s, 한 보행주기])
%                          | 'meanForce' (J = 평균 추진력 P_gate / Tplus [N])
%     objectiveMode      : 'minSpeedStateFixed' (없으면 이 값) | 'maxSpeedStateFixed'
%     speedWeight        : 평균속도 cost weight 크기 w_v (없으면 1)
%     lumbarControlBound : lumbarAct control 범위 [lower, upper] [N*m]
%                          (없거나 비어 있으면 모델의 control 범위)
%     targetSource       : target 출처 설명 (CSV에 기록)
%     gateSigma, gateThreshold : propulsion gate sigma, F0 [N]
%     model, optMode, gaitMode, guessSto ('' 이면 WoC_moco_main 기본 guess,
%                          J_targets와 길이가 같은 cell이면 case k가 guessSto{k}에서 시작)
%     timeBound, distBound : 반 걸음 기준 bound (modeAsym은 WoC_moco_main이 x2)
%     meshIntervals, convergenceTolerance, constraintTolerance, maxIterations
%     energyTol, propulsionTol : equality 검증 허용오차 (|E-E0| [energyMetric 단위],
%                          |J 지표 - J| [N*s 또는 N]). energyTol이 없으면 이전 이름
%                          effortTol을 쓴다 (effort 지표에서만)
%     resultsDir, runTag : 결과 루트와 이름. case는 <resultsDir>/<runTag>_<mode>_J<k>,
%                          요약은 <resultsDir>/<runTag>_<mode>_summary/<시각>/ 에
%                          저장한다(이전 요약을 덮어쓰지 않는다)
%     Index              : 결과 폴더 접두어 (예: 'H'). 같은 접두어의 최대 번호 다음부터
%                          case마다 H001_<기존 이름> 형태로 붙인다. 생략/빈 값은 기존 이름.
%                          재요약 시 같은 case 이름의 가장 큰 번호를 읽는다.
%     continuation       : true면 case k가 직전 수렴 case의 해에서 시작
%     runOptimizations   : false면 이미 있는 결과만 요약한다
%
%   반환: summary table (summary.csv와 같은 내용)

    validateEnergyTarget(cfg.E0);
    % propulsion 제약 입력 검사와 case 목록 (equality: J_targets, range: J_bounds의 행)
    cfg.propCases = WoC_moco_propulsionCases(cfg);
    cfg.propulsionConstraintMode = cfg.propCases(1).mode;
    if ~isfield(cfg, 'propulsionBoundTol') || isempty(cfg.propulsionBoundTol)
        cfg.propulsionBoundTol = cfg.propulsionTol;
    end
    if ~isfield(cfg, 'speedWeight'), cfg.speedWeight = 1; end
    if ~isfield(cfg, 'objectiveMode'), cfg.objectiveMode = 'minSpeedStateFixed'; end
    if ~ismember(cfg.objectiveMode, {'minSpeedStateFixed', 'maxSpeedStateFixed'})
        error('cfg.objectiveMode 는 ''minSpeedStateFixed'' 또는 ''maxSpeedStateFixed'' 이어야 합니다.');
    end
    if ~isfield(cfg, 'propulsionMetric'), cfg.propulsionMetric = 'impulse'; end
    if ~ismember(cfg.propulsionMetric, {'impulse', 'meanForce'})
        error('cfg.propulsionMetric 은 ''impulse'' 또는 ''meanForce'' 이어야 합니다.');
    end
    unit = 'N*s';
    if strcmp(cfg.propulsionMetric, 'meanForce'), unit = 'N'; end
    % 에너지 지표가 없던 이전 호출은 effort 지표다.
    if ~isfield(cfg, 'energyMetric') || isempty(cfg.energyMetric), cfg.energyMetric = 'effort'; end
    if ~ismember(cfg.energyMetric, {'effort', 'metabolicCoT'})
        error('cfg.energyMetric 은 ''effort'' 또는 ''metabolicCoT'' 이어야 합니다.');
    end
    if ~isfield(cfg, 'energyTol') || isempty(cfg.energyTol)
        if ~strcmp(cfg.energyMetric, 'effort') || ~isfield(cfg, 'effortTol')
            error('cfg.energyTol (|E - E0| 허용오차, %s 단위)을 지정하세요.', energyUnitOf(cfg.energyMetric));
        end
        cfg.energyTol = cfg.effortTol;
    elseif isfield(cfg, 'effortTol') && strcmp(cfg.energyMetric, 'metabolicCoT')
        warning('WoC_moco_runStateFixedCases:EffortTolIgnored', ...
            'cfg.effortTol은 effort 지표용이므로 metabolicCoT 판정에는 cfg.energyTol만 씁니다.');
    end
    if iscell(cfg.guessSto) && numel(cfg.guessSto) ~= numel(cfg.propCases)
        error('guessSto가 cell이면 propulsion case 수와 길이가 같아야 합니다 (%d vs %d).', ...
            numel(cfg.guessSto), numel(cfg.propCases));
    end
    if ~isfield(cfg, 'Index'), cfg.Index = ''; end
    if ~((ischar(cfg.Index) && (isrow(cfg.Index) || isempty(cfg.Index))) || ...
            (isstring(cfg.Index) && isscalar(cfg.Index)))
        error('cfg.Index는 영문자로 된 접두어 또는 빈 문자열이어야 합니다.');
    end
    indexPrefix = char(cfg.Index);
    if ~isempty(indexPrefix) && isempty(regexp(indexPrefix, '^[A-Za-z]+$', 'once'))
        error('cfg.Index는 영문자로 된 접두어 또는 빈 문자열이어야 합니다.');
    end

    % WoC_moco_main과 같은 절대경로 결과 루트를 저장, 검사, guess, 요약에 쓴다.
    resultsRoot = char(java.io.File(cfg.resultsDir).getCanonicalPath());
    modeTag = lower(erase(cfg.gaitMode, 'mode'));
    isSym = strcmpi(cfg.gaitMode, 'modeSym');
    impulseScale = 1 + isSym;     % modeSym 반주기 적분 -> 한 보행주기
    boundScale = 1 + ~isSym;      % WoC_moco_main/moco_WoC_loop의 modeAsym bound 규칙
    [~, modelName] = fileparts(cfg.model);
    nCase = numel(cfg.propCases);
    if strcmp(cfg.propulsionConstraintMode, 'range')
        % range case 이름은 구간을 쓴다 (예: JR25to35, 32.5 -> 32p5)
        jTags = {cfg.propCases.valueTag};
    else
        jTags = {cfg.propCases.indexTag};   % 기존 규칙 J1, J2, ...
    end
    caseNames = cellfun(@(j) sprintf('%s_%s_%s', cfg.runTag, modeTag, j), jTags, ...
        'UniformOutput', false);
    caseNames = indexedCaseNames(resultsRoot, caseNames, indexPrefix, cfg.runOptimizations);
    caseDirs = fullfile(resultsRoot, caseNames);

    runMessages = strings(nCase, 1);
    if cfg.runOptimizations
        % 기존 결과를 덮어쓰지 않는다 (WoC_moco_main이 실제로 쓸 절대경로 기준).
        existing = caseDirs(isfolder(caseDirs));
        if ~isempty(existing)
            error('결과 폴더가 이미 있습니다 (runTag를 바꾸세요):\n%s', strjoin(existing, '\n'));
        end
        % 잘못된 guess 경로로 case 폴더만 만들고 실패하지 않게 시작 전에 확인한다.
        guessFiles = cellstr(cfg.guessSto);
        missingGuess = guessFiles(~cellfun(@isempty, guessFiles) & ~isfile(guessFiles));
        if ~isempty(missingGuess)
            error('guess 파일이 없습니다:\n%s', strjoin(missingGuess, '\n'));
        end
        guess = cfg.guessSto;
        for k = 1:nCase
            opts = struct();
            opts.gaitMode = cfg.gaitMode;
            opts.mocoObjectiveMode = cfg.objectiveMode;
            opts.mocoEnergyMetric = cfg.energyMetric;
            opts.mocoEnergyTarget = cfg.E0;
            if isfield(cfg, 'lumbarControlBound')
                opts.mocoLumbarControlBound = cfg.lumbarControlBound;
            end
            pc = cfg.propCases(k);
            opts.mocoPropulsionConstraintMode = pc.mode;
            if strcmp(pc.mode, 'range')
                opts.mocoPropulsionBounds = [pc.lower, pc.upper];   % scalar target 없음
            else
                opts.mocoPropulsionTarget = pc.target;
            end
            opts.mocoPropulsionMetric = cfg.propulsionMetric;
            opts.mocoPropulsionGateSigma = cfg.gateSigma;
            opts.mocoPropulsionGateThreshold = cfg.gateThreshold;
            opts.mocoSpeedWeight = cfg.speedWeight;
            opts.mocoTimeBound = cfg.timeBound;
            opts.mocoDistBound = cfg.distBound;
            opts.mocoMeshIntervals = cfg.meshIntervals;
            opts.mocoConvergenceTolerance = cfg.convergenceTolerance;
            opts.mocoConstraintTolerance = cfg.constraintTolerance;
            opts.mocoMaxIterations = cfg.maxIterations;
            if iscell(guess), opts.guessInitSto = guess{k}; else, opts.guessInitSto = guess; end
            opts.resultsDir = resultsRoot;
            fprintf('\n===== %s case %d/%d: E0=%.6g %s (%s), propulsion %s %s (%s) =====\n', ...
                cfg.objectiveMode, k, nCase, cfg.E0, energyUnitOf(cfg.energyMetric), ...
                cfg.energyMetric, pc.label, unit, cfg.propulsionMetric);
            try
                WoC_moco_main(cfg.model, 1, cfg.optMode, caseNames{k}, opts);
                runMessages(k) = "completed";
            catch ME
                runMessages(k) = string(ME.message);
                warning('WoC_moco_runStateFixedCases:CaseFailed', ...
                    'Case %s did not finish: %s', caseNames{k}, ME.message);
            end
            if cfg.continuation
                solFile = solutionFile(caseDirs{k}, isSym, false);
                if isfile(solFile)
                    guess = solFile;
                end
            end
        end
    end

    rows = cell(nCase, 1);
    for k = 1:nCase
        try
            rows{k} = summarizeCase(cfg, caseDirs{k}, caseNames{k}, k, isSym, ...
                impulseScale, boundScale, modelName);
        catch ME
            % 손상된 파일 하나 때문에 다른 case의 요약이 사라지지 않게 한다.
            rows{k} = emptyRow(cfg, caseDirs{k}, caseNames{k}, k, impulseScale, boundScale);
            rows{k}.Verdict = "Summary error";
            rows{k}.SummaryError = string(ME.message);
        end
        rows{k}.RunMessage = runMessages(k);
    end
    summary = struct2table([rows{:}], 'AsArray', true);

    summaryDir = fullfile(resultsRoot, sprintf('%s_%s_summary', cfg.runTag, modeTag), ...
        char(datetime('now', 'Format', 'yyyyMMdd_HHmmss')));
    mkdir(summaryDir);
    writetable(summary, fullfile(summaryDir, 'summary.csv'));
    writeDefinitions(fullfile(summaryDir, 'definitions.txt'), cfg, impulseScale, boundScale);

    fprintf('\n===== %s summary (%s) =====\n', cfg.objectiveMode, summaryDir);
    % propulsion은 모드와 상관없이 허용 구간 [lower, upper]와 실제 값으로 보인다
    % (equality는 lower = upper = target).
    propulsionColumns = {'PropulsionConstraintMode', 'PropulsionLower', 'PropulsionUpper', ...
        'PropulsionActual', 'PropulsionConstraintValue', 'PropulsionViolation', 'PropulsionPass', ...
        'PropulsionBoundHits'};
    disp(summary(:, [{'Case', 'EnergyMetric', 'PropulsionDefinition', 'Verdict', 'SolverStatus', ...
        'Converged', 'E_target', 'E_actual', 'E_residual', 'E_pass', 'E_constraint_value'}, ...
        propulsionColumns, {'v_avg_mps', 'BoundHits'}]));

    try
        plotCases(summary, summaryDir);
    catch ME
        warning('WoC_moco_runStateFixedCases:PlotFailed', '그림 저장 실패: %s', ME.message);
    end
end


function caseNames = indexedCaseNames(resultsRoot, caseNames, indexPrefix, runOptimizations)
% 결과 루트의 폴더만 확인한다. 빠진 번호를 재사용하지 않고 최대 번호부터 이어간다.
    if isempty(indexPrefix), return; end
    entries = dir(resultsRoot);
    folderNames = {entries([entries.isdir]).name};
    pattern = ['^' regexptranslate('escape', indexPrefix) '(\d+)_(.+)$'];
    tokens = regexpi(folderNames, pattern, 'tokens', 'once');
    matched = ~cellfun(@isempty, tokens);
    folderNames = folderNames(matched);
    tokens = tokens(matched);
    numbers = cellfun(@(t) str2double(t{1}), tokens);
    originalNames = cellfun(@(t) t{2}, tokens, 'UniformOutput', false);

    if runOptimizations
        lastNumber = max([0, numbers]);
        for k = 1:numel(caseNames)
            caseNames{k} = sprintf('%s%03d_%s', indexPrefix, lastNumber + k, caseNames{k});
        end
    else
        % 저장된 결과를 다시 요약할 때 새 번호를 만들지 않는다.
        % 접두어가 없는 과거 결과는 원래 case 이름으로 계속 읽을 수 있다.
        for k = 1:numel(caseNames)
            candidates = find(strcmp(originalNames, caseNames{k}));
            if ~isempty(candidates)
                [~, latest] = max(numbers(candidates));
                caseNames{k} = folderNames{candidates(latest)};
            end
        end
    end
end


function validateEnergyTarget(E0)
% 실험 target은 사용자가 직접 넣는다. 비어 있거나 잘못된 값으로 실행하지 않는다.
% (propulsion 입력은 WoC_moco_propulsionCases가 검사한다.)
    if isempty(E0)
        error('E0가 입력되지 않았습니다. 실행 스크립트 상단의 E0에 실제 target 값을 입력하세요.');
    end
    if ~isscalar(E0) || ~isnumeric(E0) || ~isfinite(E0) || E0 <= 0
        error('E0는 양의 유한 scalar여야 합니다.');
    end
end


function f = solutionFile(resultDir, isSym, unseal)
    suffix = '';
    if unseal, suffix = '_unseal'; end
    if isSym
        name = sprintf('moco_WoC_Solution_iter01%s_kinematics_half.sto', suffix);
    else
        name = sprintf('moco_WoC_Solution_iter01%s_kinematics.sto', suffix);
    end
    f = fullfile(resultDir, 'result_1', 'moco_result', name);
end


function row = emptyRow(cfg, resultDir, caseName, k, impulseScale, boundScale)
    row = struct('Case', string(caseName), 'CaseIndex', k, ...
        'TargetSource', string(cfg.targetSource), ...
        'ObjectiveMode', string(cfg.objectiveMode), ...
        'EnergyMetric', string(cfg.energyMetric), ...
        'EnergyUnit', string(energyUnitOf(cfg.energyMetric)), ...
        'EnergyDefinition', string(energyDefinitionOf(cfg.energyMetric)), ...
        'PropulsionMetric', string(cfg.propulsionMetric), 'PropulsionDefinition', "", ...
        'Verdict', "", 'SolverStatus', "missing", 'Converged', false, ...
        'Iterations', NaN, 'SolverTime_s', NaN, 'Objective', NaN, ...
        'ObjectiveTerms', "", 'OnlySpeedCost', false, ...
        'E_target', cfg.E0, 'E_actual', NaN, 'E_residual', NaN, 'E_pass', false, ...
        'E_constraint_value', NaN, 'E_trapz', NaN, ...
        'P_gate_target', jTarget(cfg, k, 'impulse'), 'P_gate_actual', NaN, ...
        'P_gate_residual', NaN, 'P_gate_pass', false, 'P_gate_constraint_value', NaN, ...
        'P_physical', NaN, 'P_gate_minus_physical', NaN, 'Tplus_s', NaN, ...
        'MeanPropulsiveForce_N', NaN, 'MPF_target', jTarget(cfg, k, 'meanForce'), 'MPF_residual', NaN, ...
        'MPF_pass', false, 'MPF_constraint_value', NaN, 'RunPropulsionTarget', NaN, ...
        'PropulsionConstraintMode', string(cfg.propCases(k).mode), ...
        'PropulsionUnit', string(propulsionUnitOf(cfg.propulsionMetric)), ...
        'PropulsionLower', cfg.propCases(k).lower, 'PropulsionUpper', cfg.propCases(k).upper, ...
        'PropulsionActual', NaN, 'PropulsionConstraintValue', NaN, 'PropulsionViolation', NaN, ...
        'PropulsionConstraintViolation', NaN, 'PropulsionPass', false, 'PropulsionBoundHits', "", ...
        'PropulsionBoundTol', cfg.propulsionBoundTol, 'PropulsionMetadataIssue', "", ...
        'v_avg_mps', NaN, 'PhaseDistance_m', NaN, 'PhaseDuration_s', NaN, ...
        'StrideLength_m', NaN, 'StrideTime_s', NaN, 'BoundHits', "", ...
        'Effort_simpson', NaN, 'Effort_trapz', NaN, 'Effort_constraint_value', NaN, ...
        'Effort_muscle', NaN, 'Effort_lumbar', NaN, 'Effort_afo', NaN, ...
        'MetabolicEnergy_J', NaN, 'MetabolicEnergy_trapz_J', NaN, 'MetabolicEnergy_state_J', NaN, ...
        'Mass_kg', NaN, 'COMDisplacement_m', NaN, 'CoT_simpson', NaN, 'CoT_trapz', NaN, ...
        'CoT_constraint_value', NaN, 'CoT_fullStride', NaN, 'BasalRate_W', NaN, ...
        'MetabolicMuscles', NaN, 'MetabolicsSettings', "", ...
        'P_gate_trapz', NaN, 'Tplus_trapz_s', NaN, ...
        'Indep_v_avg_mps', NaN, 'Indep_P_gate', NaN, 'Indep_Tplus_s', NaN, ...
        'Indep_P_physical', NaN, 'Indep_Effort_noAFO', NaN, ...
        'GateSigma_N', cfg.gateSigma, 'GateThreshold_N', cfg.gateThreshold, ...
        'ImpulseScale', impulseScale, 'SpeedWeight', cfg.speedWeight, ...
        'Model', string(cfg.model), 'OptMode', string(cfg.optMode), ...
        'GaitMode', string(cfg.gaitMode), 'Guess', "", ...
        'MeshIntervals', cfg.meshIntervals, 'ConvergenceTol', cfg.convergenceTolerance, ...
        'ConstraintTol', cfg.constraintTolerance, 'MaxIterations', cfg.maxIterations, ...
        'TimeBound_s', string(mat2str(cfg.timeBound * boundScale)), ...
        'DistBound_m', string(mat2str(cfg.distBound * boundScale)), ...
        'EnergyTol', cfg.energyTol, 'PropulsionTol', cfg.propulsionTol, ...
        'ResultDir', string(resultDir), 'RunMessage', "", 'SummaryError', "");
end


function value = jTarget(cfg, k, metric)
% case k의 equality target이 metric의 target이면 그 값, 아니면 NaN (실행 전 기본 행).
% range case는 단일 target이 없으므로 NaN이다.
    value = NaN;
    if strcmp(cfg.propulsionMetric, metric), value = cfg.propCases(k).target; end
end


function unit = propulsionUnitOf(metric)
    if strcmp(metric, 'meanForce'), unit = 'N'; else, unit = 'N*s'; end
end


function row = summarizeCase(cfg, resultDir, caseName, k, isSym, impulseScale, ...
        boundScale, modelName)
    row = emptyRow(cfg, resultDir, caseName, k, impulseScale, boundScale);

    unseal = false;
    solFile = solutionFile(resultDir, isSym, false);
    if ~isfile(solFile)
        solFile = solutionFile(resultDir, isSym, true);
        unseal = true;
    end

    % 실행 당시 설정: target, gate 파라미터, propulsion 정의를 그대로 쓴다.
    params = readParams(fullfile(resultDir, 'result_1', 'moco_result', ...
        'moco_WoC_Solution_iter01_parameters.csv'));
    if isfield(params, 'guess_sto'), row.Guess = string(params.guess_sto); end
    if isfield(params, 'mocoObjectiveMode')
        row.ObjectiveMode = string(params.mocoObjectiveMode);
    end
    % 에너지 지표 항목이 없는 이전 실행은 effort 지표로 풀었다.
    runEnergyMetric = 'effort';
    if isfield(params, 'mocoEnergyMetric') && ~isempty(params.mocoEnergyMetric)
        runEnergyMetric = params.mocoEnergyMetric;
    end
    row.EnergyMetric = string(runEnergyMetric);
    row.EnergyUnit = string(energyUnitOf(runEnergyMetric));
    row.EnergyDefinition = string(energyDefinitionOf(runEnergyMetric));
    if isfield(params, 'mocoEnergyTarget') && ~isempty(params.mocoEnergyTarget)
        row.E_target = str2double(params.mocoEnergyTarget);
    elseif isfield(params, 'mocoEffortTarget') && strcmp(runEnergyMetric, 'effort')
        row.E_target = str2double(params.mocoEffortTarget);
    end
    if isfield(params, 'mocoPropulsionTarget')
        row.RunPropulsionTarget = str2double(params.mocoPropulsionTarget);
    end
    % mocoSpeedWeight가 생기기 전 실행은 weight 1로 풀었다.
    row.SpeedWeight = 1;
    if isfield(params, 'mocoSpeedWeight')
        row.SpeedWeight = str2double(params.mocoSpeedWeight);
    end
    isGateRun = isfield(params, 'propulsion_definition') && ...
        startsWith(params.propulsion_definition, {'gate_Iplus', 'gate_mean_force'});
    if isGateRun
        row.GateSigma_N = str2double(params.mocoPropulsionGateSigma);
        row.GateThreshold_N = str2double(params.mocoPropulsionGateThreshold);
        if startsWith(params.propulsion_definition, 'gate_mean_force')
            row.PropulsionDefinition = "gate_mean_force";
            row.PropulsionMetric = "meanForce";
            row.P_gate_target = NaN;
            row.MPF_target = row.RunPropulsionTarget;
        else
            row.PropulsionDefinition = "gate_Iplus";
            row.PropulsionMetric = "impulse";
            row.P_gate_target = row.RunPropulsionTarget;
            row.MPF_target = NaN;
        end
    elseif isfield(params, 'mocoObjectiveMode') && ...
            strcmp(params.mocoObjectiveMode, 'minSpeedStateFixed')
        % 이전 버전: smooth max(softplus) impulse를 제약했던 실행이다.
        row.PropulsionDefinition = "softplus_legacy";
        row.P_gate_target = NaN;
        row.MPF_target = NaN;
    else
        row.PropulsionDefinition = "unknown";
        row.P_gate_target = NaN;
        row.MPF_target = NaN;
    end
    % J 제약 형태와 허용 구간은 현재 cfg가 아니라 실행 당시 parameters로 정한다.
    % 이 항목이 없는 이전 결과는 equality (lower = upper = mocoPropulsionTarget).
    % 정보가 없거나 판정에 쓸 수 없으면 PropulsionMetadataIssue에 이유를 남기고
    % 판정하지 않는다 (PropulsionPass false, Verdict "Propulsion metadata ...").
    [runMode, row.PropulsionLower, row.PropulsionUpper, metaIssue] = runPropulsionInterval(params);
    row.PropulsionConstraintMode = string(runMode);
    row.PropulsionUnit = string(propulsionUnitOf(char(row.PropulsionMetric)));
    if ~isGateRun
        % softplus 등 이전 정의의 target을 gate 지표 구간으로 판정하지 않는다
        % (Verdict "Not a gate run"이 먼저 정해지므로 메타데이터 검사도 하지 않는다).
        row.PropulsionLower = NaN;
        row.PropulsionUpper = NaN;
        metaIssue = "";
    end
    row.PropulsionMetadataIssue = string(metaIssue);
    if strcmp(runMode, 'range')
        % range에는 단일 target이 없다 (구간 중심을 target으로 쓰지 않는다).
        row.RunPropulsionTarget = NaN;
        row.P_gate_target = NaN;
        row.MPF_target = NaN;
    end
    if ~isfile(solFile)
        row.Verdict = "No solution file";
        return;
    end

    hdr = readStoHeader(solFile);
    row.SolverStatus = string(hdr.status);
    row.Converged = ~unseal && strcmp(hdr.success, 'true');
    row.Iterations = str2double(hdr.num_iterations);
    row.SolverTime_s = str2double(hdr.solver_duration);
    row.Objective = str2double(hdr.objective);
    termNames = fieldnames(hdr);
    termNames = termNames(startsWith(termNames, 'objective_'));
    row.ObjectiveTerms = strjoin(erase(termNames, 'objective_'), ';');

    modelPath = fullfile(resultDir, 'result_1', 'analy_result', [modelName '_1.osim']);
    m = moco_WoC_evalGaitMetrics(modelPath, solFile, impulseScale, ...
        row.GateSigma_N, row.GateThreshold_N);
    % maxSpeedStateFixed는 weight -w_v로 풀었으므로 objective = -w_v * v_avg 이다.
    speedCost = row.SpeedWeight * m.v_avg;
    if row.ObjectiveMode == "maxSpeedStateFixed", speedCost = -speedCost; end
    row.OnlySpeedCost = isequal(termNames, {'objective_average_speed'}) && ...
        abs(row.Objective - speedCost) <= 1e-6 * max(1, abs(speedCost));
    % E_*: 실행 당시 에너지 지표 (effort 또는 metabolic CoT). 다른 지표 값은 별도 열.
    isCoT = row.EnergyMetric == "metabolicCoT";
    if isCoT
        row.E_actual = m.CoT_simpson;
        row.E_trapz = m.CoT_trapz;
    else
        row.E_actual = m.E_simpson;
        row.E_trapz = m.E_trapz;
    end
    row.E_residual = row.E_actual - row.E_target;
    row.E_pass = abs(row.E_residual) <= cfg.energyTol;
    row.Effort_simpson = m.E_simpson;
    row.Effort_trapz = m.E_trapz;
    row.Effort_muscle = m.E_muscle_simpson;
    row.Effort_lumbar = m.E_lumbar_simpson;
    row.Effort_afo = m.E_afo_simpson;
    row.MetabolicEnergy_J = m.metabolic_energy_simpson;
    row.MetabolicEnergy_trapz_J = m.metabolic_energy_trapz;
    row.Mass_kg = m.mass_kg;
    row.COMDisplacement_m = m.com_displacement;
    row.CoT_simpson = m.CoT_simpson;
    row.CoT_trapz = m.CoT_trapz;
    row.BasalRate_W = m.basal_rate_W;
    row.MetabolicMuscles = m.n_metabolic_muscles;
    row.MetabolicsSettings = string(m.metabolics_settings);
    row.P_gate_actual = m.P_gate_simpson;
    row.P_gate_residual = m.P_gate_simpson - row.P_gate_target;
    row.P_gate_pass = abs(row.P_gate_residual) <= cfg.propulsionTol;
    row.P_physical = m.P_physical_trapz;
    row.P_gate_minus_physical = m.P_gate_trapz - m.P_physical_trapz;
    row.Tplus_s = m.Tplus_simpson;
    row.MeanPropulsiveForce_N = m.mean_propulsive_force;
    row.MPF_residual = m.mean_propulsive_force - row.MPF_target;
    row.MPF_pass = abs(row.MPF_residual) <= cfg.propulsionTol;
    row.v_avg_mps = m.v_avg;
    row.PhaseDistance_m = m.distance;
    row.PhaseDuration_s = m.duration;
    row.StrideLength_m = m.distance * impulseScale;
    row.StrideTime_s = m.duration * impulseScale;
    row.P_gate_trapz = m.P_gate_trapz;
    row.Tplus_trapz_s = m.Tplus_trapz;

    % Moco가 제약한 값: 적분 상태의 최종값 (상태는 t0에서 0). CoT는 반주기
    % 에너지 / (총질량 * 반주기 COM 변위)이며 impulseScale을 곱하지 않는다.
    [sol, solLabels] = readSto(solFile);
    zE = sol(:, strcmp(solLabels, '/effortPropulsion/effort_integral'));
    zP = sol(:, strcmp(solLabels, '/effortPropulsion/propulsive_impulse'));
    zM = sol(:, strcmp(solLabels, '/effortPropulsion/mean_propulsive_force'));
    zMet = sol(:, strcmp(solLabels, '/effortPropulsion/metabolic_energy'));
    if ~isempty(zE), row.Effort_constraint_value = zE(end) / m.com_displacement; end
    if ~isempty(zMet)
        row.MetabolicEnergy_state_J = zMet(end);
        row.CoT_constraint_value = zMet(end) / (m.mass_kg * m.com_displacement);
    end
    if isCoT
        row.E_constraint_value = row.CoT_constraint_value;
    else
        row.E_constraint_value = row.Effort_constraint_value;
    end
    if ~isempty(zP), row.P_gate_constraint_value = impulseScale * zP(end); end
    if ~isempty(zM), row.MPF_constraint_value = zM(1); end

    % propulsion 제약 판정 (equality/range 공통): 저장된 control·접촉력으로 다시 계산한
    % 실행 당시 지표 J_actual (impulse: P_gate, meanForce: 평균 추진력)를 허용 구간으로 판정한다.
    % solver 적분 상태의 제약값(modeSym impulse는 반주기 해의 z_P(tf)에 scale 적용)도 함께 둔다.
    if row.PropulsionMetric == "meanForce"
        row.PropulsionActual = m.mean_propulsive_force;
        row.PropulsionConstraintValue = row.MPF_constraint_value;
    else
        row.PropulsionActual = m.P_gate_simpson;
        row.PropulsionConstraintValue = row.P_gate_constraint_value;
    end
    pcheck = WoC_moco_propulsionCheck(row.PropulsionActual, row.PropulsionLower, ...
        row.PropulsionUpper, cfg.propulsionTol, cfg.propulsionBoundTol);
    row.PropulsionViolation = pcheck.violation;
    row.PropulsionPass = pcheck.pass;
    row.PropulsionBoundHits = pcheck.hits;
    ccheck = WoC_moco_propulsionCheck(row.PropulsionConstraintValue, row.PropulsionLower, ...
        row.PropulsionUpper, cfg.propulsionTol, cfg.propulsionBoundTol);
    row.PropulsionConstraintViolation = ccheck.violation;
    if row.PropulsionMetadataIssue ~= ""
        % 구간이 유한하고 순서가 맞아도 모드와 맞지 않으면(예: equality인데 lower ~= upper)
        % 실행 당시 제약을 알 수 없으므로 판정하지 않는다.
        row.PropulsionViolation = NaN;
        row.PropulsionConstraintViolation = NaN;
        row.PropulsionPass = false;
        row.PropulsionBoundHits = "";
    end
    % getResult이 한 보행주기 궤적으로 계산한 CoT (이 header 항목이 없는 이전 파일은 NaN).
    metFile = regexprep(solFile, '_kinematics(_half)?\.sto$', '_metabolic.sto');
    if isfile(metFile)
        cot = regexp(fileread(metFile), 'CoT\(J/kg/m\)=([^\r\n]+)', 'tokens', 'once');
        if ~isempty(cot), row.CoT_fullStride = str2double(cot{1}); end
    end

    % 결과가 bound에 붙어 있는지 (반 걸음/한 걸음 phase 기준).
    tb = cfg.timeBound * boundScale;
    db = cfg.distBound * boundScale;
    hits = strings(0);
    boundTol = 1e-4;
    if abs(m.duration - min(tb)) < boundTol, hits(end+1) = "time_lb"; end
    if abs(m.duration - max(tb)) < boundTol, hits(end+1) = "time_ub"; end
    if abs(m.distance - min(db)) < boundTol, hits(end+1) = "dist_lb"; end
    if abs(m.distance - max(db)) < boundTol, hits(end+1) = "dist_ub"; end
    row.BoundHits = strjoin(hits, ';');

    % 독립 재계산: OpenSim 없이 저장된 한 보행주기 파일만 쓴다(사다리꼴 적분).
    prefix = 'moco_WoC_Solution_iter01';
    if unseal, prefix = [prefix '_unseal']; end
    mocoDir = fullfile(resultDir, 'result_1', 'moco_result');
    [kin, kinLabels] = readSto(fullfile(mocoDir, [prefix '_kinematics.sto']));
    [grf, grfLabels] = readSto(fullfile(mocoDir, [prefix '_GRF.sto']));
    t = kin(:, 1);
    tx = kin(:, strcmp(kinLabels, '/jointset/groundPelvis/pelvis_tx/value'));
    row.Indep_v_avg_mps = (tx(end) - tx(1)) / (t(end) - t(1));
    fap = [grf(:, strcmp(grfLabels, 'ground_force_r_vx')), ...
           grf(:, strcmp(grfLabels, 'ground_force_l_vx'))];
    g = 1 ./ (1 + exp(-(fap - row.GateThreshold_N) / row.GateSigma_N));
    row.Indep_P_gate = sum(trapz(grf(:, 1), fap .* g));
    row.Indep_Tplus_s = sum(trapz(grf(:, 1), g));
    row.Indep_P_physical = sum(trapz(grf(:, 1), max(fap, 0)));
    comFile = fullfile(resultDir, 'result_1', 'analy_result', ...
        '2D_gait_AFO_pc_PointKinematics_CoM_pos.sto');
    if isfile(comFile)
        % 궤적에 저장된 control(근육, lumbarAct). AFO 열이 있어도 명시적으로 제외한다.
        com = readSto(comFile);
        isControl = ~startsWith(kinLabels, {'/jointset', '/effortPropulsion'}) & ...
            ~endsWith(kinLabels, '/activation') & ~strcmp(kinLabels, 'time') & ...
            ~ismember(regexprep(kinLabels, '^.*/', ''), {'AFO_r', 'AFO_l'});
        row.Indep_Effort_noAFO = trapz(t, sum(kin(:, isControl).^2, 2)) / ...
            norm(com(end, 2:4) - com(1, 2:4));
    end

    % 판정에 쓰는 propulsion 제약: 실행 당시 지표와 구간 (equality는 lower = upper = J로
    % 기존 |J_actual - J| <= propulsionTol 판정과 같다). solver 수렴(Converged), 에너지
    % equality(E_pass), propulsion 제약(PropulsionPass), time/dist bound 접촉(BoundHits),
    % propulsion 경계 접촉(PropulsionBoundHits)은 각각 열로 따로 남는다.
    % 실행 당시 제약 정보가 없거나 맞지 않으면(PropulsionMetadataIssue) 수렴·tolerance
    % 판정보다 먼저 그 사실을 Verdict로 남긴다 ("out of tol"과 섞이지 않게).
    propulsionPass = row.PropulsionPass;
    isRange = row.PropulsionConstraintMode == "range";
    hasMetaIssue = row.PropulsionMetadataIssue ~= "";
    if row.EnergyMetric ~= string(cfg.energyMetric)
        % 다른 에너지 지표의 결과를 이 지표의 target과 tolerance로 판정하지 않는다.
        row.E_pass = false;
        row.Verdict = "Energy metric mismatch (run " + row.EnergyMetric + ...
            ", cfg " + cfg.energyMetric + ")";
    elseif ~isGateRun
        row.Verdict = "Not a gate run (" + row.PropulsionDefinition + ")";
    elseif hasMetaIssue
        row.Verdict = "Propulsion metadata " + row.PropulsionMetadataIssue;
    elseif ~row.Converged
        if row.E_pass && propulsionPass && isRange
            row.Verdict = "Not converged (constraints within tol)";
        elseif row.E_pass && propulsionPass
            row.Verdict = "Not converged (equalities within tol)";
        else
            row.Verdict = "Not converged";
        end
    elseif row.E_pass && propulsionPass
        row.Verdict = "OK";
    elseif isRange
        failed = strings(0);
        if ~row.E_pass, failed(end+1) = "energy equality"; end
        if ~propulsionPass, failed(end+1) = "propulsion range"; end
        row.Verdict = "Converged, " + strjoin(failed, " and ") + " out of tol";
    else
        row.Verdict = "Converged, equality out of tol";
    end
end


function [mode, lower, upper, issue] = runPropulsionInterval(params)
% 실행 당시 parameters에서 J 제약 형태와 허용 구간을 읽고, 판정에 쓸 수 있는지 검사한다.
%   - propulsion_lower/upper가 없는 이전 equality 결과: lower = upper = mocoPropulsionTarget
%   - 정보가 없으면 issue = "missing: ...", 있지만 판정에 쓸 수 없으면 "invalid: ...".
%     issue가 있으면 lower/upper는 읽은 값 그대로 (없으면 NaN) 두고 판정하지 않는다.
%   규칙은 실행 전 입력 검사 (WoC_moco_main, WoC_moco_propulsionCases)와 같다:
%     equality : lower = upper = target, 유한한 양수 (mocoPropulsionTarget이 있으면 같은 값)
%     range    : 유한, 0 <= lower < upper, scalar target 없음
%   WoC_moco_saveParameters가 target과 lower/upper를 같은 형식(num2str(v, 10))으로 쓰므로
%   equality 일치는 상대 1e-9로 본다.
    issue = "";
    mode = 'equality';
    if isfield(params, 'propulsion_constraint_mode') && ~isempty(strtrim(params.propulsion_constraint_mode))
        mode = strtrim(params.propulsion_constraint_mode);
    end
    [hasLower, lower, rawLower] = numericParam(params, 'propulsion_lower');
    [hasUpper, upper, rawUpper] = numericParam(params, 'propulsion_upper');
    [hasTarget, target, rawTarget] = numericParam(params, 'mocoPropulsionTarget');

    if ~ismember(mode, {'equality', 'range'})
        issue = "invalid: unknown propulsion_constraint_mode '" + mode + "'";
        return;
    end
    if hasLower ~= hasUpper
        issue = "invalid: only one of propulsion_lower/propulsion_upper saved";
        return;
    end
    if ~hasLower
        if strcmp(mode, 'range')
            issue = "missing: range run without propulsion_lower/propulsion_upper";
            return;
        end
        if ~hasTarget
            issue = "missing: no propulsion_lower/propulsion_upper or mocoPropulsionTarget";
            return;
        end
        lower = target; upper = target;
        rawLower = rawTarget; rawUpper = rawTarget;
    end
    if ~isfinite(lower) || ~isfinite(upper)
        issue = "invalid: propulsion interval is not finite [" + rawLower + ", " + rawUpper + "]";
        return;
    end
    if strcmp(mode, 'equality')
        if lower ~= upper
            issue = sprintf("invalid: equality run with lower ~= upper [%.10g, %.10g]", lower, upper);
        elseif lower <= 0
            issue = sprintf("invalid: equality target must be > 0 (%.10g)", lower);
        elseif hasTarget && ~(abs(target - lower) <= 1e-9 * max(1, abs(lower)))
            issue = string(sprintf(['invalid: mocoPropulsionTarget %s differs from ', ...
                'propulsion_lower/upper %.10g'], rawTarget, lower));
        end
    else
        if hasTarget
            issue = "invalid: range run with mocoPropulsionTarget " + rawTarget + ...
                " (range has no scalar target)";
        elseif lower < 0 || lower >= upper
            issue = sprintf("invalid: range needs 0 <= lower < upper ([%.10g, %.10g])", lower, upper);
        end
    end
end


function [has, value, raw] = numericParam(params, name)
% parameters 항목 name이 비어 있지 않으면 has = true, value = str2double (숫자가 아니면 NaN).
    has = isfield(params, name) && ~isempty(strtrim(params.(name)));
    raw = "";
    value = NaN;
    if has
        raw = string(strtrim(params.(name)));
        value = str2double(raw);
    end
end


function params = readParams(path)
% *_parameters.csv (Parameter, Value)를 struct로 읽는다. 없으면 빈 struct.
% 값은 큰따옴표로 감싸여 있고 쉼표를 포함할 수 있으므로 첫 쉼표에서 나눈다.
    params = struct();
    if ~isfile(path), return; end
    lines = readlines(path);
    for i = 2:numel(lines)
        parts = split(lines(i), ',');
        if numel(parts) < 2, continue; end
        value = strjoin(parts(2:end), ',');
        params.(matlab.lang.makeValidName(parts(1))) = ...
            char(regexprep(value, '^"(.*)"$', '$1'));
    end
    % 이전 실행의 모드 이름은 읽을 때만 현재 stateFixed 용어로 통일한다.
    if isfield(params, 'mocoObjectiveMode')
        params.mocoObjectiveMode = strrep(params.mocoObjectiveMode, ...
            'SpeedFixedEP', 'SpeedStateFixed');
    end
end


function hdr = readStoHeader(path)
    hdr = struct();
    fid = fopen(path);
    cleaner = onCleanup(@() fclose(fid));
    while true
        line = fgetl(fid);
        if ~ischar(line) || strcmp(strtrim(line), 'endheader'), break; end
        kv = strsplit(line, '=');
        if numel(kv) == 2
            hdr.(matlab.lang.makeValidName(kv{1})) = strtrim(kv{2});
        end
    end
end


function [data, labels] = readSto(path)
    fid = fopen(path);
    if fid < 0, error('파일을 열 수 없습니다: %s', path); end
    cleaner = onCleanup(@() fclose(fid));
    line = '';
    while ~strcmp(strtrim(line), 'endheader')
        line = fgetl(fid);
        if ~ischar(line), error('endheader가 없습니다: %s', path); end
    end
    labels = strsplit(strtrim(fgetl(fid)), '\t');
    data = textscan(fid, repmat('%f', 1, numel(labels)), 'CollectOutput', true);
    data = data{1};
    if isempty(data) || any(~isfinite(data(:)))
        error('데이터가 비었거나 유한하지 않습니다: %s', path);
    end
end


function writeDefinitions(path, cfg, impulseScale, boundScale)
    fid = fopen(path, 'w');
    cleaner = onCleanup(@() fclose(fid));
    isMax = strcmp(cfg.objectiveMode, 'maxSpeedStateFixed');
    speedVerb = 'minimize';
    if isMax, speedVerb = 'maximize'; end
    isCoT = strcmp(cfg.energyMetric, 'metabolicCoT');
    if isCoT
        energyName = 'CoT';
        energyLine = '    E (energy_metric metabolicCoT): MocoFinalOutputGoal metabolic_cot_constraint z_met(tf)/(mass d_COM) = E0 [J/kg/m] (E_constraint_value)';
        stateLine = 'Constraint implementation: states z_met'' = P_met, z_P'' = F_r g(F_r) + F_l g(F_l), z(t0) = 0';
    else
        energyName = 'E';
        energyLine = '    E (energy_metric effort): MocoFinalOutputGoal effort_constraint z_E(tf)/d_COM = E0 (E_constraint_value)';
        stateLine = 'Constraint implementation: states z_E'' = sum_c u_c^2, z_P'' = F_r g(F_r) + F_l g(F_l), z(t0) = 0';
    end
    energyUnit = energyUnitOf(cfg.energyMetric);
    isRange = strcmp(cfg.propulsionConstraintMode, 'range');
    if isRange
        jRel = 'lower_k <= %s <= upper_k';
        caseText = strjoin(arrayfun(@(c) sprintf('[%g, %g]', c.lower, c.upper), cfg.propCases, ...
            'UniformOutput', false), ', ');
        modeLine = sprintf(['propulsion constraint mode: range (closed interval, one case per J_bounds row: %s; ', ...
            'no scalar target, the interval center is not used)'], caseText);
    else
        jRel = '%s = J_k';
        caseText = strjoin(arrayfun(@(c) sprintf('%g', c.target), cfg.propCases, 'UniformOutput', false), ', ');
        modeLine = sprintf('propulsion constraint mode: equality (one case per J_targets value: %s)', caseText);
    end
    if strcmp(cfg.propulsionMetric, 'meanForce')
        metricLines = {
            sprintf(['%s: %s v_avg s.t. %s = E0 (endpoint equality), ' jRel ' [N]'], ...
                cfg.objectiveMode, speedVerb, energyName, 'mean propulsive force')
            sprintf('target source: %s', cfg.targetSource)
            modeLine
            'propulsion_definition: gate_mean_force (meanPropPerVel MeanPropulsiveForce m = Iplus / Tplus)'};
        mLine = '    and m(t0) = J (MPF_constraint_value). m is the same for the half and the full stride.';
        if isRange
            mLine = ['    and lower_k <= m(t0) <= upper_k (initial bounds of m; MPF_constraint_value). ', ...
                     'm is free inside the interval and the same for the half and the full stride.'];
        end
        constraintLines = {
            [stateLine ';']
            energyLine
            '    mean propulsive force (as MeanPropulsiveForce): states m'' = 0 and'
            '    r'' = g(F_r) (F_r - m) + g(F_l) (F_l - m) with bounds r(t0) = r(tf) = 0 (Iplus = m * Tplus)'
            mLine
            '    P_gate is not constrained (z_P is reported only).'};
        unitJ = 'N';
    else
        metricLines = {
            sprintf(['%s: %s v_avg s.t. %s = E0, ' jRel ' (endpoint constraints)'], ...
                cfg.objectiveMode, speedVerb, energyName, 'P_gate')
            sprintf('target source: %s', cfg.targetSource)
            modeLine
            'propulsion_definition: gate_Iplus (meanPropPerVel MeanPropulsiveForce numerator)'};
        pLine = sprintf('    %d * z_P(tf) = J (P_gate_constraint_value)', impulseScale);
        if isRange
            pLine = sprintf(['    lower_k <= %d * z_P(tf) <= upper_k (propulsion_constraint endpoint bounds ', ...
                             'MocoBounds(lower, upper); P_gate_constraint_value)'], impulseScale);
        end
        constraintLines = {
            [stateLine ';']
            '    MocoFinalOutputGoal endpoint constraints (Stage::Time outputs):'
            energyLine
            pLine};
        unitJ = 'N*s';
    end
    tolLine = {
        sprintf(['verification: |%s - E0| <= %g %s (E_pass); propulsion violation = max(lower - J, ', ...
                 'J - upper, 0) <= %g %s (PropulsionPass; equality: lower = upper = J_k)'], ...
            energyName, cfg.energyTol, energyUnit, cfg.propulsionTol, unitJ)
        sprintf(['PropulsionBoundHits: lower/upper (range) or target (equality) when |J - bound| <= %g %s ', ...
                 '(propulsionBoundTol; reported only, a value on a bound inside the interval passes)'], ...
            cfg.propulsionBoundTol, unitJ)
        sprintf(['solver tolerance (constraint %g, convergence %g), verification tolerance and the ', ...
                 'user interval width are separate settings'], cfg.constraintTolerance, cfg.convergenceTolerance)
        ['PropulsionActual: recomputed from saved controls/contact forces (Simpson); ', ...
         'PropulsionConstraintValue: solver integral state (modeSym impulse: half-stride z_P(tf) x 2)']};
    if isCoT
        [~, metInfo] = moco_WoC_addMetabolics([]);
        energyLines = {
            'E = metabolic CoT = int P_met dt / (mass * d_COM)  [J/kg/m] (energy_metric metabolicCoT)'
            '    P_met = /metabolics|total_metabolic_rate [W] of Bhargava2004SmoothedMuscleMetabolics,'
            '        every Muscle component registered (moco_WoC_addMetabolics); basal term included.'
            ['    ' metInfo.settings]
            '    mass = model total mass (MocoGoal divide_by_mass), d_COM = |r_COM(tf) - r_COM(t0)|'
            '        (divide_by_displacement). Not divided by g or the phase duration.'
            '    modeSym: half-stride energy / half-stride COM displacement (impulseScale is not applied).'
            '    The effort integral z_E is reported only (Effort_* columns), not constrained.'};
    else
        energyLines = {
            'E = (1/d_COM) * int sum_c u_c^2 dt (energy_metric effort)'
            '    c = model controls excluding AFO_r/AFO_l, weight 1: 18 muscle excitations and'
            '        lumbarAct (CoordinateActuator, optimal force 1, control = N*m). No reserve actuators.'
            '    AFO controls are explicitly excluded in both the integrator and weighted effort goal.'
            '    d_COM = |r_COM(tf) - r_COM(t0)|; identical to the weighted-mode effort goal before weight.'};
    end
    lines = [metricLines; {
        ''
        sprintf(['v_avg = (pelvis_tx(tf) - pelvis_tx(t0)) / (tf - t0)  [m/s] ', ...
                 '(only Moco cost, weight %g)'], (1 - 2*isMax) * cfg.speedWeight)}; energyLines; {
        sprintf('P_gate = %d * int_phase [F_r g(F_r) + F_l g(F_l)] dt  [N*s] (one stride, both feet)', impulseScale)
        '    F_foot = contactHeel x-force + contactFront x-force on the foot (summed per foot, then gated)'
        sprintf('    g(F) = 1 / (1 + exp(-(F - F0)/sigma)), sigma = %.4g N, F0 = %.4g N', cfg.gateSigma, cfg.gateThreshold)
        '    modeSym solves a half stride: the full stride P_gate is 2x the half-stride integral; modeAsym: 1x.'
        '    Not divided by Tplus, stride time, speed, distance, BW or a reference duration.'
        'Tplus = int sum_foot g(F_foot) dt [s]; mean_propulsive_force = P_gate / Tplus [N]'
        '    (MeanPropulsiveForce_N; the constrained metric when propulsion_definition is gate_mean_force)'
        'P_physical = int sum_foot max(F_foot, 0) dt over the saved full stride (trapezoid, comparison only)'
        ''}; constraintLines; {
        'E_actual, P_gate_actual, Tplus_s, MeanPropulsiveForce_N: recomputed from controls/contact forces'
        '    (CoT: total_metabolic_rate of the saved states and controls applied by control name to the'
        '    model, realized to Stage::Dynamics) with Simpson quadrature on the saved mesh+midpoint grid'
        '    (= Hermite-Simpson quadrature); used for E_pass/P_gate_pass/MPF_pass'
        'E_target/E_actual/E_residual/E_pass/E_constraint_value/E_trapz: the run''s energy metric (EnergyMetric)'
        'Effort_*: effort diagnostics (always effort, never CoT); Effort_afo: excluded AFO squared-control'
        '    integral per distance (diagnostic only)'
        'MetabolicEnergy_J, CoT_simpson, CoT_trapz: recomputed metabolic energy and CoT (any energy metric);'
        '    MetabolicEnergy_state_J, CoT_constraint_value: z_met(tf) of the solution (metabolicCoT runs);'
        '    CoT_fullStride: getResult full-stride CoT (*_metabolic.sto);'
        '    BasalRate_W = basal_coefficient * mass^basal_exponent (included in P_met)'
        '*_trapz: trapezoid on the same grid; Indep_*: saved full-stride .sto files only (no OpenSim)'
        'Indep_Effort_noAFO: trajectory controls with AFO_r/AFO_l explicitly excluded'
        'Runs without energy_metric in *_parameters.csv are effort runs.'
        'Targets and gate parameters of each case are read from its *_parameters.csv; runs without'
        '    propulsion_definition (earlier softplus version) are marked softplus_legacy, not gate.'
        '    The propulsion constraint mode and interval come from propulsion_constraint_mode,'
        '    propulsion_lower/upper of the run; runs without them are equality runs (lower = upper ='
        '    mocoPropulsionTarget).'
        ['    If that information is missing (no interval and no target) or unusable (non-finite, ', ...
         'lower > upper,']
        ['    equality with lower ~= upper or target <= 0, range with lower >= upper, lower < 0 or a ', ...
         'scalar target,']
        ['    unknown mode), the case is not judged: PropulsionMetadataIssue gives the reason, ', ...
         'PropulsionPass = false,']
        '    PropulsionViolation = NaN and Verdict = "Propulsion metadata missing/invalid: ...".'
        sprintf('bounds (phase): time %s s, pelvis_tx %s m; BoundHits tolerance 1e-4', ...
            mat2str(cfg.timeBound * boundScale), mat2str(cfg.distBound * boundScale))
        }; tolLine; {
        ['Verdict: solver convergence (Converged), energy equality (E_pass), propulsion constraint ', ...
         '(PropulsionPass) and time/dist bound hits (BoundHits) are judged separately;']
        '         a case that did not converge is not reported as infeasible.'
        }];
    fprintf(fid, '%s\n', lines{:});
end


function unit = energyUnitOf(energyMetric)
    if strcmp(energyMetric, 'metabolicCoT'), unit = 'J/kg/m'; else, unit = '-'; end
end


function text = energyDefinitionOf(energyMetric)
    if strcmp(energyMetric, 'metabolicCoT')
        text = ['int total_metabolic_rate dt / (mass * d_COM); Bhargava2004 smoothed, ', ...
                'basal included; solved phase'];
    else
        text = 'int sum_c u_c^2 dt / d_COM; excludes AFO_r/AFO_l; includes lumbarAct';
    end
end


function plotCases(summary, outDir)
% Case 색은 J 순서(entity)로 고정한다. 성공/실패는 마커 채움과 라벨로 표시한다.
    % 에너지 축은 한 지표만 그린다 (effort와 CoT 결과를 한 축에 섞지 않는다).
    if numel(unique(summary.EnergyMetric)) ~= 1
        error('WoC_moco_runStateFixedCases:MixedEnergyMetric', ...
            '에너지 지표가 다른 case가 섞여 있습니다: %s', strjoin(unique(summary.EnergyMetric), ', '));
    end
    isCoT = summary.EnergyMetric(1) == "metabolicCoT";
    if isCoT
        energyAxis = 'Metabolic CoT (J kg^{-1} m^{-1})';
        energyTitle = 'Metabolic CoT: target (bar) vs actual (circle)';
        energyTargetText = 'CoT_0 = %.4g J/kg/m';
        energyFile = 'cot_propulsion_target_vs_actual.png';
    else
        energyAxis = 'Effort E (-)';
        energyTitle = 'Effort: target (bar) vs actual (circle)';
        energyTargetText = 'E_0 = %.4g';
        energyFile = 'effort_propulsion_target_vs_actual.png';
    end
    colors = hex2rgb({'#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', ...
                      '#008300', '#6250d6', '#e34948'});
    ink = hex2rgb({'#0b0b0b', '#52514e'});
    n = height(summary);
    caseColor = @(k) colors(mod(k-1, size(colors, 1)) + 1, :);
    ok = summary.Verdict == "OK";
    shortVerdict = replace(summary.Verdict, ["Converged, equality out of tol", ...
        "Not converged (equalities within tol)", "Not converged (constraints within tol)", ...
        "Converged, energy equality out of tol", "Converged, propulsion range out of tol", ...
        "Converged, energy equality and propulsion range out of tol"], ...
        ["eq. out of tol", "not converged, eq. ok", "not converged, constraints ok", ...
         "E out of tol", "J out of range", "E and J out of tol"]);
    shortVerdict = regexprep(shortVerdict, '^Propulsion metadata (missing|invalid):.*$', 'J metadata $1');
    % propulsion 제약: equality는 lower = upper = target, range는 허용 구간 [lower, upper].
    % range를 단일 target처럼 그리지 않는다 (구간은 선/띠, 실제 값은 원).
    lower = summary.PropulsionLower;
    upper = summary.PropulsionUpper;
    isRangeRow = summary.PropulsionConstraintMode == "range";
    actualJ = summary.PropulsionActual;
    isMeanForce = summary.PropulsionMetric(1) == "meanForce";
    if isMeanForce
        targetUnit = 'N';
        jName = 'mean propulsive force';
        propulsionSpec = {'MPF_target', 'MeanPropulsiveForce_N', [], 'Mean propulsive force (N)'};
        extraText = '';
    else
        targetUnit = 'N\cdots';
        jName = 'P_{gate}, one stride';
        propulsionSpec = {'P_gate_target', 'P_gate_actual', 'P_physical', 'Propulsion (N\cdots, one stride)'};
        extraText = '; max(F,0) (diamond)';
    end
    if any(isRangeRow)
        targetLabel = sprintf('Propulsion J (%s, %s): allowed interval (line), actual (circle)', jName, targetUnit);
        propulsionTitle = ['J: allowed interval (band) or target (bar), actual (circle)' extraText];
    else
        targetLabel = sprintf('Propulsion target J (%s, %s)', jName, targetUnit);
        propulsionTitle = ['J: target (bar), actual (circle)' extraText];
    end
    % 그림 1의 x 위치: equality는 target (기존), range는 실제 값 (구간은 가로선)
    target = lower;
    target(isRangeRow) = actualJ(isRangeRow);
    target(~isfinite(target)) = summary.RunPropulsionTarget(~isfinite(target));
    jLabel = strings(n, 1);
    for k = 1:n
        if isRangeRow(k)
            jLabel(k) = sprintf('J%d \\in [%.3g, %.3g] %s', k, lower(k), upper(k), targetUnit);
        else
            jLabel(k) = sprintf('J%d = %.3g %s', k, target(k), targetUnit);
        end
    end

    % 1. propulsion 제약에 따른 평균속도
    fig = newFigure([640 420]);
    ax = axes(fig); hold(ax, 'on'); styleAxes(ax, ink);
    for k = 1:n
        face = caseColor(k);
        if ~ok(k), face = 'w'; end
        if isRangeRow(k)
            plot(ax, [lower(k), upper(k)], summary.v_avg_mps(k) * [1 1], '-', 'Color', caseColor(k), ...
                'LineWidth', 4, 'HandleVisibility', 'off');
        end
        plot(ax, target(k), summary.v_avg_mps(k), 'o', 'MarkerSize', 9, ...
            'LineWidth', 2, 'MarkerEdgeColor', caseColor(k), 'MarkerFaceColor', face);
        text(ax, target(k), summary.v_avg_mps(k), ...
            sprintf('  J%d: %s', k, shortVerdict(k)), 'Color', ink(2, :), 'FontSize', 9);
    end
    xlabel(ax, targetLabel);
    ylabel(ax, 'Average speed v_{avg} (m/s)');
    speedLabel = 'Minimum';
    if summary.ObjectiveMode(1) == "maxSpeedStateFixed", speedLabel = 'Maximum'; end
    title(ax, sprintf(['%s average speed at ' energyTargetText], speedLabel, summary.E_target(1)), ...
        'Color', ink(1, :));
    padLimits(ax, [target; lower(isRangeRow); upper(isRangeRow)], summary.v_avg_mps);
    saveFigure(fig, fullfile(outDir, 'speed_vs_propulsion_target.png'));

    % 2. energy / propulsion target(또는 허용 구간) vs actual (small multiples, 각자 축)
    fig = newFigure([900 380]);
    tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact');
    specs = [{'E_target', 'E_actual', [], energyAxis}; propulsionSpec];
    for p = 1:2
        ax = nexttile(tl); hold(ax, 'on'); styleAxes(ax, ink);
        for k = 1:n
            if p == 2 && isRangeRow(k)
                % 허용 구간 [lower, upper]를 띠로 그린다 (단일 target 막대 없음)
                patch(ax, k + [-0.3 0.3 0.3 -0.3], [lower(k) lower(k) upper(k) upper(k)], ...
                    caseColor(k), 'FaceAlpha', 0.15, 'EdgeColor', ink(2, :), 'LineWidth', 1);
            elseif p == 2
                plot(ax, k + [-0.3 0.3], lower(k) * [1 1], '-', 'Color', ink(2, :), 'LineWidth', 2);
            else
                plot(ax, k + [-0.3 0.3], summary.(specs{p, 1})(k) * [1 1], '-', ...
                    'Color', ink(2, :), 'LineWidth', 2);
            end
            face = caseColor(k);
            if ~ok(k), face = 'w'; end
            plot(ax, k, summary.(specs{p, 2})(k), 'o', 'MarkerSize', 9, 'LineWidth', 2, ...
                'MarkerEdgeColor', caseColor(k), 'MarkerFaceColor', face);
            if ~isempty(specs{p, 3})
                plot(ax, k, summary.(specs{p, 3})(k), 'd', 'MarkerSize', 8, ...
                    'LineWidth', 1.5, 'MarkerEdgeColor', caseColor(k));
            end
        end
        % 축이 아주 작은 residual로 확대되지 않도록 값 평균의 ±5% 이상을 보인다.
        if p == 2
            vals = [lower; upper; summary.(specs{p, 2})];
        else
            vals = [summary.(specs{p, 1}); summary.(specs{p, 2})];
        end
        if ~isempty(specs{p, 3}), vals = [vals; summary.(specs{p, 3})]; end %#ok<AGROW>
        vals = vals(isfinite(vals));
        if ~isempty(vals)
            span = max(max(vals) - min(vals), 0.1 * mean(abs(vals)));
            ylim(ax, [min(vals), max(vals)] + [-0.15, 0.15] * span);
        end
        xlim(ax, [0.4 n + 0.6]); xticks(ax, 1:n);
        xticklabels(ax, compose('J%d', 1:n));
        ylabel(ax, specs{p, 4});
        if p == 1
            title(ax, energyTitle, 'Color', ink(1, :));
        else
            title(ax, propulsionTitle, 'Color', ink(1, :));
        end
    end
    saveFigure(fig, fullfile(outDir, energyFile));

    % 3. 좌우 AP-GRF, 4. pelvis 전진속도 (한 보행주기, 해의 t0 = 0%)
    fig = newFigure([960 380]);
    tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact');
    sides = {'r', 'Right'; 'l', 'Left'};
    for p = 1:2
        ax = nexttile(tl); hold(ax, 'on'); styleAxes(ax, ink);
        for k = 1:n
            [grf, labels] = caseFile(summary(k, :), 'GRF');
            if isempty(grf), continue; end
            plot(ax, cyclePct(grf(:, 1)), grf(:, strcmp(labels, ['ground_force_' sides{p, 1} '_vx'])), ...
                'Color', caseColor(k), 'LineWidth', 2, 'LineStyle', lineStyle(ok(k)), ...
                'DisplayName', sprintf('%s (%s)', jLabel(k), shortVerdict(k)));
        end
        yline(ax, 0, 'Color', ink(2, :), 'HandleVisibility', 'off');
        xlabel(ax, 'Stride (%) from t_0 of the solved trajectory');
        ylabel(ax, 'AP-GRF on foot (N)');
        title(ax, sprintf('%s foot AP-GRF (heel + front)', sides{p, 2}), 'Color', ink(1, :));
        if p == 1, legend(ax, 'Location', 'best', 'Box', 'off'); end
    end
    saveFigure(fig, fullfile(outDir, 'ap_grf.png'));

    fig = newFigure([640 400]);
    ax = axes(fig); hold(ax, 'on'); styleAxes(ax, ink);
    for k = 1:n
        [kin, labels] = caseFile(summary(k, :), 'kinematics');
        if isempty(kin), continue; end
        plot(ax, cyclePct(kin(:, 1)), kin(:, strcmp(labels, '/jointset/groundPelvis/pelvis_tx/speed')), ...
            'Color', caseColor(k), 'LineWidth', 2, 'LineStyle', lineStyle(ok(k)), ...
            'DisplayName', sprintf('J%d: v_{avg} = %.3f m/s (%s)', k, summary.v_avg_mps(k), shortVerdict(k)));
    end
    xlabel(ax, 'Stride (%) from t_0 of the solved trajectory');
    ylabel(ax, 'Pelvis forward velocity (m/s)');
    title(ax, 'Pelvis forward velocity', 'Color', ink(1, :));
    legend(ax, 'Location', 'best', 'Box', 'off');
    saveFigure(fig, fullfile(outDir, 'pelvis_forward_velocity.png'));
end


function [data, labels] = caseFile(row, kind)
% 한 보행주기 파일 (미수렴 case는 _unseal 파일). 읽을 수 없으면 빈 값.
    data = []; labels = {};
    dirPath = fullfile(row.ResultDir, 'result_1', 'moco_result');
    for prefix = {'moco_WoC_Solution_iter01', 'moco_WoC_Solution_iter01_unseal'}
        f = fullfile(dirPath, sprintf('%s_%s.sto', prefix{1}, kind));
        if isfile(f)
            try
                [data, labels] = readSto(f);
            catch
                data = []; labels = {};
            end
            return;
        end
    end
end


function pct = cyclePct(t)
    pct = 100 * (t - t(1)) / (t(end) - t(1));
end


function s = lineStyle(isOk)
    if isOk, s = '-'; else, s = '--'; end
end


function fig = newFigure(sizePx)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 sizePx]);
end


function styleAxes(ax, ink)
    set(ax, 'Box', 'off', 'TickDir', 'out', 'XColor', ink(2, :), 'YColor', ink(2, :), ...
        'GridColor', [0.85 0.85 0.85], 'GridAlpha', 1, 'FontSize', 10);
    grid(ax, 'on');
end


function padLimits(ax, x, y)
    x = x(isfinite(x)); y = y(isfinite(y));
    pad = @(v) [min(v) max(v)] + [-1 1] * max(0.1 * (max(v) - min(v)), 0.05 * max(abs(v)) + eps);
    if ~isempty(x), xlim(ax, pad(x)); end
    if ~isempty(y), ylim(ax, pad(y)); end
end


function saveFigure(fig, path)
    exportgraphics(fig, path, 'Resolution', 150);
    close(fig);
end


function rgb = hex2rgb(hex)
    rgb = cell2mat(cellfun(@(h) sscanf(h(2:end), '%2x%2x%2x')' / 255, hex(:), ...
        'UniformOutput', false));
end
