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
%   cfg 필드:
%     E0, J_targets      : effort target, propulsion target 벡터 (지표는 propulsionMetric)
%     propulsionMetric   : 'impulse' (없으면 이 값, J = P_gate [N*s, 한 보행주기])
%                          | 'meanForce' (J = 평균 추진력 P_gate / Tplus [N])
%     objectiveMode      : 'minSpeedStateFixed' (없으면 이 값) | 'maxSpeedStateFixed'
%     speedWeight        : 평균속도 cost weight 크기 w_v (없으면 1)
%     targetSource       : target 출처 설명 (CSV에 기록)
%     gateSigma, gateThreshold : propulsion gate sigma, F0 [N]
%     model, optMode, gaitMode, guessSto ('' 이면 WoC_moco_main 기본 guess,
%                          J_targets와 길이가 같은 cell이면 case k가 guessSto{k}에서 시작)
%     timeBound, distBound : 반 걸음 기준 bound (modeAsym은 WoC_moco_main이 x2)
%     meshIntervals, convergenceTolerance, constraintTolerance, maxIterations
%     effortTol, propulsionTol : equality 검증 허용오차 (|E-E0|, |J 지표 - J| [N*s 또는 N])
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

    validateTargets(cfg.E0, cfg.J_targets);
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
    if iscell(cfg.guessSto) && numel(cfg.guessSto) ~= numel(cfg.J_targets)
        error('guessSto가 cell이면 J_targets와 길이가 같아야 합니다 (%d vs %d).', ...
            numel(cfg.guessSto), numel(cfg.J_targets));
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
    nCase = numel(cfg.J_targets);
    caseNames = arrayfun(@(k) sprintf('%s_%s_J%d', cfg.runTag, modeTag, k), ...
        1:nCase, 'UniformOutput', false);
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
            opts.mocoEffortTarget = cfg.E0;
            opts.mocoPropulsionTarget = cfg.J_targets(k);
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
            fprintf('\n===== %s case %d/%d: E0=%.6g, J=%.6g %s (%s) =====\n', ...
                cfg.objectiveMode, k, nCase, cfg.E0, cfg.J_targets(k), unit, cfg.propulsionMetric);
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
    if strcmp(cfg.propulsionMetric, 'meanForce')
        propulsionColumns = {'MPF_target', 'MeanPropulsiveForce_N', 'MPF_residual', 'P_gate_actual'};
    else
        propulsionColumns = {'P_gate_target', 'P_gate_actual', 'P_gate_residual', 'P_physical'};
    end
    disp(summary(:, [{'Case', 'PropulsionDefinition', 'Verdict', 'SolverStatus', ...
        'E_target', 'E_actual', 'E_residual'}, propulsionColumns, {'v_avg_mps', 'BoundHits'}]));

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


function validateTargets(E0, J)
% 실험 target은 사용자가 직접 넣는다. 비어 있거나 잘못된 값으로 실행하지 않는다.
    if isempty(E0) || isempty(J)
        error(['E0 또는 J_targets가 입력되지 않았습니다. 실행 스크립트 상단의 ', ...
               'E0, J_targets에 실제 target 값을 입력하세요.']);
    end
    if ~isscalar(E0) || ~isnumeric(E0) || ~isfinite(E0) || E0 <= 0
        error('E0는 양의 유한 scalar여야 합니다.');
    end
    if ~isnumeric(J) || ~isvector(J) || any(~isfinite(J)) || any(J <= 0)
        error('J_targets는 양의 유한 값의 벡터여야 합니다.');
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
        'PropulsionMetric', string(cfg.propulsionMetric), 'PropulsionDefinition', "", ...
        'Verdict', "", 'SolverStatus', "missing", 'Converged', false, ...
        'Iterations', NaN, 'SolverTime_s', NaN, 'Objective', NaN, ...
        'ObjectiveTerms', "", 'OnlySpeedCost', false, ...
        'E_target', cfg.E0, 'E_actual', NaN, 'E_residual', NaN, 'E_pass', false, ...
        'E_constraint_value', NaN, ...
        'P_gate_target', jTarget(cfg, k, 'impulse'), 'P_gate_actual', NaN, ...
        'P_gate_residual', NaN, 'P_gate_pass', false, 'P_gate_constraint_value', NaN, ...
        'P_physical', NaN, 'P_gate_minus_physical', NaN, 'Tplus_s', NaN, ...
        'MeanPropulsiveForce_N', NaN, 'MPF_target', jTarget(cfg, k, 'meanForce'), 'MPF_residual', NaN, ...
        'MPF_pass', false, 'MPF_constraint_value', NaN, 'RunPropulsionTarget', NaN, ...
        'v_avg_mps', NaN, 'PhaseDistance_m', NaN, 'PhaseDuration_s', NaN, ...
        'StrideLength_m', NaN, 'StrideTime_s', NaN, 'BoundHits', "", ...
        'E_trapz', NaN, 'E_muscle', NaN, 'E_lumbar', NaN, 'E_afo', NaN, ...
        'P_gate_trapz', NaN, 'Tplus_trapz_s', NaN, ...
        'Indep_v_avg_mps', NaN, 'Indep_P_gate', NaN, 'Indep_Tplus_s', NaN, ...
        'Indep_P_physical', NaN, 'Indep_E_noAFO', NaN, ...
        'GateSigma_N', cfg.gateSigma, 'GateThreshold_N', cfg.gateThreshold, ...
        'ImpulseScale', impulseScale, 'SpeedWeight', cfg.speedWeight, ...
        'Model', string(cfg.model), 'OptMode', string(cfg.optMode), ...
        'GaitMode', string(cfg.gaitMode), 'Guess', "", ...
        'MeshIntervals', cfg.meshIntervals, 'ConvergenceTol', cfg.convergenceTolerance, ...
        'ConstraintTol', cfg.constraintTolerance, 'MaxIterations', cfg.maxIterations, ...
        'TimeBound_s', string(mat2str(cfg.timeBound * boundScale)), ...
        'DistBound_m', string(mat2str(cfg.distBound * boundScale)), ...
        'EffortTol', cfg.effortTol, 'PropulsionTol', cfg.propulsionTol, ...
        'ResultDir', string(resultDir), 'RunMessage', "", 'SummaryError', "");
end


function value = jTarget(cfg, k, metric)
% J_k가 metric의 target이면 그 값, 아니면 NaN (실행 전 기본 행).
    value = NaN;
    if strcmp(cfg.propulsionMetric, metric), value = cfg.J_targets(k); end
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
    if isfield(params, 'mocoEffortTarget')
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
    row.E_actual = m.E_simpson;
    row.E_residual = m.E_simpson - row.E_target;
    row.E_pass = abs(row.E_residual) <= cfg.effortTol;
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
    row.E_trapz = m.E_trapz;
    row.E_muscle = m.E_muscle_simpson;
    row.E_lumbar = m.E_lumbar_simpson;
    row.E_afo = m.E_afo_simpson;
    row.P_gate_trapz = m.P_gate_trapz;
    row.Tplus_trapz_s = m.Tplus_trapz;

    % Moco가 제약한 값: 적분 상태의 최종값 (상태는 t0에서 0).
    [sol, solLabels] = readSto(solFile);
    zE = sol(:, strcmp(solLabels, '/effortPropulsion/effort_integral'));
    zP = sol(:, strcmp(solLabels, '/effortPropulsion/propulsive_impulse'));
    zM = sol(:, strcmp(solLabels, '/effortPropulsion/mean_propulsive_force'));
    if ~isempty(zE), row.E_constraint_value = zE(end) / m.com_displacement; end
    if ~isempty(zP), row.P_gate_constraint_value = impulseScale * zP(end); end
    if ~isempty(zM), row.MPF_constraint_value = zM(1); end

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
        % 궤적에 저장된 control(근육, lumbarAct). AFO는 궤적에 없으므로 제외한다.
        com = readSto(comFile);
        isControl = ~startsWith(kinLabels, {'/jointset', '/effortPropulsion'}) & ...
            ~endsWith(kinLabels, '/activation') & ~strcmp(kinLabels, 'time');
        row.Indep_E_noAFO = trapz(t, sum(kin(:, isControl).^2, 2)) / ...
            norm(com(end, 2:4) - com(1, 2:4));
    end

    % 판정에 쓰는 propulsion equality: 실행 당시 지표 (P_gate 또는 평균 추진력).
    propulsionPass = row.P_gate_pass;
    if row.PropulsionMetric == "meanForce", propulsionPass = row.MPF_pass; end
    if ~isGateRun
        row.Verdict = "Not a gate run (" + row.PropulsionDefinition + ")";
    elseif ~row.Converged
        if row.E_pass && propulsionPass
            row.Verdict = "Not converged (equalities within tol)";
        else
            row.Verdict = "Not converged";
        end
    elseif row.E_pass && propulsionPass
        row.Verdict = "OK";
    else
        row.Verdict = "Converged, equality out of tol";
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
    if strcmp(cfg.propulsionMetric, 'meanForce')
        metricLines = {
            sprintf('%s: %s v_avg s.t. E = E0 (endpoint equality), mean propulsive force = J_k [N]', ...
                cfg.objectiveMode, speedVerb)
            sprintf('target source: %s', cfg.targetSource)
            'propulsion_definition: gate_mean_force (meanPropPerVel MeanPropulsiveForce m = Iplus / Tplus)'};
        constraintLines = {
            'Constraint implementation: states z_E'' = sum_c u_c^2, z_P'' = F_r g(F_r) + F_l g(F_l), z(t0) = 0;'
            '    E: MocoFinalOutputGoal endpoint constraint z_E(tf)/d_COM = E0 (E_constraint_value)'
            '    mean propulsive force (as MeanPropulsiveForce): states m'' = 0 and'
            '    r'' = g(F_r) (F_r - m) + g(F_l) (F_l - m) with bounds r(t0) = r(tf) = 0 (Iplus = m * Tplus)'
            '    and m(t0) = J (MPF_constraint_value). m is the same for the half and the full stride.'
            '    P_gate is not constrained (z_P is reported only).'};
        tolLine = sprintf('verification tolerances: |E - E0| <= %g, |mean propulsive force - J| <= %g N', ...
            cfg.effortTol, cfg.propulsionTol);
    else
        metricLines = {
            sprintf('%s: %s v_avg s.t. E = E0, P_gate = J_k (endpoint equality constraints)', ...
                cfg.objectiveMode, speedVerb)
            sprintf('target source: %s', cfg.targetSource)
            'propulsion_definition: gate_Iplus (meanPropPerVel MeanPropulsiveForce numerator)'};
        constraintLines = {
            'Constraint implementation: states z_E'' = sum_c u_c^2, z_P'' = F_r g(F_r) + F_l g(F_l),'
            '    z(t0) = 0; MocoFinalOutputGoal endpoint constraints (Stage::Time outputs)'
            sprintf('    z_E(tf)/d_COM = E0 and %d * z_P(tf) = J (E_constraint_value, P_gate_constraint_value)', impulseScale)};
        tolLine = sprintf('verification tolerances: |E - E0| <= %g, |P_gate - J| <= %g N*s', ...
            cfg.effortTol, cfg.propulsionTol);
    end
    lines = [metricLines; {
        ''
        sprintf(['v_avg = (pelvis_tx(tf) - pelvis_tx(t0)) / (tf - t0)  [m/s] ', ...
                 '(only Moco cost, weight %g)'], (1 - 2*isMax) * cfg.speedWeight)
        'E = (1/d_COM) * int sum_c u_c^2 dt'
        '    c = all model controls, weight 1: 18 muscle excitations, lumbarAct (CoordinateActuator,'
        '        optimal force 1, control = N*m), AFO_r/AFO_l (PrescribedController values;'
        '        MocoControlGoal ignore_controlled_actuators = false). No reserve actuators.'
        '    d_COM = |r_COM(tf) - r_COM(t0)|; identical to the weighted-mode effort goal before weight.'
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
        '    with Simpson quadrature on the saved mesh+midpoint grid (= Hermite-Simpson quadrature);'
        '    used for E_pass/P_gate_pass/MPF_pass'
        '*_trapz: trapezoid on the same grid; Indep_*: saved full-stride .sto files only (no OpenSim)'
        'Indep_E_noAFO: trajectory controls only (AFO controls are not in the trajectory file)'
        'Targets and gate parameters of each case are read from its *_parameters.csv; runs without'
        '    propulsion_definition (earlier softplus version) are marked softplus_legacy, not gate.'
        sprintf('bounds (phase): time %s s, pelvis_tx %s m; BoundHits tolerance 1e-4', ...
            mat2str(cfg.timeBound * boundScale), mat2str(cfg.distBound * boundScale))
        tolLine
        'Verdict: solver convergence and equality satisfaction are judged separately;'
        '         a case that did not converge is not reported as infeasible.'
        }];
    fprintf(fid, '%s\n', lines{:});
end


function plotCases(summary, outDir)
% Case 색은 J 순서(entity)로 고정한다. 성공/실패는 마커 채움과 라벨로 표시한다.
    colors = hex2rgb({'#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', ...
                      '#008300', '#6250d6', '#e34948'});
    ink = hex2rgb({'#0b0b0b', '#52514e'});
    n = height(summary);
    caseColor = @(k) colors(mod(k-1, size(colors, 1)) + 1, :);
    ok = summary.Verdict == "OK";
    shortVerdict = replace(summary.Verdict, ["Converged, equality out of tol", ...
        "Not converged (equalities within tol)"], ["eq. out of tol", "not converged, eq. ok"]);
    isMeanForce = summary.PropulsionMetric(1) == "meanForce";
    if isMeanForce
        targetLabel = 'Propulsion target J (mean propulsive force, N)';
        targetUnit = 'N';
        propulsionSpec = {'MPF_target', 'MeanPropulsiveForce_N', [], 'Mean propulsive force (N)'};
        propulsionTitle = 'Mean propulsive force: target (bar), actual (circle)';
    else
        targetLabel = 'Propulsion target J (P_{gate}, N\cdots, one stride)';
        targetUnit = 'N\cdots';
        propulsionSpec = {'P_gate_target', 'P_gate_actual', 'P_physical', 'Propulsion (N\cdots, one stride)'};
        propulsionTitle = 'P_{gate}: target (bar), actual (circle); max(F,0) (diamond)';
    end
    target = summary.(propulsionSpec{1});
    target(~isfinite(target)) = summary.RunPropulsionTarget(~isfinite(target));

    % 1. propulsion target에 따른 평균속도
    fig = newFigure([640 420]);
    ax = axes(fig); hold(ax, 'on'); styleAxes(ax, ink);
    for k = 1:n
        face = caseColor(k);
        if ~ok(k), face = 'w'; end
        plot(ax, target(k), summary.v_avg_mps(k), 'o', 'MarkerSize', 9, ...
            'LineWidth', 2, 'MarkerEdgeColor', caseColor(k), 'MarkerFaceColor', face);
        text(ax, target(k), summary.v_avg_mps(k), ...
            sprintf('  J%d: %s', k, shortVerdict(k)), 'Color', ink(2, :), 'FontSize', 9);
    end
    xlabel(ax, targetLabel);
    ylabel(ax, 'Average speed v_{avg} (m/s)');
    speedLabel = 'Minimum';
    if summary.ObjectiveMode(1) == "maxSpeedStateFixed", speedLabel = 'Maximum'; end
    title(ax, sprintf('%s average speed at E_0 = %.4g', speedLabel, summary.E_target(1)), ...
        'Color', ink(1, :));
    padLimits(ax, target, summary.v_avg_mps);
    saveFigure(fig, fullfile(outDir, 'speed_vs_propulsion_target.png'));

    % 2. effort / propulsion target vs actual (small multiples, 각자 축)
    fig = newFigure([900 380]);
    tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact');
    specs = [{'E_target', 'E_actual', [], 'Effort E (-)'}; propulsionSpec];
    for p = 1:2
        ax = nexttile(tl); hold(ax, 'on'); styleAxes(ax, ink);
        for k = 1:n
            plot(ax, k + [-0.3 0.3], summary.(specs{p, 1})(k) * [1 1], '-', ...
                'Color', ink(2, :), 'LineWidth', 2);
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
        vals = [summary.(specs{p, 1}); summary.(specs{p, 2})];
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
            title(ax, 'Effort: target (bar) vs actual (circle)', 'Color', ink(1, :));
        else
            title(ax, propulsionTitle, 'Color', ink(1, :));
        end
    end
    saveFigure(fig, fullfile(outDir, 'effort_propulsion_target_vs_actual.png'));

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
                'DisplayName', sprintf('J%d = %.3g %s (%s)', k, target(k), targetUnit, shortVerdict(k)));
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
