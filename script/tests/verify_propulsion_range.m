function report = verify_propulsion_range(workDir)
% verify_propulsion_range
%
%   stateFixed propulsion 제약 모드(equality / range)를 점검한다 (2D 모델).
%   수렴할 때까지 풀지 않는다: 문제 구성은 solve 없이, 실행·요약 경로는 반복 3회짜리
%   짧은 solve(수렴하지 않은 _unseal 결과)로 본다.
%   결과는 workDir (기본: tempdir 아래 새 폴더)에만 쓴다.
%
%   1. 입력 검사 (WoC_moco_propulsionCases): 정상 입력과 잘못된 입력 (누락, 차원, NaN/Inf,
%      역전·같은 값·음수 하한, 모호한 동시 입력, 모드 이름)
%   2. 판정 (WoC_moco_propulsionCheck): 구간 내부, 경계, 허용오차 안/밖의 외부, equality,
%      판정할 수 없는 구간 (NaN, 한쪽 NaN, Inf, 역전 -> violation NaN, pass false)
%   3. WoC_moco_main 입력 오류가 solve 전에 실패 (결과 폴더를 만들지 않음)
%   4. 2D 모델 문제 구성 (moco_WoC_loop, mocoSkipSolve):
%      기존 옵션(모드 없음) -> equality [J, J]; impulse range -> endpoint bounds [lower, upper];
%      meanForce range -> m(t0) bounds [lower, upper] (m 초기 guess = guess 평균 추진력의
%      구간 projection); meanForce equality; min/max 방향과 유일한 speed cost; energy equality
%   5. range 짧은 실행 (WoC_moco_runStateFixedCases, mesh 10, 반복 3회): case 이름,
%      parameters metadata, summary 열과 판정, 재요약, 그림
%   6. 실행 당시 propulsion 메타데이터가 없거나 맞지 않는 결과의 재요약: 판정하지 않고
%      (PropulsionPass false, violation NaN) Verdict "Propulsion metadata missing/invalid: ..."
%
%   실패하면 error.

    import org.opensim.modeling.*
    repo = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(genpath(fullfile(repo, 'src')));
    if nargin < 1 || isempty(workDir)
        workDir = fullfile(tempdir, ['verify_propulsion_range_' char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'))]);
    end
    mkdir(workDir);
    previousLevel = Logger.getLevel();
    loggerCleanup = onCleanup(@() Logger.setLevel(previousLevel));
    Logger.setLevelString('error');
    report = struct('workDir', workDir);

    %% 1. 입력 검사
    c = WoC_moco_propulsionCases(struct('J_targets', [30 40 50]));   % 모드 생략 -> equality
    check(numel(c) == 3 && all(strcmp({c.mode}, 'equality')) && isequal([c.lower], [30 40 50]) && ...
        isequal([c.upper], [30 40 50]) && isequal({c.indexTag}, {'J1', 'J2', 'J3'}), 'equality default cases');
    c = WoC_moco_propulsionCases(struct('propulsionConstraintMode', 'range', 'J_targets', [], 'J_bounds', [30 40]));
    check(numel(c) == 1 && isnan(c.target) && c.lower == 30 && c.upper == 40 && strcmp(c.valueTag, 'JR30to40'), ...
        'single range case');
    c = WoC_moco_propulsionCases(struct('propulsionConstraintMode', 'range', 'J_bounds', [25 35; 35 45; 45.5 55]));
    check(numel(c) == 3 && isequal([c.lower; c.upper]', [25 35; 35 45; 45.5 55]) && ...
        strcmp(c(3).valueTag, 'JR45p5to55') && all(isnan([c.target])), 'three range cases (rows), no center target');
    bad = {
        struct('propulsionConstraintMode', 'equality'), 'MissingTargets'
        struct('propulsionConstraintMode', 'range', 'J_targets', []), 'MissingBounds'
        struct('propulsionConstraintMode', 'equality', 'J_targets', 30, 'J_bounds', [30 40]), 'Ambiguous'
        struct('propulsionConstraintMode', 'range', 'J_targets', 30, 'J_bounds', [30 40]), 'Ambiguous'
        struct('propulsionConstraintMode', 'range', 'J_bounds', [30; 40]), 'BoundsShape'
        struct('propulsionConstraintMode', 'range', 'J_bounds', [30 35 40]), 'BoundsShape'
        struct('propulsionConstraintMode', 'range', 'J_bounds', [30 NaN]), 'BoundsFinite'
        struct('propulsionConstraintMode', 'range', 'J_bounds', [30 Inf]), 'BoundsFinite'
        struct('propulsionConstraintMode', 'range', 'J_bounds', [40 30]), 'BoundsOrder'
        struct('propulsionConstraintMode', 'range', 'J_bounds', [30 30]), 'BoundsOrder'
        struct('propulsionConstraintMode', 'range', 'J_bounds', [-1 30]), 'BoundsOrder'
        struct('propulsionConstraintMode', 'range', 'J_bounds', [25 35; 40 35]), 'BoundsOrder'
        struct('propulsionConstraintMode', 'range', 'J_bounds', [30+1i 40]), 'BoundsShape'
        struct('propulsionConstraintMode', 'equality', 'J_targets', [30 NaN]), 'Targets'
        struct('propulsionConstraintMode', 'equality', 'J_targets', [30 -5]), 'Targets'
        struct('propulsionConstraintMode', 'interval', 'J_bounds', [30 40]), 'Mode'};
    for k = 1:size(bad, 1)
        expectError(@() WoC_moco_propulsionCases(bad{k, 1}), ['WoC_moco_propulsionCases:' bad{k, 2}], ...
            sprintf('invalid input %d (%s)', k, bad{k, 2}));
    end
    fprintf('[1] input validation: 3 valid forms, %d invalid inputs rejected\n', size(bad, 1));

    %% 2. 판정
    tol = 1e-2; hit = 1e-2;
    J = [32, 30, 40, 29.995, 40.005, 29.9, 40.2, 35];
    r = WoC_moco_propulsionCheck(J, 30, 40, tol, hit);
    check(isequal(r.violation, [0 0 0 0.005 0.005 0.1 0.2 0] .* 1) || ...
        max(abs(r.violation - [0 0 0 0.005 0.005 0.1 0.2 0])) < 1e-12, 'range violation values');
    check(isequal(r.pass, [true true true true true false false true]), 'range pass (inside, bounds, within tol, beyond tol)');
    check(isequal(r.hits, ["" "lower" "upper" "lower" "upper" "" "" ""]), 'range bound hits');
    check(r.pass(1) && r.violation(8) == 0, 'center is not the pass criterion (35 and 32 both pass)');
    e = WoC_moco_propulsionCheck([40, 40.005, 40.02], 40, 40, tol, hit);
    check(isequal(e.pass, [true true false]) && isequal(e.hits, ["target" "target" ""]), 'equality as lower = upper');
    n = WoC_moco_propulsionCheck(NaN, 30, 40, tol, hit);
    check(isnan(n.violation) && ~n.pass, 'NaN actual is not a pass');
    % 구간 정보가 없거나 쓸 수 없으면 판정하지 않는다 (MATLAB max는 NaN을 무시하므로
    % 검사가 없으면 violation 0 / pass true가 된다).
    badLower = [NaN, 30, NaN, -Inf, 40, 30];
    badUpper = [NaN, NaN, 40, Inf, 30, Inf];
    u = WoC_moco_propulsionCheck(32 * ones(1, 6), badLower, badUpper, tol, hit);
    check(all(isnan(u.violation)) && ~any(u.pass) && all(u.hits == "") && ~any(u.validBounds), ...
        'NaN / one-sided NaN / Inf / reversed bounds are not judged (violation NaN, pass false)');
    mix = WoC_moco_propulsionCheck([32, 32], [30, NaN], [40, 40], tol, hit);
    check(mix.pass(1) && mix.violation(1) == 0 && ~mix.pass(2) && isnan(mix.violation(2)), ...
        'element-wise: a valid interval next to a missing one');
    fprintf('[2] judgment: inside/boundary/outside, equality and unusable-interval checks pass\n');

    %% 3. WoC_moco_main 입력 오류 (solve 전)
    base = struct('mocoObjectiveMode', 'maxSpeedStateFixed', 'mocoEnergyMetric', 'effort', ...
        'mocoEnergyTarget', 0.1, 'gaitMode', 'modeSym', 'mocoTimeBound', [0.25 0.8], ...
        'mocoDistBound', [0.4 1.5], 'resultsDir', fullfile(workDir, 'main_errors'));
    mainBad = {
        setfield(setfield(base, 'mocoPropulsionConstraintMode', 'range'), 'mocoPropulsionTarget', 30), 'range with scalar target' %#ok<SFLD>
        setfield(setfield(base, 'mocoPropulsionConstraintMode', 'range'), 'mocoPropulsionBounds', [40 30]), 'reversed range' %#ok<SFLD>
        setfield(setfield(base, 'mocoPropulsionConstraintMode', 'range'), 'mocoPropulsionBounds', [30 NaN]), 'NaN range' %#ok<SFLD>
        setfield(base, 'mocoPropulsionConstraintMode', 'range'), 'range without bounds'
        setfield(setfield(base, 'mocoPropulsionTarget', 30), 'mocoPropulsionBounds', [30 40]), 'equality with bounds' %#ok<SFLD>
        setfield(base, 'mocoPropulsionConstraintMode', 'between'), 'unknown mode'
        struct('mocoObjectiveMode', 'weighted', 'mocoPropulsionConstraintMode', 'range', ...
            'mocoPropulsionBounds', [30 40], 'gaitMode', 'modeSym', 'mocoTimeBound', [0.25 0.8], ...
            'mocoDistBound', [0.4 1.5], 'mocoEffort', 1, 'mocoFinalTime', 0.03, ...
            'resultsDir', fullfile(workDir, 'main_errors')), 'range in weighted mode'};
    for k = 1:size(mainBad, 1)
        expectError(@() WoC_moco_main('2D_gait_AFO_pc.osim', 1, 'modeOff', 'should_not_exist', mainBad{k, 1}), ...
            '', sprintf('WoC_moco_main rejects %s', mainBad{k, 2}));
    end
    check(~isfolder(fullfile(workDir, 'main_errors', 'should_not_exist')), 'no result folder for invalid inputs');
    fprintf('[3] WoC_moco_main: %d invalid option sets fail before any solve or result folder\n', size(mainBad, 1));

    %% 4. 2D 문제 구성 (solve 없음)
    moco_WoC_loadFixedEffortPropulsionPlugin();
    modelPath = fullfile(repo, 'models', '2D_gait_AFO_pc.osim');
    guessPath = fullfile(repo, 'inputs', 'guess_init_stateFixed.sto');
    ctrlPath = fullfile(workDir, 'control_zero.sto');
    fid = fopen(ctrlPath, 'w');
    fprintf(fid, 'controls\nnRows=2\nnColumns=3\nendheader\ntime\tAFO_r\tAFO_l\n0\t0\t0\n100\t0\t0\n');
    fclose(fid);
    gm = moco_WoC_evalGaitMetrics(modelPath, guessPath, 2, 1, 5);
    report.guess = struct('E', gm.E_simpson, 'CoT', gm.CoT_simpson, 'P_gate', gm.P_gate_simpson, ...
        'meanForce', gm.mean_propulsive_force);
    common = struct('mocoEffort', 0, 'mocoFinalTime', 0, 'mocoEnergyMetric', 'effort', ...
        'mocoEnergyTarget', 0.1, 'mocoPropulsionGateSigma', 1, 'mocoPropulsionGateThreshold', 5, ...
        'mocoSpeedWeight', 1, 'gaitMode', 'modeSym', 'mocoTimeBound', [0.25 0.8], ...
        'mocoDistBound', [0.4 1.5], 'mocoMeshIntervals', 20, 'mocoConvergenceTolerance', 1e-3, ...
        'mocoConstraintTolerance', 1e-3, 'mocoMaxIterations', 100, 'mocoSkipSolve', true);
    mfLo = floor(gm.mean_propulsive_force) + 3;   % guess 평균 추진력보다 위 구간 (projection 확인용)
    variants = {
        'legacy_impulse_eq', struct('mocoObjectiveMode', 'maxSpeedStateFixed', 'mocoPropulsionMetric', 'impulse', ...
            'mocoPropulsionTarget', 30), 'equality', [30 30], -1
        'impulse_range_max', struct('mocoObjectiveMode', 'maxSpeedStateFixed', 'mocoPropulsionMetric', 'impulse', ...
            'mocoPropulsionConstraintMode', 'range', 'mocoPropulsionBounds', [25 35]), 'range', [25 35], -1
        'impulse_range_min', struct('mocoObjectiveMode', 'minSpeedStateFixed', 'mocoPropulsionMetric', 'impulse', ...
            'mocoPropulsionConstraintMode', 'range', 'mocoPropulsionBounds', [25 35]), 'range', [25 35], 1
        'meanforce_eq', struct('mocoObjectiveMode', 'maxSpeedStateFixed', 'mocoPropulsionMetric', 'meanForce', ...
            'mocoPropulsionConstraintMode', 'equality', 'mocoPropulsionTarget', 60), 'equality', [60 60], -1
        'meanforce_range', struct('mocoObjectiveMode', 'maxSpeedStateFixed', 'mocoPropulsionMetric', 'meanForce', ...
            'mocoPropulsionConstraintMode', 'range', 'mocoPropulsionBounds', [mfLo, mfLo + 10]), 'range', [mfLo, mfLo + 10], -1
        'cot_impulse_range', struct('mocoObjectiveMode', 'maxSpeedStateFixed', 'mocoPropulsionMetric', 'impulse', ...
            'mocoEnergyMetric', 'metabolicCoT', 'mocoEnergyTarget', 3.0, 'mocoLumbarControlBound', [-1 1], ...
            'mocoPropulsionConstraintMode', 'range', 'mocoPropulsionBounds', [25 35]), 'range', [25 35], -1};
    for v = 1:size(variants, 1)
        opts = common;
        f = fieldnames(variants{v, 2});
        for q = 1:numel(f), opts.(f{q}) = variants{v, 2}.(f{q}); end
        caseDir = fullfile(workDir, ['problem_' variants{v, 1}]);
        mkdir(caseDir);
        [~, setup] = moco_WoC_loop(ctrlPath, guessPath, 1, caseDir, modelPath, opts);
        rep = setup.problem.createRep();
        pc = setup.propulsionChecks;
        isMF = strcmp(opts.mocoPropulsionMetric, 'meanForce');
        check(isequal(cellFrom(rep.createCostNames()), {'average_speed'}) && ...
            rep.getCost('average_speed').getWeight() == variants{v, 5}, ...
            sprintf('%s: only cost average_speed with weight %g', variants{v, 1}, variants{v, 5}));
        energyName = 'effort_constraint';
        if strcmp(opts.mocoEnergyMetric, 'metabolicCoT'), energyName = 'metabolic_cot_constraint'; end
        eg = MocoFinalOutputGoal.safeDownCast(rep.getEndpointConstraint(energyName));
        check(isequal(goalBounds(eg), opts.mocoEnergyTarget * [1 1]), sprintf('%s: energy equality kept', variants{v, 1}));
        if isMF
            mInfo = rep.getStateInfo('/effortPropulsion/mean_propulsive_force');
            b = bounds2(mInfo.getInitialBounds());
            check(~any(strcmp(cellFrom(rep.createEndpointConstraintNames()), 'propulsion_constraint')) && ...
                isequal(b, variants{v, 4}) && ~mInfo.getFinalBounds().isSet() && ...
                isequal(bounds2(rep.getStateInfo('/effortPropulsion/mean_force_residual').getInitialBounds()), [0 0]) && ...
                isequal(bounds2(rep.getStateInfo('/effortPropulsion/mean_force_residual').getFinalBounds()), [0 0]), ...
                sprintf('%s: m(t0) bounds %s, r(t0) = r(tf) = 0, no propulsion_constraint', variants{v, 1}, mat2str(b)));
            mGuess = setup.guess.getStateMat('/effortPropulsion/mean_propulsive_force');
            if strcmp(variants{v, 3}, 'range')
                expectedM = min(max(setup.propulsion.guessMeanForce, b(1)), b(2));
                check(all(mGuess == expectedM) && b(1) < b(2) && ...
                    abs(expectedM - mean(b)) > 1e-9, ...
                    sprintf('%s: m guess = projection %.4g of guess mean force %.4g (not the center %.4g)', ...
                    variants{v, 1}, expectedM, setup.propulsion.guessMeanForce, mean(b)));
            else
                check(all(mGuess == variants{v, 4}(1)), sprintf('%s: m guess = J', variants{v, 1}));
            end
        else
            pg = MocoFinalOutputGoal.safeDownCast(rep.getEndpointConstraint('propulsion_constraint'));
            b = goalBounds(pg);
            check(isequal(b, variants{v, 4}) && strcmp(char(pg.getOutputPath()), ...
                '/effortPropulsion|scaled_propulsive_impulse') && ...
                PropertyHelper.getValueDouble(setup.model.getComponent('/effortPropulsion').getPropertyByName('impulse_scale')) == 2, ...
                sprintf('%s: propulsion_constraint endpoint bounds %s on scaled impulse (scale 2, modeSym)', ...
                variants{v, 1}, mat2str(b)));
        end
        if strcmp(variants{v, 3}, 'range')
            check(b(1) < b(2), sprintf('%s: range is not collapsed to an equality', variants{v, 1}));
        end
        check(isequal(pc.propulsionBounds, variants{v, 4}) && strcmp(pc.propulsionMode, variants{v, 3}), ...
            sprintf('%s: internal check recorded the requested bounds', variants{v, 1}));
        for s = {'/effortPropulsion/effort_integral', '/effortPropulsion/propulsive_impulse'}
            check(isequal(bounds2(rep.getStateInfo(s{1}).getInitialBounds()), [0 0]) && ...
                ~rep.getStateInfo(s{1}).getFinalBounds().isSet(), sprintf('%s: %s starts at 0, free at tf', variants{v, 1}, s{1}));
        end
        report.(variants{v, 1}) = struct('bounds', b, 'speedWeight', rep.getCost('average_speed').getWeight());
        fprintf('[4] %s: %s bounds %s, speed weight %g\n', variants{v, 1}, variants{v, 3}, mat2str(b), ...
            rep.getCost('average_speed').getWeight());
    end

    %% 5. range 짧은 실행 (수렴하지 않는 반복 3회 solve, _unseal 결과)
    cfg = struct('model', '2D_gait_AFO_pc.osim', 'objectiveMode', 'maxSpeedStateFixed', ...
        'energyMetric', 'effort', 'E0', 0.1, 'propulsionMetric', 'impulse', ...
        'propulsionConstraintMode', 'range', 'J_targets', [], 'J_bounds', [25 35; 35 45], ...
        'gaitMode', 'modeSym', 'optMode', 'modeOff', 'speedWeight', 1, 'gateSigma', 1, 'gateThreshold', 5, ...
        'meshIntervals', 10, 'convergenceTolerance', 1e-3, 'constraintTolerance', 1e-3, 'maxIterations', 3, ...
        'energyTol', 1e-3, 'propulsionTol', 1e-2, 'propulsionBoundTol', 1e-2, 'continuation', false, ...
        'runOptimizations', true, 'resultsDir', fullfile(workDir, 'results'), ...
        'Index', 'T', 'targetSource', 'verify_propulsion_range', 'timeBound', [0.25 0.8], ...
        'distBound', [0.4 1.5], 'runTag', 'stateFixed_max_jrange_verify_E0p1_X3');
    cfg.guessSto = repmat({guessPath}, 1, 2);
    mkdir(cfg.resultsDir);
    summary = WoC_moco_runStateFixedCases(cfg);
    report.summary = summary;
    check(height(summary) == 2 && all(startsWith(summary.Verdict, "Not converged")) && ...
        all(isfinite(summary.PropulsionActual)), 'range short run: 2 cases ran (not converged, files saved)');
    check(all(endsWith(summary.Case, ["_sym_JR25to35"; "_sym_JR35to45"])), 'range case names carry the interval');
    check(all(summary.PropulsionConstraintMode == "range") && isequal(summary.PropulsionLower, [25; 35]) && ...
        isequal(summary.PropulsionUpper, [35; 45]) && all(isnan(summary.P_gate_target)) && ...
        all(isnan(summary.RunPropulsionTarget)), 'summary: range bounds from metadata, no single target');
    expectedCheck = WoC_moco_propulsionCheck(summary.PropulsionActual, summary.PropulsionLower, ...
        summary.PropulsionUpper, 1e-2, 1e-2);
    check(all(abs(summary.PropulsionActual - summary.P_gate_actual) < 1e-12) && ...
        isequaln(summary.PropulsionViolation, expectedCheck.violation) && ...
        isequal(summary.PropulsionPass, expectedCheck.pass) && ...
        all(isfinite(summary.PropulsionConstraintValue)), ...
        'summary: PropulsionActual (recomputed P_gate) and constraint value (integral state) with violation/pass');
    params = readParamsFile(fullfile(char(summary.ResultDir(1)), 'result_1', 'moco_result', ...
        'moco_WoC_Solution_iter01_parameters.csv'));
    check(strcmp(params.propulsion_constraint_mode, 'range') && str2double(params.propulsion_lower) == 25 && ...
        str2double(params.propulsion_upper) == 35 && isempty(params.mocoPropulsionTarget) && ...
        strcmp(params.mocoPropulsionConstraintMode, 'range') && strcmp(params.mocoPropulsionBounds, '[25 35]'), ...
        'parameters.csv records range mode, lower/upper and no scalar target');
    % 재요약은 현재 cfg가 아니라 실행 당시 metadata를 쓴다: case 1 parameters의 구간을 바꾸면 요약도 바뀐다.
    pfile = fullfile(char(summary.ResultDir(1)), 'result_1', 'moco_result', 'moco_WoC_Solution_iter01_parameters.csv');
    txt = fileread(pfile);
    txt = regexprep(txt, 'propulsion_lower,[^\r\n]*', 'propulsion_lower,20');
    fid = fopen(pfile, 'w'); fwrite(fid, txt); fclose(fid);
    cfg2 = cfg; cfg2.runOptimizations = false;
    pause(1.1);   % summary 폴더 이름(초 단위 시각)이 앞 요약과 겹치지 않게 한다
    s2 = WoC_moco_runStateFixedCases(cfg2);
    check(s2.PropulsionLower(1) == 20 && s2.PropulsionLower(2) == 35, 're-summary judges with the run metadata, not cfg');
    dirs = dir(fullfile(cfg.resultsDir, [cfg.runTag '_sym_summary'], '*'));
    dirs = dirs([dirs.isdir] & ~startsWith({dirs.name}, '.'));
    check(numel(dirs) == 2, 're-summary writes a new summary folder (no overwrite)');
    pngs = dir(fullfile(dirs(end).folder, dirs(end).name, '*.png'));
    check(numel(pngs) == 4, 'range summary figures');
    report.summaryDir = fullfile(dirs(1).folder, dirs(1).name);
    fprintf('[5] range short run: %s\n', report.summaryDir);

    %% 6. 실행 당시 propulsion 메타데이터가 없거나 맞지 않는 결과의 재요약
    % 5의 결과 parameters를 고쳐 다시 요약한다. Verdict가 수렴/허용오차 판정보다 먼저
    % 메타데이터 문제를 보이는지 본다 (정상 메타데이터는 "Not converged ...").
    pfiles = cellfun(@(d) fullfile(d, 'result_1', 'moco_result', 'moco_WoC_Solution_iter01_parameters.csv'), ...
        cellstr(summary.ResultDir), 'UniformOutput', false);
    originals = cellfun(@fileread, pfiles, 'UniformOutput', false);
    restore = onCleanup(@() cellfun(@writeText, pfiles, originals));
    noInterval = @(t) delParam(delParam(delParam(t, 'propulsion_constraint_mode'), 'propulsion_lower'), 'propulsion_upper');
    variants = {   % 이름, parameters 변경, 기대 PropulsionMetadataIssue 시작 문자열 ("" = 정상)
        'no interval and no target', noInterval, "missing: no propulsion_lower/propulsion_upper or mocoPropulsionTarget"
        'range without interval', @(t) delParam(delParam(t, 'propulsion_lower'), 'propulsion_upper'), "missing: range run without"
        'only lower saved', @(t) delParam(t, 'propulsion_upper'), "invalid: only one of"
        'lower not a number', @(t) setParam(t, 'propulsion_lower', 'abc'), "invalid: propulsion interval is not finite"
        'upper Inf', @(t) setParam(t, 'propulsion_upper', 'Inf'), "invalid: propulsion interval is not finite"
        'reversed range', @(t) setParam(setParam(t, 'propulsion_lower', '40'), 'propulsion_upper', '30'), "invalid: range needs 0 <= lower < upper"
        'range lower == upper', @(t) setParam(setParam(t, 'propulsion_lower', '30'), 'propulsion_upper', '30'), "invalid: range needs 0 <= lower < upper"
        'negative lower', @(t) setParam(t, 'propulsion_lower', '-1'), "invalid: range needs 0 <= lower < upper"
        'range with scalar target', @(t) setParam(t, 'mocoPropulsionTarget', '30'), "invalid: range run with mocoPropulsionTarget"
        'equality with lower ~= upper', @(t) setParam(t, 'propulsion_constraint_mode', 'equality'), "invalid: equality run with lower ~= upper"
        'equality target mismatch', @(t) setParam(setParam(setParam(setParam(t, 'propulsion_constraint_mode', 'equality'), ...
            'propulsion_lower', '30'), 'propulsion_upper', '30'), 'mocoPropulsionTarget', '31'), "invalid: mocoPropulsionTarget 31 differs"
        'unknown mode', @(t) setParam(t, 'propulsion_constraint_mode', 'between'), "invalid: unknown propulsion_constraint_mode"
        'legacy equality with target (valid)', @(t) setParam(noInterval(t), 'mocoPropulsionTarget', '30'), ""
        'saved range (valid)', @(t) t, ""};
    cfg3 = cfg; cfg3.runOptimizations = false;
    for v = 1:2:size(variants, 1)
        for k = 1:2   % case 1, 2에 변형 하나씩
            writeText(pfiles{k}, variants{v + k - 1, 2}(originals{k}));
        end
        pause(1.1);
        s3 = WoC_moco_runStateFixedCases(cfg3);
        for k = 1:2
            name = variants{v + k - 1, 1};
            expected = variants{v + k - 1, 3};
            issue = s3.PropulsionMetadataIssue(k);
            if expected == ""
                ref = WoC_moco_propulsionCheck(s3.PropulsionActual(k), s3.PropulsionLower(k), ...
                    s3.PropulsionUpper(k), cfg.propulsionTol, cfg.propulsionBoundTol);
                check(issue == "" && startsWith(s3.Verdict(k), "Not converged") && ...
                    isequaln(s3.PropulsionViolation(k), ref.violation) && s3.PropulsionPass(k) == ref.pass, ...
                    sprintf('metadata %s: judged normally (issue "%s", verdict %s)', name, issue, s3.Verdict(k)));
            else
                check(startsWith(issue, expected) && s3.Verdict(k) == "Propulsion metadata " + issue && ...
                    ~s3.PropulsionPass(k) && isnan(s3.PropulsionViolation(k)) && ...
                    isnan(s3.PropulsionConstraintViolation(k)) && s3.PropulsionBoundHits(k) == "", ...
                    sprintf('metadata %s: not judged (issue "%s", verdict %s, pass %d, violation %g)', ...
                    name, issue, s3.Verdict(k), s3.PropulsionPass(k), s3.PropulsionViolation(k)));
            end
            fprintf('[6] %-38s -> %s\n', name, s3.Verdict(k));
        end
    end
    check(s3.PropulsionLower(2) == 35 && s3.PropulsionUpper(2) == 45, 'unchanged case keeps its saved interval');
    fprintf('[6] metadata: %d invalid/missing variants not judged, 2 valid variants judged\n', size(variants, 1) - 2);
    fprintf('\nAll checks passed. Work folder: %s\n', workDir);
end


function t = setParam(t, name, value)
% parameters.csv 의 한 항목 값을 바꾼다 (항목이 없으면 error).
    pattern = ['^' regexptranslate('escape', name) ',[^\r\n]*'];
    if isempty(regexp(t, pattern, 'once', 'lineanchors'))
        error('verify_propulsion_range:NoParam', 'parameters 항목 %s 이 없습니다.', name);
    end
    t = regexprep(t, pattern, [name ',"' value '"'], 'lineanchors');
end


function t = delParam(t, name)
    t = regexprep(t, ['^' regexptranslate('escape', name) ',[^\r\n]*(\r?\n)?'], '', 'lineanchors');
end


function writeText(path, txt)
    fid = fopen(path, 'w');
    fwrite(fid, txt);
    fclose(fid);
end


function expectError(fn, id, label)
    try
        fn();
    catch ME
        if ~isempty(id) && ~strcmp(ME.identifier, id)
            error('verify_propulsion_range:WrongError', 'FAILED: %s (got %s: %s)', label, ME.identifier, ME.message);
        end
        return;
    end
    error('verify_propulsion_range:NoError', 'FAILED: %s (no error)', label);
end


function params = readParamsFile(path)
    params = struct();
    lines = readlines(path);
    for i = 2:numel(lines)
        parts = split(lines(i), ',');
        if numel(parts) < 2, continue; end
        params.(matlab.lang.makeValidName(parts(1))) = char(regexprep(strjoin(parts(2:end), ','), '^"(.*)"$', '$1'));
    end
end


function b = bounds2(bounds)
    b = [bounds.getLower(), bounds.getUpper()];
end


function b = goalBounds(goal)
    import org.opensim.modeling.*
    info = goal.getPropertyByName('MocoConstraintInfo').getValueAsObject(0);
    m = MocoBounds.safeDownCast(info.getPropertyByName('bounds').getValueAsObject(0));
    b = [m.getLower(), m.getUpper()];
end


function c = cellFrom(v)
    c = cell(1, v.size());
    for k = 0:v.size()-1, c{k+1} = char(v.get(k)); end
end


function check(ok, label)
    if ~ok
        error('verify_propulsion_range:Failed', 'FAILED: %s', label);
    end
end
