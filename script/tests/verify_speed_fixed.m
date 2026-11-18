function report = verify_speed_fixed(workDir)
% verify_speed_fixed
%
%   minEnergySpeedFixed (v_COM = v0 고정, J 제약, E 최소화)를 점검한다 (2D 모델).
%   수렴할 때까지 풀지 않는다: 문제 구성은 solve 없이, 실행·요약 경로는 반복 3회짜리
%   짧은 solve(수렴하지 않은 _unseal 결과)로 본다. 결과는 workDir (기본: tempdir 아래 새 폴더)에만 쓴다.
%
%   1. WoC_moco_main 입력 오류가 solve 전에 실패 (결과 폴더를 만들지 않음): v0 없음/0 이하/
%      bound로 만들 수 없는 v0, E0 동시 지정, stateFixed 모드의 v0, 비용항 weight
%   2. WoC_moco_runStateFixedCases 입력 오류 (E0 동시 지정, v0/speedTol 없음, stateFixed의 v0)
%   3. 문제 구성 (moco_WoC_loop, mocoSkipSolve):
%      CoT + impulse range, effort + impulse equality, CoT + meanForce range ->
%      유일한 cost = E (output/정규화, weight 1), MocoAverageSpeedGoal endpoint constraint
%      (desired v0, bounds [0, 0]), J 제약 bound, E equality 없음, 적분 상태 t0 = 0;
%      기존 maxSpeedStateFixed (CoT range)는 그대로 (speed cost -1, CoT equality, 속도 제약 없음)
%   4. 짧은 실행 (WoC_moco_runStateFixedCases, mesh 10, 반복 3회, range 2 case): parameters
%      metadata, summary 열 (SpeedTarget/SpeedCOM, E_target NaN, OnlyEnergyCost), 판정, 정의 파일, 그림
%   5. 재요약 판정: 실행 당시 v0가 없거나 숫자가 아니면 현재 cfg.speedTarget으로 대신하지 않고
%      "Speed metadata missing/invalid"; objective가 E cost 하나가 아니거나 E 적분 상태값과 다르면
%      "Objective check failed" (OnlyEnergyCost false). 원래 파일로 되돌리면 정상 판정.
%   6. plot_stateFixed: v0 두 개 x J 구간 두 개 묶음에서 x = v0, y = 최소 CoT 그림과 v0별 AP-GRF 그림
%
%   실패하면 error.

    import org.opensim.modeling.*
    repo = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(genpath(fullfile(repo, 'src')));
    if nargin < 1 || isempty(workDir)
        workDir = fullfile(tempdir, ['verify_speed_fixed_' char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'))]);
    end
    mkdir(workDir);
    previousLevel = Logger.getLevel();
    loggerCleanup = onCleanup(@() Logger.setLevel(previousLevel));
    Logger.setLevelString('error');
    report = struct('workDir', workDir);

    %% 1. WoC_moco_main 입력 오류 (solve 전)
    base = struct('mocoObjectiveMode', 'minEnergySpeedFixed', 'mocoEnergyMetric', 'metabolicCoT', ...
        'mocoSpeedTarget', 1.3, 'mocoPropulsionConstraintMode', 'range', 'mocoPropulsionBounds', [25 35], ...
        'gaitMode', 'modeSym', 'mocoTimeBound', [0.25 0.8], 'mocoDistBound', [0.2 1.5], ...
        'resultsDir', fullfile(workDir, 'main_errors'));
    mainBad = {
        rmfield(base, 'mocoSpeedTarget'), 'speedFixed without v0'
        setfield(base, 'mocoSpeedTarget', 0), 'v0 = 0' %#ok<SFLD>
        setfield(base, 'mocoSpeedTarget', [1 2]), 'vector v0' %#ok<SFLD>
        setfield(base, 'mocoSpeedTarget', 7), 'v0 above dist/time bounds (max 6 m/s)' %#ok<SFLD>
        setfield(base, 'mocoSpeedTarget', 0.2), 'v0 below dist/time bounds (min 0.25 m/s)' %#ok<SFLD>
        setfield(base, 'mocoEnergyTarget', 3.0), 'speedFixed with E0' %#ok<SFLD>
        setfield(base, 'mocoEffort', 1), 'speedFixed with effort weight' %#ok<SFLD>
        setfield(setfield(base, 'mocoObjectiveMode', 'maxSpeedStateFixed'), 'mocoEnergyTarget', 3.0), 'v0 in maxSpeedStateFixed' %#ok<SFLD>
        setfield(base, 'mocoObjectiveMode', 'minEnergyStateFixed'), 'unknown mode'}; %#ok<SFLD>
    for k = 1:size(mainBad, 1)
        expectError(@() WoC_moco_main('2D_gait_AFO_pc.osim', 1, 'modeOff', 'should_not_exist', mainBad{k, 1}), ...
            '', sprintf('WoC_moco_main rejects %s', mainBad{k, 2}));
    end
    check(~isfolder(fullfile(workDir, 'main_errors', 'should_not_exist')), 'no result folder for invalid inputs');
    fprintf('[1] WoC_moco_main: %d invalid option sets fail before any solve or result folder\n', size(mainBad, 1));

    %% 2. WoC_moco_runStateFixedCases 입력 오류
    guessPath = fullfile(repo, 'inputs', 'guess_init_stateFixed.sto');
    cfg = struct('model', '2D_gait_AFO_pc.osim', 'objectiveMode', 'minEnergySpeedFixed', ...
        'energyMetric', 'metabolicCoT', 'speedTarget', 1.3, 'speedTol', 1e-4, 'propulsionMetric', 'impulse', ...
        'propulsionConstraintMode', 'range', 'J_targets', [], 'J_bounds', [25 35; 35 45], ...
        'lumbarControlBound', [-1 1], 'gaitMode', 'modeSym', 'optMode', 'modeOff', 'speedWeight', 1, ...
        'gateSigma', 1, 'gateThreshold', 5, 'meshIntervals', 10, 'convergenceTolerance', 1e-3, ...
        'constraintTolerance', 1e-3, 'maxIterations', 3, 'propulsionTol', 1e-2, 'propulsionBoundTol', 1e-2, ...
        'continuation', false, 'runOptimizations', true, 'resultsDir', fullfile(workDir, 'results'), ...
        'Index', 'T', 'targetSource', 'verify_speed_fixed', 'timeBound', [0.25 0.8], ...
        'distBound', [0.2 1.5], 'runTag', 'speedFixed_min_jrange_cot_verify_V1p3_S1');
    cfg.guessSto = repmat({guessPath}, 1, 2);
    mkdir(cfg.resultsDir);
    runnerBad = {
        setfield(cfg, 'E0', 3.0), 'speedFixed with E0' %#ok<SFLD>
        rmfield(cfg, 'speedTarget'), 'speedFixed without v0'
        rmfield(cfg, 'speedTol'), 'speedFixed without speedTol'
        setfield(setfield(cfg, 'objectiveMode', 'maxSpeedStateFixed'), 'E0', 3.0), 'stateFixed with v0'}; %#ok<SFLD>
    for k = 1:size(runnerBad, 1)
        expectError(@() WoC_moco_runStateFixedCases(runnerBad{k, 1}), '', ...
            sprintf('runner rejects %s', runnerBad{k, 2}));
    end
    check(isempty(dir(fullfile(cfg.resultsDir, 'T*'))), 'no case folder for invalid runner inputs');
    fprintf('[2] runner: %d invalid configurations rejected before any case folder\n', size(runnerBad, 1));

    %% 3. 문제 구성 (solve 없음)
    moco_WoC_loadFixedEffortPropulsionPlugin();
    modelPath = fullfile(repo, 'models', '2D_gait_AFO_pc.osim');
    ctrlPath = fullfile(workDir, 'control_zero.sto');
    fid = fopen(ctrlPath, 'w');
    fprintf(fid, 'controls\nnRows=2\nnColumns=3\nendheader\ntime\tAFO_r\tAFO_l\n0\t0\t0\n100\t0\t0\n');
    fclose(fid);
    gm = moco_WoC_evalGaitMetrics(modelPath, guessPath, 2, 1, 5);
    mfLo = floor(gm.mean_propulsive_force) - 5;
    common = struct('mocoEffort', 0, 'mocoFinalTime', 0, 'mocoPropulsionGateSigma', 1, ...
        'mocoPropulsionGateThreshold', 5, 'gaitMode', 'modeSym', 'mocoTimeBound', [0.25 0.8], ...
        'mocoDistBound', [0.2 1.5], 'mocoMeshIntervals', 20, 'mocoConvergenceTolerance', 1e-3, ...
        'mocoConstraintTolerance', 1e-3, 'mocoMaxIterations', 100, 'mocoSkipSolve', true);
    variants = {   % 이름, 옵션, 기대 cost 이름, 기대 J bound, v0 (NaN = stateFixed)
        'cot_impulse_range', struct('mocoObjectiveMode', 'minEnergySpeedFixed', 'mocoEnergyMetric', 'metabolicCoT', ...
            'mocoSpeedTarget', 1.3, 'mocoLumbarControlBound', [-1 1], 'mocoPropulsionMetric', 'impulse', ...
            'mocoPropulsionConstraintMode', 'range', 'mocoPropulsionBounds', [25 40]), 'metabolic_cot_cost', [25 40], 1.3
        'effort_impulse_eq', struct('mocoObjectiveMode', 'minEnergySpeedFixed', 'mocoEnergyMetric', 'effort', ...
            'mocoSpeedTarget', 1.2, 'mocoPropulsionMetric', 'impulse', 'mocoPropulsionTarget', 30), ...
            'effort_cost', [30 30], 1.2
        'cot_meanforce_range', struct('mocoObjectiveMode', 'minEnergySpeedFixed', 'mocoEnergyMetric', 'metabolicCoT', ...
            'mocoSpeedTarget', 1.0, 'mocoLumbarControlBound', [-1 1], 'mocoPropulsionMetric', 'meanForce', ...
            'mocoPropulsionConstraintMode', 'range', 'mocoPropulsionBounds', [mfLo, mfLo + 10]), ...
            'metabolic_cot_cost', [mfLo, mfLo + 10], 1.0
        'stateFixed_cot_range', struct('mocoObjectiveMode', 'maxSpeedStateFixed', 'mocoEnergyMetric', 'metabolicCoT', ...
            'mocoEnergyTarget', 3.0, 'mocoSpeedWeight', 1, 'mocoLumbarControlBound', [-1 1], ...
            'mocoPropulsionMetric', 'impulse', 'mocoPropulsionConstraintMode', 'range', ...
            'mocoPropulsionBounds', [25 35]), 'average_speed', [25 35], NaN};
    for v = 1:size(variants, 1)
        opts = common;
        f = fieldnames(variants{v, 2});
        for q = 1:numel(f), opts.(f{q}) = variants{v, 2}.(f{q}); end
        name = variants{v, 1};
        caseDir = fullfile(workDir, ['problem_' name]);
        mkdir(caseDir);
        [~, setup] = moco_WoC_loop(ctrlPath, guessPath, 1, caseDir, modelPath, opts);
        rep = setup.problem.createRep();
        costs = cellFrom(rep.createCostNames());
        constraints = cellFrom(rep.createEndpointConstraintNames());
        isCoT = strcmp(opts.mocoEnergyMetric, 'metabolicCoT');
        isMF = strcmp(opts.mocoPropulsionMetric, 'meanForce');
        v0 = variants{v, 5};
        check(isequal(costs, variants(v, 3)), sprintf('%s: only cost %s (got %s)', name, variants{v, 3}, strjoin(costs, ', ')));
        if isnan(v0)
            % 기존 stateFixed: speed cost -1, CoT equality, 속도 제약 없음
            eg = MocoFinalOutputGoal.safeDownCast(rep.getEndpointConstraint('metabolic_cot_constraint'));
            check(rep.getCost('average_speed').getWeight() == -1 && isequal(goalBounds(eg), [3 3]) && ...
                ~any(strcmp(constraints, 'average_speed_constraint')) && ~any(strcmp(costs, 'metabolic_cot_cost')), ...
                sprintf('%s: unchanged (speed weight -1, CoT equality [3 3], no speed constraint)', name));
        else
            eg = MocoFinalOutputGoal.safeDownCast(rep.getCost(variants{v, 3}));
            expectedOutput = '/effortPropulsion|effort_integral';
            if isCoT, expectedOutput = '/effortPropulsion|metabolic_energy'; end
            check(eg.getModeIsCost() && eg.getWeight() == 1 && strcmp(char(eg.getOutputPath()), expectedOutput) && ...
                eg.getDivideByMass() == isCoT && eg.getDivideByDisplacement() && ~eg.getDivideByDuration(), ...
                sprintf('%s: E cost on %s (divide_by_mass %d, divide_by_displacement, weight 1)', name, expectedOutput, isCoT));
            check(~any(ismember({'metabolic_cot_constraint', 'effort_constraint'}, constraints)), ...
                sprintf('%s: no energy equality constraint', name));
            sg = MocoAverageSpeedGoal.safeDownCast(rep.getEndpointConstraint('average_speed_constraint'));
            check(~isempty(sg) && sg.get_desired_average_speed() == v0 && sg.getModeIsEndpointConstraint() && ...
                isequal(goalBounds(sg), [0 0]), ...
                sprintf('%s: MocoAverageSpeedGoal endpoint constraint, desired %g m/s, bounds [0 0]', name, v0));
            check(setup.propulsionChecks.speedTarget == v0 && isnan(setup.propulsionChecks.speedWeight), ...
                sprintf('%s: internal check recorded v0', name));
        end
        if isMF
            b = bounds2(rep.getStateInfo('/effortPropulsion/mean_propulsive_force').getInitialBounds());
            check(~any(strcmp(constraints, 'propulsion_constraint')), sprintf('%s: meanForce has no propulsion_constraint', name));
        else
            b = goalBounds(MocoFinalOutputGoal.safeDownCast(rep.getEndpointConstraint('propulsion_constraint')));
        end
        check(isequal(b, variants{v, 4}) && isequal(setup.propulsionChecks.propulsionBounds, variants{v, 4}), ...
            sprintf('%s: J bounds %s', name, mat2str(b)));
        states = {'/effortPropulsion/effort_integral', '/effortPropulsion/propulsive_impulse'};
        if isCoT, states{end+1} = '/effortPropulsion/metabolic_energy'; end %#ok<AGROW>
        for s = states
            check(isequal(bounds2(rep.getStateInfo(s{1}).getInitialBounds()), [0 0]) && ...
                ~rep.getStateInfo(s{1}).getFinalBounds().isSet(), sprintf('%s: %s starts at 0, free at tf', name, s{1}));
        end
        if isfield(opts, 'mocoLumbarControlBound')
            check(isequal(bounds2(rep.getControlInfo('/lumbarAct').getBounds()), [-1 1]), ...
                sprintf('%s: lumbarAct control bound [-1 1]', name));
        end
        fprintf('[3] %s: costs {%s}, constraints {%s}, J bounds %s\n', name, strjoin(costs, ', '), ...
            strjoin(constraints, ', '), mat2str(b));
    end

    %% 4. 짧은 실행 (수렴하지 않는 반복 3회 solve, _unseal 결과)
    summary = WoC_moco_runStateFixedCases(cfg);
    report.summary = summary;
    check(height(summary) == 2 && all(startsWith(summary.Verdict, "Not converged")) && ...
        all(summary.ObjectiveMode == "minEnergySpeedFixed"), 'short run: 2 cases ran (not converged, files saved)');
    check(all(endsWith(summary.Case, ["_V1p3_S1_sym_JR25to35"; "_V1p3_S1_sym_JR35to45"])), ...
        'case names carry the speed tag and the J interval');
    check(all(summary.SpeedTarget_mps == 1.3) && all(isfinite(summary.SpeedCOM_mps)) && ...
        all(abs(summary.SpeedResidual_mps - (summary.SpeedCOM_mps - 1.3)) < 1e-12) && ...
        isequal(summary.SpeedPass, abs(summary.SpeedResidual_mps) <= 1e-4) && all(summary.SpeedTol == 1e-4), ...
        'summary: speed target, recomputed v_COM, residual and SpeedPass');
    check(all(isnan(summary.E_target)) && all(isnan(summary.E_residual)) && ~any(summary.E_pass) && ...
        all(isfinite(summary.E_actual)) && all(isfinite(summary.E_constraint_value)), ...
        'summary: E has no target (E_target/E_residual NaN), E_actual and state value reported');
    check(all(summary.OnlyEnergyCost) && ~any(summary.OnlySpeedCost) && ...
        all(summary.ObjectiveTerms == "metabolic_cot_cost") && ...
        all(abs(summary.Objective - summary.E_constraint_value) <= 1e-6 * abs(summary.E_constraint_value)), ...
        'summary: objective = metabolic_cot_cost = z_met(tf)/(mass d_COM) of the saved trajectory');
    check(all(summary.PropulsionConstraintMode == "range") && isequal(summary.PropulsionLower, [25; 35]) && ...
        isequal(summary.PropulsionUpper, [35; 45]), 'summary: J range from the run metadata');
    params = readParamsFile(fullfile(char(summary.ResultDir(1)), 'result_1', 'moco_result', ...
        'moco_WoC_Solution_iter01_parameters.csv'));
    check(strcmp(params.mocoObjectiveMode, 'minEnergySpeedFixed') && str2double(params.mocoSpeedTarget) == 1.3 && ...
        isempty(params.mocoEnergyTarget) && strcmp(params.energy_metric, 'metabolicCoT') && ...
        contains(params.energy_cost, 'metabolic_cot_cost') && contains(params.speed_constraint, 'v_des = 1.3 m/s') && ...
        ~isfield(params, 'cot_constraint') && strcmp(params.propulsion_constraint_mode, 'range') && ...
        strcmp(params.mocoLumbarControlBound, '[-1 1]'), ...
        'parameters.csv records the mode, v0, energy cost, speed constraint and J range (no CoT constraint)');
    dirs = dir(fullfile(cfg.resultsDir, [cfg.runTag '_sym_summary'], '*'));
    dirs = dirs([dirs.isdir] & ~startsWith({dirs.name}, '.'));
    sumDir = fullfile(dirs(end).folder, dirs(end).name);
    defs = fileread(fullfile(sumDir, 'definitions.txt'));
    check(contains(defs, 'minEnergySpeedFixed: minimize CoT (cost) s.t. v_COM = 1.3 m/s') && ...
        contains(defs, 'MocoAverageSpeedGoal average_speed_constraint') && contains(defs, '(SpeedPass)'), ...
        'definitions.txt describes the speed-fixed problem');
    pngs = sort(string({dir(fullfile(sumDir, '*.png')).name}));
    check(isequal(pngs, sort(["energy_vs_propulsion.png", "speed_propulsion_target_vs_actual.png", ...
        "ap_grf.png", "pelvis_forward_velocity.png"])), 'speed-fixed summary figures');
    report.summaryDir = sumDir;
    fprintf('[4] short run: %s\n', sumDir);

    %% 5. 재요약 판정: 속도 메타데이터와 objective 검사
    pfiles = cellfun(@(d) fullfile(d, 'result_1', 'moco_result', 'moco_WoC_Solution_iter01_parameters.csv'), ...
        cellstr(summary.ResultDir), 'UniformOutput', false);
    sfiles = cellfun(@(d) fullfile(d, 'result_1', 'moco_result', 'moco_WoC_Solution_iter01_unseal_kinematics_half.sto'), ...
        cellstr(summary.ResultDir), 'UniformOutput', false);
    originals = cellfun(@fileread, [pfiles; sfiles], 'UniformOutput', false);
    restore = onCleanup(@() cellfun(@writeText, [pfiles; sfiles], originals));
    cfgR = cfg; cfgR.runOptimizations = false;
    cfgR.speedTarget = 2.0;   % 현재 cfg의 v0가 실행 당시 target처럼 쓰이면 안 된다
    % (a) case 1: mocoSpeedTarget 없음, case 2: 숫자가 아님
    writeText(pfiles{1}, regexprep(originals{1}, '^mocoSpeedTarget,[^\r\n]*(\r?\n)?', '', 'lineanchors'));
    writeText(pfiles{2}, regexprep(originals{2}, '^mocoSpeedTarget,[^\r\n]*', 'mocoSpeedTarget,"abc"', 'lineanchors'));
    pause(1.1);
    s5 = WoC_moco_runStateFixedCases(cfgR);
    check(all(isnan(s5.SpeedTarget_mps)) && ~any(s5.SpeedPass) && all(isnan(s5.SpeedResidual_mps)) && ...
        startsWith(s5.Verdict(1), "Speed metadata missing:") && startsWith(s5.Verdict(2), "Speed metadata invalid:") && ...
        all(s5.SpeedMetadataIssue ~= ""), ...
        sprintf('missing/invalid v0 is not replaced by cfg (2.0) and not judged (%s | %s)', s5.Verdict(1), s5.Verdict(2)));
    % (b) parameters 복원. case 1: objective 항 이름이 다름, case 2: objective 값이 E 상태값과 다름
    cellfun(@writeText, pfiles, originals(1:2));
    writeText(sfiles{1}, regexprep(originals{3}, '^objective_metabolic_cot_cost=', 'objective_average_speed=', 'lineanchors'));
    writeText(sfiles{2}, regexprep(originals{4}, '^objective=[^\r\n]*', 'objective=9.99', 'lineanchors'));
    pause(1.1);
    s5 = WoC_moco_runStateFixedCases(cfgR);
    check(~any(s5.OnlyEnergyCost) && startsWith(s5.ObjectiveCheckIssue(1), "cost terms 'average_speed'") && ...
        startsWith(s5.ObjectiveCheckIssue(2), "objective 9.99 differs") && ...
        all(startsWith(s5.Verdict, "Objective check failed: ")) && all(s5.SpeedTarget_mps == 1.3), ...
        sprintf('objective check failures block the verdict (%s | %s)', s5.Verdict(1), s5.Verdict(2)));
    % (c) 모두 복원: 정상 판정 (반복 3회라 수렴하지 않음)
    cellfun(@writeText, [pfiles; sfiles], originals);
    pause(1.1);
    s5 = WoC_moco_runStateFixedCases(cfgR);
    check(all(s5.OnlyEnergyCost) && all(s5.SpeedMetadataIssue == "") && all(s5.ObjectiveCheckIssue == "") && ...
        isequal(s5.Verdict, summary.Verdict) && all(s5.SpeedTarget_mps == 1.3), 'restored files are judged as before');
    fprintf('[5] re-summary: speed metadata and objective check verdicts pass\n');

    %% 6. plot_stateFixed: x = v0, y = 최소 CoT (v0 두 개)
    cfg2 = cfg;
    cfg2.speedTarget = 1.5;
    cfg2.runTag = 'speedFixed_min_jrange_cot_verify_V1p5_S1';
    s6 = WoC_moco_runStateFixedCases(cfg2);
    both = [summary; s6];
    both.BoundSet = repmat("S1", height(both), 1);
    groupTag = 'speedFixed_min_jrange_cot_verify';
    writetable(both, fullfile(cfg.resultsDir, [groupTag '_bounds_summary.csv']));
    src = fileread(fullfile(repo, 'script', 'plot_stateFixed.m'));
    src = regexprep(src, '\nresultsDir = [^\n]*', sprintf('\nresultsDir = ''%s'';', strrep(cfg.resultsDir, '\', '\\')), 'once');
    src = regexprep(src, '\nrunGroups = \{.*?\}\};', sprintf('\nrunGroups = {{''%s''}};', groupTag), 'once');
    src = regexprep(src, '\nboundSet = [^\n]*', sprintf('\nboundSet = ''S1'';'), 'once');
    plotCopy = fullfile(workDir, 'plot_stateFixed_copy.m');
    writeText(plotCopy, src);
    evalin('base', sprintf('run(''%s'')', plotCopy));   % 스크립트의 clear가 이 함수 workspace를 지우지 않게 한다
    plotDirs = dir(fullfile(cfg.resultsDir, 'stateFixed_plots', '*'));
    plotDirs = plotDirs([plotDirs.isdir] & ~startsWith({plotDirs.name}, '.'));
    plotDir = fullfile(plotDirs(end).folder, plotDirs(end).name);
    tag = 'speedFixed_min_pgate_cot_S1';
    check(isfile(fullfile(plotDir, ['minE_vs_speed_' tag '.png'])) && isfile(fullfile(plotDir, ['apgrf_r_' tag '.png'])), ...
        'plot_stateFixed draws the speed-CoT and per-v0 AP-GRF figures');
    plotted = readtable(fullfile(plotDir, ['minE_vs_speed_' tag '_data.csv']));
    check(isequal(plotted.Level, [1.3; 1.3; 1.5; 1.5]) && isequal(plotted.J, [1; 2; 1; 2]) && ...
        all(isfinite(plotted.E_actual)), 'plotted data: x = v0 levels, J interval index, y = E_actual');
    report.plotDir = plotDir;
    fprintf('[6] plot_stateFixed: %s\n', plotDir);
    fprintf('\nAll checks passed. Work folder: %s\n', workDir);
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
            error('verify_speed_fixed:WrongError', 'FAILED: %s (got %s: %s)', label, ME.identifier, ME.message);
        end
        return;
    end
    error('verify_speed_fixed:NoError', 'FAILED: %s (no error)', label);
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
        error('verify_speed_fixed:Failed', 'FAILED: %s', label);
    end
end
