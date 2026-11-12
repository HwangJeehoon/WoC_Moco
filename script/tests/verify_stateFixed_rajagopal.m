function report = verify_stateFixed_rajagopal(workDir)
% verify_stateFixed_rajagopal
%
%   run_stateFixed_rajagopal.m의 문제 구성과 후처리 연결을 solve 없이 점검한다.
%   결과는 workDir (기본: tempdir 아래 새 폴더)에만 쓰며, 모델·입력·results는 수정하지 않는다.
%
%   1. 모델 load/init, component 경로 (접촉력 12, 이상적 구동기 13, AFO, controller)
%   2. 초기값 작업 사본 정렬 (pelvis_tx/tz, GRF CoP)과 정렬 상태로 다시 만든 GRF 비교
%   3. 초기값 지표 독립 재계산 (T, 거리, v_avg, CoT, P_gate; GRF 파일 적분)
%   4. J = 32/36/40 Moco 문제 구성 (moco_WoC_loop, mocoSkipSolve): 시간·거리 bound,
%      유일 cost -v_avg, CoT/P_gate 등식 제약, 주기성, 관절각·토크 구동기 bound,
%      12쌍 거리 제약, guess 포함 여부, AFO 0, multiplier/slack 보존
%   5. 전체 파이프라인 dry run (WoC_moco_runStateFixedCases, cfg.skipSolve): baseline
%      분석, GRF·대사 파일, 전체 모델 COM 분석, ID, summary.csv, 그림
%   6. 2D 기본 설정 문제 구성 (run_stateFixed.m 기본값, mocoSkipSolve)
%
%   실패하면 error. report에 수치를 담아 돌려준다.

    import org.opensim.modeling.*
    repo = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(genpath(fullfile(repo, 'src')));
    if nargin < 1 || isempty(workDir)
        workDir = fullfile(tempdir, ['verify_stateFixed_rajagopal_' char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'))]);
    end
    mkdir(workDir);
    previousLevel = Logger.getLevel();
    loggerCleanup = onCleanup(@() Logger.setLevel(previousLevel)); %#ok<NASGU>
    Logger.setLevelString('error');
    moco_WoC_loadFixedEffortPropulsionPlugin();

    modelFile = 'rajagopal_AFO.osim';
    modelPath = fullfile(repo, 'models', modelFile);
    guessSource = fullfile(repo, 'inputs', 'guess_init_rajagopal.sto');
    grfSource = fullfile(repo, 'inputs', 'GRF_init_rajagopal.sto');
    protected = {modelPath, guessSource, grfSource};
    hashes = cellfun(@fileHash, protected, 'UniformOutput', false);
    E0 = 5.6; J = [32, 36, 40]; timeBound = [0.25, 0.8]; distBound = [0.4, 1.5];
    profile = moco_WoC_modelProfile(modelFile);
    report = struct('workDir', workDir);

    %% 1. 모델과 component
    model = Model(modelPath);
    state = model.initSystem();
    check(strcmp(profile.name, 'rajagopal'), 'profile');
    check(model.hasComponent(profile.pelvisCoordinatePath), 'forward coordinate path');
    contacts = [profile.contactPaths.r, profile.contactPaths.l];
    check(numel(contacts) == 12 && all(cellfun(@(p) ~isempty(SmoothSphereHalfSpaceForce.safeDownCast( ...
        model.getComponent(p))), contacts)), '12 contact forces under /forceset');
    acts = componentsOfClass(model, 'ActivationCoordinateActuator');
    check(numel(acts) == 13, '13 ActivationCoordinateActuators');
    report.torqueOptimalForce = cellfun(@(p) ActivationCoordinateActuator.safeDownCast( ...
        model.getComponent(p)).getOptimalForce(), acts);
    groups = moco_WoC_controlGroups(model, profile);
    check(nnz(groups.isMuscle) == 80 && nnz(groups.isTorque & ~groups.isAfo) == 13 && ...
        nnz(groups.isPrescribed) == 2 && all(groups.isAfo == groups.isPrescribed), ...
        'control groups: 80 muscles, 13 torque actuators, 2 prescribed AFO');
    check(numel(groups.optimizationControls) == 93, '93 optimization controls');
    check(~any(groups.isMuscle & contains(groups.names, 'torque_')), 'torque actuators not classified as muscles');
    check(model.hasComponent('/AFO_r') && model.hasComponent('/AFO_l') && ...
        model.getControllerSet().getSize() == 1, 'AFO actuators and AFO_controller');
    check(model.getNumStateVariables() == 169, '169 physical states');
    report.mass = model.getTotalMass(state);
    fprintf('[1] model: mass %.4g kg, 169 states, 93 optimization controls, 12 contacts, 13 torque actuators\n', report.mass);

    %% 2. 정렬 사본과 GRF 재계산
    inputsDir = fullfile(workDir, 'inputs');
    align = moco_WoC_alignGaitStart(guessSource, grfSource, inputsDir, profile, 'rajagopal_init_aligned');
    traj = MocoTrajectory(align.kinematicsPath);
    tx = traj.getStateMat('/jointset/ground_pelvis/pelvis_tx/value');
    tz = traj.getStateMat('/jointset/ground_pelvis/pelvis_tz/value');
    check(tx(1) == 0 && tz(1) == 0, 'aligned pelvis_tx/tz start at 0');
    statesTraj = StatesTrajectory.createFromStatesTable(model, traj.exportToStatesTable(), false, true);
    cr = StdVectorString(); cl = StdVectorString();
    cellfun(@(p) cr.add(p), profile.contactPaths.r); cellfun(@(p) cl.add(p), profile.contactPaths.l);
    recomputed = opensimMoco.createExternalLoadsTableForGait(model, statesTraj, cr, cl);
    recomputedPath = fullfile(workDir, 'grf_recomputed_from_aligned_states.sto');
    STOFileAdapter.write(recomputed, recomputedPath);
    [aligned, la] = readSto(align.grfPath);
    [rec, lr] = readSto(recomputedPath);
    forceCols = {'ground_force_r_vx', 'ground_force_r_vy', 'ground_force_r_vz', ...
                 'ground_force_l_vx', 'ground_force_l_vy', 'ground_force_l_vz'};
    copCols = {'ground_force_r_px', 'ground_force_r_pz', 'ground_force_l_px', 'ground_force_l_pz'};
    dF = maxColumnDiff(aligned, la, rec, lr, forceCols);
    % CoP는 접촉이 거의 없을 때 정의가 불안정하므로 해당 발 vy > 20 N 구간에서 비교한다.
    dP = maxCopDiff(aligned, la, rec, lr, copCols, 20);
    report.grfRecomputeMaxForceDiff = dF;
    report.grfRecomputeMaxCopDiff = dP;
    check(dF < 1e-6 && dP < 1e-6, sprintf('aligned GRF = GRF recomputed from aligned states (force %.2g N, CoP %.2g m)', dF, dP));
    fprintf('[2] aligned copy: offsets x %.6g m, z %.6g m; GRF recompute max diff force %.2g N, CoP %.2g m\n', ...
        align.offsetX, align.offsetZ, dF, dP);

    %% 3. 초기값 지표
    gm = moco_WoC_evalGaitMetrics(modelPath, align.kinematicsPath, 1, 1.0, 5.0);
    [grf, gl] = readSto(align.grfPath);
    Fap = [grf(:, strcmp(gl, 'ground_force_r_vx')), grf(:, strcmp(gl, 'ground_force_l_vx'))];
    gGate = 1 ./ (1 + exp(-(Fap - 5) / 1));
    report.guess = struct('T', gm.duration, 'distance', gm.distance, 'v_avg', gm.v_avg, ...
        'CoT_simpson', gm.CoT_simpson, 'CoT_trapz', gm.CoT_trapz, 'P_gate_simpson', gm.P_gate_simpson, ...
        'P_gate_trapz', gm.P_gate_trapz, 'P_gate_grf_file', sum(trapz(grf(:, 1), Fap .* gGate)), ...
        'com_displacement', gm.com_displacement, 'basal_W', gm.basal_rate_W);
    check(abs(gm.duration - 1.06) < 1e-9 && abs(gm.distance - 1.325) < 1e-6 && abs(gm.v_avg - 1.25) < 1e-6, ...
        'guess T 1.06 s, distance 1.325 m, v 1.25 m/s');
    check(abs(gm.CoT_simpson - 5.665) < 0.01 && abs(gm.P_gate_simpson - 36.22) < 0.05, ...
        'guess CoT ~5.665 J/kg/m and P_gate ~36.22 N*s');
    check(abs(report.guess.P_gate_grf_file - gm.P_gate_trapz) < 1e-6, 'P_gate from GRF file = model recomputation (trapz)');
    fprintf(['[3] guess: T %.4g s, d %.5g m, v %.5g m/s, CoT %.6g (trapz %.6g) J/kg/m, P_gate %.6g ', ...
             '(trapz %.6g, GRF file %.6g) N*s, d_COM %.5g m\n'], gm.duration, gm.distance, gm.v_avg, ...
        gm.CoT_simpson, gm.CoT_trapz, gm.P_gate_simpson, gm.P_gate_trapz, report.guess.P_gate_grf_file, ...
        gm.com_displacement);

    %% 4. Moco 문제 구성 (solve 없음)
    ctrlPath = fullfile(workDir, 'control_zero.sto');
    fid = fopen(ctrlPath, 'w');
    fprintf(fid, 'controls\nnRows=2\nnColumns=3\nendheader\ntime\tAFO_r\tAFO_l\n0\t0\t0\n100\t0\t0\n');
    fclose(fid);
    source = MocoTrajectory(guessSource);
    for k = 1:numel(J)
        caseDir = fullfile(workDir, sprintf('problem_J%g', J(k)));
        mkdir(caseDir);
        opts = struct('mocoEffort', 0, 'mocoFinalTime', 0, 'mocoObjectiveMode', 'maxSpeedStateFixed', ...
            'mocoEnergyMetric', 'metabolicCoT', 'mocoEnergyTarget', E0, 'mocoPropulsionTarget', J(k), ...
            'mocoPropulsionMetric', 'impulse', 'mocoPropulsionGateSigma', 1, ...
            'mocoPropulsionGateThreshold', 5, 'mocoSpeedWeight', 1, 'gaitMode', 'modeAsym', ...
            'mocoTimeBound', timeBound, 'mocoDistBound', distBound, 'mocoMeshIntervals', 300, ...
            'mocoConvergenceTolerance', 5e-5, 'mocoConstraintTolerance', 5e-5, ...
            'mocoMaxIterations', 5000, 'mocoSkipSolve', true);
        [~, setup] = moco_WoC_loop(ctrlPath, align.kinematicsPath, 1, caseDir, modelPath, opts);
        problem = setup.problem;
        rep = problem.createRep();
        phase = problem.getPhase(0);
        ti = bounds2(phase.getTimeInitialBounds()); tf = bounds2(phase.getTimeFinalBounds());
        check(isequal(ti, [0 0]) && max(abs(tf - [0.5 1.6])) < 1e-12, sprintf('J%g time bounds t0 = 0, tf [0.5 1.6]', J(k)));
        txInfo = rep.getStateInfo('/jointset/ground_pelvis/pelvis_tx/value');
        check(isequal(bounds2(txInfo.getInitialBounds()), [0 0]) && ...
            max(abs(bounds2(txInfo.getFinalBounds()) - [0.8 3.0])) < 1e-12, ...
            sprintf('J%g pelvis_tx(t0) = 0, pelvis_tx(tf) [0.8 3.0]', J(k)));
        check(~rep.getStateInfo('/jointset/ground_pelvis/pelvis_tx/speed').getBounds().isSet() || ...
            isequal(bounds2(rep.getStateInfo('/jointset/ground_pelvis/pelvis_tx/speed').getBounds()), ...
            bounds2(rep.getStateInfo('/jointset/hip_r/hip_flexion_r/speed').getBounds())), ...
            sprintf('J%g no extra pelvis speed bound', J(k)));
        costs = cellFrom(rep.createCostNames());
        check(isequal(costs, {'average_speed'}), sprintf('J%g only cost average_speed', J(k)));
        speedGoal = rep.getCost('average_speed');
        check(speedGoal.getWeight() == -1 && strcmp(char(PropertyHelper.getValueString( ...
            speedGoal.getPropertyByName('coordinate_path'))), profile.pelvisCoordinatePath), ...
            sprintf('J%g speed goal weight -1 on %s', J(k), profile.pelvisCoordinatePath));
        cot = MocoFinalOutputGoal.safeDownCast(rep.getEndpointConstraint('metabolic_cot_constraint'));
        prop = MocoFinalOutputGoal.safeDownCast(rep.getEndpointConstraint('propulsion_constraint'));
        check(isequal(goalBounds(cot), [E0 E0]) && cot.getDivideByMass() && cot.getDivideByDisplacement(), ...
            sprintf('J%g CoT = %g equality', J(k), E0));
        check(isequal(goalBounds(prop), [J(k) J(k)]), sprintf('J%g P_gate = %g equality', J(k), J(k)));
        integrator = setup.model.getComponent('/effortPropulsion');
        check(PropertyHelper.getValueDouble(integrator.getPropertyByName('impulse_scale')) == 1 && ...
            integrator.getInput('right_contact_forces').getNumConnectees() == 6 && ...
            integrator.getInput('left_contact_forces').getNumConnectees() == 6, ...
            sprintf('J%g integrator: impulse_scale 1, 6+6 contact inputs', J(k)));
        check(isempty(setup.profileChecks.problems) && setup.profileChecks.numPeriodicStatePairs == 164 && ...
            setup.profileChecks.numPeriodicControlPairs == 93 && setup.profileChecks.framePairs == 12, ...
            sprintf('J%g periodicity (164 states, 93 controls) and 12 frame pairs', J(k)));
        check(setup.guessOutputs.afoControlMaxAbs == 0, sprintf('J%g AFO control 0 on the guess', J(k)));
        check(isempty(setup.guessBounds.violations), sprintf('J%g guess within all bounds', J(k)));
        if k == 1
            report.problem = checkBoundsDetail(rep, setup, profile, source, problem);
            report.guessOutputs = setup.guessOutputs;
        end
        report.(sprintf('problem_J%g', J(k))) = struct('tf', tf, 'costs', {costs}, ...
            'endpoint', {cellFrom(rep.createEndpointConstraintNames())}, ...
            'guess_metabolic_energy', setup.guessOutputs.metabolic_energy, ...
            'guess_scaled_impulse', setup.guessOutputs.scaled_propulsive_impulse);
        fprintf('[4] J = %g: problem OK (cost %s; tf %s; endpoint %s)\n', J(k), strjoin(costs, ','), ...
            mat2str(tf), strjoin(cellFrom(rep.createEndpointConstraintNames()), ','));
    end

    %% 5. 전체 파이프라인 dry run
    cfg = struct();
    cfg.model = modelFile; cfg.objectiveMode = 'maxSpeedStateFixed'; cfg.energyMetric = 'metabolicCoT';
    cfg.propulsionMetric = 'impulse'; cfg.gaitMode = 'modeAsym'; cfg.optMode = 'modeOff';
    cfg.speedWeight = 1; cfg.gateSigma = 1; cfg.gateThreshold = 5; cfg.meshIntervals = 300;
    cfg.convergenceTolerance = 5e-5; cfg.constraintTolerance = 5e-5; cfg.maxIterations = 5000;
    cfg.energyTol = 1e-5; cfg.propulsionTol = 1e-2; cfg.continuation = false; cfg.runOptimizations = true;
    cfg.resultsDir = fullfile(workDir, 'results'); mkdir(cfg.resultsDir);
    cfg.Index = 'R'; cfg.J_targets = J; cfg.caseNameUsesTargetValue = true;
    cfg.targetSource = 'verify_stateFixed_rajagopal'; cfg.E0 = E0;
    cfg.timeBound = timeBound; cfg.distBound = distBound;
    cfg.guessSto = repmat({align.kinematicsPath}, 1, numel(J));
    cfg.runTag = 'stateFixed_max_cot_rajagopalAFO_off_verify_E5p6_X3';
    cfg.skipSolve = true;
    summary = WoC_moco_runStateFixedCases(cfg);
    report.summary = summary;
    check(height(summary) == 3 && all(summary.Verdict == "Setup check (no solve)") && ...
        all(summary.RunMessage == "completed"), 'dry run: 3 cases completed, Verdict Setup check');
    check(all(summary.Model == modelFile) && all(summary.ModelProfile == "rajagopal") && ...
        all(summary.TimeBoundInput_s == "[0.25 0.8]") && all(summary.TimeBound_s == "[0.5 1.6]") && ...
        all(summary.DistBoundInput_m == "[0.4 1.5]") && all(summary.DistBound_m == "[0.8 3]") && ...
        isequal(summary.P_gate_target', J) && all(summary.E_target == E0) && ...
        all(summary.ObjectiveMode == "maxSpeedStateFixed") && all(summary.GaitMode == "modeAsym") && ...
        all(summary.OptMode == "modeOff") && all(summary.ContactForcesPerFoot == 6), ...
        'summary records model, bounds (input/applied), E0, J, objective, gait/opt mode');
    check(all(abs(summary.P_gate_actual - gm.P_gate_simpson) < 1e-6) && ...
        all(abs(summary.CoT_simpson - gm.CoT_simpson) < 1e-6) && all(abs(summary.v_avg_mps - 1.25) < 1e-6) && ...
        all(abs(summary.Indep_P_gate - report.guess.P_gate_grf_file) < 1e-3) && ...
        all(abs(summary.Indep_v_avg_mps - 1.25) < 1e-6) && all(isfinite(summary.Indep_Effort_noAFO)), ...
        'summary metrics from the saved files (P_gate, CoT, v_avg, Indep_*)');
    check(all(abs(summary.CoT_constraint_value - gm.CoT_trapz) < 1e-3) && ...
        all(abs(summary.P_gate_constraint_value - gm.P_gate_trapz) < 1e-6), ...
        'integral states of the guess (z_met, z_P) consistent with recomputation');
    caseDir = char(summary.ResultDir(1));
    moco = fullfile(caseDir, 'result_1', 'moco_result');
    analy = fullfile(caseDir, 'result_1', 'analy_result');
    [g1, gl1] = readSto(fullfile(moco, 'moco_WoC_Solution_iter01_GRF.sto'));
    dF = maxColumnDiff(g1, gl1, aligned, la, forceCols);
    dP = maxCopDiff(g1, gl1, aligned, la, copCols, 20);
    check(dF < 1e-6 && dP < 1e-6, 'pipeline GRF (6 contacts per foot, contact-force CoP) = aligned GRF');
    [com, cl2] = readSto(fullfile(analy, [profile.analysisPrefix profile.comFileSuffix]));
    comX = com(:, strcmp(cl2, 'center_of_mass_X'));
    check(abs((comX(end) - comX(1)) - gm.com_displacement) < 5e-3, ...
        sprintf('BodyKinematics whole-model COM displacement %.5g m (model %.5g m)', comX(end) - comX(1), gm.com_displacement));
    required = {fullfile(moco, 'moco_WoC_Solution_iter01_kinematics.sto'), ...
                fullfile(moco, 'moco_WoC_Solution_iter01_metabolic.sto'), ...
                fullfile(analy, 'rajagopal_AFO_Kinematics_q.sto'), ...
                fullfile(analy, 'id_withAssist.sto'), fullfile(analy, 'id_withoutAssist.sto'), ...
                fullfile(analy, 'rajagopal_AFO_1.osim')};
    check(all(isfile(required)), 'pipeline files: kinematics, metabolic, Kinematics_q, ID with/without assist');
    summaryDirs = dir(fullfile(cfg.resultsDir, [cfg.runTag '_asym_summary'], '*'));
    summaryDirs = summaryDirs([summaryDirs.isdir] & ~startsWith({summaryDirs.name}, '.'));
    sdir = fullfile(summaryDirs(end).folder, summaryDirs(end).name);
    pngs = dir(fullfile(sdir, '*.png'));
    check(isfile(fullfile(sdir, 'summary.csv')) && isfile(fullfile(sdir, 'definitions.txt')) && numel(pngs) == 4, ...
        'summary.csv, definitions.txt and 4 figures');
    report.summaryDir = sdir;
    fprintf('[5] dry run: %s\n', sdir);

    %% 6. 2D 기본 설정 문제 구성
    dir2d = fullfile(workDir, 'problem_2D');
    mkdir(dir2d);
    opts2d = struct('mocoEffort', 0, 'mocoFinalTime', 0, 'mocoObjectiveMode', 'maxSpeedStateFixed', ...
        'mocoEnergyMetric', 'metabolicCoT', 'mocoEnergyTarget', 3.0, 'mocoPropulsionTarget', 30, ...
        'mocoPropulsionMetric', 'impulse', 'mocoPropulsionGateSigma', 1, 'mocoPropulsionGateThreshold', 5, ...
        'mocoSpeedWeight', 1, 'gaitMode', 'modeSym', 'mocoTimeBound', timeBound, 'mocoDistBound', distBound, ...
        'mocoMeshIntervals', 300, 'mocoConvergenceTolerance', 5e-5, 'mocoConstraintTolerance', 5e-5, ...
        'mocoMaxIterations', 5000, 'mocoLumbarControlBound', [-1 1], 'mocoSkipSolve', true);
    [~, s2] = moco_WoC_loop(ctrlPath, fullfile(repo, 'inputs', 'guess_init_stateFixed.sto'), 1, dir2d, ...
        fullfile(repo, 'models', '2D_gait_AFO_pc.osim'), opts2d);
    rep2 = s2.problem.createRep();
    check(strcmp(s2.profile.name, '2D') && isequal(cellFrom(rep2.createCostNames()), {'average_speed'}) && ...
        isequal(bounds2(rep2.getControlInfo('/lumbarAct').getBounds()), [-1 1]) && ...
        isequal(bounds2(rep2.getStateInfo('/jointset/groundPelvis/pelvis_ty/value').getBounds()), [0.75 1.25]) && ...
        ~isfile(fullfile(dir2d, 'moco_WoC_guess_problem_order.sto')), ...
        '2D default problem: speed cost, lumbarAct [-1 1], 2D bounds, legacy guess path');
    fprintf('[6] 2D default (modeSym, CoT, lumbarAct bound) problem builds unchanged\n');

    check(isequal(hashes, cellfun(@fileHash, protected, 'UniformOutput', false)), ...
        'model and input files unchanged');
    fprintf('\nAll checks passed. Work folder: %s\n', workDir);
end


function out = checkBoundsDetail(rep, setup, profile, source, problem)
% 관절각, 토크 구동기, 근육 bound와 multiplier/slack 보존을 수치로 확인한다.
    import org.opensim.modeling.*
    out = struct();
    rows = setup.bounds.coordinates;
    for r = 1:size(rows, 1)
        b = bounds2(rep.getStateInfo(rows{r, 1}).getBounds());
        check(max(abs(b - rows{r, 2} * pi / 180)) < 1e-15, sprintf('coordinate bound %s', rows{r, 1}));
    end
    check(size(rows, 1) == 32, '32 coordinate bounds (18 rows, sided ones x2, 2 knee beta)');
    beta = bounds2(rep.getStateInfo('/jointset/patellofemoral_r/knee_angle_r_beta/value').getBounds());
    check(max(abs(beta - [-120 120] * pi / 180)) < 1e-15, 'knee beta [-120, 120] deg');
    knee = bounds2(rep.getStateInfo('/jointset/walker_knee_r/knee_angle_r/value').getBounds());
    check(isequal(knee, [0, 120 * pi / 180]), 'knee flexion positive [0, 120] deg');
    out.coordinateBounds = rows;
    t = setup.bounds.torque;
    lim = containers.Map(profile.torqueLimits(:, 1), profile.torqueLimits(:, 2));
    for r = 1:size(t, 1)
        base = regexprep(t{r, 2}, '_[rl]$', '');
        bExpected = lim(base) / t{r, 4};
        c = bounds2(rep.getControlInfo(t{r, 1}).getBounds());
        a = bounds2(rep.getStateInfo([t{r, 1} '/activation']).getBounds());
        check(isequal(c, [-bExpected, bExpected]) && isequal(a, c), sprintf('torque bounds %s', t{r, 1}));
    end
    out.torqueBounds = t;
    musclePaths = cellFrom(rep.createControlInfoNames());
    musclePaths = musclePaths(~startsWith(musclePaths, '/forceset/torque_'));
    for k = 1:numel(musclePaths)
        c = bounds2(rep.getControlInfo(musclePaths{k}).getBounds());
        a = bounds2(rep.getStateInfo([musclePaths{k} '/activation']).getBounds());
        check(isequal(c, [0.01 1]) && isequal(a, [0.01 1]), sprintf('muscle bounds %s', musclePaths{k}));
    end
    out.numMuscleBounds = numel(musclePaths);
    g = setup.guess;
    names = cellFrom(g.getMultiplierNames());
    check(isequal(sort(names), sort(cellFrom(source.getMultiplierNames()))), 'multiplier names preserved');
    for k = 1:numel(names)
        check(max(abs(g.getMultiplierMat(names{k}) - source.getMultiplierMat(names{k}))) == 0, ...
            sprintf('multiplier %s values preserved', names{k}));
    end
    sl = slackColumns(setup.guess, setup.injectedOsimPath);
    check(numel(sl) == 2, 'two slack columns in the problem-order guess');
    out.multipliers = names;
    out.slacks = sl;
    stateNames = cellFrom(source.getStateNames());
    for k = 1:numel(stateNames)
        check(max(abs(g.getStateMat(stateNames{k}) - sourceAligned(source, stateNames{k}))) < 1e-12, ...
            sprintf('state %s copied by name', stateNames{k}));
    end
    controlNames = cellFrom(source.getControlNames());
    for k = 1:numel(controlNames)
        check(isequal(g.getControlMat(controlNames{k}), source.getControlMat(controlNames{k})), ...
            sprintf('control %s copied by name', controlNames{k}));
    end
    out.timeFinalBounds = bounds2(problem.getPhase(0).getTimeFinalBounds());
end


function v = sourceAligned(source, name)
    v = source.getStateMat(name);
    if any(strcmp(name, {'/jointset/ground_pelvis/pelvis_tx/value', '/jointset/ground_pelvis/pelvis_tz/value'}))
        v = v - v(1);
    end
end


function names = slackColumns(~, injectedOsimPath)
% 문제 순서 guess 파일에서 slack 열 이름 (MATLAB binding에는 slack getter가 없다).
    file = fullfile(fileparts(injectedOsimPath), 'moco_WoC_guess_problem_order.sto');
    fid = fopen(file); cleaner = onCleanup(@() fclose(fid));
    header = {};
    line = fgetl(fid);
    while ~strcmp(strtrim(line), 'endheader'), header{end+1} = line; line = fgetl(fid); end %#ok<AGROW>
    labels = strsplit(strtrim(fgetl(fid)), '\t');
    count = @(key) str2double(extractAfter(header{startsWith(header, [key '='])}, '='));
    first = 1 + count('num_states') + count('num_controls') + count('num_multipliers') + count('num_derivatives');
    names = labels(first + (1:count('num_slacks')));
end


function paths = componentsOfClass(model, className)
    paths = {};
    list = model.getComponentsList();
    it = list.begin();
    while ~it.equals(list.end())
        c = it.deref();
        if strcmp(char(c.getConcreteClassName()), className)
            paths{end+1} = char(c.getAbsolutePathString()); %#ok<AGROW>
        end
        it.next();
    end
end


function d = maxColumnDiff(a, la, b, lb, cols)
    d = 0;
    for k = 1:numel(cols)
        d = max(d, max(abs(a(:, strcmp(la, cols{k})) - b(:, strcmp(lb, cols{k})))));
    end
end


function d = maxCopDiff(a, la, b, lb, cols, minVy)
    d = 0;
    for k = 1:numel(cols)
        side = cols{k}(14);
        loaded = a(:, strcmp(la, ['ground_force_' side '_vy'])) > minVy;
        d = max(d, max(abs(a(loaded, strcmp(la, cols{k})) - b(loaded, strcmp(lb, cols{k})))));
    end
end


function b = bounds2(bounds)
    b = [bounds.getLower(), bounds.getUpper()];
end


function b = goalBounds(goal)
    import org.opensim.modeling.*
    info = goal.getPropertyByName('MocoConstraintInfo').getValueAsObject(0);
    list = info.getPropertyByName('bounds');
    m = MocoBounds.safeDownCast(list.getValueAsObject(0));
    b = [m.getLower(), m.getUpper()];
end


function c = cellFrom(v)
    c = cell(1, v.size());
    for k = 0:v.size()-1, c{k+1} = char(v.get(k)); end
end


function [data, labels] = readSto(path)
    fid = fopen(path);
    cleaner = onCleanup(@() fclose(fid));
    line = '';
    while ~strcmp(strtrim(line), 'endheader'), line = fgetl(fid); end
    labels = strsplit(strtrim(fgetl(fid)), '\t');
    data = textscan(fid, repmat('%f', 1, numel(labels)), 'CollectOutput', true);
    data = data{1};
end


function check(ok, label)
    if ~ok
        error('verify_stateFixed_rajagopal:Failed', 'FAILED: %s', label);
    end
end


function hash = fileHash(path)
    fid = fopen(path, 'rb');
    cleanup = onCleanup(@() fclose(fid));
    bytes = fread(fid, Inf, '*uint8');
    md = java.security.MessageDigest.getInstance('SHA-256');
    md.update(typecast(bytes, 'int8'));
    hash = lower(reshape(dec2hex(typecast(md.digest(), 'uint8'), 2)', 1, []));
end
