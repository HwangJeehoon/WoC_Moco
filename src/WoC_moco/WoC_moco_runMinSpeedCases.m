function summary = WoC_moco_runMinSpeedCases(cfg)
% WoC_moco_runMinSpeedCases
%
%   같은 E0에 대해 propulsion target J_k마다 하나씩 다음 문제를 푼다
%   (opts.mocoObjectiveMode = 'minSpeedFixedEP'):
%
%     minimize    v_avg = (pelvis_tx(tf) - pelvis_tx(t0)) / (tf - t0)
%     subject to  E = E0, P = J_k, 기존 동역학/주기성/대칭성/bounds
%
%   각 case 결과를 저장된 궤적에서 다시 계산해 표/CSV와 그림으로 저장한다.
%   solver 수렴 여부와 equality 충족 여부를 따로 판정하며, 실패 case에서
%   target·bound·제약을 바꿔 다시 풀지 않는다.
%
%   cfg 필드:
%     E0, J_targets      : effort target, propulsion target 벡터 [N*s]
%     targetSource       : target 출처 설명 (CSV에 기록)
%     model, optMode, gaitMode, guessSto ('' 이면 WoC_moco_main 기본 guess)
%     timeBound, distBound : 반 걸음 기준 bound (modeAsym은 WoC_moco_main이 x2)
%     meshIntervals, convergenceTolerance, constraintTolerance, maxIterations
%     propulsionSmoothing : softplus 계수 s
%     effortTol, propulsionTol : equality 검증 허용오차 (|E-E0|, |P-J| [N*s])
%     resultsDir, runTag : 결과는 resultsDir/<runTag>_<mode>_J<k>, 요약은
%                          resultsDir/<runTag>_<mode>_summary 에 저장한다
%     continuation       : true면 case k가 직전 수렴 case의 해에서 시작
%     runOptimizations   : false면 이미 있는 결과만 요약한다
%
%   반환: summary table (summary.csv와 같은 내용)

    validateTargets(cfg.E0, cfg.J_targets);

    modeTag = lower(erase(cfg.gaitMode, 'mode'));
    isSym = strcmpi(cfg.gaitMode, 'modeSym');
    impulseScale = 1 + isSym;     % modeSym 반주기 적분 -> 한 보행주기
    boundScale = 1 + ~isSym;      % WoC_moco_main/moco_WoC_loop의 modeAsym bound 규칙
    [~, modelName] = fileparts(cfg.model);
    nCase = numel(cfg.J_targets);
    caseNames = arrayfun(@(k) sprintf('%s_%s_J%d', cfg.runTag, modeTag, k), ...
        1:nCase, 'UniformOutput', false);
    summaryDir = fullfile(cfg.resultsDir, sprintf('%s_%s_summary', cfg.runTag, modeTag));

    runMessages = strings(nCase, 1);
    if cfg.runOptimizations
        % 기존 결과를 덮어쓰지 않는다.
        for k = 1:nCase
            if isfolder(fullfile(cfg.resultsDir, caseNames{k}))
                error('결과 폴더가 이미 있습니다: %s (runTag를 바꾸세요)', caseNames{k});
            end
        end
        guess = cfg.guessSto;
        for k = 1:nCase
            opts = struct();
            opts.gaitMode = cfg.gaitMode;
            opts.mocoObjectiveMode = 'minSpeedFixedEP';
            opts.mocoEffortTarget = cfg.E0;
            opts.mocoPropulsionTarget = cfg.J_targets(k);
            opts.mocoPropulsionSmoothing = cfg.propulsionSmoothing;
            opts.mocoTimeBound = cfg.timeBound;
            opts.mocoDistBound = cfg.distBound;
            opts.mocoMeshIntervals = cfg.meshIntervals;
            opts.mocoConvergenceTolerance = cfg.convergenceTolerance;
            opts.mocoConstraintTolerance = cfg.constraintTolerance;
            opts.mocoMaxIterations = cfg.maxIterations;
            opts.guessInitSto = guess;
            fprintf('\n===== minSpeedFixedEP case %d/%d: E0=%.6g, J=%.6g N*s =====\n', ...
                k, nCase, cfg.E0, cfg.J_targets(k));
            try
                WoC_moco_main(cfg.model, 1, cfg.optMode, caseNames{k}, opts);
                runMessages(k) = "completed";
            catch ME
                runMessages(k) = string(ME.message);
                warning('WoC_moco_runMinSpeedCases:CaseFailed', ...
                    'Case %s did not finish: %s', caseNames{k}, ME.message);
            end
            if cfg.continuation
                solFile = solutionFile(fullfile(cfg.resultsDir, caseNames{k}), isSym, false);
                if isfile(solFile)
                    guess = solFile;
                end
            end
        end
    end

    rows = cell(nCase, 1);
    for k = 1:nCase
        rows{k} = summarizeCase(cfg, fullfile(cfg.resultsDir, caseNames{k}), ...
            caseNames{k}, k, isSym, impulseScale, boundScale, modelName);
        rows{k}.RunMessage = runMessages(k);
    end
    summary = struct2table([rows{:}]);

    if ~isfolder(summaryDir), mkdir(summaryDir); end
    writetable(summary, fullfile(summaryDir, 'summary.csv'));
    writeDefinitions(fullfile(summaryDir, 'definitions.txt'), cfg, impulseScale, boundScale);

    fprintf('\n===== minSpeedFixedEP summary (%s) =====\n', summaryDir);
    disp(summary(:, {'Case', 'Verdict', 'SolverStatus', 'E_target', 'E_actual', ...
        'E_residual', 'J_target', 'P_smooth_actual', 'P_residual', 'P_physical', ...
        'v_avg_mps', 'BoundHits'}));

    plotCases(summary, cfg, summaryDir);
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
        error('J_targets는 양의 유한 값의 벡터여야 합니다 [N*s].');
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


function row = summarizeCase(cfg, resultDir, caseName, k, isSym, impulseScale, ...
        boundScale, modelName)
    row = struct('Case', string(caseName), 'CaseIndex', k, ...
        'TargetSource', string(cfg.targetSource), 'Verdict', "", ...
        'SolverStatus', "missing", 'Converged', false, 'Iterations', NaN, ...
        'SolverTime_s', NaN, 'Objective', NaN, 'ObjectiveTerms', "", ...
        'OnlySpeedCost', false, ...
        'E_target', cfg.E0, 'E_actual', NaN, 'E_residual', NaN, 'E_pass', false, ...
        'J_target', cfg.J_targets(k), 'P_smooth_actual', NaN, 'P_residual', NaN, ...
        'P_pass', false, 'P_physical', NaN, 'P_smooth_minus_physical', NaN, ...
        'E_constraint_value', NaN, 'P_constraint_value', NaN, ...
        'v_avg_mps', NaN, 'PhaseDistance_m', NaN, 'PhaseDuration_s', NaN, ...
        'StrideLength_m', NaN, 'StrideTime_s', NaN, 'BoundHits', "", ...
        'E_trapz', NaN, 'E_muscle', NaN, 'E_lumbar', NaN, 'E_afo', NaN, ...
        'P_smooth_trapz', NaN, 'P_physical_simpson', NaN, ...
        'Indep_v_avg_mps', NaN, 'Indep_P_smooth', NaN, 'Indep_P_physical', NaN, ...
        'Indep_E_noAFO', NaN, ...
        'Model', string(cfg.model), 'OptMode', string(cfg.optMode), ...
        'GaitMode', string(cfg.gaitMode), 'Guess', "", ...
        'MeshIntervals', cfg.meshIntervals, 'ConvergenceTol', cfg.convergenceTolerance, ...
        'ConstraintTol', cfg.constraintTolerance, 'MaxIterations', cfg.maxIterations, ...
        'TimeBound_s', string(mat2str(cfg.timeBound * boundScale)), ...
        'DistBound_m', string(mat2str(cfg.distBound * boundScale)), ...
        'Smoothing', cfg.propulsionSmoothing, 'ImpulseScale', impulseScale, ...
        'EffortTol', cfg.effortTol, 'PropulsionTol', cfg.propulsionTol, ...
        'ResultDir', string(resultDir), 'RunMessage', "");

    unseal = false;
    solFile = solutionFile(resultDir, isSym, false);
    if ~isfile(solFile)
        solFile = solutionFile(resultDir, isSym, true);
        unseal = true;
    end
    paramsFile = fullfile(resultDir, 'result_1', 'moco_result', ...
        'moco_WoC_Solution_iter01_parameters.csv');
    if isfile(paramsFile)
        params = readtable(paramsFile, 'Format', '%s%s', 'TextType', 'string', ...
            'Delimiter', ',');
        row.Guess = strjoin(params.Value(params.Parameter == "guess_sto"), ";");
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
    m = moco_WoC_evalGaitMetrics(modelPath, solFile, impulseScale, cfg.propulsionSmoothing);
    row.OnlySpeedCost = isequal(termNames, {'objective_average_speed'}) && ...
        abs(row.Objective - m.v_avg) < 1e-6 * max(1, abs(m.v_avg));
    row.E_actual = m.E_simpson;
    row.E_residual = m.E_simpson - cfg.E0;
    row.E_pass = abs(row.E_residual) <= cfg.effortTol;
    row.P_smooth_actual = m.P_smooth_simpson;
    row.P_residual = m.P_smooth_simpson - cfg.J_targets(k);
    row.P_pass = abs(row.P_residual) <= cfg.propulsionTol;
    row.P_physical = m.P_physical_trapz;
    row.P_smooth_minus_physical = m.P_smooth_trapz - m.P_physical_trapz;
    row.v_avg_mps = m.v_avg;
    row.PhaseDistance_m = m.distance;
    row.PhaseDuration_s = m.duration;
    row.StrideLength_m = m.distance * impulseScale;
    row.StrideTime_s = m.duration * impulseScale;
    row.E_trapz = m.E_trapz;
    row.E_muscle = m.E_muscle_simpson;
    row.E_lumbar = m.E_lumbar_simpson;
    row.E_afo = m.E_afo_simpson;
    row.P_smooth_trapz = m.P_smooth_trapz;
    row.P_physical_simpson = m.P_physical_simpson;

    % Moco가 제약한 값: 적분 상태의 최종값 (상태는 t0에서 0).
    [sol, solLabels] = readSto(solFile);
    zE = sol(:, strcmp(solLabels, '/effortPropulsion/effort_integral'));
    zP = sol(:, strcmp(solLabels, '/effortPropulsion/positive_impulse'));
    if ~isempty(zE) && ~isempty(zP)
        row.E_constraint_value = zE(end) / m.com_displacement;
        row.P_constraint_value = impulseScale * zP(end);
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
    s = cfg.propulsionSmoothing;
    fap = [grf(:, strcmp(grfLabels, 'ground_force_r_vx')), ...
           grf(:, strcmp(grfLabels, 'ground_force_l_vx'))];
    row.Indep_P_smooth = sum(trapz(grf(:, 1), log1p(exp(s * fap)) / s));
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

    if ~row.Converged
        if row.E_pass && row.P_pass
            row.Verdict = "Not converged (equalities within tol)";
        else
            row.Verdict = "Not converged";
        end
    elseif row.E_pass && row.P_pass
        row.Verdict = "OK";
    else
        row.Verdict = "Converged, equality out of tol";
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
    cleaner = onCleanup(@() fclose(fid));
    line = '';
    while ~strcmp(strtrim(line), 'endheader')
        line = fgetl(fid);
    end
    labels = strsplit(strtrim(fgetl(fid)), '\t');
    data = textscan(fid, repmat('%f', 1, numel(labels)), 'CollectOutput', true);
    data = data{1};
end


function writeDefinitions(path, cfg, impulseScale, boundScale)
    fid = fopen(path, 'w');
    cleaner = onCleanup(@() fclose(fid));
    lines = {
        'minSpeedFixedEP: minimize v_avg s.t. E = E0, P = J_k (endpoint equality constraints)'
        sprintf('target source: %s', cfg.targetSource)
        ''
        'v_avg = (pelvis_tx(tf) - pelvis_tx(t0)) / (tf - t0)  [m/s] (only Moco cost, weight 1)'
        'E = (1/d_COM) * int sum_c u_c^2 dt'
        '    c = all model controls, weight 1: 18 muscle excitations, lumbarAct (CoordinateActuator,'
        '        optimal force 1, control = N*m), AFO_r/AFO_l (PrescribedController values;'
        '        MocoControlGoal ignore_controlled_actuators = false). No reserve actuators.'
        '    d_COM = |r_COM(tf) - r_COM(t0)|; identical to the weighted-mode effort goal before weight.'
        sprintf('P = %d * int_phase sum_foot softplus_s(F_foot) dt  [N*s] (one stride, both feet)', impulseScale)
        '    F_foot = contactHeel x-force + contactFront x-force on the foot (summed before the positive part)'
        sprintf('    softplus_s(F) = log(1 + exp(s*F))/s, s = %.4g (MocoOutputExtremumGoal maximum)', cfg.propulsionSmoothing)
        '    modeSym solves a half stride: the full stride P is 2x the half-stride integral; modeAsym: 1x.'
        '    No BW / reference-duration normalization and no reward sign (unlike the propulsion cost).'
        'P_physical = int sum_foot max(F_foot, 0) dt over the saved full stride (trapezoid)'
        ''
        'Constraint implementation: states z_E'' = sum_c u_c^2, z_P'' = sum_foot softplus_s(F_foot),'
        '    z(t0) = 0; MocoFinalOutputGoal endpoint constraints z_E(tf)/d_COM = E0 and'
        sprintf('    %d * z_P(tf) = J (E_constraint_value, P_constraint_value in summary.csv)', impulseScale)
        'E_actual, P_smooth_actual: recomputed from controls/contact forces with Simpson quadrature'
        '    on the saved mesh+midpoint grid (= Hermite-Simpson quadrature); used for E_pass/P_pass'
        '*_trapz: trapezoid on the same grid; Indep_*: saved full-stride .sto files only (no OpenSim)'
        'Indep_E_noAFO: trajectory controls only (AFO controls are not in the trajectory file)'
        sprintf('bounds (phase): time %s s, pelvis_tx %s m; BoundHits tolerance 1e-4', ...
            mat2str(cfg.timeBound * boundScale), mat2str(cfg.distBound * boundScale))
        sprintf('verification tolerances: |E - E0| <= %g, |P - J| <= %g N*s', cfg.effortTol, cfg.propulsionTol)
        'Verdict: solver convergence and equality satisfaction are judged separately;'
        '         a case that did not converge is not reported as infeasible.'
        };
    fprintf(fid, '%s\n', lines{:});
end


function plotCases(summary, cfg, outDir)
% Case 색은 J 순서(entity)로 고정한다. 성공/실패는 마커 채움과 라벨로 표시한다.
    colors = hex2rgb({'#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', ...
                      '#008300', '#6250d6', '#e34948'});
    ink = hex2rgb({'#0b0b0b', '#52514e'});
    n = height(summary);
    caseColor = @(k) colors(mod(k-1, size(colors, 1)) + 1, :);
    ok = summary.Verdict == "OK";
    shortVerdict = replace(summary.Verdict, ["Converged, equality out of tol", ...
        "Not converged (equalities within tol)"], ["eq. out of tol", "not converged, eq. ok"]);

    % 1. propulsion target에 따른 평균속도
    fig = newFigure([640 420]);
    ax = axes(fig); hold(ax, 'on'); styleAxes(ax, ink);
    for k = 1:n
        face = caseColor(k);
        if ~ok(k), face = 'w'; end
        plot(ax, summary.J_target(k), summary.v_avg_mps(k), 'o', 'MarkerSize', 9, ...
            'LineWidth', 2, 'MarkerEdgeColor', caseColor(k), 'MarkerFaceColor', face);
        text(ax, summary.J_target(k), summary.v_avg_mps(k), ...
            sprintf('  J%d: %s', k, shortVerdict(k)), 'Color', ink(2, :), 'FontSize', 9);
    end
    xlabel(ax, 'Propulsion target J (N\cdots, one stride, smooth)');
    ylabel(ax, 'Average speed v_{avg} (m/s)');
    title(ax, sprintf('Minimum average speed at E_0 = %.4g', cfg.E0), 'Color', ink(1, :));
    padLimits(ax, summary.J_target, summary.v_avg_mps);
    saveFigure(fig, fullfile(outDir, 'speed_vs_propulsion_target.png'));

    % 2. effort / propulsion target vs actual (small multiples, 각자 축)
    fig = newFigure([900 380]);
    tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact');
    specs = {'E_target', 'E_actual', [], 'Effort E (-)'; ...
             'J_target', 'P_smooth_actual', 'P_physical', 'Propulsion P (N\cdots, one stride)'};
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
            title(ax, 'Propulsion: target (bar), smooth (circle), max(F,0) (diamond)', ...
                'Color', ink(1, :));
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
                'DisplayName', sprintf('J%d = %.3g N\\cdots (%s)', k, summary.J_target(k), shortVerdict(k)));
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
% 한 보행주기 파일 (미수렴 case는 _unseal 파일).
    data = []; labels = {};
    dirPath = fullfile(row.ResultDir, 'result_1', 'moco_result');
    for prefix = {'moco_WoC_Solution_iter01', 'moco_WoC_Solution_iter01_unseal'}
        f = fullfile(dirPath, sprintf('%s_%s.sto', prefix{1}, kind));
        if isfile(f)
            [data, labels] = readSto(f);
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
    pad = @(v) [min(v) max(v)] + [-1 1] * max(0.1 * (max(v) - min(v)), 0.05 * max(abs(v)) + eps);
    if all(isfinite(x)), xlim(ax, pad(x)); end
    if all(isfinite(y)), ylim(ax, pad(y)); end
end


function saveFigure(fig, path)
    exportgraphics(fig, path, 'Resolution', 150);
    close(fig);
end


function rgb = hex2rgb(hex)
    rgb = cell2mat(cellfun(@(h) sscanf(h(2:end), '%2x%2x%2x')' / 255, hex(:), ...
        'UniformOutput', false));
end
