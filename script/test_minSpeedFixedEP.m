% test_minSpeedFixedEP.m
%
% minSpeedFixedEP 수렴 검증. 수렴한 기존 궤적에서 새 정의의 E와 P_gate를 계산해
% 검증용 target을 만든다(실험 target이 아님):
%   E0 = E(reference), J = P_gate(reference) * J_scale
%
%   C. 알려진 feasible reference: reference와 같은 모델, 보조력(modeOff), bounds,
%      mesh(300)와 tolerance로 J_scale = 1을 푼다. 수렴, E/P_gate equality,
%      목적함수 = 평균속도만, 제약값(적분 상태) = 재계산값을 assertion으로 확인한다.
%   D. 작은 mesh(50) 배치: 같은 E0에서 J_scale = [0.95 1 1.05]를 같은 guess로 독립
%      실행한다. reference 주변의 다른 J는 feasibility가 보장되지 않으므로 수렴을
%      요구하지 않고, 배치가 모든 case를 기록하는지만 assertion으로 확인한다.
%      J_scale = 1 case는 mesh 변경(300 -> 50)의 영향을 따로 보여 준다.
% C의 assertion이 하나라도 실패하면 error로 끝난다. 결과 이름에 실행 시각이
% 들어가므로 기존 결과를 덮어쓰지 않는다.
clc; clear; close all;
thisScriptDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(thisScriptDir, '..', 'src')));

resultsDir = fullfile(thisScriptDir, '..', 'results');
referenceName = 'wfrac_s0p2_sym_ref_e1_t0p3_p0';   % modeSym, modeOff, 수렴, mesh 300, tol 1e-3
stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));

cfg.model = '2D_gait_AFO_pc.osim';
cfg.optMode = 'modeOff';
cfg.gaitMode = 'modeSym';
cfg.timeBound = [0.4, 0.8];     % reference run과 같은 bound (기본값)
cfg.distBound = [0.4, 1.0];
cfg.convergenceTolerance = 1e-3;
cfg.constraintTolerance = 1e-3;
cfg.maxIterations = 3000;
cfg.gateSigma = 1.0;
cfg.gateThreshold = 5.0;
cfg.effortTol = 1e-3;
cfg.propulsionTol = 1e-2;
cfg.resultsDir = resultsDir;
cfg.continuation = false;
cfg.runOptimizations = true;

% 검증용 target: reference 궤적의 control과 접촉력에서 계산.
refDir = fullfile(resultsDir, referenceName, 'result_1');
cfg.guessSto = fullfile(refDir, 'moco_result', 'moco_WoC_Solution_iter01_kinematics_half.sto');
ref = moco_WoC_evalGaitMetrics(fullfile(refDir, 'analy_result', '2D_gait_AFO_pc_1.osim'), ...
    cfg.guessSto, 2, cfg.gateSigma, cfg.gateThreshold);
fprintf(['reference %s: E=%.6g, P_gate=%.6g N*s, Tplus=%.6g s, mean force=%.6g N, ', ...
         'P_physical=%.6g N*s, v_avg=%.4f m/s, phase %.4f s / %.4f m\n'], referenceName, ...
         ref.E_simpson, ref.P_gate_simpson, ref.Tplus_simpson, ref.mean_propulsive_force, ...
         ref.P_physical_trapz, ref.v_avg, ref.duration, ref.distance);
cfg.E0 = ref.E_simpson;

%% C. reference (mesh 300)
cfgC = cfg;
cfgC.meshIntervals = 300;
cfgC.J_targets = ref.P_gate_simpson;
cfgC.runTag = ['test_gate_ref_m300_' stamp];
cfgC.targetSource = sprintf(['verification only: E0, P_gate from converged %s ', ...
    '(not experimental targets), mesh 300'], referenceName);
summaryC = WoC_moco_runMinSpeedCases(cfgC);

%% D. 배치 (mesh 50)
cfgD = cfg;
cfgD.meshIntervals = 50;
J_scale = [0.95, 1.00, 1.05];
cfgD.J_targets = ref.P_gate_simpson * J_scale;
cfgD.runTag = ['test_gate_batch_m50_' stamp];
cfgD.targetSource = sprintf(['verification only: E0 and P_gate x [%s] from converged ', ...
    '%s (not experimental targets), mesh 50'], num2str(J_scale), referenceName);
summaryD = WoC_moco_runMinSpeedCases(cfgD);

%% Assertions
c = summaryC(1, :);
checks = {
    c.PropulsionDefinition == "gate_Iplus", "C: propulsion_definition이 gate_Iplus가 아님"
    c.Converged, "C: solver 미수렴 (" + c.SolverStatus + ")"
    c.E_pass, sprintf("C: |E - E0| = %.3g > %.3g", abs(c.E_residual), cfg.effortTol)
    c.P_gate_pass, sprintf("C: |P_gate - J| = %.3g > %.3g N*s", abs(c.P_gate_residual), cfg.propulsionTol)
    c.OnlySpeedCost, "C: objective term이 average_speed 하나가 아니거나 objective ~= v_avg (" + c.ObjectiveTerms + ")"
    abs(c.E_constraint_value - c.E_actual) <= cfg.effortTol && ...
        abs(c.P_gate_constraint_value - c.P_gate_actual) <= cfg.propulsionTol, ...
        "C: 적분 상태 제약값과 control/접촉력 재계산값이 다름"
    abs(c.Indep_P_gate - c.P_gate_trapz) <= 1e-6 * c.P_gate_trapz, "C: 저장 파일 독립 재계산 P_gate가 다름"
    abs(c.Indep_v_avg_mps - c.v_avg_mps) <= 1e-9, "C: 독립 재계산 v_avg가 다름"
    height(summaryD) == numel(J_scale) && all(summaryD.Verdict ~= ""), "D: 배치 요약에 빠진 case가 있음"
    all(summaryD.Guess == string(cfg.guessSto)), "D: case가 같은 guess에서 독립 실행되지 않음"
    };
failures = string(checks(~cellfun(@(x) isequal(x, true), checks(:, 1)), 2));

fprintf('\nC (mesh 300, J = P_gate_ref): %s, v_avg = %.4f m/s (reference %.4f m/s), bounds: %s\n', ...
    c.Verdict, c.v_avg_mps, ref.v_avg, c.BoundHits);
fprintf('D (mesh 50): %s\n', strjoin(summaryD.Case + ": " + summaryD.Verdict, ' | '));
if ~isempty(failures)
    error('test_minSpeedFixedEP FAILED:\n  %s', strjoin(failures, '\n  '));
end
fprintf('PASS: reference case converged with E and P_gate equalities and v_avg as the only cost.\n');
