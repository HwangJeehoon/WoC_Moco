% test_minSpeedFixedEP.m
%
% minSpeedFixedEP smoke test. 수렴한 기존 궤적에서 E와 P를 계산해 검증용
% target을 만들고(실험 target이 아님), 작은 mesh로 세 case를 푼다.
%   E0 = E(reference), J_k = P(reference) * J_scale(k)
% J_scale = 1 case는 reference 궤적 자체가 feasible point이다.
% 실험 target은 run_minSpeedFixedEP.m 에 입력한다.
%
% boundSet (batch 실행 예: matlab -batch "boundSet='narrow'; test_minSpeedFixedEP")
%   'default': reference run과 같은 기본 bound
%   'narrow' : reference(0.41 s, 0.68 m) 근방 bound. 기본 bound에서 iterate가
%              속도 하한 모서리(0.4 m / 0.8 s)로 가서 수렴하지 않았기 때문에,
%              수렴한 해에서 equality 구현을 확인하려는 별도 test 설정이다.
clc; clearvars -except boundSet; close all;
if ~exist('boundSet', 'var'), boundSet = 'default'; end
thisScriptDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(thisScriptDir, '..', 'src')));

resultsDir = fullfile(thisScriptDir, '..', 'results');
referenceName = 'wfrac_s0p2_sym_ref_e1_t0p3_p0';   % modeSym, modeOff, 수렴, mesh 300
J_scale = [0.95, 1.00, 1.05];

cfg.model = '2D_gait_AFO_pc.osim';
cfg.optMode = 'modeOff';
cfg.gaitMode = 'modeSym';
switch boundSet
    case 'default'
        cfg.timeBound = [0.4, 0.8];     % reference run과 같은 bound (기본값)
        cfg.distBound = [0.4, 1.0];
        runTag = 'smoke_minspeed_m50';
    case 'narrow'
        cfg.timeBound = [0.4, 0.5];
        cfg.distBound = [0.6, 0.8];
        runTag = 'smoke_minspeed_m50_nb';
    otherwise
        error('boundSet은 ''default'' 또는 ''narrow''여야 합니다.');
end
cfg.meshIntervals = 50;
cfg.convergenceTolerance = 1e-3;
cfg.constraintTolerance = 1e-3;
cfg.maxIterations = 2000;
cfg.propulsionSmoothing = 0.2;
cfg.effortTol = 1e-3;
cfg.propulsionTol = 1e-2;
cfg.resultsDir = resultsDir;
cfg.runTag = runTag;
cfg.continuation = false;
cfg.runOptimizations = true;

% 검증용 target: reference 궤적에서 계산 (moco_WoC_loop의 goal과 같은 정의).
refMocoDir = fullfile(resultsDir, referenceName, 'result_1', 'moco_result');
cfg.guessSto = fullfile(refMocoDir, 'moco_WoC_Solution_iter01_kinematics_half.sto');
ref = moco_WoC_evalGaitMetrics( ...
    fullfile(resultsDir, referenceName, 'result_1', 'analy_result', '2D_gait_AFO_pc_1.osim'), ...
    cfg.guessSto, 2, cfg.propulsionSmoothing);
fprintf(['reference %s: E=%.6g, P_smooth=%.6g N*s, P_physical=%.6g N*s, ', ...
         'v_avg=%.4f m/s, phase %.4f s / %.4f m\n'], referenceName, ref.E_simpson, ...
         ref.P_smooth_simpson, ref.P_physical_trapz, ref.v_avg, ref.duration, ref.distance);

cfg.E0 = ref.E_simpson;
cfg.J_targets = ref.P_smooth_simpson * J_scale;
cfg.targetSource = sprintf(['verification only: E0 and P x [%s] from converged ', ...
    '%s (not experimental targets), bound set %s'], num2str(J_scale), referenceName, boundSet);

summary = WoC_moco_runMinSpeedCases(cfg);
