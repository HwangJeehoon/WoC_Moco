% run_minSpeedFixedEP.m
%
% 같은 E0에 대해 propulsion target J_k (k = 1, 2, 3)마다 독립적으로 푼다.
%
%   minimize    v_avg = (pelvis_tx(tf) - pelvis_tx(t0)) / (tf - t0)
%   subject to  E = E0, P_gate = J_k, 기존 동역학/주기성/대칭성/상태·제어 bounds
%
%   E      : weighted 모드 effort goal의 weight 곱하기 전 값
%            (1/d_COM) * int sum_c u_c^2 dt, c = 근육 excitation 18개 + lumbarAct
%            (+ AFO_r/AFO_l: PrescribedController 값, modeOff에서는 0)
%   P_gate : 한 보행주기 양발 gated propulsive impulse [N*s] (meanPropPerVel의 Iplus)
%            int [F_r g(F_r) + F_l g(F_l)] dt, F_foot = heel + front x-force,
%            g(F) = 1/(1 + exp(-(F - F0)/sigma))
%            (modeSym은 반주기 적분의 2배; max(F,0) impulse, Tplus, P_gate/Tplus는
%             비교용으로 함께 보고)
%
% 결과: <resultsDir>/<runTag>_<sym|asym>_J<k>/ (case별),
%       <resultsDir>/<runTag>_<mode>_summary/<시각>/ (summary.csv, definitions.txt, 그림).
%       기존 결과 폴더는 덮어쓰지 않는다.
% 플러그인이 없으면 먼저 빌드:
%   run(fullfile('..', 'plugins', 'osimFixedEffortPropulsion', 'build_osimFixedEffortPropulsion.m'))
% 검증: test_minSpeedFixedEP_endpointStage.m, test_minSpeedFixedEP_metrics.m,
%       test_minSpeedFixedEP.m (검증용 target은 실험 target이 아니다)
clc; clear; close all;
thisScriptDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(thisScriptDir, '..', 'src')));

%% ===== 사용자 입력 =====
% 실험 target: 아직 제공되지 않았다. 비워 두면 실행하지 않고 error로 멈춘다.
E0 = [];                        % effort target (weight 곱하기 전 effort metric, 무차원)
J_targets = [];                 % [J1, J2, J3] [N*s], 한 보행주기 양발 P_gate

cfg.gateSigma = 1.0;            % propulsion gate 폭 sigma [N] (meanPropPerVel 기본값)
cfg.gateThreshold = 5.0;        % propulsion gate 중심 F0 [N] (meanPropPerVel 기본값)
cfg.model = '2D_gait_AFO_pc.osim';
cfg.guessSto = '';              % '' -> inputs/guess_init_half.sto (modeAsym: _full)
cfg.optMode = 'modeOff';        % 보조력 모드 (modeOff | modeWoC | modeSpline 등 string)
cfg.gaitMode = 'modeSym';       % 'modeSym' | 'modeAsym'
cfg.timeBound = [0.4, 0.8];     % 반 걸음 최종 시간 bound [s] (modeAsym은 x2)
cfg.distBound = [0.4, 1.0];     % 반 걸음 pelvis_tx 최종 위치 bound [m] (modeAsym은 x2)
cfg.meshIntervals = 300;
cfg.convergenceTolerance = 1e-3;
cfg.constraintTolerance = 1e-3;
cfg.maxIterations = 1000;
cfg.effortTol = 1e-3;           % |E - E0| 검증 허용오차
cfg.propulsionTol = 1e-2;       % |P_gate - J| 검증 허용오차 [N*s]
cfg.resultsDir = fullfile(thisScriptDir, '..', 'results');   % 실제 저장 루트
cfg.runTag = ['minspeed_gate_' char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'))];
cfg.continuation = false;       % true: case k가 직전 수렴 case의 해에서 시작 (CSV Guess 열에 기록)
cfg.runOptimizations = true;    % false: 같은 runTag의 기존 결과만 요약

cfg.E0 = E0;
cfg.J_targets = J_targets;
cfg.targetSource = 'experiment (user input)';

summary = WoC_moco_runMinSpeedCases(cfg);
