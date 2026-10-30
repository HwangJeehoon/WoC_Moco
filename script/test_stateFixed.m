% test_stateFixed.m
%
% 같은 E0에 대해 propulsion target J_k마다 독립적으로 푼다.
%
%   minimize    w_v * v_avg,  v_avg = (pelvis_tx(tf) - pelvis_tx(t0)) / (tf - t0)
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
%   w_v    : 평균속도 cost weight (목적함수의 유일한 항)
%
% 각 J는 같은 initial guess에서 시작하며, 한 J가 수렴하지 않아도 기록만 하고
% 다음 J를 계속 푼다(target, bound, equality는 바꾸지 않는다).
%
% 결과: <resultsDir>/<runTag>_<sym|asym>_J<k>/ (case별),
%       <resultsDir>/<runTag>_<mode>_summary/<시각>/ (summary.csv, definitions.txt, 그림).
%       기존 결과 폴더는 덮어쓰지 않는다.
% 플러그인이 없으면 먼저 빌드:
%   run(fullfile('..', 'plugins', 'osimFixedEffortPropulsion', 'build_osimFixedEffortPropulsion.m'))
clc; clear; close all;
thisScriptDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(thisScriptDir, '..', 'src')));

%% ===== 사용자 입력 =====
E0 = 0.1;                       % effort target (weight 곱하기 전 effort metric, 무차원)
J_targets = [30, 40, 50];       % [J1, J2, J3] [N*s], 한 보행주기 양발 P_gate

cfg.speedWeight = 1;            % 평균속도 cost weight w_v
cfg.gateSigma = 1.0;            % propulsion gate 폭 sigma [N] (meanPropPerVel 기본값)
cfg.gateThreshold = 5.0;        % propulsion gate 중심 F0 [N] (meanPropPerVel 기본값)
cfg.model = '2D_gait_AFO_pc.osim';
% Initial guess: propulsion_compare_no_goal (effort 1 + final time 0.03, modeOff,
% modeSym, mesh 300, tol 1e-3; E = 0.0986, P_gate = 31.7 N*s, v_avg = 1.33 m/s)
cfg.guessSto = fullfile(thisScriptDir, '..', 'results', 'propulsion_compare_no_goal', ...
    'result_1', 'moco_result', 'moco_WoC_Solution_iter01_kinematics_half.sto');
cfg.optMode = 'modeOff';        % 보조력 모드 (modeOff | modeWoC | modeSpline 등 string)
cfg.gaitMode = 'modeSym';       % 'modeSym' | 'modeAsym'
cfg.timeBound = [0.4, 0.8];     % 반 걸음 최종 시간 bound [s] (modeAsym은 x2)
cfg.distBound = [0.4, 1.0];     % 반 걸음 pelvis_tx 최종 위치 bound [m] (modeAsym은 x2)
cfg.meshIntervals = 300;
cfg.convergenceTolerance = 5e-5;
cfg.constraintTolerance = 5e-5;
cfg.maxIterations = 5000;
cfg.effortTol = 1e-3;           % |E - E0| 검증 허용오차
cfg.propulsionTol = 1e-2;       % |P_gate - J| 검증 허용오차 [N*s]
cfg.resultsDir = fullfile(thisScriptDir, '..', 'results');   % 실제 저장 루트
cfg.runTag = ['stateFixed_' char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'))];
cfg.continuation = false;       % true: case k가 직전 수렴 case의 해에서 시작 (CSV Guess 열에 기록)
cfg.runOptimizations = true;    % false: 같은 runTag의 기존 결과만 요약

cfg.E0 = E0;
cfg.J_targets = J_targets;
cfg.targetSource = 'experiment (user input)';

summary = WoC_moco_runMinSpeedCases(cfg);
