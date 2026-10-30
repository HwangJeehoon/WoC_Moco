% run_minSpeedFixedEP.m
%
% 같은 E0에 대해 propulsion target J_k (k = 1, 2, 3)마다 독립적으로 푼다.
%
%   minimize    v_avg = (pelvis_tx(tf) - pelvis_tx(t0)) / (tf - t0)
%   subject to  E = E0, P = J_k, 기존 동역학/주기성/대칭성/상태·제어 bounds
%
%   E : weighted 모드 effort goal의 weight 곱하기 전 값
%       (1/d_COM) * int sum_c u_c^2 dt, c = 근육 excitation 18개 + lumbarAct
%       (+ AFO_r/AFO_l: PrescribedController 값, modeOff에서는 0)
%   P : 한 보행주기 양발 smooth positive AP-GRF impulse [N*s]
%       sum_foot int log(1 + exp(s*F_foot))/s dt, F_foot = heel + front x-force
%       (modeSym은 반주기 적분의 2배; 후처리 max(F,0) impulse도 함께 보고)
%
% 결과: results/<runTag>_<sym|asym>_J<k>/ (case별), results/<runTag>_<mode>_summary/
%       (summary.csv, definitions.txt, 그림). 기존 결과 폴더는 덮어쓰지 않는다.
% 플러그인이 없으면 먼저 빌드:
%   run(fullfile('..', 'plugins', 'osimFixedEffortPropulsion', 'build_osimFixedEffortPropulsion.m'))
% 검증용 target으로 돌려 보려면 test_minSpeedFixedEP.m 을 쓴다.
clc; clear; close all;
thisScriptDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(thisScriptDir, '..', 'src')));

%% ===== 사용자 입력 =====
% 실험 target: 아직 제공되지 않았다. 비워 두면 실행하지 않고 error로 멈춘다.
E0 = [];                        % effort target (weight 곱하기 전 effort metric, 무차원)
J_targets = [];                 % [J1, J2, J3] [N*s], 한 보행주기 양발 smooth impulse

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
cfg.propulsionSmoothing = 0.2;  % 기존 propulsion goal 기본값과 같은 softplus 계수
cfg.effortTol = 1e-3;           % |E - E0| 검증 허용오차
cfg.propulsionTol = 1e-2;       % |P - J| 검증 허용오차 [N*s]
cfg.resultsDir = fullfile(thisScriptDir, '..', 'results');
cfg.runTag = ['minspeed_' char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'))];
cfg.continuation = false;       % true: case k가 직전 수렴 case의 해에서 시작 (CSV Guess 열에 기록)
cfg.runOptimizations = true;    % false: 같은 runTag의 기존 결과만 요약

cfg.E0 = E0;
cfg.J_targets = J_targets;
cfg.targetSource = 'experiment (user input)';

summary = WoC_moco_runMinSpeedCases(cfg);
