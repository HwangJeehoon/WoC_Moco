% fixed_time_spline.m
%
%   modeSpline 보조를 "고정 시간 구간(fixedInterval)"에 적용하는 배치 스크립트.
%
%   일반 modeSpline 과의 차이:
%     - 이전 GRF에서 stance time을 계산하지 않음
%     - 보조 구간이 매 iter 동일한 절대 시각 [t_start, t_end]로 고정됨
%     - queue.xlsx와 무관하게 독립 실행
%
%   사용법:
%     1) DEFAULTS 섹션에서 공통 기본값을 설정합니다.
%     2) JOBS 섹션에서 실행할 job 목록을 정의합니다.
%        - result_name 은 각 job 에서 반드시 지정
%        - 나머지 필드는 선택 사항 (미지정 시 defaults 값 사용)
%     3) 스크립트를 실행하면 jobs 를 순차적으로 처리합니다.
%        한 job 이 실패해도 다음 job 은 계속 실행됩니다.
%
%   저장 구조 (results/<result_name>/):
%     baseline/analy_result/      ← 초기 guess kinematics 해석 결과
%     result_i/analy_result/      ← iter i 결과 해석
%     result_i/moco_result/       ← iter i 궤적/GRF/metabolic
%     result_i/control_result/    ← iter i 보조 제어력
%
%   spline 파라미터 의미 (fixedInterval 내 정규화 [0,1] 도메인):
%     trigger : 보조 시작 위치 (0 = fixedInterval 시작점)
%     rise    : 상승 구간 길이
%     flat    : 최대값 유지 구간 길이
%     fall    : 하강 구간 길이
%     maxVal  : 최대 출력값 [0, 1]
%
%   예) fixedInterval=[0.1, 0.7], trigger=0, rise=0.2, flat=0.4, fall=0.2
%       → 0.10 s 에서 상승 시작, 0.22 s 에 최대, 0.46 s 에 하강 시작, 0.58 s 에 종료

close all;
import org.opensim.modeling.*

%% =====================================================================
%%  DEFAULTS — 모든 job 에 공통 적용되는 기본값
%%  job 구조체에서 동일 필드를 지정하면 해당 job 에서 덮어씁니다.
%% =====================================================================

defaults.model         = '2D_gait_AFO_pc_50BW.osim'; % models/ 폴더 기준 파일명
defaults.iter          = 50;                         % 반복 횟수

% modeSpline 파라미터 (fixedInterval 내 정규화 도메인 [0,1])
defaults.trigger       = 0.00;    % 보조 시작 위치
defaults.rise          = 0.60;   % 상승 구간 길이
defaults.flat          = 0.0;   % 최대값 유지 구간 길이
defaults.fall          = 0.20;   % 하강 구간 길이
defaults.maxVal        = 1.0;    % 최대 출력값 (0 ~ 1)

% 고정 보조 시간 구간 [t_start, t_end] (초)
defaults.fixedInterval = [0.0, 1.0];

% Moco 비용 가중치
defaults.mocoEffort    = 1;
defaults.mocoFinalTime = 0.03;

% Moco bounds
defaults.mocoTimeBound = [0.1 3.2];   % 반 걸음(modeSym) 기준 시간 [lb, ub]
defaults.mocoDistBound = [0.05 8];   % pelvis_tx 최종 위치 [lb, ub]

% Gait mode: 'modeSym' (반 걸음+대칭) | 'modeAsym' (한 걸음+주기성)
defaults.gaitMode      = 'modeSym';

%% =====================================================================
%%  JOBS — 실행할 job 목록
%%
%%  각 job 은 struct 로 정의합니다.
%%  result_name 은 필수. 나머지는 defaults 에서 덮어쓸 값만 기입하세요.
%%
%%  자주 쓰는 필드:
%%    result_name    : 결과 폴더명 (필수)
%%    model          : 모델 파일명
%%    iter           : 반복 횟수
%%    fixedInterval  : [t_start, t_end] (초)
%%    trigger/rise/flat/fall/maxVal : spline 형태
%%    mocoEffort, mocoFinalTime     : Moco 가중치
%%    mocoTimeBound, mocoDistBound  : Moco bounds ([lb, ub])
%%    gaitMode       : 'modeSym' | 'modeAsym'
%%
%%  ─── 예시 ─────────────────────────────────────────────────────────
%%  jobs{end+1} = struct('result_name','ctrl_v1_t0006', ...
%%                       'fixedInterval',[0.0, 0.6], 'maxVal',0.5);
%%
%%  jobs{end+1} = struct('result_name','ctrl_v2_t0107', ...
%%                       'model','2D_gait_AFO_pc_v2.osim', ...
%%                       'fixedInterval',[0.1, 0.7], 'maxVal',0.7, ...
%%                       'rise',0.2, 'fall',0.3);
%%
%%  jobs{end+1} = struct('result_name','ctrl_v1_asym', ...
%%                       'gaitMode','modeAsym', 'iter',5);
%% =====================================================================

jobs = {};

% 1.2, 1.4는 보조 아예 안들어감. 1도 간당간당함
% 셋업 : 
% defaults.trigger       = 0.00;    % 보조 시작 위치
% defaults.rise          = 0.30;   % 상승 구간 길이
% defaults.flat          = 0.0;   % 최대값 유지 구간 길이
% defaults.fall          = 0.10;   % 하강 구간 길이
% defaults.maxVal        = 1.0;    % 최대 출력값 (0 ~ 1)

% jobs{1} = struct('result_name', 'fixedTimeSpline_0.6', ...
%                      'fixedInterval', [0.0, 0.6]);
% 
% jobs{2} = struct('result_name', 'fixedTimeSpline_0.8', ...
%                      'fixedInterval', [0.2, 0.8]);
% 
% jobs{3} = struct('result_name', 'fixedTimeSpline_1', ...
%                      'fixedInterval', [0.4, 1.0]);
% 
% jobs{4} = struct('result_name', 'fixedTimeSpline_1.2', ...
%                      'fixedInterval', [0.6, 1.2]);
% 
% jobs{5} = struct('result_name', 'fixedTimeSpline_1.4', ...
%                      'fixedInterval', [0.8, 1.4]);


% defaults.trigger       = 0.00;    % 보조 시작 위치
% defaults.rise          = 0.60;   % 상승 구간 길이
% defaults.flat          = 0.0;   % 최대값 유지 구간 길이
% defaults.fall          = 0.20;   % 하강 구간 길이
% defaults.maxVal        = 1.0;    % 최대 출력값 (0 ~ 1)

% jobs{1} = struct('result_name', 'fixedTimeSpline2_0.2', ...
%                      'fixedInterval', [0.0, 0.2]);
% 
% jobs{2} = struct('result_name', 'fixedTimeSpline2_0.25', ...
%                      'fixedInterval', [0.05, 0.25]);
% 
% jobs{3} = struct('result_name', 'fixedTimeSpline2_0.3', ...
%                      'fixedInterval', [0.1, 0.3]);

jobs{1} = struct('result_name', 'fixedTimeSpline2_0.35', ...
                     'fixedInterval', [0.15, 0.35]);

jobs{2} = struct('result_name', 'fixedTimeSpline2_0.4', ...
                     'fixedInterval', [0.2, 0.4]);

jobs{3} = struct('result_name', 'fixedTimeSpline2_0.45', ...
                     'fixedInterval', [0.25, 0.45]);

jobs{4} = struct('result_name', 'fixedTimeSpline2_0.5', ...
                     'fixedInterval', [0.3, 0.5]);

jobs{5} = struct('result_name', 'fixedTimeSpline2_0.55', ...
                     'fixedInterval', [0.35, 0.55]);

jobs{6} = struct('result_name', 'fixedTimeSpline2_0.6', ...
                     'fixedInterval', [0.4, 0.6]);
%% =====================================================================
%%  END OF CONFIGURATION — 이하 수정 불필요
%% =====================================================================

%% ── 공통 경로 설정 ────────────────────────────────────────────────
scriptDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(scriptDir, '..', 'src')));

inputPath      = fullfile(scriptDir, '..', 'inputs');
modelBasePath  = fullfile(scriptDir, '..', 'models');
AnalySetupPath = fullfile(inputPath, 'analysis_setup.xml');

%% ── 배치 실행 ─────────────────────────────────────────────────────
nJobs     = numel(jobs);
statusLog = cell(nJobs, 2);   % {result_name, 'OK' / 'FAIL: ...'}

% 디버깅용 전역 타이머 리셋 (fts_runJob 내 toc_global() 과 연동)
clear fts_runJob   % persistent 변수 초기화

for jj = 1:nJobs

    %% defaults + job 병합
    cfg  = defaults;
    flds = fieldnames(jobs{jj});
    for k = 1:numel(flds)
        cfg.(flds{k}) = jobs{jj}.(flds{k});
    end

    %% result_name 필수 확인
    if ~isfield(cfg, 'result_name') || isempty(cfg.result_name)
        error('Job %d: result_name 이 지정되지 않았습니다.', jj);
    end

    fprintf('\n========================================\n');
    fprintf(' [Job %d/%d]  %s\n', jj, nJobs, cfg.result_name);
    fprintf('========================================\n');

    try
        fts_runJob(cfg, inputPath, modelBasePath, AnalySetupPath, scriptDir);
        statusLog{jj,1} = cfg.result_name;
        statusLog{jj,2} = 'OK';
        fprintf('[완료] Job %d: %s\n', jj, cfg.result_name);

    catch ME
        statusLog{jj,1} = cfg.result_name;
        statusLog{jj,2} = sprintf('FAIL: %s', ME.message);
        fprintf('[오류] Job %d (%s): %s\n', jj, cfg.result_name, ME.message);
        fprintf('  → 다음 job 으로 계속합니다.\n');
    end
end

%% ── 최종 요약 ─────────────────────────────────────────────────────
fprintf('\n======== 배치 완료 요약 ========\n');
nOK = 0;
for jj = 1:nJobs
    tag = statusLog{jj,2};
    if strcmp(tag,'OK'), nOK = nOK + 1; end
    fprintf('  [%d/%d] %-30s : %s\n', jj, nJobs, statusLog{jj,1}, tag);
end
fprintf('  완료: %d / %d\n', nOK, nJobs);
fprintf('================================\n');


%% =====================================================================
%%  fts_runJob — 단일 job 실행 (별도 함수 파일로 분리)
%% =====================================================================
% 아래 함수를 script/ 폴더에 fts_runJob.m 으로 저장하세요.
% (MATLAB 스크립트는 로컬 함수를 지원하지 않으므로 별도 파일 필요)
%
%   function fts_runJob(cfg, inputPath, modelBasePath, AnalySetupPath, scriptDir)
%   → 내용: script/fts_runJob.m 참조
