function WoC_moco_main(model, iter, a, b, cost, p, q, optMode, result_name, optResume)
% WoC_moco_main
%
%   호출 예시:
%     % 일반 실행
%     WoC_moco_main(model, iter, a, b, cost, p, q, 'modeWoC', 'my_result')
%
%     % Resume 실행 (optResume.resume_name 이 있으면 자동으로 resume mode)
%     optResume.resume_name = 'my_result\result_300';
%     WoC_moco_main(model, iter, a, b, cost, p, q, 'modeWoC', 'my_result_continued', optResume)
%
%   optMode (string):
%     'modeWoC'    : WoC QP 기반 최적 보조 토크 (기본 동작)
%     'modeOff'    : 보조력 없음 (AFO = 0, QP 생략)
%     'modeCustom' : 사용자 정의 보조력 (TODO)
%
%   optMode (struct) — modeSpline 전용:
%     .type    = 'modeSpline'
%     .trigger : control 시작 시각 (정규화 도메인 [0, 0.6] 기준)
%     .rise    : 상승 구간 길이
%     .flat    : 최대값 유지 구간 길이
%     .fall    : 하강 구간 길이
%     .maxVal  : 최대 출력값 (0 ~ 1)
%     조건: trigger + rise + flat + fall <= 0.6

clc; close all;

%% --------------------------------------------------
%  optMode 파싱 (string / struct 양쪽 허용)
% ---------------------------------------------------
if nargin < 8 || isempty(optMode)
    optMode = 'modeWoC';
end

if ischar(optMode) || isstring(optMode)
    modeType   = char(optMode);
    modeParams = struct();
elseif isstruct(optMode)
    if ~isfield(optMode, 'type')
        error('optMode가 struct인 경우 type 필드가 필요합니다.');
    end
    modeType   = optMode.type;
    modeParams = optMode;
else
    error('optMode는 string 또는 struct 이어야 합니다.');
end

validModes = {'modeWoC', 'modeOff', 'modeSpline', 'modeCustom'};
if ~ismember(modeType, validModes)
    error('Unknown optMode: ''%s''. Valid options: modeWoC, modeOff, modeSpline, modeCustom.', modeType);
end

% modeSpline 필수 필드 검증 (루프 진입 전에 미리 확인)
if strcmpi(modeType, 'modeSpline')
    splineFields = {'trigger', 'rise', 'flat', 'fall', 'maxVal'};
    for sf = splineFields
        if ~isfield(modeParams, sf{1})
            error('modeSpline: optMode.%s 가 필요합니다.', sf{1});
        end
    end
end

%% --------------------------------------------------
%  result_name / optResume 파싱
% ---------------------------------------------------
if nargin < 9 || isempty(result_name)
    error('result_name 을 지정해야 합니다.');
end

if nargin < 10 || isempty(optResume)
    optResume = struct();
end

% resume_name 이 있으면 resume mode
resume_mode = isfield(optResume, 'resume_name') && ~isempty(optResume.resume_name);
if resume_mode
    resume_name = optResume.resume_name;
else
    resume_name = '';
end

%% --------------------------------------------------
%  0. baseFolder = 이 파일이 있는 폴더
% ---------------------------------------------------
if isempty(mfilename)  % 스크립트처럼 실행하는 경우
    thisFile = matlab.desktop.editor.getActiveFilename;
else
    thisFile = mfilename("fullpath");
end
baseFolder = fileparts(thisFile);

%% --------------------------------------------------
%  1. 파라미터 설정 + Output 폴더명 지정
% ---------------------------------------------------

% 사용할 model명 지정
ModelNameOsim = model;
[~, modelName, ~] = fileparts(model);

% QP Parameter (modeWoC에서만 사용)
iterNum          = iter;
coeffi_cost      = a;
coeffi_smoothing = b;

% Main Cost (modeWoC에서만 사용)
eta_tau   = false;
eta_tau_w = false;
eta_tau_v = false;
switch lower(cost)
    case 'et'
        eta_tau = true;
    case 'etw'
        eta_tau_w = true;
    case 'etv'
        eta_tau_v = true;
    otherwise
        if strcmpi(modeType, 'modeWoC')
            error("Unknown cost type: %s. Use 'et', 'etw', or 'etv'.", cost);
        end
end

% Moco Parameter
MocoOpts.weight_effort    = p;
MocoOpts.weight_finalTime = q;

% Output 폴더
OutputFolder = fullfile(baseFolder,'..','results', result_name);
if ~exist(OutputFolder, 'dir')
    mkdir(OutputFolder);
end

%% --------------------------------------------------
%  2. 초기 데이터 경로 설정
% ---------------------------------------------------
inputPath      = fullfile(baseFolder, '..','inputs');
guessInitSto   = fullfile(inputPath, 'guess_init_v5.sto');
grfInitSto     = fullfile(inputPath, 'GRF_init_v5.sto');
AnalySetupPath = fullfile(inputPath, 'analysis_setup.xml');
modelPath      = fullfile(baseFolder, '..','models');

%% --------------------------------------------------
%  2.5 실행 범위 결정 (일반 / Resume)
% ---------------------------------------------------
if ~resume_mode
    baseIter  = 0;
    startIter = 1;
    endIter   = iterNum;
    fprintf('=== [%s] Normal mode: result_1 -> result_%d ===\n', modeType, endIter);
else
    % resume_name 가 baseFolder 기준 상대경로 (예: 'my_result\result_300')
    resumeAbsDir = fullfile(baseFolder,'..','results', resume_name);

    % result_XXX에서 XXX 파싱
    [~, resumeFolderName] = fileparts(resumeAbsDir);
    tok = regexp(resumeFolderName, '^result_(\d+)$', 'tokens', 'once');
    if isempty(tok)
        error('optResume.resume_name 마지막 폴더명이 result_### 형식이어야 합니다: %s', resumeFolderName);
    end
    baseIter  = str2double(tok{1});
    startIter = baseIter + 1;
    endIter   = baseIter + iterNum;

    fprintf('=== [%s] Resume mode: base = result_%d, run result_%d -> result_%d ===\n', ...
        modeType, baseIter, startIter, endIter);
end

%% --------------------------------------------------
%  3. 메인 루프
% ---------------------------------------------------
for i = startIter:endIter
    fprintf('===== Iteration %d / %d =====\n', i, iterNum+baseIter);

    %------------------------------------------------
    % 3-1. 이번 iteration에서 사용할 경로 결정
    %------------------------------------------------
    if ~resume_mode
        if i == 1
            kinStoForAnaly    = guessInitSto;
            grfStoPath        = grfInitSto;
            modelPathForAnaly = fullfile(modelPath, ModelNameOsim);
        else
            prevResultDir     = fullfile(OutputFolder, sprintf('result_%d', i-1), 'moco_result');
            kinStoForAnaly    = fullfile(prevResultDir, sprintf('moco_WoC_Solution_iter%02d_kinematics.sto', i-1));
            grfStoPath        = fullfile(prevResultDir, sprintf('moco_WoC_Solution_iter%02d_GRF.sto', i-1));
            prevAnalyDir      = fullfile(OutputFolder, sprintf('result_%d', i-1), 'analy_result');
            modelPathForAnaly = fullfile(prevAnalyDir, sprintf('%s_%d.osim', modelName, i-1));
        end
    else
        if i == startIter
            prevResultDir     = fullfile(resumeAbsDir, 'moco_result');
            kinStoForAnaly    = fullfile(prevResultDir, sprintf('moco_WoC_Solution_iter%02d_kinematics.sto', baseIter));
            grfStoPath        = fullfile(prevResultDir, sprintf('moco_WoC_Solution_iter%02d_GRF.sto', baseIter));
            prevAnalyDir      = fullfile(resumeAbsDir, 'analy_result');
            modelPathForAnaly = fullfile(prevAnalyDir, sprintf('%s_%d.osim', modelName, baseIter));
        else
            prevResultDir     = fullfile(OutputFolder, sprintf('result_%d', i-1), 'moco_result');
            kinStoForAnaly    = fullfile(prevResultDir, sprintf('moco_WoC_Solution_iter%02d_kinematics.sto', i-1));
            grfStoPath        = fullfile(prevResultDir, sprintf('moco_WoC_Solution_iter%02d_GRF.sto', i-1));
            prevAnalyDir      = fullfile(OutputFolder, sprintf('result_%d', i-1), 'analy_result');
            modelPathForAnaly = fullfile(prevAnalyDir, sprintf('%s_%d.osim', modelName, i-1));
        end
    end

    % 이번 iteration의 result_i 폴더들
    iterRootDir      = fullfile(OutputFolder, sprintf('result_%d', i));
    AnalyResultDir   = fullfile(iterRootDir, 'analy_result');
    mocoResultDir    = fullfile(iterRootDir, 'moco_result');
    controlResultDir = fullfile(iterRootDir, 'control_result');

    if ~exist(iterRootDir, 'dir');       mkdir(iterRootDir);       end
    if ~exist(AnalyResultDir, 'dir');    mkdir(AnalyResultDir);    end
    if ~exist(mocoResultDir, 'dir');     mkdir(mocoResultDir);     end
    if ~exist(controlResultDir, 'dir');  mkdir(controlResultDir);  end

    %------------------------------------------------
    % 3-2 ~ 3-6. 모드별 control reference 생성
    %------------------------------------------------
    switch modeType

        case 'modeWoC'
            %--------------------------------------------
            % WoC mode: QP 기반 최적 보조 토크 계산
            %--------------------------------------------

            % 3-2. Analy 수행: CoM, CoP_R 좌표 .sto 추출
            WoC_moco_analysis(AnalySetupPath, ...
                'modelPath',         modelPathForAnaly, ...
                'kinematicsStoPath', kinStoForAnaly, ...
                'resultsDir',        AnalyResultDir);

            CoMPath   = fullfile(AnalyResultDir, '2D_gait_AFO_pc_PointKinematics_CoM_pos.sto');
            CoP_RPath = fullfile(AnalyResultDir, '2D_gait_AFO_pc_PointKinematics_CoP_R_pos.sto');

            % 3-3. GRF에서 오른발 stance time 추출
            [stanceTimeR_101, fullTime] = WoC_moco_detectStance(grfStoPath);

            % 3-4. QP input (eta, w, t) 계산
            optsCalQP = struct();
            optsCalQP.CoMFields   = {'state_0','state_1','state_2'};
            optsCalQP.CoP_RFields = {'state_0','state_1','state_2'};
            optsCalQP.apAxis   = 'x';
            optsCalQP.vertAxis = 'y';
            optsCalQP.wPath  = fullfile(AnalyResultDir, '2D_gait_AFO_pc_Kinematics_u.sto');
            optsCalQP.wField = 'ankle_angle_r';
            optsCalQP.vPath  = fullfile(AnalyResultDir, '2D_gait_AFO_pc_Kinematics_u.sto');
            optsCalQP.vField = 'pelvis_tx';

            [etaR_101, wR_101, v_101, dt] = WoC_moco_cal_QP_input( ...
                CoMPath, CoP_RPath, stanceTimeR_101, optsCalQP);
            fprintf('dt = %.6f\n', dt);

            % 3-5. QP 풀어서 tau_R(stance 구간 control) 계산
            qpOpts           = struct();
            qpOpts.alpha     = coeffi_cost;
            qpOpts.beta      = coeffi_smoothing;
            qpOpts.tauDotMax = 10;

            one_101 = ones(101,1);
            if eta_tau
                QP_in     = one_101;  qpColName = 'one_101';
            elseif eta_tau_w
                QP_in     = wR_101;   qpColName = 'wR_101';
            elseif eta_tau_v
                QP_in     = v_101;    qpColName = 'v_101';
            else
                QP_in     = one_101;  qpColName = 'one_101';
            end
            if (eta_tau + eta_tau_w + eta_tau_v) > 1
                error('Main Cost flag는 eta_tau / eta_tau_w / eta_tau_v 중 하나만 true여야 합니다.');
            end
            tau_R = WoC_moco_solveQP(etaR_101, QP_in, dt, qpOpts);

            % 3-6. 전체 시간축 control.sto + data.csv 출력
            writeOpts.dataColName = qpColName;
            WoC_moco_writeControl(controlResultDir, ...
                fullTime, stanceTimeR_101, tau_R, etaR_101, QP_in, writeOpts);

            controlRefStoPath = fullfile(controlResultDir, 'control.sto');

        case 'modeOff'
            %--------------------------------------------
            % Off mode: 보조력 없음 (AFO = 0 전구간)
            % Analysis / QP 생략. 영벡터 control.sto 생성.
            %--------------------------------------------
            controlRefStoPath = fullfile(controlResultDir, 'control.sto');
            fid = fopen(controlRefStoPath, 'w');
            if fid == -1
                error('control.sto 파일을 열 수 없습니다: %s', controlRefStoPath);
            end
            fprintf(fid, 'controls\n');
            fprintf(fid, 'nRows=2\n');
            fprintf(fid, 'nColumns=3\n');
            fprintf(fid, 'endheader\n');
            fprintf(fid, 'time\tAFO_r\tAFO_l\n');
            fprintf(fid, '0\t0\t0\n');
            fprintf(fid, '1\t0\t0\n');
            fclose(fid);

        case 'modeSpline'
            %--------------------------------------------
            % Spline mode: trapezoid cubic Hermite 보조력 프로파일
            %
            %   stanceTime → [0, 0.6] 정규화 도메인에서 spline 생성 후
            %   실제 stanceTime 축으로 매핑하여 control.sto 출력.
            %   Analysis / QP 생략. GRF stance 검출만 수행.
            %--------------------------------------------

            % Stance time 추출 (control 주입 시간 범위 + fullTime 필요)
            [stanceTimeR_101, fullTime] = WoC_moco_detectStance(grfStoPath);

            % Spline control 생성 (정규화 도메인 → 101포인트 tau 벡터)
            tau_R = WoC_moco_buildSplineControl(modeParams, numel(stanceTimeR_101));

            % Control.sto + data.csv 출력
            % (eta, w 는 modeSpline에서 의미 없으므로 placeholder 사용)
            dummyEta             = zeros(numel(stanceTimeR_101), 1);
            dummyW               = ones(numel(stanceTimeR_101), 1);
            writeOpts_sp         = struct();
            writeOpts_sp.dataColName = 'spline';
            WoC_moco_writeControl(controlResultDir, ...
                fullTime, stanceTimeR_101, tau_R, dummyEta, dummyW, writeOpts_sp);

            controlRefStoPath = fullfile(controlResultDir, 'control.sto');

        case 'modeCustom'
            %--------------------------------------------
            % Custom mode: TODO
            % 사용자 정의 보조력 프로파일을 적용하는 mode.
            % controlRefStoPath 를 설정하는 로직을 여기에 구현.
            %--------------------------------------------
            error('[modeCustom] is not yet implemented.');

    end  % switch modeType

    %------------------------------------------------
    % 3-7. Moco loop에 사용할 guess 경로 결정
    %------------------------------------------------
    if ~resume_mode
        if i == 1
            guessStoPath = guessInitSto;
        else
            prevHalf     = fullfile(OutputFolder, sprintf('result_%d', i-1), 'moco_result', ...
                sprintf('moco_WoC_Solution_iter%02d_kinematics_half.sto', i-1));
            guessStoPath = prevHalf;
        end
    else
        if i == startIter
            prevHalf     = fullfile(resumeAbsDir, 'moco_result', ...
                sprintf('moco_WoC_Solution_iter%02d_kinematics_half.sto', baseIter));
            guessStoPath = prevHalf;
        else
            prevHalf     = fullfile(OutputFolder, sprintf('result_%d', i-1), 'moco_result', ...
                sprintf('moco_WoC_Solution_iter%02d_kinematics_half.sto', i-1));
            guessStoPath = prevHalf;
        end
    end

    %------------------------------------------------
    % 3-8. Moco loop: control reference → 새로운 보행 궤적
    %------------------------------------------------
    baseOsimPath = fullfile(modelPath, ModelNameOsim);
    sol = moco_WoC_loop(controlRefStoPath, guessStoPath, i, AnalyResultDir, baseOsimPath, MocoOpts);

    %------------------------------------------------
    % 3-9. Moco 결과 저장 (kinematics, GRF 등)
    %------------------------------------------------
    resOpts           = struct();
    resOpts.modelPath = fullfile(AnalyResultDir, sprintf('%s_%d.osim', modelName, i));
    resOpts.prefix    = sprintf('moco_WoC_Solution_iter%02d', i);
    moco_WoC_getResult(sol, mocoResultDir, resOpts);

    fprintf('Iteration %d done.\n', i);
end

fprintf('All iterations finished.\n');
end
