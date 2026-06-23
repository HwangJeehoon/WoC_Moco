function WoC_moco_main(model, iter, optMode, result_name, opts, optResume)
% WoC_moco_main
%
%   호출 예시:
%     % 일반 실행
%     WoC_moco_main(model, iter, 'modeWoC', 'my_result')
%     WoC_moco_main(model, iter, 'modeWoC', 'my_result', opts)
%
%     % Resume 실행 (optResume.resume_name 이 있으면 자동으로 resume mode)
%     optResume.resume_name = 'my_result\result_300';
%     WoC_moco_main(model, iter, 'modeWoC', 'my_result_continued', opts, optResume)
%
%   optMode (string):
%     'modeWoC'    : WoC QP 기반 최적 보조 토크 (기본 동작)
%     'modeOff'    : 보조력 없음 (AFO = 0, QP 생략)
%
%   optMode (struct) — modeSpline 전용:
%     .type    = 'modeSpline'
%     .trigger : control 시작 시각 (정규화 도메인 [0, 1) 기준)
%     .rise    : 상승 구간 길이
%     .flat    : 최대값 유지 구간 길이
%     .fall    : 하강 구간 길이 (trigger+rise+flat+fall > 1 이면 wrap-around)
%     .maxVal  : 최대 출력값 (0 ~ 1)
%     조건: trigger < 1
%
%   opts (선택 struct):
%     .QP_effort    : QP 비용 계수 (default = 0.01)
%     .QP_smooth    : QP 스무딩 계수 (default = 0)
%     .cost         : Main cost 종류 'et'|'etw'|'etv' (default = 'et')
%     .mocoEffort   : Moco effort goal weight (default = 1)
%     .mocoFinalTime: Moco final time goal weight (default = 0.03)
%     .gaitMode      : 'modeSym' | 'modeAsym' (default = 'modeSym')
%     .mocoTimeBound : Moco 시간 상한 [lb, ub] 또는 scalar (default = [0.4, 0.8])
%     .mocoDistBound : pelvis_tx 최종 위치 bound [lb, ub] 또는 scalar (default = [0.4, 1.0])
%     .guessInitSto  : Initial guess로 사용할 sto 위치
%
%   저장 구조:
%     results/<result_name>/baseline/analy_result/   ← 초기 guess kinematics 해석 결과
%     results/<result_name>/result_i/analy_result/   ← iter i moco 결과 해석
%     results/<result_name>/result_i/moco_result/    ← iter i 궤적/GRF/metabolic
%     results/<result_name>/result_i/control_result/ ← iter i 보조 제어력

close all;
import org.opensim.modeling.*

%% --------------------------------------------------
%  optMode 파싱 (string / struct 양쪽 허용)
% ---------------------------------------------------
if nargin < 3 || isempty(optMode)
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

validModes = {'modeWoC', 'modeOff', 'modeSpline', 'modeTorqAmp'};
if ~ismember(modeType, validModes)
    error('Unknown optMode: ''%s''. Valid options: modeWoC, modeOff, modeSpline, modeTorqAmp.', modeType);
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

% modeTorqAmp 필수 필드 검증
if strcmpi(modeType, 'modeTorqAmp')
    if ~isfield(modeParams, 'maxVal')
        error('modeTorqAmp: optMode.maxVal 가 필요합니다.');
    end
    if modeParams.maxVal < 0 || modeParams.maxVal > 1
        error('modeTorqAmp: maxVal 은 [0, 1] 범위여야 합니다 (%.4f).', modeParams.maxVal);
    end
    if isfield(modeParams, 'cutoffHz') && modeParams.cutoffHz <= 0
        error('modeTorqAmp: cutoffHz 는 양수여야 합니다 (%.4f). 필터 비활성화는 cutoffHz=0 대신 필드를 생략하세요.', modeParams.cutoffHz);
    end
end

%% --------------------------------------------------
%  result_name / opts / optResume 파싱
% ---------------------------------------------------
if nargin < 4 || isempty(result_name)
    error('result_name 을 지정해야 합니다.');
end

if nargin < 5 || isempty(opts)
    opts = struct();
end

if nargin < 6 || isempty(optResume)
    optResume = struct();
end

% ID / Date (run_queue 에서 전달되는 메타 정보, 선택적)
runID   = getOpt(opts, 'ID',   '');
runDate = getOpt(opts, 'Date', '');

% opts 기본값
if ~isfield(opts, 'QP_effort') || isempty(opts.QP_effort)
    QP_effort = 0.01;
    warning('WoC_moco_main: opts.QP_effort 가 지정되지 않아 default(0.01) 를 사용합니다.');
else
    QP_effort = opts.QP_effort;
end

if ~isfield(opts, 'QP_smooth') || isempty(opts.QP_smooth)
    QP_smooth = 0;
    warning('WoC_moco_main: opts.QP_smooth 가 지정되지 않아 default(0) 를 사용합니다.');
else
    QP_smooth = opts.QP_smooth;
end

if ~isfield(opts, 'cost') || isempty(opts.cost)
    cost = 'et';
    warning('WoC_moco_main: opts.cost 가 지정되지 않아 default(et) 를 사용합니다.');
else
    cost = opts.cost;
end

if ~isfield(opts, 'mocoEffort') || isempty(opts.mocoEffort)
    mocoEffort = 1;
    warning('WoC_moco_main: opts.mocoEffort 가 지정되지 않아 default(1) 를 사용합니다.');
else
    mocoEffort = opts.mocoEffort;
end

if ~isfield(opts, 'mocoFinalTime') || isempty(opts.mocoFinalTime)
    mocoFinalTime = 0.03;
    warning('WoC_moco_main: opts.mocoFinalTime 가 지정되지 않아 default(0.03) 를 사용합니다.');
else
    mocoFinalTime = opts.mocoFinalTime;
end

if ~isfield(opts, 'gaitMode') || isempty(opts.gaitMode)
    gaitMode = 'modeSym';
    warning('WoC_moco_main: opts.gaitMode 가 지정되지 않아 default(''modeSym'') 를 사용합니다.');
else
    gaitMode = opts.gaitMode;
end

if ~ismember(gaitMode, {'modeSym', 'modeAsym'})
    error('opts.gaitMode 는 ''modeSym'' 또는 ''modeAsym'' 이어야 합니다.');
end

if ~isfield(opts, 'mocoTimeBound') || isempty(opts.mocoTimeBound)
    mocoTimeBound = [0.4, 0.8];
    warning('WoC_moco_main: opts.mocoTimeBound 가 지정되지 않아 default([0.4, 0.8]) 를 사용합니다.');
else
    mocoTimeBound = opts.mocoTimeBound;
end

if ~isfield(opts, 'mocoDistBound') || isempty(opts.mocoDistBound)
    mocoDistBound = [0.4 1.0];
    warning('WoC_moco_main: opts.mocoDistBound 가 지정되지 않아 default([0.4 1.0]) 를 사용합니다.');
else
    mocoDistBound = opts.mocoDistBound;
end

%% --------------------------------------------------
%  파라미터 출력
% ---------------------------------------------------
fprintf('=== WoC_moco_main Parameters ===\n');
if ~isempty(runID),   fprintf('  ID         : %s\n', runID);   end
if ~isempty(runDate), fprintf('  Date       : %s\n', runDate); end
fprintf('  model      : %s\n', model);
fprintf('  iter       : %d\n', iter);
fprintf('  optMode    : %s\n', modeType);
fprintf('  result_name: %s\n', result_name);
fprintf('  --- opts ---\n');
fprintf('  QP_effort    : %.4g\n', QP_effort);
fprintf('  QP_smooth    : %.4g\n', QP_smooth);
fprintf('  cost         : %s\n',   cost);
fprintf('  mocoEffort   : %.4g\n', mocoEffort);
fprintf('  mocoFinalTime: %.4g\n', mocoFinalTime);
fprintf('  gaitMode     : %s\n',   gaitMode);
fprintf('  mocoTimeBound: %s\n',   mat2str(mocoTimeBound));
fprintf('  mocoDistBound: %s\n',   mat2str(mocoDistBound));
fprintf('================================\n');

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
coeffi_cost      = QP_effort;
coeffi_smoothing = QP_smooth;

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
MocoOpts.mocoEffort       = mocoEffort;
MocoOpts.mocoFinalTime    = mocoFinalTime;
MocoOpts.gaitMode         = gaitMode;
MocoOpts.mocoTimeBound    = mocoTimeBound;
MocoOpts.mocoDistBound    = mocoDistBound;

% Output 폴더
OutputFolder = fullfile(baseFolder,'..','results', result_name);
if ~exist(OutputFolder, 'dir')
    mkdir(OutputFolder);
end

%% --------------------------------------------------
%  2. 초기 데이터 경로 설정
% ---------------------------------------------------
inputPath      = fullfile(baseFolder, '..','inputs');

% 모델명에서 height suffix 탐지 (예: _150cm, _160cm, ...)
heightTok = regexp(modelName, '_(\d+cm)', 'tokens', 'once');
if ~isempty(heightTok)
    heightSuffix = ['_' heightTok{1}];
else
    heightSuffix = '';
end

if strcmpi(gaitMode, 'modeAsym')
    guessInitSto = fullfile(inputPath, ['guess_init_full' heightSuffix '.sto']);
    grfInitSto   = fullfile(inputPath, 'GRF_init_full.sto');
else  % modeSym
    guessInitSto = fullfile(inputPath, ['guess_init_half' heightSuffix '.sto']);
    grfInitSto   = fullfile(inputPath, 'GRF_init_full.sto'); % GRF는 modeSym에서도 full 사용 -> 보조력 계산 시 full GRF traj가 필요
end

[~, guessInitName, guessInitExt] = fileparts(guessInitSto);
fprintf('  initial guess : %s%s\n', guessInitName, guessInitExt);
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
%  Pre-loop: Baseline analysis (일반 모드에서만 실행)
%
%  opts.guessInitSto 가 있으면 (preceding modeOff trial 존재):
%    - preceding modeOff 의 GRF → baseline/moco_result/baseline_GRF.sto 복사
%    - preceding modeOff 의 id_withAssist.sto → baseline/analy_result/ 복사
%    - baseline analysis 입력 kinematics = opts.guessInitSto
%  없으면:
%    - baseline analysis 입력 kinematics = default guess_init
% ---------------------------------------------------
if ~resume_mode
    baselineAnalyDir = fullfile(OutputFolder, 'baseline', 'analy_result');
    baselineMocoDir  = fullfile(OutputFolder, 'baseline', 'moco_result');
    if ~exist(baselineAnalyDir, 'dir'), mkdir(baselineAnalyDir); end

    hasPrecedingOff = isfield(opts, 'guessInitSto') && ~isempty(opts.guessInitSto);

    if hasPrecedingOff
        mocoResDir_off  = fileparts(opts.guessInitSto);
        iterRootDir_off = fileparts(mocoResDir_off);
        analyResDir_off = fullfile(iterRootDir_off, 'analy_result');

        if ~exist(baselineMocoDir, 'dir'), mkdir(baselineMocoDir); end

        % GRF 복사 → baseline/moco_result/baseline_GRF.sto
        grfFiles = dir(fullfile(mocoResDir_off, '*_GRF.sto'));
        if ~isempty(grfFiles)
            copyfile(fullfile(mocoResDir_off, grfFiles(1).name), ...
                     fullfile(baselineMocoDir, 'baseline_GRF.sto'));
            fprintf('[baseline] GRF copied: %s → baseline_GRF.sto\n', grfFiles(1).name);
        else
            warning('[baseline] preceding modeOff moco_result 에서 *_GRF.sto 를 찾지 못했습니다.');
        end

        % kinematics 복사 → baseline/moco_result/ (파일명 그대로)
        kinFiles = dir(fullfile(mocoResDir_off, '*_kinematics*.sto'));
        for kf = kinFiles'
            copyfile(fullfile(mocoResDir_off, kf.name), fullfile(baselineMocoDir, kf.name));
            fprintf('[baseline] kinematics copied: %s\n', kf.name);
        end
        if isempty(kinFiles)
            warning('[baseline] preceding modeOff moco_result 에서 *_kinematics*.sto 를 찾지 못했습니다.');
        end

        % id_withAssist.sto 복사 → baseline/analy_result/
        idStoSrc = fullfile(analyResDir_off, 'id_withAssist.sto');
        if isfile(idStoSrc)
            copyfile(idStoSrc, fullfile(baselineAnalyDir, 'id_withAssist.sto'));
            fprintf('[baseline] id_withAssist.sto copied\n');
        else
            warning('[baseline] preceding modeOff analy_result 에서 id_withAssist.sto 를 찾지 못했습니다: %s', analyResDir_off);
        end

        baselineKinPath = opts.guessInitSto;
        fprintf('=== Baseline analysis (preceding modeOff → baseline/analy_result) ===\n');
    else
        baselineKinPath = guessInitSto;
        fprintf('=== Baseline analysis (guess_init → baseline/analy_result) ===\n');
    end

    WoC_moco_analysis(AnalySetupPath, ...
        'modelPath',         fullfile(modelPath, ModelNameOsim), ...
        'kinematicsStoPath', baselineKinPath, ...
        'resultsDir',        baselineAnalyDir);
    fprintf('Baseline analysis done.\n');
end

%% --------------------------------------------------
%  3. 메인 루프
% ---------------------------------------------------
sealedOccurred = false;
sealedIter     = -1;

for i = startIter:endIter
    fprintf('===== Iteration %d / %d =====\n', i, iterNum+baseIter);

    %------------------------------------------------
    % 3-1. 이전 iter의 analysis 결과 경로 및 GRF 경로 결정
    %
    %  prevAnalyDir : control 생성의 입력 데이터 소스
    %    - i==1 (일반): baseline/analy_result/
    %    - i==startIter (resume): resumeAbsDir/analy_result/
    %    - 그 외: result_(i-1)/analy_result/
    %
    %  prevGrfPath : stance 검출용 GRF
    %    - i==1 (일반): inputs/GRF_init_*.sto
    %    - i==startIter (resume): resumeAbsDir/moco_result/...GRF.sto
    %    - 그 외: result_(i-1)/moco_result/...GRF.sto
    %------------------------------------------------
    if ~resume_mode && i == 1
        prevAnalyDir    = fullfile(OutputFolder, 'baseline', 'analy_result');
        baselineGrfPath = fullfile(OutputFolder, 'baseline', 'moco_result', 'baseline_GRF.sto');
        if isfield(opts, 'guessInitSto') && ~isempty(opts.guessInitSto) && isfile(baselineGrfPath)
            prevGrfPath = baselineGrfPath;
        else
            prevGrfPath = grfInitSto;
        end
    elseif resume_mode && i == startIter
        prevAnalyDir = fullfile(resumeAbsDir, 'analy_result');
        prevGrfPath  = fullfile(resumeAbsDir, 'moco_result', ...
                           sprintf('moco_WoC_Solution_iter%02d_GRF.sto', baseIter));
    else
        prevIdx      = i - 1;
        prevAnalyDir = fullfile(OutputFolder, sprintf('result_%d', prevIdx), 'analy_result');
        prevGrfPath  = fullfile(OutputFolder, sprintf('result_%d', prevIdx), 'moco_result', ...
                           sprintf('moco_WoC_Solution_iter%02d_GRF.sto', prevIdx));
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
    %
    %  modeWoC    : QP 기반 WoC 제어기
    %  modeOff    : 보조력 없음 (영벡터 control.sto 생성)
    %  modeSpline : prevGrfPath에서 stance 추출 후 spline 생성
    %------------------------------------------------
    switch modeType

        case 'modeWoC'
            %--------------------------------------------
            % WoC mode: QP 기반 최적 보조 토크 계산
            % (analysis 결과는 prevAnalyDir에서 읽음 — 이전 iter 결과)
            %--------------------------------------------

            CoMPath   = fullfile(prevAnalyDir, '2D_gait_AFO_pc_PointKinematics_CoM_pos.sto');
            CoP_RPath = fullfile(prevAnalyDir, '2D_gait_AFO_pc_PointKinematics_CoP_R_pos.sto');

            % 3-3. GRF에서 오른발 stance time 추출
            [stanceTimeR_101, fullTime] = WoC_moco_detectStance(prevGrfPath);

            % 3-4. QP input (eta, w, t) 계산
            optsCalQP = struct();
            optsCalQP.CoMFields   = {'state_0','state_1','state_2'};
            optsCalQP.CoP_RFields = {'state_0','state_1','state_2'};
            optsCalQP.apAxis   = 'x';
            optsCalQP.vertAxis = 'y';
            optsCalQP.wPath  = fullfile(prevAnalyDir, '2D_gait_AFO_pc_Kinematics_u.sto');
            optsCalQP.wField = 'ankle_angle_r';
            optsCalQP.vPath  = fullfile(prevAnalyDir, '2D_gait_AFO_pc_Kinematics_u.sto');
            optsCalQP.vField = 'pelvis_tx';

            [etaR_101, wR_101, v_101, dt] = WoC_moco_cal_QP_input( ...
                CoMPath, CoP_RPath, stanceTimeR_101, optsCalQP);
            fprintf('dt = %.6f\n', dt);

            % 3-5. QP 풀어서 tau_R(stance 구간 control) 계산
            qpOpts             = struct();
            qpOpts.QP_effort   = coeffi_cost;
            qpOpts.QP_smooth   = coeffi_smoothing;
            qpOpts.tauDotMax   = 10;

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
            fprintf(fid, '100\t0\t0\n');
            fclose(fid);

        case 'modeSpline'
            %--------------------------------------------
            % Spline mode: trapezoid cubic Hermite 보조력 프로파일
            %
            %   보행 주기 [0, 1] 정규화 도메인에서 spline 생성 후
            %   fullTime 전체 구간에 매핑하여 control.sto 출력.
            %   trigger+rise+flat+fall > 1 이면 fall이 주기 초반에 wrap.
            %   Analysis / QP 생략.
            %--------------------------------------------

            % fullTime 추출 (GRF 파일의 전체 시간 축)
            [~, fullTime] = WoC_moco_detectStance(prevGrfPath);

            % Spline control 생성 ([0, 1] 정규화 도메인 → fullTime 포인트 수 tau 벡터)
            tau_R = WoC_moco_buildSplineControl(modeParams, numel(fullTime));

            % Control.sto + data.csv 출력 (전체 보행 주기에 적용)
            % (eta, w 는 modeSpline에서 의미 없으므로 placeholder 사용)
            dummyEta             = zeros(numel(fullTime), 1);
            dummyW               = ones(numel(fullTime), 1);
            writeOpts_sp         = struct();
            writeOpts_sp.dataColName = 'spline';
            % modeSym: AFO_r 의 50~100% 구간을 AFO_l 의 0~50% 구간에 매핑
            % (왼발이 오른발과 50% 위상차를 가지므로, 반 주기 풀기에서 누락되는
            %  오른발 50%+ 보조력을 왼발 0%+ 에 주입)
            if strcmpi(gaitMode, 'modeSym')
                T_half = (fullTime(end) - fullTime(1)) / 2;
                writeOpts_sp.controlLeft = interp1(fullTime, tau_R, fullTime + T_half, 'linear', 0);
            end
            WoC_moco_writeControl(controlResultDir, ...
                fullTime, fullTime, tau_R, dummyEta, dummyW, writeOpts_sp);

            controlRefStoPath = fullfile(controlResultDir, 'control.sto');

        case 'modeTorqAmp'
            %--------------------------------------------
            % TorqAmp mode: 이전 iter 의 id_withAssist.sto 에서
            %   ankle_angle_r_moment 를 읽어 부호 반전(plantarflexion=양수),
            %   음수 클리핑 후 [0, maxVal] 로 스케일한 control.sto 출력.
            %   Analysis / QP 생략.
            %
            %   i==1        : baseline/analy_result/id_withAssist.sto  (preceding modeOff 복사본)
            %   i==startIter(resume): resumeAbsDir/analy_result/id_withAssist.sto
            %   i>1         : result_(i-1)/analy_result/id_withAssist.sto
            %--------------------------------------------

            if ~resume_mode && i == 1
                idStoPath = fullfile(OutputFolder, 'baseline', 'analy_result', 'id_withAssist.sto');
            elseif resume_mode && i == startIter
                idStoPath = fullfile(resumeAbsDir, 'analy_result', 'id_withAssist.sto');
            else
                idStoPath = fullfile(OutputFolder, sprintf('result_%d', i-1), 'analy_result', 'id_withAssist.sto');
            end

            if ~isfile(idStoPath)
                error('modeTorqAmp: id_withAssist.sto 를 찾을 수 없습니다: %s', idStoPath);
            end
            fprintf('[modeTorqAmp] 발목 토크 참조 파일: %s\n', idStoPath);

            cutoffHz_ta = getOpt(modeParams, 'cutoffHz', 6);
            [tau_R_raw, t_id] = WoC_moco_buildTorqAmpControl(idStoPath, modeParams.maxVal, cutoffHz_ta);
            if strcmpi(gaitMode, 'modeSym')
                % i=1 에서 id_withAssist.sto 가 half-cycle만 커버할 수 있으므로
                % GRF 기반 fullTime 에 보간하여 time 축을 full gait cycle 로 통일
                [~, fullTime] = WoC_moco_detectStance(prevGrfPath);
                tau_R = interp1(t_id, tau_R_raw, fullTime, 'linear', 0);
            else  % modeAsym
                % id_withAssist.sto 가 항상 full gait cycle을 커버하므로 시간축 직접 사용
                fullTime = t_id;
                tau_R    = tau_R_raw;
            end

            dummyEta_ta          = zeros(numel(fullTime), 1);
            dummyW_ta            = ones(numel(fullTime), 1);
            writeOpts_ta         = struct();
            writeOpts_ta.dataColName = 'torqAmp';
            % modeSym: 오른발 50~100% plantarflexion torque 를 왼발 0~50% 에 주입
            if strcmpi(gaitMode, 'modeSym')
                T_half = (fullTime(end) - fullTime(1)) / 2;
                writeOpts_ta.controlLeft = interp1(fullTime, tau_R, fullTime + T_half, 'linear', 0);
            end
            WoC_moco_writeControl(controlResultDir, ...
                fullTime, fullTime, tau_R, dummyEta_ta, dummyW_ta, writeOpts_ta);

            controlRefStoPath = fullfile(controlResultDir, 'control.sto');

    end  % switch modeType

    %------------------------------------------------
    % 3-7. Moco loop에 사용할 guess 경로 결정
    %------------------------------------------------
    if ~resume_mode
        if i == 1
            if isfield(opts, 'guessInitSto') && ~isempty(opts.guessInitSto)
                guessStoPath = opts.guessInitSto;
                [~, gn, ge] = fileparts(opts.guessInitSto);
                fprintf('  [PRECEDING OFF TRIAL -> Initial Guess] %s%s\n', gn, ge);
            else
                guessStoPath = guessInitSto;
                fprintf('  [Default Initial Guess] \n');
            end
        else
            prevMocoDir = fullfile(OutputFolder, sprintf('result_%d', i-1), 'moco_result');
            if strcmpi(gaitMode, 'modeSym')
                guessStoPath = fullfile(prevMocoDir, ...
                    sprintf('moco_WoC_Solution_iter%02d_kinematics_half.sto', i-1));
            else  % modeAsym
                guessStoPath = fullfile(prevMocoDir, ...
                    sprintf('moco_WoC_Solution_iter%02d_kinematics.sto', i-1));
            end
        end
    else
        if i == startIter
            prevMocoDir = fullfile(resumeAbsDir, 'moco_result');
        else
            prevMocoDir = fullfile(OutputFolder, sprintf('result_%d', i-1), 'moco_result');
        end
        prevIterIdx = i - 1;
        if i == startIter
            prevIterIdx = baseIter;
        end
        if strcmpi(gaitMode, 'modeSym')
            guessStoPath = fullfile(prevMocoDir, ...
                sprintf('moco_WoC_Solution_iter%02d_kinematics_half.sto', prevIterIdx));
        else  % modeAsym
            guessStoPath = fullfile(prevMocoDir, ...
                sprintf('moco_WoC_Solution_iter%02d_kinematics.sto', prevIterIdx));
        end
    end

    %------------------------------------------------
    % 3-8. Moco loop: control reference → 새로운 보행 궤적
    %       주입된 osim은 AnalyResultDir에 저장됨
    %------------------------------------------------
    baseOsimPath = fullfile(modelPath, ModelNameOsim);
    sol = moco_WoC_loop(controlRefStoPath, guessStoPath, i, AnalyResultDir, baseOsimPath, MocoOpts);

    %------------------------------------------------
    % 3-8b. Sealed 검사: solver 실패(미수렴/max iter) 시
    %        unseal 후 _unseal 접미사로 저장하고 루프 종료.
    %        수렴하지 않은 궤적을 이후 analysis / ID에 넘기지 않는다.
    %------------------------------------------------
    if sol.isSealed()
        warning('[Iter %d] Solver did not converge (solution sealed). Saving partial result and stopping.', i);
        sol.unseal();
        resOpts_unseal           = struct();
        resOpts_unseal.modelPath = fullfile(AnalyResultDir, sprintf('%s_%d.osim', modelName, i));
        resOpts_unseal.prefix    = sprintf('moco_WoC_Solution_iter%02d_unseal', i);
        resOpts_unseal.gaitMode  = gaitMode;
        moco_WoC_getResult(sol, mocoResultDir, resOpts_unseal);
        sealedOccurred = true;
        sealedIter     = i;
        break;
    end

    %------------------------------------------------
    % 3-9. Moco 결과 저장 (kinematics, GRF, metabolic)
    %------------------------------------------------
    resOpts           = struct();
    resOpts.modelPath = fullfile(AnalyResultDir, sprintf('%s_%d.osim', modelName, i));
    resOpts.prefix    = sprintf('moco_WoC_Solution_iter%02d', i);
    resOpts.gaitMode  = gaitMode;
    moco_WoC_getResult(sol, mocoResultDir, resOpts);

    %------------------------------------------------
    % 3-10. 현재 iter 결과에 대한 analysis (전 모드 공통)
    %
    %  result_i/moco_result/kinematics.sto → result_i/analy_result/
    %  다음 iter의 control 생성 입력으로 사용된다.
    %------------------------------------------------
    currKinPath = fullfile(mocoResultDir, sprintf('moco_WoC_Solution_iter%02d_kinematics.sto', i));
    currOsmPath = fullfile(AnalyResultDir, sprintf('%s_%d.osim', modelName, i));
    WoC_moco_analysis(AnalySetupPath, ...
        'modelPath',         currOsmPath, ...
        'kinematicsStoPath', currKinPath, ...
        'resultsDir',        AnalyResultDir);

    %------------------------------------------------
    % 3-11. Inverse Dynamics (with / without assist)
    %
    %  id_withAssist.sto   : iter에서 사용한 model (analy_result에 저장된 model)
    %  id_withoutAssist.sto: origin model (입력 model)
    %  Results saved to analy_result/
    %------------------------------------------------

    idXmlPath      = fullfile(inputPath, 'id_setup.xml');
    kinQStoPath    = fullfile(AnalyResultDir, '2D_gait_AFO_pc_Kinematics_q.sto');
    % GRF_setup_id.xml이 analy_result/에 저장되므로 moco_result/는 상대경로 ../moco_result/
    currGrfStoPath = sprintf('../moco_result/moco_WoC_Solution_iter%02d_GRF.sto', i);

    % GRF_setup.xml의 datafile을 현재 iter GRF.sto 경로로 업데이트 후 임시 저장
    grfXmlDoc = xmlread(fullfile(inputPath, 'GRF_setup.xml'));
    grfXmlDoc.getElementsByTagName('datafile').item(0).setTextContent(currGrfStoPath);
    tempGrfXml = fullfile(AnalyResultDir, 'GRF_setup_id.xml');
    xmlwrite(tempGrfXml, grfXmlDoc);

    % with assist (iter에서 사용한 model)
    idTool = InverseDynamicsTool(idXmlPath);
    idTool.setModelFileName(currOsmPath);
    idTool.setCoordinatesFileName(kinQStoPath);
    idTool.setExternalLoadsFileName(tempGrfXml);
    idTool.setResultsDir(AnalyResultDir);
    idTool.setOutputGenForceFileName('id_withAssist.sto');
    idTool.run();

    % without assist (origin model)
    idTool2 = InverseDynamicsTool(idXmlPath);
    idTool2.setModelFileName(baseOsimPath);
    idTool2.setCoordinatesFileName(kinQStoPath);
    idTool2.setExternalLoadsFileName(tempGrfXml);
    idTool2.setResultsDir(AnalyResultDir);
    idTool2.setOutputGenForceFileName('id_withoutAssist.sto');
    idTool2.run();

    fprintf('Iteration %d done.\n', i);
end

if sealedOccurred
    error('WoC:sealedSolution', ...
        'Solver did not converge at iteration %d (solution sealed). Partial result saved with _unseal suffix.', ...
        sealedIter);
end

fprintf('All iterations finished.\n');
end


%% ---- 옵션 읽기용 헬퍼 ----
function val = getOpt(s, field, defaultVal)
    if isfield(s, field) && ~isempty(s.(field))
        val = s.(field);
    else
        val = defaultVal;
    end
end
