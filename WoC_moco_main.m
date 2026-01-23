function WoC_moco_main
clc; clear; close all;

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
%  1. 파라미터 설정
% ---------------------------------------------------

% QP Parameter
iterNum          = 100;    % 원하는 iteration 수
coeffi_cost      = 5;    % QP alpha
coeffi_smoothing = 10;    % QP beta

% Main Cost
eta_tau = true;
eta_tau_w = false;
eta_tau_v = false;

% Moco Parameter
MocoOpts.weight_effort     = 1.0;    % Effort goal weight
MocoOpts.weight_finalTime  = 0.03;    % Final time goal weight

%% --------------------------------------------------
%  2. Output 폴더명 지정
% ---------------------------------------------------
OutputFolderName = 'et_a5b10';             % 원하는 폴더명
OutputFolder     = fullfile(baseFolder, OutputFolderName);
if ~exist(OutputFolder, 'dir')
    mkdir(OutputFolder);
end

%% --------------------------------------------------
%  3. 초기 데이터 경로 설정
%    (파일 이름은 환경에 맞게 수정해서 사용)
% ---------------------------------------------------
% AFO control = 0 인 초기 가이트(kinematics guess)
guessInitSto = fullfile(baseFolder, 'guess_init_v5.sto');

% 초기 GRF (full stride) – 첫 루프에서 stance 추출용
grfInitSto   = fullfile(baseFolder, 'GRF_init_v5.sto');

% AnalyzeTool(Pk, kinematics) setup XML
AnalySetupPath  = fullfile(baseFolder, 'analysis_setup.xml');

%% --------------------------------------------------
%  4. 메인 루프
% ---------------------------------------------------
for i = 1:iterNum
    fprintf('===== Iteration %d / %d =====\n', i, iterNum);

    %------------------------------------------------
    % 4-1. 이번 iteration에서 사용할
    %      (1) analy용 kinematics, (2) stance 검출용 GRF
    %------------------------------------------------
    if i == 1
        % 첫 iteration: tracking solution 기반 초기 가이트 사용
        kinStoForAnaly = guessInitSto;
        grfStoPath  = grfInitSto;
    else
        % 이후 iteration: 직전 iteration의 moco 결과 사용
        prevResultDir = fullfile(OutputFolder, sprintf('result_%d', i-1), 'moco_result');
        kinStoForAnaly   = fullfile(prevResultDir, sprintf('moco_WoC_Solution_iter%02d_kinematics.sto', i-1));
        grfStoPath    = fullfile(prevResultDir, sprintf('moco_WoC_Solution_iter%02d_GRF.sto', i-1));
    end

    % 이번 iteration의 result_i 폴더들
    iterRootDir    = fullfile(OutputFolder, sprintf('result_%d', i));
    AnalyResultDir    = fullfile(iterRootDir, 'analy_result');
    mocoResultDir  = fullfile(iterRootDir, 'moco_result');
    controlResultDir = fullfile(iterRootDir, 'control_result');

    if ~exist(iterRootDir, 'dir');       mkdir(iterRootDir);       end
    if ~exist(AnalyResultDir, 'dir');    mkdir(AnalyResultDir);       end
    if ~exist(mocoResultDir, 'dir');     mkdir(mocoResultDir);     end
    if ~exist(controlResultDir, 'dir');  mkdir(controlResultDir);  end

    %------------------------------------------------
    % 4-2. Analy 수행: CoM, CoP_R 좌표 .sto 추출
    %------------------------------------------------
    % analy_setup.xml 안에 PointKinematics가 정의되어 있고,
    % AnalyzeTool 결과 디렉터리가 AnalyResultDir가 되도록 override
    WoC_moco_analysis(AnalySetupPath, ...
        'kinematicsStoPath', kinStoForAnaly, ...
        'resultsDir', AnalyResultDir);

    % Analy 결과 파일명 (환경에 맞게 수정 가능)
    CoMPath  = fullfile(AnalyResultDir, '2D_gait_AFO_pc_PointKinematics_CoM_pos.sto');
    CoP_RPath = fullfile(AnalyResultDir, '2D_gait_AFO_pc_PointKinematics_CoP_R_pos.sto');

    %------------------------------------------------
    % 4-3. GRF에서 오른발 stance time 추출
    %------------------------------------------------
    [stanceTimeR_101, fullTime] = WoC_moco_detectStance(grfStoPath);

    %------------------------------------------------
    % 4-4. QP input (eta, w, t) 계산
    %------------------------------------------------
    optsCalQP = struct();
    % PointKinematics .sto 안에서 CoM, CoP_R의 x,y,z 필드 이름 지정
    optsCalQP.CoMFields   = {'state_0','state_1','state_2'};
    optsCalQP.CoP_RFields = {'state_0','state_1','state_2'};
    % AP/vertical 축 (예: x=AP, y=vertical)
    optsCalQP.apAxis   = 'x';
    optsCalQP.vertAxis = 'y';
    % w,v 관련 지정 (path 지정도 여기서 해야됨)
    optsCalQP.wPath = fullfile(AnalyResultDir, '2D_gait_AFO_pc_Kinematics_u.sto'); 
    optsCalQP.wField = 'ankle_angle_r';
    optsCalQP.vPath = fullfile(AnalyResultDir, '2D_gait_AFO_pc_Kinematics_u.sto'); 
    optsCalQP.vField = 'pelvis_tx';

    [etaR_101, wR_101, v_101, dt] = WoC_moco_cal_QP_input( ...
        CoMPath, CoP_RPath, stanceTimeR_101, optsCalQP);
    fprintf('dt = %.6f', dt)
    %------------------------------------------------
    % 4-5. QP 풀어서 tau_R(stance 구간 control) 계산
    %------------------------------------------------
    qpOpts       = struct();
    qpOpts.alpha = coeffi_cost;
    qpOpts.beta  = coeffi_smoothing;
    qpOpts.tauDotMax  = 10;

    % QP main cost 선택
    one_101 = ones(101,1);
    if     eta_tau
        QP_in    = one_101;
        qpColName = 'one_101';
    elseif eta_tau_w
        QP_in    = wR_101;
        qpColName = 'wR_101';
    elseif eta_tau_v
        QP_in    = v_101;
        qpColName = 'v_101';
    else 
        QP_in    = one_101; % 아무것도 안 켰으면 기본값
        qpColName = 'one_101';
    end
    if (eta_tau + eta_tau_w + eta_tau_v) > 1
        error('Main Cost flag는 eta_tau / eta_tau_w / eta_tau_v 중 하나만 true여야 합니다.');
    end
    tau_R = WoC_moco_solveQP(etaR_101, QP_in, dt, qpOpts);

    %------------------------------------------------
    % 4-6. 전체 시간축 control.sto + data.csv 출력
    %------------------------------------------------
    writeOpts.dataColName = qpColName;
    WoC_moco_writeControl(controlResultDir, ...
        fullTime, stanceTimeR_101, tau_R, etaR_101, QP_in, writeOpts);

    % control reference 파일 (AFO_right control.sto)
    controlRefStoPath = fullfile(controlResultDir, 'control.sto');

    %------------------------------------------------
    % 4-7. Moco loop: QP 결과 control을 reference로 넣고
    %      새로운 gait solution 계산
    %------------------------------------------------
    if i == 1
        % 첫 iteration: 초기 guess는 tracking solution 사용
        guessStoPath = guessInitSto;
    else
        % 이후 iteration: 직전 iteration의 kinematics를 guess로 사용
        prevKinematicsSto = fullfile( ...
            fullfile(OutputFolder, sprintf('result_%d', i-1), 'moco_result'), ...
            sprintf('moco_WoC_Solution_iter%02d_kinematics_half.sto', i-1));
        guessStoPath = prevKinematicsSto;
    end

    sol = moco_WoC_loop(controlRefStoPath, guessStoPath, i, AnalyResultDir, MocoOpts);

    %------------------------------------------------
    % 4-8. moco 결과 저장 (kinematics, GRF 등)
    %------------------------------------------------
    resOpts           = struct();
    resOpts.modelPath = fullfile(baseFolder, '2D_gait_AFO_pc.osim');
    resOpts.prefix    = sprintf('moco_WoC_Solution_iter%02d', i);

    moco_WoC_getResult(sol, mocoResultDir, resOpts);

    fprintf('Iteration %d done.\n', i);
end

fprintf('All iterations finished.\n');
end
