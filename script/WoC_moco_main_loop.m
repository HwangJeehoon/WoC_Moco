% 호출 규약:
%   WoC_moco_main(model, iter, optMode, result_name)
%   WoC_moco_main(model, iter, optMode, result_name, opts)
%   WoC_moco_main(model, iter, optMode, result_name, opts, optResume)
%
%   optMode: 'modeWoC'    - WoC QP 기반 최적 보조 토크
%            'modeOff'    - 보조력 없음 (AFO=0, QP 생략)
%            struct       - modeSpline (optMode.type = 'modeSpline', + 파라미터 필드)
%
%   opts (선택 struct):
%     .alpha    : QP 비용 계수 (default = 0.01)
%     .beta     : QP 스무딩 계수 (default = 0)
%     .cost     : Main cost 종류 'et'|'etw'|'etv' (default = 'et')
%     .effort   : Moco effort goal weight (default = 1)
%     .finalTime: Moco final time goal weight (default = 0.03)
%     .gaitMode : 'modeSym' | 'modeAsym' (default = 'modeSym')
%
%   Resume 예시 (optResume.resume_name 이 있으면 자동 resume):
%     optResume.resume_name = 'et_a001b0_iter600\result_600';
%     WoC_moco_main('2D_gait_AFO_pc.osim', 400, 'modeWoC', 'et_a001b0_iter600to1000', opts, optResume)

%% test
clc; clear; close all;
opts.alpha = 0.01;
WoC_moco_main('2D_gait_AFO_pc.osim', 2, 'modeWoC', 'test1', opts)

clc; clear; close all;
WoC_moco_main('2D_gait_AFO_pc_20BW.osim', 2, 'modeOff', 'test2')

clc; clear; close all;
optMode.type    = 'modeSpline';
optMode.trigger = 0.1;
optMode.rise    = 0.1;
optMode.flat    = 0.2;
optMode.fall    = 0.1;
optMode.maxVal  = 1.0;
WoC_moco_main('2D_gait_AFO_pc_30BW.osim', 2, optMode, 'test3')

clc; clear; close all;
optResume.resume_name = 'test3\result_1';
WoC_moco_main('2D_gait_AFO_pc_20BW.osim', 2, 'modeOff', 'test4', struct(), optResume)

clc; clear; close all;
optMode.type    = 'modeSpline';
optMode.trigger = 0.1;
optMode.rise    = 0.1;
optMode.flat    = 0.0;
optMode.fall    = 0.1;
optMode.maxVal  = 0.8;
WoC_moco_main('2D_gait_AFO_pc_30BW.osim', 2, optMode, 'test5')

%% modeWoC 예시

% clc; clear; close all;
% opts.alpha = 0.01;
% WoC_moco_main('2D_gait_AFO_pc.osim', 300, 'modeWoC', 'GRF_debug_et_a001b0_iter300', opts)

% clc; clear; close all;
% opts.alpha = 0.01;
% optResume.resume_name = 'et_a001b0_iter600\result_600';
% WoC_moco_main('2D_gait_AFO_pc.osim', 400, 'modeWoC', 'et_a001b0_iter600to1000', opts, optResume)

% clc; clear; close all;
% opts.alpha = 0.3;
% WoC_moco_main('2D_gait_AFO_pc.osim', 300, 'modeWoC', 'et_a03b0_iter300', opts)

% clc; clear; close all;
% opts.alpha = 0.01;
% WoC_moco_main('2D_gait_AFO_pc_30BW.osim', 300, 'modeWoC', 'et_a001b0_iter300_30BW', opts)

% clc; clear; close all;
% opts.alpha = 0.05;
% WoC_moco_main('2D_gait_AFO_pc_30BW.osim', 300, 'modeWoC', 'et_a005b0_iter300_30BW', opts)

% clc; clear; close all;
% opts.alpha = 0.1;
% WoC_moco_main('2D_gait_AFO_pc_30BW.osim', 300, 'modeWoC', 'et_a01b0_iter300_30BW', opts)

% clc; clear; close all;
% opts.alpha = 0.3;
% WoC_moco_main('2D_gait_AFO_pc_30BW.osim', 300, 'modeWoC', 'et_a03b0_iter300_30BW', opts)

% clc; clear; close all;
% opts.alpha = 0.5;
% WoC_moco_main('2D_gait_AFO_pc_30BW.osim', 300, 'modeWoC', 'et_a05b0_iter300_30BW', opts)

% clc; clear; close all;
% opts.alpha = 0.01;
% optResume.resume_name = 'et_a001b0_iter1000\result_1000';
% WoC_moco_main('2D_gait_AFO_pc.osim', 500, 'modeWoC', 'et_a001b0_iter1000to1500', opts, optResume)

% clc; clear; close all;
% opts.alpha = 0.01;
% WoC_moco_main('2D_gait_AFO_pc_40BW.osim', 300, 'modeWoC', 'et_a001b0_iter300_40BW', opts)

% clc; clear; close all;
% opts.alpha = 0.05;
% WoC_moco_main('2D_gait_AFO_pc_40BW.osim', 300, 'modeWoC', 'et_a005b0_iter300_40BW', opts)

% clc; clear; close all;
% opts.alpha = 0.1;
% WoC_moco_main('2D_gait_AFO_pc_40BW.osim', 300, 'modeWoC', 'et_a01b0_iter300_40BW', opts)

% clc; clear; close all;
% opts.alpha = 0.3;
% WoC_moco_main('2D_gait_AFO_pc_40BW.osim', 300, 'modeWoC', 'et_a03b0_iter300_40BW', opts)

% clc; clear; close all;
% opts.alpha = 0.5;
% WoC_moco_main('2D_gait_AFO_pc_40BW.osim', 300, 'modeWoC', 'et_a05b0_iter300_40BW', opts)

% clc; clear; close all;
% opts.alpha = 0.01;
% WoC_moco_main('2D_gait_AFO_pc_20BW.osim', 300, 'modeWoC', 'et_a001b0_iter300_20BW', opts)

% clc; clear; close all;
% opts.alpha = 0.05;
% WoC_moco_main('2D_gait_AFO_pc_20BW.osim', 300, 'modeWoC', 'et_a005b0_iter300_20BW', opts)  % 258번에서 오류뜸

% clc; clear; close all;
% opts.alpha = 0.01;
% optResume.resume_name = 'et_a005b0_iter300_20BW\result_258';
% WoC_moco_main('2D_gait_AFO_pc_20BW.osim', 45, 'modeWoC', 'et_a005b0_iter258to300_20BW', opts, optResume)

% clc; clear; close all;
% opts.alpha = 0.1;
% WoC_moco_main('2D_gait_AFO_pc_20BW.osim', 300, 'modeWoC', 'et_a01b0_iter300_20BW', opts)

% clc; clear; close all;
% opts.alpha = 0.3;
% WoC_moco_main('2D_gait_AFO_pc_20BW.osim', 300, 'modeWoC', 'et_a03b0_iter300_20BW', opts)  % 36번까지 돌리고 끊음

% clc; clear; close all;
% opts.alpha = 0.3;
% optResume.resume_name = 'et_a03b0_iter300_20BW\result_36';
% WoC_moco_main('2D_gait_AFO_pc_20BW.osim', 270, 'modeWoC', 'et_a005b0_iter36to300_20BW', opts, optResume)

% clc; clear; close all;
% opts.alpha = 0.5;
% WoC_moco_main('2D_gait_AFO_pc_20BW.osim', 300, 'modeWoC', 'et_a05b0_iter300_20BW', opts)

clc; clear; close all;
opts.alpha = 0.01;
WoC_moco_main('2D_gait_AFO_pc_20BW.osim', 300, 'modeWoC', 'test2', opts)

%% modeOff 예시

% clc; clear; close all;
% WoC_moco_main('2D_gait_AFO_pc_20BW.osim', 1, 'modeOff', 'off_20BW')

%% modeSpline 예시
%   optMode를 struct로 전달. trigger+rise+flat+fall <= 0.6, maxVal in [0,1]
%
%   정규화 도메인 [0, 0.6] 기준 파형 예시:
%     (trigger=0.1, rise=0.1, flat=0.2, fall=0.1, max=0.8)
%     → 0.1에서 시작해 0.1 동안 상승, 0.2 동안 0.8 유지, 0.1 동안 하강

% clc; clear; close all;
% optMode.type    = 'modeSpline';
% optMode.trigger = 0.1;
% optMode.rise    = 0.1;
% optMode.flat    = 0.2;
% optMode.fall    = 0.1;
% optMode.maxVal  = 0.8;
% WoC_moco_main('2D_gait_AFO_pc_20BW.osim', 300, optMode, 'spline_t01r01f02d01m08_20BW')

%% 근육 장애 (modeWoC)

% clc; clear; close all;
% opts.alpha = 0.01;
% WoC_moco_main('2D_gait_AFO_pc_sol25.osim', 300, 'modeWoC', 'et_a001b0_iter300_50BW_sol25', opts)

% clc; clear; close all;
% opts.alpha = 0.01;
% WoC_moco_main('2D_gait_AFO_pc_gastr25.osim', 300, 'modeWoC', 'et_a001b0_iter300_50BW_gastr25', opts)

%% Off 추출 (moco_WoC_loop_extractOff 직접 사용 - 단발성 추출용)
% clc; clear; close all;
%
% if isempty(mfilename)
%     thisFile = matlab.desktop.editor.getActiveFilename;
% else
%     thisFile = mfilename("fullpath");
% end
% baseFolder = fileparts(thisFile);
%
% Model = '2D_gait_AFO_pc_off_sol25.osim';
% resultDir = fullfile(baseFolder, "sol25_Off");
% guessInit ='guess_init_v5.sto';
% sol = moco_WoC_loop_extractOff(guessInit,Model);
% resOpts.modelPath = Model;
% moco_WoC_getResult(sol,resultDir,resOpts);
%
% Model2 = '2D_gait_AFO_pc_gastr25.osim';
% resultDir2 = fullfile(baseFolder, "gastr25_Off");
% guessInit ='guess_init_v5.sto';
% sol2 = moco_WoC_loop_extractOff(guessInit,Model2);
% resOpts.modelPath = Model2;
% moco_WoC_getResult(sol2,resultDir2,resOpts);

%% Asym test
% clc; clear; close all;
%
% if isempty(mfilename)
%     thisFile = matlab.desktop.editor.getActiveFilename;
% else
%     thisFile = mfilename("fullpath");
% end
% baseFolder = fileparts(thisFile);
%
% Model = '2D_gait_AFO_pc_off.osim';
% ModelPath = fullfile(baseFolder,'..','models',Model);
% guessInit ='guess_init_v5.sto';
% guessPath = fullfile(baseFolder,'..','inputs',guessInit);
% sol = moco_WoC_loop_asym(guessPath,ModelPath);
% resultDir = fullfile(baseFolder,'..',"results\Compare_Asym");
% resOpts.modelPath = ModelPath;
% moco_WoC_getResult(sol,resultDir,resOpts);
%
% Model = '2D_gait_AFO_pc_off.osim';
% ModelPath = fullfile(baseFolder,'..','models',Model);
% guessInit ='guess_init_v5.sto';
% guessPath = fullfile(baseFolder,'..','inputs',guessInit);
% sol = moco_WoC_loop_extractOff(guessPath,ModelPath);
% resultDir = fullfile(baseFolder,'..',"results\Compare_Sym");
% resOpts.modelPath = ModelPath;
% moco_WoC_getResult(sol,resultDir,resOpts);
%
% dataAsym = fullfile(baseFolder,'..',"results\Compare_Asym\moco_WoC_Solution_kinematics_half.sto");
% dataAsym = get_opensim_STO2(dataAsym);
% dataSym  = fullfile(baseFolder,'..',"results\Compare_Sym\moco_WoC_Solution_kinematics.sto");
% dataSym = get_opensim_STO2(dataSym);
%
% figure; hold on
% plot(dataAsym.time, dataAsym.x_jointset_ankle_l_ankle_angle_l_value)
% plot(dataSym.time, dataSym.x_jointset_ankle_l_ankle_angle_l_value)
% ylabel('Ankle(L)')
% xlabel('time')
% legend('Full', 'Half')
%
% figure; hold on
% plot(dataAsym.time, dataAsym.x_jointset_knee_l_knee_angle_l_value)
% plot(dataSym.time, dataSym.x_jointset_knee_l_knee_angle_l_value)
% ylabel('Knee(L)')
% xlabel('time')
% legend('Full', 'Half')
%
% figure; hold on
% plot(dataAsym.time, dataAsym.x_jointset_hip_l_hip_flexion_l_value)
% plot(dataSym.time, dataSym.x_jointset_hip_l_hip_flexion_l_value)
% ylabel('Hip(L)')
% xlabel('time')
% legend('Full', 'Half')
