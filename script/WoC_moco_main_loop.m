% 일반 : WoC_moco_main(model, iter, alpha, beta, main cost, p, q, result_dir)
% 이어서 돌리기 : WoC_moco_main(model, iter, a, b, cost, p, q, result_dir, resume_mode, resume_dir)
% (resume_dir 예: 'et_a001b0_iter300\result_300' -> baseFolder 상대 경로로 넣어야 함)


% clc; clear; close all;
% WoC_moco_main('2D_gait_AFO_pc.osim',300, 0.01, 0,'et',1,0.03,'GRF_debug_et_a001b0_iter300', false) 
% 
% clc; clear; close all;
% WoC_moco_main('2D_gait_AFO_pc.osim',400, 0.01, 0,'et',1,0.03,'et_a001b0_iter600to1000', true, 'et_a001b0_iter600\result_600') 
% 
% clc; clear; close all;
% WoC_moco_main('2D_gait_AFO_pc.osim',300, 0.3, 0,'et',1,0.03,'et_a03b0_iter300', false)

% clc; clear; close all;
% WoC_moco_main('2D_gait_AFO_pc_30BW.osim',300, 0.01, 0,'et',1,0.03,'et_a001b0_iter300_30BW', false)
% 
% clc; clear; close all;
% WoC_moco_main('2D_gait_AFO_pc_30BW.osim',300, 0.05, 0,'et',1,0.03,'et_a005b0_iter300_30BW', false)
% 
% clc; clear; close all;
% WoC_moco_main('2D_gait_AFO_pc_30BW.osim',300, 0.1, 0,'et',1,0.03,'et_a01b0_iter300_30BW', false)

% clc; clear; close all;
% WoC_moco_main('2D_gait_AFO_pc_30BW.osim',300, 0.3, 0,'et',1,0.03,'et_a03b0_iter300_30BW', false)
% 
% clc; clear; close all;
% WoC_moco_main('2D_gait_AFO_pc_30BW.osim',300, 0.5, 0,'et',1,0.03,'et_a05b0_iter300_30BW', false)
% 
% clc; clear; close all;
% WoC_moco_main('2D_gait_AFO_pc.osim',500, 0.01, 0,'et',1,0.03,'et_a001b0_iter1000to1500', true, 'et_a001b0_iter1000\result_1000')

% clc; clear; close all;
% WoC_moco_main('2D_gait_AFO_pc_40BW.osim',300, 0.01, 0,'et',1,0.03,'et_a001b0_iter300_40BW', false)
% 
% clc; clear; close all;
% WoC_moco_main('2D_gait_AFO_pc_40BW.osim',300, 0.05, 0,'et',1,0.03,'et_a005b0_iter300_40BW', false)

% clc; clear; close all;
% WoC_moco_main('2D_gait_AFO_pc_40BW.osim',300, 0.1, 0,'et',1,0.03,'et_a01b0_iter300_40BW', false)
% 
% clc; clear; close all;
% WoC_moco_main('2D_gait_AFO_pc_40BW.osim',300, 0.3, 0,'et',1,0.03,'et_a03b0_iter300_40BW', false)

% clc; clear; close all;
% WoC_moco_main('2D_gait_AFO_pc_40BW.osim',300, 0.5, 0,'et',1,0.03,'et_a05b0_iter300_40BW', false)

% clc; clear; close all;
% WoC_moco_main('2D_gait_AFO_pc_20BW.osim',300, 0.01, 0,'et',1,0.03,'et_a001b0_iter300_20BW', false)

% clc; clear; close all;
% WoC_moco_main('2D_gait_AFO_pc_20BW.osim',300, 0.05, 0,'et',1,0.03,'et_a005b0_iter300_20BW', false) % 이거 돌리다가 258번에서 오류뜸

% clc; clear; close all;
% WoC_moco_main('2D_gait_AFO_pc_20BW.osim',45, 0.01, 0,'et',1,0.03,'et_a005b0_iter258to300_20BW', true, 'et_a005b0_iter300_20BW\result_258')

% clc; clear; close all;
% WoC_moco_main('2D_gait_AFO_pc_20BW.osim',300, 0.1, 0,'et',1,0.03,'et_a01b0_iter300_20BW', false)

% clc; clear; close all;
% WoC_moco_main('2D_gait_AFO_pc_20BW.osim',300, 0.3, 0,'et',1,0.03,'et_a03b0_iter300_20BW', false) % 36번까지 돌리고 끊음

% clc; clear; close all;
% WoC_moco_main('2D_gait_AFO_pc_20BW.osim',270, 0.3, 0,'et',1,0.03,'et_a005b0_iter36to300_20BW', true, 'et_a03b0_iter300_20BW\result_36')

% clc; clear; close all;
% WoC_moco_main('2D_gait_AFO_pc_20BW.osim',300, 0.5, 0,'et',1,0.03,'et_a05b0_iter300_20BW', false)

% clc; clear; close all;
% WoC_moco_main('2D_gait_AFO_pc_20BW.osim',45, 0.05, 0,'et',1,0.03,'et_a005b0_iter258to300_20BW', true, 'et_a005b0_iter300_20BW\result_258') %위에서 이거 잘못 돌림

% clc; clear; close all;
% WoC_moco_main('2D_gait_AFO_pc_20BW.osim',300, 0.1, 0,'et',1,0.03,'Debug_et_a01b0_iter300_20BW', false)

% clc; clear; close all;
% WoC_moco_main('2D_gait_AFO_pc_20BW.osim',300, 0.01, 0,'et',1,0.03,'Debug_et_a001b0_iter300_20BW', false)

clc; clear; close all;
WoC_moco_main('2D_gait_AFO_pc_20BW.osim',300, 0.01, 0,'et',1,0.03,'test2', false)

%% Off 추출
clc; clear; close all;

if isempty(mfilename)  % 스크립트처럼 실행하는 경우
    thisFile = matlab.desktop.editor.getActiveFilename;
else
    thisFile = mfilename("fullpath");
end
baseFolder = fileparts(thisFile);

Model = '2D_gait_AFO_pc_off_sol25.osim';
resultDir = fullfile(baseFolder, "sol25_Off");
guessInit ='guess_init_v5.sto';
sol = moco_WoC_loop_extractOff(guessInit,Model);
resOpts.modelPath = Model;
moco_WoC_getResult(sol,resultDir,resOpts);


Model2 = '2D_gait_AFO_pc_gastr25.osim';
resultDir2 = fullfile(baseFolder, "gastr25_Off");
guessInit ='guess_init_v5.sto';
sol2 = moco_WoC_loop_extractOff(guessInit,Model2);
resOpts.modelPath = Model2;
moco_WoC_getResult(sol2,resultDir2,resOpts);


%% Asym test
clc; clear; close all;

if isempty(mfilename)  % 스크립트처럼 실행하는 경우
    thisFile = matlab.desktop.editor.getActiveFilename;
else
    thisFile = mfilename("fullpath");
end
baseFolder = fileparts(thisFile);

Model = '2D_gait_AFO_pc_off.osim';
ModelPath = fullfile(baseFolder,'..','models',Model);
guessInit ='guess_init_v5.sto';
guessPath = fullfile(baseFolder,'..','inputs',guessInit);
sol = moco_WoC_loop_asym(guessPath,ModelPath);
resultDir = fullfile(baseFolder,'..',"results\Compare_Asym");
resOpts.modelPath = ModelPath;
moco_WoC_getResult(sol,resultDir,resOpts);

Model = '2D_gait_AFO_pc_off.osim';
ModelPath = fullfile(baseFolder,'..','models',Model);
guessInit ='guess_init_v5.sto';
guessPath = fullfile(baseFolder,'..','inputs',guessInit);
sol = moco_WoC_loop_extractOff(guessPath,ModelPath);
resultDir = fullfile(baseFolder,'..',"results\Compare_Sym");
resOpts.modelPath = ModelPath;
moco_WoC_getResult(sol,resultDir,resOpts);

dataAsym = fullfile(baseFolder,'..',"results\Compare_Asym\moco_WoC_Solution_kinematics_half.sto");
dataAsym = get_opensim_STO2(dataAsym);
dataSym  = fullfile(baseFolder,'..',"results\Compare_Sym\moco_WoC_Solution_kinematics.sto");
dataSym = get_opensim_STO2(dataSym);

figure; hold on
plot(dataAsym.time, dataAsym.x_jointset_ankle_l_ankle_angle_l_value)
plot(dataSym.time, dataSym.x_jointset_ankle_l_ankle_angle_l_value)
ylabel('Ankle(L)')
xlabel('time')
legend('Full', 'Half')

figure; hold on
plot(dataAsym.time, dataAsym.x_jointset_knee_l_knee_angle_l_value)
plot(dataSym.time, dataSym.x_jointset_knee_l_knee_angle_l_value)
ylabel('Knee(L)')
xlabel('time')
legend('Full', 'Half')

figure; hold on
plot(dataAsym.time, dataAsym.x_jointset_hip_l_hip_flexion_l_value)
plot(dataSym.time, dataSym.x_jointset_hip_l_hip_flexion_l_value)
ylabel('Hip(L)')
xlabel('time')
legend('Full', 'Half')
%% 근육 장애

% clc; clear; close all;
% WoC_moco_main('2D_gait_AFO_pc_sol25.osim',300, 0.01, 0,'et',1,0.03,'et_a001b0_iter300_50BW_sol25', false)

% clc; clear; close all;
% WoC_moco_main('2D_gait_AFO_pc_gastr25.osim',300, 0.01, 0,'et',1,0.03,'et_a001b0_iter300_50BW_gastr25', false)