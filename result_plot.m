clc;
clear;
close all;

%% baseFolder
if isempty(mfilename)  % 스크립트처럼 실행하는 경우
    thisFile = matlab.desktop.editor.getActiveFilename;
else
    thisFile = mfilename("fullpath");
end
baseFolder = fileparts(thisFile);

%% 기본 설정
OutputFolderName = 'et_a5b10';
figureFolderName = fullfile(OutputFolderName,'\fig');
iterNum          = 100;

OutputFolder   = fullfile(baseFolder, OutputFolderName);
FigureFolder   = fullfile(baseFolder, figureFolderName);

if ~exist(FigureFolder, 'dir')
    mkdir(FigureFolder);
end

grfInitSto     = fullfile(baseFolder, 'GRF_init_for_plotBaseline_v5.sto');        % GRF baseline -> full stride
guessInitSto   = fullfile(baseFolder, 'guess_init_for_plotBaseline_v5.sto');      % baseline -> full stride

pelvisField  = matlab.lang.makeValidName('/jointset/groundPelvis/pelvis_tx/value');
gastrocField = matlab.lang.makeValidName('/gastroc_r/activation');
soleusField  = matlab.lang.makeValidName('/soleus_r/activation');
afoField     = matlab.lang.makeValidName('/AFO_r');

%% Baseline 읽기 및 시간 0~1 normalize

% ---------------- GRF baseline STO 읽기 (inline) ----------------
fid = fopen(grfInitSto,'r');
if fid == -1
    error('Cannot open %s', grfInitSto);
end

line = fgetl(fid);
while ischar(line)
    if startsWith(strtrim(line),'endheader')
        break;
    end
    line = fgetl(fid);
end

varLine = fgetl(fid);
names   = strsplit(strtrim(varLine));
data    = fscanf(fid, '%f', [numel(names), Inf])';
fclose(fid);

grf0 = struct();
for k = 1:numel(names)
    fn = matlab.lang.makeValidName(names{k});
    grf0.(fn) = data(:,k);
end
% ---------------------------------------------------------------

t0       = grf0.time(:);
vx0      = grf0.ground_force_r_vx(:);   % 컬럼 이름 환경에 맞게 확인
t0_norm  = (t0 - t0(1)) / (t0(end) - t0(1));

% ---------------- Guess baseline STO 읽기 (inline) --------------
fid = fopen(guessInitSto,'r');
if fid == -1
    error('Cannot open %s', guessInitSto);
end

line = fgetl(fid);
while ischar(line)
    if startsWith(strtrim(line),'endheader')
        break;
    end
    line = fgetl(fid);
end

varLine = fgetl(fid);
names   = strsplit(strtrim(varLine));
data    = fscanf(fid, '%f', [numel(names), Inf])';
fclose(fid);

guess0 = struct();
for k = 1:numel(names)
    fn = matlab.lang.makeValidName(names{k});
    guess0.(fn) = data(:,k);
end
% ---------------------------------------------------------------

tg0          = guess0.time(:);
pelv0        = guess0.(pelvisField)(:);
avgSpeed0    = (pelv0(end) - pelv0(1)) / (tg0(end) - tg0(1));
gastrocAct0  = guess0.(gastrocField)(:);
soleusAct0   = guess0.(soleusField)(:);
tg0_norm     = (tg0 - tg0(1)) / (tg0(end) - tg0(1));

%% iteration별 GRF control 읽기
tNormGRF   = cell(iterNum, 1);
vxIter     = cell(iterNum, 1);
tNormCtrl  = cell(iterNum, 1);
uIter      = cell(iterNum, 1);

tNormData     = cell(iterNum, 1);
etaIter       = cell(iterNum, 1);
avgSpeedIter  = zeros(iterNum, 1);

tNormKin   = cell(iterNum, 1);
gastrocAct = cell(iterNum, 1);
soleusAct  = cell(iterNum, 1);

afoKinIter  = cell(iterNum, 1);
tNormKin2   = cell(iterNum, 1);
tNormCtrl2  = cell(iterNum, 1);

strideLength = zeros(iterNum, 1);

% ----------- cost 저장용 배열 -----------
objective_total      = nan(iterNum, 1);
objective_effort     = nan(iterNum, 1);
objective_final_time = nan(iterNum, 1);
% ---------------------------------------------

for i = 1:iterNum
    % result_i 폴더
    iterRootDir   = fullfile(OutputFolder, sprintf('result_%d', i));
    mocoResultDir = fullfile(iterRootDir, 'moco_result');
    controlDir    = fullfile(iterRootDir, 'control_result');
    pkDir         = fullfile(iterRootDir, 'analy_result');

    % ---------------- GRF STO 읽기 (inline) ----------------
    grfPath_i = fullfile(mocoResultDir, sprintf('moco_WoC_Solution_iter%02d_GRF.sto', i)); % -> full stride
    fid = fopen(grfPath_i,'r');
    if fid == -1
        error('Cannot open %s', grfPath_i);
    end

    line = fgetl(fid);
    while ischar(line)
        if startsWith(strtrim(line),'endheader')
            break;
        end
        line = fgetl(fid);
    end

    varLine = fgetl(fid);
    names   = strsplit(strtrim(varLine));
    data    = fscanf(fid, '%f', [numel(names), Inf])';
    fclose(fid);

    grf_i = struct();
    for k = 1:numel(names)
        fn = matlab.lang.makeValidName(names{k});
        grf_i.(fn) = data(:,k);
    end
    % -------------------------------------------------------

    ti        = grf_i.time(:);
    vxi       = grf_i.ground_force_r_vx(:);

    tNormGRF{i} = ti;
    vxIter{i}   = vxi;

    % ---------------- control STO 읽기 (inline) -------------
    ctrlPath_i = fullfile(controlDir, 'control.sto');
    fid = fopen(ctrlPath_i,'r');
    if fid == -1
        error('Cannot open %s', ctrlPath_i);
    end

    line = fgetl(fid);
    while ischar(line)
        if startsWith(strtrim(line),'endheader')
            break;
        end
        line = fgetl(fid);
    end

    varLine = fgetl(fid);
    names   = strsplit(strtrim(varLine));
    data    = fscanf(fid, '%f', [numel(names), Inf])';
    fclose(fid);

    ctrl_i = struct();
    for k = 1:numel(names)
        fn = matlab.lang.makeValidName(names{k});
        ctrl_i.(fn) = data(:,k);
    end
    % -------------------------------------------------------

    tci           = ctrl_i.time(:);
    ui            = ctrl_i.AFO_r(:);

    tNormCtrl{i}  = tci;
    tNormCtrl2{i} = tci;
    uIter{i}      = ui;

    % data 파일
    dataPath_i = fullfile(controlDir, 'data.csv');
    data_i     = readtable(dataPath_i);
    tci        = data_i.stanceTime(:);
    ui         = data_i.eta(:);

    if i == 1
        eta_baseline      = data_i.eta(:);
        time_eta_baseline = data_i.stanceTime(:);
    end

    tNormData{i} = tci;
    etaIter{i}   = ui;

    % ---------------- kinematics STO 읽기 (inline) ----------
    kinPath_i = fullfile(mocoResultDir, sprintf('moco_WoC_Solution_iter%02d_kinematics.sto', i)); % -> full stride
    fid = fopen(kinPath_i,'r');
    if fid == -1
        error('Cannot open %s', kinPath_i);
    end

    line = fgetl(fid);
    while ischar(line)
        if startsWith(strtrim(line),'endheader')
            break;
        end
        line = fgetl(fid);
    end

    varLine = fgetl(fid);
    names   = strsplit(strtrim(varLine));
    data    = fscanf(fid, '%f', [numel(names), Inf])';
    fclose(fid);

    kin_i = struct();
    for k = 1:numel(names)
        fn = matlab.lang.makeValidName(names{k});
        kin_i.(fn) = data(:,k);
    end
    % -------------------------------------------------------

    tk        = kin_i.time(:);
    tNormKin{i} = tk;

    if isfield(kin_i, pelvisField)
        pelv = kin_i.(pelvisField)(:);
        avgSpeedIter(i) = (pelv(end) - pelv(1)) / (tk(end) - tk(1));
    end

    if isfield(kin_i, gastrocField)
        gastrocAct{i} = kin_i.(gastrocField)(:);
    else
        gastrocAct{i} = [];
    end

    if isfield(kin_i, soleusField)
        soleusAct{i} = kin_i.(soleusField)(:);
    else
        soleusAct{i} = [];
    end

    % ---------------- (추가) cost 헤더 파싱 -----------------
    costPath_i = fullfile(mocoResultDir, sprintf('moco_WoC_Solution_iter%02d_kinematics_half.sto', i));
    if exist(costPath_i, 'file') == 2
        fid = fopen(costPath_i,'r');
        if fid == -1
            error('Cannot open %s', costPath_i);
        end

        obj      = nan;
        objEff   = nan;
        objTime  = nan;

        line = fgetl(fid);
        while ischar(line)
            s = strtrim(line);

            % endheader 만나면 중단
            if startsWith(s, 'endheader')
                break;
            end

            % objective 계열 파싱
            if startsWith(s, 'objective=')
                obj = sscanf(s, '%*[^=]=%f');
            elseif startsWith(s, 'objective_effort=')
                objEff = sscanf(s, '%*[^=]=%f');
            elseif startsWith(s, 'objective_final_time=')
                objTime = sscanf(s, '%*[^=]=%f');
            end

            line = fgetl(fid);
        end
        fclose(fid);

        objective_total(i)      = obj;
        objective_effort(i)     = objEff;
        objective_final_time(i) = objTime;

        % objective가 비어있고 effort time만 있으면 합으로 채움
        if isnan(objective_total(i)) && ~isnan(objective_effort(i)) && ~isnan(objective_final_time(i))
            objective_total(i) = objective_effort(i) + objective_final_time(i);
        end
    end
    % -------------------------------------------------------

    % ---------------- pk STO 읽기 (inline) ------------------
    pkPath_i = fullfile(pkDir, sprintf('2D_gait_AFO_pc_PointKinematics_CoP_L_pos.sto')); % -> half stride
    fid = fopen(pkPath_i,'r');
    if fid == -1
        error('Cannot open %s', pkPath_i);
    end

    line = fgetl(fid);
    while ischar(line)
        if startsWith(strtrim(line),'endheader')
            break;
        end
        line = fgetl(fid);
    end

    varLine = fgetl(fid);
    names   = strsplit(strtrim(varLine));
    data    = fscanf(fid, '%f', [numel(names), Inf])';
    fclose(fid);

    pk_i = struct();
    for k = 1:numel(names)
        fn = matlab.lang.makeValidName(names{k});
        pk_i.(fn) = data(:,k);
    end
    % -------------------------------------------------------

    stride_i = pk_i.state_0(:);
    num = round(length(pk_i.state_0(:))/2);
    if i == 1
        strideLength(i) = stride_i(end) - stride_i(1);
    else
        strideLength(i) = stride_i(num) - stride_i(1);
    end
end

%% 색깔 조절
colors = zeros(iterNum,3);

blue  = [0 0 1];
red   = [1 0 0];
white = [1 1 1];

minWhiteMix = 0.0;
maxWhiteMix = 0.95;

p = 2.2;

for i = 1:iterNum
    t = (i-1)/(iterNum-1);
    d = abs(2*t - 1);
    mixW = minWhiteMix + (maxWhiteMix - minWhiteMix) * (1 - d^p);

    base = (1-t)*blue + t*red;
    colors(i,:) = (1-mixW)*base + mixW*white;
end

colors(1,:) = blue;
colors(end,:) = red;

%% 1) GRF plot (ground_force_r_vx)
figure('Color','w','Position',[0 0 1200 800]);
hold on; box on;

plot(t0_norm, vx0, 'k', 'LineWidth', 4, 'DisplayName', 'baseline');

for i = 1:iterNum
    plot(tNormGRF{i}, vxIter{i}, 'Color', colors(i,:), ...
        'DisplayName', sprintf('iter %d', i), LineWidth=1);
end
yline(0, '--');
xlabel('Time (s)');
ylabel('Force (N)');
title(sprintf('%s, apGRF',OutputFolderName), 'Interpreter', 'none')
set(gca, fontsize=25)
exportgraphics(gcf, fullfile(FigureFolder, '01_apGRF_right.png'), 'Resolution', 300);

%% 2) Control plot (AFO_right)
figure('Color','w','Position',[0 0 1200 800]);
hold on; box on;

for i = 1:iterNum
    plot(tNormCtrl{i}, uIter{i}, 'Color', colors(i,:), ...
        'DisplayName', sprintf('iter %d', i), LineWidth=1);
end

xlabel('Time (s)');
ylabel('Control (0~1)');
title(sprintf('%s, AFO Optimal Control',OutputFolderName), 'Interpreter', 'none')
set(gca, fontsize=25)
exportgraphics(gcf, fullfile(FigureFolder, '02_control_AFO_right.png'), 'Resolution', 300);

%% 3) eta plot
figure('Color','w','Position',[0 0 1200 800]);
hold on; box on;

plot(time_eta_baseline, eta_baseline, 'k', 'LineWidth', 4, 'DisplayName', 'baseline');

for i = 1:iterNum
    plot(tNormData{i}, etaIter{i}, 'Color', colors(i,:), ...
        'DisplayName', sprintf('iter %d', i), LineWidth=1);
end

yline(0, '--');
xlabel('Time (s)');
ylabel('Eta');
title(sprintf('%s, Propulsive Transfer Ratio',OutputFolderName), 'Interpreter', 'none')
set(gca, fontsize=25)
exportgraphics(gcf, fullfile(FigureFolder, '03_eta_right.png'), 'Resolution', 300);

%% 4) 평균 보행 속도 plot
figure('Color','w','Position',[0 0 1200 800]);
hold on; box on;

plot(0, avgSpeed0, 'o-', 'Color',"black",'LineWidth', 10, 'DisplayName', 'baseline');

iters = 1:iterNum;
plot(iters, avgSpeedIter, 'o-', 'LineWidth', 1.5, ...
    'Color', [0 0.5 0.0], 'DisplayName', 'iter speed');

xlabel('Iteration');
ylabel('Avg Walking Speed (m/s)');
xlim([-1 iterNum+1])
final_stepTime = tNormKin{iterNum}(end);
title(sprintf('%s, Final Step Time: %.4f',OutputFolderName, final_stepTime), 'Interpreter', 'none')
set(gca, fontsize=25)
exportgraphics(gcf, fullfile(FigureFolder, '04_avg_walking_speed.png'), 'Resolution', 300);

%% 5) gastroc activation plot
figure('Color','w','Position',[0 0 1200 800]);
hold on; box on;

plot(tg0_norm, gastrocAct0, 'k', 'LineWidth', 4, 'DisplayName', 'baseline');

for i = 1:iterNum
    plot(tNormKin{i}, gastrocAct{i}, 'Color', colors(i,:), ...
        'DisplayName', sprintf('iter %d', i), LineWidth=1);
end

xlabel('Time (s)');
ylabel('Muscle Activation (0~1)');
title(sprintf('%s, Gastroc Activation',OutputFolderName), 'Interpreter', 'none')
set(gca, fontsize=25)
exportgraphics(gcf, fullfile(FigureFolder, '05_gastroc_r_activation.png'), 'Resolution', 300);

%% 6) sol activation plot
figure('Color','w','Position',[0 0 1200 800]);
hold on; box on;

plot(tg0_norm, soleusAct0, 'k', 'LineWidth', 4, 'DisplayName', 'baseline');

for i = 1:iterNum
    plot(tNormKin{i}, soleusAct{i}, 'Color', colors(i,:), ...
        'DisplayName', sprintf('iter %d', i), LineWidth=1);
end

xlabel('Time (s)');
ylabel('Muscle Activation (0~1)');
title(sprintf('%s, Soleus Activation',OutputFolderName), 'Interpreter', 'none')
set(gca, fontsize=25)
exportgraphics(gcf, fullfile(FigureFolder, '06_soleus_r_activation.png'), 'Resolution', 300);

%% 8) stride 비교
figure('Color','w','Position',[0 0 1200 800]);
hold on; box on;

plot(0, strideLength(1), 'o-', 'Color',"black",'LineWidth', 10, 'DisplayName', 'baseline');

iters = 1:iterNum-1;
plot(iters, strideLength(2:end), 'o-', 'LineWidth', 1.5, ...
    'Color', [0 0.5 0.0], 'DisplayName', 'iter speed');

xlabel('Iteration');
ylabel('Stride length (m)');
xlim([-1 iterNum+1])
title(sprintf('%s, Stride Length',OutputFolderName), 'Interpreter', 'none')
set(gca, fontsize=25)
exportgraphics(gcf, fullfile(FigureFolder, '08_stride_length.png'), 'Resolution', 300);

%% 9) cost 비교 (objective effort final_time total)
figure('Color','w','Position',[0 0 1200 800]);
hold on; box on;

iters = 1:iterNum;

plot(iters, objective_effort, 'o-', 'LineWidth', 1.5, 'DisplayName', 'objective effort');
plot(iters, objective_final_time, 'o-', 'LineWidth', 1.5, 'DisplayName', 'objective final time');
plot(iters, objective_total, 'o-', 'LineWidth', 1.5, 'DisplayName', 'objective total');

xlabel('Iteration');
ylabel('Cost');
xlim([-1 iterNum+1])
title(sprintf('%s, Objective Cost',OutputFolderName), 'Interpreter', 'none')
legend('Effort','FinalTime', 'Total', 'Location','best')
set(gca, fontsize=25)
exportgraphics(gcf, fullfile(FigureFolder, '09_objective_cost.png'), 'Resolution', 300);
