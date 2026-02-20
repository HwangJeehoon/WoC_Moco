clc; clear; close all;

%% baseFolder
if isempty(mfilename)
    thisFile = matlab.desktop.editor.getActiveFilename;
else
    thisFile = mfilename("fullpath");
end
baseFolder = fileparts(thisFile);

%% ====== 분석하고 싶은 output 설정 ======
outs = struct([]);

outs(1).name    = 'et_a001b0_iter300';
outs(1).iterNum = 300;

outs(2).name    = 'et_a005b0_iter300';
outs(2).iterNum = 300;

outs(3).name    = 'et_a01b0_iter300';
outs(3).iterNum = 300;

outs(4).name    = 'et_a03b0_iter300';
outs(4).iterNum = 300;

outs(5).name    = 'et_a05b0_iter300';
outs(5).iterNum = 300;

outs(6).name    = 'et_a1b0_iter300';
outs(6).iterNum = 300;

%% ====== Plot 저장 경로 설정 ======
FigureFolder = fullfile(baseFolder,'\analysis_fig');
if ~exist(FigureFolder, 'dir')
    mkdir(FigureFolder);
end

%% ====== field name 정의 ======
pelvisField  = matlab.lang.makeValidName('/jointset/groundPelvis/pelvis_tx/value');
pelvisSpeedField = matlab.lang.makeValidName('/jointset/groundPelvis/pelvis_tx/speed');
ankleAngVelField = matlab.lang.makeValidName('ankle_angle_r');

gastrocField = matlab.lang.makeValidName('/gastroc_r/activation');
soleusField  = matlab.lang.makeValidName('/soleus_r/activation');
tibAntField  = matlab.lang.makeValidName('/tib_ant_r/activation');
rectFemField  = matlab.lang.makeValidName('/rect_fem_r/activation');

hamStringField  = matlab.lang.makeValidName('/hamstrings_r/activation');
biFemshField  = matlab.lang.makeValidName('/bifemsh_r/activation');
glutMaxField  = matlab.lang.makeValidName('/glut_max_r/activation');
iliopsoasField  = matlab.lang.makeValidName('/iliopsoas_r/activation');
vastiField  = matlab.lang.makeValidName('/vasti_r/activation');

%% ====== 결과 저장 구조체 ======
All = struct([]);

for o = 1:numel(outs)

    outName = outs(o).name;
    iterNum = outs(o).iterNum;

    outDir  = fullfile(baseFolder, outName);

    All(o).name         = outName;
    All(o).iterNum      = iterNum;
    All(o).outputFolder = outDir;

    All(o).avgSpeedIter = nan(iterNum,1);
    All(o).iter(iterNum) = struct();

    for i = 1:iterNum
        mocoDir = fullfile(outDir, sprintf('result_%d', i), 'moco_result');

        % ---- GRF ----
        grfPath = fullfile(mocoDir, sprintf('moco_WoC_Solution_iter%02d_GRF.sto', i));
        fid = fopen(grfPath,'r');

        line = fgetl(fid);
        while ischar(line)
            if startsWith(strtrim(line),'endheader'), break; end
            line = fgetl(fid);
        end

        names = strsplit(strtrim(fgetl(fid)));
        data  = fscanf(fid, '%f', [numel(names), Inf])';
        fclose(fid);

        idxT  = find(strcmp(names,'time'),1);
        idxVx = find(strcmp(names,'ground_force_r_vx'),1);

        tGRF = data(:,idxT);
        vx   = data(:,idxVx);

        All(o).iter(i).grf.t  = tGRF;
        All(o).iter(i).grf.vx = vx;

        % ---- Kinematics ----
        kinPath = fullfile(mocoDir, sprintf('moco_WoC_Solution_iter%02d_kinematics.sto', i));
        fid = fopen(kinPath,'r');

        line = fgetl(fid);
        while ischar(line)
            if startsWith(strtrim(line),'endheader'), break; end
            line = fgetl(fid);
        end

        names = strsplit(strtrim(fgetl(fid)));
        data  = fscanf(fid, '%f', [numel(names), Inf])';
        fclose(fid);

        fn = matlab.lang.makeValidName(names);
        idxT  = find(strcmp(fn,'time'),1);
        idxPx = find(strcmp(fn, pelvisField),1);
        idxVxPel = find(strcmp(fn, pelvisSpeedField),1);

        idxGa = find(strcmp(fn, gastrocField),1);
        idxSa = find(strcmp(fn, soleusField),1);
        idxTA  = find(strcmp(fn, tibAntField),1);
        idxRF  = find(strcmp(fn, rectFemField),1);

        idxHS  = find(strcmp(fn, hamStringField),1);
        idxBFS = find(strcmp(fn, biFemshField),1);
        idxGM  = find(strcmp(fn, glutMaxField),1);
        idxIL  = find(strcmp(fn, iliopsoasField),1);
        idxVA  = find(strcmp(fn, vastiField),1);

        tKin = data(:,idxT);
        pelv = data(:,idxPx);
        vPel = data(:,idxVxPel);

        gAct = data(:,idxGa);
        sAct = data(:,idxSa);
        tibAnt = data(:,idxTA);
        rectFem = data(:,idxRF);

        hamStr  = data(:,idxHS);
        biFemsh = data(:,idxBFS);
        glutMax = data(:,idxGM);
        iliopsoas = data(:,idxIL);
        vasti   = data(:,idxVA);

        avgSpeed = (pelv(end) - pelv(1)) / (tKin(end) - tKin(1));

        All(o).iter(i).kin.t          = tKin;
        All(o).iter(i).kin.pelvisTx   = pelv;
        All(o).iter(i).kin.pelvisTxSpeed = vPel;
        All(o).iter(i).avgSpeed = avgSpeed;

        All(o).iter(i).kin.gastrocAct = gAct;
        All(o).iter(i).kin.soleusAct  = sAct;
        All(o).iter(i).kin.tibAntAct   = tibAnt;
        All(o).iter(i).kin.rectFemAct  = rectFem;
        
        All(o).iter(i).kin.hamStrAct   = hamStr;
        All(o).iter(i).kin.biFemshAct  = biFemsh;
        All(o).iter(i).kin.glutMaxAct  = glutMax;
        All(o).iter(i).kin.iliopsoasAct = iliopsoas;
        All(o).iter(i).kin.vastiAct    = vasti;


        % ---- Analy (ankle angular velocity) ----
        analyDir = fullfile(outDir, sprintf('result_%d', i), 'analy_result');
        KinematicsPath = fullfile(analyDir, '2D_gait_AFO_pc_Kinematics_u.sto');

        fid = fopen(KinematicsPath,'r');
        line = fgetl(fid);
        while ischar(line)
            if startsWith(strtrim(line),'endheader'), break; end
            line = fgetl(fid);
        end

        names = strsplit(strtrim(fgetl(fid)));
        data  = fscanf(fid, '%f', [numel(names), Inf])';
        fclose(fid);

        fn = matlab.lang.makeValidName(names);
        idxT  = find(strcmp(fn,'time'),1);
        idxW  = find(strcmp(fn, ankleAngVelField),1);

        All(o).iter(i).analy.t = data(:,idxT);
        All(o).iter(i).analy.w = data(:,idxW);


        % ---- Control ----
        controlDir = fullfile(outDir, sprintf('result_%d', i), 'control_result');
        ctrlPath   = fullfile(controlDir, 'control.sto');
        fid = fopen(ctrlPath,'r');

        line = fgetl(fid);
        while ischar(line)
            if startsWith(strtrim(line),'endheader'), break; end
            line = fgetl(fid);
        end

        names = strsplit(strtrim(fgetl(fid)));
        data  = fscanf(fid, '%f', [numel(names), Inf])';
        fclose(fid);

        idxT = find(strcmp(names,'time'),1);
        idxU = find(strcmp(names,'AFO_r'),1);

        tCtrl = data(:,idxT);
        uCtrl = data(:,idxU);

        All(o).iter(i).ctrl.t = tCtrl;
        All(o).iter(i).ctrl.u = uCtrl;
    end
end

%% ===== baseline metrics (Off_GRF.sto, Off_kinematics.sto) =====
grfInitSto   = fullfile(baseFolder, 'Off_GRF.sto');
guessInitSto = fullfile(baseFolder, 'Off_kinematics.sto');

% --- baseline propulsion ---
fid = fopen(grfInitSto,'r');
line = fgetl(fid);
while ischar(line)
    if startsWith(strtrim(line),'endheader'), break; end
    line = fgetl(fid);
end
names = strsplit(strtrim(fgetl(fid)));
data  = fscanf(fid, '%f', [numel(names), Inf])';
fclose(fid);

t0  = data(:, strcmp(names,'time'));
vx0 = data(:, strcmp(names,'ground_force_r_vx'));
baselineProp = trapz(t0, max(vx0,0));
baselinePeakAp = max(vx0);

% --- baseline CMAPD, Speed ---
fid = fopen(guessInitSto,'r');
line = fgetl(fid);
while ischar(line)
    if startsWith(strtrim(line),'endheader'), break; end
    line = fgetl(fid);
end
names = strsplit(strtrim(fgetl(fid)));
data  = fscanf(fid, '%f', [numel(names), Inf])';
fclose(fid);

fn = matlab.lang.makeValidName(names);
tk0   = data(:, strcmp(fn,'time'));
pelv0 = data(:, strcmp(fn, pelvisField));
vPel0 = data(:, strcmp(fn, pelvisSpeedField));

g0    = data(:, strcmp(fn, gastrocField));
s0    = data(:, strcmp(fn, soleusField));
ta0  = data(:, strcmp(fn, tibAntField));
rf0  = data(:, strcmp(fn, rectFemField));

hs0  = data(:, strcmp(fn, hamStringField));
bfs0 = data(:, strcmp(fn, biFemshField));
gm0  = data(:, strcmp(fn, glutMaxField));
il0  = data(:, strcmp(fn, iliopsoasField));
va0  = data(:, strcmp(fn, vastiField));

% Cal baseline metric 

stride0 = pelv0(end) - pelv0(1);

baselineCMAPD_GS     = (trapz(tk0,g0) + trapz(tk0,s0)) / stride0;
baselineCMAPD_Shank  = (trapz(tk0,g0) + trapz(tk0,s0) + trapz(tk0,ta0)) / stride0;
baselineCMAPD_4set   = (trapz(tk0,g0) + trapz(tk0,s0) + trapz(tk0,ta0) + trapz(tk0,rf0)) / stride0;
baselineCMAPD_whole  = ( ...
    trapz(tk0,g0) + trapz(tk0,s0) + trapz(tk0,ta0) + trapz(tk0,rf0) + ...
    trapz(tk0,hs0) + trapz(tk0,bfs0) + trapz(tk0,gm0) + trapz(tk0,il0) + trapz(tk0,va0) ) / stride0;

baselineSpeed = stride0 / (tk0(end) - tk0(1));
baselineElapsed = tk0(end) - tk0(1);
baselineStride = stride0;
baselineApWork = trapz(tk0, max(vx0,0) .* vPel0);

%% ===== metrics per output + gradient colors =====
nOut = numel(All);
baseColors = lines(nOut);           % output별 기준색
minMix = 0.25;                      % 1번 iter의 "연함" 정도 (0~1, 클수록 더 하얘짐)
ms = 30;                            % marker size
optimalForce = 300;                 % AFO(PathActuator)의 optimal force

for o = 1:nOut
    iterNum = All(o).iterNum;

    CMAPD_GS     = nan(iterNum,1);
    CMAPD_Shank  = nan(iterNum,1);
    CMAPD_4set   = nan(iterNum,1);
    CMAPD_whole  = nan(iterNum,1);

    Speed      = nan(iterNum,1);
    deltaProp  = nan(iterNum,1);
    Prop       = nan(iterNum,1);
    integralF  = nan(iterNum,1);

    elapsedTime  = nan(iterNum,1);
    strideLength  = nan(iterNum,1);
    peakApGRF    = nan(iterNum,1);
    dP_over_dist = nan(iterNum,1);
    dP_over_time = nan(iterNum,1);
    apWorkFromGRF = nan(iterNum,1);
    posWorkFromAFO = nan(iterNum,1);

    % iter 그라데이션 색 (white -> baseColor)
    a = linspace(minMix, 1, iterNum)';          % 1: 연함, end: 진함
    iterColor = (1-a)*[1 1 1] + a*baseColors(o,:);  % [iterNum x 3]

    for i = 1:iterNum
        % propulsion
        t  = All(o).iter(i).grf.t(:);
        vx = All(o).iter(i).grf.vx(:);
        prop = trapz(t, max(vx,0));
        Prop(i) = prop;
        deltaProp(i) = prop - baselineProp;

        % CMAPD
        dist = pelv(end) - pelv(1);
        tk   = All(o).iter(i).kin.t(:);
        pelv = All(o).iter(i).kin.pelvisTx(:);

        gAct = All(o).iter(i).kin.gastrocAct(:);
        sAct = All(o).iter(i).kin.soleusAct(:);
        taAct  = All(o).iter(i).kin.tibAntAct(:);
        rfAct  = All(o).iter(i).kin.rectFemAct(:);

        hsAct  = All(o).iter(i).kin.hamStrAct(:);
        bfsAct = All(o).iter(i).kin.biFemshAct(:);
        gmAct  = All(o).iter(i).kin.glutMaxAct(:);
        ilAct  = All(o).iter(i).kin.iliopsoasAct(:);
        vaAct  = All(o).iter(i).kin.vastiAct(:);

        CMAPD_GS(i)    = (trapz(tk,gAct) + trapz(tk,sAct)) / dist;
        CMAPD_Shank(i) = (trapz(tk,gAct) + trapz(tk,sAct) + trapz(tk,taAct)) / dist;
        CMAPD_4set(i)  = (trapz(tk,gAct) + trapz(tk,sAct) + trapz(tk,taAct) + trapz(tk,rfAct)) / dist;
        CMAPD_whole(i) = ( ...
            trapz(tk,gAct) + trapz(tk,sAct) + trapz(tk,taAct) + trapz(tk,rfAct) + ...
            trapz(tk,hsAct) + trapz(tk,bfsAct) + trapz(tk,gmAct) + trapz(tk,ilAct) + trapz(tk,vaAct) ) / dist;

        % speed
        Speed(i) = All(o).iter(i).avgSpeed;

        % integral(F) 
        tc = All(o).iter(i).ctrl.t(:);
        u  = All(o).iter(i).ctrl.u(:);
        integralF(i) = trapz(tc, u * optimalForce);

        % elapsed time, walking distance
        elapsedTime(i) = tk(end) - tk(1);
        strideLength(i) = dist;

        % peak apGRF
        peakApGRF(i) = max(vx);

        % normalized delta propulsion
        dP_over_dist(i) = deltaProp(i) / dist;
        dP_over_time(i) = deltaProp(i) / elapsedTime(i);

        % apWorkFromGRF = ∫(GRF_ap * max(v,0)) dt
        vPel = All(o).iter(i).kin.pelvisTxSpeed(:);
        apWorkFromGRF(i) = trapz(t, max(vx,0) .* vPel);

        % PosWorkFromAFO = ∫(F * max(w,0)) dt
        w  = - All(o).iter(i).analy.w(:); % OpenSim에선 Plantar 방향이 음수이므로 마이너스 추가 필요
        F   = u * optimalForce;
        posWorkFromAFO(i) = trapz(tc, F .* max(w,0));
    end

    % Save metrics
    All(o).metric.CMAPD_GS    = CMAPD_GS;
    All(o).metric.CMAPD_Shank = CMAPD_Shank;
    All(o).metric.CMAPD_4set  = CMAPD_4set;
    All(o).metric.CMAPD_whole = CMAPD_whole;

    All(o).metric.Speed = Speed;
    All(o).metric.deltaProp = deltaProp;
    All(o).metric.Prop = Prop;
    All(o).metric.integralF  = integralF;
    All(o).metric.color = iterColor;

    All(o).metric.elapsedTime  = elapsedTime;
    All(o).metric.strideLength  = strideLength;
    All(o).metric.peakApGRF    = peakApGRF;
    All(o).metric.dP_over_dist = dP_over_dist;
    All(o).metric.dP_over_time = dP_over_time;
    All(o).metric.apWorkFromGRF = apWorkFromGRF;
    All(o).metric.posWorkFromAFO = posWorkFromAFO;
end


%% ===== cluster plots =====

% legend용 더미 핸들
dummy = gobjects(nOut,1);

% 점을 몇 개씩 찍을지
sampleStep = 10;   % 10개 중 1개만 표시



% 1) CMAPD_GS vs Speed
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    idx = 1:sampleStep:All(o).iterNum;
    scatter(All(o).metric.CMAPD_GS(idx), All(o).metric.Speed(idx), ms, All(o).metric.color(idx,:), 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
hBase = scatter(baselineCMAPD_GS, baselineSpeed, 1000, 'k', 'filled', 'Marker', 'p');
xlabel('CMAPD'); ylabel('Gait speed (m/s)');
title('CMAPD(GS) vs Gait speed');
set(gca,'FontSize',25);
lg = {All.name}; lg{end+1} = 'baseline';
legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'CMAPD_GS_Speed.png'), 'Resolution', 300);



% 2) CMAPD_GS vs delta(Propulsion)
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    idx = 1:sampleStep:All(o).iterNum;
    scatter(All(o).metric.CMAPD_GS(idx), All(o).metric.deltaProp(idx), ms, All(o).metric.color(idx,:), 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
hBase = scatter(baselineCMAPD_GS, 0, 1000, 'k', 'filled', 'Marker', 'p');
xlabel('CMAPD'); ylabel('\Delta Propulsion (N·s)');
title('CMAPD(GS) vs \Delta Propulsion');
set(gca,'FontSize',25);
lg = {All.name}; lg{end+1} = 'baseline';
legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'CMAPD_GS_Propulsion.png'), 'Resolution', 300);



% 3) Speed vs delta(Propulsion)
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    idx = 1:sampleStep:All(o).iterNum;
    scatter(All(o).metric.Speed(idx), All(o).metric.deltaProp(idx), ms, All(o).metric.color(idx,:), 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
hBase = scatter(baselineSpeed, 0, 1000, 'k', 'filled', 'Marker', 'p');
xlabel('Gait speed (m/s)'); ylabel('\Delta Propulsion (N·s)');
title('Gait speed vs \Delta Propulsion');
set(gca,'FontSize',25);
lg = {All.name}; lg{end+1} = 'baseline';
legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'Speed_Propulsion.png'), 'Resolution', 300);



% 4) CMAPD_GS vs integral(F)
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    idx = 1:sampleStep:All(o).iterNum;
    scatter(All(o).metric.CMAPD_GS(idx), All(o).metric.integralF(idx), ms, All(o).metric.color(idx,:), 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
xlabel('CMAPD'); ylabel('Integral(F) (N·s)');
title('CMAPD(GS) vs Integral(F)');
set(gca,'FontSize',25);
lg = {All.name};
legend(dummy, lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'CMAPD_GS_inteF.png'), 'Resolution', 300);



% 5) elapsedTime vs strideLength
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    idx = 1:sampleStep:All(o).iterNum;
    scatter(All(o).metric.elapsedTime(idx), All(o).metric.strideLength(idx), ms, All(o).metric.color(idx,:), 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
hBase = scatter(baselineElapsed, baselineStride, 1000, 'k', 'filled', 'Marker', 'p');
xlabel('Elapsed time (s)'); ylabel('Stride length (m)');
title('Elapsed time vs Stride length');
set(gca,'FontSize',25);
lg = {All.name}; lg{end+1} = 'baseline';
legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'Elapsed_Stride.png'), 'Resolution', 300);



% 6) delta(propulsion) vs integral(F)
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    idx = 1:sampleStep:All(o).iterNum;
    scatter(All(o).metric.deltaProp(idx), All(o).metric.integralF(idx), ms, All(o).metric.color(idx,:), 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
xlabel('\Delta Propulsion (N·s)'); ylabel('Integral(F) (N·s)');
title('\Delta Propulsion vs Integral(F)');
set(gca,'FontSize',25);
lg = {All.name};
legend(dummy, lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'dProp_inteF.png'), 'Resolution', 300);



% 7) Speed vs dP_over_dist
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    idx = 1:sampleStep:All(o).iterNum;
    scatter(All(o).metric.Speed(idx), All(o).metric.dP_over_dist(idx), ms, All(o).metric.color(idx,:), 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
xlabel('Gait speed (m/s)'); ylabel('\Delta Propulsion / distance (N·s/m)');
title('Gait speed vs \Delta Propulsion / distance');
set(gca,'FontSize',25);
lg = {All.name};
legend(dummy, lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'Speed_dP_over_dist.png'), 'Resolution', 300);



% 8) Speed vs dP_over_time
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    idx = 1:sampleStep:All(o).iterNum;
    scatter(All(o).metric.Speed(idx), All(o).metric.dP_over_time(idx), ms, All(o).metric.color(idx,:), 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
xlabel('Gait speed (m/s)'); ylabel('\Delta Propulsion / time (N)');
title('Gait speed vs \Delta Propulsion / time');
set(gca,'FontSize',25);
lg = {All.name};
legend(dummy, lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'Speed_dP_over_time.png'), 'Resolution', 300);



% 9) CMAPD_GS vs apWork
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    idx = 1:sampleStep:All(o).iterNum;
    scatter(All(o).metric.CMAPD_GS(idx), All(o).metric.apWorkFromGRF(idx), ms, All(o).metric.color(idx,:), 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
hBase = scatter(baselineCMAPD_GS, baselineApWork, 1000, 'k', 'filled', 'Marker', 'p');
xlabel('CMAPD'); ylabel('Work (J)');
title('CMAPD(GS) vs Positive AP Work');
set(gca,'FontSize',25);
lg = {All.name}; lg{end+1} = 'baseline';
legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'CMAPD_GS_apWork.png'), 'Resolution', 300);


% 10) Speed vs peakApGRF
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    idx = 1:sampleStep:All(o).iterNum;
    scatter(All(o).metric.Speed(idx), All(o).metric.peakApGRF(idx), ms, All(o).metric.color(idx,:), 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
hBase = scatter(baselineSpeed, baselinePeakAp, 1000, 'k', 'filled', 'Marker', 'p');
xlabel('Gait speed (m/s)'); ylabel('Peak apGRF (N)');
title('Gait speed vs Peak apGRF');
set(gca,'FontSize',25);
lg = {All.name}; lg{end+1} = 'baseline';
legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'Speed_peakAp.png'), 'Resolution', 300);

%% 추가

% 11) apWork vs Speed
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    idx = 1:sampleStep:All(o).iterNum;
    scatter(All(o).metric.apWorkFromGRF(idx), All(o).metric.Speed(idx), ms, All(o).metric.color(idx,:), 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
hBase = scatter(baselineApWork, baselineSpeed, 1000, 'k', 'filled', 'Marker', 'p');
xlabel('apWork (J)'); ylabel('Gait speed (m/s)');
title('apWork vs Gait speed');
set(gca,'FontSize',25);
lg = {All.name}; lg{end+1} = 'baseline';
legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'apWork_Speed.png'), 'Resolution', 300);


% 12) apWork vs peakApGRF
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    idx = 1:sampleStep:All(o).iterNum;
    scatter(All(o).metric.apWorkFromGRF(idx), All(o).metric.peakApGRF(idx), ms, All(o).metric.color(idx,:), 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
hBase = scatter(baselineApWork, baselinePeakAp, 1000, 'k', 'filled', 'Marker', 'p');
xlabel('apWork (J)'); ylabel('Peak apGRF (N)');
title('apWork vs Peak apGRF');
set(gca,'FontSize',25);
lg = {All.name}; lg{end+1} = 'baseline';
legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'apWork_peakApGRF.png'), 'Resolution', 300);


% 13) apWork vs delta(Propulsion)
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    idx = 1:sampleStep:All(o).iterNum;
    scatter(All(o).metric.apWorkFromGRF(idx), All(o).metric.deltaProp(idx), ms, All(o).metric.color(idx,:), 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
hBase = scatter(baselineApWork, 0, 1000, 'k', 'filled', 'Marker', 'p');
xlabel('apWork (J)'); ylabel('\Delta Propulsion (N·s)');
title('apWork vs \Delta Propulsion');
set(gca,'FontSize',25);
lg = {All.name}; lg{end+1} = 'baseline';
legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'apWork_dProp.png'), 'Resolution', 300);


% 14) apWork vs integral(F)
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    idx = 1:sampleStep:All(o).iterNum;
    scatter(All(o).metric.apWorkFromGRF(idx), All(o).metric.integralF(idx), ms, All(o).metric.color(idx,:), 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
xlabel('apWork (J)'); ylabel('Integral(F) (N·s)');
title('apWork vs Integral(F)');
set(gca,'FontSize',25);
lg = {All.name};
legend(dummy, lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'apWork_inteF.png'), 'Resolution', 300);


% 15) apWork vs CMAPD_Shank
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    idx = 1:sampleStep:All(o).iterNum;
    scatter(All(o).metric.apWorkFromGRF(idx), All(o).metric.CMAPD_Shank(idx), ms, All(o).metric.color(idx,:), 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
hBase = scatter(baselineApWork, baselineCMAPD_Shank, 1000, 'k', 'filled', 'Marker', 'p');
xlabel('apWork (J)'); ylabel('CMAPD\_Shank');
title('apWork vs CMAPD\_Shank');
set(gca,'FontSize',25);
lg = {All.name}; lg{end+1} = 'baseline';
legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'apWork_CMAPD_Shank.png'), 'Resolution', 300);


% 16) apWork vs CMAPD_4set
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    idx = 1:sampleStep:All(o).iterNum;
    scatter(All(o).metric.apWorkFromGRF(idx), All(o).metric.CMAPD_4set(idx), ms, All(o).metric.color(idx,:), 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
hBase = scatter(baselineApWork, baselineCMAPD_4set, 1000, 'k', 'filled', 'Marker', 'p');
xlabel('apWork (J)'); ylabel('CMAPD\_4set');
title('apWork vs CMAPD\_4set');
set(gca,'FontSize',25);
lg = {All.name}; lg{end+1} = 'baseline';
legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'apWork_CMAPD_4set.png'), 'Resolution', 300);


% 17) apWork vs CMAPD_whole
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    idx = 1:sampleStep:All(o).iterNum;
    scatter(All(o).metric.apWorkFromGRF(idx), All(o).metric.CMAPD_whole(idx), ms, All(o).metric.color(idx,:), 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
hBase = scatter(baselineApWork, baselineCMAPD_whole, 1000, 'k', 'filled', 'Marker', 'p');
xlabel('apWork (J)'); ylabel('CMAPD\_whole');
title('apWork vs CMAPD\_whole');
set(gca,'FontSize',25);
lg = {All.name}; lg{end+1} = 'baseline';
legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'apWork_CMAPD_whole.png'), 'Resolution', 300);


% 18) PosWork vs Speed
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    idx = 1:sampleStep:All(o).iterNum;
    scatter(All(o).metric.posWorkFromAFO(idx), All(o).metric.Speed(idx), ms, All(o).metric.color(idx,:), 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
xlabel('PosWork (arb)'); ylabel('Gait speed (m/s)');
title('PosWork vs Gait speed');
set(gca,'FontSize',25);
lg = {All.name}; lg{end+1} = 'baseline';
legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'PosWork_Speed.png'), 'Resolution', 300);


% 19) PosWork vs peakApGRF
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    idx = 1:sampleStep:All(o).iterNum;
    scatter(All(o).metric.posWorkFromAFO(idx), All(o).metric.peakApGRF(idx), ms, All(o).metric.color(idx,:), 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
xlabel('PosWork (arb)'); ylabel('Peak apGRF (N)');
title('PosWork vs Peak apGRF');
set(gca,'FontSize',25);
lg = {All.name}; lg{end+1} = 'baseline';
legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'PosWork_peakApGRF.png'), 'Resolution', 300);


% 20) PosWork vs delta(Propulsion)
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    idx = 1:sampleStep:All(o).iterNum;
    scatter(All(o).metric.posWorkFromAFO(idx), All(o).metric.deltaProp(idx), ms, All(o).metric.color(idx,:), 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
xlabel('PosWork (arb)'); ylabel('\Delta Propulsion (N·s)');
title('PosWork vs \Delta Propulsion');
set(gca,'FontSize',25);
lg = {All.name}; lg{end+1} = 'baseline';
legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'PosWork_dProp.png'), 'Resolution', 300);


% 21) PosWork vs integral(F)
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    idx = 1:sampleStep:All(o).iterNum;
    scatter(All(o).metric.posWorkFromAFO(idx), All(o).metric.integralF(idx), ms, All(o).metric.color(idx,:), 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
xlabel('PosWork (arb)'); ylabel('Integral(F) (N·s)');
title('PosWork vs Integral(F)');
set(gca,'FontSize',25);
lg = {All.name};
legend(dummy, lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'PosWork_inteF.png'), 'Resolution', 300);


% 22) PosWork vs CMAPD_Shank
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    idx = 1:sampleStep:All(o).iterNum;
    scatter(All(o).metric.posWorkFromAFO(idx), All(o).metric.CMAPD_Shank(idx), ms, All(o).metric.color(idx,:), 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
xlabel('PosWork (arb)'); ylabel('CMAPD\_Shank');
title('PosWork vs CMAPD\_Shank');
set(gca,'FontSize',25);
lg = {All.name}; lg{end+1} = 'baseline';
legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'PosWork_CMAPD_Shank.png'), 'Resolution', 300);


% 23) PosWork vs CMAPD_4set
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    idx = 1:sampleStep:All(o).iterNum;
    scatter(All(o).metric.posWorkFromAFO(idx), All(o).metric.CMAPD_4set(idx), ms, All(o).metric.color(idx,:), 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
xlabel('PosWork (arb)'); ylabel('CMAPD\_4set');
title('PosWork vs CMAPD\_4set');
set(gca,'FontSize',25);
lg = {All.name}; lg{end+1} = 'baseline';
legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'PosWork_CMAPD_4set.png'), 'Resolution', 300);


% 24) PosWork vs CMAPD_whole
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    idx = 1:sampleStep:All(o).iterNum;
    scatter(All(o).metric.posWorkFromAFO(idx), All(o).metric.CMAPD_whole(idx), ms, All(o).metric.color(idx,:), 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
xlabel('PosWork (arb)'); ylabel('CMAPD\_whole');
title('PosWork vs CMAPD\_whole');
set(gca,'FontSize',25);
lg = {All.name}; lg{end+1} = 'baseline';
legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'PosWork_CMAPD_whole.png'), 'Resolution', 300);