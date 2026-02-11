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

outs(4).name    = 'et_a05b0_iter300';
outs(4).iterNum = 300;

outs(5).name    = 'et_a1b0_iter300';
outs(5).iterNum = 300;

%% ====== field name 정의 ======
pelvisField  = matlab.lang.makeValidName('/jointset/groundPelvis/pelvis_tx/value');
gastrocField = matlab.lang.makeValidName('/gastroc_r/activation');
soleusField  = matlab.lang.makeValidName('/soleus_r/activation');

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
        idxGa = find(strcmp(fn, gastrocField),1);
        idxSa = find(strcmp(fn, soleusField),1);

        tKin = data(:,idxT);
        pelv = data(:,idxPx);
        gAct = data(:,idxGa);
        sAct = data(:,idxSa);

        avgSpeed = (pelv(end) - pelv(1)) / (tKin(end) - tKin(1));

        All(o).iter(i).kin.t          = tKin;
        All(o).iter(i).kin.pelvisTx   = pelv;
        All(o).iter(i).kin.gastrocAct = gAct;
        All(o).iter(i).kin.soleusAct  = sAct;

        All(o).iter(i).avgSpeed = avgSpeed;

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
g0    = data(:, strcmp(fn, gastrocField));
s0    = data(:, strcmp(fn, soleusField));

dist0 = pelv0(end) - pelv0(1);
baselineCMAPD = (trapz(tk0,g0) + trapz(tk0,s0)) / dist0;
baselineSpeed = dist0 / (tk0(end) - tk0(1));

baselineElapsed = tk0(end) - tk0(1);
baselineWalkingDist = dist0;


%% ===== metrics per output + gradient colors =====
nOut = numel(All);
baseColors = lines(nOut);           % output별 기준색
minMix = 0.25;                      % 1번 iter의 "연함" 정도 (0~1, 클수록 더 하얘짐)
ms = 30;                            % marker size
optimalForce = 300;                 % AFO(PathActuator)의 optimal force

for o = 1:nOut

    iterNum = All(o).iterNum;

    CMAPD_sol  = nan(iterNum,1);
    CMAPD_gast = nan(iterNum,1);
    CMAPD_tot  = nan(iterNum,1);
    Speed      = nan(iterNum,1);
    deltaProp  = nan(iterNum,1);
    integralF  = nan(iterNum,1);

    elapsedTime  = nan(iterNum,1);
    walkingDist  = nan(iterNum,1);
    peakApGRF    = nan(iterNum,1);
    dP_over_dist = nan(iterNum,1);
    dP_over_time = nan(iterNum,1);

    % iter 그라데이션 색 (white -> baseColor)
    a = linspace(minMix, 1, iterNum)';          % 1: 연함, end: 진함
    iterColor = (1-a)*[1 1 1] + a*baseColors(o,:);  % [iterNum x 3]

    for i = 1:iterNum

        % propulsion
        t  = All(o).iter(i).grf.t(:);
        vx = All(o).iter(i).grf.vx(:);
        prop = trapz(t, max(vx,0));
        deltaProp(i) = prop - baselineProp;

        % CMAPD
        tk   = All(o).iter(i).kin.t(:);
        pelv = All(o).iter(i).kin.pelvisTx(:);
        gAct = All(o).iter(i).kin.gastrocAct(:);
        sAct = All(o).iter(i).kin.soleusAct(:);

        dist = pelv(end) - pelv(1);
        CMAPD_sol(i) = trapz(tk, sAct) / dist;
        CMAPD_gast(i) = trapz(tk, gAct) / dist;
        CMAPD_tot(i) = (trapz(tk, gAct) + trapz(tk, sAct)) / dist;

        % speed
        Speed(i) = All(o).iter(i).avgSpeed;

        % integral(F) 
        tc = All(o).iter(i).ctrl.t(:);
        u  = All(o).iter(i).ctrl.u(:);
        integralF(i) = trapz(tc, u * optimalForce);

        % elapsed time, walking distance
        elapsedTime(i) = tk(end) - tk(1);
        walkingDist(i) = dist;

        % peak apGRF
        peakApGRF(i) = max(vx);

        % normalized delta propulsion
        dP_over_dist(i) = deltaProp(i) / dist;
        dP_over_time(i) = deltaProp(i) / elapsedTime(i);
    end

    All(o).metric.CMAPD_sol = CMAPD_sol;
    All(o).metric.CMAPD_gast = CMAPD_gast;
    All(o).metric.CMAPD_tot = CMAPD_tot;
    All(o).metric.Speed = Speed;
    All(o).metric.deltaProp = deltaProp;
    All(o).metric.integralF  = integralF;
    All(o).metric.color = iterColor;

    All(o).metric.elapsedTime  = elapsedTime;
    All(o).metric.walkingDist  = walkingDist;
    All(o).metric.peakApGRF    = peakApGRF;
    All(o).metric.dP_over_dist = dP_over_dist;
    All(o).metric.dP_over_time = dP_over_time;
end


%% ===== cluster plots =====

FigureFolder = fullfile(baseFolder,'\analysis_fig');
if ~exist(FigureFolder, 'dir')
    mkdir(FigureFolder);
end

% legend용 더미 핸들
dummy = gobjects(nOut,1);

% 1) CMAPD vs Speed
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    scatter(All(o).metric.CMAPD_tot, All(o).metric.Speed, ms, All(o).metric.color, 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
hBase = scatter(baselineCMAPD, baselineSpeed, 1000, 'k', 'filled', 'Marker', 'p');
xlabel('CMAPD'); ylabel('Gait speed (m/s)');
title('CMAPD vs Gait speed');
set(gca,'FontSize',25);
lg = {All.name};
lg{end+1} = 'baseline';
legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'CMAPD_Speed.png'), 'Resolution', 300);


% 2) CMAPD vs delta(Propulsion)
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    scatter(All(o).metric.CMAPD_tot, All(o).metric.deltaProp, ms, All(o).metric.color, 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
hBase = scatter(baselineCMAPD, 0, 1000, 'k', 'filled', 'Marker', 'p');
xlabel('CMAPD'); ylabel('\Delta Propulsion (N·s)');
title('CMAPD vs \Delta Propulsion');
set(gca,'FontSize',25);
lg = {All.name};
lg{end+1} = 'baseline';
legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'CMAPD_Propulsion.png'), 'Resolution', 300);


% 3) Speed vs delta(Propulsion)
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    scatter(All(o).metric.Speed, All(o).metric.deltaProp, ms, All(o).metric.color, 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
hBase = scatter(baselineSpeed, 0, 1000, 'k', 'filled', 'Marker', 'p');
xlabel('Gait speed (m/s)'); ylabel('\Delta Propulsion (N·s)');
title('Gait speed vs \Delta Propulsion');
set(gca,'FontSize',25);
lg = {All.name};
lg{end+1} = 'baseline';
legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'Speed_Propulsion.png'), 'Resolution', 300);


% 4) integral(F) vs CMAPD
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    scatter(All(o).metric.CMAPD_tot, All(o).metric.integralF, ms, All(o).metric.color, 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
xlabel('CMAPD'); ylabel('Integral(F) (N·s)'); 
title('CMAPD vs Integral(F)');
set(gca,'FontSize',25);
lg = {All.name};
legend([dummy;], lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'CMAPD_inteF.png'), 'Resolution', 300);


% 5) elapsedTime vs walkingDist
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    scatter(All(o).metric.elapsedTime, All(o).metric.walkingDist, ms, All(o).metric.color, 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
hBase = scatter(baselineElapsed, baselineWalkingDist, 1000, 'k', 'filled', 'Marker', 'p');
xlabel('Elapsed time (s)'); ylabel('Walking distance (m)');
title('Elapsed time vs Walking distance');
set(gca,'FontSize',25);
lg = {All.name}; lg{end+1} = 'baseline';
legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'Elapsed_WalkingDist.png'), 'Resolution', 300);


% 6) delta(propulsion) vs integral(F)
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    scatter(All(o).metric.deltaProp, All(o).metric.integralF, ms, All(o).metric.color, 'filled');
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
    scatter(All(o).metric.Speed, All(o).metric.dP_over_dist, ms, All(o).metric.color, 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
xlabel('Gait speed (m/s)'); ylabel('\Delta Propulsion / distance (N·s/m)');
title('Gait speed vs \Delta Propulsion / distance');
set(gca,'FontSize',25);
lg = {All.name};
legend(dummy, lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'Speed_dP_over_dist.png'), 'Resolution', 300);


% 8) Speed vs dP_over_dist
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    scatter(All(o).metric.Speed, All(o).metric.dP_over_time, ms, All(o).metric.color, 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
xlabel('Gait speed (m/s)'); ylabel('\Delta Propulsion / time (N)');
title('Gait speed vs \Delta Propulsion / time');
set(gca,'FontSize',25);
lg = {All.name};
legend(dummy, lg, 'Location','best', 'Interpreter','none');
exportgraphics(gcf, fullfile(FigureFolder, 'Speed_dP_over_time.png'), 'Resolution', 300);