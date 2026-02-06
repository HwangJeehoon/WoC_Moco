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

outs(1).name    = 'et_a01b0_iter300';
outs(1).iterNum = 300;

outs(2).name    = 'et_a05b0_iter100';
outs(2).iterNum = 100;

% outs(3).name    = '...';
% outs(3).iterNum = ...;

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

%% ===== metrics per output =====
for o = 1:numel(All)

    iterNum = All(o).iterNum;

    CMAPD_sol = nan(iterNum,1);
    CMAPD_gast = nan(iterNum,1);
    CMAPD_tot = nan(iterNum,1);
    Speed = nan(iterNum,1);
    deltaProp = nan(iterNum,1);

    for i = 1:iterNum

        % --- propulsion ---
        t  = All(o).iter(i).grf.t(:);
        vx = All(o).iter(i).grf.vx(:);
        prop = trapz(t, max(vx,0));

        deltaProp(i) = prop - baselineProp;

        % --- CMAPD ---
        tk   = All(o).iter(i).kin.t(:);
        pelv = All(o).iter(i).kin.pelvisTx(:);
        gAct = All(o).iter(i).kin.gastrocAct(:);
        sAct = All(o).iter(i).kin.soleusAct(:);

        dist = pelv(end) - pelv(1);
        CMAPD_sol(i) = trapz(tk, sAct) / dist;
        CMAPD_gast(i) = trapz(tk, gAct) / dist;
        CMAPD_tot(i) = (trapz(tk, gAct) + trapz(tk, sAct)) / dist;

        % --- speed ---
        Speed(i) = All(o).iter(i).avgSpeed;
    end

    All(o).metric.CMAPD_sol = CMAPD_sol;
    All(o).metric.CMAPD_gast = CMAPD_gast;
    All(o).metric.CMAPD_tot = CMAPD_tot;
    All(o).metric.Speed = Speed;
    All(o).metric.deltaProp = deltaProp;
end


%% ===== metrics per output + gradient colors =====
nOut = numel(All);
baseColors = lines(nOut);          % output별 기준색
minMix = 0.25;                      % 1번 iter의 "연함" 정도 (0~1, 클수록 더 하얘짐)
ms = 30;                            % marker size

for o = 1:nOut

    iterNum = All(o).iterNum;

    CMAPD_sol = nan(iterNum,1);
    CMAPD_gast = nan(iterNum,1);
    CMAPD_tot = nan(iterNum,1);
    Speed = nan(iterNum,1);
    deltaProp(i) = prop - baselineProp;

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
    end

    All(o).metric.CMAPD_sol = CMAPD_sol;
    All(o).metric.CMAPD_gast = CMAPD_gast;
    All(o).metric.CMAPD_tot = CMAPD_tot;
    All(o).metric.Speed = Speed;
    All(o).metric.deltaProp = deltaProp;
    All(o).metric.color = iterColor;
end


%% ===== cluster plots =====

% legend용 더미 핸들
dummy = gobjects(nOut,1);

% 1) CMAPD vs Speed
figure('Color','w','Position',[0 0 1200 800]);
hold on; box on;
for o = 1:nOut
    x = All(o).metric.CMAPD_tot;
    y = All(o).metric.Speed;
    c = All(o).metric.color;
    scatter(x, y, ms, c, 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:),'MarkerEdgeColor',baseColors(o,:));
end
scatter(baselineCMAPD, baselineSpeed, 120, 'k', 'filled', 'Marker', 'p'); % baseline
plot(nan,nan,'kp','MarkerFaceColor','k','MarkerEdgeColor','k');          % legend용
xlabel('CMAPD'); ylabel('Gait speed (m/s)');
title('CMAPD vs Gait speed');
set(gca,'FontSize',18);
legend([dummy; gca().Children(1)], [string({All.name}) "baseline"], 'Location','best', 'Interpreter','none');


% 2) CMAPD vs delta(Propulsion)
figure('Color','w','Position',[0 0 1200 800]);
hold on; box on;
for o = 1:nOut
    x = All(o).metric.CMAPD_tot;
    y = All(o).metric.deltaProp;
    c = All(o).metric.color;
    scatter(x, y, ms, c, 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:),'MarkerEdgeColor',baseColors(o,:));
end
scatter(baselineCMAPD, 0, 120, 'k', 'filled', 'Marker', 'p');
plot(nan,nan,'kp','MarkerFaceColor','k','MarkerEdgeColor','k');
xlabel('CMAPD'); ylabel('\Delta Propulsion (N·s)');
title('CMAPD vs \Delta Propulsion');
set(gca,'FontSize',18);
legend([dummy; gca().Children(1)], [string({All.name}) "baseline"], 'Location','best', 'Interpreter','none');


% 3) Speed vs delta(Propulsion)
figure('Color','w','Position',[0 0 1200 800]);
hold on; box on;
for o = 1:nOut
    x = All(o).metric.Speed;
    y = All(o).metric.deltaProp;
    c = All(o).metric.color;
    scatter(x, y, ms, c, 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:),'MarkerEdgeColor',baseColors(o,:));
end
scatter(baselineSpeed, 0, 120, 'k', 'filled', 'Marker', 'p');
plot(nan,nan,'kp','MarkerFaceColor','k','MarkerEdgeColor','k');
xlabel('Gait speed (m/s)'); ylabel('\Delta Propulsion (N·s)');
title('Gait speed vs \Delta Propulsion');
set(gca,'FontSize',18);
legend([dummy; gca().Children(1)], [string({All.name}) "baseline"], 'Location','best', 'Interpreter','none');
