clc; clear; close all;

%% baseFolder
if isempty(mfilename)
    thisFile = matlab.desktop.editor.getActiveFilename;
else
    thisFile = mfilename("fullpath");
end
baseFolder = fileparts(thisFile);
resultFolder = fullfile(baseFolder, '..', 'results');

%% ====== 분석하고 싶은 output 설정 ======
% AFO force 설정
AFO_Force_50BW = 300;
AFO_Force_40BW = 243;
AFO_Force_30BW = 182;
AFO_Force_20BW = 121;

% % 같은 힘 레벨에서 파라미터 간 분석
outs = struct([]);
outs(1).name    = 'et_a001b0_iter300_50BW';
outs(1).iterNum = 300;
outs(1).optimalForce = AFO_Force_50BW;
outs(2).name    = 'et_a005b0_iter300_50BW';
outs(2).iterNum = 300;
outs(2).optimalForce = AFO_Force_50BW;
outs(3).name    = 'et_a01b0_iter300_50BW';
outs(3).iterNum = 300;
outs(3).optimalForce = AFO_Force_50BW;
outs(4).name    = 'et_a03b0_iter300_50BW';
outs(4).iterNum = 300;
outs(4).optimalForce = AFO_Force_50BW;
outs(5).name    = 'et_a05b0_iter300_50BW';
outs(5).iterNum = 300;
outs(5).optimalForce = AFO_Force_50BW;
outs(6).name    = 'et_a1b0_iter300_50BW';
outs(6).iterNum = 300;
outs(6).optimalForce = AFO_Force_50BW;
FigureFolder = fullfile(resultFolder,'\analysis_fig\50BW');
if ~exist(FigureFolder, 'dir')
    mkdir(FigureFolder);
end

% outs = struct([]);
% outs(1).name    = 'et_a001b0_iter300_40BW';
% outs(1).iterNum = 300;
% outs(1).optimalForce = AFO_Force_40BW;
% outs(2).name    = 'et_a005b0_iter300_40BW';
% outs(2).iterNum = 300;
% outs(2).optimalForce = AFO_Force_40BW;
% outs(3).name    = 'et_a01b0_iter300_40BW';
% outs(3).iterNum = 300;
% outs(3).optimalForce = AFO_Force_40BW;
% outs(4).name    = 'et_a03b0_iter300_40BW';
% outs(4).iterNum = 300;
% outs(4).optimalForce = AFO_Force_40BW;
% outs(5).name    = 'et_a05b0_iter300_40BW';
% outs(5).iterNum = 300;
% outs(5).optimalForce = AFO_Force_40BW;
% FigureFolder = fullfile(resultFolder,'\analysis_fig\40BW');
% if ~exist(FigureFolder, 'dir')
%     mkdir(FigureFolder);
% end

% outs = struct([]);
% outs(1).name    = 'et_a001b0_iter300_30BW';
% outs(1).iterNum = 300;
% outs(1).optimalForce = AFO_Force_30BW;
% outs(2).name    = 'et_a005b0_iter300_30BW';
% outs(2).iterNum = 300;
% outs(2).optimalForce = AFO_Force_30BW;
% outs(3).name    = 'et_a01b0_iter300_30BW';
% outs(3).iterNum = 300;
% outs(3).optimalForce = AFO_Force_30BW;
% outs(4).name    = 'et_a03b0_iter300_30BW';
% outs(4).iterNum = 300;
% outs(4).optimalForce = AFO_Force_30BW;
% outs(5).name    = 'et_a05b0_iter300_30BW';
% outs(5).iterNum = 300;
% outs(5).optimalForce = AFO_Force_30BW;
% FigureFolder = fullfile(resultFolder,'\analysis_fig\30BW');
% if ~exist(FigureFolder, 'dir')
%     mkdir(FigureFolder);
% end


% outs = struct([]);
% outs(1).name    = 'et_a001b0_iter300_20BW';
% outs(1).iterNum = 300;
% outs(1).optimalForce = AFO_Force_20BW;
% outs(2).name    = 'et_a005b0_iter300_20BW';
% outs(2).iterNum = 300;
% outs(2).optimalForce = AFO_Force_20BW;
% outs(3).name    = 'et_a01b0_iter300_20BW';
% outs(3).iterNum = 300;
% outs(3).optimalForce = AFO_Force_20BW;
% outs(4).name    = 'et_a03b0_iter300_20BW';
% outs(4).iterNum = 300;
% outs(4).optimalForce = AFO_Force_20BW;
% outs(5).name    = 'et_a05b0_iter300_20BW';
% outs(5).iterNum = 300;
% outs(5).optimalForce = AFO_Force_20BW;
% FigureFolder = fullfile(resultFolder,'\analysis_fig\20BW');
% if ~exist(FigureFolder, 'dir')
%     mkdir(FigureFolder);
% end

% % 같은 파라미터에서 힘 레벨 간 분석
outs = struct([]);
outs(1).name    = 'et_a001b0_iter300_50BW';
outs(1).iterNum = 300;
outs(1).optimalForce = AFO_Force_50BW;
outs(2).name    = 'et_a001b0_iter300_40BW';
outs(2).iterNum = 300;
outs(2).optimalForce = AFO_Force_40BW;
outs(3).name    = 'et_a001b0_iter300_30BW';
outs(3).iterNum = 300;
outs(3).optimalForce = AFO_Force_30BW;
outs(4).name    = 'et_a001b0_iter300_20BW';
outs(4).iterNum = 300;
outs(4).optimalForce = AFO_Force_20BW;
FigureFolder = fullfile(resultFolder,'\analysis_fig\a001b0');
if ~exist(FigureFolder, 'dir')
    mkdir(FigureFolder);
end

% outs = struct([]);
% outs(1).name    = 'et_a005b0_iter300_50BW';
% outs(1).iterNum = 300;
% outs(1).optimalForce = AFO_Force_50BW;
% outs(2).name    = 'et_a005b0_iter300_40BW';
% outs(2).iterNum = 300;
% outs(2).optimalForce = AFO_Force_40BW;
% outs(3).name    = 'et_a005b0_iter300_30BW';
% outs(3).iterNum = 300;
% outs(3).optimalForce = AFO_Force_30BW;
% outs(4).name    = 'et_a005b0_iter300_20BW';
% outs(4).iterNum = 300;
% outs(4).optimalForce = AFO_Force_20BW;
% FigureFolder = fullfile(resultFolder,'\analysis_fig\a005b0');
% if ~exist(FigureFolder, 'dir')
%     mkdir(FigureFolder);
% end

% outs = struct([]);
% outs(1).name    = 'et_a01b0_iter300_50BW';
% outs(1).iterNum = 300;
% outs(1).optimalForce = AFO_Force_50BW;
% outs(2).name    = 'et_a01b0_iter300_40BW';
% outs(2).iterNum = 300;
% outs(2).optimalForce = AFO_Force_40BW;
% outs(3).name    = 'et_a01b0_iter300_30BW';
% outs(3).iterNum = 300;
% outs(3).optimalForce = AFO_Force_30BW;
% outs(4).name    = 'et_a01b0_iter300_20BW';
% outs(4).iterNum = 300;
% outs(4).optimalForce = AFO_Force_20BW;
% FigureFolder = fullfile(resultFolder,'\analysis_fig\a01b0');
% if ~exist(FigureFolder, 'dir')
%     mkdir(FigureFolder);
% end

% outs = struct([]);
% outs(1).name    = 'et_a03b0_iter300_50BW';
% outs(1).iterNum = 300;
% outs(1).optimalForce = AFO_Force_50BW;
% outs(2).name    = 'et_a03b0_iter300_40BW';
% outs(2).iterNum = 300;
% outs(2).optimalForce = AFO_Force_40BW;
% outs(3).name    = 'et_a03b0_iter300_30BW';
% outs(3).iterNum = 300;
% outs(3).optimalForce = AFO_Force_30BW;
% outs(4).name    = 'et_a03b0_iter300_20BW';
% outs(4).iterNum = 300;
% outs(4).optimalForce = AFO_Force_20BW;
% FigureFolder = fullfile(resultFolder,'\analysis_fig\a03b0');
% if ~exist(FigureFolder, 'dir')
%     mkdir(FigureFolder);
% end

% outs = struct([]);
% outs(1).name    = 'et_a05b0_iter300_50BW';
% outs(1).iterNum = 300;
% outs(1).optimalForce = AFO_Force_50BW;
% outs(2).name    = 'et_a05b0_iter300_40BW';
% outs(2).iterNum = 300;
% outs(2).optimalForce = AFO_Force_40BW;
% outs(3).name    = 'et_a05b0_iter300_30BW';
% outs(3).iterNum = 300;
% outs(3).optimalForce = AFO_Force_30BW;
% outs(4).name    = 'et_a05b0_iter300_20BW';
% outs(4).iterNum = 300;
% outs(4).optimalForce = AFO_Force_20BW;
% FigureFolder = fullfile(resultFolder,'\analysis_fig\a05b0');
% if ~exist(FigureFolder, 'dir')
%     mkdir(FigureFolder);
% end

% % 같은 파라미터/힘에서 정상과 장애 차이 분석
% outs = struct([]);
% outs(1).name    = 'et_a001b0_iter300_50BW';
% outs(1).iterNum = 300;
% outs(1).optimalForce = AFO_Force_50BW;
% outs(2).name    = 'et_a001b0_iter300_50BW_sol12.5';
% outs(2).iterNum = 300;
% outs(2).optimalForce = AFO_Force_50BW;
% outs(3).name    = 'et_a001b0_iter300_50BW_sol25';
% outs(3).iterNum = 300;
% outs(3).optimalForce = AFO_Force_50BW;
% outs(4).name    = 'et_a001b0_iter300_50BW_gastr12.5';
% outs(4).iterNum = 300;
% outs(4).optimalForce = AFO_Force_50BW;
% outs(5).name    = 'et_a001b0_iter300_50BW_gastr25';
% outs(5).iterNum = 300;
% outs(5).optimalForce = AFO_Force_50BW;
% FigureFolder = fullfile(resultFolder,'\analysis_fig\a001b0_disabled');
% if ~exist(FigureFolder, 'dir')
%     mkdir(FigureFolder);
% end

% % 전체
outs = struct([]);
outs(1).name    = 'et_a001b0_iter300_20BW';
outs(1).iterNum = 300;
outs(1).optimalForce = AFO_Force_20BW;
outs(2).name    = 'et_a005b0_iter300_20BW';
outs(2).iterNum = 300;
outs(2).optimalForce = AFO_Force_20BW;
outs(3).name    = 'et_a01b0_iter300_20BW';
outs(3).iterNum = 300;
outs(3).optimalForce = AFO_Force_20BW;
outs(4).name    = 'et_a03b0_iter300_20BW';
outs(4).iterNum = 300;
outs(4).optimalForce = AFO_Force_20BW;
outs(5).name    = 'et_a05b0_iter300_20BW';
outs(5).iterNum = 300;
outs(5).optimalForce = AFO_Force_20BW;

outs(6).name    = 'et_a001b0_iter300_30BW';
outs(6).iterNum = 300;
outs(6).optimalForce = AFO_Force_30BW;
outs(7).name    = 'et_a005b0_iter300_30BW';
outs(7).iterNum = 300;
outs(7).optimalForce = AFO_Force_30BW;
outs(8).name    = 'et_a01b0_iter300_30BW';
outs(8).iterNum = 300;
outs(8).optimalForce = AFO_Force_30BW;
outs(9).name    = 'et_a03b0_iter300_30BW';
outs(9).iterNum = 300;
outs(9).optimalForce = AFO_Force_30BW;
outs(10).name    = 'et_a05b0_iter300_30BW';
outs(10).iterNum = 300;
outs(10).optimalForce = AFO_Force_30BW;

outs(11).name    = 'et_a001b0_iter300_40BW';
outs(11).iterNum = 300;
outs(11).optimalForce = AFO_Force_40BW;
outs(12).name    = 'et_a005b0_iter300_40BW';
outs(12).iterNum = 300;
outs(12).optimalForce = AFO_Force_40BW;
outs(13).name    = 'et_a01b0_iter300_40BW';
outs(13).iterNum = 300;
outs(13).optimalForce = AFO_Force_40BW;
outs(14).name    = 'et_a03b0_iter300_40BW';
outs(14).iterNum = 300;
outs(14).optimalForce = AFO_Force_40BW;
outs(15).name    = 'et_a05b0_iter300_40BW';
outs(15).iterNum = 300;
outs(15).optimalForce = AFO_Force_40BW;

outs(16).name    = 'et_a001b0_iter300_50BW';
outs(16).iterNum = 300;
outs(16).optimalForce = AFO_Force_50BW;
outs(17).name    = 'et_a005b0_iter300_50BW';
outs(17).iterNum = 300;
outs(17).optimalForce = AFO_Force_50BW;
outs(18).name    = 'et_a01b0_iter300_50BW';
outs(18).iterNum = 300;
outs(18).optimalForce = AFO_Force_50BW;
outs(19).name    = 'et_a03b0_iter300_50BW';
outs(19).iterNum = 300;
outs(19).optimalForce = AFO_Force_50BW;
outs(20).name    = 'et_a05b0_iter300_50BW';
outs(20).iterNum = 300;
outs(20).optimalForce = AFO_Force_50BW;

FigureFolder = fullfile(resultFolder,'\analysis_fig\tot');
if ~exist(FigureFolder, 'dir')
    mkdir(FigureFolder);
end


%% ====== Plot 저장 경로 설정 ======
% FigureFolder = fullfile(resultFolder,'\analysis_fig');
% if ~exist(FigureFolder, 'dir')
%     mkdir(FigureFolder);
% end

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

    outDir  = fullfile(resultFolder, outName);

    All(o).name         = outName;
    All(o).iterNum      = iterNum;
    All(o).outputFolder = outDir;
    All(o).optimalForce = outs(o).optimalForce;
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
inputFolder = fullfile(baseFolder, '..', 'inputs');
grfInitSto   = fullfile(inputFolder, 'Off_GRF.sto');
guessInitSto = fullfile(inputFolder, 'Off_kinematics.sto');

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
baseline = struct();
baseline.nameGRf = grfInitSto;
baseline.nameGuess = guessInitSto;
baseline.Prop = trapz(t0, max(vx0,0));
baseline.peakApGRF = max(vx0);

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
baseline.CMAPD_GS     = (trapz(tk0,g0) + trapz(tk0,s0)) / stride0;
baseline.CMAPD_Shank  = (trapz(tk0,g0) + trapz(tk0,s0) + trapz(tk0,ta0)) / stride0;
baseline.CMAPD_4set   = (trapz(tk0,g0) + trapz(tk0,s0) + trapz(tk0,ta0) + trapz(tk0,rf0)) / stride0;
baseline.CMAPD_whole  = ( ...
    trapz(tk0,g0) + trapz(tk0,s0) + trapz(tk0,ta0) + trapz(tk0,rf0) + ...
    trapz(tk0,hs0) + trapz(tk0,bfs0) + trapz(tk0,gm0) + trapz(tk0,il0) + trapz(tk0,va0) ) / stride0;

baseline.Speed = stride0 / (tk0(end) - tk0(1));
baseline.elapsedTime = tk0(end) - tk0(1);
baseline.strideLength = stride0;
baseline.PosCoMWork = trapz(tk0, max(vx0,0) .* vPel0);
baseline.PosCoMWorkOverDist = trapz(tk0, max(vx0,0) .* vPel0) / stride0;

legLength = 0.85;
g = 9.801;
baseline.Fr = (baseline.Speed)^2/(g*legLength);
baseline.ChacLength = stride0 / legLength;

%% ===== metrics per output + gradient colors =====
nOut = numel(All);
baseColors = lines(nOut);           % output별 기준색
minMix = 0.25;                      % 1번 iter의 "연함" 정도 (0~1, 클수록 더 하얘짐)
ms = 30;                            % marker size
momentArmAFO = 0.07;                % AFO(PathActuator)의 moment arm -> 고정된 값 사용
legLength = 0.85;
g = 9.801;

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
    PosCoMWork = nan(iterNum,1);
    PosCoMWorkOverDist = nan(iterNum,1);
    PosAnkWork = nan(iterNum,1);
    PosAnkWorkOverDist = nan(iterNum,1);
    Fr = nan(iterNum,1);
    ChacLength = nan(iterNum,1);
    
    % iter 그라데이션 색 (white -> baseColor)
    a = linspace(minMix, 1, iterNum)';          % 1: 연함, end: 진함
    iterColor = (1-a)*[1 1 1] + a*baseColors(o,:);  % [iterNum x 3]

    for i = 1:iterNum
        % propulsion
        t  = All(o).iter(i).grf.t(:);
        vx = All(o).iter(i).grf.vx(:);
        prop = trapz(t, max(vx,0));
        Prop(i) = prop;
        deltaProp(i) = prop - baseline.Prop;

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
        optF = All(o).optimalForce;
        integralF(i) = trapz(tc, u * optF);

        % elapsed time, walking distance
        elapsedTime(i) = tk(end) - tk(1);
        strideLength(i) = dist;

        % peak apGRF
        peakApGRF(i) = max(vx);

        % normalized delta propulsion
        dP_over_dist(i) = deltaProp(i) / dist;
        dP_over_time(i) = deltaProp(i) / elapsedTime(i);

        % PosCoMWork = ∫(GRF_ap * max(v,0)) dt
        vPel = All(o).iter(i).kin.pelvisTxSpeed(:);
        PosCoMWork(i) = trapz(t, max(vx,0) .* vPel);
        PosCoMWorkOverDist(i) = trapz(t, max(vx,0) .* vPel)/dist;

        % PosAnkWork = ∫(F * max(w,0)) dt
        w  = - All(o).iter(i).analy.w(:); % OpenSim에선 Plantar 방향이 음수이므로 마이너스 추가 필요
        Tau   = u * optF * momentArmAFO;
        PosAnkWork(i) = trapz(tc, Tau .* max(w,0));
        PosAnkWorkOverDist(i) = trapz(tc, Tau .* max(w,0))/dist;

        % 기타
        Fr(i) = (Speed(i).^2)/ (g*legLength);
        ChacLength(i) = strideLength(i)/legLength;
    end

    % Save metrics
    All(o).metric.CMAPD_GS    = CMAPD_GS;
    All(o).metric.CMAPD_Shank = CMAPD_Shank;
    All(o).metric.CMAPD_4set  = CMAPD_4set;
    All(o).metric.CMAPD_whole = CMAPD_whole;
    All(o).metric.Speed = Speed;
    All(o).metric.deltaProp = deltaProp;
    All(o).metric.Prop = Prop;
    All(o).metric.Effort  = integralF;
    All(o).metric.elapsedTime  = elapsedTime;
    All(o).metric.strideLength  = strideLength;
    All(o).metric.peakApGRF    = peakApGRF;
    All(o).metric.dP_over_dist = dP_over_dist;
    All(o).metric.dP_over_time = dP_over_time;
    All(o).metric.PosCoMWork = PosCoMWork;
    All(o).metric.PosAnkWork = PosAnkWork;
    All(o).metric.PosCoMWorkOverDist = PosCoMWorkOverDist;
    All(o).metric.PosAnkWorkOverDist = PosAnkWorkOverDist;   
    All(o).metric.Fr = Fr;
    All(o).metric.ChacLength = ChacLength;   

    All(o).color = iterColor;
end

%% ===== Export csv =====

% 저장할 csv 파일명
outFile = 'All_metric_export2.csv';
csvDir = fullfile(resultFolder,outFile);

% metric 이름 목록
metricNames = fieldnames(All(1).metric);
nMetric = numel(metricNames);
nExp = numel(All);

% 각 All(i)의 row 수 자동 계산
nRowsEach = zeros(nExp,1);
for i = 1:nExp
    thisLen = zeros(nMetric,1);
    for m = 1:nMetric
        thisLen(m) = numel(All(i).metric.(metricNames{m}));
    end

    % 한 실험 안의 metric 길이가 서로 다른 경우 체크
    if any(thisLen ~= thisLen(1))
        error('All(%d).metric 안의 metric들 길이가 서로 다릅니다.', i);
    end

    nRowsEach(i) = thisLen(1);
end

% 전체 row 수 계산
% header 1행 + baseline 1행 + 각 실험 데이터 행
nTotalRows = 1 + 1 + sum(nRowsEach);

% cell 배열 미리 할당
C = cell(nTotalRows, nMetric + 1);

% header
C(1,:) = [{'name'}, metricNames(:)'];

% baseline 1행 작성
C{2,1} = 'baseline';
for m = 1:nMetric
    thisMetric = metricNames{m};

    if isfield(baseline, thisMetric) && ~isempty(baseline.(thisMetric))
        val = baseline.(thisMetric);
        val = val(1);  % baseline은 1개 값만 사용
    else
        val = NaN;
    end

    C{2,m+1} = val;
end

% All(i) 데이터 작성
rowStart = 3;

for i = 1:nExp
    nRow = nRowsEach(i);
    rowEnd = rowStart + nRow - 1;

    % 1열에 이름 반복
    C(rowStart:rowEnd, 1) = {All(i).name};

    % 각 metric을 세로로 넣기
    for m = 1:nMetric
        thisMetric = metricNames{m};
        val = All(i).metric.(thisMetric);
        val = val(:);  % 무조건 열벡터

        C(rowStart:rowEnd, m+1) = num2cell(val);
    end

    rowStart = rowEnd + 1;
end

% 저장
writecell(C, csvDir);


%% ===== cluster plots =====

% legend용 더미 핸들
dummy = gobjects(nOut,1);

% 점을 몇 개씩 찍을지
sampleStep = 10;   % n개 중 1개만 표시


% %%%%%%% CMAPD_GS vs 다른 애들 %%%%%%%
% 
% % CMAPD_GS vs Speed
% figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
% for o = 1:nOut
%     idx = 1:sampleStep:All(o).iterNum;
%     scatter(All(o).metric.CMAPD_GS(idx), All(o).metric.Speed(idx), ms, All(o).color(idx,:), 'filled');
%     dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
% end
% hBase = scatter(baseline.CMAPD_GS, baseline.Speed, 1000, 'k', 'filled', 'Marker', 'p');
% xlabel('CMAPD (s/m)'); ylabel('Speed (m/s)');
% title('CMAPD\_GS vs Gait speed');
% set(gca,'FontSize',25);
% lg = {All.name}; lg{end+1} = 'baseline';
% legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none', 'FontSize',15);
% exportgraphics(gcf, fullfile(FigureFolder, 'CMAPD_GS_Speed.png'), 'Resolution', 300);
% 
% 
% % CMAPD_GS vs delta(Propulsion)
% figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
% for o = 1:nOut
%     idx = 1:sampleStep:All(o).iterNum;
%     scatter(All(o).metric.CMAPD_GS(idx), All(o).metric.deltaProp(idx), ms, All(o).color(idx,:), 'filled');
%     dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
% end
% hBase = scatter(baseline.CMAPD_GS, 0, 1000, 'k', 'filled', 'Marker', 'p');
% xlabel('CMAPD (s/m)'); ylabel('\Delta Propulsion (N·s)');
% title('CMAPD\_GS vs \Delta Propulsion');
% set(gca,'FontSize',25);
% lg = {All.name}; lg{end+1} = 'baseline';
% legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none', 'FontSize',15);
% exportgraphics(gcf, fullfile(FigureFolder, 'CMAPD_GS_Propulsion.png'), 'Resolution', 300);
% 
% 
% % CMAPD_GS vs Effort
% figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
% for o = 1:nOut
%     idx = 1:sampleStep:All(o).iterNum;
%     scatter(All(o).metric.CMAPD_GS(idx), All(o).metric.Effort(idx), ms, All(o).color(idx,:), 'filled');
%     dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
% end
% xlabel('CMAPD (s/m)'); ylabel('Effort (N·s)');
% title('CMAPD\_GS vs Effort(\int F dt)');
% set(gca,'FontSize',25);
% lg = {All.name};
% legend(dummy, lg, 'Location','best', 'Interpreter','none', 'FontSize',15);
% exportgraphics(gcf, fullfile(FigureFolder, 'CMAPD_GS_Effort.png'), 'Resolution', 300);
% 
% 
% %%%%%%% PosCoMWork vs 다른 애들 %%%%%%%
% 
% % PosCoMWork vs Speed
% figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
% for o = 1:nOut
%     idx = 1:sampleStep:All(o).iterNum;
%     scatter(All(o).metric.PosCoMWork(idx), All(o).metric.Speed(idx), ms, All(o).color(idx,:), 'filled');
%     dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
% end
% hBase = scatter(baseline.PosCoMWork, baseline.Speed, 1000, 'k', 'filled', 'Marker', 'p');
% xlabel('Work (J)'); ylabel('Speed (m/s)');
% title('Positive CoM Work vs Gait speed');
% set(gca,'FontSize',25);
% lg = {All.name}; lg{end+1} = 'baseline';
% legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none', 'FontSize',15);
% exportgraphics(gcf, fullfile(FigureFolder, 'PosCoMWork_Speed.png'), 'Resolution', 300);
% 
% 
% % PosCoMWork vs peakApGRF
% figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
% for o = 1:nOut
%     idx = 1:sampleStep:All(o).iterNum;
%     scatter(All(o).metric.PosCoMWork(idx), All(o).metric.peakApGRF(idx), ms, All(o).color(idx,:), 'filled');
%     dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
% end
% hBase = scatter(baseline.PosCoMWork, baseline.PeakAp, 1000, 'k', 'filled', 'Marker', 'p');
% xlabel('Work (J)'); ylabel('Force (N)');
% title('Positive CoM Work vs Peak apGRF');
% set(gca,'FontSize',25);
% lg = {All.name}; lg{end+1} = 'baseline';
% legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none', 'FontSize',15);
% exportgraphics(gcf, fullfile(FigureFolder, 'PosCoMWork_peakApGRF.png'), 'Resolution', 300);
% 
% 
% % PosCoMWork vs delta(Propulsion)
% figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
% for o = 1:nOut
%     idx = 1:sampleStep:All(o).iterNum;
%     scatter(All(o).metric.PosCoMWork(idx), All(o).metric.deltaProp(idx), ms, All(o).color(idx,:), 'filled');
%     dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
% end
% hBase = scatter(baseline.PosCoMWork, 0, 1000, 'k', 'filled', 'Marker', 'p');
% xlabel('Work (J)'); ylabel('\Delta Propulsion (N·s)');
% title('Positive CoM Work vs \Delta Propulsion');
% set(gca,'FontSize',25);
% lg = {All.name}; lg{end+1} = 'baseline';
% legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none', 'FontSize',15);
% exportgraphics(gcf, fullfile(FigureFolder, 'PosCoMWork_dProp.png'), 'Resolution', 300);
% 
% 
% % PosCoMWork vs Effort
% figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
% for o = 1:nOut
%     idx = 1:sampleStep:All(o).iterNum;
%     scatter(All(o).metric.PosCoMWork(idx), All(o).metric.Effort(idx), ms, All(o).color(idx,:), 'filled');
%     dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
% end
% xlabel('Work (J)'); ylabel('Effort (N·s)');
% title('Positive CoM Work vs Effort(\int F dt)');
% set(gca,'FontSize',25);
% lg = {All.name};
% legend(dummy, lg, 'Location','best', 'Interpreter','none', 'FontSize',15);
% exportgraphics(gcf, fullfile(FigureFolder, 'PosCoMWork_Effort.png'), 'Resolution', 300);
% 
% 
% % PosCoMWork vs CMAPD_GS
% figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
% for o = 1:nOut
%     idx = 1:sampleStep:All(o).iterNum;
%     scatter(All(o).metric.PosCoMWork(idx), All(o).metric.CMAPD_GS(idx), ms, All(o).color(idx,:), 'filled');
%     dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
% end
% hBase = scatter(baseline.PosCoMWork, baseline.CMAPD_GS, 1000, 'k', 'filled', 'Marker', 'p');
% xlabel('Work (J)'); ylabel('CMAPD (s/m)');
% title('Positive CoM Work vs CMAPD\_GS');
% set(gca,'FontSize',25);
% lg = {All.name}; lg{end+1} = 'baseline';
% legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none', 'FontSize',15);
% exportgraphics(gcf, fullfile(FigureFolder, 'PosCoMWork_CMAPD_GS.png'), 'Resolution', 300);
% 
% 
% % PosCoMWork vs CMAPD_Shank
% figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
% for o = 1:nOut
%     idx = 1:sampleStep:All(o).iterNum;
%     scatter(All(o).metric.PosCoMWork(idx), All(o).metric.CMAPD_Shank(idx), ms, All(o).color(idx,:), 'filled');
%     dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
% end
% hBase = scatter(baseline.PosCoMWork, baseline.CMAPD_Shank, 1000, 'k', 'filled', 'Marker', 'p');
% xlabel('Work (J)'); ylabel('CMAPD (s/m)');
% title('Positive CoM Work vs CMAPD\_Shank');
% set(gca,'FontSize',25);
% lg = {All.name}; lg{end+1} = 'baseline';
% legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none', 'FontSize',15);
% exportgraphics(gcf, fullfile(FigureFolder, 'PosCoMWork_CMAPD_Shank.png'), 'Resolution', 300);
% 
% 
% % PosCoMWork vs CMAPD_4set
% figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
% for o = 1:nOut
%     idx = 1:sampleStep:All(o).iterNum;
%     scatter(All(o).metric.PosCoMWork(idx), All(o).metric.CMAPD_4set(idx), ms, All(o).color(idx,:), 'filled');
%     dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
% end
% hBase = scatter(baseline.PosCoMWork, baseline.CMAPD_4set, 1000, 'k', 'filled', 'Marker', 'p');
% xlabel('Work (J)'); ylabel('CMAPD (s/m)');
% title('Positive CoM Work vs CMAPD\_4set');
% set(gca,'FontSize',25);
% lg = {All.name}; lg{end+1} = 'baseline';
% legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none', 'FontSize',15);
% exportgraphics(gcf, fullfile(FigureFolder, 'PosCoMWork_CMAPD_4set.png'), 'Resolution', 300);
% 
% 
% % PosCoMWork vs CMAPD_whole
% figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
% for o = 1:nOut
%     idx = 1:sampleStep:All(o).iterNum;
%     scatter(All(o).metric.PosCoMWork(idx), All(o).metric.CMAPD_whole(idx), ms, All(o).color(idx,:), 'filled');
%     dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
% end
% hBase = scatter(baseline.PosCoMWork, baseline.CMAPD_whole, 1000, 'k', 'filled', 'Marker', 'p');
% xlabel('Work (J)'); ylabel('CMAPD (s/m)');
% title('Positive CoM Work vs CMAPD\_whole');
% set(gca,'FontSize',25);
% lg = {All.name}; lg{end+1} = 'baseline';
% legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none', 'FontSize',15);
% exportgraphics(gcf, fullfile(FigureFolder, 'PosCoMWork_CMAPD_whole.png'), 'Resolution', 300);
% 
% 
% %%%%%%% PosAnkWork vs 다른 애들 %%%%%%%
% 
% % PosAnkWork vs Speed
% figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
% for o = 1:nOut
%     idx = 1:sampleStep:All(o).iterNum;
%     scatter(All(o).metric.PosAnkWork(idx), All(o).metric.Speed(idx), ms, All(o).color(idx,:), 'filled');
%     dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
% end
% xlabel('Work (J)'); ylabel('Speed (m/s)');
% title('Positive Ank Work vs Gait speed');
% set(gca,'FontSize',25);
% lg = {All.name}; lg{end+1} = 'baseline';
% legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none', 'FontSize',15);
% exportgraphics(gcf, fullfile(FigureFolder, 'PosAnkWork_Speed.png'), 'Resolution', 300);
% 
% 
% % PosAnkWork vs peakApGRF
% figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
% for o = 1:nOut
%     idx = 1:sampleStep:All(o).iterNum;
%     scatter(All(o).metric.PosAnkWork(idx), All(o).metric.peakApGRF(idx), ms, All(o).color(idx,:), 'filled');
%     dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
% end
% xlabel('Work (J)'); ylabel('Force (N)');
% title('Positive Ank Work vs Peak apGRF');
% set(gca,'FontSize',25);
% lg = {All.name}; lg{end+1} = 'baseline';
% legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none', 'FontSize',15);
% exportgraphics(gcf, fullfile(FigureFolder, 'PosAnkWork_peakApGRF.png'), 'Resolution', 300);
% 
% 
% % PosWork vs delta(Propulsion)
% figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
% for o = 1:nOut
%     idx = 1:sampleStep:All(o).iterNum;
%     scatter(All(o).metric.PosAnkWork(idx), All(o).metric.deltaProp(idx), ms, All(o).color(idx,:), 'filled');
%     dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
% end
% xlabel('Work (J)'); ylabel('\Delta Propulsion (N·s)');
% title('Positive Ank Work vs \Delta Propulsion');
% set(gca,'FontSize',25);
% lg = {All.name}; lg{end+1} = 'baseline';
% legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none', 'FontSize',15);
% exportgraphics(gcf, fullfile(FigureFolder, 'PosAnkWork_dProp.png'), 'Resolution', 300);
% 
% 
% % PosWork vs integral(F)
% figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
% for o = 1:nOut
%     idx = 1:sampleStep:All(o).iterNum;
%     scatter(All(o).metric.PosAnkWork(idx), All(o).metric.Effort(idx), ms, All(o).color(idx,:), 'filled');
%     dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
% end
% xlabel('Work (J)'); ylabel('Effort (N·s)');
% title('Positive Ank Work vs Effort(\int F dt)');
% set(gca,'FontSize',25);
% lg = {All.name};
% legend(dummy, lg, 'Location','best', 'Interpreter','none', 'FontSize',15);
% exportgraphics(gcf, fullfile(FigureFolder, 'PosAnkWork_Effort.png'), 'Resolution', 300);
% 
% 
% % PosWork vs CMAPD_Shank
% figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
% for o = 1:nOut
%     idx = 1:sampleStep:All(o).iterNum;
%     scatter(All(o).metric.PosAnkWork(idx), All(o).metric.CMAPD_Shank(idx), ms, All(o).color(idx,:), 'filled');
%     dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
% end
% xlabel('Work (J)'); ylabel('CMAPD (s/m)');
% title('Positive Ank Work vs CMAPD\_Shank');
% set(gca,'FontSize',25);
% lg = {All.name}; lg{end+1} = 'baseline';
% legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none', 'FontSize',15);
% exportgraphics(gcf, fullfile(FigureFolder, 'PosAnkWork_CMAPD_Shank.png'), 'Resolution', 300);
% 
% 
% % PosWork vs CMAPD_4set
% figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
% for o = 1:nOut
%     idx = 1:sampleStep:All(o).iterNum;
%     scatter(All(o).metric.PosAnkWork(idx), All(o).metric.CMAPD_4set(idx), ms, All(o).color(idx,:), 'filled');
%     dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
% end
% xlabel('Work (J)'); ylabel('CMAPD (s/m)');
% title('Positive Ank Work vs CMAPD\_4set');
% set(gca,'FontSize',25);
% lg = {All.name}; lg{end+1} = 'baseline';
% legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none', 'FontSize',15);
% exportgraphics(gcf, fullfile(FigureFolder, 'PosAnkWork_CMAPD_4set.png'), 'Resolution', 300);
% 
% 
% % PosWork vs CMAPD_whole
% figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
% for o = 1:nOut
%     idx = 1:sampleStep:All(o).iterNum;
%     scatter(All(o).metric.PosAnkWork(idx), All(o).metric.CMAPD_whole(idx), ms, All(o).color(idx,:), 'filled');
%     dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
% end
% xlabel('Work (J)'); ylabel('CMAPD (s/m)');
% title('Positive Ank Work vs CMAPD\_whole');
% set(gca,'FontSize',25);
% lg = {All.name}; lg{end+1} = 'baseline';
% legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none', 'FontSize',15);
% exportgraphics(gcf, fullfile(FigureFolder, 'PosAnkWork_CMAPD_whole.png'), 'Resolution', 300);
% 
% %%%%%%% 기타 %%%%%%%
% 
% % Speed vs delta(Propulsion)
% figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
% for o = 1:nOut
%     idx = 1:sampleStep:All(o).iterNum;
%     scatter(All(o).metric.Speed(idx), All(o).metric.deltaProp(idx), ms, All(o).color(idx,:), 'filled');
%     dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
% end
% hBase = scatter(baseline.Speed, 0, 1000, 'k', 'filled', 'Marker', 'p');
% xlabel('Speed (m/s)'); ylabel('\Delta Propulsion (N·s)');
% title('Gait speed vs \Delta Propulsion');
% set(gca,'FontSize',25);
% lg = {All.name}; lg{end+1} = 'baseline';
% legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none', 'FontSize',15);
% exportgraphics(gcf, fullfile(FigureFolder, 'Speed_Propulsion.png'), 'Resolution', 300);
% 
% 
% % Speed vs dP_over_dist
% figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
% for o = 1:nOut
%     idx = 1:sampleStep:All(o).iterNum;
%     scatter(All(o).metric.Speed(idx), All(o).metric.dP_over_dist(idx), ms, All(o).color(idx,:), 'filled');
%     dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
% end
% xlabel('Speed (m/s)'); ylabel('\Delta Propulsion / distance (N·s/m)');
% title('Gait speed vs \Delta Propulsion / distance');
% set(gca,'FontSize',25);
% lg = {All.name};
% legend(dummy, lg, 'Location','best', 'Interpreter','none', 'FontSize',15);
% exportgraphics(gcf, fullfile(FigureFolder, 'Speed_dP_over_dist.png'), 'Resolution', 300);
% 
% 
% % Speed vs dP_over_time
% figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
% for o = 1:nOut
%     idx = 1:sampleStep:All(o).iterNum;
%     scatter(All(o).metric.Speed(idx), All(o).metric.dP_over_time(idx), ms, All(o).color(idx,:), 'filled');
%     dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
% end
% xlabel('Speed (m/s)'); ylabel('\Delta Propulsion / time (N)');
% title('Gait speed vs \Delta Propulsion / time');
% set(gca,'FontSize',25);
% lg = {All.name};
% legend(dummy, lg, 'Location','best', 'Interpreter','none', 'FontSize',15);
% exportgraphics(gcf, fullfile(FigureFolder, 'Speed_dP_over_time.png'), 'Resolution', 300);
% 
% 
% % Speed vs peakApGRF
% figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
% for o = 1:nOut
%     idx = 1:sampleStep:All(o).iterNum;
%     scatter(All(o).metric.Speed(idx), All(o).metric.peakApGRF(idx), ms, All(o).color(idx,:), 'filled');
%     dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
% end
% hBase = scatter(baseline.Speed, baseline.PeakAp, 1000, 'k', 'filled', 'Marker', 'p');
% xlabel('Speed (m/s)'); ylabel('Force (N)');
% title('Gait speed vs Peak apGRF');
% set(gca,'FontSize',25);
% lg = {All.name}; lg{end+1} = 'baseline';
% legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none', 'FontSize',15);
% exportgraphics(gcf, fullfile(FigureFolder, 'Speed_peakAp.png'), 'Resolution', 300);
% 
% 
% % elapsedTime vs strideLength
% figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
% for o = 1:nOut
%     idx = 1:sampleStep:All(o).iterNum;
%     scatter(All(o).metric.elapsedTime(idx), All(o).metric.strideLength(idx), ms, All(o).color(idx,:), 'filled');
%     dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
% end
% hBase = scatter(baseline.Elapsed, baseline.Stride, 1000, 'k', 'filled', 'Marker', 'p');
% xlabel('Time (s)'); ylabel('Length (m)');
% title('Elapsed time vs Stride length');
% set(gca,'FontSize',25);
% lg = {All.name}; lg{end+1} = 'baseline';
% legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none', 'FontSize',15);
% exportgraphics(gcf, fullfile(FigureFolder, 'Elapsed_Stride.png'), 'Resolution', 300);
% 
% 
% % delta(propulsion) vs Effort
% figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
% for o = 1:nOut
%     idx = 1:sampleStep:All(o).iterNum;
%     scatter(All(o).metric.deltaProp(idx), All(o).metric.Effort(idx), ms, All(o).color(idx,:), 'filled');
%     dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
% end
% xlabel('\Delta Propulsion (N·s)'); ylabel('Effort (N·s)');
% title('\Delta Propulsion vs Effort(\int F dt)');
% set(gca,'FontSize',25);
% lg = {All.name};
% legend(dummy, lg, 'Location','best', 'Interpreter','none', 'FontSize',15);
% exportgraphics(gcf, fullfile(FigureFolder, 'dProp_Effort.png'), 'Resolution', 300);


% PosCoMWork vs Speed
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    idx = 1:sampleStep:All(o).iterNum;
    scatter(All(o).metric.PosCoMWorkOverDist(idx), All(o).metric.Speed(idx), ms, All(o).color(idx,:), 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
hBase = scatter(baseline.PosCoMWorkOverDist, baseline.Speed, 1000, 'k', 'filled', 'Marker', 'p');
xlabel('Work (J)'); ylabel('Speed (m/s)');
title('Positive CoM Work over Dist vs Gait speed');
set(gca,'FontSize',25);
lg = {All.name}; lg{end+1} = 'baseline';
legend([dummy; hBase], lg, 'Location','best', 'Interpreter','none', 'FontSize',15);
exportgraphics(gcf, fullfile(FigureFolder, 'PosCoMWorkOverDist_Speed.png'), 'Resolution', 300);

% PosCoMWork vs Speed
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    idx = 1:sampleStep:All(o).iterNum;
    scatter(All(o).metric.PosAnkWorkOverDist(idx), All(o).metric.Speed(idx), ms, All(o).color(idx,:), 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
xlabel('Work (J)'); ylabel('Speed (m/s)');
title('Positive Ank Work over Dist vs Gait speed');
set(gca,'FontSize',25);
lg = {All.name};
legend(dummy, lg, 'Location','best', 'Interpreter','none', 'FontSize',15);
exportgraphics(gcf, fullfile(FigureFolder, 'PosCoMWorkOverDist_Speed.png'), 'Resolution', 300);


% Fr*CL vs Speed
figure('Color','w','Position',[0 0 1200 800]); hold on; box on;
for o = 1:nOut
    idx = 1:sampleStep:All(o).iterNum;
    scatter((All(o).metric.Fr(idx).^1).*(All(o).metric.ChacLength(idx)).^-3, All(o).metric.dP_over_dist(idx), ms, All(o).color(idx,:), 'filled');
    dummy(o) = plot(nan,nan,'o','MarkerFaceColor',baseColors(o,:), 'MarkerEdgeColor',baseColors(o,:));
end
xlabel('f(Fr,CL)'); 
xlabel('Fr*CL^-3'); 
ylabel('Y');
ylabel('delta Prop over dist');
% title('Positive Ank Work over Dist vs Gait speed');
set(gca,'FontSize',25);
lg = {All.name};
legend(dummy, lg, 'Location','bestoutside', 'Interpreter','none', 'FontSize',15);
% exportgraphics(gcf, fullfile(FigureFolder, 'PosCoMWorkOverDist_Speed.png'), 'Resolution', 300);
