% result_plot_queue.m
%
% 실험 결과를 일괄 플롯하고 results/<result_name>/Fig/ 에 저장하는 스크립트.
%
% ── 모드 1: Queue 모드 (simulation_queue.xlsx completed_queue 기반) ──────────
%   plotRows 에 completed_queue 의 Excel 행 번호를 지정 (헤더 포함한 실제 행 번호).
%   directFolders 는 비워두세요 (빈 셀 배열 {}).
%
% ── 모드 2: Direct 모드 (폴더명 직접 지정) ──────────────────────────────────
%   directFolders 에 struct 배열로 폴더명과 iter 를 지정하세요.
%   plotRows 는 무시됩니다.
%   예)
%     directFolders = {
%         struct('result_name','fixedTimeSpline_0.8', 'iter',50),
%         struct('result_name','fixedTimeSpline_1.0', 'iter',50, 'model','2D_gait_AFO_pc_50BW.osim'),
%     };
%   선택 필드:
%     .model : optimalForce 결정용 모델명 (생략 시 300 N 기본값)
%     .id    : 그래프 제목 접두사 (생략 시 result_name 사용)

clc;
clear;
close all;

%% ─── 사용자 설정 ─────────────────────────────────────────────────────────────

% ── Queue 모드: completed_queue 의 Excel 행 번호 ──────────────────────────────
% plotRows = 9:55;
% plotRows = [111 128];

% ── Direct 모드: 폴더명 직접 지정 (비워두면 Queue 모드로 동작) ────────────────
directFolders = {};
directFolders = {
    % struct('result_name','fixedTimeSpline_0.8', 'iter',50),
    struct('result_name','fixedTimeSpline_1', 'iter',50),
    struct('result_name','fixedTimeSpline_1.2', 'iter',50),
};

%% ─── 경로 설정 ───────────────────────────────────────────────────────────────
if isempty(mfilename)
    thisFile = matlab.desktop.editor.getActiveFilename;
else
    thisFile = mfilename('fullpath');
end
baseFolder   = fileparts(thisFile);
rootFolder   = fullfile(baseFolder, '..');
resultFolder = fullfile(rootFolder, 'results');
inputFolder  = fullfile(rootFolder, 'inputs');
QUEUE_XLSX   = fullfile(baseFolder, 'simulation_queue.xlsx');
SHEET_DONE   = 'completed_queue';

%% ─── jobs 배열 구성 (Queue 모드 / Direct 모드 공통) ─────────────────────────
% jobs(k): .id_str / .result_name / .model_str / .iterNum
if ~isempty(directFolders)
    %% Direct 모드: directFolders → jobs
    nJobs = numel(directFolders);
    jobs  = struct('id_str',{}, 'result_name',{}, 'model_str',{}, 'iterNum',{});
    for k = 1:nJobs
        df = directFolders{k};
        if ~isfield(df,'result_name') || isempty(df.result_name)
            error('directFolders{%d}: result_name 이 지정되지 않았습니다.', k);
        end
        if ~isfield(df,'iter') || isempty(df.iter)
            error('directFolders{%d}: iter 이 지정되지 않았습니다.', k);
        end
        jobs(k).result_name = df.result_name;
        jobs(k).iterNum     = round(df.iter);
        jobs(k).model_str   = getStructField(df, 'model', '');
        jobs(k).id_str      = getStructField(df, 'id',    df.result_name);
    end
    fprintf('[Direct 모드] %d 개 폴더 처리\n', nJobs);

else
    %% Queue 모드: plotRows → jobs
    [hdr_d, colNames, data_d] = readSheet(QUEUE_XLSX, SHEET_DONE);
    nDataRows  = size(data_d, 1);
    headerRows = size(hdr_d, 1) + 1;   % endheader 까지의 행 수 + 열 이름 행

    % plotRows 는 항상 xlsx Excel 행 번호로 해석 → 데이터 인덱스로 변환
    dataIdx = plotRows - headerRows;
    fprintf('[Queue 모드] plotRows → 데이터 인덱스: %s  (headerRows=%d)\n', ...
            mat2str(dataIdx), headerRows);

    if max(dataIdx) > nDataRows || min(dataIdx) < 1
        error('plotRows(%s) 가 유효 범위(1~%d)를 벗어납니다.\n  입력한 Excel 행 번호에서 headerRows(%d)를 뺀 값이 범위를 벗어났습니다.', ...
              mat2str(dataIdx), nDataRows, headerRows);
    end

    nJobs = numel(dataIdx);
    jobs  = struct('id_str',{}, 'result_name',{}, 'model_str',{}, 'iterNum',{});
    for k = 1:nJobs
        rowIdx = dataIdx(k);
        jobs(k).id_str      = getCellStr(data_d{rowIdx, colIdx(colNames, 'ID')});
        jobs(k).result_name = getCellStr(data_d{rowIdx, colIdx(colNames, 'result_name')});
        jobs(k).model_str   = getCellStr(data_d{rowIdx, colIdx(colNames, 'model')});
        jobs(k).iterNum     = round(getCellNum(data_d{rowIdx, colIdx(colNames, 'iter')}));
    end
end

%% ─── 공통 필드 이름 (모델 구조 공통) ─────────────────────────────────────────
pelvisField      = matlab.lang.makeValidName('/jointset/groundPelvis/pelvis_tx/value');
pelvisSpeedField = matlab.lang.makeValidName('/jointset/groundPelvis/pelvis_tx/speed');
gastrocField     = matlab.lang.makeValidName('/gastroc_r/activation');
soleusField      = matlab.lang.makeValidName('/soleus_r/activation');
ankleAngVelField = matlab.lang.makeValidName('ankle_angle_r');
momentArmAFO     = 0.07;   % AFO moment arm [m]

%% ─── job 별 플롯 ─────────────────────────────────────────────────────────────
statusLog = cell(nJobs, 2);   % {result_name, 'OK' / 'SKIP: ...'}

for k = 1:nJobs

    %% 메타 정보
    id_str      = jobs(k).id_str;
    result_name = jobs(k).result_name;
    model_str   = jobs(k).model_str;
    iterNum     = jobs(k).iterNum;

    fprintf('\n[%s] result: %s  model: %s  iter: %d\n', ...
            id_str, result_name, model_str, iterNum);

    OutputFolder = fullfile(resultFolder, result_name);

    %% 결과 폴더 존재 여부 확인 (시뮬레이션 미완 시 skip)
    if ~exist(OutputFolder, 'dir')
        msg = sprintf('결과 폴더 없음: %s', OutputFolder);
        fprintf('  [SKIP] %s\n', msg);
        statusLog{k,1} = result_name;
        statusLog{k,2} = ['SKIP: ' msg];
        continue;
    end

    try   % ← 파일 누락 등 오류 발생 시 skip하고 다음 job 진행

    FigureFolder = fullfile(OutputFolder, 'Fig');
    if ~exist(FigureFolder, 'dir')
        mkdir(FigureFolder);
    end

    %% optimalForce: 모델 이름의 BW 접미사에서 자동 결정
    optimalForce = bwToOptimalForce(model_str);
    fprintf('  optimalForce = %d N  (model: %s)\n', optimalForce, model_str);

    %% Baseline 설정
    % TODO: modeAsym 포함 실험에서는 별도 Baseline 파일 사용 필요.
    %       (예: Off_kinematics_asym.sto / Off_GRF_asym.sto 등)
    grfInitSto   = fullfile(inputFolder, 'Off_GRF.sto');
    guessInitSto = fullfile(inputFolder, 'Off_kinematics.sto');

    %% Baseline 읽기
    grf0   = readSto(grfInitSto);
    guess0 = readSto(guessInitSto);

    t0           = grf0.time(:);
    vx0          = grf0.ground_force_r_vx(:);
    tg0          = guess0.time(:);
    pelv0        = guess0.(pelvisField)(:);
    avgSpeed0    = (pelv0(end) - pelv0(1)) / (tg0(end) - tg0(1));
    gastrocAct0  = guess0.(gastrocField)(:);
    soleusAct0   = guess0.(soleusField)(:);
    pelvSpeed0   = guess0.(pelvisSpeedField)(:);
    stride0      = pelv0(end) - pelv0(1);
    vFwd0_onGRF  = interp1(tg0, pelvSpeed0, t0, 'linear');
    PosCoMWork0  = trapz(t0, max(vx0, 0) .* vFwd0_onGRF);

    %% 변수 사전 정의
    tGRF         = cell(iterNum, 1);
    vxIter       = cell(iterNum, 1);
    tCtrl        = cell(iterNum, 1);
    uIter        = cell(iterNum, 1);
    tData        = cell(iterNum, 1);
    etaIter      = cell(iterNum, 1);
    avgSpeedIter = zeros(iterNum, 1);
    tKin         = cell(iterNum, 1);
    gastrocAct   = cell(iterNum, 1);
    soleusAct    = cell(iterNum, 1);
    strideLength = zeros(iterNum, 1);
    tAnaly       = cell(iterNum, 1);
    ankVelIter   = cell(iterNum, 1);
    objective_total      = nan(iterNum, 1);
    objective_effort     = nan(iterNum, 1);
    objective_final_time = nan(iterNum, 1);
    PosCoMWork   = nan(iterNum, 1);
    PosAnkWork   = nan(iterNum, 1);
    eta_baseline      = [];
    time_eta_baseline = [];
    hasEta     = false;   % data.csv (eta) 존재 여부
    hasAnkWork = false;   % analy Kinematics_u 존재 여부

    %% 반복별 데이터 읽기
    for i = 1:iterNum
        iterRootDir   = fullfile(OutputFolder, sprintf('result_%d', i));
        mocoResultDir = fullfile(iterRootDir, 'moco_result');
        controlDir    = fullfile(iterRootDir, 'control_result');
        pkDir         = fullfile(iterRootDir, 'analy_result');

        % GRF
        grf_i     = readSto(fullfile(mocoResultDir, sprintf('moco_WoC_Solution_iter%02d_GRF.sto', i)));
        tGRF{i}   = grf_i.time(:);
        vxIter{i} = grf_i.ground_force_r_vx(:);

        % Control
        ctrl_i   = readSto(fullfile(controlDir, 'control.sto'));
        tCtrl{i} = ctrl_i.time(:);
        uIter{i} = ctrl_i.AFO_r(:);

        % data.csv (eta) — modeOff 등에서는 없을 수 있음
        csvPath_i = fullfile(controlDir, 'data.csv');
        if exist(csvPath_i, 'file') == 2
            csv_i      = readtable(csvPath_i);
            tData{i}   = csv_i.stanceTime(:);
            etaIter{i} = csv_i.eta(:);
            if i == 1
                eta_baseline      = csv_i.eta(:);
                time_eta_baseline = csv_i.stanceTime(:);
            end
            hasEta = true;
        else
            if i == 1
                warning('[%s] data.csv 없음 → eta 플롯 건너뜀', result_name);
            end
        end

        % Kinematics
        kin_i   = readSto(fullfile(mocoResultDir, sprintf('moco_WoC_Solution_iter%02d_kinematics.sto', i)));
        tk      = kin_i.time(:);
        tKin{i} = tk;
        pelv    = kin_i.(pelvisField)(:);
        avgSpeedIter(i) = (pelv(end) - pelv(1)) / (tk(end) - tk(1));
        strideLength(i) = pelv(end) - pelv(1);
        gastrocAct{i}   = kin_i.(gastrocField)(:);
        soleusAct{i}    = kin_i.(soleusField)(:);

        pelvSpeed     = kin_i.(pelvisSpeedField)(:);
        vFwd_onGRF    = interp1(tk, pelvSpeed, tGRF{i}, 'linear');
        PosCoMWork(i) = trapz(tGRF{i}, max(vxIter{i}, 0) .* vFwd_onGRF);

        % Analy (ankle angular velocity) — analy_result 없을 수 있음
        analyPath_i = fullfile(pkDir, '2D_gait_AFO_pc_Kinematics_u.sto');
        if exist(analyPath_i, 'file') == 2
            analy_i      = readSto(analyPath_i);
            ta           = analy_i.time(:);
            wa           = -analy_i.(ankleAngVelField)(:);   % plantarflexion 방향을 양수로
            tAnaly{i}    = ta;
            ankVelIter{i} = wa;

            tauCtrl      = uIter{i} * optimalForce * momentArmAFO;
            tau_on_analy = interp1(tCtrl{i}, tauCtrl, ta, 'linear', 'extrap');
            PosAnkWork(i) = trapz(ta, tau_on_analy .* max(wa, 0));
            hasAnkWork = true;
        else
            if i == 1
                warning('[%s] Kinematics_u.sto 없음 → 발목 일 플롯 건너뜀', result_name);
            end
        end

        % Objective (kinematics_half 헤더)
        costPath_i = fullfile(mocoResultDir, sprintf('moco_WoC_Solution_iter%02d_kinematics_half.sto', i));
        if exist(costPath_i, 'file') == 2
            [objective_total(i), objective_effort(i), objective_final_time(i)] = parseObjective(costPath_i);
        end
    end

    %% 색상 그라디언트 (blue → red, 중간은 흰색 혼합)
    colors      = makeColorGradient(iterNum);
    titlePrefix = sprintf('[%s]', id_str);

    %% 1) apGRF
    fig = figure('Color','w','Position',[0 0 1200 800],'Visible','off');
    hold on; box on;
    plot(t0, vx0, 'k', 'LineWidth', 4, 'DisplayName', 'baseline');
    for i = 1:iterNum
        plot(tGRF{i}, vxIter{i}, 'Color', colors(i,:), 'LineWidth', 1);
    end
    yline(0, '--');
    xlabel('Time (s)'); ylabel('Force (N)');
    title(sprintf('%s apGRF', titlePrefix), 'Interpreter', 'none');
    set(gca, 'FontSize', 25);
    exportgraphics(fig, fullfile(FigureFolder, '01_apGRF_right.png'), 'Resolution', 300);
    close(fig);

    %% 2) AFO Control
    fig = figure('Color','w','Position',[0 0 1200 800],'Visible','off');
    hold on; box on;
    for i = 1:iterNum
        plot(tCtrl{i}, uIter{i}, 'Color', colors(i,:), 'LineWidth', 1);
    end
    xlabel('Time (s)'); ylabel('Control (0~1)');
    ylim([0 1]);
    title(sprintf('%s AFO Optimal Control', titlePrefix), 'Interpreter', 'none');
    set(gca, 'FontSize', 25);
    exportgraphics(fig, fullfile(FigureFolder, '02_control_AFO_right.png'), 'Resolution', 300);
    close(fig);

    %% 3) Propulsive Transfer Ratio (eta) — data.csv 있을 때만
    if hasEta
        fig = figure('Color','w','Position',[0 0 1200 800],'Visible','off');
        hold on; box on;
        plot(time_eta_baseline, eta_baseline, 'k', 'LineWidth', 4, 'DisplayName', 'baseline');
        for i = 1:iterNum
            if ~isempty(etaIter{i})
                plot(tData{i}, etaIter{i}, 'Color', colors(i,:), 'LineWidth', 1);
            end
        end
        yline(0, '--');
        xlabel('Time (s)'); ylabel('Eta');
        title(sprintf('%s Propulsive Transfer Ratio', titlePrefix), 'Interpreter', 'none');
        set(gca, 'FontSize', 25);
        exportgraphics(fig, fullfile(FigureFolder, '03_eta_right.png'), 'Resolution', 300);
        close(fig);
    end

    %% 4) 평균 보행 속도
    fig = figure('Color','w','Position',[0 0 1200 800],'Visible','off');
    hold on; box on;
    plot(0, avgSpeed0, 'o', 'Color', 'black', 'LineWidth', 10, 'DisplayName', 'baseline');
    plot(1:iterNum, avgSpeedIter, 'o-', 'LineWidth', 1.5, 'Color', [0 0.5 0]);
    xlabel('Iteration'); ylabel('Avg Walking Speed (m/s)');
    xlim([-1 iterNum+1]);
    title(sprintf('%s Final Step Time: %.4f', titlePrefix, tKin{iterNum}(end)), 'Interpreter', 'none');
    set(gca, 'FontSize', 25);
    exportgraphics(fig, fullfile(FigureFolder, '04_avg_walking_speed.png'), 'Resolution', 300);
    close(fig);

    %% 5) Gastroc Activation
    fig = figure('Color','w','Position',[0 0 1200 800],'Visible','off');
    hold on; box on;
    plot(tg0, gastrocAct0, 'k', 'LineWidth', 4, 'DisplayName', 'baseline');
    for i = 1:iterNum
        plot(tKin{i}, gastrocAct{i}, 'Color', colors(i,:), 'LineWidth', 1);
    end
    xlabel('Time (s)'); ylabel('Muscle Activation (0~1)');
    title(sprintf('%s Gastroc Activation', titlePrefix), 'Interpreter', 'none');
    set(gca, 'FontSize', 25);
    exportgraphics(fig, fullfile(FigureFolder, '05_gastroc_r_activation.png'), 'Resolution', 300);
    close(fig);

    %% 6) Soleus Activation
    fig = figure('Color','w','Position',[0 0 1200 800],'Visible','off');
    hold on; box on;
    plot(tg0, soleusAct0, 'k', 'LineWidth', 4, 'DisplayName', 'baseline');
    for i = 1:iterNum
        plot(tKin{i}, soleusAct{i}, 'Color', colors(i,:), 'LineWidth', 1);
    end
    xlabel('Time (s)'); ylabel('Muscle Activation (0~1)');
    title(sprintf('%s Soleus Activation', titlePrefix), 'Interpreter', 'none');
    set(gca, 'FontSize', 25);
    exportgraphics(fig, fullfile(FigureFolder, '06_soleus_r_activation.png'), 'Resolution', 300);
    close(fig);

    %% 7) Stride Length
    fig = figure('Color','w','Position',[0 0 1200 800],'Visible','off');
    hold on; box on;
    plot(0, stride0, 'o', 'Color', 'black', 'LineWidth', 10, 'DisplayName', 'baseline');
    plot(1:iterNum, strideLength, 'o-', 'LineWidth', 1.5);
    xlabel('Iteration'); ylabel('Stride length (m)');
    xlim([-1 iterNum+1]);
    title(sprintf('%s Stride Length', titlePrefix), 'Interpreter', 'none');
    set(gca, 'FontSize', 25);
    exportgraphics(fig, fullfile(FigureFolder, '07_stride_length.png'), 'Resolution', 300);
    close(fig);

    %% 8) Objective Cost
    fig = figure('Color','w','Position',[0 0 1200 800],'Visible','off');
    hold on; box on;
    plot(1:iterNum, objective_effort,     'o-', 'LineWidth', 1.5);
    plot(1:iterNum, objective_final_time, 'o-', 'LineWidth', 1.5);
    plot(1:iterNum, objective_total,      'o-', 'LineWidth', 1.5);
    xlabel('Iteration'); ylabel('Cost');
    xlim([-1 iterNum+1]);
    legend('Effort', 'FinalTime', 'Total', 'Location', 'best');
    title(sprintf('%s Objective Cost', titlePrefix), 'Interpreter', 'none');
    set(gca, 'FontSize', 25);
    exportgraphics(fig, fullfile(FigureFolder, '08_objective_cost.png'), 'Resolution', 300);
    close(fig);

    %% 9) Positive CoM Work
    fig = figure('Color','w','Position',[0 0 1200 800],'Visible','off');
    hold on; box on;
    plot(0, PosCoMWork0, 'o', 'Color', 'black', 'LineWidth', 10);
    plot(1:iterNum, PosCoMWork, 'o-', 'LineWidth', 1.5, 'Color', [0 0.5 0]);
    xlabel('Iteration'); ylabel('Work (J)');
    xlim([-1 iterNum+1]);
    title(sprintf('%s Positive CoM Work', titlePrefix), 'Interpreter', 'none');
    set(gca, 'FontSize', 25);
    exportgraphics(fig, fullfile(FigureFolder, '09_PosCoMWork.png'), 'Resolution', 300);
    close(fig);

    %% 10) Positive Ankle Work — Kinematics_u.sto 있을 때만
    if hasAnkWork
        fig = figure('Color','w','Position',[0 0 1200 800],'Visible','off');
        hold on; box on;
        plot(1:iterNum, PosAnkWork, 'o-', 'LineWidth', 1.5, 'Color', [0 0.5 0]);
        xlabel('Iteration'); ylabel('Work (J)');
        xlim([-1 iterNum+1]);
        title(sprintf('%s Positive Ankle Work', titlePrefix), 'Interpreter', 'none');
        set(gca, 'FontSize', 25);
        exportgraphics(fig, fullfile(FigureFolder, '10_PosAnkWork.png'), 'Resolution', 300);
        close(fig);
    end

    fprintf('  -> Fig 저장 완료: %s\n', FigureFolder);
    statusLog{k,1} = result_name;
    statusLog{k,2} = 'OK';

    catch ME   % ← try 대응
        msg = ME.message;
        fprintf('  [SKIP] 오류 발생: %s\n', msg);
        statusLog{k,1} = result_name;
        statusLog{k,2} = ['SKIP: ' msg];
    end
end

%% ─── 완료 요약 ───────────────────────────────────────────────────────────────
fprintf('\n======== 플롯 요약 ========\n');
for k = 1:nJobs
    fprintf('  [%d/%d] %-30s : %s\n', k, nJobs, statusLog{k,1}, statusLog{k,2});
end
fprintf('===========================\n');


%% ─── 로컬 함수 ───────────────────────────────────────────────────────────────

function s = readSto(filepath)
% STO 파일을 읽어 컬럼 이름을 필드로 갖는 struct 반환.
    fid = fopen(filepath, 'r');
    if fid == -1, error('Cannot open %s', filepath); end
    line = fgetl(fid);
    while ischar(line)
        if startsWith(strtrim(line), 'endheader'), break; end
        line = fgetl(fid);
    end
    varLine = fgetl(fid);
    names   = strsplit(strtrim(varLine));
    data    = fscanf(fid, '%f', [numel(names), Inf])';
    fclose(fid);
    s = struct();
    for k = 1:numel(names)
        s.(matlab.lang.makeValidName(names{k})) = data(:, k);
    end
end

% ─────────────────────────────────────────────────────────────────────────────

function force = bwToOptimalForce(modelName)
% 모델 파일 이름의 _xxBW 접미사에서 AFO PathActuator optimal force (N) 결정.
%   50BW -> 300 N
%   40BW -> 243 N
%   30BW -> 182 N
%   20BW -> 121 N
%   BW 표기 없음 -> 300 N (기본값)
    bwTable = {'50BW', 300; '40BW', 243; '30BW', 182; '20BW', 121};
    force = 300;
    for k = 1:size(bwTable, 1)
        if contains(modelName, ['_' bwTable{k,1}])
            force = bwTable{k, 2};
            return;
        end
    end
end

% ─────────────────────────────────────────────────────────────────────────────

function colors = makeColorGradient(n)
% blue(1번) → red(마지막) 그라디언트, 중간 값은 흰색이 혼합되어 밝아짐.
    blue = [0 0 1];  red = [1 0 0];  white = [1 1 1];
    p = 2.2;  minW = 0.0;  maxW = 0.95;
    colors = zeros(n, 3);
    for i = 1:n
        t    = (i - 1) / max(n - 1, 1);
        d    = abs(2*t - 1);
        mixW = minW + (maxW - minW) * (1 - d^p);
        base = (1 - t)*blue + t*red;
        colors(i, :) = (1 - mixW)*base + mixW*white;
    end
    colors(1, :)   = blue;
    colors(end, :) = red;
end

% ─────────────────────────────────────────────────────────────────────────────

function [obj, objEff, objTime] = parseObjective(filepath)
% kinematics_half.sto 헤더에서 objective 값들을 파싱.
    obj = nan;  objEff = nan;  objTime = nan;
    fid = fopen(filepath, 'r');
    if fid == -1, return; end
    line = fgetl(fid);
    while ischar(line)
        s = strtrim(line);
        if startsWith(s, 'endheader'), break; end
        if startsWith(s, 'objective=')
            obj    = sscanf(s, '%*[^=]=%f');
        elseif startsWith(s, 'objective_effort=')
            objEff = sscanf(s, '%*[^=]=%f');
        elseif startsWith(s, 'objective_final_time=')
            objTime = sscanf(s, '%*[^=]=%f');
        end
        line = fgetl(fid);
    end
    fclose(fid);
end

% ─────────────────────────────────────────────────────────────────────────────

function [header_block, col_names, data] = readSheet(xlsx_path, sheet_name)
% xlsx 시트를 읽어 header_block / col_names / data 로 분리.
    raw    = readcell(xlsx_path, 'Sheet', sheet_name, 'UseExcel', false);
    eh_row = find(cellfun(@(x) ischar(x) && strcmpi(strtrim(x), 'endheader'), raw(:, 1)), 1);
    if isempty(eh_row)
        error('시트 "%s" 에 "endheader" 행이 없습니다.', sheet_name);
    end
    header_block  = raw(1:eh_row, :);
    cn_row        = eh_row + 1;
    col_names_raw = raw(cn_row, :);
    valid         = ~cellfun(@isCellEmpty, col_names_raw);
    last_c        = find(valid, 1, 'last');
    col_names     = col_names_raw(1:last_c);
    nCols         = last_c;
    if size(raw, 1) > cn_row
        raw_data = raw(cn_row+1:end, 1:min(nCols, size(raw, 2)));
        if size(raw_data, 2) < nCols
            raw_data(:, end+1:nCols) = {missing};
        end
        non_empty = any(~cellfun(@isCellEmpty, raw_data), 2);
        data = raw_data(non_empty, :);
    else
        data = cell(0, nCols);
    end
end

% ─────────────────────────────────────────────────────────────────────────────

function idx = colIdx(col_names, name)
    idx = find(strcmp(col_names, name), 1);
    if isempty(idx), idx = 0; end
end

function val = getCellNum(x)
    if isnumeric(x) && isscalar(x)
        val = double(x);
    elseif ischar(x) || isstring(x)
        val = str2double(char(x));
    else
        val = NaN;
    end
end

function str = getCellStr(x)
    if ischar(x)
        str = strtrim(x);
    elseif isstring(x) && ~ismissing(x)
        str = strtrim(char(x));
    elseif isnumeric(x) && isscalar(x) && ~isnan(x)
        str = num2str(x);
    else
        str = '';
    end
end

function tf = isCellEmpty(x)
    if isnumeric(x) || islogical(x)
        tf = isempty(x) || (isscalar(x) && isnan(x));
    elseif ischar(x)
        tf = isempty(strtrim(x));
    elseif isstring(x)
        tf = ismissing(x) || strlength(x) == 0;
    else
        tf = true;
    end
end

% ─────────────────────────────────────────────────────────────────────────────

function val = getStructField(s, field, defaultVal)
% struct 에서 field 값을 읽고, 없거나 비어 있으면 defaultVal 반환.
    if isfield(s, field) && ~isempty(s.(field))
        val = s.(field);
    else
        val = defaultVal;
    end
end
