% fix_analy_structure.m
%
% completed_queue의 기존 result 폴더를 최신 WoC_moco_main 파일 구조로 재배치.
%
% OLD 구조 (baseline 폴더 없음):
%   result_1/analy_result/ = baseline 분석 결과
%   result_i/analy_result/ = result_(i-1) moco 출력 분석 결과
%   result_N/analy_result/ = result_(N-1) moco 출력 분석 결과 (마지막 iter 분석 없음)
%
% NEW 구조 (baseline 폴더 있음):
%   baseline/analy_result/ = baseline 분석 결과
%   result_i/analy_result/ = result_i moco 출력 분석 결과
%
% Case A — result_1/analy_result/에 Kinematics_q.sto가 있는 경우 (OLD 구조)
%   .sto 파일을 한 칸씩 이동, 마지막 iter는 분석 재실행
%
% Case B — analy_result/에 .osim만 있는 경우 (분석 결과 없음)
%   baseline + 전체 iter 분석을 새로 실행

clc; clear;

%% 경로 설정
if isempty(mfilename)
    thisFile = matlab.desktop.editor.getActiveFilename;
else
    thisFile = mfilename('fullpath');
end
scriptDir   = fileparts(thisFile);
projectRoot = fullfile(scriptDir, '..');
srcDir      = fullfile(projectRoot, 'src');
inputPath   = fullfile(projectRoot, 'inputs');
modelPath   = fullfile(projectRoot, 'models');
resultsPath = fullfile(projectRoot, 'results');
queueXlsx   = fullfile(scriptDir, 'simulation_queue.xlsx');

addpath(srcDir);
addpath(fullfile(srcDir, 'WoC_moco'));

AnalySetupPath = fullfile(inputPath, 'analysis_setup.xml');
ANALY_MARKER   = '2D_gait_AFO_pc_Kinematics_q.sto';

%% completed_queue 읽기
raw = readcell(queueXlsx, 'Sheet', 'completed_queue', 'UseExcel', false);
eh_row   = find(cellfun(@(x) ischar(x) && strcmpi(strtrim(x), 'endheader'), raw(:,1)), 1);
colNames = raw(eh_row+1, :);

ci_result = find(strcmp(colNames, 'result_name'), 1);
ci_model  = find(strcmp(colNames, 'model'),       1);
ci_iter   = find(strcmp(colNames, 'iter'),         1);
ci_gait   = find(strcmp(colNames, 'gaitMode'),    1);

data = raw(eh_row+2:end, :);
data = data(any(~cellfun(@isCellEmptyLocal, data), 2), :);

fprintf('completed_queue 행 수: %d\n', size(data, 1));

%% 각 result 처리
for k = 1:size(data, 1)
    result_name = getCellStrLocal(data{k, ci_result});
    modelFile   = getCellStrLocal(data{k, ci_model});
    iterNum     = getCellNumLocal(data{k, ci_iter});
    gaitMode    = getCellStrLocal(data{k, ci_gait});
    if isempty(gaitMode), gaitMode = 'modeSym'; end
    if isempty(result_name) || isnan(iterNum), continue; end
    iterNum = round(iterNum);

    resultRoot = fullfile(resultsPath, result_name);
    if ~exist(resultRoot, 'dir'), continue; end

    % 이미 new 구조이면 스킵
    if exist(fullfile(resultRoot, 'baseline'), 'dir')
        fprintf('[SKIP] %s — baseline 폴더가 이미 있음\n', result_name);
        continue;
    end

    res1AnalyDir = fullfile(resultRoot, 'result_1', 'analy_result');
    if ~exist(res1AnalyDir, 'dir'), continue; end

    hasAnalysis = isfile(fullfile(res1AnalyDir, ANALY_MARKER));

    %% Case A: OLD 구조 — .sto 파일 재배치 + 마지막 iter 분석
    if hasAnalysis
        fprintf('[Case A] %s — .sto 재배치 + result_%d 분석\n', result_name, iterNum);

        % 1. baseline/analy_result 생성, result_1/*.sto 이동
        baselineAnalyDir = fullfile(resultRoot, 'baseline', 'analy_result');
        if ~exist(baselineAnalyDir, 'dir'), mkdir(baselineAnalyDir); end
        moveStoFiles(res1AnalyDir, baselineAnalyDir);

        % 2. result_i/*.sto → result_(i-1)/analy_result/ (i = 2..N)
        for i = 2:iterNum
            srcAnalyDir  = fullfile(resultRoot, sprintf('result_%d', i), 'analy_result');
            destAnalyDir = fullfile(resultRoot, sprintf('result_%d', i-1), 'analy_result');
            if ~exist(destAnalyDir, 'dir'), mkdir(destAnalyDir); end
            moveStoFiles(srcAnalyDir, destAnalyDir);
        end

        % 3. 마지막 iter (result_N) 분석 실행
        runIterAnalysis(resultRoot, iterNum, AnalySetupPath);

    %% Case B: 분석 없음 — baseline + 전체 iter 분석 신규 실행
    else
        fprintf('[Case B] %s — baseline + 전체 %d iter 분석\n', result_name, iterNum);

        % baseline/analy_result 생성 및 분석
        baselineAnalyDir = fullfile(resultRoot, 'baseline', 'analy_result');
        if ~exist(baselineAnalyDir, 'dir'), mkdir(baselineAnalyDir); end

        if ~isfile(fullfile(baselineAnalyDir, ANALY_MARKER))
            if strcmpi(gaitMode, 'modeAsym')
                guessInitSto = fullfile(inputPath, 'guess_init_full.sto');
            else
                guessInitSto = fullfile(inputPath, 'guess_init_half.sto');
            end
            originModelPath = fullfile(modelPath, modelFile);
            WoC_moco_analysis(AnalySetupPath, ...
                'modelPath',         originModelPath, ...
                'kinematicsStoPath', guessInitSto, ...
                'resultsDir',        baselineAnalyDir);
            fprintf('  baseline 분석 완료\n');
        else
            fprintf('  [SKIP] baseline 분석 이미 있음\n');
        end

        % 각 iter 분석
        for i = 1:iterNum
            runIterAnalysis(resultRoot, i, AnalySetupPath);
        end
    end

    fprintf('  → %s 완료\n\n', result_name);
end

fprintf('모든 처리 완료.\n');


%% ─── 로컬 함수 ─────────────────────────────────────────────────────────────

function moveStoFiles(srcFolder, destFolder)
    files = dir(fullfile(srcFolder, '*.sto'));
    for f = 1:numel(files)
        movefile(fullfile(srcFolder, files(f).name), ...
                 fullfile(destFolder, files(f).name), 'f');
    end
end

function runIterAnalysis(resultRoot, iterIdx, AnalySetupPath)
    ANALY_MARKER = '2D_gait_AFO_pc_Kinematics_q.sto';
    analyDir = fullfile(resultRoot, sprintf('result_%d', iterIdx), 'analy_result');
    mocoDir  = fullfile(resultRoot, sprintf('result_%d', iterIdx), 'moco_result');
    kinPath  = fullfile(mocoDir, sprintf('moco_WoC_Solution_iter%02d_kinematics.sto', iterIdx));

    if ~isfile(kinPath)
        fprintf('  [SKIP] result_%d: kinematics.sto 없음\n', iterIdx);
        return;
    end
    if isfile(fullfile(analyDir, ANALY_MARKER))
        fprintf('  [SKIP] result_%d: 분석 이미 있음\n', iterIdx);
        return;
    end

    osimFiles = dir(fullfile(analyDir, '*.osim'));
    if isempty(osimFiles)
        fprintf('  [WARN] result_%d: osim 파일 없음 — 건너뜀\n', iterIdx);
        return;
    end
    iterModelPath = fullfile(analyDir, osimFiles(1).name);

    WoC_moco_analysis(AnalySetupPath, ...
        'modelPath',         iterModelPath, ...
        'kinematicsStoPath', kinPath, ...
        'resultsDir',        analyDir);
    fprintf('  result_%d 분석 완료\n', iterIdx);
end

function val = getCellNumLocal(x)
    if isnumeric(x) && isscalar(x)
        val = double(x);
    elseif ischar(x) || isstring(x)
        val = str2double(char(x));
    else
        val = NaN;
    end
end

function str = getCellStrLocal(x)
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

function tf = isCellEmptyLocal(x)
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
