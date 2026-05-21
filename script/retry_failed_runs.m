% retry_failed_runs.m
%
% 지정한 queue.xlsx의 Complete=-1 행을 재시도.
%   - 수렴 여부와 무관하게 모든 iter 실행
%   - 미수렴 시: sol.unseal() 후 trajectory 저장
%   - 수렴 시  : 정상 저장 (analysis + ID 포함)
%   - 전체 MATLAB 로그를 results/<result_name>/retry_log.txt 로 저장
%
% 사용법: QUEUE_XLSX 경로를 지정하고 실행

clc; close all;

%% ─── 사용자 설정 ────────────────────────────────────────────────────────────
QUEUE_XLSX  = 'simulation_queue.xlsx';   % ← 여기에 대상 xlsx 경로 지정
SHEET_QUEUE = 'simulation_queue';
%% ─────────────────────────────────────────────────────────────────────────────

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

addpath(srcDir);
addpath(fullfile(srcDir, 'WoC_moco'));

import org.opensim.modeling.*

AnalySetupPath = fullfile(inputPath, 'analysis_setup.xml');
idXmlPath      = fullfile(inputPath, 'id_setup.xml');
grfXmlBase     = fullfile(inputPath, 'GRF_setup.xml');

%% queue 읽기 (Complete=-1 행만 추출)
if ~isfile(QUEUE_XLSX)
    error('queue 파일을 찾을 수 없습니다: %s', QUEUE_XLSX);
end
raw = readcell(QUEUE_XLSX, 'Sheet', SHEET_QUEUE, 'UseExcel', false);
eh_row   = find(cellfun(@(x) ischar(x) && strcmpi(strtrim(x),'endheader'), raw(:,1)), 1);
colNames = raw(eh_row+1, :);
data     = raw(eh_row+2:end, :);
data     = data(any(~cellfun(@isCellEmptyL, data), 2), :);

ci = @(name) find(strcmp(colNames, name), 1);
ci_complete = ci('Complete');
ci_id       = ci('ID');
ci_model    = ci('model');
ci_iter     = ci('iter');
ci_optMode  = ci('optMode_type');
ci_result   = ci('result_name');
ci_effort   = ci('mocoEffort');
ci_ft       = ci('mocoFinalTime');
ci_gait     = ci('gaitMode');
ci_tb       = ci('mocoTimeBound');
ci_db       = ci('mocoDistBound');

% Complete=-1 행 필터
failedMask = false(size(data,1),1);
for k = 1:size(data,1)
    v = getCellNumL(data{k, ci_complete});
    if ~isnan(v) && v == -1
        failedMask(k) = true;
    end
end
failedData = data(failedMask, :);
fprintf('재시도 대상: %d 건\n\n', size(failedData,1));

if isempty(failedData)
    fprintf('Complete=-1 행이 없습니다.\n');
    return;
end

%% 각 실패 run 재시도
for k = 1:size(failedData,1)
    result_name = getCellStrL(failedData{k, ci_result});
    modelFile   = getCellStrL(failedData{k, ci_model});
    iterNum     = round(getCellNumL(failedData{k, ci_iter}));
    modeType    = getCellStrL(failedData{k, ci_optMode});
    gaitMode    = getCellStrL(failedData{k, ci_gait});
    mocoEffort  = getCellNumL(failedData{k, ci_effort});
    mocoFT      = getCellNumL(failedData{k, ci_ft});
    timeBound   = parseVecL(getCellStrL(failedData{k, ci_tb}));
    distBound   = parseVecL(getCellStrL(failedData{k, ci_db}));

    if isempty(gaitMode),   gaitMode   = 'modeSym';  end
    if isnan(mocoEffort),   mocoEffort = 1;           end
    if isnan(mocoFT),       mocoFT     = 0.03;        end
    if isempty(timeBound),  timeBound  = [0.4, 0.8];  end
    if isempty(distBound),  distBound  = [0.4, 1.0];  end

    resultRoot = fullfile(resultsPath, result_name);
    if ~exist(resultRoot, 'dir'), mkdir(resultRoot); end

    % diary 시작 (전체 run 로그)
    logFile = fullfile(resultRoot, 'retry_log.txt');
    diary(logFile);
    diary on;

    fprintf('======================================================\n');
    fprintf(' RETRY: %s\n', result_name);
    fprintf('   model       : %s\n', modelFile);
    fprintf('   iter        : %d\n', iterNum);
    fprintf('   optMode     : %s\n', modeType);
    fprintf('   gaitMode    : %s\n', gaitMode);
    fprintf('   mocoEffort  : %.4g\n', mocoEffort);
    fprintf('   mocoFinalTime: %.4g\n', mocoFT);
    fprintf('   mocoTimeBound: %s\n', mat2str(timeBound));
    fprintf('   mocoDistBound: %s\n', mat2str(distBound));
    fprintf('======================================================\n');

    MocoOpts.mocoEffort    = mocoEffort;
    MocoOpts.mocoFinalTime = mocoFT;
    MocoOpts.gaitMode      = gaitMode;
    MocoOpts.mocoTimeBound = timeBound;
    MocoOpts.mocoDistBound = distBound;

    baseOsimPath = fullfile(modelPath, modelFile);
    [~, modelName] = fileparts(modelFile);

    % gaitMode에 따른 initial guess 경로
    if strcmpi(gaitMode, 'modeAsym')
        initGuessSto = fullfile(inputPath, 'guess_init_full.sto');
    else
        initGuessSto = fullfile(inputPath, 'guess_init_half.sto');
    end

    prevGuessSto = initGuessSto;  % iter 1 guess

    for i = 1:iterNum
        fprintf('\n----- Iteration %d / %d -----\n', i, iterNum);

        iterRootDir      = fullfile(resultRoot, sprintf('result_%d', i));
        AnalyResultDir   = fullfile(iterRootDir, 'analy_result');
        mocoResultDir    = fullfile(iterRootDir, 'moco_result');
        controlResultDir = fullfile(iterRootDir, 'control_result');
        for d = {iterRootDir, AnalyResultDir, mocoResultDir, controlResultDir}
            if ~exist(d{1}, 'dir'), mkdir(d{1}); end
        end

        %% control.sto 생성 (modeType에 따라)
        controlStoPath = fullfile(controlResultDir, 'control.sto');
        switch lower(modeType)
            case 'modeoff'
                fid = fopen(controlStoPath, 'w');
                fprintf(fid, 'controls\nnRows=2\nnColumns=3\nendheader\n');
                fprintf(fid, 'time\tAFO_r\tAFO_l\n0\t0\t0\n100\t0\t0\n');
                fclose(fid);
            otherwise
                % modeWoC / modeSpline 등은 별도 구현 필요
                warning('modeType ''%s''는 retry 스크립트에서 지원되지 않습니다. modeOff로 대체합니다.', modeType);
                fid = fopen(controlStoPath, 'w');
                fprintf(fid, 'controls\nnRows=2\nnColumns=3\nendheader\n');
                fprintf(fid, 'time\tAFO_r\tAFO_l\n0\t0\t0\n100\t0\t0\n');
                fclose(fid);
        end

        %% moco_WoC_loop 실행
        sol = moco_WoC_loop(controlStoPath, prevGuessSto, i, AnalyResultDir, baseOsimPath, MocoOpts);

        converged = sol.success();
        if converged
            fprintf('  [수렴] iter %d 정상 완료\n', i);
        else
            fprintf('  [미수렴] iter %d — unseal 후 저장\n', i);
            sol.unseal();
        end

        %% moco 결과 저장
        currOsmPath = fullfile(AnalyResultDir, sprintf('%s_%d.osim', modelName, i));
        resOpts.modelPath = currOsmPath;
        resOpts.prefix    = sprintf('moco_WoC_Solution_iter%02d', i);
        resOpts.gaitMode  = gaitMode;
        try
            moco_WoC_getResult(sol, mocoResultDir, resOpts);
        catch ME
            fprintf('  [WARN] moco_WoC_getResult 실패: %s\n', ME.message);
            % fallback: trajectory만 직접 저장
            rawStoPath = fullfile(mocoResultDir, sprintf('moco_WoC_Solution_iter%02d_kinematics_raw.sto', i));
            try
                sol.write(rawStoPath);
                fprintf('  → raw trajectory 저장: %s\n', rawStoPath);
            catch ME2
                fprintf('  [WARN] raw 저장도 실패: %s\n', ME2.message);
            end
        end

        %% 다음 iter guess 결정
        if converged
            if strcmpi(gaitMode, 'modeAsym')
                nextGuess = fullfile(mocoResultDir, sprintf('moco_WoC_Solution_iter%02d_kinematics.sto', i));
            else
                nextGuess = fullfile(mocoResultDir, sprintf('moco_WoC_Solution_iter%02d_kinematics_half.sto', i));
            end
        else
            % 미수렴이면 raw 또는 kinematics 시도, 없으면 이전 guess 유지
            rawPath = fullfile(mocoResultDir, sprintf('moco_WoC_Solution_iter%02d_kinematics_raw.sto', i));
            kinPath = fullfile(mocoResultDir, sprintf('moco_WoC_Solution_iter%02d_kinematics.sto', i));
            if isfile(kinPath)
                nextGuess = kinPath;
            elseif isfile(rawPath)
                nextGuess = rawPath;
            else
                nextGuess = prevGuessSto;  % 이전 guess 그대로 유지
                fprintf('  guess 파일 없음 — 이전 guess 재사용: %s\n', nextGuess);
            end
        end
        prevGuessSto = nextGuess;

        %% 수렴한 경우: analysis + ID
        if converged
            currKinPath = fullfile(mocoResultDir, sprintf('moco_WoC_Solution_iter%02d_kinematics.sto', i));
            try
                WoC_moco_analysis(AnalySetupPath, ...
                    'modelPath',         currOsmPath, ...
                    'kinematicsStoPath', currKinPath, ...
                    'resultsDir',        AnalyResultDir);
                fprintf('  analysis 완료\n');
            catch ME
                fprintf('  [WARN] analysis 실패: %s\n', ME.message);
            end

            % Inverse Dynamics
            kinQSto    = fullfile(AnalyResultDir, '2D_gait_AFO_pc_Kinematics_q.sto');
            grfStoFile = fullfile(mocoResultDir, sprintf('moco_WoC_Solution_iter%02d_GRF.sto', i));
            if isfile(kinQSto) && isfile(grfStoFile)
                try
                    grfRelPath = sprintf('../moco_result/moco_WoC_Solution_iter%02d_GRF.sto', i);
                    grfXmlDoc  = xmlread(grfXmlBase);
                    grfXmlDoc.getElementsByTagName('datafile').item(0).setTextContent(grfRelPath);
                    tempGrfXml = fullfile(AnalyResultDir, 'GRF_setup_id.xml');
                    xmlwrite(tempGrfXml, grfXmlDoc);

                    idTool = InverseDynamicsTool(idXmlPath);
                    idTool.setModelFileName(currOsmPath);
                    idTool.setCoordinatesFileName(kinQSto);
                    idTool.setExternalLoadsFileName(tempGrfXml);
                    idTool.setResultsDir(AnalyResultDir);
                    idTool.setOutputGenForceFileName('id_withAssist.sto');
                    idTool.run();

                    idTool2 = InverseDynamicsTool(idXmlPath);
                    idTool2.setModelFileName(baseOsimPath);
                    idTool2.setCoordinatesFileName(kinQSto);
                    idTool2.setExternalLoadsFileName(tempGrfXml);
                    idTool2.setResultsDir(AnalyResultDir);
                    idTool2.setOutputGenForceFileName('id_withoutAssist.sto');
                    idTool2.run();
                    fprintf('  ID 완료\n');
                catch ME
                    fprintf('  [WARN] ID 실패: %s\n', ME.message);
                end
            end
        else
            fprintf('  미수렴 — analysis/ID 건너뜀\n');
        end
    end % iter loop

    diary off;
    fprintf('\n로그 저장 완료: %s\n\n', logFile);

end % failed run loop

fprintf('모든 재시도 완료.\n');


%% ─── 로컬 함수 ─────────────────────────────────────────────────────────────

function val = getCellNumL(x)
    if isnumeric(x) && isscalar(x)
        val = double(x);
    elseif ischar(x) || isstring(x)
        val = str2double(char(x));
    else
        val = NaN;
    end
end

function str = getCellStrL(x)
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

function tf = isCellEmptyL(x)
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

function v = parseVecL(s)
% '[0.1 3.2]' 형식의 문자열을 숫자 벡터로 변환
    v = str2num(s); %#ok<ST2NM>
    if isempty(v), v = []; end
end
