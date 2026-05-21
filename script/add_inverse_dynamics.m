% add_inverse_dynamics.m
%
% completed_queue의 기존 result에 대해 Inverse Dynamics를 실행.
% fix_analy_structure.m 실행 후 사용 (Kinematics_q.sto가 있어야 함).
%
% 각 result_i/analy_result/에 대해:
%   id_withAssist.sto   : iter에서 사용한 model로 ID 실행
%   id_withoutAssist.sto: origin model로 ID 실행
%
% 이미 id_withAssist.sto가 있으면 스킵.

clc; clear;

%% 경로 설정
if isempty(mfilename)
    thisFile = matlab.desktop.editor.getActiveFilename;
else
    thisFile = mfilename('fullpath');
end
scriptDir   = fileparts(thisFile);
projectRoot = fullfile(scriptDir, '..');
inputPath   = fullfile(projectRoot, 'inputs');
modelPath   = fullfile(projectRoot, 'models');
resultsPath = fullfile(projectRoot, 'results');
queueXlsx   = fullfile(scriptDir, 'simulation_queue_example.xlsx');

import org.opensim.modeling.*

idXmlPath  = fullfile(inputPath, 'id_setup.xml');
grfXmlBase = fullfile(inputPath, 'GRF_setup.xml');

%% completed_queue 읽기
raw = readcell(queueXlsx, 'Sheet', 'completed_queue', 'UseExcel', false);
eh_row   = find(cellfun(@(x) ischar(x) && strcmpi(strtrim(x), 'endheader'), raw(:,1)), 1);
colNames = raw(eh_row+1, :);

ci_result = find(strcmp(colNames, 'result_name'), 1);
ci_iter   = find(strcmp(colNames, 'iter'),         1);

data = raw(eh_row+2:end, :);
data = data(any(~cellfun(@isCellEmptyLocal, data), 2), :);

fprintf('completed_queue 행 수: %d\n', size(data, 1));

%% 각 result 처리
for k = 1:size(data, 1)
    result_name = getCellStrLocal(data{k, ci_result});
    iterNum     = getCellNumLocal(data{k, ci_iter});
    if isempty(result_name) || isnan(iterNum), continue; end
    iterNum = round(iterNum);

    resultRoot = fullfile(resultsPath, result_name);
    if ~exist(resultRoot, 'dir'), continue; end

    fprintf('[%s] 처리 중...\n', result_name);

    for i = 1:iterNum
        analyDir = fullfile(resultRoot, sprintf('result_%d', i), 'analy_result');
        mocoDir  = fullfile(resultRoot, sprintf('result_%d', i), 'moco_result');

        kinQSto    = fullfile(analyDir, '2D_gait_AFO_pc_Kinematics_q.sto');
        withAssist = fullfile(analyDir, 'id_withAssist.sto');

        if ~isfile(kinQSto)
            fprintf('  [SKIP] result_%d: Kinematics_q.sto 없음\n', i);
            continue;
        end
        if isfile(withAssist)
            fprintf('  [SKIP] result_%d: ID 이미 존재\n', i);
            continue;
        end

        % GRF.sto 확인
        grfStoAbs = fullfile(mocoDir, sprintf('moco_WoC_Solution_iter%02d_GRF.sto', i));
        if ~isfile(grfStoAbs)
            fprintf('  [WARN] result_%d: GRF.sto 없음 — 건너뜀\n', i);
            continue;
        end

        % iter model (.osim) 찾기
        osimFiles = dir(fullfile(analyDir, '*.osim'));
        if isempty(osimFiles)
            fprintf('  [WARN] result_%d: osim 없음 — 건너뜀\n', i);
            continue;
        end
        iterModelPath = fullfile(analyDir, osimFiles(1).name);

        % origin model: modelName_i.osim → modelName.osim
        originModelFile = regexprep(osimFiles(1).name, sprintf('_%d\\.osim$', i), '.osim');
        originModelPath = fullfile(modelPath, originModelFile);
        if ~isfile(originModelPath)
            fprintf('  [WARN] result_%d: origin model %s 없음 — 건너뜀\n', i, originModelFile);
            continue;
        end

        % GRF_setup_id.xml 생성 (datafile을 analy_result 기준 상대경로로)
        grfRelPath = sprintf('../moco_result/moco_WoC_Solution_iter%02d_GRF.sto', i);
        grfXmlDoc  = xmlread(grfXmlBase);
        grfXmlDoc.getElementsByTagName('datafile').item(0).setTextContent(grfRelPath);
        tempGrfXml = fullfile(analyDir, 'GRF_setup_id.xml');
        xmlwrite(tempGrfXml, grfXmlDoc);

        % with assist (iter model)
        idTool = InverseDynamicsTool(idXmlPath);
        idTool.setModelFileName(iterModelPath);
        idTool.setCoordinatesFileName(kinQSto);
        idTool.setExternalLoadsFileName(tempGrfXml);
        idTool.setResultsDir(analyDir);
        idTool.setOutputGenForceFileName('id_withAssist.sto');
        idTool.run();

        % without assist (origin model)
        idTool2 = InverseDynamicsTool(idXmlPath);
        idTool2.setModelFileName(originModelPath);
        idTool2.setCoordinatesFileName(kinQSto);
        idTool2.setExternalLoadsFileName(tempGrfXml);
        idTool2.setResultsDir(analyDir);
        idTool2.setOutputGenForceFileName('id_withoutAssist.sto');
        idTool2.run();

        fprintf('  result_%d ID 완료\n', i);
    end

    fprintf('  → %s 완료\n\n', result_name);
end

fprintf('모든 처리 완료.\n');


%% ─── 로컬 함수 ─────────────────────────────────────────────────────────────

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
