% cal_metabolic_queue.m
%
% simulation_queue.xlsx 의 completed_queue 를 읽어,
% 모든 완료된 실험에 대해 cal_metabolic 을 일괄 실행한다.
%
% 실행 방법:
%   1) 아래 SKIP_IF_EXISTS 설정 (true: 이미 저장된 결과 건너뜀)
%   2) 스크립트 실행

clear;

%% ── 설정 ──────────────────────────────────────────────────────────────────
SKIP_IF_EXISTS = true;   % true: metabolic.sto 이미 있으면 건너뜀

%% ── 경로 설정 ─────────────────────────────────────────────────────────────
if isempty(mfilename)
    thisFile = matlab.desktop.editor.getActiveFilename;
else
    thisFile = mfilename('fullpath');
end
scriptDir    = fileparts(thisFile);
QUEUE_XLSX   = fullfile(scriptDir, 'simulation_queue.xlsx');
resultsRoot  = fullfile(scriptDir, '..', 'results');
SHEET_DONE   = 'completed_queue';

%% ── completed_queue 읽기 ──────────────────────────────────────────────────
[~, colNames, data_d] = readSheet(QUEUE_XLSX, SHEET_DONE);

ci_id     = colIdx(colNames, 'ID');
ci_result = colIdx(colNames, 'result_name');
ci_iter   = colIdx(colNames, 'iter');

if ci_result == 0
    error('"completed_queue" 시트에 "result_name" 열이 없습니다.');
end
if ci_iter == 0
    error('"completed_queue" 시트에 "iter" 열이 없습니다.');
end

nRows = size(data_d, 1);
fprintf('completed_queue: %d 행 로드\n', nRows);

%% ── 일괄 실행 ─────────────────────────────────────────────────────────────
nDone = 0;
nSkip = 0;
nFail = 0;

for i = 1:nRows
    resultName = getCellStr(data_d{i, ci_result});
    iterNum    = round(getCellNum(data_d{i, ci_iter}));
    idStr      = '';
    if ci_id > 0, idStr = getCellStr(data_d{i, ci_id}); end

    if isempty(resultName)
        warning('행 %d: result_name 비어 있음, 건너뜀.', i);
        continue
    end
    if isnan(iterNum) || iterNum < 1
        warning('행 %d (%s): iter 값 이상 (%s), 건너뜀.', i, resultName, ...
                num2str(data_d{i, ci_iter}));
        continue
    end

    iterNums = 1:iterNum;

    %% 이미 결과가 있으면 건너뜀 (SKIP_IF_EXISTS=true 시)
    if SKIP_IF_EXISTS
        allExist = true;
        for ri = iterNums
            mocoDir  = fullfile(resultsRoot, resultName, sprintf('result_%d', ri), 'moco_result');
            existing = dir(fullfile(mocoDir, '*metabolic.sto'));
            if isempty(existing)
                allExist = false;
                break
            end
        end
        if allExist
            fprintf('[건너뜀] %s (iter 1~%d 모두 metabolic.sto 존재)\n', resultName, iterNum);
            nSkip = nSkip + 1;
            continue
        end
    end

    fprintf('\n============================\n');
    fprintf('[%d/%d] %s  (ID=%s, iter 1~%d)\n', i, nRows, resultName, idStr, iterNum);
    fprintf('============================\n');

    try
        cal_metabolic(resultName, iterNums, resultsRoot);
        nDone = nDone + 1;
    catch ME
        fprintf('[오류] %s : %s\n', resultName, ME.message);
        nFail = nFail + 1;
    end
end

fprintf('\n완료: %d  건너뜀: %d  실패: %d\n', nDone, nSkip, nFail);


%% ── 로컬 함수 ─────────────────────────────────────────────────────────────

function [header_block, col_names, data] = readSheet(xlsx_path, sheet_name)
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
