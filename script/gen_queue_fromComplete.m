% gen_queue_fromComplete.m
%
% completed_queue 시트의 modeOff 완료 행을 읽어,
% 각 행에 대응하는 modeSpline queue 행을 simulation_queue 에 추가하는 스크립트.
%
% 사용법:
%   1) 아래 "설정" 섹션을 편집
%   2) 스크립트 실행

clear;

%% ── 설정 ──────────────────────────────────────────────────────────────────
QUEUE_FILE = 'simulation_queue_example.xlsx';   % ← 대상 파일명

% completed_queue 데이터 행 기준(헤더 제외) 이 행부터 끝까지 처리
START_ROW = 1;

% modeSpline 파라미터 세트 — 여러 개 지정하면 소스 행 1개당 N개 생성
spline_params = {
    struct('trigger', 0.1, 'rise', 0.1, 'flat', 0.1, 'fall', 0.1, 'maxVal', 1.0)
    struct('trigger', 0.1, 'rise', 0.2, 'flat', 0.3, 'fall', 0.1, 'maxVal', 0.5)
};

%% ── 파일 읽기 ──────────────────────────────────────────────────────────────
thisDir     = fileparts(mfilename('fullpath'));
QUEUE_XLSX  = fullfile(thisDir, QUEUE_FILE);
SHEET_QUEUE = 'simulation_queue';
SHEET_DONE  = 'completed_queue';

if ~isfile(QUEUE_XLSX)
    error('xlsx 파일을 찾을 수 없습니다: %s', QUEUE_XLSX);
end

[hdr_q, colNames, data_q] = readSheet(QUEUE_XLSX, SHEET_QUEUE);
[hdr_d, ~,        data_d] = readSheet(QUEUE_XLSX, SHEET_DONE);

%% ── 소스 행 필터링 ─────────────────────────────────────────────────────────
if START_ROW > size(data_d, 1)
    error('START_ROW(%d)가 completed_queue 행 수(%d)를 초과합니다.', ...
        START_ROW, size(data_d, 1));
end
data_src = data_d(START_ROW:end, :);

% modeOff 행만 선택
ci_om = colIdx(colNames, 'optMode_type');
is_modeOff = false(size(data_src, 1), 1);
for k = 1:size(data_src, 1)
    if ci_om > 0 && strcmpi(getCellStr(data_src{k, ci_om}), 'modeOff')
        is_modeOff(k) = true;
    end
end
data_src = data_src(is_modeOff, :);

n_src    = size(data_src, 1);
n_spline = numel(spline_params);
fprintf('modeOff 소스 행: %d  ×  spline 세트: %d  →  총 %d 행 생성\n', ...
    n_src, n_spline, n_src * n_spline);

%% ── ID 카운터 초기화 ────────────────────────────────────────────────────────
known_prefixes = {'SF','SW','SP','AF','AW','AP'};
id_max = struct();
for k = 1:numel(known_prefixes)
    id_max.(known_prefixes{k}) = getIDCounter(hdr_d, known_prefixes{k});
end

ci_id = colIdx(colNames, 'ID');
if ci_id > 0
    for k = 1:size(data_q, 1)
        existing_id = getCellStr(data_q{k, ci_id});
        tok = regexp(existing_id, '^([A-Za-z]{2})(\d+)-', 'tokens', 'once');
        if numel(tok) == 2
            pf  = upper(tok{1});
            num = str2double(tok{2});
            if ~isfield(id_max, pf), id_max.(pf) = 0; end
            if num > id_max.(pf),    id_max.(pf) = num; end
        end
    end
end
id_counters = id_max;

%% ── 새 행 생성 ─────────────────────────────────────────────────────────────
nCols    = numel(colNames);
today    = datestr(now, 'yyyymmdd');
n_new    = n_src * n_spline;
new_rows = repmat({''}, n_new, nCols);

inherit_str = {'model','gaitMode','cost'};
inherit_num = {'iter','mocoEffort','mocoFinalTime'};
inherit_vec = {'mocoTimeBound','mocoDistBound'};

row_idx = 0;
for s = 1:n_src
    for p = 1:n_spline
        row_idx = row_idx + 1;
        row = repmat({''}, 1, nCols);
        sp  = spline_params{p};

        % gaitMode 파악 (ID prefix 계산용)
        gm = getCellStr(data_src{s, colIdx(colNames, 'gaitMode')});
        if isempty(gm), gm = 'modeSym'; end

        % ID 생성
        prefix = makeIDPrefix(gm, 'modeSpline');
        if ~isfield(id_counters, prefix), id_counters.(prefix) = 0; end
        id_counters.(prefix) = id_counters.(prefix) + 1;

        src_iter = getCellNum(data_src{s, colIdx(colNames, 'iter')});
        if isnan(src_iter), src_iter = 0; end
        id_str = sprintf('%s%03d-%d', prefix, id_counters.(prefix), round(src_iter));

        % 고정 메타
        row = wcol(row, colNames, 'ID',           id_str);
        row = wcol(row, colNames, 'Date',         today);
        row = wcol(row, colNames, 'result_name',  id_str);
        row = wcol(row, colNames, 'optMode_type', 'modeSpline');

        % 소스에서 상속: 문자열
        for fn = inherit_str
            ci = colIdx(colNames, fn{1});
            if ci > 0
                v = getCellStr(data_src{s, ci});
                if ~isempty(v), row = wcol(row, colNames, fn{1}, v); end
            end
        end

        % 소스에서 상속: 숫자
        for fn = inherit_num
            ci = colIdx(colNames, fn{1});
            if ci > 0
                v = getCellNum(data_src{s, ci});
                if ~isnan(v), row = wcol(row, colNames, fn{1}, v); end
            end
        end

        % 소스에서 상속: 벡터
        for fn = inherit_vec
            ci = colIdx(colNames, fn{1});
            if ci > 0
                v = parseVecFromCell(data_src{s, ci});
                if ~isempty(v), row = wcol(row, colNames, fn{1}, vecStr(v)); end
            end
        end

        % spline 파라미터
        for fn = {'trigger','rise','flat','fall','maxVal'}
            if isfield(sp, fn{1})
                row = wcol(row, colNames, fn{1}, sp.(fn{1}));
            end
        end

        new_rows(row_idx, :) = row;
    end
end

%% ── simulation_queue 에 append 후 저장 ─────────────────────────────────────
writeSheet(QUEUE_XLSX, SHEET_QUEUE, hdr_q, colNames, [data_q; new_rows]);
fprintf('simulation_queue 에 %d 행 추가 완료.\n', n_new);

fprintf('\n추가된 ID 목록:\n');
ci_id_out = colIdx(colNames, 'ID');
for r = 1:n_new
    fprintf('  %s\n', new_rows{r, ci_id_out});
end


%% ══ 로컬 함수 ══════════════════════════════════════════════════════════════

function row = wcol(row, colNames, name, val)
    ci = colIdx(colNames, name);
    if ci > 0, row{ci} = val; end
end

function s = vecStr(v)
    if isscalar(v), s = v;
    else,           s = mat2str(v);
    end
end

function v = parseVecFromCell(raw)
    if isnumeric(raw) && ~isempty(raw) && ~(isscalar(raw) && isnan(raw))
        v = raw;
    else
        s = getCellStr(raw);
        if isempty(s)
            v = [];
        else
            parsed = str2num(s); %#ok<ST2NM>
            if isempty(parsed), v = []; else, v = parsed; end
        end
    end
end

% ── xlsx 읽기/쓰기 헬퍼 (gen_queue.m 과 동일) ───────────────────────────────

function [header_block, col_names, data] = readSheet(xlsx_path, sheet_name)
    raw    = readcell(xlsx_path, 'Sheet', sheet_name, 'UseExcel', false);
    eh_row = find(cellfun(@(x) ischar(x) && strcmpi(strtrim(x),'endheader'), raw(:,1)), 1);
    if isempty(eh_row)
        error('시트 "%s" 에 "endheader" 행이 없습니다.', sheet_name);
    end
    header_block  = raw(1:eh_row, :);
    cn_row        = eh_row + 1;
    col_names_raw = raw(cn_row, :);
    valid  = ~cellfun(@isCellEmpty, col_names_raw);
    last_c = find(valid, 1, 'last');
    if isempty(last_c)
        error('시트 "%s" 의 열 이름 행이 비어 있습니다.', sheet_name);
    end
    col_names = col_names_raw(1:last_c);
    nCols     = last_c;
    if size(raw, 1) > cn_row
        raw_data = raw(cn_row+1:end, 1:min(nCols, size(raw,2)));
        if size(raw_data, 2) < nCols
            raw_data(:, end+1:nCols) = {missing};
        end
        non_empty = any(~cellfun(@isCellEmpty, raw_data), 2);
        data = raw_data(non_empty, :);
    else
        data = cell(0, nCols);
    end
end

function writeSheet(xlsx_path, sheet_name, header_block, col_names, data)
    nCols = numel(col_names);
    try
        old_raw  = readcell(xlsx_path, 'Sheet', sheet_name, 'UseExcel', false);
        old_rows = size(old_raw, 1);
        old_cols = size(old_raw, 2);
    catch
        old_rows = 0; old_cols = 0;
    end
    content  = assembleContent(header_block, col_names, data, nCols);
    new_rows = size(content, 1);
    new_cols = size(content, 2);
    max_rows = max(old_rows, new_rows);
    max_cols = max(old_cols, new_cols);
    if max_rows > new_rows
        content(end+1:max_rows, :) = repmat({''}, max_rows-new_rows, size(content,2));
    end
    if max_cols > size(content, 2)
        content(:, end+1:max_cols) = repmat({''}, size(content,1), max_cols-size(content,2));
    end
    writecell(content, xlsx_path, 'Sheet', sheet_name, 'UseExcel', false);
end

function content = assembleContent(header_block, col_names, data, nCols)
    hdr = header_block;
    if size(hdr,2) < nCols, hdr(:,end+1:nCols) = {''}; end
    hdr = hdr(:,1:nCols);
    cn_row = col_names(:)';
    if isempty(data)
        dat = cell(0, nCols);
    else
        dat = data(:, 1:min(nCols, size(data,2)));
        if size(dat,2) < nCols, dat(:,end+1:nCols) = {''}; end
    end
    content = [replaceMissing(hdr); replaceMissing(cn_row); replaceMissing(dat)];
end

function C = replaceMissing(C)
    for k = 1:numel(C)
        x = C{k};
        if isnumeric(x) && isscalar(x) && isnan(x)
            C{k} = '';
        elseif isa(x, 'missing')
            C{k} = '';
        elseif isstring(x) && isscalar(x) && ismissing(x)
            C{k} = '';
        end
    end
end

function cnt = getIDCounter(hdr_d, prefix)
    cnt = 0;
    for k = 1:size(hdr_d, 1)
        cell_val = hdr_d{k, 1};
        if ischar(cell_val) && strcmpi(strtrim(cell_val), prefix)
            v = getCellNum(hdr_d{k, 2});
            if ~isnan(v), cnt = v; end
            return;
        end
    end
end

function prefix = makeIDPrefix(gait_mode, opt_mode_type)
    switch lower(gait_mode)
        case 'modeasym', sym_ch = 'A';
        otherwise,       sym_ch = 'S';
    end
    switch lower(strtrim(opt_mode_type))
        case 'modeoff',    mode_ch = 'F';
        case 'modewoc',    mode_ch = 'W';
        case 'modespline', mode_ch = 'P';
        otherwise,         mode_ch = 'W';
    end
    prefix = [sym_ch, mode_ch];
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
