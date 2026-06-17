% gen_queue_fromComplete.m
%
% completed_queue 시트의 modeOff 완료 행을 읽어,
% 각 행에 대응하는 modeSpline / modeTorqAmp queue 행을 simulation_queue 에 추가하는 스크립트.
%
% 사용법:
%   1) 아래 "설정" 섹션을 편집
%      - modeSpline 만 생성: spline_params 채우고 torqamp_params 비워둠
%      - modeTorqAmp 만 생성: spline_params 비우고 torqamp_params 채움
%      - 둘 다 생성: 둘 다 채움
%   2) 스크립트 실행

clear;

%% ── 설정 ──────────────────────────────────────────────────────────────────
QUEUE_FILE = 'simulation_queue.xlsx';   % ← 대상 파일명

% xlsx 절대 행 번호 기준 범위 지정 (헤더 행 포함 카운트)
% END_ROW를 비워 두면 끝까지 처리
START_ROW = 1350;
END_ROW   = [];

% modeSpline 파라미터 세트 — 비워 두면 modeSpline 행을 생성하지 않음
spline_params = {
    struct('trigger', 0.27, 'rise', 0.26, 'flat', 0.0, 'fall', 0.1, 'maxVal', 1.0) % 0.5 Nm/kg
    struct('trigger', 0.27, 'rise', 0.26, 'flat', 0.0, 'fall', 0.1, 'maxVal', 0.6) % 0.3 Nm/kg
    struct('trigger', 0.27, 'rise', 0.26, 'flat', 0.0, 'fall', 0.1, 'maxVal', 0.2) % 0.1 Nm/kg
};

% modeTorqAmp 파라미터 세트 — 비워 두면 modeTorqAmp 행을 생성하지 않음
torqamp_params = {
    % struct('maxVal', 1.0)
    % struct('maxVal', 0.6)
    % struct('maxVal', 0.2)
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
% xlsx 절대 행 번호 → 데이터 인덱스 변환
% hdr_d 행 수 = endheader까지의 header block, +1은 열 이름 행
hdr_offset     = size(hdr_d, 1) + 1;  % header block + 열 이름 행
data_start_idx = START_ROW - hdr_offset;
if data_start_idx < 1, data_start_idx = 1; end
if data_start_idx > size(data_d, 1)
    error('START_ROW(%d)가 completed_queue 마지막 데이터 행(xlsx %d)을 초과합니다.', ...
        START_ROW, hdr_offset + size(data_d, 1));
end

if isempty(END_ROW)
    data_end_idx = size(data_d, 1);
else
    data_end_idx = END_ROW - hdr_offset;
    if data_end_idx > size(data_d, 1), data_end_idx = size(data_d, 1); end
end

data_src = data_d(data_start_idx:data_end_idx, :);

% modeOff 행만 선택
ci_om = colIdx(colNames, 'optMode_type');
is_modeOff = false(size(data_src, 1), 1);
for k = 1:size(data_src, 1)
    if ci_om > 0 && strcmpi(getCellStr(data_src{k, ci_om}), 'modeOff')
        is_modeOff(k) = true;
    end
end
data_src = data_src(is_modeOff, :);

n_src     = size(data_src, 1);
n_spline  = numel(spline_params);
n_torqamp = numel(torqamp_params);

if n_spline == 0 && n_torqamp == 0
    error('spline_params 와 torqamp_params 가 모두 비어 있습니다. 하나 이상 설정하세요.');
end

fprintf('modeOff 소스 행: %d\n', n_src);
if n_spline  > 0, fprintf('  modeSpline  세트: %d  →  %d 행\n', n_spline,  n_src * n_spline);  end
if n_torqamp > 0, fprintf('  modeTorqAmp 세트: %d  →  %d 행\n', n_torqamp, n_src * n_torqamp); end
fprintf('총 %d 행 생성\n', n_src * (n_spline + n_torqamp));

%% ── 새 행 생성 ─────────────────────────────────────────────────────────────
nCols    = numel(colNames);
today    = datestr(now, 'yymmdd');
n_new    = n_src * (n_spline + n_torqamp);
new_rows = repmat({''}, n_new, nCols);

inherit_str = {'model','gaitMode','cost'};
inherit_num = {'iter','mocoEffort','mocoFinalTime'};
inherit_vec = {'mocoTimeBound','mocoDistBound'};

row_idx = 0;
for s = 1:n_src

    % ── modeSpline 세트 ──
    for p = 1:n_spline
        row_idx = row_idx + 1;
        sp  = spline_params{p};
        gm  = getCellStr(data_src{s, colIdx(colNames, 'gaitMode')});
        if isempty(gm), gm = 'modeSym'; end

        if n_spline > 1, suffix = sprintf('_S%d', p); else, suffix = ''; end
        id_str = makeNewID(getCellStr(data_src{s, colIdx(colNames,'ID')}), ...
                           makeIDPrefix(gm, 'modeSpline'), suffix);

        row = repmat({''}, 1, nCols);
        row = wcol(row, colNames, 'ID',           id_str);
        row = wcol(row, colNames, 'Date',         today);
        row = wcol(row, colNames, 'result_name',  id_str);
        row = wcol(row, colNames, 'optMode_type', 'modeSpline');
        row = wcol(row, colNames, 'Complete',     0);
        row = inheritFields(row, data_src(s,:), colNames, inherit_str, inherit_num, inherit_vec);

        for fn = {'trigger','rise','flat','fall','maxVal'}
            if isfield(sp, fn{1}), row = wcol(row, colNames, fn{1}, sp.(fn{1})); end
        end
        new_rows(row_idx, :) = row;
    end

    % ── modeTorqAmp 세트 ──
    for p = 1:n_torqamp
        row_idx = row_idx + 1;
        tp  = torqamp_params{p};
        gm  = getCellStr(data_src{s, colIdx(colNames, 'gaitMode')});
        if isempty(gm), gm = 'modeSym'; end

        if n_torqamp > 1, suffix = sprintf('_T%d', p); else, suffix = ''; end
        id_str = makeNewID(getCellStr(data_src{s, colIdx(colNames,'ID')}), ...
                           makeIDPrefix(gm, 'modeTorqAmp'), suffix);

        row = repmat({''}, 1, nCols);
        row = wcol(row, colNames, 'ID',           id_str);
        row = wcol(row, colNames, 'Date',         today);
        row = wcol(row, colNames, 'result_name',  id_str);
        row = wcol(row, colNames, 'optMode_type', 'modeTorqAmp');
        row = wcol(row, colNames, 'Complete',     0);
        row = inheritFields(row, data_src(s,:), colNames, inherit_str, inherit_num, inherit_vec);

        if isfield(tp, 'maxVal'), row = wcol(row, colNames, 'maxVal', tp.maxVal); end
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

function id_str = makeNewID(src_id, prefix, suffix)
% 소스 ID 에서 번호/iter 부분을 추출해 새 prefix + suffix 로 조합.
    tok = regexp(src_id, '^[A-Za-z]{2}(\d+)-(\d+)$', 'tokens', 'once');
    if numel(tok) == 2
        num_part  = tok{1};
        iter_part = tok{2};
    else
        warning('소스 ID 파싱 실패(%s), prefix만 교체합니다.', src_id);
        num_part  = src_id(3:end);
        iter_part = '';
    end
    if isempty(iter_part)
        id_str = sprintf('%s%s%s', prefix, num_part, suffix);
    else
        id_str = sprintf('%s%s%s-%s', prefix, num_part, suffix, iter_part);
    end
end

function row = inheritFields(row, src_row, colNames, str_fields, num_fields, vec_fields)
% 소스 행에서 str/num/vec 필드를 row 에 복사.
    for fn = str_fields
        ci = colIdx(colNames, fn{1});
        if ci > 0
            v = getCellStr(src_row{ci});
            if ~isempty(v), row = wcol(row, colNames, fn{1}, v); end
        end
    end
    for fn = num_fields
        ci = colIdx(colNames, fn{1});
        if ci > 0
            v = getCellNum(src_row{ci});
            if ~isnan(v), row = wcol(row, colNames, fn{1}, v); end
        end
    end
    for fn = vec_fields
        ci = colIdx(colNames, fn{1});
        if ci > 0
            v = parseVecFromCell(src_row{ci});
            if ~isempty(v), row = wcol(row, colNames, fn{1}, vecStr(v)); end
        end
    end
end

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


function prefix = makeIDPrefix(gait_mode, opt_mode_type)
    switch lower(gait_mode)
        case 'modeasym', sym_ch = 'A';
        otherwise,       sym_ch = 'S';
    end
    switch lower(strtrim(opt_mode_type))
        case 'modeoff',     mode_ch = 'F';
        case 'modewoc',     mode_ch = 'W';
        case 'modespline',  mode_ch = 'P';
        case 'modetorqamp', mode_ch = 'T';
        otherwise,          mode_ch = 'W';
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
