% gen_queue.m
%
% simulation_queue 시트에 새 queue 행을 추가하는 스크립트.
% 파라미터를 여러 값으로 지정하면 Cartesian product 로 모든 조합을 생성한다.
%
% 사용법:
%   1) 아래 "파라미터 지정" 섹션을 편집
%   2) 스크립트 실행

clear;

%% ── 파일 설정 ──────────────────────────────────────────────────────────────
QUEUE_FILE  = 'simulation_queue.xlsx';   % ← 대상 파일명을 여기서 지정

thisDir     = fileparts(mfilename('fullpath'));
QUEUE_XLSX  = fullfile(thisDir, QUEUE_FILE);
SHEET_QUEUE = 'simulation_queue';
SHEET_DONE  = 'completed_queue';

%% ── 파라미터 지정 ──────────────────────────────────────────────────────────
% ※ 여러 값을 지정하면 Cartesian product 로 모든 조합 생성.
% ※ 빈 배열([]) 또는 빈 셀({}) → xlsx 에서 해당 열 공란 처리.

% models = {
%     '2D_gait_AFO_pc_50kg_150cm_R0.9.osim'
%     '2D_gait_AFO_pc_50kg_150cm_R1.osim'
%     '2D_gait_AFO_pc_50kg_150cm_R1.1.osim'
% };
models = {};
weights = [50 60 70 80 90];
heights = [150 160 170 180 190];
Rs = {'0.9', '1', '1.1'};
idx = 1;
for w = weights
    for h = heights
        for r = 1:length(Rs)
            % % base model: no _v suffix
            % models{idx,1} = sprintf('2D_gait_AFO_pc_%dkg_%dcm_R%s.osim', ...
            %     w, h, Rs{r});
            % idx = idx + 1;
            % _v1 ~ _v10
            for v = 1:9
            % for v = 10
                models{idx,1} = sprintf('2D_gait_AFO_pc_%dkg_%dcm_R%s_v%d.osim', ...
                    w, h, Rs{r}, v);
                idx = idx + 1;
            end

        end
    end
end

p.iter          = 1;
p.optMode_type  = {'modeOff'};       % 'modeOff' | 'modeWoC' | 'modeSpline'
% p.gaitMode      = {'modeSym'};       % 'modeSym' | 'modeAsym'
p.gaitMode      = {'modeAsym'};       % 'modeSym' | 'modeAsym'
p.mocoEffort    = [1];
p.mocoFinalTime = [0.003; 0.03; 0.3; 1];
p.mocoTimeBound = {[0.1 3.2]};                % 예: {[0.4 0.8]} 또는 {[0.4 0.8],[0.3 0.9]}
p.mocoDistBound = {[0.05 8]};
p.QP_effort     = [];
p.QP_smooth     = [];
p.cost          = {};                % 'et' | 'etw' | 'etv'
p.trigger       = [];               % modeSpline 전용
p.rise          = [];
p.flat          = [];
p.fall          = [];
p.maxVal        = [];
p.resume_name   = {};

%% ── 기존 sheet 읽기 ────────────────────────────────────────────────────────
if ~isfile(QUEUE_XLSX)
    error('xlsx 파일을 찾을 수 없습니다: %s', QUEUE_XLSX);
end
[hdr_q, colNames, data_q] = readSheet(QUEUE_XLSX, SHEET_QUEUE);
[hdr_d, ~,        ~      ] = readSheet(QUEUE_XLSX, SHEET_DONE);

%% ── prefix 별 현재 최대 ID 인덱스 파악 ─────────────────────────────────────
% 1) completed_queue 헤더 기반
known_prefixes = {'SF','SW','SP','AF','AW','AP'};
id_max = struct();
for k = 1:numel(known_prefixes)
    id_max.(known_prefixes{k}) = getIDCounter(hdr_d, known_prefixes{k});
end

% 2) simulation_queue 기존 행에서도 최대 인덱스 확인
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

% 이번 세션 내 할당 카운터 (id_max 에서 이어받아 증가)
id_counters = id_max;

%% ── Cartesian product 생성 ─────────────────────────────────────────────────
param_lists         = struct();
param_lists.model   = models(:)';   % 항상 포함

numeric_fields = {'iter','mocoEffort','mocoFinalTime','QP_effort','QP_smooth', ...
                  'trigger','rise','flat','fall','maxVal'};
string_fields  = {'optMode_type','gaitMode','cost','resume_name'};
vec_fields     = {'mocoTimeBound','mocoDistBound'};

for fn = numeric_fields
    f = fn{1};
    if isfield(p, f) && ~isempty(p.(f))
        param_lists.(f) = num2cell(p.(f)(:)');
    end
end
for fn = string_fields
    f = fn{1};
    if isfield(p, f) && ~isempty(p.(f))
        if ischar(p.(f))
            param_lists.(f) = {p.(f)};
        else
            param_lists.(f) = p.(f)(:)';
        end
    end
end
for fn = vec_fields
    f = fn{1};
    if isfield(p, f) && ~isempty(p.(f))
        if iscell(p.(f))
            param_lists.(f) = p.(f)(:)';
        else
            param_lists.(f) = {p.(f)};   % 단일 벡터를 셀로 감쌈
        end
    end
end

combos = buildCombos(param_lists);
fprintf('생성할 queue 수: %d\n', numel(combos));

%% ── 새 행 생성 ─────────────────────────────────────────────────────────────
nCols    = numel(colNames);
today    = datestr(now, 'yyyy-mm-dd');
new_rows = repmat({''}, numel(combos), nCols);

for c = 1:numel(combos)
    combo = combos{c};
    row   = repmat({''}, 1, nCols);

    % gaitMode / optMode_type 먼저 파악 (ID 생성에 필요)
    gm = fieldOrDef(combo, 'gaitMode',     '');
    om = fieldOrDef(combo, 'optMode_type', '');

    % ── ID 자동 생성 ────────────────────────────────────────────────────
    prefix = makeIDPrefix(gm, om);
    if ~isfield(id_counters, prefix), id_counters.(prefix) = 0; end
    id_counters.(prefix) = id_counters.(prefix) + 1;
    iter_val = fieldOrDef(combo, 'iter', 0);
    if isnan(iter_val), iter_val = 0; end
    id_str = sprintf('%s%03d-%d', prefix, id_counters.(prefix), iter_val);

    % ── 고정 메타 열 ────────────────────────────────────────────────────
    row = wcol(row, colNames, 'ID',          id_str);
    row = wcol(row, colNames, 'Date',        today);
    row = wcol(row, colNames, 'result_name', id_str);  % ID 와 동일

    % ── 문자열 필드 ─────────────────────────────────────────────────────
    for fn = {'model','optMode_type','gaitMode','cost','resume_name'}
        v = fieldOrDef(combo, fn{1}, '');
        if ~isempty(v), row = wcol(row, colNames, fn{1}, v); end
    end

    % ── 숫자 필드 ───────────────────────────────────────────────────────
    for fn = {'iter','mocoEffort','mocoFinalTime','QP_effort','QP_smooth', ...
              'trigger','rise','flat','fall','maxVal'}
        v = fieldOrDef(combo, fn{1}, NaN);
        if ~isnan(v), row = wcol(row, colNames, fn{1}, v); end
    end

    % ── 벡터 필드 (스칼라는 숫자, 벡터는 '[0.4 0.8]' 형식 문자열) ──────
    for fn = {'mocoTimeBound','mocoDistBound'}
        v = fieldOrDef(combo, fn{1}, []);
        if ~isempty(v), row = wcol(row, colNames, fn{1}, vecStr(v)); end
    end

    new_rows(c, :) = row;
end

%% ── simulation_queue 에 append 후 저장 ─────────────────────────────────────
writeSheet(QUEUE_XLSX, SHEET_QUEUE, hdr_q, colNames, [data_q; new_rows]);
fprintf('simulation_queue 에 %d 행 추가 완료.\n', numel(combos));

fprintf('\n추가된 ID 목록:\n');
for c = 1:numel(combos)
    fprintf('  %s\n', new_rows{c, colIdx(colNames,'ID')});
end


%% ══ 로컬 함수 ══════════════════════════════════════════════════════════════

function combos = buildCombos(param_lists)
% 각 파라미터 값 목록을 Cartesian product 로 전개.
% param_lists : struct (field → 1×N cell array of values)
% combos      : 1×M cell array of structs (조합 1개 = struct 1개)
    fields = fieldnames(param_lists);
    combos = {struct()};
    for fi = 1:numel(fields)
        fname = fields{fi};
        vals  = param_lists.(fname);
        expanded = cell(1, numel(combos) * numel(vals));
        idx = 0;
        for ci = 1:numel(combos)
            for vi = 1:numel(vals)
                idx = idx + 1;
                s = combos{ci};
                s.(fname) = vals{vi};
                expanded{idx} = s;
            end
        end
        combos = expanded;
    end
end

% ─────────────────────────────────────────────────────────────────────────

function row = wcol(row, colNames, name, val)
% row 셀 배열의 name 열에 val 기입.
    ci = colIdx(colNames, name);
    if ci > 0, row{ci} = val; end
end

function val = fieldOrDef(s, fname, default_val)
% struct s 에서 fname 필드 값 읽기. 없으면 default_val 반환.
    if isfield(s, fname), val = s.(fname);
    else,                 val = default_val;
    end
end

function s = vecStr(v)
% 숫자 벡터를 xlsx 저장용 값으로 변환.
% 스칼라는 숫자 그대로, 벡터는 '[a b]' 형식 문자열.
    if isscalar(v), s = v;
    else,           s = mat2str(v);
    end
end

% ── xlsx 읽기/쓰기 헬퍼 (run_queue.m 과 동일) ──────────────────────────────

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
        dat = data(:, 1:min(nCols,size(data,2)));
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
