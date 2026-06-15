% WoC_moco_run_queue  (xlsx 버전)
%
% simulation_queue.xlsx 를 읽어 WoC_moco_main 을 순차 실행하는 스크립트.
%
% xlsx 시트 구성:
%   simulation_queue 시트:
%     A열 주석 행: default 값, ID rule 등 / endheader 행
%     열 이름 행 : ID, Date, model, iter, optMode_type, trigger, rise, flat,
%                 fall, maxVal, result_name, QP_effort, QP_smooth, cost,
%                 mocoEffort, mocoFinalTime, mocoTimeBound, mocoDistBound,
%                 gaitMode, resume_name, Complete
%     데이터 행
%
%   Completed_queue 시트:
%     헤더 카운터 행: A열=접두사(SF/SW/SP/AF/AW/AP), B열=누적 완료 수
%                    / endheader 행
%     열 이름 행 : (simulation_queue 와 동일)
%     데이터 행  : Complete=1 로 완료된 행들
%
% 동작:
%   Complete=0  → 시뮬레이션 실행
%     성공(1)   → Completed_queue 로 이동 + ID 카운터 증가
%                 simulation_queue 에서 해당 행 제거
%     실패(-1)  → simulation_queue 에 Complete=-1 로 그대로 유지
%
% 사용법:
%   1) QUEUE_XLSX 변수에 xlsx 경로 설정
%   2) 스크립트 실행

clear; close all;

%% ── 설정 ──────────────────────────────────────────────────────────────────
QUEUE_XLSX  = 'simulation_queue_example.xlsx';
SHEET_QUEUE = 'simulation_queue';
SHEET_DONE  = 'completed_queue';

% results 폴더 경로 (findModeOffGuess 에서 파일 경로 조립 시 사용)
thisScriptDir = fileparts(mfilename('fullpath'));
RESULTS_BASE  = fullfile(thisScriptDir, '..', 'results');

%% ── 읽기 ──────────────────────────────────────────────────────────────────
if ~isfile(QUEUE_XLSX)
    error('대기열 파일을 찾을 수 없습니다: %s', QUEUE_XLSX);
end

[hdr_q, colNames, data_q] = readSheet(QUEUE_XLSX, SHEET_QUEUE);
[hdr_d, ~,        data_d] = readSheet(QUEUE_XLSX, SHEET_DONE);

% 필수 열 확인
required_cols = {'ID','model','iter','optMode_type','result_name','Complete'};
for c = required_cols
    if colIdx(colNames, c{1}) == 0
        error('xlsx "simulation_queue" 시트에 필수 열 ''%s'' 가 없습니다.', c{1});
    end
end

ci_complete = colIdx(colNames, 'Complete');
ci_id       = colIdx(colNames, 'ID');
ci_model    = colIdx(colNames, 'model');
ci_result   = colIdx(colNames, 'result_name');

%% ── 대기열 실행 ────────────────────────────────────────────────────────────
rows_to_move = false(size(data_q, 1), 1);

% 자동 ID 할당용 로컬 카운터 (이번 세션에서 새로 배정한 수 추적)
local_id_counters = struct();
% hdr_d 스냅샷: 루프 중 incrementIDCounter로 hdr_d가 변해도 base 카운터는
% 세션 시작 시점 값으로 고정해야 double-counting을 방지할 수 있음
hdr_d_init = hdr_d;

for i = 1:size(data_q, 1)

    complete_val = getCellNum(data_q{i, ci_complete});
    if ~isnan(complete_val) && complete_val ~= 0
        continue   % 이미 완료(1) 또는 오류(-1) → 건너뜀
    end

    id_str    = getCellStr(data_q{i, ci_id});
    model_str = getCellStr(data_q{i, ci_model});
    res_str   = getCellStr(data_q{i, ci_result});

    % ── 자동 ID / result_name 생성 ────────────────────────────────────────
    id_auto_assigned  = false;
    res_auto_assigned = false;

    if isempty(id_str)
        gm_raw = getCellStr(data_q{i, colIdx(colNames,'gaitMode')});
        if isempty(gm_raw), gm_raw = 'modeSym'; end
        om_raw = getCellStr(data_q{i, colIdx(colNames,'optMode_type')});
        it_val = getCellNum(data_q{i, colIdx(colNames,'iter')});
        if isnan(it_val), it_val = 0; end

        auto_prefix = makeIDPrefix(gm_raw, om_raw);
        pf = auto_prefix;   % struct 필드명 (예: 'SW')
        if isfield(local_id_counters, pf)
            local_id_counters.(pf) = local_id_counters.(pf) + 1;
        else
            local_id_counters.(pf) = 1;
        end
        base_cnt = getIDCounter(hdr_d_init, auto_prefix);
        new_idx  = base_cnt + local_id_counters.(pf);

        id_str = sprintf('%s%03d-%d', auto_prefix, new_idx, it_val);
        data_q{i, ci_id} = id_str;
        id_auto_assigned = true;
        warning('[자동 ID 생성] 행 %d: ID가 비어 있어 자동 생성합니다 → %s', i, id_str);
    end

    if isempty(res_str)
        res_str = id_str;
        data_q{i, ci_result} = res_str;
        res_auto_assigned = true;
        warning('[자동 result_name 생성] 행 %d: result_name이 비어 있어 ID와 동일하게 설정합니다 → %s', i, res_str);
    end

    % 자동 할당된 경우 즉시 xlsx에 반영 (크래시 복구 대비)
    if id_auto_assigned || res_auto_assigned
        writeSheet(QUEUE_XLSX, SHEET_QUEUE, hdr_q, colNames, data_q(~rows_to_move, :));
    end

    fprintf('\n========================================\n');
    fprintf(' [%d/%d] ID=%s  %s — %s\n', i, size(data_q,1), id_str, model_str, res_str);
    fprintf('========================================\n');

    try
        % 이전 반복에서 남은 optMode 변수 초기화
        clear optMode

        % ── 기본 파라미터 ──────────────────────────────────────────────
        model       = getCellStr(data_q{i, colIdx(colNames,'model')});
        iter        = getCellNum(data_q{i, colIdx(colNames,'iter')});
        result_name = getCellStr(data_q{i, colIdx(colNames,'result_name')});

        % ── optMode 파싱 ───────────────────────────────────────────────
        modeType = getCellStr(data_q{i, colIdx(colNames,'optMode_type')});

        if strcmpi(modeType, 'modeSpline')
            optMode.type    = 'modeSpline';
            optMode.trigger = getSheetNum(data_q, i, colNames, 'trigger');
            optMode.rise    = getSheetNum(data_q, i, colNames, 'rise');
            optMode.flat    = getSheetNum(data_q, i, colNames, 'flat');
            optMode.fall    = getSheetNum(data_q, i, colNames, 'fall');
            optMode.maxVal  = getSheetNum(data_q, i, colNames, 'maxVal');
        else
            optMode = modeType;
        end

        % ── opts 파싱 ──────────────────────────────────────────────────
        opts = struct();
        opts = setOptField(opts, data_q(i,:), colNames, 'QP_effort',    'num');
        opts = setOptField(opts, data_q(i,:), colNames, 'QP_smooth',    'num');
        opts = setOptField(opts, data_q(i,:), colNames, 'cost',         'str');
        opts = setOptField(opts, data_q(i,:), colNames, 'mocoEffort',   'num');
        opts = setOptField(opts, data_q(i,:), colNames, 'mocoFinalTime','num');
        opts = setOptField(opts, data_q(i,:), colNames, 'gaitMode',     'str');
        opts = setOptField(opts, data_q(i,:), colNames, 'mocoTimeBound','vec');
        opts = setOptField(opts, data_q(i,:), colNames, 'mocoDistBound','vec');

        % ID / Date 메타 정보 전달
        if ~isempty(id_str),   opts.ID   = id_str;   end
        if ci_id > 0
            date_str = getCellStr(data_q{i, colIdx(colNames,'Date')});
            if ~isempty(date_str), opts.Date = date_str; end
        end

        % ── optResume 파싱 ─────────────────────────────────────────────
        optResume = struct();
        ci_resume = colIdx(colNames, 'resume_name');
        if ci_resume > 0
            rn = getCellStr(data_q{i, ci_resume});
            if ~isempty(rn)
                optResume.resume_name = rn;
            end
        end

        % ── modeOff 결과를 initial guess로 대체 (modeSpline / modeWoC 에서만) ──
        if ~strcmpi(modeType, 'modeOff')
            offGuessPath = findModeOffGuess(data_d, colNames, model, opts, RESULTS_BASE);
            if ~isempty(offGuessPath)
                opts.guessInitSto = offGuessPath;
                fprintf('[initial guess] modeOff 결과 사용: %s\n', offGuessPath);
            end
        end

        % ── WoC_moco_main 호출 ─────────────────────────────────────────
        if isfield(optResume, 'resume_name')
            WoC_moco_main(model, iter, optMode, result_name, opts, optResume);
        elseif ~isempty(fieldnames(opts))
            WoC_moco_main(model, iter, optMode, result_name, opts);
        else
            WoC_moco_main(model, iter, optMode, result_name);
        end

        % ── 완료 처리: Completed_queue 로 이동 ────────────────────────
        data_q{i, ci_complete} = 1;
        rows_to_move(i) = true;

        % Completed_queue 에 행 추가
        data_d = [data_d; data_q(i, :)]; %#ok<AGROW>

        % ID 접두사 카운터 증가 (예: 'SW001-1' → 'SW')
        if numel(id_str) >= 2
            hdr_d = incrementIDCounter(hdr_d, upper(id_str(1:2)));
        end

        fprintf('\n[완료] 행 %d (%s) — Completed_queue 로 이동.\n', i, id_str);

    catch ME
        fprintf('\n[오류] 행 %d (%s): %s\n', i, id_str, ME.message);
        data_q{i, ci_complete} = -1;
    end

    % ── 즉시 저장 ──────────────────────────────────────────────────────
    remaining_q = data_q(~rows_to_move, :);
    writeSheet(QUEUE_XLSX, SHEET_QUEUE, hdr_q, colNames, remaining_q);
    writeSheet(QUEUE_XLSX, SHEET_DONE,  hdr_d, colNames, data_d);

end

fprintf('\n모든 대기열 처리 완료.\n');


%% ── 로컬 함수 ─────────────────────────────────────────────────────────────

function [header_block, col_names, data] = readSheet(xlsx_path, sheet_name)
% xlsx 시트를 읽어 header_block / col_names / data 로 분리.
% endheader 행을 기준으로 분리한다.
    raw = readcell(xlsx_path, 'Sheet', sheet_name, 'UseExcel', false);

    % endheader 행 탐색
    eh_row = find(cellfun(@(x) ischar(x) && strcmpi(strtrim(x),'endheader'), raw(:,1)), 1);
    if isempty(eh_row)
        error('시트 "%s" 에 "endheader" 행이 없습니다.', sheet_name);
    end

    header_block  = raw(1:eh_row, :);
    cn_row        = eh_row + 1;
    col_names_raw = raw(cn_row, :);

    % 유효 열 (trailing 빈 열 제거)
    valid  = ~cellfun(@isCellEmpty, col_names_raw);
    last_c = find(valid, 1, 'last');
    if isempty(last_c)
        error('시트 "%s" 의 열 이름 행이 비어 있습니다.', sheet_name);
    end
    col_names = col_names_raw(1:last_c);
    nCols     = last_c;

    % 데이터 행
    if size(raw, 1) > cn_row
        raw_data = raw(cn_row+1:end, 1:min(nCols, size(raw,2)));
        % 열 수 부족하면 보충
        if size(raw_data, 2) < nCols
            raw_data(:, end+1:nCols) = {missing};
        end
        % 완전히 빈 행 제거
        non_empty = any(~cellfun(@isCellEmpty, raw_data), 2);
        data = raw_data(non_empty, :);
    else
        data = cell(0, nCols);
    end
end

% ─────────────────────────────────────────────────────────────────────────

function writeSheet(xlsx_path, sheet_name, header_block, col_names, data)
% xlsx 시트에 header_block + col_names 행 + data 를 쓴다.
% 이전 내용보다 행 수가 적을 경우 남은 셀을 공백으로 덮어써 지운다.
    nCols = numel(col_names);

    % 기존 크기 파악
    try
        old_raw  = readcell(xlsx_path, 'Sheet', sheet_name, 'UseExcel', false);
        old_rows = size(old_raw, 1);
        old_cols = size(old_raw, 2);
    catch
        old_rows = 0;  old_cols = 0;
    end

    % 시트 전체 내용 조립
    content   = assembleContent(header_block, col_names, data, nCols);
    new_rows  = size(content, 1);
    new_cols  = size(content, 2);

    % 이전보다 작으면 여분 영역을 빈 칸으로 패딩해서 덮어씀
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

% ─────────────────────────────────────────────────────────────────────────

function content = assembleContent(header_block, col_names, data, nCols)
% header_block + col_names 행 + data 를 하나의 cell 배열로 조립.
    % header_block 열 맞추기
    hdr = header_block;
    if size(hdr, 2) < nCols
        hdr(:, end+1:nCols) = {''};
    end
    hdr = hdr(:, 1:nCols);

    % col_names 행 (1×nCols)
    cn_row = col_names(:)';

    % data 열 맞추기
    if isempty(data)
        dat = cell(0, nCols);
    else
        dat = data(:, 1:min(nCols, size(data,2)));
        if size(dat, 2) < nCols
            dat(:, end+1:nCols) = {''};
        end
    end

    % missing → '' 변환 후 조립
    content = [replaceMissing(hdr); replaceMissing(cn_row); replaceMissing(dat)];
end

% ─────────────────────────────────────────────────────────────────────────

function C = replaceMissing(C)
% missing / NaN 셀을 '' 으로 대체.
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

% ─────────────────────────────────────────────────────────────────────────

function hdr = incrementIDCounter(hdr, prefix)
% Completed_queue 헤더에서 prefix(예:'SW') 행의 B열 카운터를 1 증가.
% 없으면 endheader 앞에 새 행을 추가한다.
    for k = 1:size(hdr, 1)
        cell_val = hdr{k, 1};
        if ischar(cell_val) && strcmpi(strtrim(cell_val), prefix)
            cur = getCellNum(hdr{k, 2});
            if isnan(cur), cur = 0; end
            hdr{k, 2} = cur + 1;
            return;
        end
    end
    % 없으면 endheader 앞에 삽입
    eh_row = find(cellfun(@(x) ischar(x) && strcmpi(strtrim(x),'endheader'), hdr(:,1)), 1);
    new_row        = repmat({''}, 1, size(hdr, 2));
    new_row{1}     = prefix;
    new_row{2}     = 1;
    if ~isempty(eh_row)
        hdr = [hdr(1:eh_row-1,:); new_row; hdr(eh_row:end,:)];
    else
        hdr = [hdr; new_row];
    end
end

% ─────────────────────────────────────────────────────────────────────────

function idx = colIdx(col_names, name)
% 열 이름 배열에서 name 의 인덱스 반환. 없으면 0.
    idx = find(strcmp(col_names, name), 1);
    if isempty(idx), idx = 0; end
end

% ─────────────────────────────────────────────────────────────────────────

function val = getCellNum(x)
% readcell 값에서 숫자 추출. 없으면 NaN.
    if isnumeric(x) && isscalar(x)
        val = double(x);
    elseif ischar(x) || isstring(x)
        val = str2double(char(x));
    else
        val = NaN;
    end
end

function str = getCellStr(x)
% readcell 값에서 문자열 추출.
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
% readcell 값이 비어 있는지 확인.
    if isnumeric(x) || islogical(x)
        tf = isempty(x) || (isscalar(x) && isnan(x));
    elseif ischar(x)
        tf = isempty(strtrim(x));
    elseif isstring(x)
        tf = ismissing(x) || strlength(x) == 0;
    else
        tf = true;   % missing 타입 등
    end
end

% ─────────────────────────────────────────────────────────────────────────

function val = getSheetNum(data, row_i, col_names, col_name)
% modeSpline 파라미터 전용: 값이 없으면 오류.
    idx = colIdx(col_names, col_name);
    if idx == 0
        error('modeSpline에 필요한 열 ''%s'' 가 xlsx에 없습니다.', col_name);
    end
    val = getCellNum(data{row_i, idx});
    if isnan(val)
        error('열 ''%s'' 의 값이 비어 있거나 숫자가 아닙니다.', col_name);
    end
end

function prefix = makeIDPrefix(gait_mode, opt_mode_type)
% gaitMode + optMode_type → 2자리 접두사 (예: 'SW', 'AF', 'SP')
    switch lower(gait_mode)
        case 'modeasym', sym_ch = 'A';
        otherwise,       sym_ch = 'S';   % modeSym 또는 미입력
    end
    switch lower(strtrim(opt_mode_type))
        case 'modeoff',    mode_ch = 'F';
        case 'modewoc',    mode_ch = 'W';
        case 'modespline', mode_ch = 'P';
        otherwise,         mode_ch = 'W';
    end
    prefix = [sym_ch, mode_ch];
end

% ─────────────────────────────────────────────────────────────────────────

function cnt = getIDCounter(hdr_d, prefix)
% completed_queue 헤더에서 prefix 의 현재 카운터 값을 반환. 없으면 0.
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

% ─────────────────────────────────────────────────────────────────────────

function opts = setOptField(opts, row, col_names, col_name, data_type)
% 열이 존재하고 값이 있으면 opts 구조체에 필드 추가.
% data_type: 'num' (스칼라 숫자) | 'str' (문자열) | 'vec' (벡터, 예: [0.4 0.8])
    idx = colIdx(col_names, col_name);
    if idx == 0, return; end
    raw = row{idx};
    if isCellEmpty(raw), return; end

    switch data_type
        case 'num'
            v = getCellNum(raw);
            if ~isnan(v), opts.(col_name) = v; end
        case 'str'
            s = getCellStr(raw);
            if ~isempty(s), opts.(col_name) = s; end
        case 'vec'
            if isnumeric(raw) && ~isnan(raw)
                opts.(col_name) = raw;
            else
                v = str2num(getCellStr(raw)); %#ok<ST2NM>
                if ~isempty(v), opts.(col_name) = v; end
            end
    end
end

% ─────────────────────────────────────────────────────────────────────────

function guessStoPath = findModeOffGuess(data_d, colNames, model_str, opts, resultsBase)
% completed_queue 에서 조건이 동일한 modeOff 결과를 찾아 guess 경로를 반환.
% 매칭 조건: model, gaitMode, mocoEffort, mocoFinalTime, mocoTimeBound, mocoDistBound
% 여러 개 매칭 시 마지막(가장 최근 완료) 행 사용. 없거나 파일 없으면 '' 반환.
    guessStoPath = '';
    if isempty(data_d), return; end

    % 현재 job의 유효 파라미터 (WoC_moco_main 기본값과 동일)
    cur_gaitMode      = getOptLocal(opts, 'gaitMode',      'modeSym');
    cur_mocoEffort    = getOptLocal(opts, 'mocoEffort',    1);
    cur_mocoFinalTime = getOptLocal(opts, 'mocoFinalTime', 0.03);
    cur_timeBound     = getOptLocal(opts, 'mocoTimeBound', [0.4 0.8]);
    cur_distBound     = getOptLocal(opts, 'mocoDistBound', [0.4 1.0]);

    ci_mod  = colIdx(colNames, 'model');
    ci_om   = colIdx(colNames, 'optMode_type');
    ci_gm   = colIdx(colNames, 'gaitMode');
    ci_eff  = colIdx(colNames, 'mocoEffort');
    ci_ft   = colIdx(colNames, 'mocoFinalTime');
    ci_tb   = colIdx(colNames, 'mocoTimeBound');
    ci_db   = colIdx(colNames, 'mocoDistBound');
    ci_res  = colIdx(colNames, 'result_name');
    ci_iter = colIdx(colNames, 'iter');

    last_match = 0;
    for k = 1:size(data_d, 1)
        if ci_mod > 0 && ~strcmp(getCellStr(data_d{k, ci_mod}), model_str), continue; end
        if ci_om  > 0 && ~strcmpi(getCellStr(data_d{k, ci_om}),  'modeOff'), continue; end

        gm = 'modeSym';
        if ci_gm > 0
            s = getCellStr(data_d{k, ci_gm});
            if ~isempty(s), gm = s; end
        end
        if ~strcmpi(gm, cur_gaitMode), continue; end

        if ci_eff > 0
            v = getCellNum(data_d{k, ci_eff}); if isnan(v), v = 1; end
            if abs(v - cur_mocoEffort) > 1e-9, continue; end
        end
        if ci_ft > 0
            v = getCellNum(data_d{k, ci_ft}); if isnan(v), v = 0.03; end
            if abs(v - cur_mocoFinalTime) > 1e-9, continue; end
        end
        if ci_tb > 0
            v = parseVecFromCell(data_d{k, ci_tb}, [0.4 0.8]);
            if ~vecEq(v, cur_timeBound), continue; end
        end
        if ci_db > 0
            v = parseVecFromCell(data_d{k, ci_db}, [0.4 1.0]);
            if ~vecEq(v, cur_distBound), continue; end
        end

        last_match = k;
    end

    if last_match == 0, return; end

    res_name = getCellStr(data_d{last_match, ci_res});
    iter_off = getCellNum(data_d{last_match, ci_iter});
    if isempty(res_name) || isnan(iter_off) || iter_off < 1, return; end
    iter_off = round(iter_off);

    mocoDir = fullfile(resultsBase, res_name, sprintf('result_%d', iter_off), 'moco_result');
    if strcmpi(cur_gaitMode, 'modeSym')
        candidate = fullfile(mocoDir, sprintf('moco_WoC_Solution_iter%02d_kinematics_half.sto', iter_off));
    else
        candidate = fullfile(mocoDir, sprintf('moco_WoC_Solution_iter%02d_kinematics.sto', iter_off));
    end

    if isfile(candidate)
        guessStoPath = candidate;
    else
        warning('[findModeOffGuess] 파일 없음 (매칭 성공, 파일 탐색 실패): %s', candidate);
    end
end

% ─────────────────────────────────────────────────────────────────────────

function val = getOptLocal(opts, field, default_val)
    if isfield(opts, field) && ~isempty(opts.(field))
        val = opts.(field);
    else
        val = default_val;
    end
end

% ─────────────────────────────────────────────────────────────────────────

function v = parseVecFromCell(raw, default_val)
% readcell 값에서 숫자 벡터 추출. 실패하면 default_val 반환.
    if isnumeric(raw) && ~isscalar(raw)
        v = raw;
    elseif isnumeric(raw) && isscalar(raw) && ~isnan(raw)
        v = raw;
    else
        s = getCellStr(raw);
        if isempty(s)
            v = default_val;
        else
            parsed = str2num(s); %#ok<ST2NM>
            if isempty(parsed), v = default_val; else, v = parsed; end
        end
    end
end

% ─────────────────────────────────────────────────────────────────────────

function tf = vecEq(a, b)
% 두 숫자 벡터가 허용 오차(1e-9) 내에서 동일한지 확인.
    tf = (numel(a) == numel(b)) && all(abs(a(:) - b(:)) <= 1e-9);
end
