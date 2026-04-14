% WoC_moco_run_queue
%
% CSV 시뮬레이션 대기열을 읽어 WoC_moco_main을 순차 실행하는 스크립트.
% 각 행의 시뮬레이션이 완료되면 Complete 열에 1을 기록하여 CSV를 저장한다.
% 오류 발생 시 Complete 열에 -1을 기록하고 다음 행으로 진행한다.
%
% CSV 열 구성:
%   model         : 모델 파일명 (예: 2D_gait_AFO_pc.osim)
%   iter          : 반복 횟수
%   optMode_type  : 'modeWoC' | 'modeOff' | 'modeSpline'
%   trigger       : (modeSpline) 제어 시작 시각, 나머지는 비워둠
%   rise          : (modeSpline) 상승 구간 길이
%   flat          : (modeSpline) 최대값 유지 구간 길이
%   fall          : (modeSpline) 하강 구간 길이
%   maxVal        : (modeSpline) 최대 출력값
%   result_name   : 결과 저장 폴더명
%   QP_effort     : opts.QP_effort  (빈 칸이면 기본값 사용)
%   QP_smooth     : opts.QP_smooth
%   cost          : opts.cost       (예: et | etw | etv)
%   mocoEffort    : opts.mocoEffort
%   mocoFinalTime : opts.mocoFinalTime
%   gaitMode      : opts.gaitMode   (modeSym | modeAsym)
%   resume_name   : optResume.resume_name (빈 칸이면 resume 없음)
%   Complete      : 0 = 미실행, 1 = 완료, -1 = 오류
%
% 사용법:
%   1) queue_csv_path 변수를 대기열 CSV 파일 경로로 설정
%   2) 스크립트 실행

clear; close all;

%% ── 설정 ──────────────────────────────────────────────────────────────────
queue_csv_path = 'simulation_queue.csv';   % ← CSV 경로를 여기에 지정

%% ── CSV 읽기 ───────────────────────────────────────────────────────────────
if ~isfile(queue_csv_path)
    error('대기열 CSV 파일을 찾을 수 없습니다: %s', queue_csv_path);
end

% endheader 행을 찾아 그 다음 줄을 열 이름, 이후를 데이터로 읽기
raw_lines = readlines(queue_csv_path);
endheader_idx = find(startsWith(strtrim(raw_lines), 'endheader'), 1);
if isempty(endheader_idx)
    col_name_line = 1;          % endheader 없으면 첫 줄을 열 이름으로 간주
    header_lines  = string.empty(0,1);
else
    col_name_line = endheader_idx + 1;
    header_lines  = raw_lines(1:endheader_idx);   % endheader 줄까지 저장
end

iopts = detectImportOptions(queue_csv_path, 'TextType', 'string');
iopts.VariableNamesLine = col_name_line;
iopts.DataLines         = [col_name_line + 1, Inf];
T = readtable(queue_csv_path, iopts);

required_cols = {'model','iter','optMode_type','result_name','Complete'};
for c = required_cols
    if ~ismember(c{1}, T.Properties.VariableNames)
        error('CSV에 필수 열 ''%s'' 가 없습니다.', c{1});
    end
end

%% ── 대기열 실행 ────────────────────────────────────────────────────────────
for i = 1:height(T)

    if T.Complete(i) ~= 0
        continue   % 이미 완료(1) 또는 오류(-1) → 건너뜀
    end

    fprintf('\n========================================\n');
    fprintf(' [%d/%d] %s — %s\n', i, height(T), T.model(i), T.result_name(i));
    fprintf('========================================\n');

    try
        % model, iter, result_name
        model       = char(T.model(i));
        iter        = T.iter(i);
        result_name = char(T.result_name(i));

        % optMode 파싱
        modeType = char(T.optMode_type(i));

        if strcmpi(modeType, 'modeSpline')
            optMode.type    = 'modeSpline';
            optMode.trigger = getNum(T, i, 'trigger');
            optMode.rise    = getNum(T, i, 'rise');
            optMode.flat    = getNum(T, i, 'flat');
            optMode.fall    = getNum(T, i, 'fall');
            optMode.maxVal  = getNum(T, i, 'maxVal');
        else
            optMode = modeType;   % 'modeWoC' 또는 'modeOff' 문자열 그대로
        end

        % opts 파싱 (값이 있는 필드만 설정)
        opts = struct();
        opts = setOptField(opts, T, i, 'QP_effort',     'num');
        opts = setOptField(opts, T, i, 'QP_smooth',     'num');
        opts = setOptField(opts, T, i, 'cost',           'str');
        opts = setOptField(opts, T, i, 'mocoEffort',     'num');
        opts = setOptField(opts, T, i, 'mocoFinalTime',  'num');
        opts = setOptField(opts, T, i, 'gaitMode',       'str');

        % optResume 파싱
        optResume = struct();
        if ismember('resume_name', T.Properties.VariableNames)
            rn = T.resume_name(i);
            if ~ismissing(rn) && strlength(rn) > 0
                optResume.resume_name = char(rn);
            end
        end

        % WoC_moco_main 호출
        if isfield(optResume, 'resume_name')
            WoC_moco_main(model, iter, optMode, result_name, opts, optResume);
        elseif ~isempty(fieldnames(opts))
            WoC_moco_main(model, iter, optMode, result_name, opts);
        else
            WoC_moco_main(model, iter, optMode, result_name);
        end

        % 완료 표시
        T.Complete(i) = 1;
        fprintf('\n[완료] 행 %d 저장 중...\n', i);

    catch ME
        fprintf('\n[오류] 행 %d: %s\n', i, ME.message);
        T.Complete(i) = -1;
    end

    % CSV 즉시 저장 (헤더 줄 보존)
    saveTableWithHeader(T, queue_csv_path, header_lines);
end

fprintf('\n모든 대기열 처리 완료.\n');

%% ── 로컬 함수 ─────────────────────────────────────────────────────────────

function saveTableWithHeader(T, csv_path, header_lines)
% 헤더 줄(endheader 포함)을 유지하면서 테이블을 CSV로 저장.
% 숫자 열의 NaN은 빈칸으로 저장한다.

    % NaN → 빈칸 변환: 숫자 열을 string으로 바꾸되 NaN만 ""로 대체
    T_write = T;
    for col = T_write.Properties.VariableNames
        v = T_write.(col{1});
        if isnumeric(v)
            s = string(v);
            s(isnan(v)) = "";
            T_write.(col{1}) = s;
        end
    end

    % 테이블을 임시 파일에 먼저 쓰기
    tmp_file = [tempname '.csv'];
    writetable(T_write, tmp_file);
    table_lines = readlines(tmp_file);
    delete(tmp_file);

    % 끝의 빈 줄 제거
    table_lines = table_lines(strlength(table_lines) > 0);

    % 헤더 + 테이블 합쳐서 쓰기
    fid = fopen(csv_path, 'w');
    for k = 1:numel(header_lines)
        fprintf(fid, '%s\n', header_lines(k));
    end
    for k = 1:numel(table_lines)
        fprintf(fid, '%s\n', table_lines(k));
    end
    fclose(fid);
end

function val = getNum(T, i, colName)
% 숫자 열에서 값을 읽어 반환. 열이 없거나 NaN이면 오류.
    if ~ismember(colName, T.Properties.VariableNames)
        error('modeSpline에 필요한 열 ''%s'' 가 CSV에 없습니다.', colName);
    end
    raw = T.(colName)(i);
    if isnumeric(raw)
        val = raw;
    else
        val = str2double(raw);
    end
    if isnan(val)
        error('열 ''%s'' 의 값이 비어 있거나 숫자가 아닙니다.', colName);
    end
end

function opts = setOptField(opts, T, i, colName, dataType)
% 열이 존재하고 값이 있으면 opts 구조체에 필드를 추가.
    if ~ismember(colName, T.Properties.VariableNames)
        return
    end
    raw = T.(colName)(i);
    if isnumeric(raw)
        if ~isnan(raw)
            opts.(colName) = raw;
        end
    else
        % string / char 열
        if ~ismissing(raw) && strlength(raw) > 0
            if strcmp(dataType, 'num')
                v = str2double(raw);
                if ~isnan(v)
                    opts.(colName) = v;
                end
            else
                opts.(colName) = char(raw);
            end
        end
    end
end
