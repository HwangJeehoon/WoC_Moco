function [eta_101, wR_101, v_101, dt] = WoC_moco_cal_QP_input( ...
    CoMPath, CoP_RPath, stanceTime, opts)
% WoC_moco_cal_QP_input
%
%   CoM, CoP_R point kinematics .sto와
%   (옵션) w_R .sto, v_R .sto를 받아
%   주어진 stanceTime 구간에 대해
%   eta, w_R, v_R을 stanceTime 기준으로 리샘플해 반환
%
% 입력
%   CoMPath    : CoM .sto
%   CoP_RPath  : CoP_R .sto
%   stanceTime : [Ns x 1] stance time 벡터
%
%   opts : struct
%     공통
%       .timeField    : default 'time'
%       .apAxis       : default 'x'
%       .vertAxis     : default 'y'
%       .targetLength : default numel(stanceTime)
%
%     좌표 필드
%       .CoMFields    : {1x3}
%       .CoP_RFields  : {1x3}
%
%     w_R 필드
%       .wPath        : w_R .sto 경로 (없으면 w=1)
%       .wField       : w_R 1D field name
%
%     v_R 필드
%       .vPath        : v_R .sto 경로 (없으면 v=0)
%       .vField       : v_R 1D field name 
%
% 출력
%   eta_101 : [targetLength x 1]
%   w_101   : [targetLength x 1]
%   v_101   : [targetLength x 1]
%   dt      : scalar, (t_end - t_start)/(targetLength-1)

    %% 0) 옵션
    if nargin < 4
        opts = struct();
    end

    if isempty(stanceTime)
        error('stanceTime must be provided and non-empty.')
    end
    t_query = stanceTime(:);
    if numel(t_query) < 2
        error('stanceTime must have at least 2 samples.')
    end

    timeField    = getOpt(opts, 'timeField',    'time');
    apAxisChar   = getOpt(opts, 'apAxis',       'x');
    vertAxisChar = getOpt(opts, 'vertAxis',     'y');
    targetLength = getOpt(opts, 'targetLength', numel(t_query));

    if ~isfield(opts,'CoMFields') || numel(opts.CoMFields) ~= 3
        error('opts.CoMFields must be 1x3 cell array.')
    end
    if ~isfield(opts,'CoP_RFields') || numel(opts.CoP_RFields) ~= 3
        error('opts.CoP_RFields must be 1x3 cell array.')
    end

    %% 1) .sto 읽기
    CoMData   = readSTO(CoMPath);
    CoP_RData = readSTO(CoP_RPath);

    if ~isfield(CoMData, timeField)
        error('Time field "%s" not found in CoM data.', timeField)
    end
    t_CoM = CoMData.(timeField)(:);

    if ~isfield(CoP_RData, timeField)
        error('Time field "%s" not found in CoP_R data.', timeField)
    end

    %% 2) stanceTime 구간 인덱싱
    tStart = max(min(t_query), min(t_CoM));
    tEnd   = min(max(t_query), max(t_CoM));
    if tStart >= tEnd
        error('stanceTime range is outside data range.')
    end

    useIdx = (t_CoM >= tStart) & (t_CoM <= tEnd);
    if sum(useIdx) < 2
        error('Not enough samples within stanceTime range.')
    end
    t_sel  = t_CoM(useIdx);
    Ns_sel = sum(useIdx);

    %% 3) CoM CoP_R 추출
    ptCoM   = extractPointXYZ(CoMData,   opts.CoMFields,   useIdx, Ns_sel, 'CoM');
    ptCoP_R = extractPointXYZ(CoP_RData, opts.CoP_RFields, useIdx, Ns_sel, 'CoP_R');

    %% 4) eta 계산
    eta_raw = local_cal_TLA_eff(ptCoM, ptCoP_R, apAxisChar, vertAxisChar);
    eta_raw = eta_raw(:);

    %% 5) w_R 계산
    w_raw = ones(Ns_sel, 1);
    if isfield(opts,'wPath') && ~isempty(opts.wPath) && isfield(opts,'wField') && ~isempty(opts.wField)
        wData = readSTO(opts.wPath);
        if ~isfield(wData, timeField)
            error('Time field "%s" not found in w data.', timeField)
        end
        w_raw = -extractSeries1D(wData, opts.wField, useIdx, Ns_sel, 'w_R'); % Opensim에서는 Plantar ankle move가 음의 방향임 -> 마이너스 추가 필요
    end

    %% 6) v_R 계산
    v_raw = zeros(Ns_sel, 1);
    if isfield(opts,'vPath') && ~isempty(opts.vPath) && isfield(opts,'vField') && ~isempty(opts.vField)
        vData = readSTO(opts.vPath);
        if ~isfield(vData, timeField)
            error('Time field "%s" not found in v data.', timeField)
        end
        v_raw = extractSeries1D(vData, opts.vField, useIdx, Ns_sel, 'v_R');
    end

    %% 7) stanceTime 기준 리샘플
    if targetLength ~= numel(t_query)
        t_query_resampled = linspace(t_query(1), t_query(end), targetLength).';
    else
        t_query_resampled = t_query;
    end

    %% 8) dt 계산
    dt = (t_query_resampled(end) - t_query_resampled(1)) / (targetLength - 1);

    %% 9) Output 정리
    eta_101 = interp1(t_sel, eta_raw, t_query_resampled, 'linear');
    wR_101   = interp1(t_sel, w_raw,   t_query_resampled, 'linear');
    v_101   = interp1(t_sel, v_raw,   t_query_resampled, 'linear');

    if any(isnan(eta_101)), eta_101 = fillmissing(eta_101,'nearest'); end
    if any(isnan(wR_101)),   wR_101   = fillmissing(wR_101,  'nearest'); end
    if any(isnan(v_101)),   v_101   = fillmissing(v_101,  'nearest'); end

end

%% ---- 3D point ----
function pt = extractPointXYZ(kinData, fieldNames, useIdx, Ns_sel, label)
    pt = zeros(Ns_sel, 3);
    lastIdx = find(useIdx, 1, 'last');
    for k = 1:3
        fname = fieldNames{k};
        if ~isfield(kinData, fname)
            error('%s: field "%s" not found.', label, fname)
        end
        v = kinData.(fname)(:);
        if numel(v) < lastIdx
            error('%s: field "%s" too short.', label, fname)
        end
        pt(:,k) = v(useIdx);
    end
end

%% ---- 1D series (w_R, v_R) ----
function s = extractSeries1D(dataStruct, fieldName, useIdx, Ns_sel, label)
    fieldName = matlab.lang.makeValidName(fieldName);
    if ~isfield(dataStruct, fieldName)
        error('%s: field "%s" not found.', label, fieldName)
    end
    v = dataStruct.(fieldName)(:);
    lastIdx = find(useIdx, 1, 'last');
    if numel(v) < lastIdx
        error('%s: field "%s" too short.', label, fieldName)
    end
    s = v(useIdx);
    if numel(s) ~= Ns_sel
        error('%s: extracted length mismatch.', label)
    end
end

%% ---- eta = cos(TLA) ----
function tla_eff = local_cal_TLA_eff(pt1, pt2, ap_axis, vert_axis)
    ap_axis   = lower(ap_axis);
    vert_axis = lower(vert_axis);

    switch ap_axis
        case 'x', idx_ap = 1;
        case 'y', idx_ap = 2;
        case 'z', idx_ap = 3;
        otherwise, error('apAxis must be x y z')
    end
    switch vert_axis
        case 'x', idx_vert = 1;
        case 'y', idx_vert = 2;
        case 'z', idx_vert = 3;
        otherwise, error('vertAxis must be x y z')
    end

    ap_diff   = pt1(:, idx_ap)   - pt2(:, idx_ap);
    vert_diff = pt1(:, idx_vert) - pt2(:, idx_vert);

    TLA = atan2(vert_diff, ap_diff);
    tla_eff = cos(TLA);
end

function val = getOpt(s, field, defaultVal)
    if isfield(s, field) && ~isempty(s.(field))
        val = s.(field);
    else
        val = defaultVal;
    end
end

function outputStructure = readSTO(filename)
    if exist(filename, 'file') ~= 2
        error('파일이 존재하지 않습니다: %s', filename);
    end

    fid = fopen(filename, 'r');
    if fid == -1
        error('파일을 열 수 없습니다: %s', filename);
    end

    headerLine = '';
    while ischar(headerLine)
        headerLine = fgetl(fid);
        if startsWith(headerLine, 'endheader')
            break;
        end
    end

    variableNamesLine = fgetl(fid);
    variableNames = strsplit(strtrim(variableNamesLine));

    data = fscanf(fid, '%f', [length(variableNames), Inf])';
    fclose(fid);

    outputStructure = struct();
    for i = 1:length(variableNames)
        fieldName = matlab.lang.makeValidName(variableNames{i});
        outputStructure.(fieldName) = data(:, i);
    end
end
