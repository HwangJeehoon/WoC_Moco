function [t_stance_101, t_full] = WoC_moco_detectStance(grfStoPath, opts)
% WoC_moco_detectStance
%
%   GRF .sto 파일에서 지정한 변수의 ground reaction force를 읽어
%   stance phase (GRF가 threshold 위에 있는 구간)의
%   시작(상승)과 끝(하강)을 찾아내고
%   해당 stance 구간을 101×1 time vector로 리샘플해서 반환하는 함수.
%
%   여러 stance 구간이 감지될 경우 가장 긴 구간을 사용한다.
%   t_full은 stance = gait cycle의 stanceFraction(default 0.6)이라 가정해
%   full gait cycle 구간으로 산출한다.
%
% 입력:
%   grfStoPath : GRF .sto 파일 경로
%
%   opts : struct (전부 선택)
%       .timeField     : 시간 필드 이름 (default = 'time')
%       .grfField      : GRF 값이 들어 있는 필드 이름 (default = 'ground_force_r_vy')
%       .threshold     : stance 판단용 GRF 임계값 (default = 30 N)
%       .minDuration   : stance 최소 시간(초) 조건 (default = 0)
%       .targetLength  : 출력 time vector 길이 (default = 101)
%       .stanceFraction: stance가 gait cycle에서 차지하는 비율 (default = 0.6)
%
% 출력:
%   t_stance_101 : [targetLength x 1] stance phase 구간의 시간 벡터
%   t_full       : [round(targetLength/stanceFraction) x 1] full gait cycle 시간 벡터

    %% 0) 옵션 & 데이터 읽기
    if nargin < 2
        opts = struct();
    end

    timeField      = getOpt(opts, 'timeField',      'time');
    grfField       = getOpt(opts, 'grfField',       'ground_force_r_vy');
    targetLength   = getOpt(opts, 'targetLength',   101);
    minDuration    = getOpt(opts, 'minDuration',    0.0);
    stanceFraction = getOpt(opts, 'stanceFraction', 0.6);

    grfData = readSTO(grfStoPath);

    % 시간 벡터
    if ~isfield(grfData, timeField)
        error('Time field "%s" not found in GRF data.', timeField);
    end
    t = grfData.(timeField)(:);
    if numel(t) < 2
        error('Time vector must have at least 2 samples.');
    end

    % GRF 필드
    if isempty(grfField)
        error('opts.grfField must be specified (GRF column name in .sto).');
    end
    if ~isfield(grfData, grfField)
        error('GRF field "%s" not found in GRF data.', grfField);
    end
    grf = grfData.(grfField)(:);

    if numel(grf) ~= numel(t)
        warning('Length of GRF vector and time vector are different. Using min length.');
        N = min(numel(grf), numel(t));
        grf = grf(1:N);
        t   = t(1:N);
    end

    %% 1) threshold 설정
    if isfield(opts, 'threshold') && ~isempty(opts.threshold)
        thr = opts.threshold;
    else
        % default: 30
        thr = 30;
    end

    % 이진 마스크: stance 여부
    isStance = grf > thr;

    %% 2) stance 시작/끝 인덱스 찾기 (rising / falling edge)
    % diff(isStance) == 1 : 0 → 1 변화 (stance 시작)
    % diff(isStance) == -1: 1 → 0 변화 (stance 종료)

    dMask = diff(isStance);
    startIdxCandidates = find(dMask ==  1) + 1;  % 상승 순간 index
    endIdxCandidates   = find(dMask == -1);      % 하강 순간 index

    % 에지 케이스 처리: 시작부터 stance인 경우, 첫 시작 인덱스를 1로 두기
    if isStance(1)
        startIdxCandidates = [1; startIdxCandidates(:)];
    end
    % 끝까지 stance인 경우, 마지막을 종료 인덱스로 추가
    if isStance(end)
        endIdxCandidates = [endIdxCandidates(:); numel(isStance)];
    end

    if isempty(startIdxCandidates) || isempty(endIdxCandidates)
        error('No stance phase detected with threshold = %.4f.', thr);
    end

    % startIdx < endIdx인 첫 번째 유효 페어 선택
    stancePairs = [];
    for i = 1:numel(startIdxCandidates)
        sIdx = startIdxCandidates(i);
        eIdx = endIdxCandidates(endIdxCandidates > sIdx);
        if ~isempty(eIdx)
            stancePairs = [stancePairs; [sIdx, eIdx(1)]]; %#ok<AGROW>
        end
    end

    if isempty(stancePairs)
        error('Could not find a valid (start, end) stance pair.');
    end

    % 최소 기간 조건 적용 후 가장 긴 구간 선택
    durations = t(stancePairs(:,2)) - t(stancePairs(:,1));
    validMask = durations >= minDuration;

    if any(validMask)
        validPairs     = stancePairs(validMask, :);
        validDurations = durations(validMask);
    else
        % minDuration을 만족하는 구간이 없으면 전체 중 가장 긴 것 사용
        warning('No stance pair satisfied minDuration=%.4f. Using longest stance pair.', minDuration);
        validPairs     = stancePairs;
        validDurations = durations;
    end

    [~, maxI] = max(validDurations);
    sIdx = validPairs(maxI, 1);
    eIdx = validPairs(maxI, 2);

    % stance 구간 time 서브셋
    t_stance = t(sIdx:eIdx);

    %% 3) stance time을 targetLength×1로 리샘플
    if numel(t_stance) < 2
        error('Detected stance segment is too short (<2 samples).');
    end

    t_stance_101 = linspace(t_stance(1), t_stance(end), targetLength).';

    %% 4) full gait cycle time 계산 (stance = stanceFraction of full cycle)
    stance_dur = t_stance(end) - t_stance(1);
    full_dur   = stance_dur / stanceFraction;
    n_full     = round(targetLength / stanceFraction);
    t_full     = linspace(t_stance(1), t_stance(1) + full_dur, n_full).';
end


%% ---- 옵션 읽기용 헬퍼 ----
function val = getOpt(s, field, defaultVal)
    if isfield(s, field) && ~isempty(s.(field))
        val = s.(field);
    else
        val = defaultVal;
    end
end


%% ---- STO 읽기용 헬퍼 (앞에서 쓰던 것 그대로 재사용) ----
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
    if ~ischar(variableNamesLine)
        error('변수 이름을 읽을 수 없습니다.');
    end
    variableNames = strsplit(strtrim(variableNamesLine));

    data = fscanf(fid, '%f', [length(variableNames), Inf])';
    fclose(fid);

    outputStructure = struct();
    for i = 1:length(variableNames)
        fieldName = matlab.lang.makeValidName(variableNames{i});
        outputStructure.(fieldName) = data(:, i);
    end
end
