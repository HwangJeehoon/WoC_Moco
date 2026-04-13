function WoC_moco_writeControl(outputDir, fullTime, stanceTime, tau_R, eta, w, opts)
% WoC_moco_writeControl
%
%   fullTime(N×1), stanceTime(M×1), tau_R(M×1), eta(M×1), w(M×1)를 받아
%   1) fullTime 전체 구간에 대해 control.sto 생성
%      - 0 ~ stanceTime(1) 구간: AFO_r = 0
%      - stanceTime(1) ~ stanceTime(end) 구간: AFO_r = tau_R (interp)
%      - stanceTime(end) ~ fullTime(end) 구간: AFO_r = 0
%      - AFO_l는 전 구간 0
%   2) stanceTime, tau_R, eta, w를 data.csv로 저장
%
% opts (선택)
%   .label         : (default 'controls')
%   .columnNameR   : (default 'AFO_r')
%   .columnNameL   : (default 'AFO_l')
%   .dataColName   : w 컬럼명 override (opts 우선)

    %% 0) 입력 체크 및 옵션 처리
    if nargin < 7
        opts = struct();
    end

    if isempty(fullTime)
        error('fullTime must be non-empty.');
    end
    if isempty(stanceTime) || isempty(tau_R) || isempty(eta) || isempty(w)
        error('stanceTime, tau_R, eta, w must be non-empty.');
    end

    fullTime   = fullTime(:);
    stanceTime = stanceTime(:);
    tau_R      = tau_R(:);
    eta        = eta(:);
    w          = w(:);

    M = numel(stanceTime);
    if numel(tau_R) ~= M || numel(eta) ~= M || numel(w) ~= M
        error('stanceTime, tau_R, eta, w must all have the same length.');
    end

    if any(diff(fullTime) <= 0)
        error('fullTime must be strictly increasing.');
    end
    if any(diff(stanceTime) <= 0)
        error('stanceTime must be strictly increasing.');
    end

    if ~exist(outputDir, 'dir')
        mkdir(outputDir);
    end

    label    = getOpt(opts, 'label',       'controls');
    colNameR = getOpt(opts, 'columnNameR', 'AFO_r');
    colNameL = getOpt(opts, 'columnNameL', 'AFO_l');

    stanceStart = stanceTime(1);
    stanceEnd   = stanceTime(end);

    N = numel(fullTime);
    nColumns = 3;  % time + AFO_r + AFO_l

    %% 1) fullTime 전체에서 control 값 embed
    controlRight = zeros(N, 1);
    controlLeft  = zeros(N, 1); % 항상 0

    stanceMask = (fullTime >= stanceStart) & (fullTime <= stanceEnd);
    if any(stanceMask)
        t_in = fullTime(stanceMask);
        % stance 밖은 어차피 mask로 안 들어오므로 extrap=0은 안전장치
        u_in = interp1(stanceTime, tau_R, t_in, 'linear', 0);
        controlRight(stanceMask) = u_in;
    end

    %% 2) control.sto 파일 쓰기
    stoPath = fullfile(outputDir, 'control.sto');
    fid = fopen(stoPath, 'w');
    if fid == -1
        error('Could not open %s for writing.', stoPath);
    end

    fprintf(fid, '%s\n', label);
    fprintf(fid, 'nRows=%d\n', N);
    fprintf(fid, 'nColumns=%d\n', nColumns);
    fprintf(fid, 'endheader\n');
    fprintf(fid, 'time\t%s\t%s\n', colNameR, colNameL);

    for i = 1:N
        fprintf(fid, '%.10g\t%.10g\t%.10g\n', fullTime(i), controlRight(i), controlLeft(i));
    end
    fclose(fid);

    %% 3) data.csv 작성 (stanceTime, tau_R, eta, w)
    % 우선순위: opts.dataColName > inputname(6) > 'w'
    wColName = getOpt(opts, 'dataColName', inputname(6));
    if isempty(wColName)
        wColName = 'w';
    end
    wColName = matlab.lang.makeValidName(wColName);

    csvPath = fullfile(outputDir, 'data.csv');
    fid_csv = fopen(csvPath, 'w');
    if fid_csv == -1
        error('Could not open %s for writing.', csvPath);
    end

    fprintf(fid_csv, 'stanceTime,tau,eta,%s\n', wColName);
    for i = 1:M
        fprintf(fid_csv, '%.10g,%.10g,%.10g,%.10g\n', ...
            stanceTime(i), tau_R(i), eta(i), w(i));
    end
    fclose(fid_csv);
end

%% ---- 옵션 읽기용 헬퍼 ----
function val = getOpt(s, field, defaultVal)
    if isfield(s, field) && ~isempty(s.(field))
        val = s.(field);
    else
        val = defaultVal;
    end
end
