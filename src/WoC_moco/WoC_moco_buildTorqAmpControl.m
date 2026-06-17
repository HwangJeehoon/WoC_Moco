function [tau, t] = WoC_moco_buildTorqAmpControl(idStoPath, maxVal, cutoffHz)
% WoC_moco_buildTorqAmpControl
%
%   id_withAssist.sto 에서 ankle_angle_r_moment 를 읽어
%   plantarflexion 방향(양수)만 추출한 뒤 [0, maxVal] 로 스케일한
%   제어 프로파일을 생성한다.
%
%   처리 순서:
%     1) ankle_angle_r_moment 부호 반전 (plantarflexion = 양수)
%     2) 2차 Butterworth low-pass filter (zero-phase, filtfilt)
%     3) 음수값(dorsiflexion moment) → 0 으로 클리핑
%     4) peak 값으로 정규화 후 maxVal 스케일
%
%   입력:
%     idStoPath : id_withAssist.sto 경로
%     maxVal    : 출력 최대값 [0, 1]
%     cutoffHz  : low-pass filter 차단 주파수 (Hz, 기본값 6)
%                 0 이하로 설정하면 필터 적용 안 함
%
%   출력:
%     tau : [N×1] 제어값 (0 ~ maxVal)
%     t   : [N×1] 시간 벡터 (id_withAssist.sto 의 time 열)

    if nargin < 3 || isempty(cutoffHz)
        cutoffHz = 6;
    end

    if maxVal < 0 || maxVal > 1
        error('WoC_moco_buildTorqAmpControl: maxVal 은 [0, 1] 범위여야 합니다 (%.4f).', maxVal);
    end

    data = readSTO(idStoPath);

    if ~isfield(data, 'time')
        error('WoC_moco_buildTorqAmpControl: id_withAssist.sto 에 time 열이 없습니다.');
    end
    if ~isfield(data, 'ankle_angle_r_moment')
        error('WoC_moco_buildTorqAmpControl: id_withAssist.sto 에 ankle_angle_r_moment 열이 없습니다.');
    end

    t   = data.time(:);
    raw = -data.ankle_angle_r_moment(:);  % plantarflexion 방향을 양수로 반전

    % Low-pass filter
    fs = 1 / mean(diff(t));
    if cutoffHz > 0 && cutoffHz < fs / 2
        [b, a] = butter(2, cutoffHz / (fs / 2), 'low');
        raw = filtfilt(b, a, raw);
        fprintf('[modeTorqAmp] LPF 적용: %.4g Hz  (fs=%.1f Hz)\n', cutoffHz, fs);
    elseif cutoffHz > 0
        warning('WoC_moco_buildTorqAmpControl: cutoffHz(%.4g) >= Nyquist(%.1f). 필터 생략.', cutoffHz, fs/2);
    end

    raw(raw < 0) = 0;  % dorsiflexion moment 무시

    peakVal = max(raw);
    if peakVal <= 0
        warning('WoC_moco_buildTorqAmpControl: plantarflexion torque 가 모두 0 이하입니다. 영벡터 반환.');
        tau = zeros(numel(t), 1);
    else
        tau = (raw / peakVal) * maxVal;
    end
end


%% ---- STO 파일 읽기 헬퍼 ----
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
        fclose(fid);
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
