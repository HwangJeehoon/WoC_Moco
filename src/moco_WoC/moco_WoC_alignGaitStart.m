function info = moco_WoC_alignGaitStart(guessPath, grfPath, outDir, profile, outName)
% moco_WoC_alignGaitStart
%
%   초기 trajectory와 그에 대응하는 GRF의 작업용 사본을 만든다. 원본 파일은 읽기만 한다.
%   전진(pelvis_tx)과 좌우(pelvis_tz) 좌표가 t0에서 0이 되도록 같은 상수를 빼고,
%   GRF의 CoP x/z(ground_force_*_px, *_pz)도 같은 값만큼 옮긴다. 바닥은 수평 무한
%   평면이므로 접촉력과 동역학은 바뀌지 않는다 (속도, 힘, 자유 모멘트도 그대로).
%   모든 열은 이름으로 찾고, 나머지 열(상태, control, multiplier, slack)과 header는
%   그대로 둔다.
%
%   입력:
%     guessPath : MocoTrajectory .sto (시간 0에서 시작)
%     grfPath   : 같은 시간 격자의 GRF .sto (createExternalLoadsTableForGait 형식)
%     outDir    : 사본을 쓸 폴더 (없으면 만든다)
%     profile   : moco_WoC_modelProfile 결과 (전진 좌표 경로)
%     outName   : 사본 이름 접두어 -> <outName>_kinematics.sto, <outName>_GRF.sto
%   출력 info : 경로, offset, 시간 범위

    if ~exist(outDir, 'dir'), mkdir(outDir); end
    txPath = [profile.pelvisCoordinatePath '/value'];
    tzPath = [regexprep(profile.pelvisCoordinatePath, 'pelvis_tx$', 'pelvis_tz') '/value'];

    [hdr, labels, data] = readStoText(guessPath);
    tx = find(strcmp(labels, txPath));
    tz = find(strcmp(labels, tzPath));
    if numel(tx) ~= 1 || numel(tz) ~= 1
        error('moco_WoC_alignGaitStart: %s 또는 %s 열이 guess에 없습니다.', txPath, tzPath);
    end
    if abs(data(1, 1)) > 1e-12
        error('moco_WoC_alignGaitStart: guess가 시간 0에서 시작하지 않습니다 (t0 = %g).', data(1, 1));
    end
    info.offsetX = data(1, tx);
    info.offsetZ = data(1, tz);
    data(:, tx) = data(:, tx) - info.offsetX;
    data(:, tz) = data(:, tz) - info.offsetZ;
    info.kinematicsPath = fullfile(outDir, [outName '_kinematics.sto']);
    writeStoText(info.kinematicsPath, hdr, labels, data);

    [ghdr, glabels, gdata] = readStoText(grfPath);
    if size(gdata, 1) ~= size(data, 1) || max(abs(gdata(:, 1) - data(:, 1))) > 1e-9
        error('moco_WoC_alignGaitStart: GRF 시간 격자가 guess와 다릅니다.');
    end
    cols = struct('x', {{'ground_force_r_px', 'ground_force_l_px'}}, ...
                  'z', {{'ground_force_r_pz', 'ground_force_l_pz'}});
    for name = [cols.x, cols.z]
        if nnz(strcmp(glabels, name{1})) ~= 1
            error('moco_WoC_alignGaitStart: GRF에 %s 열이 없습니다.', name{1});
        end
    end
    for name = cols.x, gdata(:, strcmp(glabels, name{1})) = gdata(:, strcmp(glabels, name{1})) - info.offsetX; end
    for name = cols.z, gdata(:, strcmp(glabels, name{1})) = gdata(:, strcmp(glabels, name{1})) - info.offsetZ; end
    info.grfPath = fullfile(outDir, [outName '_GRF.sto']);
    writeStoText(info.grfPath, ghdr, glabels, gdata);

    info.sourceGuess = guessPath;
    info.sourceGrf = grfPath;
    info.time = [data(1, 1), data(end, 1)];
    info.numTimes = size(data, 1);
    info.pelvisTxRange = [data(1, tx), data(end, tx)];
    fid = fopen(fullfile(outDir, [outName '_alignment.txt']), 'w');
    fprintf(fid, 'source trajectory: %s\nsource GRF: %s\n', guessPath, grfPath);
    fprintf(fid, 'subtracted from %s and GRF *_px: %.17g m\n', txPath, info.offsetX);
    fprintf(fid, 'subtracted from %s and GRF *_pz: %.17g m\n', tzPath, info.offsetZ);
    fprintf(fid, 'time %.6g-%.6g s (%d points), aligned pelvis_tx %.6g -> %.6g m\n', ...
        info.time, info.numTimes, info.pelvisTxRange);
    fclose(fid);
end


function [header, labels, data] = readStoText(path)
    fid = fopen(path);
    if fid < 0, error('moco_WoC_alignGaitStart: 파일을 열 수 없습니다: %s', path); end
    cleaner = onCleanup(@() fclose(fid));
    header = {};
    while true
        line = fgetl(fid);
        if ~ischar(line), error('moco_WoC_alignGaitStart: endheader가 없습니다: %s', path); end
        if strcmp(strtrim(line), 'endheader'), break; end
        header{end+1} = line; %#ok<AGROW>
    end
    labels = strsplit(strtrim(fgetl(fid)), '\t');
    data = textscan(fid, repmat('%f', 1, numel(labels)), 'CollectOutput', true);
    data = data{1};
    if isempty(data) || any(~isfinite(data(:)))
        error('moco_WoC_alignGaitStart: 데이터가 비었거나 유한하지 않습니다: %s', path);
    end
end


function writeStoText(path, header, labels, data)
    fid = fopen(path, 'w');
    if fid < 0, error('moco_WoC_alignGaitStart: 파일을 쓸 수 없습니다: %s', path); end
    cleaner = onCleanup(@() fclose(fid));
    fprintf(fid, '%s\n', header{:});
    fprintf(fid, 'endheader\n');
    fprintf(fid, '%s\n', strjoin(labels, '\t'));
    fprintf(fid, [strjoin(repmat({'%.17g'}, 1, numel(labels)), '\t') '\n'], data');
end
