% gen_guess_for_models.m
%
%   original model과 scaled model 목록을 받아,
%   다리 길이 차이(Δleg)를 계산한 뒤
%   guess_init_half.sto / guess_init_full.sto 의 pelvis_ty/value 를 보정하여
%   새로운 initial guess 파일을 inputs/ 에 저장합니다.
%
%   보정 공식:
%     pelvis_ty_new = pelvis_ty_base × (new_leg / base_leg)   [비례 스케일]
%
%     [검증] 2D_gait_AFO_pc_long (0.9 scale) / _short (1.1 scale) 와의 오차: 0.1 mm
%     (단순 Δleg 덧셈 공식은 ~8.7 mm 오차)
%
%   다리 길이 정의:
%     leg_length = |knee_r 부모 프레임 translation y| + |ankle_r 부모 프레임 translation y|
%                = femur 길이 + tibia 길이
%
%   출력 파일명:
%     inputs/guess_init_half_<model_suffix>.sto
%     inputs/guess_init_full_<model_suffix>.sto
%     (model_suffix = 입력 모델 파일명의 .osim 제거 후 base 모델명 제거한 부분)
%     예) 2D_gait_AFO_pc_v1.osim  → suffix = _v1  → guess_init_half__v1.sto
%         (원하는 이름 형식이 다르면 outputSuffix 를 수동 지정하세요)

%% =====================================================================
%%  USER CONFIGURATION
%% =====================================================================

% ── 기준 모델 (models/ 폴더 기준 파일명) ─────────────────────────────
baseModel = '2D_gait_AFO_pc.osim';

% ── 보정할 모델 목록 (models/ 폴더 기준) ─────────────────────────────
scaledModels = {
    '2D_gait_AFO_pc_v1.osim',
    '2D_gait_AFO_pc_v2.osim',
    '2D_gait_AFO_pc_v3.osim',
};

% ── 기준 guess 파일명 (inputs/ 폴더 기준) ────────────────────────────
baseGuessHalf = 'guess_init_half.sto';
baseGuessFull = 'guess_init_full.sto';

%% =====================================================================
%%  END OF CONFIGURATION
%% =====================================================================

scriptDir  = fileparts(mfilename('fullpath'));
modelsDir  = fullfile(scriptDir, '..', 'models');
inputsDir  = fullfile(scriptDir, '..', 'inputs');

%% 기준 모델 다리 길이 추출
baseOsimPath = fullfile(modelsDir, baseModel);
assert(isfile(baseOsimPath), 'Base model not found: %s', baseOsimPath);
base_leg = getLegLength(baseOsimPath);
fprintf('Base model : %s\n', baseModel);
fprintf('  leg length = %.6f m (%.4f cm)\n', base_leg, base_leg*100);

%% 기준 guess 파일 경로
halfPath = fullfile(inputsDir, baseGuessHalf);
fullPath = fullfile(inputsDir, baseGuessFull);
assert(isfile(halfPath), 'guess_init_half not found: %s', halfPath);
assert(isfile(fullPath), 'guess_init_full not found: %s', fullPath);

%% 각 scaled model 처리
[~, baseName] = fileparts(baseModel);   % e.g. '2D_gait_AFO_pc'

nModels = numel(scaledModels);
fprintf('\n처리할 모델: %d 개\n', nModels);

for k = 1:nModels
    modelFile    = scaledModels{k};
    modelOsimPath = fullfile(modelsDir, modelFile);

    if ~isfile(modelOsimPath)
        fprintf('[SKIP] 파일 없음: %s\n', modelFile);
        continue;
    end

    [~, modelName] = fileparts(modelFile);   % e.g. '2D_gait_AFO_pc_v1'

    % base 이름을 접두사로 제거 → suffix 추출
    if startsWith(modelName, baseName)
        suffix = modelName(length(baseName)+1:end);   % e.g. '_v1'
    else
        suffix = ['_' modelName];
    end

    % 다리 길이 계산 및 보정 비율 산출
    new_leg    = getLegLength(modelOsimPath);
    legRatio   = new_leg / base_leg;   % pelvis_ty 보정 비율

    fprintf('\n[%d/%d] %s\n', k, nModels, modelFile);
    fprintf('  leg length = %.6f m (%.4f cm)\n', new_leg, new_leg*100);
    fprintf('  Δleg       = %+.6f m (%+.4f cm)\n', new_leg - base_leg, (new_leg-base_leg)*100);
    fprintf('  leg ratio  = %.8f\n', legRatio);
    fprintf('  suffix     = ''%s''\n', suffix);

    % 출력 파일 경로
    outHalf = fullfile(inputsDir, sprintf('guess_init_half%s.sto', suffix));
    outFull = fullfile(inputsDir, sprintf('guess_init_full%s.sto', suffix));

    % pelvis_ty 비례 보정 후 저장
    correctSto(halfPath, outHalf, legRatio);
    correctSto(fullPath, outFull, legRatio);

    fprintf('  저장: %s\n', outHalf);
    fprintf('  저장: %s\n', outFull);
end

fprintf('\n완료.\n');


%% ─────────────────────────────────────────────────────────────────────
%  getLegLength : osim XML 에서 knee_r / ankle_r 의 부모 프레임
%                 translation y 절댓값의 합(femur + tibia)을 반환
%% ─────────────────────────────────────────────────────────────────────
function leg_len = getLegLength(osimPath)
    doc = xmlread(osimPath);
    femur = getJointParentTransY(doc, 'knee_r');
    tibia = getJointParentTransY(doc, 'ankle_r');
    leg_len = abs(femur) + abs(tibia);
end

function ty = getJointParentTransY(doc, jointName)
%   PinJoint name="<jointName>" 의 첫 번째 PhysicalOffsetFrame translation y 반환
    joints = doc.getElementsByTagName('PinJoint');
    for j = 0:joints.getLength-1
        jNode = joints.item(j);
        if ~strcmp(char(jNode.getAttribute('name')), jointName), continue; end

        % PhysicalOffsetFrame 중 첫 번째 = 부모 프레임
        frames = jNode.getElementsByTagName('PhysicalOffsetFrame');
        if frames.getLength == 0
            error('PinJoint "%s" 에서 PhysicalOffsetFrame 를 찾지 못했습니다.', jointName);
        end
        parentFrame = frames.item(0);
        tNodes = parentFrame.getElementsByTagName('translation');
        if tNodes.getLength == 0
            error('PinJoint "%s" 부모 프레임에 translation 이 없습니다.', jointName);
        end
        vals = str2double(strsplit(strtrim(char(tNodes.item(0).getTextContent()))));
        ty   = vals(2);   % y 성분
        return;
    end
    error('PinJoint name="%s" 를 찾지 못했습니다.', jointName);
end


%% ─────────────────────────────────────────────────────────────────────
%  correctSto : STO 파일을 읽어 pelvis_ty/value 컬럼을 legRatio 로 곱한 뒤
%               새 파일로 저장 (pelvis_ty_new = pelvis_ty_base × legRatio)
%% ─────────────────────────────────────────────────────────────────────
function correctSto(inPath, outPath, legRatio)
%   - 헤더 라인(endheader 포함)은 그대로 복사
%   - 컬럼명 라인은 그대로 복사
%   - 데이터 라인의 pelvis_ty/value 컬럼에만 legRatio 를 곱함

    fid = fopen(inPath, 'r');
    if fid == -1, error('Cannot open: %s', inPath); end

    lines = {};
    while ~feof(fid)
        lines{end+1} = fgetl(fid); %#ok<AGROW>
    end
    fclose(fid);

    % endheader 행 위치 탐색
    ehRow = find(cellfun(@(l) ischar(l) && strcmpi(strtrim(l), 'endheader'), lines), 1);
    if isempty(ehRow)
        error('endheader 를 찾을 수 없습니다: %s', inPath);
    end
    colRow  = ehRow + 1;    % 컬럼명 행
    dataStart = ehRow + 2;  % 데이터 첫 행

    % pelvis_ty/value 컬럼 인덱스 (1-based)
    colNames = strsplit(strtrim(lines{colRow}), '\t');
    tyIdx = find(contains(colNames, 'pelvis_ty/value'), 1);
    if isempty(tyIdx)
        error('pelvis_ty/value 컬럼을 찾을 수 없습니다: %s', inPath);
    end

    % 데이터 라인 수정
    for i = dataStart:numel(lines)
        if ~ischar(lines{i}) || isempty(strtrim(lines{i})), continue; end
        fields = strsplit(lines{i}, '\t');
        if numel(fields) < tyIdx, continue; end
        old_val = str2double(fields{tyIdx});
        fields{tyIdx} = sprintf('%.9g', old_val * legRatio);
        lines{i} = strjoin(fields, '\t');
    end

    % 출력
    fid = fopen(outPath, 'w');
    if fid == -1, error('Cannot write: %s', outPath); end
    for i = 1:numel(lines)
        if ischar(lines{i})
            fprintf(fid, '%s\n', lines{i});
        end
    end
    fclose(fid);
end
