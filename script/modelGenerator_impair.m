function modelGenerator_impair()
% modelGenerator_impair  --  근육 약화 / 섬유 길이 변경 모델 생성기
%
% =========================================================================
%  ★ 설정 영역 ★
% =========================================================================
    baseModelFiles = {                        % models/ 폴더 기준 파일명 (여러 개 지정 가능)
'2D_gait_AFO_pc_50kg_150cm_R1.osim';
'2D_gait_AFO_pc_50kg_160cm_R1.osim';
'2D_gait_AFO_pc_50kg_170cm_R1.osim';
'2D_gait_AFO_pc_50kg_180cm_R1.osim';
'2D_gait_AFO_pc_50kg_190cm_R1.osim';
'2D_gait_AFO_pc_60kg_150cm_R1.osim';
'2D_gait_AFO_pc_60kg_160cm_R1.osim';
'2D_gait_AFO_pc_60kg_170cm_R1.osim';
'2D_gait_AFO_pc_60kg_180cm_R1.osim';
'2D_gait_AFO_pc_60kg_190cm_R1.osim';
'2D_gait_AFO_pc_70kg_150cm_R1.osim';
'2D_gait_AFO_pc_70kg_160cm_R1.osim';
'2D_gait_AFO_pc_70kg_170cm_R1.osim';
'2D_gait_AFO_pc_70kg_180cm_R1.osim';
'2D_gait_AFO_pc_70kg_190cm_R1.osim';
'2D_gait_AFO_pc_80kg_150cm_R1.osim';
'2D_gait_AFO_pc_80kg_160cm_R1.osim';
'2D_gait_AFO_pc_80kg_170cm_R1.osim';
'2D_gait_AFO_pc_80kg_180cm_R1.osim';
'2D_gait_AFO_pc_80kg_190cm_R1.osim';
'2D_gait_AFO_pc_90kg_150cm_R1.osim';
'2D_gait_AFO_pc_90kg_160cm_R1.osim';
'2D_gait_AFO_pc_90kg_170cm_R1.osim';
'2D_gait_AFO_pc_90kg_180cm_R1.osim';
'2D_gait_AFO_pc_90kg_190cm_R1.osim';
'2D_gait_AFO_pc_50kg_150cm_R0.9.osim';
'2D_gait_AFO_pc_50kg_150cm_R1.1.osim';
'2D_gait_AFO_pc_50kg_160cm_R0.9.osim';
'2D_gait_AFO_pc_50kg_160cm_R1.1.osim';
'2D_gait_AFO_pc_50kg_170cm_R0.9.osim';
'2D_gait_AFO_pc_50kg_170cm_R1.1.osim';
'2D_gait_AFO_pc_50kg_180cm_R0.9.osim';
'2D_gait_AFO_pc_50kg_180cm_R1.1.osim';
'2D_gait_AFO_pc_50kg_190cm_R0.9.osim';
'2D_gait_AFO_pc_50kg_190cm_R1.1.osim';
'2D_gait_AFO_pc_60kg_150cm_R0.9.osim';
'2D_gait_AFO_pc_60kg_150cm_R1.1.osim';
'2D_gait_AFO_pc_60kg_160cm_R0.9.osim';
'2D_gait_AFO_pc_60kg_160cm_R1.1.osim';
'2D_gait_AFO_pc_60kg_170cm_R0.9.osim';
'2D_gait_AFO_pc_60kg_170cm_R1.1.osim';
'2D_gait_AFO_pc_60kg_180cm_R0.9.osim';
'2D_gait_AFO_pc_60kg_180cm_R1.1.osim';
'2D_gait_AFO_pc_60kg_190cm_R0.9.osim';
'2D_gait_AFO_pc_60kg_190cm_R1.1.osim';
'2D_gait_AFO_pc_70kg_150cm_R0.9.osim';
'2D_gait_AFO_pc_70kg_150cm_R1.1.osim';
'2D_gait_AFO_pc_70kg_160cm_R0.9.osim';
'2D_gait_AFO_pc_70kg_160cm_R1.1.osim';
'2D_gait_AFO_pc_70kg_170cm_R0.9.osim';
'2D_gait_AFO_pc_70kg_170cm_R1.1.osim';
'2D_gait_AFO_pc_70kg_180cm_R0.9.osim';
'2D_gait_AFO_pc_70kg_180cm_R1.1.osim';
'2D_gait_AFO_pc_70kg_190cm_R0.9.osim';
'2D_gait_AFO_pc_70kg_190cm_R1.1.osim';
'2D_gait_AFO_pc_80kg_150cm_R0.9.osim';
'2D_gait_AFO_pc_80kg_150cm_R1.1.osim';
'2D_gait_AFO_pc_80kg_160cm_R0.9.osim';
'2D_gait_AFO_pc_80kg_160cm_R1.1.osim';
'2D_gait_AFO_pc_80kg_170cm_R0.9.osim';
'2D_gait_AFO_pc_80kg_170cm_R1.1.osim';
'2D_gait_AFO_pc_80kg_180cm_R0.9.osim';
'2D_gait_AFO_pc_80kg_180cm_R1.1.osim';
'2D_gait_AFO_pc_80kg_190cm_R0.9.osim';
'2D_gait_AFO_pc_80kg_190cm_R1.1.osim';
'2D_gait_AFO_pc_90kg_150cm_R0.9.osim';
'2D_gait_AFO_pc_90kg_150cm_R1.1.osim';
'2D_gait_AFO_pc_90kg_160cm_R0.9.osim';
'2D_gait_AFO_pc_90kg_160cm_R1.1.osim';
'2D_gait_AFO_pc_90kg_170cm_R0.9.osim';
'2D_gait_AFO_pc_90kg_170cm_R1.1.osim';
'2D_gait_AFO_pc_90kg_180cm_R0.9.osim';
'2D_gait_AFO_pc_90kg_180cm_R1.1.osim';
'2D_gait_AFO_pc_90kg_190cm_R0.9.osim';
'2D_gait_AFO_pc_90kg_190cm_R1.1.osim';
    };

    % 변경할 근육 이름 (모델 내 이름과 동일하게)
    % muscleNames = {'soleus_r'};
    % muscleNames = {'gastroc_r'};
    % muscleNames = {'soleus_r', 'gastroc_r', 'tib_ant_r'};
    % muscleNames = {'soleus_r', 'gastroc_r', 'tib_ant_r', 'rect_fem_r'};
    % muscleNames = {'soleus_r', 'gastroc_r'}; % Right PF
    % muscleNames = {'soleus_r', 'gastroc_r', 'tib_ant_r', 'rect_fem_r', 'hamstrings_r','bifemsh_r', 'glut_max_r', 'iliopsoas_r', 'vasti_r'}; % Whole right
    muscleNames = {'soleus_r', 'gastroc_r', 'tib_ant_r', 'rect_fem_r', 'hamstrings_r','bifemsh_r', 'glut_max_r', 'iliopsoas_r', 'vasti_r', ...
                   'soleus_l', 'gastroc_l', 'tib_ant_l', 'rect_fem_l', 'hamstrings_l','bifemsh_l', 'glut_max_l', 'iliopsoas_l', 'vasti_l'}; % Whole body

    % 변경 여부 선택 (둘 다 true이면 동시에 변경)
    modifyForce = true;   % max_isometric_force 변경 여부
    % modifyForce = false;   
    % modifyFiber = true;  % optimal_fiber_length 변경 여부
    modifyFiber = false;  

    % 각 행 = 하나의 모델 조합
    % 사용하지 않는 열(modifyForce=false → forceScale 무시, modifyFiber=false → fiberScale 무시)
    % forceScale  fiberScale
    force1 = 0.25;
    force2 = 0.125;
    force3 = 0.0625;
    force4 = 0.7;
    fiber1 = 0.85;
    fiber2 = 0.70;
    fiber3 = 0.55;

    % impairments = [
    %     force1   1.00;
    %     force2   1.00;
    %     force3   1.00;
    %     1.0   fiber1;
    %     1.0   fiber2;
    %     1.0   fiber3;
    % ];

    % impairments = [
    %     1.0   fiber1;
    %     1.0   fiber2;
    %     1.0   fiber3;
    % ];

    % impairments = [
    %     force1   1.00;
    %     force2   1.00;
    %     force3   1.00;
    % ];
    
    impairments = [
        force4   1.00;
    ];
    recordToXlsx = true;   % false: 파일만 생성, xlsx 기록 생략
% =========================================================================

    import org.opensim.modeling.*

    % ── 경로 설정 ──
    scriptDir  = fileparts(mfilename('fullpath'));
    projectDir = fileparts(scriptDir);
    modelsDir  = fullfile(projectDir, 'models');
    xlsxFile   = fullfile(scriptDir, 'simulation_queue.xlsx');

    if ~exist(modelsDir, 'dir'), mkdir(modelsDir); end

    assert(modifyForce || modifyFiber, 'modifyForce와 modifyFiber 중 하나는 true여야 합니다.');

    nNew = size(impairments, 1);
    totalGenerated = 0;

    for bIdx = 1:length(baseModelFiles)
        baseModelFile = baseModelFiles{bIdx};

        if ~isfile(baseModelFile)
            baseModelFile = fullfile(modelsDir, baseModelFile);
        end
        assert(isfile(baseModelFile), 'Base model not found: %s', baseModelFile);

        [~, baseName, ~] = fileparts(baseModelFile);
        originName = [baseName '.osim'];

        % ── xlsx 기존 개수 확인 (이전 base model의 업데이트를 반영하기 위해 루프마다 새로 읽음) ──
        startIdx = 7; % version 시작 명
        sheetData = {}; endHdrRow = 1; colHdrRow = 2;
        originCounts = containers.Map('KeyType','char','ValueType','double');
        if recordToXlsx
            [sheetData, endHdrRow, colHdrRow, originCounts] = parseModelsSheet(xlsxFile);
            if isKey(originCounts, originName)
                startIdx = originCounts(originName) + 1;
            end
        end
        fprintf('\n=== [%d/%d] Origin: %s  (%d~%d 생성 예정) ===\n', ...
            bIdx, length(baseModelFiles), originName, startIdx, startIdx + nNew - 1);

        % ── base model에서 원래 값 읽기 (getMuscles() 대신 component iterator 사용) ──
        % getMuscles()는 sub-component 안에 있는 근육을 찾지 못함 (cal_meta_test.m 참고)
        refModel    = Model(baseModelFile);
        origForce   = containers.Map('KeyType','char','ValueType','double');
        origFiber   = containers.Map('KeyType','char','ValueType','double');
        musclePaths = containers.Map('KeyType','char','ValueType','char');

        compList = refModel.getComponentsList();
        it = compList.begin();
        while ~it.equals(compList.end())
            comp = it.deref();
            mus  = Muscle.safeDownCast(comp);
            if ~isempty(mus)
                mName = char(mus.getName());
                if ismember(mName, muscleNames)
                    musclePaths(mName) = char(comp.getAbsolutePathString());
                    origForce(mName)   = mus.getMaxIsometricForce();
                    origFiber(mName)   = mus.getOptimalFiberLength();
                end
            end
            it.next();
        end

        fprintf('\n원본 근육 값:\n');
        for k = 1:length(muscleNames)
            mName = muscleNames{k};
            assert(isKey(musclePaths, mName), 'Muscle not found in model: %s', mName);
            fprintf('  %-22s  force=%8.2f N  fiber=%.6f m\n', ...
                mName, origForce(mName), origFiber(mName));
        end

        % ── 모델 생성 루프 ──
        newModelNames = cell(nNew, 1);
        abnormalDescs = cell(nNew, 1);

        for i = 1:nNew
            fScale = impairments(i, 1);
            bScale = impairments(i, 2);
            idx    = startIdx + i - 1;

            fprintf('\n[%d/%d] idx=%d', i, nNew, idx);
            if modifyForce, fprintf('  force×%.4f', fScale); end
            if modifyFiber, fprintf('  fiber×%.4f', bScale); end
            fprintf('\n');

            outName = sprintf('%s_v%d.osim', baseName, idx);
            outFile = fullfile(modelsDir, outName);
            newModelNames{i} = outName;

            % 모델 복사 후 근육 값 수정 (절대 경로 + updComponent 사용)
            model = Model(baseModelFile);
            for k = 1:length(muscleNames)
                mName = muscleNames{k};
                mus   = Muscle.safeDownCast(model.updComponent(musclePaths(mName)));
                if modifyForce
                    mus.setMaxIsometricForce(origForce(mName) * fScale);
                end
                if modifyFiber
                    mus.setOptimalFiberLength(origFiber(mName) * bScale);
                end
            end

            model.initSystem();
            model.print(outFile);
            fprintf('Saved: %s\n', outFile);

            % muscles / force_scale / fiber_scale 분리 저장
            musStr = strjoin(muscleNames, ',');
            if modifyForce, forceVal = fScale; else, forceVal = 1; end
            if modifyFiber, fiberVal = bScale; else, fiberVal = 1; end
            abnormalDescs{i} = {musStr, forceVal, fiberVal};
        end

        % ── xlsx 업데이트 ──
        if recordToXlsx
            updateModelsSheet(xlsxFile, newModelNames, originName, abnormalDescs, ...
                sheetData, endHdrRow, colHdrRow, originCounts);
        end

        totalGenerated = totalGenerated + nNew;
    end

    fprintf('\nAll done. %d model(s) generated from %d base model(s).\n', ...
        totalGenerated, length(baseModelFiles));
end


% =========================================================================
%  xlsx 파싱 / 업데이트
% =========================================================================

function [sheetData, endHdrRow, colHdrRow, originCounts] = parseModelsSheet(xlsxFile)
    COL_ORIGIN = 2;

    try
        sheetData = readcell(xlsxFile, 'Sheet', 'models');
    catch
        sheetData = {'endheader'; {'Names','origin','Date','thigh','shank','afo','muscles','force_scale','fiber_scale'}};
    end

    endHdrRow = [];
    for r = 1:size(sheetData, 1)
        v = sheetData{r, 1};
        if ischar(v) && strcmpi(strtrim(v), 'endheader')
            endHdrRow = r;
            break;
        end
    end
    if isempty(endHdrRow), endHdrRow = 1; end
    colHdrRow = endHdrRow + 1;

    originCounts = containers.Map('KeyType','char','ValueType','double');
    for r = 1:endHdrRow - 1
        name = sheetData{r, 1};
        cnt  = sheetData{r, 2};
        if ischar(name) && ~isempty(name) && isnumeric(cnt) && ~isnan(cnt)
            originCounts(name) = cnt;
        end
    end

    dataCounts = containers.Map('KeyType','char','ValueType','double');
    for r = colHdrRow + 1 : size(sheetData, 1)
        if size(sheetData, 2) < COL_ORIGIN, continue; end
        orig = sheetData{r, COL_ORIGIN};
        if ~ischar(orig) || isempty(orig), continue; end
        if isKey(dataCounts, orig)
            dataCounts(orig) = dataCounts(orig) + 1;
        else
            dataCounts(orig) = 1;
        end
    end
    keys_dc = keys(dataCounts);
    for k = 1:length(keys_dc)
        originCounts(keys_dc{k}) = dataCounts(keys_dc{k});
    end
end


function updateModelsSheet(xlsxFile, newModelNames, originName, abnormalDescs, ...
        sheetData, endHdrRow, colHdrRow, originCounts)

    dateStr = datestr(now, 'yymmdd');
    nNew    = length(newModelNames);
    COL_N   = 9;   % Names origin Date thigh shank afo muscles force_scale fiber_scale

    if isKey(originCounts, originName)
        originCounts(originName) = originCounts(originName) + nNew;
    else
        originCounts(originName) = nNew;
    end

    existingData = {};
    for r = colHdrRow + 1 : size(sheetData, 1)
        row = sheetData(r, 1:min(end, COL_N));
        allEmpty = true;
        for c = 1:length(row)
            if ~isCellMissing(row{c}), allEmpty = false; break; end
        end
        if ~allEmpty
            existingData{end+1} = padRow(row, COL_N);
        end
    end

    newData = cell(nNew, COL_N);
    for i = 1:nNew
        newData{i, 1} = newModelNames{i};
        newData{i, 2} = originName;
        newData{i, 3} = dateStr;
        % cols 4,5,6 (thigh, shank, afo): 해당 없으므로 비워둠
        desc = abnormalDescs{i};   % {muscles_csv, force_scale, fiber_scale}
        newData{i, 7} = desc{1};   % muscles (comma-separated)
        newData{i, 8} = desc{2};   % force_scale (1 = 변경 없음)
        newData{i, 9} = desc{3};   % fiber_scale (1 = 변경 없음)
    end

    originKeys    = keys(originCounts);
    nOrigins      = length(originKeys);
    nExistingData = length(existingData);
    totalRows     = nOrigins + 1 + 1 + nExistingData + nNew;

    fullSheet = cell(totalRows, COL_N);

    for k = 1:nOrigins
        fullSheet{k, 1} = originKeys{k};
        fullSheet{k, 2} = originCounts(originKeys{k});
    end

    fullSheet{nOrigins + 1, 1} = 'endheader';

    colHeaders = {'Names','origin','Date','thigh','shank','afo','muscles','force_scale','fiber_scale'};
    for c = 1:COL_N
        fullSheet{nOrigins + 2, c} = colHeaders{c};
    end

    dataStart = nOrigins + 3;
    for r = 1:nExistingData
        fullSheet(dataStart + r - 1, :) = existingData{r};
    end

    newStart = dataStart + nExistingData;
    for r = 1:nNew
        fullSheet(newStart + r - 1, 1:COL_N) = newData(r, :);
    end

    for ri = 1:size(fullSheet, 1)
        for ci = 1:size(fullSheet, 2)
            if isCellMissing(fullSheet{ri, ci})
                fullSheet{ri, ci} = '';
            end
        end
    end

    writecell(fullSheet, xlsxFile, 'Sheet', 'models', 'Range', 'A1', ...
        'WriteMode', 'overwritesheet');
    fprintf('Updated models sheet in: %s\n', xlsxFile);
end


% =========================================================================
%  헬퍼
% =========================================================================

function tf = isCellMissing(x)
    tf = isa(x, 'missing') || (isnumeric(x) && isscalar(x) && isnan(x)) || ...
         (ischar(x) && isempty(x));
end

function row = padRow(row, n)
    if length(row) < n
        row{n} = missing;
    end
end
