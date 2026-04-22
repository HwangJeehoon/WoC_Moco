function modelGenerator()
% modelGenerator  --  그냥 실행하면 됨
%
% =========================================================================
%  ★ 설정 영역 ★
% =========================================================================
    baseModelFile  = '2D_gait_AFO_pc.osim';   % models/ 폴더 기준 파일명

    scalingFactors = [          % [thigh, shank] 각 행이 하나의 조합
        1.00  1.5
        1.00  0.5
        1.00  2.0
    ];
% =========================================================================

    import org.opensim.modeling.*

    % ===== 경로 =====
    scriptDir  = fileparts(mfilename('fullpath'));
    projectDir = fileparts(scriptDir);
    modelsDir  = fullfile(projectDir, 'models');
    xlsxFile   = fullfile(scriptDir, 'simulation_queue.xlsx');

    if ~exist(modelsDir, 'dir'), mkdir(modelsDir); end

    % baseModelFile 위치 확인
    if ~isfile(baseModelFile)
        baseModelFile = fullfile(modelsDir, baseModelFile);
    end
    assert(isfile(baseModelFile), 'Base model not found: %s', baseModelFile);

    [~, baseName, ~] = fileparts(baseModelFile);
    originName = [baseName '.osim'];

    % ===== xlsx에서 동일 origin의 기존 개수 → 시작 index 결정 =====
    [sheetData, endHdrRow, colHdrRow, originCounts] = parseModelsSheet(xlsxFile);
    existingCount = 0;
    if isKey(originCounts, originName)
        existingCount = originCounts(originName);
    end
    startIdx = existingCount + 1;
    fprintf('Origin: %s  (기존 %d개, v%d~v%d 생성 예정)\n', ...
        originName, existingCount, startIdx, startIdx + size(scalingFactors,1) - 1);

    % ===== AFO 벡터 저장 (스케일 전) =====
    % P2(calcn)는 local 좌표 불변 → calcn과 함께 이동
    % P1(tibia)은 "P2_new_ground + 원래 AFO 벡터(P1-P2)"로 재배치
    % → AFO의 방향·길이가 완벽히 보존됨
    refModel = Model(baseModelFile);
    refState  = refModel.initSystem();
    refAFO    = getAFOVectors(refModel, refState);

    % ===== scaling 루프 =====
    nNew          = size(scalingFactors, 1);
    newModelNames = cell(nNew, 1);

    for i = 1:nNew
        scale_thigh = scalingFactors(i, 1);
        scale_shank = scalingFactors(i, 2);
        vIdx        = startIdx + i - 1;

        fprintf('\n[%d/%d] v%d  thigh=%.4f  shank=%.4f\n', ...
            i, nNew, vIdx, scale_thigh, scale_shank);

        outName = sprintf('%s_v%d.osim', baseName, vIdx);
        outFile = fullfile(modelsDir, outName);
        newModelNames{i} = outName;

        model = Model(baseModelFile);

        scaler = ModelScaler();
        scaler.setApply(true);
        scaler.setPreserveMassDist(true);
        scaler.setPrintResultFiles(false);
        scaler.setOutputModelFileName([tempname '.osim']);
        scaler.setOutputScaleFileName([tempname '.xml']);

        order = ArrayStr();
        order.append('manualScale');
        scaler.setScalingOrder(order);

        addBodyScale(scaler, 'femur_l', scale_thigh);
        addBodyScale(scaler, 'femur_r', scale_thigh);
        addBodyScale(scaler, 'tibia_l', scale_shank);
        addBodyScale(scaler, 'tibia_r', scale_shank);

        try
            ok = scaler.processModel(model, '', -1);
        catch ME
            fprintf('processModel failed (v%d): %s\n', vIdx, ME.message);
            rethrow(ME);
        end
        assert(ok == 1, 'ModelScaler.processModel() failed (v%d).', vIdx);

        % scaled 모델 초기화 후 P1 재배치 (P2 자연 이동 + AFO 벡터 유지)
        scaledState = model.initSystem();
        restoreAFOTibiaPoints(model, scaledState, refAFO);

        model.print(outFile);
        fprintf('Saved: %s\n', outFile);
    end

    % ===== xlsx 업데이트 =====
    updateModelsSheet(xlsxFile, newModelNames, originName, scalingFactors, ...
        sheetData, endHdrRow, colHdrRow, originCounts);

    fprintf('\nAll done. %d model(s) generated.\n', nNew);
end


% =========================================================================
%  xlsx 파싱 / 업데이트
% =========================================================================

function [sheetData, endHdrRow, colHdrRow, originCounts] = parseModelsSheet(xlsxFile)
% sheetData    : readcell 결과 전체 (없으면 최소 초기값)
% endHdrRow    : 'endheader' 가 있는 행 번호
% colHdrRow    : 컬럼 헤더 행 번호 (endHdrRow + 1)
% originCounts : containers.Map  origin -> count

    COL_ORIGIN = 2;   % models 시트에서 origin 이 위치한 열

    try
        sheetData = readcell(xlsxFile, 'Sheet', 'models');
    catch
        sheetData = {'endheader'; 'Names','origin','Date','thigh','shank','afo','abnormal'};
    end

    % endheader 행 위치 찾기
    endHdrRow = [];
    for r = 1:size(sheetData, 1)
        v = sheetData{r, 1};
        if ischar(v) && strcmpi(strtrim(v), 'endheader')
            endHdrRow = r;
            break;
        end
    end
    if isempty(endHdrRow)
        endHdrRow = 1;
    end
    colHdrRow = endHdrRow + 1;

    % 헤더 영역(endheader 위)에 기록된 origin counts 읽기
    originCounts = containers.Map('KeyType','char','ValueType','double');
    for r = 1:endHdrRow - 1
        name = sheetData{r, 1};
        cnt  = sheetData{r, 2};
        if ischar(name) && ~isempty(name) && isnumeric(cnt) && ~isnan(cnt)
            originCounts(name) = cnt;
        end
    end

    % 데이터 행에서 origin 개수 재집계 (헤더와 일치 여부 검증용으로도 활용)
    for r = colHdrRow + 1 : size(sheetData, 1)
        if size(sheetData, 2) < COL_ORIGIN, continue; end
        orig = sheetData{r, COL_ORIGIN};
        if ~ischar(orig) || isempty(orig), continue; end
        if ~isKey(originCounts, orig)
            originCounts(orig) = 0;
        end
    end
    % 실제 데이터 행 수로 덮어씀 (헤더와 데이터가 불일치할 경우 데이터 우선)
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
    % dataCounts 로 originCounts 갱신
    keys_dc = keys(dataCounts);
    for k = 1:length(keys_dc)
        originCounts(keys_dc{k}) = dataCounts(keys_dc{k});
    end
end


function updateModelsSheet(xlsxFile, newModelNames, originName, scalingFactors, ...
        sheetData, endHdrRow, colHdrRow, originCounts)

    dateStr = datestr(now, 'yymmdd');
    nNew    = length(newModelNames);
    COL_N   = 7;   % Names origin Date thigh shank afo abnormal

    % origin count 업데이트
    if isKey(originCounts, originName)
        originCounts(originName) = originCounts(originName) + nNew;
    else
        originCounts(originName) = nNew;
    end

    % ── 기존 데이터 행 수집 (colHdrRow 이후) ──
    existingData = {};
    for r = colHdrRow + 1 : size(sheetData, 1)
        row = sheetData(r, 1:min(end, COL_N));
        % 빈 행이면 건너뜀
        allEmpty = true;
        for c = 1:length(row)
            if ~isCellMissing(row{c}), allEmpty = false; break; end
        end
        if ~allEmpty
            existingData{end+1} = padRow(row, COL_N); %#ok<AGROW>
        end
    end

    % ── 새 데이터 행 ──
    newData = cell(nNew, COL_N);
    for i = 1:nNew
        newData{i, 1} = newModelNames{i};
        newData{i, 2} = originName;
        newData{i, 3} = dateStr;
        newData{i, 4} = scalingFactors(i, 1);
        newData{i, 5} = scalingFactors(i, 2);
    end

    % ── 시트 전체 재구성 ──
    originKeys   = keys(originCounts);
    nOrigins     = length(originKeys);
    nExistingData = length(existingData);
    totalRows    = nOrigins + 1 + 1 + nExistingData + nNew;
    %               header   + endheader + colheader + existing + new

    fullSheet = cell(totalRows, COL_N);

    % 헤더 (origin counts)
    for k = 1:nOrigins
        fullSheet{k, 1} = originKeys{k};
        fullSheet{k, 2} = originCounts(originKeys{k});
    end

    % endheader
    fullSheet{nOrigins + 1, 1} = 'endheader';

    % 컬럼 헤더
    colHeaders = {'Names','origin','Date','thigh','shank','afo','abnormal'};
    for c = 1:COL_N
        fullSheet{nOrigins + 2, c} = colHeaders{c};
    end

    % 기존 데이터
    dataStart = nOrigins + 3;
    for r = 1:nExistingData
        fullSheet(dataStart + r - 1, :) = existingData{r};
    end

    % 새 데이터
    newStart = dataStart + nExistingData;
    for r = 1:nNew
        fullSheet(newStart + r - 1, 1:COL_N) = newData(r, :);
    end

    % missing / NaN 셀을 '' 로 변환 (writecell이 missing 타입 거부)
    for ri = 1:size(fullSheet, 1)
        for ci = 1:size(fullSheet, 2)
            if isCellMissing(fullSheet{ri, ci})
                fullSheet{ri, ci} = '';
            end
        end
    end

    % 시트 전체 쓰기
    writecell(fullSheet, xlsxFile, 'Sheet', 'models', 'Range', 'A1', ...
        'WriteMode', 'overwritesheet');
    fprintf('Updated models sheet in: %s\n', xlsxFile);
end


% =========================================================================
%  헬퍼
% =========================================================================

function addBodyScale(scaler, segmentName, factor)
    import org.opensim.modeling.*
    sc = Scale();
    sc.setApply(true);
    sc.setSegmentName(segmentName);
    sc.setScaleFactors(Vec3(factor, factor, factor));
    scaler.addScale(sc);
end

function tf = isCellMissing(x)
    tf = isa(x, 'missing') || (isnumeric(x) && isscalar(x) && isnan(x)) || ...
         (ischar(x) && isempty(x));
end

function row = padRow(row, n)
    if length(row) < n
        row{n} = missing;
    end
end


% =========================================================================
%  AFO path point 보정
%
%  구조: P1 → tibia (scaled),  P2 → calcn (calcn은 미스케일, tibia scaling으로 이동)
%
%  [핵심 원리]
%  - P2(calcn)는 local 좌표를 변경하지 않음 → calcn과 함께 자연스럽게 이동
%  - P1(tibia)는 "P2_new_ground + 원래 AFO 벡터(V = P1_orig - P2_orig)"로 재배치
%  → AFO의 방향과 길이(절대 크기)가 완벽히 보존됨
% =========================================================================

function afo = getAFOVectors(model, state)
% 스케일 전 AFO 벡터 V = P1_ground - P2_ground 저장
    p1_r = getPathPointGroundPos(model, state, '/AFO_r', 'AFO_r-P1');
    p2_r = getPathPointGroundPos(model, state, '/AFO_r', 'AFO_r-P2');
    afo.v_r = p1_r - p2_r;   % AFO 벡터 (방향 + 길이)

    p1_l = getPathPointGroundPos(model, state, '/AFO_l', 'AFO_l-P1');
    p2_l = getPathPointGroundPos(model, state, '/AFO_l', 'AFO_l-P2');
    afo.v_l = p1_l - p2_l;
end

function restoreAFOTibiaPoints(model, state, afo)
% P2의 새 ground 위치를 읽고, P1 = P2_new + V 로 tibia local 좌표 재설정
% P2(calcn)는 건드리지 않음

    % AFO_r
    p2_r_new = getPathPointGroundPos(model, state, '/AFO_r', 'AFO_r-P2');
    p1_r_target = p2_r_new + afo.v_r;
    setPathPointFromGroundPos(model, state, '/AFO_r', 'AFO_r-P1', p1_r_target);

    % AFO_l
    p2_l_new = getPathPointGroundPos(model, state, '/AFO_l', 'AFO_l-P2');
    p1_l_target = p2_l_new + afo.v_l;
    setPathPointFromGroundPos(model, state, '/AFO_l', 'AFO_l-P1', p1_l_target);
end

function groundPos = getPathPointGroundPos(model, state, actuatorPath, pointName)
    import org.opensim.modeling.*
    assert(model.hasComponent(actuatorPath), 'Component not found: %s', actuatorPath);
    pathAct = PathActuator.safeDownCast(model.updComponent(actuatorPath));
    assert(~isempty(pathAct), '%s is not a PathActuator.', actuatorPath);
    pt = PathPoint.safeDownCast(pathAct.updGeometryPath().updPathPointSet().get(pointName));
    assert(~isempty(pt), 'Path point %s not found under %s.', pointName, actuatorPath);

    v = pt.getLocationInGround(state);
    groundPos = [v.get(0), v.get(1), v.get(2)];
end

function setPathPointFromGroundPos(model, state, actuatorPath, pointName, groundPos)
    import org.opensim.modeling.*
    assert(model.hasComponent(actuatorPath), 'Component not found: %s', actuatorPath);
    pathAct = PathActuator.safeDownCast(model.updComponent(actuatorPath));
    assert(~isempty(pathAct), '%s is not a PathActuator.', actuatorPath);
    pt = PathPoint.safeDownCast(pathAct.updGeometryPath().updPathPointSet().get(pointName));
    assert(~isempty(pt), 'Path point %s not found under %s.', pointName, actuatorPath);

    % ground 좌표 → P1이 붙은 body(tibia)의 local 좌표로 역변환
    groundVec = Vec3(groundPos(1), groundPos(2), groundPos(3));
    bodyFrame = pt.getParentFrame();
    ground    = model.getGround();
    localVec  = ground.findStationLocationInAnotherFrame(state, groundVec, bodyFrame);

    pt.set_location(localVec);
end
