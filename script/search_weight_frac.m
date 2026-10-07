%% search_weight_frac.m
% Moco cost weight 정규화 탐색/검증 스크립트.
%
% 각 gaitMode(modeSym, modeAsym)에서 아래 reference-normalization 조합을
% 독립적으로 푼다. 모든 term은 해 .sto header에 저장된, weight가 곱해진
% objective 기여분이다.
%
%   A. ref (1,1,1) -> norm (1,1,1), norm (1,1,0), norm (1,0,1)
%      (기존 결과 이름을 유지해 완료된 case를 재사용한다.)
%   B. ref (1,1,0) -> norm (1,1,0)
%   C. ref (1,0,1) -> norm (1,0,1)
%
% 각 norm case의 활성 weight는 해당 reference의 활성 term 역수로 둔다.
% 예: ref (1,1,0)의 norm (1,1,0)은
%      (1/|J_effort|, 1/|J_time|, 0)이다.
%
% 참고:
%   - propulsion은 reward라 header term이 음수다. 따라서 |J_p|를 쓴다.
%   - propulsion term은 objective_propulsion_r + objective_propulsion_l이다.
%   - modeSym은 _kinematics_half.sto, modeAsym은 _kinematics.sto를 읽는다.
%   - 한 case가 실패하거나 sealed/unseal 결과만 남아도, 이를 Failed/Skipped로
%     기록한 뒤 나머지 case 및 다음 gaitMode를 계속 수행한다.
%   - 이미 존재하는 불완전 결과 폴더는 덮어쓰지 않고 건너뛴다. 재시도하려면
%     해당 result 폴더를 직접 삭제하거나 result 이름을 바꿔야 한다.
%   - propulsion reference duration(T_ref)은 모든 case에서 1 s로 고정한다.
clc; clear; close all;
thisScriptDir = fileparts(mfilename('fullpath'));
if isempty(thisScriptDir)
    % Current Section 실행 시 mfilename이 비어도, repository root와
    % script 폴더 중 현재 작업 폴더에 맞는 위치를 선택한다.
    if isfile(fullfile(pwd, 'search_weight_frac.m'))
        thisScriptDir = pwd;
    elseif isfile(fullfile(pwd, 'script', 'search_weight_frac.m'))
        thisScriptDir = fullfile(pwd, 'script');
    else
        error(['search_weight_frac.m 위치를 찾지 못했습니다. 작업 폴더를 ', ...
               'repository root 또는 script 폴더로 설정하세요.']);
    end
end
addpath(genpath(fullfile(thisScriptDir, '..', 'src')));

modelName = '2D_gait_AFO_pc.osim';
resultsDir = fullfile(thisScriptDir, '..', 'results');
gaitModes = {'modeSym', 'modeAsym'};

% ref (1,1,1)에서는 기존처럼 세 norm mask를 모두 실행한다. 추가한
% reference는 자신과 같은 활성 mask의 norm case만 실행한다.
referenceSpecs = makeReferenceSpecs();

optsBase = struct();
optsBase.mocoPropulsionSmoothing = 1.0;
optsBase.mocoPropulsionReferenceDuration = 1.0;

summaryTables = cell(numel(gaitModes), 1);
for iMode = 1:numel(gaitModes)
    opts = optsBase;
    opts.gaitMode = gaitModes{iMode};
    modeTag = lower(erase(gaitModes{iMode}, 'mode'));   % 'sym' / 'asym'

    nRows = sum(1 + arrayfun(@(spec) size(spec.normMasks, 1), referenceSpecs));
    caseLabels = strings(nRows, 1);
    referenceLabels = strings(nRows, 1);
    statuses = strings(nRows, 1);
    weights = nan(nRows, 3);
    terms = nan(nRows, 4);
    iRow = 0;

    for iSpec = 1:numel(referenceSpecs)
        spec = referenceSpecs(iSpec);

        % Reference case.
        iRow = iRow + 1;
        referenceRow = iRow;
        caseLabels(iRow) = "ref " + formatMask(spec.referenceWeights);
        referenceLabels(iRow) = "self";
        weights(iRow, :) = spec.referenceWeights;
        refName = sprintf('wfrac_%s_ref_%s', modeTag, spec.tag);
        [terms(iRow, :), refComplete, statuses(iRow)] = runCase( ...
            modelName, resultsDir, refName, opts, spec.referenceWeights);

        if refComplete
            fprintf(['[%s] reference %s objective: effort=%.6g, time=%.6g, ', ...
                     'propulsion=%.6g, total=%.6g\n'], ...
                    gaitModes{iMode}, formatMask(spec.referenceWeights), terms(iRow, :));
        end

        % Matching/legacy norm cases that use the current reference term.
        for iMask = 1:size(spec.normMasks, 1)
            iRow = iRow + 1;
            mask = spec.normMasks(iMask, :);
            caseLabels(iRow) = "norm " + formatMask(mask);
            referenceLabels(iRow) = "from ref " + formatMask(spec.referenceWeights);

            if ~refComplete
                statuses(iRow) = "Skipped: reference unavailable";
                fprintf('건너뜀: %s (%s)\n', caseLabels(iRow), statuses(iRow));
                continue;
            end

            [weights(iRow, :), weightsValid, weightStatus] = ...
                makeNormalizedWeights(terms(referenceRow, 1:3), mask);
            if ~weightsValid
                statuses(iRow) = "Skipped: " + weightStatus;
                fprintf('건너뜀: %s (%s)\n', caseLabels(iRow), statuses(iRow));
                continue;
            end

            normName = makeNormResultName(modeTag, spec, mask);
            [terms(iRow, :), ~, statuses(iRow)] = runCase( ...
                modelName, resultsDir, normName, opts, weights(iRow, :));
        end
    end

    ratio = makeRatioStrings(terms(:, 1:3));
    summaryTables{iMode} = table(caseLabels, referenceLabels, statuses, ...
        weights(:, 1), weights(:, 2), weights(:, 3), ...
        terms(:, 1), terms(:, 2), terms(:, 3), terms(:, 4), ratio, ...
        'VariableNames', {'Case', 'NormalizationReference', 'Status', ...
        'w_effort', 'w_time', 'w_propulsion', 'J_effort', 'J_time', ...
        'J_propulsion', 'J_total', 'Ratio_abs'});
end

% Moco 로그에 묻히지 않도록 마지막에 mode별 결과를 모아서 출력한다.
for iMode = 1:numel(gaitModes)
    fprintf('\n===== %s =====\n', gaitModes{iMode});
    fprintf(['norm run의 target은 활성 term에 대해 (J_effort, J_time, ', ...
             'J_propulsion) = (1, 1, -1) 이다. Ratio_abs = |J| / |J_effort|\n']);
    disp(summaryTables{iMode});
end


function specs = makeReferenceSpecs()
% 각 reference와 그 reference에서 유도할 norm mask를 정의한다.
    specs(1).tag = 'e1t1p1';
    specs(1).referenceWeights = [1 1 1];
    specs(1).normMasks = [1 1 1; 1 1 0; 1 0 1];
    specs(1).useLegacyNormNames = true;

    specs(2).tag = 'e1t1p0';
    specs(2).referenceWeights = [1 1 0];
    specs(2).normMasks = [1 1 0];
    specs(2).useLegacyNormNames = false;

    specs(3).tag = 'e1t0p1';
    specs(3).referenceWeights = [1 0 1];
    specs(3).normMasks = [1 0 1];
    specs(3).useLegacyNormNames = false;
end


function resultName = makeNormResultName(modeTag, spec, mask)
% 새 reference 기반 norm case는 기존 ref(1,1,1) 기반 case와 분리한다.
    if spec.useLegacyNormNames
        resultName = sprintf('wfrac_%s_norm_e%dt%dp%d', modeTag, mask);
    else
        resultName = sprintf('wfrac_%s_norm_from_ref_%s_e%dt%dp%d', ...
            modeTag, spec.tag, mask);
    end
end


function [weights, isValid, status] = makeNormalizedWeights(referenceTerms, mask)
% 활성 objective term만 reference 절댓값의 역수로 정규화한다.
    weights = zeros(1, 3);
    isValid = false;
    status = "";
    active = logical(mask);
    magnitudes = abs(referenceTerms);
    invalid = active & (~isfinite(magnitudes) | magnitudes <= eps);
    if any(invalid)
        status = "reference has zero/non-finite active term";
        return;
    end
    weights(active) = 1 ./ magnitudes(active);
    isValid = true;
end


function [terms, isComplete, status] = runCase(modelName, resultsDir, resultName, opts, weights)
% weights = (effort, time, propulsion)로 modeOff 1회 solve 후 objective term 반환.
% 완료 결과는 재사용한다. 실패/unseal 결과 또는 solver error는 기록만 하고
% false를 반환해 다음 case가 계속 실행되도록 한다.
    terms = nan(1, 4);
    isComplete = false;
    status = "";
    opts.mocoEffort = weights(1);
    opts.mocoFinalTime = weights(2);
    opts.mocoPropulsionWeight = weights(3);

    resultDir = fullfile(resultsDir, resultName);
    mocoResultDir = fullfile(resultDir, 'result_1', 'moco_result');
    if strcmpi(opts.gaitMode, 'modeSym')
        solFile = fullfile(mocoResultDir, ...
            'moco_WoC_Solution_iter01_kinematics_half.sto');
    else  % modeAsym
        solFile = fullfile(mocoResultDir, ...
            'moco_WoC_Solution_iter01_kinematics.sto');
    end

    if isfile(solFile)
        fprintf('완료된 case 재사용: %s\n', resultName);
        status = "Reused";
    elseif isfolder(resultDir)
        status = "Skipped: incomplete result folder exists";
        fprintf('불완전 case 건너뜀: %s\n', resultName);
        return;
    else
        try
            WoC_moco_main(modelName, 1, 'modeOff', resultName, opts);
        catch ME
            status = "Failed: " + string(ME.identifier);
            if strlength(ME.identifier) == 0
                status = "Failed: solver or output error";
            end
            warning('search_weight_frac:CaseFailed', ...
                'Case %s failed. Continuing.\n%s', resultName, ME.message);
            return;
        end

        if ~isfile(solFile)
            status = "Failed: completed file was not written";
            warning('search_weight_frac:MissingSolution', ...
                'Case %s did not write the expected solution. Continuing.', resultName);
            return;
        end
        status = "Completed";
    end

    try
        terms = readObjectiveTerms(solFile);
        isComplete = true;
    catch ME
        terms = nan(1, 4);
        status = "Failed: objective header unreadable";
        warning('search_weight_frac:UnreadableHeader', ...
            'Case %s header could not be read. Continuing.\n%s', resultName, ME.message);
    end
end


function ratio = makeRatioStrings(termTriples)
% effort 기준 절댓값 비율을 summary table용 문자열로 만든다.
    nRows = size(termTriples, 1);
    ratio = strings(nRows, 1);
    for iRow = 1:nRows
        row = termTriples(iRow, :);
        if ~all(isfinite(row)) || abs(row(1)) <= eps
            ratio(iRow) = "n/a";
        else
            values = abs(row) ./ abs(row(1));
            ratio(iRow) = sprintf('%.3f : %.3f : %.3f', values);
        end
    end
end


function label = formatMask(mask)
% (effort, time, propulsion) mask/weight vector을 표기한다.
    label = sprintf('(%d,%d,%d)', mask);
end


function terms = readObjectiveTerms(stoPath)
% Moco 해 .sto 헤더에서 weight가 곱해진 objective term을 읽는다.
%   terms = [effort, final_time, propulsion(r+l), total]
% weight 0인 propulsion goal은 추가되지 않으므로 header에 없으면 0으로 둔다.
    txt = fileread(stoPath);
    keys = {'objective_effort', 'objective_final_time', ...
            'objective_propulsion_r', 'objective_propulsion_l', 'objective'};
    vals = zeros(1, numel(keys));
    for k = 1:numel(keys)
        tok = regexp(txt, ['(?m)^' keys{k} '=([^\r\n]+)'], 'tokens', 'once');
        if ~isempty(tok)
            vals(k) = str2double(tok{1});
        elseif ~startsWith(keys{k}, 'objective_propulsion')
            error('%s 를 찾지 못했습니다: %s', keys{k}, stoPath);
        end
    end
    terms = [vals(1), vals(2), vals(3) + vals(4), vals(5)];
end
