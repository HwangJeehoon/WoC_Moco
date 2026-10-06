%% search_weight_frac.m
% Moco cost weight 정규화 검증 스크립트.
%
% gaitMode(modeSym, modeAsym)마다 독립적으로 아래 4 run을 수행한다.
%   ref  (1,1,1) : (effort, time, propulsion) weight = (1, 1, 1)로 풀어
%                  기준 objective term J = (J_e, J_t, J_p)를 얻는다.
%   norm (1,1,1) : weight = (1/|J_e|, 1/|J_t|, 1/|J_p|)
%   norm (1,1,0) : weight = (1/|J_e|, 1/|J_t|, 0)
%   norm (1,0,1) : weight = (1/|J_e|, 0, 1/|J_p|)
% 정규화된 run에서 각 term이 1 : 1 : 1 (mask가 0인 항목은 0)로 유지되는지 본다.
%
% 참고:
%   - .sto 헤더의 objective_* 는 weight가 곱해진 기여분이다.
%   - propulsion goal은 reward라 term이 음수다. 따라서 |J_p|의 역수를 쓰고,
%     정규화 후 기대값은 J_propulsion = -1 이다. 비율은 |J|로 비교한다.
%   - modeSym은 _kinematics_half.sto, modeAsym은 _kinematics.sto 헤더에서 읽는다.
%     (modeSym의 full 파일은 createPeriodicTrajectory 결과라 objective가 없다.)
%   - 완료된 case는 재사용한다. ref 결과를 지우고 다시 돌릴 때는
%     같은 mode의 norm 결과도 함께 지워야 새 weight가 반영된다.
%   - propulsion reference duration(T_ref)은 gaitMode와 무관하게 모든 case에서
%     1 s로 고정한다. norm run에서는 T_ref가 |J_p| 안에서 상쇄되지만,
%     ref (1,1,1) run의 실질 propulsion weight 1/(2*BW*T_ref)를 정하므로
%     기준 해에는 영향을 준다.
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
% 정규화 weight에 곱할 (effort, time, propulsion) mask.
masks = [1 1 1; 1 1 0; 1 0 1];
caseLabels = ["ref (1,1,1)"; "norm (1,1,1)"; "norm (1,1,0)"; "norm (1,0,1)"];

optsBase = struct();
optsBase.mocoPropulsionSmoothing = 1.0;
optsBase.mocoPropulsionReferenceDuration = 1.0;

summaryTables = cell(numel(gaitModes), 1);
for iMode = 1:numel(gaitModes)
    opts = optsBase;
    opts.gaitMode = gaitModes{iMode};
    modeTag = lower(erase(gaitModes{iMode}, 'mode'));   % 'sym' / 'asym'

    weights = zeros(numel(caseLabels), 3);
    terms = zeros(numel(caseLabels), 4);

    % 1) Reference: (1,1,1)
    weights(1, :) = [1 1 1];
    terms(1, :) = runCase(modelName, resultsDir, ...
        sprintf('wfrac_%s_ref_e1t1p1', modeTag), opts, weights(1, :));
    fprintf(['[%s] reference (1,1,1) objective: effort=%.6g, time=%.6g, ', ...
             'propulsion=%.6g, total=%.6g\n'], gaitModes{iMode}, terms(1, :));

    % 2-3) reference term의 역수로 정규화한 weight에 mask를 곱해 다시 푼다.
    wNorm = 1 ./ abs(terms(1, 1:3));
    for iMask = 1:size(masks, 1)
        weights(iMask + 1, :) = masks(iMask, :) .* wNorm;
        terms(iMask + 1, :) = runCase(modelName, resultsDir, ...
            sprintf('wfrac_%s_norm_e%dt%dp%d', modeTag, masks(iMask, :)), ...
            opts, weights(iMask + 1, :));
    end

    ratio = abs(terms(:, 1:3)) ./ abs(terms(:, 1));
    summaryTables{iMode} = table(caseLabels, weights(:, 1), weights(:, 2), ...
        weights(:, 3), terms(:, 1), terms(:, 2), terms(:, 3), terms(:, 4), ...
        compose('%.3f : %.3f : %.3f', ratio), 'VariableNames', ...
        {'Case', 'w_effort', 'w_time', 'w_propulsion', 'J_effort', ...
        'J_time', 'J_propulsion', 'J_total', 'Ratio_abs'});
end

% Moco 로그에 묻히지 않도록 마지막에 mode별 결과를 모아서 출력한다.
for iMode = 1:numel(gaitModes)
    fprintf('\n===== %s =====\n', gaitModes{iMode});
    fprintf(['목표: norm run에서 (J_effort, J_time, J_propulsion) = ', ...
             '(1, 1, -1) .* mask, Ratio_abs = |J| / |J_effort|\n']);
    disp(summaryTables{iMode});
end


function terms = runCase(modelName, resultsDir, resultName, opts, weights)
% weights = (effort, time, propulsion)로 modeOff 1회 solve 후 objective term 반환.
% 이미 완료된 case는 재사용한다.
    opts.mocoEffort = weights(1);
    opts.mocoFinalTime = weights(2);
    opts.mocoPropulsionWeight = weights(3);

    mocoResultDir = fullfile(resultsDir, resultName, 'result_1', 'moco_result');
    if strcmpi(opts.gaitMode, 'modeSym')
        solFile = fullfile(mocoResultDir, ...
            'moco_WoC_Solution_iter01_kinematics_half.sto');
    else  % modeAsym
        solFile = fullfile(mocoResultDir, ...
            'moco_WoC_Solution_iter01_kinematics.sto');
    end

    if isfile(solFile)
        fprintf('완료된 case 재사용: %s\n', resultName);
    else
        assert(~isfolder(fullfile(resultsDir, resultName)), ...
            '불완전한 결과 폴더가 있습니다: %s. 삭제하거나 새 결과 이름을 사용하세요.', ...
            resultName);
        WoC_moco_main(modelName, 1, 'modeOff', resultName, opts);
    end
    terms = readObjectiveTerms(solFile);
end


function terms = readObjectiveTerms(stoPath)
% Moco 해 .sto 헤더에서 weight가 곱해진 objective term을 읽는다.
%   terms = [effort, final_time, propulsion(r+l), total]
% weight 0인 propulsion goal은 추가되지 않으므로 헤더에 없으면 0으로 둔다.
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
