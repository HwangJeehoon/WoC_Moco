function cases = WoC_moco_propulsionCases(cfg)
% WoC_moco_propulsionCases
%
%   stateFixed 실행의 propulsion 제약 입력을 검사하고 case 목록으로 정규화한다.
%   실행 스크립트(실행 전 검사)와 WoC_moco_runStateFixedCases가 같은 규칙을 쓴다.
%
%   cfg 필드:
%     propulsionConstraintMode : 'equality' (없거나 비면 이 값, 기존 동작) | 'range'
%     J_targets : equality 전용. 양의 유한 값 벡터, 원소 하나가 독립 case 하나
%                 (J_actual = J_k)
%     J_bounds  : range 전용. n x 2 행렬 [lower, upper], 행 하나가 독립 case 하나
%                 (lower <= J_actual <= upper, 닫힌 구간). 0 <= lower < upper, 유한 실수.
%                 구간 중심은 target으로 쓰지 않는다.
%   equality에 J_bounds, range에 J_targets가 함께 들어 있으면 모호하므로 error.
%
%   출력 cases (struct 배열, case 순서):
%     mode   : 'equality' | 'range'
%     target : equality target (range는 NaN)
%     lower, upper : 허용 구간 (equality는 lower = upper = target)
%     indexTag : 'J<k>' (기존 case 이름 규칙)
%     valueTag : equality 'J<값>', range 'JR<lower>to<upper>' ('.'은 'p')
%     label  : 로그용 문자열

    mode = 'equality';
    if isfield(cfg, 'propulsionConstraintMode') && ~isempty(cfg.propulsionConstraintMode)
        mode = char(cfg.propulsionConstraintMode);
    end
    if ~ismember(mode, {'equality', 'range'})
        error('WoC_moco_propulsionCases:Mode', ...
            'cfg.propulsionConstraintMode 는 ''equality'' 또는 ''range'' 여야 합니다 (입력: %s).', mode);
    end
    targets = [];
    if isfield(cfg, 'J_targets'), targets = cfg.J_targets; end
    bounds = [];
    if isfield(cfg, 'J_bounds'), bounds = cfg.J_bounds; end

    if strcmp(mode, 'equality')
        if ~isempty(bounds)
            error('WoC_moco_propulsionCases:Ambiguous', ...
                ['propulsionConstraintMode = ''equality'' 인데 J_bounds도 지정되어 있습니다. ', ...
                 'equality는 J_targets만, range는 J_bounds만 지정하세요.']);
        end
        if isempty(targets)
            error('WoC_moco_propulsionCases:MissingTargets', ...
                ['equality 모드의 J_targets가 입력되지 않았습니다. 실행 스크립트 상단의 ', ...
                 'J_targets에 실제 target 값을 입력하세요.']);
        end
        if ~isnumeric(targets) || ~isreal(targets) || ~isvector(targets) || ...
                any(~isfinite(targets)) || any(targets <= 0)
            error('WoC_moco_propulsionCases:Targets', 'J_targets는 양의 유한 값의 벡터여야 합니다.');
        end
        targets = double(targets(:)');
        n = numel(targets);
        lower = targets; upper = targets; tgt = targets;
    else
        if ~isempty(targets)
            error('WoC_moco_propulsionCases:Ambiguous', ...
                ['propulsionConstraintMode = ''range'' 인데 J_targets도 지정되어 있습니다. ', ...
                 'range는 J_bounds의 [lower, upper]만 쓰고 scalar target을 쓰지 않습니다 ', ...
                 '(J_targets = []로 두세요).']);
        end
        if isempty(bounds)
            error('WoC_moco_propulsionCases:MissingBounds', ...
                'range 모드의 J_bounds가 입력되지 않았습니다. 각 행에 [lower, upper]를 입력하세요.');
        end
        if ~isnumeric(bounds) || ~isreal(bounds) || ~ismatrix(bounds) || size(bounds, 2) ~= 2
            error('WoC_moco_propulsionCases:BoundsShape', ...
                'J_bounds는 n x 2 실수 행렬 [lower, upper]이어야 합니다 (입력 크기: %s).', ...
                mat2str(size(bounds)));
        end
        bounds = double(bounds);
        if any(~isfinite(bounds(:)))
            error('WoC_moco_propulsionCases:BoundsFinite', 'J_bounds에 NaN 또는 Inf가 있습니다: %s', ...
                mat2str(bounds));
        end
        bad = find(bounds(:, 1) < 0 | bounds(:, 1) >= bounds(:, 2));
        if ~isempty(bad)
            error('WoC_moco_propulsionCases:BoundsOrder', ...
                'J_bounds의 각 행은 0 <= lower < upper 여야 합니다 (행 %s: %s).', ...
                mat2str(bad'), mat2str(bounds(bad, :)));
        end
        n = size(bounds, 1);
        lower = bounds(:, 1)'; upper = bounds(:, 2)'; tgt = NaN(1, n);
    end

    cases = struct('mode', {}, 'target', {}, 'lower', {}, 'upper', {}, ...
        'indexTag', {}, 'valueTag', {}, 'label', {});
    for k = 1:n
        if strcmp(mode, 'equality')
            valueTag = ['J' numTag(tgt(k))];
            label = sprintf('equality J = %g', tgt(k));
        else
            valueTag = ['JR' numTag(lower(k)) 'to' numTag(upper(k))];
            label = sprintf('range %g <= J <= %g', lower(k), upper(k));
        end
        cases(k) = struct('mode', mode, 'target', tgt(k), 'lower', lower(k), 'upper', upper(k), ...
            'indexTag', sprintf('J%d', k), 'valueTag', valueTag, 'label', label);
    end
end


function s = numTag(v)
    s = strrep(sprintf('%g', v), '.', 'p');
end
