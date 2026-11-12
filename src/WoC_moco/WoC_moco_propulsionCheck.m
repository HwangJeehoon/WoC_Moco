function c = WoC_moco_propulsionCheck(actual, lower, upper, passTol, hitTol)
% WoC_moco_propulsionCheck
%
%   propulsion 제약 판정 (equality는 lower = upper = target으로 같은 규칙):
%     violation = max([lower - actual, actual - upper, 0])
%     pass      = violation <= passTol            (결과 검증 허용오차)
%   구간 중심과의 차이는 판정에 쓰지 않는다. 하한/상한에 붙은 결과도 구간 안이면 유효하다.
%   어느 경계에 붙었는지는 hitTol (경계 접촉 판정 허용오차)로 따로 표시한다:
%     hits = "lower" (|actual - lower| <= hitTol), "upper" (|actual - upper| <= hitTol),
%            equality(lower == upper)에서는 "target", 둘 다 아니면 "".
%   solver tolerance, passTol, hitTol, 사용자가 정한 구간 폭은 서로 다른 값이다.
%
%   입력은 같은 크기의 배열이어도 된다 (원소별 판정).
%   판정할 수 없는 원소는 violation NaN, pass false, hits "" 이다:
%     - actual이 NaN
%     - lower 또는 upper가 유한하지 않음 (NaN: 제약 정보 없음, +-Inf: 구간이 아님)
%     - lower > upper (순서가 뒤바뀐 구간)
%   (MATLAB의 max는 NaN을 무시하므로, 이 검사가 없으면 정보가 없는 구간이
%    violation 0 / pass true가 된다.)
%   c.validBounds 는 구간이 판정 가능한지 (유한하고 lower <= upper) 원소별로 알려준다.

    sz = size(actual);
    lower = lower .* ones(sz); upper = upper .* ones(sz);
    c.validBounds = isfinite(lower) & isfinite(upper) & lower <= upper;
    judgeable = c.validBounds & ~isnan(actual);
    c.violation = max(max(lower - actual, actual - upper), 0);
    c.violation(~judgeable) = NaN;
    c.pass = judgeable & c.violation <= passTol;
    c.hits = strings(sz);
    isEq = lower == upper;
    nearLower = judgeable & abs(actual - lower) <= hitTol;
    nearUpper = judgeable & abs(actual - upper) <= hitTol;
    c.hits(isEq & nearLower) = "target";
    c.hits(~isEq & nearLower) = "lower";
    c.hits(~isEq & nearUpper) = "upper";
    c.hits(~isEq & nearLower & nearUpper) = "lower;upper";
end
