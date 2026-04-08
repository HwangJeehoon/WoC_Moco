function tau = WoC_moco_buildSplineControl(params, N)
% WoC_moco_buildSplineControl
%
%   스탠스 구간을 [0, 0.6]으로 정규화한 도메인에서
%   trapezoid 형태의 cubic Hermite spline control profile을 생성.
%
%   각 구간 경계에서 기울기를 0으로 맞춘 3차 Hermite 보간을 사용하므로
%   모든 접합점에서 C1 연속성(값 + 1차 도함수 연속)을 보장.
%
%   ┌ params 필드 ───────────────────────────────────────┐
%   │  .trigger  : control 시작 시각 (정규화, >= 0)       │
%   │  .rise     : 상승 구간 길이                         │
%   │  .flat     : 최대값 유지 구간 길이                   │
%   │  .fall     : 하강 구간 길이                         │
%   │  .maxVal   : 최대 출력값 (0 ~ 1)                   │
%   └────────────────────────────────────────────────────┘
%
%   제약:
%     trigger + rise + flat + fall <= 0.6
%     maxVal in [0, 1]
%
%   파형 모양 (정규화 시간축 0 ~ 0.6):
%
%     maxVal ──────┬───────────┬──────
%                 /             \
%                /               \
%     0 ────────┘                 └──────
%            trigger  t2    t3   t4
%
%   입력:
%     params : struct (위 필드 참조)
%     N      : 출력 포인트 수 (stanceTime 포인트 수와 동일하게 설정)
%              default = 101
%
%   출력:
%     tau    : [N x 1] control 값 (0 ~ maxVal)

    if nargin < 2 || isempty(N)
        N = 101;
    end

    trigger = params.trigger;
    rise    = params.rise;
    flat    = params.flat;
    fall    = params.fall;
    maxVal  = params.maxVal;

    %% 입력 검증
    if any([trigger, rise, flat, fall] < 0)
        error('WoC_moco_buildSplineControl: trigger, rise, flat, fall 은 0 이상이어야 합니다.');
    end
    total = trigger + rise + flat + fall;
    if total > 0.6 + 1e-9
        error(['WoC_moco_buildSplineControl: trigger+rise+flat+fall = %.4f 이 ' ...
               '0.6을 초과합니다.'], total);
    end
    if maxVal < 0 || maxVal > 1
        error('WoC_moco_buildSplineControl: maxVal 은 [0, 1] 범위여야 합니다.');
    end
    if N < 2
        error('WoC_moco_buildSplineControl: N 은 2 이상이어야 합니다.');
    end

    %% 정규화 시간축 및 구간 경계
    t_norm = linspace(0, 0.6, N)';
    tau    = zeros(N, 1);

    t1 = trigger;               % rise  시작
    t2 = trigger + rise;        % flat  시작
    t3 = trigger + rise + flat; % fall  시작
    t4 = total;                 % control 종료

    %% 각 포인트에 대해 구간별 cubic Hermite 적용
    %
    %  rise  구간: h(s) = 3s² - 2s³  (smooth step, s ∈ [0,1])
    %              p = maxVal * h(s)
    %
    %  fall  구간: p = maxVal * (1 - h(s))
    %
    %  접합점에서 h'(0) = 0, h'(1) = 0 이므로 기울기 연속 보장

    for k = 1:N
        t = t_norm(k);

        if t < t1
            % 0 ~ trigger: 영역
            tau(k) = 0;

        elseif t < t2
            % rise 구간: [t1, t2)
            if rise > 0
                s = (t - t1) / rise;          % s ∈ [0, 1)
                tau(k) = maxVal * (3*s^2 - 2*s^3);
            else
                tau(k) = maxVal;              % rise=0: 즉시 최대값
            end

        elseif t <= t3
            % flat 구간: [t2, t3]
            tau(k) = maxVal;

        elseif t <= t4
            % fall 구간: (t3, t4]
            if fall > 0
                s = (t - t3) / fall;          % s ∈ (0, 1]
                tau(k) = maxVal * (1 - 3*s^2 + 2*s^3);
            else
                tau(k) = 0;                   % fall=0: 즉시 0
            end

        else
            % t4 ~ 0.6: 영역
            tau(k) = 0;
        end
    end

end
