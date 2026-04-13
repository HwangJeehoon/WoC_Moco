function OptimToq = WoC_moco_solveQP(eta, w, dt, opts)
% solveQP
%   maximize  J(tau) = sum(eta .* tau .* w) - alpha*sum(tau.^2) - beta*sum(tau_2dot.^2)
%   subject to
%       tauMin <= tau_i <= tauMax
%       |tau_dot_i| <= tauDotMax
%       tau(1) = 0, tau(N) = 0
%
% 입력:
%   eta  : [N x 1] 또는 [1 x N] 벡터
%   w    : [N x 1] 또는 [1 x N] 벡터
%   opts : struct (필드는 선택)
%       .QP_effort : tau.^2 weight (default = 1)
%       .QP_smooth : (tau_2dot).^2 weight (default = 1)
%       .tauMin    : 최소 bound (default = 0)
%       .tauMax    : 최대 bound (default = 1)
%       .tauDotMax : |tau_dot| bound (default = 1)
%       .qpOptions : quadprog options (default = interior-point-convex, Display off)
%
% 출력:
%   controlTraj : [N x 1] 최적 tau

    %-----------------------------
    % 0) 입력 전처리 및 기본값 설정
    %-----------------------------
    eta = eta(:); 
    w = w(:);
    N   = numel(eta);
    
    if nargin < 3
        opts = struct();
    end

    % QP_effort, QP_smooth
    if ~isfield(opts,'QP_effort') || isempty(opts.QP_effort) % square coeffi
        alpha = 1.0;
    else
        alpha = opts.QP_effort;
    end

    if ~isfield(opts,'QP_smooth') || isempty(opts.QP_smooth) % 2dot coeffi
        beta = 1.0;
    else
        beta = opts.QP_smooth;
    end

    % tau bounds
    if ~isfield(opts,'tauMin') || isempty(opts.tauMin)
        tauMin = 0;
    else
        tauMin = opts.tauMin;
    end

    if ~isfield(opts,'tauMax') || isempty(opts.tauMax)
        tauMax = 1;
    else
        tauMax = opts.tauMax;
    end

    % |tau_dot| bound
    if ~isfield(opts,'tauDotMax') || isempty(opts.tauDotMax)
        tauDotMax = 1;
    else
        tauDotMax = opts.tauDotMax;
    end

    % quadprog options
    if ~isfield(opts,'qpOptions') || isempty(opts.qpOptions)
        qpOptions = optimoptions('quadprog', ...
            'Display',  'none', ...
            'Algorithm','interior-point-convex');
    else
        qpOptions = opts.qpOptions;
    end

    %-----------------------------------
    % 1) 목적함수 계수 만들기
    %    J(tau) = sum(eta .* tau .* w) ...
    %             - alpha * sum(tau.^2) ...
    %             - beta  * sum(tau_2dot.^2)
    %-----------------------------------

    % (1) 선형항 c = eta .* w
    c = eta .* w;   % [N x 1]

    % (2) 2차항: I (tau.^2 용)
    I_N = speye(N);

    % (3) 2차미분 행렬 D2 (central difference)
    %     tau_2dot(i) ≈ tau(i-1) - 2*tau(i) + tau(i+1), i = 2..N-1
    e = ones(N + 2,1);
    D2_full = spdiags([e -2*e e], -1:1, N+ 2, N + 2);  % [N x N]
    D2 = D2_full(2:N+1, :);                     % [(N-2) x N]
    D2H = D2.' * D2;
    D2H = D2H(2:N+1, 2:N+1);

    % (4) tau_2dot term: (D2*tau)'(D2*tau) = tau'*(D2'*D2)*tau
    Q = alpha * I_N + beta * D2H;   % [N x N]

    % quadprog용 H, f (minimize 0.5*tau'*H*tau + f'*tau = -J)
    H = 2 * Q;       % 0.5 * tau' * H * tau = tau' * Q * tau
    f = -c;          % -c' * tau

    %-----------------------------------
    % 2) 제약조건 설정
    %-----------------------------------

    % (1) bound: tauMin <= tau <= tauMax
    lb = tauMin * ones(N,1);
    ub = tauMax * ones(N,1);

    % (2) |tau_dot| <= tauDotMax
    %     tau_dot(i) ≈ tau(i+1) - tau(i) (forward difference)
    e = ones(N,1);
    D1 = spdiags([-e e], [0 1], N-1, N);  % [N-1 x N], tau_dot = D1 * tau

    % |D1*tau| <= tauDotMax  →  두 개의 부등식:
    %  D1*tau <=  tauDotMax
    % -D1*tau <=  tauDotMax
    A = [ D1;
         -D1];
    b = (dt*tauDotMax) * ones(2*(N-1), 1);

    % (3) boundary condition: tau(1) = 0, tau(N) = 0
    Aeq = zeros(2, N);
    Aeq(1,1)   = 1;   % tau(1)
    Aeq(2,end) = 1;   % tau(N)
    beq = [0; 0];

    %-----------------------------------
    % 3) quadprog 호출
    %-----------------------------------
    [tau_opt, fval, exitflag, output, lambda] = quadprog( ...
        H, f, ...
        A, b, ...
        Aeq, beq, ...
        lb, ub, ...
        [], qpOptions);

    if exitflag <= 0
        warning('solveQP: quadprog did not converge (exitflag = %d)', exitflag);
    end

    % 최종 출력
    OptimToq = tau_opt;

    % result.tau     = tau_opt;
    % result.fval    = fval;
    % result.exitflag= exitflag;
    % result.output  = output;
    % result.lambda  = lambda;
end
