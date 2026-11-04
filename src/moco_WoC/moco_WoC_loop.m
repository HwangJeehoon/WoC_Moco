function moco_WoC_Solution = moco_WoC_loop(controlInitStoPath, guessStoPath, i, resultsDir, modelPath, opts)
% moco_WoC_loop
%
%   AFO reference control(.sto)와 initial guess(.sto)를 받아
%   moco_WoC gait prediction 문제를 세팅하고 solve한 뒤
%   MocoTrajectory를 반환하는 함수.
%
%   입력:
%     controlInitStoPath : AFO reference control .sto 경로
%     guessStoPath       : initial guess trajectory .sto 경로
%     i                  : iteration 번호 (주입 osim 파일명에 사용)
%     resultsDir         : 주입된 osim을 저장할 디렉터리
%     modelPath          : base .osim 경로
%     opts               : (선택) struct
%       .mocoEffort      : effort goal weight (default = 1.0)
%       .mocoFinalTime   : final time goal weight (default = 0.03)
%       .mocoPropulsionWeight : 양의 apGRF impulse 최대화 weight
%                               (default = 0; 0이면 비활성)
%       .mocoPropulsionSmoothing : smooth positive-force 계수 [0.2, 1]
%                                  (default = 0.2)
%       .mocoPropulsionReferenceDuration : impulse 정규화 기준 시간(s)
%       .mocoObjectiveMode : 'weighted' (default) | 'minSpeedStateFixed' | 'maxSpeedStateFixed'
%                            'minSpeedStateFixed'은 평균속도만 최소화하고 effort와
%                            propulsion을 endpoint equality로 고정한다
%                            (플러그인 osimFixedEffortPropulsion 필요).
%                            'maxSpeedStateFixed'은 같은 제약에서 평균속도를 최대화한다.
%       .mocoEnergyMetric : min/maxSpeedStateFixed에서 E가 고정하는 에너지 지표
%                            'effort' (default): (1/d_COM) int sum_c u_c^2 dt [-]
%                            'metabolicCoT': Bhargava2004 total_metabolic_rate의
%                            적분 / (mass * d_COM) [J/kg/m]
%       .mocoEnergyTarget : E0 (단위는 mocoEnergyMetric)
%       .mocoEffortTarget : mocoEnergyTarget이 없을 때 effort 지표의 E0 (이전 이름)
%       .mocoPropulsionMetric : min/maxSpeedStateFixed에서 J가 고정하는 지표
%                            'impulse' (default): 한 보행주기 gated Iplus [N*s]
%                            'meanForce': 평균 추진력 Iplus / Tplus [N]
%       .mocoPropulsionTarget : 'minSpeedStateFixed'의 J (단위는 mocoPropulsionMetric)
%       .mocoPropulsionGateSigma / .mocoPropulsionGateThreshold :
%                            'minSpeedStateFixed' gate g(F) = 1/(1+exp(-(F-F0)/sigma))의
%                            sigma, F0 [N] (default = 1, 5)
%       .mocoSpeedWeight : 'minSpeedStateFixed' 평균속도 cost weight 크기 (default = 1;
%                          'maxSpeedStateFixed'에서는 -mocoSpeedWeight로 쓴다)
%       .mocoMeshIntervals : CasADi mesh interval 수 (default = 300)
%       .mocoConvergenceTolerance : IPOPT convergence tolerance (default = 1e-3)
%       .mocoConstraintTolerance  : IPOPT constraint tolerance (default = 1e-3)
%       .mocoMaxIterations        : IPOPT 최대 반복 수 (default = 1000)
%       .gaitMode        : 'modeSym'  - 반 걸음 풀고 좌우 대칭 복사 (default)
%                          'modeAsym' - 한 걸음 전체를 풀고 주기성만 강제

    import org.opensim.modeling.*

    weight_effort    = opts.mocoEffort;
    weight_finalTime = opts.mocoFinalTime;
    weight_propulsion = getOpt(opts, 'mocoPropulsionWeight', 0);
    propulsionSmoothing = getOpt(opts, 'mocoPropulsionSmoothing', 0.2);
    propulsionReferenceDuration = getOpt(opts, ...
        'mocoPropulsionReferenceDuration', []);
    objectiveMode = getOpt(opts, 'mocoObjectiveMode', 'weighted');
    if ~ismember(objectiveMode, {'weighted', 'minSpeedStateFixed', 'maxSpeedStateFixed'})
        error(['opts.mocoObjectiveMode 는 ''weighted'', ''minSpeedStateFixed'', ', ...
               '''maxSpeedStateFixed'' 중 하나여야 합니다.']);
    end
    useStateFixed = ismember(objectiveMode, {'minSpeedStateFixed', 'maxSpeedStateFixed'});
    propulsionMetric = getOpt(opts, 'mocoPropulsionMetric', 'impulse');
    useMeanForce = useStateFixed && strcmp(propulsionMetric, 'meanForce');
    energyMetric = getOpt(opts, 'mocoEnergyMetric', 'effort');
    if ~ismember(energyMetric, {'effort', 'metabolicCoT'})
        error('opts.mocoEnergyMetric 는 ''effort'' 또는 ''metabolicCoT'' 여야 합니다.');
    end
    useCoT = useStateFixed && strcmp(energyMetric, 'metabolicCoT');
    energyTarget = getOpt(opts, 'mocoEnergyTarget', []);
    if isempty(energyTarget) && strcmp(energyMetric, 'effort')
        energyTarget = getOpt(opts, 'mocoEffortTarget', []);
    end
    metabolicRatePath = '';
    gateSigma = getOpt(opts, 'mocoPropulsionGateSigma', 1.0);
    gateThreshold = getOpt(opts, 'mocoPropulsionGateThreshold', 5.0);
    integratorPath = '/effortPropulsion';
    gaitMode         = opts.gaitMode;
    % modeSym은 반 걸음을 풀므로 한 보행주기 propulsion은 반주기 적분의 2배다.
    impulseScale = 1 + strcmpi(gaitMode, 'modeSym');
    mocoTimeBound    = opts.mocoTimeBound;
    mocoDistBound    = opts.mocoDistBound;

    %--------------------------------------------------------------
    % 1. MocoStudy 및 문제 정의
    %--------------------------------------------------------------
    study = MocoStudy();
    study.setName('moco_WoC');
    problem = study.updProblem();

    %--------------------------------------------------------------
    % 1-2. Prescribed controller 설정 (control.sto → GCVSpline 주입)
    %--------------------------------------------------------------
    half_order     = 1;
    error_variance = 0;
    col_r = 'AFO_r';
    col_l = 'AFO_l';

    baseOsimPath = modelPath;
    [~, modelName, ~] = fileparts(baseOsimPath);

    % STO 읽기
    sto = Storage(controlInitStoPath);
    tArr = ArrayDouble();
    sto.getTimeColumn(tArr);
    N = tArr.getSize();

    labels = sto.getColumnLabels();
    nCol   = labels.getSize();

    idx_r = -1; idx_l = -1;
    for k0 = 0:nCol-1
        nm = char(labels.get(k0));
        if strcmp(nm, col_r), idx_r = k0; end
        if strcmp(nm, col_l), idx_l = k0; end
    end
    if idx_r < 0 || idx_l < 0
        error('control.sto에 %s 또는 %s 컬럼이 없음', col_r, col_l);
    end

    yArr_r = ArrayDouble(); sto.getDataColumn(col_r, yArr_r);
    yArr_l = ArrayDouble(); sto.getDataColumn(col_l, yArr_l);

    x = zeros(1, N); yr = zeros(1, N); yl = zeros(1, N);
    for ii = 0:N-1
        x(ii+1)  = tArr.get(ii);
        yr(ii+1) = yArr_r.get(ii);
        yl(ii+1) = yArr_l.get(ii);
    end

    x_str  = strtrim(sprintf(' %.17g', x));
    yr_str = strtrim(sprintf(' %.17g', yr));
    yl_str = strtrim(sprintf(' %.17g', yl));
    w_str  = strtrim(sprintf(' %.17g', ones(1,N)));
    c_str  = strtrim(sprintf(' %.17g', zeros(1,N)));

    % base osim XML 로드 → GCVSpline 주입
    doc = xmlread(baseOsimPath);

    pcNode = [];
    allPC  = doc.getElementsByTagName('PrescribedController');
    for p = 0:allPC.getLength-1
        n = allPC.item(p);
        a = n.getAttributes().getNamedItem('name');
        if ~isempty(a) && strcmp(char(a.getValue()), 'AFO_controller')
            pcNode = n; break;
        end
    end
    if isempty(pcNode)
        error('osim에서 PrescribedController name="AFO_controller"를 못 찾음');
    end

    fsNode = [];
    kids = pcNode.getChildNodes();
    for kk = 0:kids.getLength-1
        n = kids.item(kk);
        if n.getNodeType() == n.ELEMENT_NODE && strcmp(char(n.getNodeName()), 'FunctionSet')
            a = n.getAttributes().getNamedItem('name');
            if ~isempty(a) && strcmp(char(a.getValue()), 'ControlFunctions')
                fsNode = n; break;
            end
        end
    end
    if isempty(fsNode)
        error('AFO_controller 아래 FunctionSet name="ControlFunctions"를 못 찾음');
    end

    objectsNode = [];
    kids = fsNode.getChildNodes();
    for kk = 0:kids.getLength-1
        n = kids.item(kk);
        if n.getNodeType() == n.ELEMENT_NODE && strcmp(char(n.getNodeName()), 'objects')
            objectsNode = n; break;
        end
    end
    if isempty(objectsNode)
        error('ControlFunctions 아래 <objects>를 못 찾음');
    end

    while objectsNode.hasChildNodes()
        objectsNode.removeChild(objectsNode.getFirstChild());
    end

    g1 = doc.createElement('GCVSpline'); g1.setAttribute('name', col_r);
    e = doc.createElement('half_order');     e.appendChild(doc.createTextNode(num2str(half_order))); g1.appendChild(e);
    e = doc.createElement('error_variance'); e.appendChild(doc.createTextNode(num2str(error_variance))); g1.appendChild(e);
    e = doc.createElement('x');             e.appendChild(doc.createTextNode(x_str));  g1.appendChild(e);
    e = doc.createElement('y');             e.appendChild(doc.createTextNode(yr_str)); g1.appendChild(e);
    e = doc.createElement('weights');       e.appendChild(doc.createTextNode(w_str));  g1.appendChild(e);
    e = doc.createElement('coefficients');  e.appendChild(doc.createTextNode(c_str));  g1.appendChild(e);
    objectsNode.appendChild(g1);

    g2 = doc.createElement('GCVSpline'); g2.setAttribute('name', col_l);
    e = doc.createElement('half_order');     e.appendChild(doc.createTextNode(num2str(half_order))); g2.appendChild(e);
    e = doc.createElement('error_variance'); e.appendChild(doc.createTextNode(num2str(error_variance))); g2.appendChild(e);
    e = doc.createElement('x');             e.appendChild(doc.createTextNode(x_str));  g2.appendChild(e);
    e = doc.createElement('y');             e.appendChild(doc.createTextNode(yl_str)); g2.appendChild(e);
    e = doc.createElement('weights');       e.appendChild(doc.createTextNode(w_str));  g2.appendChild(e);
    e = doc.createElement('coefficients');  e.appendChild(doc.createTextNode(c_str));  g2.appendChild(e);
    objectsNode.appendChild(g2);

    injectedOsimPath = fullfile(resultsDir, sprintf('%s_%d.osim', modelName, i));
    xmlwrite(injectedOsimPath, doc);

    modelProcessor = ModelProcessor(injectedOsimPath);
    problem.setModelProcessor(modelProcessor);
    model = modelProcessor.process();
    if useStateFixed
        % 적분 상태 component는 Moco 문제의 모델에만 추가한다.
        % 저장되는 injected osim은 플러그인 없이도 읽을 수 있게 유지된다.
        moco_WoC_loadFixedEffortPropulsionPlugin();
        if useCoT
            % 대사 모델도 Moco 문제의 모델에만 추가한다 (모든 실제 근육 등록 검사 포함).
            [~, metabolicsInfo] = moco_WoC_addMetabolics(model);
            metabolicRatePath = metabolicsInfo.outputPath;
        end
        addEffortPropulsionIntegrator(model, integratorPath, ...
            gateSigma, gateThreshold, impulseScale, useMeanForce, metabolicRatePath);
        problem.setModelProcessor(ModelProcessor(model));
    end
    modelState = model.initSystem();
    if useCoT
        checkMetabolicEnergyRate(model, modelState, integratorPath, metabolicsInfo);
    end

    %--------------------------------------------------------------
    % 2. Goal 설정
    %--------------------------------------------------------------
    numStates  = model.getNumStateVariables();
    stateNames = model.getStateVariableNames();

    if strcmpi(gaitMode, 'modeSym')
        %----------------------------------------------------------
        % modeSym: 좌우 대칭 goal (반 걸음 → createPeriodicTrajectory)
        %   - jointset 좌표, Activation : _r ↔ _l 교차 페어
        %   - pelvis_tx 제외
        %----------------------------------------------------------
        symmetryGoal = MocoPeriodicityGoal('symmetryGoal');
        problem.addGoal(symmetryGoal);

        for i = 0:numStates-1
            currentStateName = char(stateNames.get(i));
            sName = string(currentStateName);
            if startsWith(sName, "/jointset")
                if contains(sName, "_r")
                    symmetryGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName, ...
                        char(regexprep(sName, "_r", "_l"))));
                elseif contains(sName, "_l")
                    symmetryGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName, ...
                        char(regexprep(sName, "_l", "_r"))));
                else
                    if ~contains(sName, "pelvis_tx/value") && ~contains(sName, "/activation")
                        symmetryGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName));
                    end
                end
            end
        end

        for i = 0:numStates-1
            currentStateName = char(stateNames.get(i));
            sName = string(currentStateName);
            if endsWith(sName, "/activation")
                if contains(sName, "_r")
                    symmetryGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName, ...
                        char(regexprep(sName, "_r", "_l"))));
                elseif contains(sName, "_l")
                    symmetryGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName, ...
                        char(regexprep(sName, "_l", "_r"))));
                end
            end
        end

        symmetryGoal.addControlPair(MocoPeriodicityGoalPair('/lumbarAct'));

    elseif strcmpi(gaitMode, 'modeAsym')
        %----------------------------------------------------------
        % modeAsym: 주기성 goal (한 걸음 전체)
        %   - 모든 state에 동일 side 페어 (끝 = 시작)
        %   - pelvis_tx 제외
        %----------------------------------------------------------
        periodicGoal = MocoPeriodicityGoal('periodicGoal');
        problem.addGoal(periodicGoal);

        for i = 0:numStates-1
            currentStateName = char(stateNames.get(i));
            sName = string(currentStateName);
            if contains(sName, 'pelvis_tx/value')
                continue;
            end
            % 적분 상태는 0에서 시작해 증가하므로 주기성을 걸지 않는다.
            if startsWith(sName, [integratorPath '/'])
                continue;
            end
            periodicGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName));
        end

        periodicGoal.addControlPair(MocoPeriodicityGoalPair('/lumbarAct'));

    else
        error('moco_WoC_loop: 알 수 없는 gaitMode = ''%s''. modeSym 또는 modeAsym 이어야 합니다.', gaitMode);
    end

    if useStateFixed
        % Minimum (minSpeedStateFixed) or maximum (maxSpeedStateFixed) average
        % speed at fixed effort and propulsion.
        %
        %   minimize   +/- v_avg = (pelvis_tx(tf) - pelvis_tx(t0)) / (tf - t0)
        %   subject to E = mocoEnergyTarget, P = mocoPropulsionTarget
        %
        % Moco minimizes the objective, so maxSpeedStateFixed gives the speed
        % goal the weight -mocoSpeedWeight.
        %
        % E is the effort goal of the weighted mode before its weight. P is
        % the gated propulsive impulse Iplus of MeanPropulsiveForce
        % (meanPropPerVel_goal): each foot's heel + front AP force F is
        % gated by g(F) = 1/(1+exp(-(F-F0)/sigma)) and F*g(F) is summed over
        % the feet. Both integrals are states of the
        % EffortPropulsionIntegrator component (plugin
        % osimFixedEffortPropulsion), zero at t0:
        %   z_E' = sum_{c excluding AFO_r/AFO_l} u_c^2,
        %   z_P' = F_r g(F_r) + F_l g(F_l),
        % and the equalities are MocoFinalOutputGoal endpoint constraints
        %   z_E(tf) / d_COM = E0,   impulseScale * z_P(tf) = J.
        % (MocoControlGoal does not support endpoint constraint mode, and an
        % integral endpoint constraint goal makes the constraint Jacobian
        % about 27 times slower to evaluate.) The effort, final time and
        % propulsion costs are not added, so v_avg is the only cost.
        % modeSym solves a half stride whose right and left halves together
        % form one foot's full cycle, so the full stride P is twice the
        % half-stride integral. E (per COM displacement) and v_avg are equal
        % for the half and full stride.
        %
        % mocoPropulsionMetric = 'meanForce' fixes the mean propulsive force
        % m = Iplus / Tplus (Tplus = integral of g(F_r) + g(F_l)) instead of P,
        % as in MeanPropulsiveForce (meanPropPerVel_goal): the component's
        % constant state m and residual state r' = sum g(F) (F - m) with
        % r(t0) = r(tf) = 0 give Iplus = m * Tplus, and m(t0) = J. m is the
        % same for the half and full stride.
        %
        % mocoEnergyMetric = 'metabolicCoT' fixes the metabolic cost of
        % transport instead of E (still an endpoint equality, not a cost):
        % the model also gets the Bhargava2004SmoothedMuscleMetabolics of
        % moco_WoC_addMetabolics (every Muscle component registered), its
        % total_metabolic_rate P_met [W, basal included] drives the
        % component's state z_met' = P_met, z_met(t0) = 0, and
        %   z_met(tf) / (mass * d_COM) = E0  [J/kg/m]
        % is a MocoFinalOutputGoal endpoint constraint (divide_by_mass,
        % divide_by_displacement). modeSym uses the half-stride energy and
        % the half-stride COM displacement (impulseScale does not apply).
        speedWeight = getOpt(opts, 'mocoSpeedWeight', 1.0);
        if strcmp(objectiveMode, 'maxSpeedStateFixed')
            speedWeight = -speedWeight;
        end
        addStateFixedGoals(problem, integratorPath, energyMetric, energyTarget, ...
            opts.mocoPropulsionTarget, speedWeight, useMeanForce);
        energyText = sprintf('E0=%.6g (effort)', energyTarget);
        if useCoT, energyText = sprintf('E0=%.6g J/kg/m (metabolic CoT)', energyTarget); end
        if useMeanForce
            fprintf(['Moco %s: speed goal weight=%.4g, %s, J=%.6g N ', ...
                     '(mean propulsive force, gate sigma %.4g N, F0 %.4g N)\n'], objectiveMode, ...
                     speedWeight, energyText, opts.mocoPropulsionTarget, ...
                     gateSigma, gateThreshold);
        else
            fprintf(['Moco %s: speed goal weight=%.4g, %s, J=%.6g N*s ', ...
                     '(impulse scale %d, gate sigma %.4g N, F0 %.4g N)\n'], objectiveMode, ...
                     speedWeight, energyText, opts.mocoPropulsionTarget, ...
                     impulseScale, gateSigma, gateThreshold);
        end
    else
        % Effort goal (공통)
        effortGoal = MocoControlGoal('effort', weight_effort);
        effortGoal.setExponent(2); % 2로 사용
        effortGoal.setDivideByDisplacement(true);
        effortGoal.setWeightForControlPattern('(^|.*/)AFO_[rl]$', 0);
        problem.addGoal(effortGoal);

        % Final time goal (공통)
        finalTimeGoal = MocoFinalTimeGoal('final_time', weight_finalTime);
        finalTimeGoal.setDivideByDisplacement(true);
        problem.addGoal(finalTimeGoal);
    end

    % Positive anteroposterior GRF impulse goal (optional).
    %
    % The generated GRF columns are the sum of each foot's heel/front
    % sphere-force x-components. MocoOutputExtremumGoal with the 'maximum'
    % mode provides a smooth approximation of max(F_AP, 0). The negative
    % goal weight rewards propulsion because Moco minimizes the objective.
    % Do not divide by the optimized phase duration: this goal must remain
    % proportional to the physical positive impulse (N*s), not mean force.
    % (WoC_moco_main keeps the weight 0 in the min/maxSpeedStateFixed modes.)
    if weight_propulsion > 0
        bodyWeight = model.getTotalMass(modelState) * 9.80665;
        if isempty(propulsionReferenceDuration)
            propulsionReferenceDuration = mean(mocoTimeBound);
            if strcmpi(gaitMode, 'modeAsym')
                propulsionReferenceDuration = 2 * propulsionReferenceDuration;
            end
        end

        if ~isscalar(propulsionReferenceDuration) || ...
                ~isfinite(propulsionReferenceDuration) || ...
                propulsionReferenceDuration <= 0
            error(['moco_WoC_loop: propulsion reference duration must be ', ...
                   'a positive finite scalar.']);
        end

        % Dividing by BW * fixed reference duration makes the user-facing
        % weight dimensionless while preserving the impulse objective.
        weightPerSide = -weight_propulsion / ...
            (2 * bodyWeight * propulsionReferenceDuration);
        addPropulsionGoal(problem, 'r', weightPerSide, propulsionSmoothing);
        addPropulsionGoal(problem, 'l', weightPerSide, propulsionSmoothing);
        % 설정값 로그는 WoC_moco_main 파라미터 블록에서 출력한다.
    end

    %--------------------------------------------------------------
    % 3. Bounds 설정
    %--------------------------------------------------------------
    if strcmpi(gaitMode, 'modeSym')
        % 반 걸음 bounds
        problem.setTimeBounds(0, mocoTimeBound);
        problem.setStateInfo('/jointset/groundPelvis/pelvis_tx/value', [0, 10], 0.0, mocoDistBound);

    elseif strcmpi(gaitMode, 'modeAsym')
        % 한 걸음 bounds (modeSym 대비 시간 및 이동 거리 ~2배)
        problem.setTimeBounds(0, mocoTimeBound*2);
        problem.setStateInfo('/jointset/groundPelvis/pelvis_tx/value', [0, 20], 0.0, mocoDistBound*2);
    end

    % 나머지 bounds (공통)
    % problem.setStateInfo('/jointset/groundPelvis/pelvis_tilt/value', [-20*pi/180, -10*pi/180]);
    % problem.setStateInfo('/jointset/groundPelvis/pelvis_ty/value',   [0.75, 1.25]);
    % problem.setStateInfo('/jointset/hip_l/hip_flexion_l/value',      [-10*pi/180, 60*pi/180]);
    % problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value',      [-10*pi/180, 60*pi/180]);
    % problem.setStateInfo('/jointset/knee_l/knee_angle_l/value',      [-50*pi/180, 0]);
    % problem.setStateInfo('/jointset/knee_r/knee_angle_r/value',      [-50*pi/180, 0]);
    % problem.setStateInfo('/jointset/ankle_l/ankle_angle_l/value',    [-15*pi/180, 25*pi/180]);
    % problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value',    [-15*pi/180, 25*pi/180]);
    % problem.setStateInfo('/jointset/lumbar/lumbar/value',            [0, 20*pi/180]);

    problem.setStateInfo('/jointset/groundPelvis/pelvis_tilt/value', [-20*pi/180, 10*pi/180]);
    problem.setStateInfo('/jointset/groundPelvis/pelvis_ty/value',   [0.75, 1.25]);
    problem.setStateInfo('/jointset/hip_l/hip_flexion_l/value',      [-35*pi/180, 75*pi/180]);
    problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value',      [-35*pi/180, 75*pi/180]);
    problem.setStateInfo('/jointset/knee_l/knee_angle_l/value',      [-90*pi/180, 10*pi/180]);
    problem.setStateInfo('/jointset/knee_r/knee_angle_r/value',      [-90*pi/180, 10*pi/180]);
    problem.setStateInfo('/jointset/ankle_l/ankle_angle_l/value',    [-40*pi/180, 30*pi/180]);
    problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value',    [-40*pi/180, 30*pi/180]);
    problem.setStateInfo('/jointset/lumbar/lumbar/value',            [-20*pi/180, 20*pi/180]);
    %--------------------------------------------------------------
    % 4. Solver 설정
    %--------------------------------------------------------------
    solver = study.initCasADiSolver();
    solver.set_num_mesh_intervals(getOpt(opts, 'mocoMeshIntervals', 300));
    solver.set_verbosity(2);
    solver.set_optim_solver('ipopt');
    solver.set_optim_convergence_tolerance(getOpt(opts, 'mocoConvergenceTolerance', 1e-3));
    solver.set_optim_constraint_tolerance(getOpt(opts, 'mocoConstraintTolerance', 1e-3));
    solver.set_optim_max_iterations(getOpt(opts, 'mocoMaxIterations', 1000));

    guessTraj = MocoTrajectory(guessStoPath);
    meanForceTarget = [];
    if useMeanForce, meanForceTarget = opts.mocoPropulsionTarget; end
    [guessTraj, guessMetrics] = matchIntegratorGuess(guessTraj, model, problem, integratorPath, ...
        useStateFixed, injectedOsimPath, guessStoPath, meanForceTarget, useCoT);
    solver.setGuess(guessTraj);

    %--------------------------------------------------------------
    % 5. 문제 풀기
    %--------------------------------------------------------------
    if useStateFixed
        checkStateFixedGoals(problem, integratorPath, meanForceTarget, energyMetric, ...
            energyTarget, metabolicRatePath);
        checkConstraintOutputsOnGuess(study, problem, model, guessTraj, integratorPath, ...
            energyMetric, useMeanForce, impulseScale, guessMetrics, metabolicRatePath);
    end
    moco_WoC_Solution = study.solve();

end


function addPropulsionGoal(problem, side, weight, smoothing)
% Add one foot's positive AP-GRF impulse reward.
% SpatialVec index 3 is the translational x component (force-x).
    import org.opensim.modeling.*

    goal = MocoOutputExtremumGoal(['propulsion_' side], weight);
    goal.setOutputPath(['/contactHeel_' side '|sphere_force']);
    goal.setSecondOutputPath(['/contactFront_' side '|sphere_force']);
    goal.setOperation('addition');
    goal.setOutputIndex(3);
    goal.setExtremumType('maximum');
    goal.setSmoothingFactor(smoothing);
    goal.setExponent(1);
    goal.setDivideByDisplacement(false);
    goal.setDivideByDuration(false);
    problem.addGoal(goal);
end


function addEffortPropulsionIntegrator(model, componentPath, gateSigma, ...
        gateThreshold, impulseScale, includeMeanForce, metabolicRatePath)
% Add the plugin component that integrates the effort rate and the gated
% propulsive impulse rate of each foot (sum of its heel/front sphere_force
% x-components, as in addPropulsionGoal and MeanPropulsiveForce), plus the
% mean propulsive force states when includeMeanForce is true and the
% metabolic energy state (input connected to metabolicRatePath, e.g.
% /metabolics|total_metabolic_rate) when metabolicRatePath is not empty.
    import org.opensim.modeling.*

    component = Component.safeDownCast( ...
        OpenSimObject.newInstanceOfType('EffortPropulsionIntegrator'));
    component.setName(componentPath(2:end));
    PropertyHelper.setValueDouble(gateSigma, component.updPropertyByName('gate_sigma'));
    PropertyHelper.setValueDouble(gateThreshold, ...
        component.updPropertyByName('gate_threshold'));
    PropertyHelper.setValueDouble(impulseScale, ...
        component.updPropertyByName('impulse_scale'));
    PropertyHelper.setValueBool(includeMeanForce, ...
        component.updPropertyByName('include_mean_force'));
    PropertyHelper.setValueBool(~isempty(metabolicRatePath), ...
        component.updPropertyByName('include_metabolic_energy'));
    if ~isempty(metabolicRatePath)
        component.updInput('metabolic_rate').setConnecteePath(metabolicRatePath);
    end
    sides = {'r', 'right_contact_forces'; 'l', 'left_contact_forces'};
    for k = 1:size(sides, 1)
        input = component.updInput(sides{k, 2});
        input.appendConnecteePath(['/contactHeel_' sides{k, 1} '|sphere_force']);
        input.appendConnecteePath(['/contactFront_' sides{k, 1} '|sphere_force']);
    end
    model.addComponent(component);
    model.finalizeConnections();
end


function addStateFixedGoals(problem, componentPath, energyMetric, energyTarget, ...
        propulsionTarget, speedWeight, useMeanForce)
% Add the v_avg cost (weight speedWeight, negative to maximize), the energy
% endpoint equality constraint (E = effort or metabolic CoT, energyMetric)
% and either the P endpoint equality constraint or (useMeanForce) the
% bounds that fix the mean propulsive force.
    import org.opensim.modeling.*

    speedGoal = MocoGoal.safeDownCast( ...
        OpenSimObject.newInstanceOfType('MocoCoordinateAverageSpeedGoal'));
    speedGoal.setName('average_speed');
    speedGoal.setWeight(speedWeight);
    PropertyHelper.setValueString('/jointset/groundPelvis/pelvis_tx', ...
        speedGoal.updPropertyByName('coordinate_path'));
    problem.addGoal(speedGoal);

    energyGoal = MocoFinalOutputGoal(energyConstraintName(energyMetric));
    if strcmp(energyMetric, 'metabolicCoT')
        % CoT = z_met(tf) / (mass * d_COM) [J/kg/m] (MocoGoal mass and COM
        % displacement of the solved phase; no g, no duration).
        energyGoal.setOutputPath([componentPath '|metabolic_energy']);
        energyGoal.setDivideByMass(true);
        problem.setStateInfo([componentPath '/metabolic_energy'], [-Inf, Inf], 0);
    else
        % E = z_E(tf) / d_COM: same displacement normalization as the effort goal.
        energyGoal.setOutputPath([componentPath '|effort_integral']);
    end
    energyGoal.setDivideByDisplacement(true);
    energyGoal.setMode('endpoint_constraint');
    energyGoal.setEndpointConstraintBounds(equalityBounds(energyTarget));
    problem.addGoal(energyGoal);

    if useMeanForce
        % Mean propulsive force: r(t0) = r(tf) = 0 gives Iplus = m * Tplus,
        % and m(t0) = J (m' = 0 carries it over the phase). Only the
        % initial value of m is bounded so that no defect constraint row
        % has fixed variables only.
        problem.setStateInfo([componentPath '/mean_propulsive_force'], ...
            [-Inf, Inf], propulsionTarget);
        problem.setStateInfo([componentPath '/mean_force_residual'], ...
            [-Inf, Inf], 0, 0);
    else
        % P_gate = impulse_scale * z_P(tf) [N*s, one stride].
        propulsionGoal = MocoFinalOutputGoal('propulsion_constraint');
        propulsionGoal.setOutputPath([componentPath '|scaled_propulsive_impulse']);
        propulsionGoal.setMode('endpoint_constraint');
        propulsionGoal.setEndpointConstraintBounds(equalityBounds(propulsionTarget));
        problem.addGoal(propulsionGoal);
    end

    % Both integrals start at 0 (z_P is only reported when useMeanForce, and
    % z_E only when the energy metric is the metabolic CoT). Their final
    % values are free: no periodicity on the accumulated integrals.
    problem.setStateInfo([componentPath '/effort_integral'], [-Inf, Inf], 0);
    problem.setStateInfo([componentPath '/propulsive_impulse'], [-Inf, Inf], 0);
end


function name = energyConstraintName(energyMetric)
    if strcmp(energyMetric, 'metabolicCoT')
        name = 'metabolic_cot_constraint';
    else
        name = 'effort_constraint';
    end
end


function bounds = equalityBounds(value)
    import org.opensim.modeling.*
    bounds = StdVectorMocoBounds();
    bounds.add(MocoBounds(value));
end


function [guessTraj, m] = matchIntegratorGuess(guessTraj, model, problem, ...
        componentPath, useIntegrator, modelPath, guessPath, meanForceTarget, useCoT)
% Make the guess match the integral states of the problem. Integral states
% already in the guess (from a min/maxSpeedStateFixed solution with either
% energy metric, including the earlier softplus version) are dropped; when
% the mode is used, the states the problem has are rebuilt as running
% integrals along the guess itself. With a mean force target J (not
% empty), m = J and r is the running integral of sum g(F) (F - J): the
% guess then satisfies the dynamics of m and r, and r(tf) = Iplus - J *
% Tplus of the guess is the only violated bound. With useCoT, z_met is the
% running integral of the guess's own metabolic rate (moco_WoC_evalGaitMetrics,
% same metabolics as the problem). m returns the guess metrics ([] when the
% integrator is not used).
    import org.opensim.modeling.*
    m = [];

    statesTable = guessTraj.exportToStatesTable();
    labels = statesTable.getColumnLabels();
    oldStates = {};
    for k = 0:labels.size()-1
        if startsWith(char(labels.get(k)), [componentPath '/'])
            oldStates{end+1} = char(labels.get(k)); %#ok<AGROW>
        end
    end
    if ~useIntegrator && isempty(oldStates)
        return;
    end
    for k = 1:numel(oldStates)
        statesTable.removeColumn(oldStates{k});
    end

    if useIntegrator
        component = model.getComponent(componentPath);
        m = moco_WoC_evalGaitMetrics(modelPath, guessPath, 1, ...
            PropertyHelper.getValueDouble(component.getPropertyByName('gate_sigma')), ...
            PropertyHelper.getValueDouble(component.getPropertyByName('gate_threshold')), ...
            struct('metabolics', useCoT));
        statePaths = {[componentPath '/effort_integral'], [componentPath '/propulsive_impulse']};
        values = [cumtrapz(m.time, m.effort_rate), ...
                  cumtrapz(m.time, m.gate_impulse_rate)];
        if ~isempty(meanForceTarget)
            statePaths = [statePaths, {[componentPath '/mean_propulsive_force'], ...
                                       [componentPath '/mean_force_residual']}];
            values = [values, repmat(meanForceTarget, numel(m.time), 1), ...
                      cumtrapz(m.time, m.gate_impulse_rate - meanForceTarget * m.gate_time_rate)];
            fprintf(['Mean force guess: m = %.6g N, r(tf) = Iplus - J*Tplus = %.4g N*s ', ...
                     '(guess mean force %.6g N)\n'], meanForceTarget, values(end, end), ...
                     trapz(m.time, m.gate_impulse_rate) / trapz(m.time, m.gate_time_rate));
        end
        if useCoT
            statePaths = [statePaths, {[componentPath '/metabolic_energy']}];
            values = [values, cumtrapz(m.time, m.metabolic_rate)];
            fprintf(['Metabolic guess: z_met(tf) = %.6g J, mass %.6g kg, d_COM %.6g m, ', ...
                     'CoT = %.6g J/kg/m (Simpson %.6g)\n'], values(end, end), m.mass_kg, ...
                     m.com_displacement, m.CoT_trapz, m.CoT_simpson);
        end

        % Moco 4.5 maps guess states to the problem by position. The guess
        % columns are in system order and the component's states come last,
        % so append them in their system order.
        systemOrder = opensimSimulation.createStateVariableNamesInSystemOrder(model);
        systemNames = cell(systemOrder.size(), 1);
        for k = 0:systemOrder.size()-1
            systemNames{k+1} = char(systemOrder.get(k));
        end
        [position, order] = sort(cellfun(@(p) find(strcmp(systemNames, p)), statePaths));
        if ~isequal(position, numel(systemNames) - (numel(statePaths)-1:-1:0))
            error('moco_WoC_loop: 적분 상태가 system order의 마지막 %d개 상태가 아닙니다.', ...
                numel(statePaths));
        end
        for k = order
            statesTable.appendColumn(statePaths{k}, Vector.createFromMat(values(:, k)));
        end
    end
    guessTraj = MocoTrajectory.createFromStatesControlsTables( ...
        problem.createRep(), statesTable, guessTraj.exportToControlsTable());
end


function checkStateFixedGoals(problem, componentPath, meanForceTarget, ...
        energyMetric, energyTarget, metabolicRatePath)
% Check on the problem Moco solves that v_avg is the only cost, that the
% energy (effort E or metabolic CoT) and P are endpoint constraints
% (createRep initializes the goals, so an unsupported endpoint constraint
% mode would also throw here) and that both read the optimizer's states:
% MocoCasADiSolver copies them into the SimTK::State only for goals that
% depend on Stage::Time or later.
% With a mean force target J, the energy is the only endpoint constraint
% and the state bounds must be m(t0) = J and r(t0) = r(tf) = 0.
% The energy constraint must read the metric's output with the metric's
% normalization and bounds [E0, E0], the other energy constraint must be
% absent, and no periodicity pair may contain an integral state. With the
% metabolic CoT, every Muscle of the Moco model must be registered in its
% metabolics, whose total_metabolic_rate drives the integrator.
    import org.opensim.modeling.*
    names = @(v) arrayfun(@(k) char(v.get(k)), 0:v.size()-1, 'UniformOutput', false);
    rep = problem.createRep();
    costs = names(rep.createCostNames());
    constraints = names(rep.createEndpointConstraintNames());
    fprintf('Moco costs: %s | endpoint constraints: %s\n', ...
        strjoin(costs, ', '), strjoin(constraints, ', '));
    energyName = energyConstraintName(energyMetric);
    otherEnergyName = setdiff({'effort_constraint', 'metabolic_cot_constraint'}, energyName);
    expected = {energyName, 'propulsion_constraint'};
    if ~isempty(meanForceTarget)
        expected = {energyName};
    end
    if ~isequal(costs, {'average_speed'}) || ~all(ismember(expected, constraints)) || ...
            any(ismember(otherEnergyName, constraints)) || ...
            (~isempty(meanForceTarget) && ismember('propulsion_constraint', constraints))
        error('moco_WoC_loop: min/maxSpeedStateFixed 문제의 cost/endpoint constraint 구성이 예상과 다릅니다.');
    end

    % Energy constraint definition on the problem Moco solves.
    goal = MocoFinalOutputGoal.safeDownCast(rep.getEndpointConstraint(energyName));
    bounds = constraintBounds(goal);
    isCoT = strcmp(energyMetric, 'metabolicCoT');
    expectedOutput = [componentPath '|effort_integral'];
    if isCoT, expectedOutput = [componentPath '|metabolic_energy']; end
    fprintf(['  %s: output %s, divide_by_mass %d, divide_by_displacement %d, ', ...
             'endpoint constraint %d, bounds %s (E0 %g)\n'], energyName, ...
             char(goal.getOutputPath()), goal.getDivideByMass(), goal.getDivideByDisplacement(), ...
             goal.getModeIsEndpointConstraint(), mat2str(bounds), energyTarget);
    if ~strcmp(char(goal.getOutputPath()), expectedOutput) || ...
            goal.getDivideByMass() ~= isCoT || ~goal.getDivideByDisplacement() || ...
            goal.getDivideByDuration() || ~goal.getModeIsEndpointConstraint() || ...
            ~isequal(bounds, [energyTarget, energyTarget])
        error('moco_WoC_loop: %s 의 output/정규화/bound가 %s 정의와 다릅니다.', ...
            energyName, energyMetric);
    end
    energyState = [componentPath '/effort_integral'];
    if isCoT, energyState = [componentPath '/metabolic_energy']; end
    initial = rep.getStateInfo(energyState).getInitialBounds();
    final = rep.getStateInfo(energyState).getFinalBounds();
    if initial.getLower() ~= 0 || initial.getUpper() ~= 0 || final.isSet()
        error('moco_WoC_loop: %s 는 t0에서 0이고 tf에서 자유여야 합니다.', energyState);
    end
    for k = 1:numel(constraints)
        periodic = MocoPeriodicityGoal.safeDownCast(rep.getEndpointConstraint(constraints{k}));
        if isempty(periodic), continue; end
        pairs = periodic.getPropertyByName('state_pairs');
        for p = 0:pairs.size()-1
            pair = pairs.getValueAsObject(p);
            ends = {char(PropertyHelper.getValueString(pair.getPropertyByName('initial_variable'))), ...
                    char(PropertyHelper.getValueString(pair.getPropertyByName('final_variable')))};
            if any(startsWith(ends, [componentPath '/']))
                error('moco_WoC_loop: %s 에 적분 상태의 주기성 pair가 있습니다: %s -> %s', ...
                    constraints{k}, ends{:});
            end
        end
    end
    if isCoT
        repModel = rep.getModelBase();
        metabolicsPath = extractBefore(metabolicRatePath, '|');
        numMuscles = moco_WoC_checkMetabolics(repModel, metabolicsPath);
        connectee = char(repModel.getComponent(componentPath).getInput('metabolic_rate').getConnecteePath());
        fprintf('  Moco model: %d muscles registered in %s; %s/metabolic_rate <- %s\n', ...
            numMuscles, metabolicsPath, componentPath, connectee);
        if ~strcmp(connectee, metabolicRatePath)
            error('moco_WoC_loop: 적분기의 metabolic_rate 입력이 %s 가 아니라 %s 에 연결돼 있습니다.', ...
                metabolicRatePath, connectee);
        end
    end
    if ~isempty(meanForceTarget)
        % m(t0) = J and r(t0) = r(tf) = 0 on the problem Moco solves.
        mInit = rep.getStateInfo([componentPath '/mean_propulsive_force']).getInitialBounds();
        rInfo = rep.getStateInfo([componentPath '/mean_force_residual']);
        rBounds = [rInfo.getInitialBounds().getLower(), rInfo.getInitialBounds().getUpper(), ...
                   rInfo.getFinalBounds().getLower(), rInfo.getFinalBounds().getUpper()];
        fprintf('  mean_propulsive_force initial bounds: [%g, %g] N | mean_force_residual t0/tf bounds: %s\n', ...
            mInit.getLower(), mInit.getUpper(), mat2str(rBounds));
        if mInit.getLower() ~= meanForceTarget || mInit.getUpper() ~= meanForceTarget || any(rBounds ~= 0)
            error('moco_WoC_loop: 평균 추진력 상태의 bound가 m(t0) = J, r(t0) = r(tf) = 0 이 아닙니다.');
        end
    end
    stageNames = {'Topology', 'Model (or lower)', 'Instance', 'Time', 'Position', 'Velocity', ...
                  'Dynamics', 'Acceleration', 'Report'};
    for name = expected
        level = stageLevel(rep.getEndpointConstraint(name{1}).getStageDependency());
        fprintf('  %s stage dependency: %s\n', name{1}, stageNames{level});
        if level < 4
            error('moco_WoC_loop: %s 의 stage dependency(%s)가 Stage::Time보다 낮습니다.', ...
                name{1}, stageNames{level});
        end
    end
end


function checkMetabolicEnergyRate(model, state, componentPath, metabolicsInfo)
% On the model given to Moco: at the same time and pose, change every muscle
% activation and excitation (0.1 -> 0.5) and check that
%   (1) total_metabolic_rate changes and exceeds the basal rate (a model
%       whose muscles are not registered gives the basal constant only), and
%   (2) the integrator's z_met' equals total_metabolic_rate.
    import org.opensim.modeling.*
    metabolics = Bhargava2004SmoothedMuscleMetabolics.safeDownCast( ...
        model.getComponent(metabolicsInfo.componentPath));
    integrator = model.getComponent(componentPath);
    controlNames = opensimSimulation.createControlNamesFromModel(model);
    names = arrayfun(@(k) char(controlNames.get(k)), 0:controlNames.size()-1, ...
        'UniformOutput', false);
    muscleControl = ismember(names, metabolicsInfo.musclePaths);
    if nnz(muscleControl) ~= metabolicsInfo.numMuscles
        error('moco_WoC_loop: 근육 control 수(%d)가 등록 근육 수(%d)와 다릅니다.', ...
            nnz(muscleControl), metabolicsInfo.numMuscles);
    end
    s = State(state);
    levels = [0.1, 0.5];
    rate = zeros(size(levels));
    derivative = zeros(size(levels));
    for i = 1:numel(levels)
        for k = 1:metabolicsInfo.numMuscles
            Muscle.safeDownCast(model.getComponent(metabolicsInfo.musclePaths{k})) ...
                .setActivation(s, levels(i));
        end
        model.realizeVelocity(s);
        u = model.getControls(s).getAsMat();
        u(muscleControl) = levels(i);
        model.setControls(s, Vector.createFromMat(u));
        model.realizeAcceleration(s);
        rate(i) = metabolics.getTotalMetabolicRate(s);
        derivative(i) = integrator.getStateVariableDerivativeValue(s, 'metabolic_energy');
    end
    basal = metabolicsInfo.basal_coefficient * ...
        model.getTotalMass(s) ^ metabolicsInfo.basal_exponent;
    fprintf(['Metabolic rate check (same time/pose): activation/excitation %s -> ', ...
             'P_met %s W (basal %.4g W), z_met'' %s W\n'], mat2str(levels), ...
             mat2str(rate, 8), basal, mat2str(derivative, 8));
    if any(abs(derivative - rate) > 1e-9 * max(1, abs(rate)))
        error('moco_WoC_loop: z_met''이 total_metabolic_rate와 다릅니다.');
    end
    if abs(rate(2) - rate(1)) <= 1e-6 * abs(rate(1)) || max(rate) <= basal * (1 + 1e-6)
        error(['moco_WoC_loop: 근육 activation/excitation을 바꿔도 대사율이 변하지 않거나 ', ...
               'basal 값뿐입니다.']);
    end
end


function checkConstraintOutputsOnGuess(study, problem, model, guessTraj, componentPath, ...
        energyMetric, useMeanForce, impulseScale, guessMetrics, metabolicRatePath)
% Evaluate the constrained outputs along the guess on the model Moco
% solves (MocoStudy.analyze): each output must equal its state column of
% the guess at every time (no stale state) and grow over the phase (not a
% constant). With the metabolic CoT, the metabolic rate the integrator
% reads must equal the independent recomputation of moco_WoC_evalGaitMetrics
% (same states and controls, controls applied by name).
% analyze needs every model control, so a copy of the guess gets the
% controller (PrescribedController AFO) controls added; the solver keeps
% the original guess.
    import org.opensim.modeling.*
    guessTraj = MocoTrajectory.createFromStatesControlsTables(problem.createRep(), ...
        guessTraj.exportToStatesTable(), guessTraj.exportToControlsTable());
    guessTraj.generateControlsFromModelControllers(model);
    isCoT = strcmp(energyMetric, 'metabolicCoT');
    checks = {[componentPath '|effort_integral'], [componentPath '/effort_integral'], 1};
    if isCoT
        checks = {[componentPath '|metabolic_energy'], [componentPath '/metabolic_energy'], 1};
    end
    if ~useMeanForce
        checks(end+1, :) = {[componentPath '|scaled_propulsive_impulse'], ...
            [componentPath '/propulsive_impulse'], impulseScale};
    end
    outputs = checks(:, 1);
    if isCoT, outputs{end+1} = metabolicRatePath; end
    patterns = StdVectorString();
    for k = 1:numel(outputs)
        patterns.add(regexptranslate('escape', outputs{k}));
    end
    table = study.analyze(guessTraj, patterns);
    column = @(label) columnValues(table, label);
    for k = 1:size(checks, 1)
        value = column(checks{k, 1});
        expected = checks{k, 3} * guessTraj.getStateMat(checks{k, 2});
        err = max(abs(value - expected));
        fprintf('  guess %s: tf value %.6g, max |output - %g*state| %.3g\n', ...
            checks{k, 1}, value(end), checks{k, 3}, err);
        if err > 1e-9 * max(1, max(abs(expected))) || ~(value(end) > value(1))
            error('moco_WoC_loop: %s 가 guess 상태와 다르거나 phase 동안 증가하지 않습니다.', ...
                checks{k, 1});
        end
    end
    if isCoT
        rate = column(metabolicRatePath);
        err = max(abs(rate - guessMetrics.metabolic_rate));
        fprintf(['  guess %s (Moco model) vs independent recomputation: max |diff| %.3g W ', ...
                 '(range %.4g-%.4g W)\n'], metabolicRatePath, err, min(rate), max(rate));
        if err > 1e-6 * max(abs(rate))
            error('moco_WoC_loop: Moco 모델의 대사율이 독립 재계산과 다릅니다 (max %.3g W).', err);
        end
    end
end


function bounds = constraintBounds(goal)
% [lower, upper] of each endpoint constraint bound of a MocoGoal, one row per
% goal output. MocoGoal.getConstraintInfo() is opaque in the MATLAB
% binding, so read its unnamed MocoConstraintInfo property instead.
    import org.opensim.modeling.*
    info = goal.getPropertyByName('MocoConstraintInfo').getValueAsObject(0);
    list = info.getPropertyByName('bounds');
    bounds = zeros(list.size(), 2);
    for k = 0:list.size()-1
        b = MocoBounds.safeDownCast(list.getValueAsObject(k));
        bounds(k+1, :) = [b.getLower(), b.getUpper()];
    end
end


function v = columnValues(table, label)
    col = table.getDependentColumn(label);
    v = zeros(col.size(), 1);
    for i = 0:col.size()-1
        v(i+1) = col.get(i);
    end
end


function level = stageLevel(stage)
% SimTK::Stage level (Topology = 1, Model = 2, Instance = 3, Time = 4, ...).
% The MATLAB binding has no level accessor, so count the steps down through
% the runtime stages (Instance ... Report); Model or lower returns 2.
    level = 2;
    while stage.isInRuntimeRange()
        level = level + 1;
        stage = stage.prev();
    end
end


function val = getOpt(s, field, defaultVal)
    if isfield(s, field) && ~isempty(s.(field))
        val = s.(field);
    else
        val = defaultVal;
    end
end
