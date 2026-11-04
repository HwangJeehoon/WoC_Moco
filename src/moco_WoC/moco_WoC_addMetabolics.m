function [metabolics, info] = moco_WoC_addMetabolics(model)
% moco_WoC_addMetabolics
%
%   Bhargava2004SmoothedMuscleMetabolics를 모델의 모든 실제 Muscle component에
%   연결해 추가한다. 최적화(moco_WoC_loop의 metabolic CoT 제약)와 결과 재계산
%   (moco_WoC_evalGaitMetrics, moco_WoC_getResult)이 같은 설정을 쓰도록 이 함수만
%   대사 모델을 만든다.
%
%   설정 (moco_afo_optim의 addMetabolics / buildModelWithMetabolics와 같다):
%     name '/metabolics', use_smoothing = true, 나머지 property는 OpenSim 기본값
%     (basal_coefficient 1.2, basal_exponent 1, muscle_effort_scaling_factor 1,
%      include_negative_mechanical_work true, forbid_negative_total_power true,
%      enforce_minimum_heat_rate_per_muscle true, smoothing_type tanh, ...).
%     total_metabolic_rate [W]에는 basal 항 basal_coefficient * mass^basal_exponent가
%     포함되며 빼지 않는다.
%
%   근육은 model.getMuscles()가 아니라 전체 component 목록에서 찾는다. 이 모델의
%   근육은 <ForceSet>이 아니라 root <components>에 있어 getMuscles()가 비어 있다.
%   추가한 뒤 finalizeConnections()로 socket을 연결하고, 근육마다 entry가 정확히
%   하나이며 그 muscle socket이 그 근육에 연결되는지 검사한다.
%
%   입력:
%     model : org.opensim.modeling.Model. 비어 있으면([]) 모델에 추가하지 않고
%             설정만 info로 돌려준다 (실행 전 메타데이터 기록용).
%   출력:
%     metabolics : 추가한 component (model이 비면 모델에 붙지 않은 component)
%     info       : struct (componentPath, settings 문자열, 개별 설정, 근육 수/경로)

    import org.opensim.modeling.*

    componentName = 'metabolics';
    metabolics = Bhargava2004SmoothedMuscleMetabolics();
    metabolics.setName(componentName);
    metabolics.set_use_smoothing(true);

    info = struct();
    info.componentPath = ['/' componentName];
    info.outputPath = ['/' componentName '|total_metabolic_rate'];
    info.use_smoothing = logical(metabolics.get_use_smoothing());
    info.smoothing_type = char(metabolics.get_smoothing_type());
    info.velocity_smoothing = metabolics.get_velocity_smoothing();
    info.power_smoothing = metabolics.get_power_smoothing();
    info.heat_rate_smoothing = metabolics.get_heat_rate_smoothing();
    info.basal_coefficient = metabolics.get_basal_coefficient();
    info.basal_exponent = metabolics.get_basal_exponent();
    info.muscle_effort_scaling_factor = metabolics.get_muscle_effort_scaling_factor();
    info.include_negative_mechanical_work = logical(metabolics.get_include_negative_mechanical_work());
    info.forbid_negative_total_power = logical(metabolics.get_forbid_negative_total_power());
    info.enforce_minimum_heat_rate_per_muscle = logical(metabolics.get_enforce_minimum_heat_rate_per_muscle());
    info.use_force_dependent_shortening_prop_constant = ...
        logical(metabolics.get_use_force_dependent_shortening_prop_constant());
    info.settings = sprintf(['Bhargava2004SmoothedMuscleMetabolics %s: use_smoothing=%d ', ...
        'smoothing_type=%s velocity_smoothing=%g power_smoothing=%g heat_rate_smoothing=%g ', ...
        'basal_coefficient=%g basal_exponent=%g muscle_effort_scaling_factor=%g ', ...
        'include_negative_mechanical_work=%d forbid_negative_total_power=%d ', ...
        'enforce_minimum_heat_rate_per_muscle=%d use_force_dependent_shortening_prop_constant=%d ', ...
        'muscle parameters default; total_metabolic_rate includes basal'], ...
        info.componentPath, info.use_smoothing, info.smoothing_type, info.velocity_smoothing, ...
        info.power_smoothing, info.heat_rate_smoothing, info.basal_coefficient, info.basal_exponent, ...
        info.muscle_effort_scaling_factor, info.include_negative_mechanical_work, ...
        info.forbid_negative_total_power, info.enforce_minimum_heat_rate_per_muscle, ...
        info.use_force_dependent_shortening_prop_constant);
    info.musclePaths = {};
    info.numMuscles = 0;
    if isempty(model)
        return;
    end

    % ModelProcessor.process()가 돌려준 모델은 component 목록이 아직 만들어지지
    % 않았으므로 먼저 property로부터 component tree를 만든다.
    model.finalizeFromProperties();
    % 같은 모델에 대사 모델이 이미 있으면 중복으로 더하지 않고 멈춘다.
    existing = metabolicsComponents(model);
    if ~isempty(existing)
        error('moco_WoC_addMetabolics: 모델에 이미 대사 component가 있습니다: %s', ...
            strjoin(existing, ', '));
    end

    [musclePaths, muscleNames] = findMuscles(model);
    if isempty(musclePaths)
        error('moco_WoC_addMetabolics: 모델에서 Muscle component를 찾지 못했습니다.');
    end
    if numel(unique(muscleNames)) ~= numel(muscleNames)
        error('moco_WoC_addMetabolics: 이름이 같은 근육이 있어 entry 이름이 겹칩니다.');
    end
    for k = 1:numel(musclePaths)
        metabolics.addMuscle(muscleNames{k}, Muscle.safeDownCast(model.getComponent(musclePaths{k})));
    end
    model.addComponent(metabolics);
    model.finalizeConnections();
    metabolics = Bhargava2004SmoothedMuscleMetabolics.safeDownCast( ...
        model.getComponent(info.componentPath));

    moco_WoC_checkMetabolics(model, info.componentPath);
    info.musclePaths = musclePaths;
    info.numMuscles = numel(musclePaths);
end


function [paths, names] = findMuscles(model)
    import org.opensim.modeling.*
    paths = {}; names = {};
    list = model.getComponentsList();
    it = list.begin();
    while ~it.equals(list.end())
        muscle = Muscle.safeDownCast(it.deref());
        if ~isempty(muscle)
            paths{end+1} = char(muscle.getAbsolutePathString()); %#ok<AGROW>
            names{end+1} = char(muscle.getName()); %#ok<AGROW>
        end
        it.next();
    end
end


function names = metabolicsComponents(model)
    names = {};
    list = model.getComponentsList();
    it = list.begin();
    while ~it.equals(list.end())
        comp = it.deref();
        if any(strcmp(char(comp.getConcreteClassName()), ...
                {'Bhargava2004SmoothedMuscleMetabolics', 'Umberger2010MuscleMetabolicsProbe', ...
                 'Bhargava2004MuscleMetabolicsProbe'}))
            names{end+1} = char(comp.getAbsolutePathString()); %#ok<AGROW>
        end
        it.next();
    end
end
