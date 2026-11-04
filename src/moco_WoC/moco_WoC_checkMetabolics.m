function numMuscles = moco_WoC_checkMetabolics(model, componentPath)
% moco_WoC_checkMetabolics
%
%   모델의 모든 실제 Muscle component가 대사 component(componentPath, 기본
%   '/metabolics')에 빠짐없이 등록됐는지 검사하고 근육 수를 돌려준다.
%     - 전체 component 목록의 Muscle 수 > 0 (model.getMuscles()는 쓰지 않는다)
%     - 근육마다 muscle socket이 그 근육으로 연결된 entry가 정확히 하나
%     - entry 수 = getNumMetabolicMuscles() = 근육 수
%   하나라도 어긋나면 error. 연결이 끝난 모델(finalizeConnections 또는
%   initSystem 이후)에서 호출한다.

    import org.opensim.modeling.*
    if nargin < 2 || isempty(componentPath)
        componentPath = '/metabolics';
    end
    if ~model.hasComponent(componentPath)
        error('moco_WoC_checkMetabolics: 대사 component가 없습니다: %s', componentPath);
    end
    metabolics = Bhargava2004SmoothedMuscleMetabolics.safeDownCast( ...
        model.getComponent(componentPath));
    if isempty(metabolics)
        error('moco_WoC_checkMetabolics: %s 는 Bhargava2004SmoothedMuscleMetabolics가 아닙니다.', ...
            componentPath);
    end

    musclePaths = {};
    connected = {};
    list = model.getComponentsList();
    it = list.begin();
    while ~it.equals(list.end())
        comp = it.deref();
        path = char(comp.getAbsolutePathString());
        if ~isempty(Muscle.safeDownCast(comp))
            musclePaths{end+1} = path; %#ok<AGROW>
        elseif startsWith(path, [componentPath '/']) && strcmp(char(comp.getConcreteClassName()), ...
                'Bhargava2004SmoothedMuscleMetabolics_MuscleParameters')
            connectee = Component.safeDownCast(comp.getSocket('muscle').getConnecteeAsObject());
            connected{end+1} = char(connectee.getAbsolutePathString()); %#ok<AGROW>
        end
        it.next();
    end

    numMuscles = numel(musclePaths);
    problems = {};
    if numMuscles == 0
        problems{end+1} = '모델에 Muscle component가 없습니다';
    end
    for k = 1:numMuscles
        count = sum(strcmp(connected, musclePaths{k}));
        if count ~= 1
            problems{end+1} = sprintf('%s: entry %d개', musclePaths{k}, count); %#ok<AGROW>
        end
    end
    extra = setdiff(connected, musclePaths);
    if ~isempty(extra)
        problems{end+1} = sprintf('근육이 아닌 대상에 연결된 entry: %s', strjoin(extra, ', '));
    end
    numEntries = metabolics.getNumMetabolicMuscles();
    if numel(connected) ~= numMuscles || numEntries ~= numMuscles
        problems{end+1} = sprintf('entry %d개 (getNumMetabolicMuscles %d), 근육 %d개', ...
            numel(connected), numEntries, numMuscles);
    end
    if ~isempty(problems)
        error('moco_WoC_checkMetabolics: 근육 등록 검사 실패 (%s):\n  %s', componentPath, ...
            strjoin(problems, '\n  '));
    end
end
