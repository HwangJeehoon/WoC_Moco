function G = moco_WoC_controlGroups(model, profile)
% moco_WoC_controlGroups
%
%   모델 control을 component class로 분류한다 (이름의 _r/_l 접미사로 근육을
%   판정하지 않는다: Rajagopal의 상지 토크 구동기도 _r/_l로 끝난다).
%
%   입력:
%     model   : component tree가 만들어진 Model (initSystem 또는
%               finalizeConnections 이후)
%     profile : moco_WoC_modelProfile 결과 (lumbar 구동기 경로)
%   출력 G (struct, 각 필드는 createControlNamesFromModel 순서의 열 벡터):
%     names        : control 이름 (= actuator 절대 경로)
%     isPrescribed : controller(PrescribedController AFO 등)가 정하는 control
%     isMuscle     : Muscle component
%     isAfo        : AFO_r / AFO_l
%     isLumbar     : profile.lumbarControlPaths
%     isTorque     : 근육·AFO가 아닌 이상적 구동기 (lumbar 포함)
%     optimizationControls : Moco가 최적화하는 control (prescribed 제외)

    import org.opensim.modeling.*
    controlNames = opensimSimulation.createControlNamesFromModel(model);
    n = controlNames.size();
    G.names = cell(n, 1);
    for k = 0:n-1
        G.names{k+1} = char(controlNames.get(k));
    end
    prescribed = controlledActuatorPaths(model);
    G.isPrescribed = ismember(G.names, prescribed);
    G.isMuscle = false(n, 1);
    for k = 1:n
        G.isMuscle(k) = ~isempty(Muscle.safeDownCast(model.getComponent(G.names{k})));
    end
    G.isAfo = ismember(regexprep(G.names, '^.*/', ''), {'AFO_r', 'AFO_l'});
    G.isLumbar = ismember(G.names, profile.lumbarControlPaths);
    G.isTorque = ~G.isMuscle & ~G.isAfo;
    G.optimizationControls = G.names(~G.isPrescribed);
end


function paths = controlledActuatorPaths(model)
% 모델의 controller(PrescribedController 등)가 control을 정하는 actuator 경로.
    import org.opensim.modeling.*
    paths = {};
    list = model.getComponentsList();
    it = list.begin();
    while ~it.equals(list.end())
        controller = Controller.safeDownCast(it.deref());
        if ~isempty(controller)
            socket = controller.getSocket('actuators');
            for i = 0:socket.getNumConnectees()-1
                paths{end+1} = char(Component.safeDownCast( ...
                    socket.getConnecteeAsObject(i)).getAbsolutePathString()); %#ok<AGROW>
            end
        end
        it.next();
    end
end
