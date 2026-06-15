% set_AFO_optimal_force.m
%
% models/ 폴더의 .osim 파일에서 AFO_r, AFO_l PathActuator 의
% <optimal_force> 를 파일명의 질량(_NNkg) 에 비례해 일괄 수정.
%
% 사용법:
%   1) 아래 설정 섹션에서 BASE_FORCE_50KG 와 DRY_RUN 을 지정
%   2) 스크립트 실행

clear;

%% ── 설정 ──────────────────────────────────────────────────────────────────
BASE_FORCE_50KG = 50;    % 50kg 기준 optimal_force 값 (N)
                         % 예) 50 → _60kg 모델은 60 N, _90kg 모델은 90 N

DRY_RUN = true;          % true : 실제 파일 수정 없이 변경 내용만 출력
                         % false: 파일 직접 수정

%% ── 모델 폴더 탐색 ─────────────────────────────────────────────────────────
thisDir   = fileparts(mfilename('fullpath'));
modelsDir = fullfile(thisDir, '..', 'models');

files = dir(fullfile(modelsDir, '*.osim'));
fprintf('발견한 .osim 파일 수: %d\n', numel(files));
if DRY_RUN
    fprintf('[DRY_RUN 모드] 파일은 수정되지 않습니다.\n');
end
fprintf('\n');

nModified = 0;
nSkipped  = 0;

for k = 1:numel(files)
    fname = files(k).name;
    fpath = fullfile(files(k).folder, fname);

    % 파일명에서 질량 파싱 (_NNkg 패턴)
    tok = regexp(fname, '_(\d+)kg', 'tokens', 'once');
    if isempty(tok)
        fprintf('[건너뜀] 질량 정보 없음: %s\n', fname);
        nSkipped = nSkipped + 1;
        continue;
    end
    mass_kg   = str2double(tok{1});
    new_force = BASE_FORCE_50KG * (mass_kg / 50);

    % XML 읽기 (현재 값 확인 + 수정 공용)
    doc = xmlread(fpath);
    [cur_r, cur_l] = readAFOForces(doc);

    fprintf('[%s]  mass=%dkg  AFO_r: %g→%g  AFO_l: %g→%g\n', ...
        fname, mass_kg, cur_r, new_force, cur_l, new_force);

    if ~DRY_RUN
        setAFOForces(doc, new_force);
        xmlwrite(fpath, doc);
        nModified = nModified + 1;
    end
end

fprintf('\n');
if DRY_RUN
    fprintf('DRY_RUN 완료 (파일 변경 없음). 적용하려면 DRY_RUN = false 로 설정 후 재실행.\n');
else
    fprintf('수정 완료: %d개  건너뜀: %d개\n', nModified, nSkipped);
end


%% ── 로컬 함수 ──────────────────────────────────────────────────────────────

function [force_r, force_l] = readAFOForces(doc)
% AFO_r, AFO_l PathActuator 의 현재 optimal_force 값을 읽어 반환.
    force_r = NaN;
    force_l = NaN;
    list = doc.getElementsByTagName('PathActuator');
    for j = 0:list.getLength()-1
        node = list.item(j);
        name = char(node.getAttribute('name'));
        val  = getOptimalForce(node);
        if strcmp(name, 'AFO_r'), force_r = val; end
        if strcmp(name, 'AFO_l'), force_l = val; end
    end
end

function setAFOForces(doc, new_force)
% AFO_r, AFO_l PathActuator 의 optimal_force 를 new_force 로 수정.
    list = doc.getElementsByTagName('PathActuator');
    for j = 0:list.getLength()-1
        node = list.item(j);
        name = char(node.getAttribute('name'));
        if strcmp(name, 'AFO_r') || strcmp(name, 'AFO_l')
            ofNodes = node.getElementsByTagName('optimal_force');
            if ofNodes.getLength() > 0
                ofNodes.item(0).getFirstChild().setNodeValue(num2str(new_force));
            end
        end
    end
end

function val = getOptimalForce(pathActuatorNode)
% PathActuator 노드에서 optimal_force 값을 읽음.
    val = NaN;
    ofNodes = pathActuatorNode.getElementsByTagName('optimal_force');
    if ofNodes.getLength() > 0
        txt = char(ofNodes.item(0).getFirstChild().getNodeValue());
        val = str2double(strtrim(txt));
    end
end
