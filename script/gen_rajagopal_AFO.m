function report = gen_rajagopal_AFO(balanceRepoDir)
%GEN_RAJAGOPAL_AFO Build the balance-exo-sim subject19 model with zero-output AFOs.
%   report = gen_rajagopal_AFO() locates the sibling balance-exo-sim-master
%   checkout, including the extra directory produced by extracting its ZIP.
%   An explicit repository root can also be supplied as balanceRepoDir.
%
%   The preprocessing mirrors code/result.py:create_model_processor_base:
%   lumbar_stiffness = 1, reserve_strength = 0, explicit tendon dynamics.
%   Subtalar/MTP and upper-limb motion are retained; only wrists are welded.
%   Gastrocnemius/soleus (6 muscles) have compliant tendons; 74 are rigid.
%   Source contact forces, passive forces, body properties and muscle paths
%   are retained. The 68.5 kg subject19 model replaces the previous generic
%   model. No gait optimization or run_stateFixed changes are performed.
%
%   AFO geometry comes from models/Rajagopal2015_pc.osim. Attachment points
%   are scaled by each body's subject19/reference mesh scale-factor ratio.
%   /AFO_r and /AFO_l are root PathActuators (optimal_force = 300 N), driven
%   by AFO_controller/ControlFunctions with zero GCVSplines compatible with
%   the XML injection used by moco_WoC_loop.m.
%
%   The saved candidate must pass trajectory compatibility, zero-AFO force
%   and dynamics checks before replacing models/rajagopal_AFO.osim.
%   Requires the configured MATLAB OpenSim 4.5.x bindings.

import org.opensim.modeling.*
repoRoot = fileparts(fileparts(mfilename('fullpath')));
if nargin < 1 || isempty(balanceRepoDir)
    balanceRepoDir = fullfile(repoRoot, '..', 'Moco', 'balance-exo-sim-master');
end
if ~isfile(fullfile(balanceRepoDir, 'code', 'result.py'))
    balanceRepoDir = fullfile(balanceRepoDir, 'balance-exo-sim-master');
end
sourcePath = fullfile(balanceRepoDir, 'results', 'experiments', ...
    'subject19', 'subject19_final.osim');
trajectoryPath = fullfile(balanceRepoDir, 'results', 'unperturbed', ...
    'subject19', 'unperturbed.sto');
afoReferencePath = fullfile(repoRoot, 'models', 'Rajagopal2015_pc.osim');
controlReferencePath = fullfile(repoRoot, 'models', '2D_gait_AFO_pc.osim');
outputPath = fullfile(repoRoot, 'models', 'rajagopal_AFO.osim');
inputs = {sourcePath, trajectoryPath, afoReferencePath, controlReferencePath};
assert(all(cellfun(@isfile, inputs)), 'gen_rajagopal_AFO:MissingInput', ...
    'Missing input; check balanceRepoDir and the reference models.');
inputHashes = cellfun(@fileHash, inputs, 'UniformOutput', false);
previousLevel = Logger.getLevel();
loggerCleanup = onCleanup(@() Logger.setLevel(previousLevel)); %#ok<NASGU>
Logger.setLevelString('error');

native = preprocessSubject19(sourcePath);
model = Model(native);
model.setName('rajagopal_AFO');
model.initSystem();  % A copied Model needs its component tree rebuilt.
attachments = addAfo(model, afoReferencePath, controlReferencePath);
model.finalizeConnections();
model.initSystem();

candidatePath = [tempname(fullfile(repoRoot, 'models')) '.osim'];
candidateCleanup = onCleanup(@() deleteIfPresent(candidatePath)); %#ok<NASGU>
model.print(candidatePath);
report = verifySavedModel(native, candidatePath, trajectoryPath);
assert(isequal(inputHashes, cellfun(@fileHash, inputs, 'UniformOutput', false)), ...
    'gen_rajagopal_AFO:InputChanged', 'An input/reference file changed.');
[ok, message] = movefile(candidatePath, outputPath, 'f');
assert(ok, 'gen_rajagopal_AFO:PublishFailed', '%s', message);
report.outputPath = outputPath;
report.sourcePath = sourcePath;
report.trajectoryPath = trajectoryPath;
report.attachments = attachments;
report.openSimVersion = char(opensimCommon.GetVersion());

fprintf('Created: %s\n', outputPath);
fprintf('Source: subject19, mass %.3f kg, %d muscles, %d coordinates\n', ...
    report.mass, report.muscles, report.coordinates);
fprintf('Subtalar/MTP unlocked; wrists welded; shoulders/elbows/forearms retained.\n');
fprintf('Compliant muscles: %s\n', strjoin(report.compliantMuscles, ', '));
fprintf('Trajectory compatible: %d states, %d decision controls; both AFOs prescribed.\n', ...
    report.states, report.controls);
fprintf('Zero-AFO max GRF difference %.3g N; max state-derivative difference %.3g\n', ...
    report.maxGrfDifference, report.maxDerivativeDifference);
fprintf('Mean vertical GRF %.6f N (BW %.6f N)\n', report.meanVerticalForce, report.bodyWeight);
fprintf('AFO ankle moment arms over checked poses: right [%g, %g], left [%g, %g] m\n', ...
    min(report.ankleMomentArms(:, 1)), max(report.ankleMomentArms(:, 1)), ...
    min(report.ankleMomentArms(:, 2)), max(report.ankleMomentArms(:, 2)));
fprintf('All model, controller, trajectory and input-preservation checks passed.\n');
end


function model = preprocessSubject19(sourcePath)
% Keep this sequence aligned with balance-exo-sim/code/result.py:80-187.
import org.opensim.modeling.*
model = Model(sourcePath);
state = model.initSystem();
mass = model.getTotalMass(state);
assert(abs(mass - 68.5) < 1e-9, 'Unexpected subject19 body mass.');
assert(model.getControllerSet().getSize() == 0, 'Unexpected source controller.');
for name = {'subtalar_angle_l', 'subtalar_angle_r', 'mtp_angle_l', 'mtp_angle_r'}
    model.updCoordinateSet().get(name{1}).set_locked(false);
end

names = {'arm_flex', 'arm_add', 'arm_rot', 'elbow_flex', 'pro_sup'};
strengths = [0.5, 0.5, 0.5, 0.2, 0.2];
for i = 1:numel(names)
    for side = {'_l', '_r'}
        addTorqueActuator(model, [names{i} side{1}], strengths(i) * mass);
    end
end
names = {'lumbar_extension', 'lumbar_bending', 'lumbar_rotation'};
stiffnesses = [1.0, 1.5, 0.5];
for i = 1:numel(names)
    addTorqueActuator(model, names{i}, mass);
    spring = SpringGeneralizedForce(names{i});
    spring.setName(['passive_stiffness_' names{i}]);
    spring.setStiffness(stiffnesses(i) * mass);
    spring.setViscosity(2.0);
    model.addForce(spring);
end
model.finalizeConnections();
processor = ModelProcessor(model);
wrists = StdVectorString();
wrists.add('radius_hand_l');
wrists.add('radius_hand_r');
processor.append(ModOpReplaceJointsWithWelds(wrists));
processor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
processor.append(ModOpIgnoreTendonCompliance());
processor.append(ModOpFiberDampingDGF(0.01));
model = processor.process();
model.initSystem();
for i = 0:model.getMuscles().getSize()-1
    muscle = DeGrooteFregly2016Muscle.safeDownCast(model.updMuscles().get(i));
    name = char(muscle.getName());
    if contains(name, 'gas') || contains(name, 'soleus')
        muscle.set_ignore_tendon_compliance(false);
        muscle.set_tendon_strain_at_one_norm_force(0.10);
        muscle.set_passive_fiber_strain_at_one_norm_force(2.0);
    end
end
model.finalizeConnections();
model.initSystem();
end


function addTorqueActuator(model, coordinate, strength)
import org.opensim.modeling.*
actuator = ActivationCoordinateActuator();
actuator.set_coordinate(coordinate);
actuator.setName(['torque_' coordinate]);
actuator.setOptimalForce(strength);
actuator.setMinControl(-1.0);
actuator.setMaxControl(1.0);
model.addForce(actuator);
end


function attachments = addAfo(model, afoReferencePath, controlReferencePath)
import org.opensim.modeling.*
reference = Model(afoReferencePath);
reference.initSystem();
controls = Model(controlReferencePath);
controls.initSystem();
referenceController = PrescribedController.safeDownCast( ...
    controls.getControllerSet().get('AFO_controller'));
controller = PrescribedController();
controller.setName('AFO_controller');
controller.set_enabled(true);
controller.set_interpolation_method(referenceController.get_interpolation_method());
attachments = struct('actuator', {}, 'frame', {}, 'scaleRatio', {}, 'location', {});
for name = {'AFO_r', 'AFO_l'}
    donor = PathActuator.safeDownCast(reference.getForceSet().get(name{1}));
    assert(donor.getGeometryPath().getWrapSet().getSize() == 0, ...
        'The AFO reference contains unsupported wrapping.');
    actuator = PathActuator();
    actuator.setName(name{1});
    actuator.set_appliesForce(donor.get_appliesForce());
    actuator.setMinControl(donor.getMinControl());
    actuator.setMaxControl(donor.getMaxControl());
    actuator.setOptimalForce(donor.getOptimalForce());
    points = donor.getGeometryPath().getPathPointSet();
    for i = 0:points.getSize()-1
        point = PathPoint.safeDownCast(points.get(i));
        assert(~isempty(point), 'Only fixed PathPoints are supported for this AFO.');
        framePath = char(point.getParentFrame().getAbsolutePathString());
        frame = PhysicalFrame.safeDownCast(model.getComponent(framePath));
        [donorScale, meshName] = bodyMeshScale(point.getParentFrame(), '');
        targetScale = bodyMeshScale(frame, meshName);
        ratio = targetScale ./ donorScale;
        location = vector3(point.get_location()) .* ratio;
        actuator.addNewPathPoint(char(point.getName()), frame, Vec3(location(1), location(2), location(3)));
        attachments(end+1) = struct('actuator', name{1}, 'frame', framePath, ...
            'scaleRatio', ratio, 'location', location); %#ok<AGROW>
    end
    actuator.updGeometryPath().setDefaultColor(donor.getGeometryPath().getDefaultColor());
    model.addComponent(actuator);  % /AFO_r, /AFO_l, not /forceset/AFO_*
end
model.finalizeConnections();
for name = {'AFO_r', 'AFO_l'}
    controller.addActuator(Actuator.safeDownCast(model.getComponent(['/' name{1}])));
    spline = GCVSpline.safeDownCast(referenceController.get_ControlFunctions().get(name{1}).clone());
    assert(~isempty(spline), 'Expected a GCVSpline reference.');
    y = spline.getPropertyByName('y');
    for i = 0:y.size()-1
        assert(PropertyHelper.getValueDouble(y, i) == 0, 'AFO reference spline must be zero.');
    end
    controller.prescribeControlForActuator(name{1}, spline);
end
model.addController(controller);
end


function [scale, meshName] = bodyMeshScale(frame, requestedMesh)
% Compare the same underlying mesh so its surface attachment is preserved.
import org.opensim.modeling.*
body = Body.safeDownCast(frame);
assert(~isempty(body), 'AFO attachment must be on a Body frame.');
count = body.getPropertyByName('attached_geometry').size();
for i = 0:count-1
    mesh = Mesh.safeDownCast(body.get_attached_geometry(i));
    if isempty(mesh), continue; end
    [~, name, extension] = fileparts(char(mesh.get_mesh_file()));
    meshName = [name extension];
    if isempty(requestedMesh) || strcmpi(meshName, requestedMesh)
        scale = vector3(mesh.get_scale_factors());
        assert(all(isfinite(scale) & scale > 0), 'Invalid body mesh scale factors.');
        return
    end
end
error('gen_rajagopal_AFO:MeshNotFound', 'No matching mesh on body %s.', char(body.getName()));
end


function report = verifySavedModel(native, candidatePath, trajectoryPath)
import org.opensim.modeling.*
model = Model(candidatePath);
state = model.initSystem();
baseState = native.initSystem();
report.mass = model.getTotalMass(state);
report.bodyWeight = report.mass * abs(model.getGravity().get(1));
report.coordinates = model.getCoordinateSet().getSize();
report.muscles = model.getMuscles().getSize();
assert(strcmp(char(model.getName()), 'rajagopal_AFO'), 'Wrong model name.');
assert(abs(report.mass - native.getTotalMass(baseState)) < 1e-10, 'Body mass changed.');
assert(report.coordinates == 35 && report.muscles == 80, 'Unexpected coordinate/muscle count.');
assert(model.getBodySet().isEqualTo(native.getBodySet()), 'Body properties changed.');
assert(model.getJointSet().isEqualTo(native.getJointSet()), 'Joint properties changed.');
assert(model.getContactGeometrySet().isEqualTo(native.getContactGeometrySet()), 'Contact geometry changed.');
assert(model.getForceSet().isEqualTo(native.getForceSet()), 'Native force properties changed.');
assert(model.getMarkerSet().isEqualTo(native.getMarkerSet()), 'Markers changed.');
for i = 0:model.getCoordinateSet().getSize()-1
    assert(~model.getCoordinateSet().get(i).get_locked(), 'Locked coordinate remains.');
end
for side = {'r', 'l'}
    for joint = {'subtalar', 'mtp'}
        j = model.getJointSet().get([joint{1} '_' side{1}]);
        assert(strcmp(char(j.getConcreteClassName()), 'PinJoint'), 'Foot DOF was welded.');
    end
    assert(strcmp(char(model.getJointSet().get(['radius_hand_' side{1}]).getConcreteClassName()), 'WeldJoint'), ...
        'Wrist is not welded.');
    assert(strcmp(char(model.getJointSet().get(['acromial_' side{1}]).getConcreteClassName()), 'CustomJoint'), ...
        'Shoulder motion changed.');
    afo = PathActuator.safeDownCast(model.getComponent(['/AFO_' side{1}]));
    assert(~isempty(afo) && afo.getOptimalForce() == 300, 'AFO actuator missing or incorrectly scaled.');
end
report.compliantMuscles = {};
for i = 0:report.muscles-1
    muscle = DeGrooteFregly2016Muscle.safeDownCast(model.getMuscles().get(i));
    assert(~isempty(muscle), 'Muscle was not converted to DGF.');
    if ~muscle.get_ignore_tendon_compliance()
        report.compliantMuscles{end+1} = char(muscle.getName());
        assert(muscle.get_tendon_strain_at_one_norm_force() == 0.10 && ...
            muscle.get_passive_fiber_strain_at_one_norm_force() == 2.0, 'Plantarflexor parameters changed.');
    end
end
expected = {'gaslat_r', 'gasmed_r', 'soleus_r', 'gaslat_l', 'gasmed_l', 'soleus_l'};
assert(isequal(sort(report.compliantMuscles), sort(expected)), 'Wrong compliant muscle set.');

trajectory = MocoTrajectory(trajectoryPath);
study = MocoStudy();
problem = study.updProblem();
problem.setModelAsCopy(model);
time = trajectory.getTimeMat();
problem.setTimeBounds(time(1), time(end));
problem.addGoal(MocoControlGoal('effort'));
rep = problem.createRep();
assert(trajectory.isCompatible(rep), 'Subject19 trajectory is not compatible with the saved model.');
report.states = rep.getNumStates();
report.controls = rep.createControlInfoNames().size();
assert(report.states == 169 && report.controls == 93, 'Unexpected Moco state/control counts.');
decisionNames = stringsFromVector(rep.createControlInfoNames());
assert(~any(contains(decisionNames, 'AFO')), 'AFO controls must be prescribed.');
solver = study.initCasADiSolver();
solver.set_num_mesh_intervals(5);
solver.set_parallel(0);
guess = solver.createGuess();
assert(all(isfinite(guess.getStatesTrajectoryMat()), 'all') && ...
    all(isfinite(guess.getControlsTrajectoryMat()), 'all'), 'Nonfinite Moco guess.');

states = StatesTrajectory.createFromStatesTable(model, trajectory.exportToStatesTable(), false, false);
baseStates = StatesTrajectory.createFromStatesTable(native, trajectory.exportToStatesTable(), false, false);
contactNames = {'Heel', 'LateralRearfoot', 'LateralMidfoot', 'LateralToe', 'MedialToe', 'MedialMidfoot'};
verticalForce = zeros(numel(time), 1);
report.maxGrfDifference = 0;
report.maxDerivativeDifference = 0;
report.ankleMomentArms = zeros(3, 2);
checkRows = unique([1, ceil(numel(time)/2), numel(time)]);
controlNames = stringsFromVector(trajectory.getControlNames());
U = trajectory.getControlsTrajectoryMat();
baseOrder = stringsFromVector(opensimSimulation.createStateVariableNamesInSystemOrder(native));
newOrder = stringsFromVector(opensimSimulation.createStateVariableNamesInSystemOrder(model));
assert(isequal(baseOrder, newOrder), 'AFO changed the system state ordering.');
for i = 0:states.getSize()-1
    s = states.get(i); b = baseStates.get(i);
    model.realizeVelocity(s); native.realizeVelocity(b);
    for sideIndex = 1:2
        sides = {'r', 'l'}; side = sides{sideIndex};
        afo = PathActuator.safeDownCast(model.getComponent(['/AFO_' side]));
        assert(afo.getControl(s) == 0, 'Nonzero initial AFO control.');
        for j = 1:numel(contactNames)
            path = ['/forceset/contact' contactNames{j} '_' side];
            f = SmoothSphereHalfSpaceForce.safeDownCast(model.getComponent(path)).getSphereForce(s);
            bf = SmoothSphereHalfSpaceForce.safeDownCast(native.getComponent(path)).getSphereForce(b);
            verticalForce(i+1) = verticalForce(i+1) + f.get(1).get(1);
            for axis = 0:2
                report.maxGrfDifference = max(report.maxGrfDifference, abs(f.get(1).get(axis) - bf.get(1).get(axis)));
            end
        end
        row = find(checkRows == i+1, 1);
        if ~isempty(row)
            report.ankleMomentArms(row, sideIndex) = afo.computeMomentArm(s, model.getCoordinateSet().get(['ankle_angle_' side]));
        end
    end
    if ismember(i+1, checkRows)
        baseDerivative = evaluateDerivative(native, b, controlNames, U(i+1, :));
        newDerivative = evaluateDerivative(model, s, controlNames, U(i+1, :));
        report.maxDerivativeDifference = max(report.maxDerivativeDifference, max(abs(baseDerivative - newDerivative)));
    end
end
assert(report.maxGrfDifference < 1e-9, 'Zero AFO changed contact forces.');
assert(report.maxDerivativeDifference < 1e-8, 'Zero AFO changed model dynamics.');
assert(all(isfinite(report.ankleMomentArms), 'all') && all(report.ankleMomentArms < -1e-3, 'all'), ...
    'AFO ankle moment arms must be finite and plantarflexing.');
report.meanVerticalForce = trapz(time, verticalForce) / (time(end) - time(1));
verifyController(model, candidatePath);
end


function derivative = evaluateDerivative(model, state, trajectoryNames, values)
import org.opensim.modeling.*
names = stringsFromVector(opensimSimulation.createControlNamesFromModel(model));
u = zeros(numel(names), 1);
for i = 1:numel(names)
    column = find(strcmp(trajectoryNames, names{i}), 1);
    if ~isempty(column)
        u(i) = values(column);
    else
        assert(ismember(names{i}, {'/AFO_r', '/AFO_l'}), 'Unexpected missing trajectory control.');
    end
end
model.setControls(state, Vector.createFromMat(u));
model.realizeAcceleration(state);
derivative = state.getYDot().getAsMat();
assert(all(isfinite(derivative)), 'Nonfinite model derivative.');
end


function verifyController(model, candidatePath)
import org.opensim.modeling.*
assert(model.getControllerSet().getSize() == 1, 'Unexpected controller count.');
controller = PrescribedController.safeDownCast(model.getControllerSet().get('AFO_controller'));
assert(~isempty(controller) && controller.get_enabled(), 'AFO controller is disabled/missing.');
functions = controller.get_ControlFunctions();
assert(functions.getSize() == 2 && strcmp(char(functions.getName()), 'ControlFunctions'), 'Wrong FunctionSet.');
for name = {'AFO_r', 'AFO_l'}
    assert(~isempty(GCVSpline.safeDownCast(functions.get(name{1}))), 'Missing AFO GCVSpline.');
end
doc = xmlread(candidatePath);
nodes = doc.getElementsByTagName('PrescribedController');
assert(nodes.getLength() == 1, 'Wrong controller XML structure.');
assert(strcmp(char(nodes.item(0).getAttribute('name')), 'AFO_controller'), 'Wrong controller XML name.');
% Match moco_WoC_loop: replace the existing functions in the saved XML.
% prescribeControlForActuator appends functions instead of replacing them.
sets = nodes.item(0).getElementsByTagName('FunctionSet');
objects = [];
for i = 0:sets.getLength()-1
    if strcmp(char(sets.item(i).getAttribute('name')), 'ControlFunctions')
        objects = sets.item(i).getElementsByTagName('objects').item(0);
        break
    end
end
assert(~isempty(objects), 'Missing ControlFunctions XML objects node.');
while objects.hasChildNodes(), objects.removeChild(objects.getFirstChild()); end
names = {'AFO_r', 'AFO_l'}; values = [0.3, -0.2]; x = linspace(0, 1, 11);
for i = 1:2
    spline = doc.createElement('GCVSpline'); spline.setAttribute('name', names{i});
    appendElement(doc, spline, 'half_order', '1');
    appendElement(doc, spline, 'error_variance', '0');
    appendElement(doc, spline, 'x', strtrim(sprintf(' %.17g', x)));
    appendElement(doc, spline, 'y', strtrim(sprintf(' %.17g', values(i) * ones(size(x)))));
    appendElement(doc, spline, 'weights', strtrim(sprintf(' %.17g', ones(size(x)))));
    appendElement(doc, spline, 'coefficients', strtrim(sprintf(' %.17g', zeros(size(x)))));
    objects.appendChild(spline);
end
injectedPath = [tempname(fileparts(candidatePath)) '.osim'];
cleanup = onCleanup(@() deleteIfPresent(injectedPath)); %#ok<NASGU>
xmlwrite(injectedPath, doc);
copy = Model(injectedPath);
s = copy.initSystem(); s.setTime(0.37); copy.realizeVelocity(s);
assert(abs(ScalarActuator.safeDownCast(copy.getComponent('/AFO_r')).getControl(s) - 0.3) < 1e-12 && ...
    abs(ScalarActuator.safeDownCast(copy.getComponent('/AFO_l')).getControl(s) + 0.2) < 1e-12, ...
    'AFO controls do not reach the intended actuators.');
end


function appendElement(doc, parent, name, value)
element = doc.createElement(name);
element.appendChild(doc.createTextNode(value));
parent.appendChild(element);
end


function values = vector3(v)
values = [v.get(0), v.get(1), v.get(2)];
end


function values = stringsFromVector(v)
values = cell(v.size(), 1);
for i = 0:v.size()-1, values{i+1} = char(v.get(i)); end
end


function hash = fileHash(path)
fid = fopen(path, 'rb');
assert(fid >= 0, 'Cannot read input file: %s', path);
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
bytes = fread(fid, Inf, '*uint8');
md = java.security.MessageDigest.getInstance('SHA-256');
md.update(typecast(bytes, 'int8'));
hash = lower(reshape(dec2hex(typecast(md.digest(), 'uint8'), 2)', 1, []));
end


function deleteIfPresent(path)
if isfile(path), delete(path); end
end
