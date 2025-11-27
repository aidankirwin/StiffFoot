import opensim as osim
import numpy as np
import math
import csv

# ---------- User settings ----------
model_file = 'models/prosthesisModel_2.osim'        # your modified Rajagopal + prosthesis
coords_file = 'sto/coords_modified.sto'         # reconstructed coordinates (states reference)
external_loads = None                # set to '' if you don't have it
desired_speed = 1.2                            # adjust to match your data if known
n_mesh = 75
# -----------------------------------

# List of removed coordinates (patella angles)
REMOVED_COORDS = ['knee_angle_r_beta', 'knee_angle_l_beta']

def add_foot_ground_contact(model, ground_contact_space, foot_body_name, contact_sphere_radius, sphere_location_in_foot):
    foot_body = model.getBodySet().get(foot_body_name)

    # Define the ContactSphere on the foot
    foot_contact_sphere = osim.ContactSphere(
        contact_sphere_radius,
        sphere_location_in_foot,
        foot_body
    )
    foot_contact_sphere.setName(f'{foot_body_name}_ContactSphere')
    model.addContactGeometry(foot_contact_sphere)

    # Create the SmoothSphereHalfSpaceForce
    sshs_force = osim.SmoothSphereHalfSpaceForce()
    sshs_force.setName(f'{foot_body_name}_GroundForce')
    sshs_force.connectSocket_sphere(foot_contact_sphere)
    sshs_force.connectSocket_half_space(ground_contact_space)

    model.get_ComponentSet().addComponent(sshs_force)
    model.finalizeConnections()

    print(f'Added ground contact for {foot_body_name}')

# setup mocotrack
track = osim.MocoTrack()
track.setName('tracking_visualize_contact')

# load model
model = osim.Model(model_file)

# ------------------------- FOOT GROUND CONTACT ---------------------------------
foot_radius = 0.03
ground = model.getGround()

ground_contact_space = osim.ContactHalfSpace(
    osim.Vec3(0, 0, 0),
    osim.Vec3(0, 0, 0),
    ground
)
ground_contact_space.setName('GroundContactSpace')
model.addContactGeometry(ground_contact_space)

add_foot_ground_contact(model, ground_contact_space, 'calcn_l', foot_radius, osim.Vec3(0, -0.01, 0))
add_foot_ground_contact(model, ground_contact_space, 'segment_12', foot_radius, osim.Vec3(0, -0.01, 0))

# Add visualization geometry for the ground plane
ground_plane = osim.Brick(osim.Vec3(5, 0.01, 5))  # large flat plane
ground_plane.setColor(osim.Vec3(0.8, 0.8, 0.8))  # light gray
ground_plane.setOpacity(0.5)
ground.attachGeometry(ground_plane)

print('Added ground plane visualization')

# ------------------------- METABOLICS ---------------------------------
metabolics = osim.Bhargava2004SmoothedMuscleMetabolics()
metabolics.setName('metabolic_cost')
metabolics.set_use_smoothing(True)

muscles = model.updMuscles()

for imuscle in range(muscles.getSize()):
    muscle = osim.Muscle.safeDownCast(muscles.get(imuscle))
    muscle_name = muscle.getName()
    
    metabolics.addMuscle(muscle_name, muscle)

model.addComponent(metabolics)
model.finalizeConnections()

# ------------------------- MODEL PROCESSOR ---------------------------------
mp = osim.ModelProcessor(model)

if external_loads:
    mp.append(osim.ModOpAddExternalLoads(external_loads))

track.setModel(mp)

# ---------------------- STATE TRACKING GOAL --------------------------------
tableProcessor = osim.TableProcessor(coords_file)
tableProcessor.append(osim.TabOpUseAbsoluteStateNames())
tableProcessor.append(osim.TabOpAppendCoupledCoordinateValues())
tableProcessor.append(osim.TabOpAppendCoordinateValueDerivativesAsSpeeds())
track.setStatesReference(tableProcessor)

track.set_states_global_tracking_weight(30)
track.set_allow_unused_references(True)
track.set_track_reference_position_derivatives(True)
track.set_apply_tracked_states_to_guess(True)
track.set_initial_time(0.48)
track.set_final_time(1.61)
track.set_mesh_interval(0.02)

study = track.initialize()
problem = study.updProblem()

# -------------------------- PERIODICITY GOAL --------------------------------
processed_model = mp.process()
processed_model.initSystem()

periodicityGoal = osim.MocoPeriodicityGoal('periodicity')
coordinates = processed_model.getCoordinateSet()
for icoord in range(coordinates.getSize()):
    coordinate = coordinates.get(icoord)
    coordName = coordinate.getName()

    if 'beta' in coordName: continue 

    if not '_tx' in coordName:
        valueName = coordinate.getStateVariableNames().get(0)
        periodicityGoal.addStatePair(
                osim.MocoPeriodicityGoalPair(valueName))
    speedName = coordinate.getStateVariableNames().get(1)
    periodicityGoal.addStatePair(osim.MocoPeriodicityGoalPair(speedName))

muscles = processed_model.getMuscles()
for imusc in range(muscles.getSize()):
    muscle = muscles.get(imusc)
    stateName = muscle.getStateVariableNames().get(0)
    periodicityGoal.addStatePair(osim.MocoPeriodicityGoalPair(stateName))
    controlName = muscle.getAbsolutePathString()
    periodicityGoal.addControlPair(
            osim.MocoPeriodicityGoalPair(controlName))

actuators = processed_model.getActuators()
for iactu in range(actuators.getSize()):
    actu = osim.CoordinateActuator.safeDownCast(actuators.get(iactu))
    if actu is not None: 
        controlName = actu.getAbsolutePathString()
        periodicityGoal.addControlPair(
                osim.MocoPeriodicityGoalPair(controlName))

problem.addGoal(periodicityGoal)

effort = osim.MocoControlGoal.safeDownCast(problem.updGoal("control_effort"))
effort.setWeight(0.1)
effort.setWeightForControlPattern('.*pelvis.*', 10)

# -------------------------- METABOLIC COST GOAL --------------------------------
metGoal = osim.MocoOutputGoal('met',0.1)
problem.addGoal(metGoal)
metGoal.setOutputPath('/metabolic_cost|total_metabolic_rate')
metGoal.setDivideByDisplacement(True)
metGoal.setDivideByMass(True)

# -------------------------- CLEANUP BOUNDS --------------------------------
coordinatesUpdated = tableProcessor.process(processed_model)
labels = coordinatesUpdated.getColumnLabels()
index = coordinatesUpdated.getNearestRowIndexForTime(0.48)

for label in labels:
    coord_name = label.split('/')[-2]
    
    if coord_name in REMOVED_COORDS:
        print(f'Skipping removed coordinate: {coord_name}')
        continue
    
    try:
        state_info = processed_model.getCoordinateSet().get(coord_name)
    except:
        print(f'Warning: Coordinate {coord_name} not found in model, skipping')
        continue
    
    value = coordinatesUpdated.getDependentColumn(label).to_numpy()
    value = [np.pi * (v / 180.0) for v in value]

    x0 = value[index]
    model_lb = state_info.getRangeMin()
    model_ub = state_info.getRangeMax()

    if 'pelvis' in label:
        lower = -0.5
        upper = 0.5
    elif '/speed' in label:
        lower = x0 - 0.1
        upper = x0 + 0.1
    else:
        lower = x0 - 0.05
        upper = x0 + 0.05

    lower = max(np.min(value), lower)
    upper = min(np.max(value), upper)
    lower = max(model_lb, lower)
    upper = min(model_ub, upper)

    if lower > upper:
        lower = x0 - 1e-6
        upper = x0 + 1e-6
    
    problem.setStateInfo(label, [], [lower, upper])

problem.setStateInfo(
    '/jointset/ground_pelvis/pelvis_ty/value',
    [], 
    [0.85, 1.1],
    1.0
)

problem.setStateInfo(
    '/jointset/ground_pelvis/pelvis_ty/speed',
    [], 
    [-0.5, 0.5],
    0.0
)

# ------------------ CONTROL REGULARIZATION ----------
control_reg = osim.MocoControlGoal("control_reg", 1e-2)
control_reg.setDivideByDisplacement(False)
problem.addGoal(control_reg)

# ------------------------------ SOLVER -------------------------------------
solver = osim.MocoCasADiSolver.safeDownCast(study.updSolver())
solver.resetProblem(problem)
solver.set_num_mesh_intervals(40)
solver.set_verbosity(2)
solver.set_optim_solver('ipopt')
solver.set_optim_convergence_tolerance(1e-4)
solver.set_optim_constraint_tolerance(1e-4)
solver.set_optim_max_iterations(5)  # Set to 5 iterations
solver.set_transcription_scheme('legendre-gauss-radau-3')
solver.set_kinematic_constraint_method('Bordalba2023')
solver.set_optim_convergence_tolerance(1e-2)
solver.set_optim_constraint_tolerance(1e-4)
solver.resetProblem(problem)

# ------------------------------ SOLVE --------------------------------------
print('Solving muscle-driven STATE TRACKING with contact visualization...')
solution = study.solve()

try:
    fullStride = osim.createPeriodicTrajectory(solution)
    fullStride.write('output/gait_cycle_with_contact.sto')

    print('   ')
    print(f"The metabolic cost of transport is: {10*solution.getObjectiveTerm('met'):.3f} J/kg/m")
    print('   ')
    
    # Visualize the solution with the contact spheres and ground plane
    print('Opening visualizer...')
    study.visualize(solution)
    
except:
    if not solution.success():
        print('Solver failed. Unsealing solution for debugging.')
        solution.unseal()
        solution.write('failed_solution_with_contact.sto')