import opensim as osim
import numpy as np
import math
import csv

# ---------- User settings ----------
model_file = 'models/prosthesisModel_9.osim'        # your modified Rajagopal + prosthesis
coords_file = 'sto/coords_modified.sto'         # reconstructed coordinates (states reference)
external_loads = None                # set to '' if you don't have it
desired_speed = 1.2                            # adjust to match your data if known
n_mesh = 75
# -----------------------------------

# List of removed coordinates (patella angles)
REMOVED_COORDS = ['knee_angle_r_beta', 'knee_angle_l_beta']

# List of removed muscles (quadriceps that attached to patella)
REMOVED_MUSCLES = ['recfem_r', 'vasint_r', 'vaslat_r', 'vasmed_r',
                   'recfem_l', 'vasint_l', 'vaslat_l', 'vasmed_l']

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

    # Define Contact Force Parameters
    stiffness = 4e5
    dissipation = 2.0
    static_friction = 0.8
    dynamic_friction = 0.4
    transition_velocity = 0.2

    # Create the SmoothSphereHalfSpaceForce
    sshs_force = osim.SmoothSphereHalfSpaceForce()
    sshs_force.setName(f'{foot_body_name}_GroundForce')
    sshs_force.connectSocket_sphere(foot_contact_sphere)
    sshs_force.connectSocket_half_space(ground_contact_space)
    sshs_force.set_stiffness(stiffness)
    sshs_force.set_dissipation(dissipation)
    sshs_force.set_static_friction(static_friction)
    sshs_force.set_dynamic_friction(dynamic_friction)
    sshs_force.set_transition_velocity(transition_velocity)

    model.get_ComponentSet().addComponent(sshs_force)
    model.finalizeConnections()

    print(f'Added ground contact for {foot_body_name} with stiffness {stiffness}')

# setup mocotrack
track = osim.MocoTrack()
track.setName('tracking')

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

# ------------------------- METABOLICS ---------------------------------
# Add metabolic cost model - SKIP REMOVED MUSCLES
metabolics = osim.Bhargava2004SmoothedMuscleMetabolics()
metabolics.setName('metabolic_cost')
metabolics.set_use_smoothing(True)

muscles = model.updMuscles()
muscles_added = 0
muscles_skipped = 0

for imuscle in range(muscles.getSize()):
    muscle = osim.Muscle.safeDownCast(muscles.get(imuscle))
    muscle_name = muscle.getName()
    
    # Skip removed muscles
    if muscle_name in REMOVED_MUSCLES:
        muscles_skipped += 1
        print(f'Skipping removed muscle: {muscle_name}')
        continue
    
    metabolics.addMuscle(muscle_name, muscle)
    muscles_added += 1
    
print(f'Added {muscles_added} muscles to metabolics (skipped {muscles_skipped})')
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

# Metabolic cost
metGoal = osim.MocoOutputGoal('met',0.1)
problem.addGoal(metGoal)
metGoal.setOutputPath('/metabolic_cost|total_metabolic_rate')
metGoal.setDivideByDisplacement(True)
metGoal.setDivideByMass(True)

# -------------------------- CLEANUP BOUNDS --------------------------------
# Get processed model for coordinate checking
processed_model = mp.process()
processed_model.initSystem()

coordinatesUpdated = tableProcessor.process(processed_model)
labels = coordinatesUpdated.getColumnLabels()
index = coordinatesUpdated.getNearestRowIndexForTime(0.48)

for label in labels:
    # Extract coordinate name from state path
    coord_name = label.split('/')[-2]  # second-to-last element
    
    # Skip removed coordinates
    if coord_name in REMOVED_COORDS:
        print(f'Skipping removed coordinate: {coord_name}')
        continue
    
    # Check if coordinate exists in model
    try:
        state_info = processed_model.getCoordinateSet().get(coord_name)
    except:
        print(f'Warning: Coordinate {coord_name} not found in model, skipping')
        continue
    
    value = coordinatesUpdated.getDependentColumn(label).to_numpy()
    value = [np.pi * (v / 180.0) for v in value]  # deg to rad

    # get the initial value from the reference
    x0 = value[index]

    # Get the model-defined joint limits
    model_lb = state_info.getRangeMin()
    model_ub = state_info.getRangeMax()

    # tight bounds for pelvis coords causes bound violations
    if 'pelvis' in label:
        lower = -0.5
        upper = 0.5
    # set bounds based on variable (speed or position) and the initial value
    elif '/speed' in label:
        lower = x0 - 0.1
        upper = x0 + 0.1
    else:
        lower = x0 - 0.05
        upper = x0 + 0.05

    # Clip to trajectory min/max
    lower = max(np.min(value), lower)
    upper = min(np.max(value), upper)

    # Clip to model limits
    lower = max(model_lb, lower)
    upper = min(model_ub, upper)

    if lower > upper:
        # fallback to x0  small epsilon
        lower = x0 - 1e-6
        upper = x0 + 1e-6
    
    problem.setStateInfo(label, [], [lower, upper])

# pelvis_ty bounds
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

# ------------------------------ SOLVER -------------------------------------
solver = osim.MocoCasADiSolver.safeDownCast(study.updSolver())
solver.resetProblem(problem)
solver.set_num_mesh_intervals(50)
solver.set_verbosity(2)
solver.set_optim_solver('ipopt')
solver.set_optim_convergence_tolerance(1e-4)
solver.set_optim_constraint_tolerance(1e-4)
solver.set_optim_max_iterations(500)
solver.set_transcription_scheme('legendre-gauss-radau-3')
solver.set_kinematic_constraint_method('Bordalba2023')
solver.set_optim_convergence_tolerance(1e-2)
solver.set_optim_constraint_tolerance(1e-4)
solver.resetProblem(problem)

# ------------------------------ SOLVE --------------------------------------
print('Solving muscle-driven STATE TRACKING ...')
solution = study.solve()

try:
    fullStride = osim.createPeriodicTrajectory(solution)
    fullStride.write('output/gait_cycle.sto')

    print('   ')
    print(f"The metabolic cost of transport is: {10*solution.getObjectiveTerm('met'):.3f} J/kg/m")
    print('   ')
except:
    if not solution.success():
        print('Solver failed. Unsealing solution for debugging.')
        solution.unseal()
        solution.write('failed_solution.sto')
