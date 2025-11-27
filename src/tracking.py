import opensim as osim
import numpy as np
import math
import csv

def solve_metabolic_tracking(model=None):
    """
    Set up and solve a muscle-driven MocoTrack problem to track
    experimental gait data using a modified prosthesis model with
    foot-ground contact and metabolic cost.
    """

    # ---------- User settings ----------
    model_file = 'models/prosthesisModel_3.osim'        # your modified Rajagopal + prosthesis
    coords_file = 'sto/coords_modified.sto'         # reconstructed coordinates (states reference)
    # -----------------------------------

    # List of removed coordinates (patella angles)
    # REMOVED_COORDS = ['knee_angle_r_beta', 'knee_angle_l_beta']
    REMOVED_COORDS = []     # for experimenting with no patella

    def add_foot_ground_contact(model, ground_contact_space):
        """
        Adds four contact spheres:
        1. Heel of left foot (calcn_l)
        2. Mid-foot left (calcn_l)
        3. Mid-foot right (segment_12)
        4. Heel of right foot (segment_20)
        """
        # Parameters
        z_offset = -0.01  # meters below foot origin
        heel_radius = 0.025
        mid_radius = 0.02

        # Helper function to add a sphere
        def add_sphere(body_name, radius, pos):
            foot_body = model.getBodySet().get(body_name)
            sphere = osim.ContactSphere(radius, pos, foot_body)
            sphere.setName(f"{body_name}_ContactSphere")
            model.addContactGeometry(sphere)

            # Create contact force
            force = osim.SmoothSphereHalfSpaceForce()
            force.setName(f"{body_name}_GroundForce")
            force.connectSocket_sphere(sphere)
            force.connectSocket_half_space(ground_contact_space)

            # Soft contact parameters
            force.set_stiffness(5000)
            force.set_dissipation(1.0)
            force.set_static_friction(0.6)
            force.set_dynamic_friction(0.5)
            force.set_transition_velocity(0.2)

            # Add to ForceSet so Moco can see it (even if outputs aren't exposed yet)
            model.updForceSet().adoptAndAppend(force)

        # Left foot
        add_sphere('calcn_l', heel_radius, osim.Vec3(0.0, 0.0, z_offset))  # Heel
        add_sphere('toes_l', mid_radius, osim.Vec3(0.0, 0.0, z_offset))     # Toes

        # Right foot
        add_sphere('segment_14', mid_radius, osim.Vec3(0.0, 0.0, z_offset))   # Toes
        add_sphere('segment_20', heel_radius, osim.Vec3(0.0, 0.0, z_offset)) # Heel

        model.finalizeConnections()
        print("Added 4 contact spheres with SoftSphereHalfSpaceForce.")

    # setup mocotrack
    track = osim.MocoTrack()
    track.setName('tracking')

    if model is None:
        # load model
        model = osim.Model(model_file)

    # ------------------------- FOOT GROUND CONTACT ---------------------------------
    ground = model.getGround()

    ground_contact_space = osim.ContactHalfSpace(
        osim.Vec3(0, -0.01, 0),
        osim.Vec3(0, 0, 90*np.pi/180),
        ground
    )
    ground_contact_space.setName('GroundContactSpace')
    model.addContactGeometry(ground_contact_space)

    add_foot_ground_contact(model, ground_contact_space)

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

    # Get processed model for coordinate checking
    processed_model = mp.process()
    processed_model.initSystem()

    # Constrain the states and controls to be periodic.
    periodicityGoal = osim.MocoPeriodicityGoal('periodicity')
    coordinates = processed_model.getCoordinateSet()
    for icoord in range(coordinates.getSize()):
        coordinate = coordinates.get(icoord)
        coordName = coordinate.getName()

        # Exclude the knee_angle_l/r_beta coordinates from the periodicity
        # constraint because they are coupled to the knee_angle_l/r
        # coordinates.
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


    # -------------------------- METABOLIC COST GOAL --------------------------------
    # Metabolic cost; total metabolic rate includes activation heat rate,
    # maintenance heat rate, shortening heat rate, mechanical work rate, and
    # basal metabolic rate.
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
        [0.9, 1.05],
        0.975         # initial guess around midpoint
    )

    problem.setStateInfo(
        '/jointset/ground_pelvis/pelvis_ty/speed',
        [], 
        [-0.5, 0.5],
        0.0
    )

    # ------------------ CONTROL REGULARIZATION (stabilizes solution) ----------
    control_reg = osim.MocoControlGoal("control_reg", 1e-2)
    control_reg.setDivideByDisplacement(False)
    problem.addGoal(control_reg)


    # ------------------------------ SOLVER -------------------------------------
    solver = osim.MocoCasADiSolver.safeDownCast(study.updSolver())
    solver.resetProblem(problem)

    solver.set_num_mesh_intervals(75)  # matches your user setting
    solver.set_optim_max_iterations(1000)  # increase slowly

    solver.set_verbosity(2)
    solver.set_optim_solver('ipopt')
    solver.set_optim_convergence_tolerance(1e-2)
    solver.set_optim_constraint_tolerance(1e-2)

    solver.set_transcription_scheme('legendre-gauss-radau-3')
    solver.set_kinematic_constraint_method('Bordalba2023')
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

if __name__ == "__main__":
    solve_metabolic_tracking()
