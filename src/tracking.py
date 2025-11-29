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
    model_file = 'models/prosthesisModel_5.osim'        # the modified Rajagopal + prosthesis
    coords_file = 'sto/coords_modified_new.sto'         # reconstructed coordinates (states reference)
    # -----------------------------------

    def add_foot_ground_contact(model, ground_contact_space):
        """
        Adds four contact spheres:
        1. Heel of left foot (calcn_l)
        2. Toe left (toes_l)
        3. Toe right (segment_14)
        4. Heel of right foot (segment_20)
        """
        # Foot contact sphere parameters, somewhat arbitrary
        z_offset    = -0.01  # meters that the contact sphere will be placed below the foot
        heel_radius = 0.025
        mid_radius  = 0.02

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

            # Set force parameters
            force.set_stiffness(1e6) # 1 MPa, per https://doi.org/10.1371/journal.pcbi.1012219
            force.set_dissipation(1.0)

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
        osim.Vec3(0, 0.05, 0),
        osim.Vec3(0, 0, 90*np.pi/180),  # plane is horizontal, so the normal force points up
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

    t0 = 0.45
    tf = 1.51
    track.set_initial_time(t0)
    track.set_final_time(tf)

    track.set_mesh_interval(0.02)

    study = track.initialize()
    problem = study.updProblem()

    # -------------------------- PERIODICITY GOAL --------------------------------

    # Get processed model for coordinate checking
    processed_model = mp.process()
    processed_model.initSystem()

    # Constrain the states and controls to be periodic.
    # periodicityGoal = osim.MocoPeriodicityGoal('periodicity')
    # coordinates = processed_model.getCoordinateSet()

    # # Filters: skip translations and internal prosthesis/segment coords that
    # # often create infeasible equality constraints.
    # skip_substrings = ['_tx', '_ty', '_tz', 'pelvis_tx', 'pelvis_ty', 'pelvis_tz']
    # skip_prefixes = ['joint_segment_', 'segment_']

    # for icoord in range(coordinates.getSize()):
    #     coordinate = coordinates.get(icoord)
    #     coordName = coordinate.getName()

    #     # Exclude the knee_angle_l/r_beta coordinates from the periodicity
    #     # constraint because they are coupled to the knee_angle_l/r
    #     # coordinates.
    #     if 'beta' in coordName: continue 

    #     # Skip translations and prosthesis/segment internals
    #     if any(s in coordName for s in skip_substrings) or any(coordName.startswith(p) for p in skip_prefixes):
    #         print(f"Skipping (filtered) periodicity for: {coordName}")
    #         continue

    #     valueName = coordinate.getStateVariableNames().get(0)
    #     periodicityGoal.addStatePair(
    #             osim.MocoPeriodicityGoalPair(valueName))
    #     speedName = coordinate.getStateVariableNames().get(1)
    #     periodicityGoal.addStatePair(osim.MocoPeriodicityGoalPair(speedName))
    #     print(f"Added periodicity for coordinate: {coordName}")

    # muscles = processed_model.getMuscles()
    # for imusc in range(muscles.getSize()):
    #     muscle = muscles.get(imusc)
    #     stateName = muscle.getStateVariableNames().get(0)
    #     periodicityGoal.addStatePair(osim.MocoPeriodicityGoalPair(stateName))
    #     controlName = muscle.getAbsolutePathString()
    #     periodicityGoal.addControlPair(
    #             osim.MocoPeriodicityGoalPair(controlName))
    #     print(f"Added periodicity for muscle: {muscle.getName()}")

    # actuators = processed_model.getActuators()
    # for iactu in range(actuators.getSize()):
    #     actu = osim.CoordinateActuator.safeDownCast(actuators.get(iactu))
    #     if actu is not None: 
    #         controlName = actu.getAbsolutePathString()
    #         periodicityGoal.addControlPair(
    #                 osim.MocoPeriodicityGoalPair(controlName))
    #         print(f"Added periodicity for actuator: {actu.getName()}")

    # problem.addGoal(periodicityGoal)


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
    index_0 = coordinatesUpdated.getNearestRowIndexForTime(t0)
    index_f = coordinatesUpdated.getNearestRowIndexForTime(tf)

    for label in labels:
        value = coordinatesUpdated.getDependentColumn(label).to_numpy()

        if 'segment_' in label:
            problem.setStateInfo(label, [-2, 2], [-1, 1], [-1, 1])
            continue

        # get the initial value from the reference to set initial and final bounds
        x0 = value[index_0]
        # get the final value from the reference to set initial and final bounds
        xf = value[index_f]

        # get the min and max of the entire trajectory to set bounds for the full phase
        xmin = np.min(value)
        xmax = np.max(value)

        # set bounds based on variable (speed or position) and the initial value
        if '/speed' in label:
            margin = 0.3
        else:
            margin = 0.2

        # initial and final bounds
        lower_init = x0 - margin
        upper_init = x0 + margin
        lower_final = xf - margin
        upper_final = xf + margin
        # full phase bounds
        lower = xmin - margin
        upper = xmax + margin
        
        # set state info parameters: name, full phase bounds, initial bounds, final bounds
        problem.setStateInfo(label, [lower, upper], [lower_init, upper_init], [lower_final, upper_final])

    # ------------------------------ SOLVER -------------------------------------
    solver = osim.MocoCasADiSolver.safeDownCast(study.updSolver())

    solver.set_num_mesh_intervals(200)
    solver.set_optim_max_iterations(1000)  # increase slowly

    solver.set_verbosity(2)
    solver.set_optim_solver('ipopt')
    solver.set_optim_convergence_tolerance(1e-1)
    solver.set_optim_constraint_tolerance(1e-1)

    solver.set_transcription_scheme('hermite-simpson') # was legendre-gauss-radau-3
    solver.set_kinematic_constraint_method('Bordalba2023')
    solver.set_optim_hessian_approximation('limited-memory')
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
