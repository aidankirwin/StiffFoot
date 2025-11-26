import opensim as osim
import numpy as np
import math
import csv

# ---------- User settings ----------
model_file = "models/prosthesisModel_mocoReady.osim"        # your modified Rajagopal + prosthesis
coords_file = "sto/coords_modified.sto"         # reconstructed coordinates (states reference)
external_loads = None                # set to "" if you don't have it
desired_speed = 1.2                            # adjust to match your data if known
n_mesh = 75
# -----------------------------------

# Don't want to set the pelvis dof bounds too tight
# pelvis_dofs = [
#     '/ground_pelvis/pelvis_tilt/value',
#     '/ground_pelvis/pelvis_list/value',
#     '/ground_pelvis/pelvis_rotation/value',
#     '/ground_pelvis/pelvis_tx/value',
#     '/ground_pelvis/pelvis_ty/value',
#     '/ground_pelvis/pelvis_tz/value'
# ]

# setup mocotrack
track = osim.MocoTrack()
track.setName("tracking")

# load model
model = osim.Model(model_file)

# add metabolic cost model
metabolics = osim.Bhargava2004SmoothedMuscleMetabolics()
metabolics.setName('metabolic_cost')
metabolics.set_use_smoothing(True)

# Set minimum muscle controls and activations to 0 (default is 0.01).
muscles = model.updMuscles()
for imuscle in range(muscles.getSize()):
    muscle = osim.Muscle.safeDownCast(muscles.get(imuscle))
    metabolics.addMuscle(muscle.getName(), muscle)
    
model.addComponent(metabolics)
model.finalizeConnections()

# ------------------------- MODEL PROCESSOR ---------------------------------
mp = osim.ModelProcessor(model)
# mp.append(osim.ModOpIgnoreTendonCompliance())
# mp.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
# mp.append(osim.ModOpIgnorePassiveFiberForcesDGF())
# mp.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))
# Add external loads file (if using GRFs)
if external_loads:
    mp.append(osim.ModOpAddExternalLoads(external_loads))

track.setModel(mp)

# ---------------------- STATE TRACKING GOAL --------------------------------

# Reference table
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

# Instead of calling solve(), call initialize() to receive a pre-configured
# MocoStudy object based on the settings above. Use this to customize the
# problem beyond the MocoTrack interface.
study = track.initialize()

# Get a reference to the MocoControlCost that is added to every MocoTrack
# problem by default and set the overall weight to 0.1.
problem = study.updProblem()

# Constrain the states and controls to be periodic.
periodicityGoal = osim.MocoPeriodicityGoal("periodicity")

model = mp.process()
model.initSystem()

# Add all periodicity pairs
for i in range(model.getNumStateVariables()):
    currentStateName = str(model.getStateVariableNames().getitem(i))
    if 'pelvis_tx/value' not in currentStateName:
        periodicityGoal.addStatePair(osim.MocoPeriodicityGoalPair(currentStateName))
    
forceSet = model.getForceSet()
for i in range(forceSet.getSize()):
    force = forceSet.get(i)
    if "Actuator" in force.getConcreteClassName():  # Only actuators with controls
        forcePath = forceSet.get(i).getAbsolutePathString()
        periodicityGoal.addControlPair(osim.MocoPeriodicityGoalPair(forcePath))

problem.addGoal(periodicityGoal)

effort = osim.MocoControlGoal.safeDownCast(problem.updGoal("control_effort"))
effort.setWeight(0.1)

# Put larger individual weights on the pelvis CoordinateActuators, which act 
# as the residual, or 'hand-of-god', forces which we would like to keep as small
# as possible.
effort.setWeightForControlPattern('.*pelvis.*', 10)

# Metabolic cost; total metabolic rate includes activation heat rate,
# maintenance heat rate, shortening heat rate, mechanical work rate, and
# basal metabolic rate.
metGoal = osim.MocoOutputGoal('met',0.1)
problem.addGoal(metGoal)
metGoal.setOutputPath('/metabolic_cost|total_metabolic_rate')
metGoal.setDivideByDisplacement(True)
metGoal.setDivideByMass(True)

# -------------------------- CLEANUP BOUNDS --------------------------------
# Constrain initial states to be close to the reference
coordinatesUpdated = tableProcessor.process(model)
labels = coordinatesUpdated.getColumnLabels()
index = coordinatesUpdated.getNearestRowIndexForTime(0.48)

for label in labels:
    value = coordinatesUpdated.getDependentColumn(label).to_numpy()
    value = [np.pi * (v / 180.0) for v in value]  # deg to rad

    # get the initial value from the reference
    x0 = value[index]

    # Get the model-defined joint limits
    coord_name = label.split('/')[-2]  # second-to-last element
    state_info = model.getCoordinateSet().get(coord_name)

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
        # fallback to x0 ± small epsilon
        lower = x0 - 1e-6
        upper = x0 + 1e-6
    
    problem.setStateInfo(label, [], [lower, upper])


# ------------------ CONTROL REGULARIZATION (stabilizes solution) ----------
control_reg = osim.MocoControlGoal("control_reg", 1e-2)
control_reg.setDivideByDisplacement(False)
problem.addGoal(control_reg)

# ------------------------------ SOLVER -------------------------------------
solver = osim.MocoCasADiSolver.safeDownCast(study.updSolver())
solver.resetProblem(problem)
solver.set_num_mesh_intervals(50)
solver.set_verbosity(2)
solver.set_verbosity(2)
solver.set_optim_solver('ipopt')
solver.set_optim_convergence_tolerance(1e-4)
solver.set_optim_constraint_tolerance(1e-4)
solver.set_optim_max_iterations(5)
# Use the Legendre-Gauss-Radau transcription scheme, a psuedospectral 
# scheme with high integration accuracy.
solver.set_transcription_scheme('legendre-gauss-radau-3')
# Use the Bordalba et al. (2023) kinematic constraint method.
solver.set_kinematic_constraint_method('Bordalba2023')
# Set the solver's convergence and constraint tolerances.
solver.set_optim_convergence_tolerance(1e-2)
solver.set_optim_constraint_tolerance(1e-4)
# We've updated the MocoProblem, so call resetProblem() to pass the updated
# problem to the solver.
solver.resetProblem(problem)
solver.setGuess("bounds")  # midpoints of bounds

# ------------------------------ SOLVE --------------------------------------
print("Solving muscle-driven STATE TRACKING ...")
solution = study.solve()

try:
    # Create a full stride from the periodic single step solution.
    # For details, view the Doxygen documentation for createPeriodicTrajectory().
    fullStride = osim.createPeriodicTrajectory(solution)
    fullStride.write('output/gait_cycle.sto')

    print('   ')
    print(f"The metabolic cost of transport is: {10*solution.getObjectiveTerm('met'):.3f} J/kg/m")
    print('   ')
except:
    if not solution.success():
        print("Solver failed. Unsealing solution for debugging.")
        solution.unseal() # Unseal the solution
        solution.write("failed_solution.sto") # Save the trajectory data to a file