import opensim as osim
import numpy as np
import math
import csv

# ---------- User settings ----------
model_file = "models/prosthesisModel_6.osim"        # your modified Rajagopal + prosthesis
coords_file = "sto/coords_modified.sto"         # reconstructed coordinates (states reference)
external_loads = None                # set to "" if you don't have it
output_warmstart = "sto/warmstart.sto"
desired_speed = 1.2                            # adjust to match your data if known
n_mesh = 75
# -----------------------------------

model = osim.Model(model_file)
model.initSystem()

# Set minimum muscle controls and activations to 0 (default is 0.01).
muscles = model.updMuscles()
for imuscle in range(muscles.getSize()):
    muscle = osim.Millard2012EquilibriumMuscle.safeDownCast(muscles.get(imuscle))
    muscle.setMinimumActivation(0.0)
    muscle.setMinControl(0.0)

model.finalizeConnections()

study = osim.MocoStudy()
study.setName("muscle_state_tracking")
problem = study.updProblem()

# ------------------------- MODEL PROCESSOR ---------------------------------
mp = osim.ModelProcessor(model)
mp.append(osim.ModOpIgnoreTendonCompliance())
mp.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
mp.append(osim.ModOpIgnorePassiveFiberForcesDGF())
mp.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))
# Add external loads file (if using GRFs)
if external_loads:
    mp.append(osim.ModOpAddExternalLoads(external_loads))

# ---------------------- STATE TRACKING GOAL --------------------------------
track = osim.MocoTrack()
track.setName("tracking")

track.setModel(mp)

# Reference table
tableProcessor = osim.TableProcessor(coords_file)
tableProcessor.append(osim.TabOpUseAbsoluteStateNames())
tableProcessor.append(osim.TabOpAppendCoupledCoordinateValues())
tableProcessor.append(osim.TabOpAppendCoordinateValueDerivativesAsSpeeds())
track.setStatesReference(tableProcessor)
track.set_allow_unused_references(True)
track.set_track_reference_position_derivatives(True)
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
effort = osim.MocoControlGoal.safeDownCast(problem.updGoal("control_effort"))
effort.setWeight(0.1)

# Put larger individual weights on the pelvis CoordinateActuators, which act 
# as the residual, or 'hand-of-god', forces which we would like to keep as small
# as possible.
effort.setWeightForControlPattern('.*pelvis.*', 10)

# Constrain the states and controls to be periodic.
periodicityGoal = osim.MocoPeriodicityGoal("periodicity")
model = mp.process()
model.initSystem()
for i in range(model.getNumStateVariables()):
    currentStateName = str(model.getStateVariableNames().getitem(i))
    if 'pelvis_tx/value' not in currentStateName:
        periodicityGoal.addStatePair(osim.MocoPeriodicityGoalPair(currentStateName))
    
forceSet = model.getForceSet()
for i in range(forceSet.getSize()):
    force = forceSet.get(i)
    if "Actuator" in force.getConcreteClassName():
        forcePath = forceSet.get(i).getAbsolutePathString()
        periodicityGoal.addControlPair(osim.MocoPeriodicityGoalPair(forcePath))

problem.addGoal(periodicityGoal)

# -------------------------- CLEANUP BOUNDS --------------------------------
# Constrain initial states to be close to the reference
coordinatesUpdated = tableProcessor.process(model)
labels = coordinatesUpdated.getColumnLabels()
index = coordinatesUpdated.getNearestRowIndexForTime(0.48)

coordinates = model.getCoordinateSet()
for icoord in range(coordinates.getSize()):
    coord = coordinates.get(icoord)
    stateNames = coord.getStateVariableNames()  # position and speed

    # Find the matching reference column, if it exists
    coord_label = coord.getName()
    if coord_label in labels:
        print(coord_label)
        value = coordinatesUpdated.getDependentColumn(coord_label).to_numpy()
        center_value = value[index]

        # Set bounds for position
        pos_lower = center_value - 0.05
        pos_upper = center_value + 0.05
        problem.setStateInfo(stateNames.get(0), [], [pos_lower, pos_upper])

        # Optionally, set bounds for speed
        # speed_lower = center_value - 0.1
        # speed_upper = center_value + 0.1
        # problem.setStateInfo(stateNames.get(1), [], [speed_lower, speed_upper])

    else:
        print(f"Skipping nonexistent reference for {coord_label}")


# ------------------ CONTROL REGULARIZATION (stabilizes solution) ----------
control_reg = osim.MocoControlGoal("control_reg", 1e-2)
control_reg.setDivideByDisplacement(False)
problem.addGoal(control_reg)

# ------------------------- SPEED GOAL (optional) ---------------------------
speed_goal = osim.MocoAverageSpeedGoal("avg_speed")
speed_goal.set_desired_average_speed(desired_speed)
problem.addGoal(speed_goal)

# ------------------------------ SOLVER -------------------------------------
solver = osim.MocoCasADiSolver.safeDownCast(study.updSolver())
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
tracking_solution = study.solve()
tracking_solution.write(output_warmstart)

print("✓ Warm start written to:", output_warmstart)