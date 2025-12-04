import os
import opensim as osim
import re
import math
import sys


# Define the optimal control problem
# ==================================
study = osim.MocoStudy()
study.setName('gaitPrediction')

problem = study.updProblem()
modelProcessor = osim.ModelProcessor('models/model_degroote.osim')
problem.setModelProcessor(modelProcessor)

# Goals
# =====

# Symmetry (to permit simulating only one step)
symmetryGoal = osim.MocoPeriodicityGoal('symmetryGoal')
problem.addGoal(symmetryGoal)
model = modelProcessor.process()
model.initSystem()

# Symmetric coordinate values (except for pelvis_tx) and speeds
for i in range(model.getNumStateVariables()):
    currentStateName = model.getStateVariableNames().getitem(i)
    if currentStateName.startswith('/jointset'):
        if currentStateName.__contains__('_r'):
            pair = osim.MocoPeriodicityGoalPair(currentStateName, 
                                                re.sub(r'_r', '_l', currentStateName))
            symmetryGoal.addStatePair(pair)
        if currentStateName.__contains__('_l'):
            pair = osim.MocoPeriodicityGoalPair(currentStateName, 
                                                re.sub(r'_l', '_r', currentStateName))
            symmetryGoal.addStatePair(pair)
        if not (currentStateName.__contains__('_r') 
                or currentStateName.__contains__('_l') 
                or currentStateName.__contains__('pelvis_tx/value') 
                or currentStateName.__contains__('/activation')):
            symmetryGoal.addStatePair(osim.MocoPeriodicityGoalPair(currentStateName))

# Symmetric muscle activations
for i in range(model.getNumStateVariables()):
    currentStateName = model.getStateVariableNames().getitem(i)
    if currentStateName.endswith('/activation'):
        if currentStateName.__contains__('_r'):
            pair = osim.MocoPeriodicityGoalPair(currentStateName, 
                                                re.sub(r'_r', '_l', currentStateName))
            symmetryGoal.addStatePair(pair)
        if currentStateName.__contains__('_l'):
            pair = osim.MocoPeriodicityGoalPair(currentStateName, 
                                                re.sub(r'_l', '_r', currentStateName))
            symmetryGoal.addStatePair(pair)

# Symmetric coordinate actuator controls
symmetryGoal.addControlPair(osim.MocoPeriodicityGoalPair('/lumbarAct'))

# Prescribed average gait speed
speedGoal = osim.MocoAverageSpeedGoal('speed')
problem.addGoal(speedGoal)
speedGoal.set_desired_average_speed(1.2)

# Effort over distance
effortGoal = osim.MocoControlGoal('effort', 10)
problem.addGoal(effortGoal)
effortGoal.setExponent(3)
effortGoal.setDivideByDisplacement(True)

# Bounds
# ======
problem.setTimeBounds(0, [0.4, 0.6])
problem.setStateInfo('/jointset/groundPelvis/pelvis_tilt/value', [-20*math.pi/180, -10*math.pi/180])
problem.setStateInfo('/jointset/groundPelvis/pelvis_tx/value', [0, 1])
problem.setStateInfo('/jointset/groundPelvis/pelvis_ty/value', [0.75, 1.25])
problem.setStateInfo('/jointset/hip_l/hip_flexion_l/value', [-10*math.pi/180, 60*math.pi/180])
problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value', [-10*math.pi/180, 60*math.pi/180])
problem.setStateInfo('/jointset/knee_l/knee_angle_l/value', [-50*math.pi/180, 0])
problem.setStateInfo('/jointset/knee_r/knee_angle_r/value', [-50*math.pi/180, 0])
problem.setStateInfo('/jointset/ankle_l/ankle_angle_l/value', [-15*math.pi/180, 25*math.pi/180])
problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value', [-15*math.pi/180, 25*math.pi/180])
problem.setStateInfo('/jointset/lumbar/lumbar/value', [0, 20*math.pi/180])


# Configure the solver
# ====================
solver = study.initCasADiSolver()
solver.set_num_mesh_intervals(50)
solver.set_verbosity(2)
solver.set_optim_solver('ipopt')
solver.set_optim_convergence_tolerance(1e-4)
solver.set_optim_constraint_tolerance(1e-4)
solver.set_optim_max_iterations(1000)
solver.setGuess(solver.createGuess())

# Solve problem
# =============
gaitPredictionSolution = study.solve()

# Create a full stride from the periodic single step solution.
# For details, view the Doxygen documentation for createPeriodicTrajectory().
fullStride = osim.createPeriodicTrajectory(gaitPredictionSolution)
fullStride.write('gaitPrediction_solution_fullStride.sto')


# Visualize the result.
study.visualize(fullStride)

# Extract ground reaction forces
# ==============================
contact_r = osim.StdVectorString()
contact_l = osim.StdVectorString()
contact_r.append('contactHeel_r')
contact_r.append('contactFront_r')
contact_l.append('contactHeel_l')
contact_l.append('contactFront_l')

# Create a conventional ground reaction forces file by summing the contact
# forces of contact spheres on each foot.
# For details, view the Doxygen documentation for
# createExternalLoadsTableForGait().
externalForcesTableFlat = osim.createExternalLoadsTableForGait(model, 
                                                               fullStride, contact_r, contact_l)
osim.STOFileAdapter.write(externalForcesTableFlat, 
                          'gaitPrediction_solutionGRF_fullStride.sto')