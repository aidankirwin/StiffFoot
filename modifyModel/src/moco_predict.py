import opensim as osim
import math
import re

# ----------------------------------------
# STEP 2: PREDICTIVE SIMULATION
# ----------------------------------------

study = osim.MocoStudy()
study.setName('gaitPrediction')

problem = study.updProblem()
modelProcessor = osim.ModelProcessor('2D_gait.osim')
problem.setModelProcessor(modelProcessor)

# ------------------------------
# Periodicity (one step)
# ------------------------------
sym = osim.MocoPeriodicityGoal('periodicity')
problem.addGoal(sym)

model = modelProcessor.process()
model.initSystem()

# Periodicity on joint states.
for i in range(model.getNumStateVariables()):
    name = model.getStateVariableNames().getitem(i)
    # coordinate state
    if name.startswith('/jointset'):
        if '_r' in name:
            sym.addStatePair(osim.MocoPeriodicityGoalPair(name, name.replace('_r','_l')))
        elif '_l' in name:
            sym.addStatePair(osim.MocoPeriodicityGoalPair(name, name.replace('_l','_r')))
        else:
            # everything except pelvis forward translation
            if 'pelvis_tx/value' not in name:
                sym.addStatePair(osim.MocoPeriodicityGoalPair(name))

# Muscle activations periodicity.
for i in range(model.getNumStateVariables()):
    name = model.getStateVariableNames().getitem(i)
    if name.endswith('/activation'):
        if '_r' in name:
            sym.addStatePair(osim.MocoPeriodicityGoalPair(name, name.replace('_r','_l')))
        elif '_l' in name:
            sym.addStatePair(osim.MocoPeriodicityGoalPair(name.replace('_l','_r'), name))

# Lumbar actuator control periodic.
sym.addControlPair(osim.MocoPeriodicityGoalPair('/lumbarAct'))

# ------------------------------
# Speed goal (for target walking speed)
# ------------------------------
speed = osim.MocoAverageSpeedGoal('speed')
speed.set_desired_average_speed(1.2)
problem.addGoal(speed)

# ------------------------------
# Effort or metabolic goal
# ------------------------------
effort = osim.MocoControlGoal('effort', weight=10)
effort.setExponent(3)
effort.setDivideByDisplacement(True)
problem.addGoal(effort)

# ------------------------------
# Bounds
# ------------------------------
problem.setTimeBounds(0, [0.4, 0.6])
problem.setStateInfo('/jointset/groundPelvis/pelvis_tx/value', [0, 1])
problem.setStateInfo('/jointset/groundPelvis/pelvis_ty/value', [0.75, 1.25])
problem.setStateInfo('/jointset/groundPelvis/pelvis_tilt/value', [-20*math.pi/180, -10*math.pi/180])
problem.setStateInfo('/jointset/hip_l/hip_flexion_l/value', [-10*math.pi/180, 60*math.pi/180])
problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value', [-10*math.pi/180, 60*math.pi/180])
problem.setStateInfo('/jointset/knee_l/knee_angle_l/value', [-50*math.pi/180, 0])
problem.setStateInfo('/jointset/knee_r/knee_angle_r/value', [-50*math.pi/180, 0])
problem.setStateInfo('/jointset/ankle_l/ankle_angle_l/value', [-15*math.pi/180, 25*math.pi/180])
problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value', [-15*math.pi/180, 25*math.pi/180])
problem.setStateInfo('/jointset/lumbar/lumbar/value', [0, 20*math.pi/180])

# ------------------------------
# Solver settings
# ------------------------------
solver = study.initCasADiSolver()
solver.set_num_mesh_intervals(50)
solver.set_verbosity(2)
solver.set_optim_max_iterations(1000)

# Use warm start from tracking
warmstart = osim.MocoTrajectory('warmStart_solution.sto')
solver.setGuess(warmstart)

# ------------------------------
# Solve
# ------------------------------
solution = study.solve()
solution.write('predictive_solution.sto')

print("Predictive simulation complete. Saved as predictive_solution.sto")
