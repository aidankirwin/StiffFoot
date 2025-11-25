import opensim as osim
import math
import re

# ----------------------------------------
# STEP 1: TRACKING SIMULATION
# ----------------------------------------

study = osim.MocoStudy()
study.setName("gait_tracking")

problem = study.updProblem()
modelProcessor = osim.ModelProcessor('2D_gait.osim')
problem.setModelProcessor(modelProcessor)

# Load reference kinematics from the example data.
trackingTable = osim.TimeSeriesTable('referenceCoordinates.sto')

# Tracking goal.
track = osim.MocoTrackGoal('tracking')
track.setReference(trackingTable)
track.setAllowUnusedReferences(True)
track.setWeight(1.0)
problem.addGoal(track)

# Speed goal (so walking speed is sensible).
speedGoal = osim.MocoAverageSpeedGoal('speed')
speedGoal.set_desired_average_speed(1.2)
problem.addGoal(speedGoal)

# Effort
effort = osim.MocoControlGoal('effort', weight=10)
effort.setExponent(3)
effort.setDivideByDisplacement(True)
problem.addGoal(effort)

# Basic bounds.
problem.setTimeBounds(0, [0.4, 0.6])
problem.setStateInfo('/jointset/groundPelvis/pelvis_ty/value', [0.75, 1.25])
problem.setStateInfo('/jointset/groundPelvis/pelvis_tx/value', [0, 1])
problem.setStateInfo('/jointset/groundPelvis/pelvis_tilt/value', [-20*math.pi/180, -10*math.pi/180])

# Configure solver.
solver = study.initCasADiSolver()
solver.set_num_mesh_intervals(50)
solver.set_verbosity(2)

# Solve.
solution = study.solve()
solution.write('warmStart_solution.sto')

print("Tracking simulation complete. Warm start saved as warmStart_solution.sto")
