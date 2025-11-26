import opensim as osim
import math

# ===========================================
# Predictive 3D gait using Rajagopal model
# minimizing Umberger metabolic cost
# ===========================================

study = osim.MocoStudy()
study.setName("predictive3Dgait")

problem = study.updProblem()

# Load and process the full 3D musculoskeletal model.
modelProcessor = osim.ModelProcessor("models/prosthesisModel.osim")
modelProcessor.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
modelProcessor.append(osim.ModOpIgnorePassiveFiberForcesDGF())
modelProcessor.append(osim.ModOpIgnoreTendonCompliance())
problem.setModelProcessor(modelProcessor)

model = modelProcessor.process()
model.initSystem()

# ===========================================
# Periodicity for one right-foot step
# ===========================================
periodic = osim.MocoPeriodicityGoal("periodicity")
problem.addGoal(periodic)

stateNames = model.getStateVariableNames()
for i in range(stateNames.getSize()):
    name = stateNames.get(i)
    # coordinate or activation
    if "_r" in name:
        periodic.addStatePair(osim.MocoPeriodicityGoalPair(name, name.replace("_r","_l")))
    elif "_l" in name:
        periodic.addStatePair(osim.MocoPeriodicityGoalPair(name, name.replace("_l","_r")))
    else:
        # pelvic translation x is excluded
        if "pelvis_tx" not in name:
            periodic.addStatePair(osim.MocoPeriodicityGoalPair(name))

# ===========================================
# Target walking speed
# ===========================================
speed = osim.MocoAverageSpeedGoal("speed")
speed.set_desired_average_speed(1.3)  # typical 3D walking speed
problem.addGoal(speed)

# ===========================================
# Umberger metabolic cost
# ===========================================
met = osim.MocoUmberger2010MuscleMetabolicsGoal("metabolic")
met.setDivideByMass(True)
met.setUseMusclePhysiology(True)
problem.addGoal(met)

# ===========================================
# Bounds
# ===========================================
problem.setTimeBounds(0, [0.8, 1.4])  # typical gait cycle duration
problem.setStateInfo("/jointset/ground_pelvis/pelvis_tx/value", [0, 1.5])
problem.setStateInfo("/jointset/ground_pelvis/pelvis_ty/value", [0.8, 1.2])

# (OPTIONAL: Add joint-level bounds if solver behaves poorly)

# ===========================================
# Solver
# ===========================================
solver = study.initCasADiSolver()
solver.set_num_mesh_intervals(80)
solver.set_optim_max_iterations(2000)
solver.set_verbosity(2)

# ===========================================
# Warm start from tracking result
# ===========================================
guess = osim.MocoTrajectory("3d_warmstart.sto")
solver.setGuess(guess)

# ===========================================
# Solve
# ===========================================
solution = study.solve()
solution.write("predictive3D_solution.sto")

print("Predictive 3D gait simulation complete.")