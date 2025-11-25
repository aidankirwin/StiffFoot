import opensim as osim

print("Using OpenSim version:", osim.GetVersionAndDate())

model_path = "gait10dof18musc.osim"
modelProcessor = osim.ModelProcessor(model_path)
modelProcessor.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
modelProcessor.append(osim.ModOpIgnorePassiveFiberForcesDGF())
modelProcessor.append(osim.ModOpAddReserves(10))


model = modelProcessor.process()

metabolics = osim.Bhargava2004SmoothedMuscleMetabolics()
metabolics.setName("metabolics")
metabolics.set_use_smoothing(True)

for m in model.getMuscles():
    metabolics.addMuscle(m.getName(), osim.Muscle.safeDownCast(model.getComponent(m.getName())))

model.addComponent(metabolics)
model.finalizeConnections()

study = osim.MocoStudy()
problem = study.updProblem()
problem.setModel(model)

problem.setTimeBounds(osim.MocoInitialBounds(0), osim.MocoFinalBounds(1))

#joint = model.getJointSet().get("ground_pelvis")

problem.setStateInfo(
    "/jointset/ground_pelvis/pelvis_tx/value",
    osim.MocoBounds(0, 2),
    osim.MocoInitialBounds(0),
    osim.MocoFinalBounds(1)
)

problem.setStateInfo(
    "/jointset/ground_pelvis/pelvis_ty/value",
    osim.MocoBounds(0.8, 1.1)
)

problem.setStateInfo(
    "/jointset/ground_pelvis/pelvis_tilt/value",
    osim.MocoBounds(-0.2, 0.2),
    osim.MocoInitialBounds(0),
    osim.MocoFinalBounds(0)
)

problem.setStateInfoPattern("/forceset/.*/activation", 
                            osim.MocoBounds(0.01, 1),
                            osim.MocoInitialBounds(0.01, 0.2),
                            osim.MocoFinalBounds(0.01, 1))

metabolic_goal = osim.MocoOutputGoal("metabolic_energy", 1.0)
metabolic_goal.setOutputPath("/metabolics|total_metabolic_rate")
problem.addGoal(metabolic_goal)

solver = study.initCasADiSolver()
solver.set_num_mesh_intervals(30)
solver.set_optim_convergence_tolerance(1e-2)
solver.set_optim_constraint_tolerance(1e-2)
solver.set_optim_max_iterations(1000)
solver.setGuess("bounds")

print("Starting optimization...")
solution = study.solve()

if solution.success():
    print("SUCCESS!")
    print("Total metabolic energy:", solution.getObjective())
    solution.write("walking_minimum_metabolic_solution.sto")
    print("Solution saved!")
else:
    print("Optimization failed due to numerical issues with muscle model")