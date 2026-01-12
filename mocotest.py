import opensim as osim

print(osim.GetVersionAndDate())   # should print an OpenSim version like 4.5.x

# Check that Moco is available:
study = osim.MocoStudy()
problem = study.updProblem()
solver = study.initCasADiSolver()

print("Moco is working!")