import opensim as osim
study = osim.MocoStudy()
solver = study.initCasADiSolver()
print("Moco and IPOPT are working!")