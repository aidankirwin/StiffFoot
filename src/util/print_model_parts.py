import opensim as osim

model = osim.Model("models/prosthesisModel_5_bounds_fixed.osim")

model.initSystem()

print("==== Coordinates ====")
for i in range(model.getCoordinateSet().getSize()):
    coord = model.getCoordinateSet().get(i)
    print(i, coord.getName())
    # print(coord.getStateVariableNames())

print("==== Bodies ====")
for i in range(model.getBodySet().getSize()):
    body = model.getBodySet().get(i)
    print(i, body.getName())

print("\n==== Joints ====")
for i in range(model.getJointSet().getSize()):
    joint = model.getJointSet().get(i)
    print(i, joint.getName(), "  parent:", joint.getParentFrame().getName(), "  child:", joint.getChildFrame().getName())

print("\n==== Muscles ====")
# OpenSim 4.5 — muscles are part of Model.getMuscles()
for i in range(model.getMuscles().getSize()):
    m = model.getMuscles().get(i)
    print(i, m.getName())

print(osim.GetVersionAndDate())