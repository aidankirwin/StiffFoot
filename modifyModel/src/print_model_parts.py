import opensim as osim

model = osim.Model("GenericAmputee_r.osim")

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