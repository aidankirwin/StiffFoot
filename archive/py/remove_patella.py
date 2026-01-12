import opensim as osim

# Load model
model = osim.Model('original_model/GenericAmputee.osim')
print('Loaded model')

# Find muscles that attach to patella
muscles_to_remove = []
for i in range(model.getMuscles().getSize()):
    muscle = model.getMuscles().get(i)
    muscle_name = muscle.getName()
    
    # Check if any path points attach to patella
    geom_path = muscle.getGeometryPath()
    path_point_set = geom_path.getPathPointSet()
    
    for j in range(path_point_set.getSize()):
        path_point = path_point_set.get(j)
        body_name = path_point.getBodyName()
        if 'patella' in body_name.lower():
            muscles_to_remove.append(muscle_name)
            print(f'Muscle {muscle_name} attaches to {body_name}')
            break

print(f'\nMuscles to remove: {len(muscles_to_remove)}')

# Remove muscles
for muscle_name in muscles_to_remove:
    try:
        idx = model.getMuscles().getIndex(muscle_name)
        model.updForceSet().remove(idx)
        print(f'Removed muscle: {muscle_name}')
    except:
        print(f'Failed to remove: {muscle_name}')

# Find and remove coordinate couplers related to patella
print('\nChecking constraints...')
constraints_to_remove = []
for i in range(model.getConstraintSet().getSize()):
    constraint = model.getConstraintSet().get(i)
    constraint_name = constraint.getName()
    if 'patellofemoral' in constraint_name.lower():
        constraints_to_remove.append(constraint_name)
        print(f'Constraint to remove: {constraint_name}')

# Remove constraints in reverse order to avoid index issues
for constraint_name in reversed(constraints_to_remove):
    try:
        idx = model.getConstraintSet().getIndex(constraint_name)
        model.updConstraintSet().remove(idx)
        print(f'Removed constraint: {constraint_name}')
    except Exception as e:
        print(f'Failed to remove constraint {constraint_name}: {e}')

# Find and remove patella joints
joints_to_remove = []
for i in range(model.getJointSet().getSize()):
    joint = model.getJointSet().get(i)
    joint_name = joint.getName()
    
    # Check if joint involves patella
    child_frame = joint.getChildFrame()
    child_name = child_frame.findBaseFrame().getName()
    
    if 'patella' in child_name.lower() or 'patella' in joint_name.lower():
        joints_to_remove.append(joint_name)
        print(f'Joint to remove: {joint_name} (child: {child_name})')

print(f'\nRemoving {len(joints_to_remove)} joints')
for joint_name in joints_to_remove:
    try:
        idx = model.getJointSet().getIndex(joint_name)
        model.updJointSet().remove(idx)
        print(f'Removed joint: {joint_name}')
    except Exception as e:
        print(f'Failed to remove joint {joint_name}: {e}')

# Remove patella bodies
bodies_to_remove = ['patella_r', 'patella_l']
print(f'\nRemoving {len(bodies_to_remove)} bodies')
for body_name in bodies_to_remove:
    try:
        idx = model.getBodySet().getIndex(body_name)
        model.updBodySet().remove(idx)
        print(f'Removed body: {body_name}')
    except Exception as e:
        print(f'Failed to remove body {body_name}: {e}')

# Save modified model
print('\nFinalizing model...')
model.finalizeConnections()
state = model.initSystem()
print('Model assembled successfully!')

output_path = 'models/GenericAmputee_no_patella.osim'
model.printToXML(output_path)
print(f'Saved: {output_path}')
