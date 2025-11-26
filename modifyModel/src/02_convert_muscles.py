import opensim as osim

input_model_file = "models/new_model.osim"
output_model_file = "models/prosthesisModel_5.osim"

model = osim.Model(input_model_file)
model.initSystem()

# Collect muscles to convert
muscles_to_convert = []
for i in range(model.getMuscles().getSize()):
    m = model.getMuscles().get(i)
    # detect Millard2012EquilibriumMuscle type
    if isinstance(m, osim.Millard2012EquilibriumMuscle):
        muscles_to_convert.append(m.getName())

for name in muscles_to_convert:
    old_m = model.updComponent(name)  # or model.getMuscles().get(name)
    # Create new DeGroote muscle (give same name)
    new_m = osim.DeGrooteFregly2016Muscle(name)
    # Copy basic mechanical params
    new_m.set_max_isometric_force(old_m.getMaxIsometricForce())
    new_m.set_optimal_fiber_length(old_m.getOptimalFiberLength())
    new_m.set_tendon_slack_length(old_m.getTendonSlackLength())
    new_m.set_pennation_angle_at_optimal(old_m.getPennationAngleAtOptimal())
    # Max contraction velocity (units: optimal_fiber_lengths_per_second)
    try:
        new_m.set_max_contraction_velocity(old_m.getMaxContractionVelocity())
    except Exception:
        # method name may vary by API; check available getters on your version
        pass

    # Activation/deactivation time constants (if present)
    try:
        new_m.set_activation_time_constant(old_m.getActivationTimeConstant())
        new_m.set_deactivation_time_constant(old_m.getDeactivationTimeConstant())
    except Exception:
        pass

    # Copy geometry path (path points). Wraps require extra handling.
    old_geom_path = old_m.getGeometryPath()
    for j in range(old_geom_path.getPathPointSet().getSize()):
        pp = old_geom_path.getPathPointSet().get(j)
        # Create a PathPoint attached to the same body with the same local location
        new_pp = osim.PathPoint(pp.getName(),
                                model.getBodySet().get(pp.getBody().getName()),
                                pp.getLocation())
        new_m.addNewPathPoint(new_pp)

    # Optional: copy other scalar properties you care about (damping, etc.)
    try:
        new_m.set_fiber_damping(old_m.getFiberDamping())
    except Exception:
        pass

    # Add new muscle to the model
    model.addComponent(new_m)

    # Remove old muscle (do after adding new to avoid disconnect issues)
    model.removeComponent(old_m.getAbsolutePathString())

# Finalize and save
model.initSystem()
model.printToXML(output_model_file)
print("Saved converted model to", output_model_file)