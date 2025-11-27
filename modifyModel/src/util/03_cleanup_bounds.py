import opensim as osim
import math

input = "models/prosthesisModel_9.osim"
output = "models/prosthesisModel_9.osim"

# Load your model
model = osim.Model(input)
state = model.initSystem()


# Default anatomical limits for known coordinates
default_coord_limits = {
    "hip_flexion": (-math.pi/2, math.pi),
    "hip_adduction": (-math.pi/2, math.pi/2),
    "hip_rotation": (-math.pi, math.pi),
    "knee_angle": (0, 2*math.pi/3),
    "ankle_angle": (-math.pi/4, math.pi/4),
    "elbow_flex": (0, 2*math.pi/3),
    "shoulder_flex": (-math.pi/2, math.pi/2),
    "shoulder_abduction": (-math.pi/2, math.pi/2),
    "shoulder_rotation": (-math.pi, math.pi),
    "pro_sup": (0, math.pi),
    "lumbar_bending": (-math.pi/2, math.pi/2),
    "lumbar_rotation": (-math.pi, math.pi),
    "lumbar_extension": (-math.pi/2, math.pi/2),
    "pelvis_tx": (-0.2, 0.2),
    "pelvis_ty": (-0.2, 0.2),
    "pelvis_tz": (-0.2, 0.2),
    "pelvis_tilt": (-math.pi/4, math.pi/4),
    "pelvis_list": (-math.pi/4, math.pi/4),
    "pelvis_rotation": (-math.pi, math.pi)
}

# Fallback range for any coordinate not in the dictionary
fallback_range = (-math.pi/2, math.pi/2)  # conservative ±90°

# Iterate over all coordinates
for coord in model.getCoordinateSet():
    name = coord.getName()
    lower = coord.getRangeMin()
    upper = coord.getRangeMax()

    print(f" {name}: range [{coord.getRangeMin()}, {coord.getRangeMax()}]")

    # If coordinate has extreme bounds
    if lower <= -1e5 or math.isinf(lower) or upper >= 1e5 or math.isinf(upper):
        # Use default if we have a match, otherwise fallback
        applied = False
        for key, (l, u) in default_coord_limits.items():
            if key in name:
                coord.setRangeMin(l)
                coord.setRangeMax(u)
                applied = True
                break
        if not applied:
            coord.setRangeMin(fallback_range[0])
            coord.setRangeMax(fallback_range[1])
        print(f"Adjusted {name}: range [{coord.getRangeMin()}, {coord.getRangeMax()}]")

for coord in model.getCoordinateSet():
    if math.isinf(coord.getRangeMin()) or math.isinf(coord.getRangeMax()):
        print(coord.getName(), coord.getRangeMin(), coord.getRangeMax())

for force in model.getForceSet():
    if hasattr(force, "getMinControl") and hasattr(force, "getMaxControl"):
        min_ctrl = force.getMinControl()
        max_ctrl = force.getMaxControl()
        if math.isinf(min_ctrl) or math.isinf(max_ctrl):
            print(force.getName(), min_ctrl, max_ctrl)


# Save modified model
model.printToXML(output)
