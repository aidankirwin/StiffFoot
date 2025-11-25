import opensim as osim
import pandas as pd
import numpy as np

def get_relative_angle(row, df, deg=False):
    """
    Compute the relative angle between a segment and its parent.
    
    Parameters
    ----------
    row : pd.Series
        The current segment row (must have 'Segment', 'Parent', 'start_x', 'start_y', 'end_x', 'end_y').
    df : pd.DataFrame
        The full DataFrame containing all segments.
    deg : bool
        If True, returns the angle in degrees. Default is radians.
    
    Returns
    -------
    float
        The relative angle (signed) between the segment and its parent.
    """
    if row['Parent'] == 0:
        # No parent (e.g., connects to pylon)
        return 0.0
    
    # Find parent row
    parent_row = df[df['Segment'] == row['Parent']].iloc[0]
    
    # Segment vectors
    seg_vec = np.array([row['end_x'] - row['start_x'], row['end_y'] - row['start_y']])
    parent_vec = np.array([parent_row['end_x'] - parent_row['start_x'],
                           parent_row['end_y'] - parent_row['start_y']])
    
    # Compute signed angle using arctan2
    dot = np.dot(parent_vec, seg_vec)
    det = parent_vec[0]*seg_vec[1] - parent_vec[1]*seg_vec[0]
    angle = np.arctan2(det, dot)
    
    if deg:
        angle = np.degrees(angle)
    
    return angle

# Load model
model = osim.Model("original_model/GenericAmputee_r.osim")
print("Loaded model")

# Load segment coordinates from CSV
df = pd.read_csv("segments.csv")
print(df)

# Parameters
segment_radius = 0.01   # visualization radius
segment_mass = 0.05
segment_inertia_val = 0  # simple thin cylinder inertia

# Initial parent is the pylon
parent_body = model.getBodySet().get("pylon_r")
parent_length = 0.05 # length along parent z-axis from joint origin to distal end

for idx, row in df.iterrows():
    '''
    Attach prosthetic foot segments to the pylon.
    Rules:
    1. Each segment is described in segments.csv by its vertices (start and end position)
    2. The vertices are relative to the distal end of the pylon (0,0)
    '''
    seg_name = f"segment_{int(row['Segment'])}"
    
    # Compute segment vector in 2D
    dx = row['end_x'] - row['start_x']
    dy = row['end_y'] - row['start_y']
    length = round(np.sqrt(dx**2 + dy**2), 3) 
    
    # Mass center at middle of segment
    mass_center = osim.Vec3(0, 0, length/2)
    
    # Cylinder geometry
    cyl = osim.Cylinder(segment_radius, length)

    # angle = get_relative_angle(row, df, False)

    if idx != 0:
        parent_body = model.getBodySet().get(f"segment_{int(row['Parent'])}")
    # Create new segment
    segment = osim.Body(
        seg_name,
        segment_mass,
        mass_center,
        osim.Inertia(segment_inertia_val, segment_inertia_val, segment_inertia_val)
    )
    model.addBody(segment)
    segment.attachGeometry(cyl)

    # PinJoint: connect to distal end of parent
    joint = osim.PinJoint(
        f"joint_{seg_name}",
        parent_body,
        osim.Vec3(0,-2*parent_length,0),  # distal end of parent
        osim.Vec3(0,0,0),              # parent orientation
        segment,
        osim.Vec3(0,0,0),              # child proximal end
        osim.Vec3(0,0,0)          # child orientation along z
    )
    model.addJoint(joint)
    
    # Update parent for next iteration
    parent_body = segment
    parent_length = length  # length along local z-axis for next joint

# Finalize and initialize system
model.finalizeConnections()
state = model.initSystem()
print("Segments added/modified and model assembled successfully.")

# Save the updated model
model.printToXML("models/new_model.osim")
print("Saved model as new_model.osim")