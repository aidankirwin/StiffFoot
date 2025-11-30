import opensim as osim
import pandas as pd
import numpy as np

model_file = "original_model/GenericAmputee_r.osim"

def generate_model_with_segments(save_model=False, stiffness_array=None):
    """
    Generate a modified OpenSim model by adding prosthetic foot segments
    defined in 'segments.csv' to the base model 'original_model/GenericAmputee_no_patella.osim'.
    The modified model is saved as 'models/new_model.osim'.
    """
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
            return np.radians(-90.0)
        
        # Find parent row
        df['Segment'] = df['Segment'].astype(int)
        df['Parent'] = df['Parent'].astype(int)
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
            angle = np.radians(angle)
        
        return angle

    # Load model
    model = osim.Model(model_file)
    print("Loaded model")

    # Load segment coordinates from CSV
    df = pd.read_csv("segments.csv")
    print(df)

    # Parameters
    segment_radius = 0.01    # visualization radius
    segment_mass = 0.025     # mass of each segment, assume constant
    segment_inertia = [segment_mass * segment_radius**2 / 4 , segment_mass * segment_radius**2 / 4 , segment_mass * segment_radius**2 / 2]

    # These are used to calculate the initial values of the viscoelastic elements
    young_modulus = 1.4e9  # Pa (base untis: N/m^2)
    area_moment_of_inertia = (np.pi * segment_radius**4) / 4 # m^4

    # Initial parent is the pylon
    parent_body = model.getBodySet().get("pylon_r")
    parent_length = 0.155 # length along parent z-axis from joint origin to distal end

    damping = 5.73  # N*m/(rad/s)

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
        dx = dx / 2
        dy = dy / 2
        length = round(np.sqrt(dx**2 + dy**2), 3) 
        
        # Mass center at middle of segment
        mass_center = osim.Vec3(0, 0, length/2)
        
        # Cylinder geometry
        cyl = osim.Cylinder(segment_radius, length)

        angle = get_relative_angle(row, df, False)

        if stiffness_array is None:
            # Update rotational stiffness based on segment length
            k_rot = young_modulus * area_moment_of_inertia / length  # N*m/rad
            print(f"Segment {seg_name} length: {length} m, rotational stiffness: {k_rot} N*m/rad")
        else:
            k_rot = stiffness_array[idx]
            print(f"Segment {seg_name} length: {length} m, rotational stiffness from array: {k_rot} N*m/rad")

        if idx != 0:
            parent_body = model.getBodySet().get(f"segment_{int(row['Parent'])}")
        # Create new segment
        segment = osim.Body(
            seg_name,
            segment_mass,
            mass_center,
            osim.Inertia(segment_inertia[0], segment_inertia[1], segment_inertia[2])
        )
        model.addBody(segment)
        segment.attachGeometry(cyl)

        # PinJoint: connect to distal end of parent
        joint = osim.PinJoint(
            f"joint_{seg_name}",
            parent_body,
            osim.Vec3(0,-parent_length,0),  # distal end of parent
            osim.Vec3(0,0,angle),              # parent orientation
            segment,
            osim.Vec3(0,length,0),              # child proximal end
            osim.Vec3(0,0,0)          # child orientation along z
        )
        model.addJoint(joint)
        # TODO: maybe lock the first joint?
        # if idx == 0:
        #     coord = joint.upd_coordinates(0) # joint_segment_1_coord_0
        #     coord.set_locked(0, True)

        # use translation-only Transforms to avoid Rotation ctor overload issues
        parent_frame = joint.getParentFrame()
        child_frame = joint.getChildFrame()

        # Attach viscoelastic element (damper + spring) to joint
        ve = osim.BushingForce(
            f"viscoelastic_{seg_name}",
            parent_frame,
            child_frame,
            osim.Vec3(0,0,0),                # translational stiffness
            osim.Vec3(0,0,k_rot),            # rotational stiffness [N*m/rad]
            osim.Vec3(0,0,0),                # translational damping
            osim.Vec3(0,0,damping)           # rotational damping [N*m/(rad/s)]
        )
        model.addForce(ve)

        # Set coordinate default values
        coord = model.getCoordinateSet().get(f"joint_{seg_name}_coord_0")
        coord.setDefaultValue(0)
        # coord.setDefaultSpeed(0)
        
        # Update parent for next iteration
        parent_body = segment
        parent_length = length  # length along local z-axis for next joint

    # Finalize and initialize system
    model.finalizeConnections()
    model.initSystem()
    print("Segments added/modified and model assembled successfully.")

    if save_model:
        # Save the updated model
        model.printToXML("models/new_model.osim")
        print("Saved model as new_model.osim")
    
    return model

if __name__ == "__main__":
    generate_model_with_segments(save_model=True)