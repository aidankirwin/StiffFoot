import pandas as pd

# ========== USER SETTINGS =================================================== #

original_sto = "sto/coordinates_deg.csv"
output_sto = "sto/coords_modified_new.sto"

# 20 joint segment coordinates
added_dofs = [f"/jointset/joint_segment_{i}/joint_segment_{i}_coord_0/value" for i in range(1, 21)]

# Initialization value for missing DOFs
default_value = 0.0

# DOFs that must be removed (removed from model)
remove_dofs = [
    "ankle_angle_r"  # deleted right ankle due to prosthesis
]

# ========== COORDINATE ORDER FROM YOUR MODIFIED MODEL ======================= #

# desired_order = [
#     "pelvis_tilt",
#     "pelvis_list",
#     "pelvis_rotation",
#     "pelvis_tx",
#     "pelvis_ty",
#     "pelvis_tz",
#     "hip_flexion_r",
#     "hip_adduction_r",
#     "hip_rotation_r",
#     "knee_angle_r",
#     "knee_angle_r_beta",
#     "socket_rotation_r",
#     "socket_piston_r",
#     "SP_rotation_r",
#     "hip_flexion_l",
#     "hip_adduction_l",
#     "hip_rotation_l",
#     "knee_angle_l",
#     "knee_angle_l_beta",
#     "ankle_angle_l",
#     "subtalar_angle_l",
#     "mtp_angle_l",
#     "lumbar_extension",
#     "lumbar_bending",
#     "lumbar_rotation",
#     "arm_flex_r",
#     "arm_add_r",
#     "arm_rot_r",
#     "elbow_flex_r",
#     "pro_sup_r",
#     "wrist_flex_r",
#     "wrist_dev_r",
#     "arm_flex_l",
#     "arm_add_l",
#     "arm_rot_l",
#     "elbow_flex_l",
#     "pro_sup_l",
#     "wrist_flex_l",
#     "wrist_dev_l",
# ] + segment_dofs

# =========================================================================== #


# Load original STO
df = pd.read_csv(original_sto)

cols_to_drop = [
    col for col in df.columns
    if any(sub in col for sub in remove_dofs)
]

df = df.drop(columns=cols_to_drop)

# Add missing DOFs with default initialization
for dof in added_dofs:
    if dof not in df.columns:
        df[dof] = default_value
        print(f"Added column: {dof} = {default_value}")

print(df.columns)

# convert all values to radians
for col in df.columns:
    if col != "time":
        df[col] = df[col].apply(lambda x: x * (3.141592653589793 / 180.0))
        print(f"Converted column to radians: {col}")

# # Reorder
# df = df[["time"] + desired_order]

print(len(df.columns))
# Write new STO
with open(output_sto, "w") as f:
    f.write("sto_file_version=1.0\n")
df.to_csv(output_sto, sep='\t', index=False, mode='a')

print("Successfully wrote:", output_sto)
