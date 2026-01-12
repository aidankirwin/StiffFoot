import pandas as pd

original_sto = "sto/coordinates.csv"
output_sto = "sto/coordinates_amputee.sto"

# 20 joint segment coordinates
added_dofs = [f"/jointset/joint_segment_{i}/joint_segment_{i}_coord_0/value" for i in range(1, 21)]

# Initialization value for missing DOFs
default_value = 0.0

# DOFs that must be removed (removed from model)
remove_dofs = [
    "ankle_angle_r"  # deleted right ankle due to prosthesis
]

# Load original STO
df = pd.read_csv(original_sto)

print(f'original # of columns: {len(df.columns)}')

cols_to_drop = [
    col for col in df.columns
    if any(sub in col for sub in remove_dofs)
]

print(f'columns to remove: {cols_to_drop}')

df = df.drop(columns=cols_to_drop)

# Add missing DOFs with default initialization
# for dof in added_dofs:
#     if dof not in df.columns:
#         df[dof] = default_value
#         print(f"Added column: {dof} = {default_value}")

print(df.columns)

# convert all values to radians
for col in df.columns:
    if col not in ["time", "/jointset/ground_pelvis/pelvis_tx/value", "/jointset/ground_pelvis/pelvis_ty/value", "/jointset/ground_pelvis/pelvis_tz/value"]:
        df[col] = df[col].apply(lambda x: x * (3.141592653589793 / 180.0))

        print(f"Converted column to radians: {col}")

# # Reorder
# df = df[["time"] + desired_order]

print(f'updated # of columns: {len(df.columns)}')
# Write new STO
with open(output_sto, "w") as f:
    f.write("sto_file_version=1.0\n")
df.to_csv(output_sto, sep='\t', index=False, mode='a')

print("Successfully wrote:", output_sto)
