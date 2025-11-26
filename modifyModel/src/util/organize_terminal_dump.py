import re

"""
This script processes a raw terminal dump of model variable bounds,
cleans them up according to predefined criteria, and outputs a structured
CSV file to be inputted to our tracking study.
"""

# Step 1: Read in the raw dump file
with open("bounds/bounds.txt", "r") as f:
    lines = f.readlines()

# Step 2: Parse the dump into a structured list
# Each entry will be a dictionary: {"name": ..., "lower": ..., "upper": ...}
bounds_list = []

pattern = re.compile(r'\[info\]\s+(\S+)\. bounds: \[([^\]]+)\]')

for line in lines:
    match = pattern.search(line)
    if match:
        name = match.group(1)
        lower, upper = match.group(2).split(",")
        bounds_list.append({
            "name": name,
            "lower": float(lower),
            "upper": float(upper)
        })

# Step 3: Define maximum allowed bounds
MAX_ABS = 10        # max absolute value for joint coordinates
MIN_ACTIVATION = 0.01
MAX_ACTIVATION = 1.0
MAX_NORMALIZED_FORCE = 5.0
MAX_SPEED = 50.0

# Step 4: Update bounds based on categories
for entry in bounds_list:
    name = entry["name"]
    lower = entry["lower"]
    upper = entry["upper"]

    # Adjust activations
    if "activation" in name:
        entry["lower"] = max(lower, MIN_ACTIVATION)
        entry["upper"] = min(upper, MAX_ACTIVATION)
    # Adjust normalized tendon forces
    elif "normalized_tendon_force" in name:
        entry["lower"] = max(lower, 0)
        entry["upper"] = min(upper, MAX_NORMALIZED_FORCE)
    # Adjust speeds
    elif "speed" in name:
        entry["lower"] = max(lower, -MAX_SPEED)
        entry["upper"] = min(upper, MAX_SPEED)
    # Adjust joint coordinates
    elif "value" in name:
        # Skip activations/forces (already handled)
        if not any(x in name for x in ["activation", "normalized_tendon_force"]):
            entry["lower"] = max(lower, -MAX_ABS)
            entry["upper"] = min(upper, MAX_ABS)

# Step 5: Optional - write back to a new CSV for review
import csv

with open("bounds/processed_bounds.csv", "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["name", "lower_bound", "upper_bound"])
    for entry in bounds_list:
        writer.writerow([entry["name"], entry["lower"], entry["upper"]])

print("Processed bounds saved to processed_bounds.csv")
