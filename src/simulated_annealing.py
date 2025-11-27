from generate_model import generate_model_with_segments
from convert_muscles import convert_muscles_to_degroote
from tracking import solve_metabolic_tracking

import opensim as osim

def run_full_pipeline():
    # Set up logging to file
    osim.Logger.addFileSink('full_log.txt')

    # Step 1: Generate modified model with prosthetic segments
    print("Generating modified model with prosthetic segments...")
    generate_model_with_segments(save_model=True)

    # Step 2: Convert muscles to DeGrooteFregly2016Muscle
    print("Converting muscles to DeGrooteFregly2016Muscle...")
    processed_model = convert_muscles_to_degroote(save_model=True)

    # Step 3: Solve metabolic/tracking problem
    print("Solving tracking and metabolic cost problem...")
    solve_metabolic_tracking(model=processed_model)

if __name__ == "__main__":
    run_full_pipeline()