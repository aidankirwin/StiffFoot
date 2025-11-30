from generate_model import generate_model_with_segments
from convert_muscles import convert_muscles_to_degroote
from gait_simulation import solve_metabolic_tracking

import opensim as osim

from scipy.optimize import dual_annealing

def run_full_pipeline(x):
    # Step 1: Generate modified model with prosthetic segments
    print("Generating modified model with prosthetic segments...")
    model = generate_model_with_segments()

    # Step 2: Convert muscles to DeGrooteFregly2016Muscle
    print("Converting muscles to DeGrooteFregly2016Muscle...")
    model_degroote_fregly = convert_muscles_to_degroote(model=model)

    # Step 3: Solve metabolic/tracking problem
    print("Solving tracking and metabolic cost problem...")
    met = solve_metabolic_tracking(model=model_degroote_fregly, iterations=10)
    return met

def run_simulated_annealing():
    """
    Runs simulated annealing 
    """

    # Define bounds for parameters to optimize
    bounds = [ ()]

if __name__ == "__main__":
    # Set up logging to file
    osim.Logger.addFileSink('full_log.txt')

    run_simulated_annealing()