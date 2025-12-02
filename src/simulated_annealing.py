import opensim as osim

# for running simulated annealing
from generate_model import generate_model_with_segments
from convert_muscles import convert_muscles_to_degroote
from gait_simulation import solve_metabolic_tracking
from scipy.optimize import dual_annealing   # scipy simulated annealing function

# saving and plotting results
import pickle
import matplotlib.pyplot as plt

init_stiffness = [785, 1221, 1334, 1334, 1334, 1334, 1334, 1334, 1099, 1334, 
                  916, 610, 916, 523, 366, 1221, 1334, 1334, 1334, 1221]

def run_full_pipeline(x):
    """
    Run the full pipeline of generating the model, converting muscles,
    and solving the metabolic tracking problem for a given set of
    prosthetic segment stiffness parameters.
    """

    # Step 1: Generate modified model with prosthetic segments
    print("Generating modified model with prosthetic segments...")
    model = generate_model_with_segments(stiffness_array=x)

    # Step 2: Convert muscles to DeGrooteFregly2016Muscle
    print("Converting muscles to DeGrooteFregly2016Muscle...")
    model_degroote_fregly = convert_muscles_to_degroote(model=model)

    # Step 3: Solve metabolic/tracking problem
    print("Solving tracking and metabolic cost problem...")
    met = solve_metabolic_tracking(model=model_degroote_fregly, iterations=10)
    return met

def run_simulated_annealing():
    """
    Runs the whole simulated annealing pipeline to optimize prosthetic segment stiffnesses
    to minimize metabolic cost during gait simulation.
    """

    # Define bounds for parameters to optimize
    # Bounds for 20 stiffnesses are 3.4 Nm deg−1 and 23.3 Nm deg−1 (194.8 to 1334 Nms rad−1)
    bounds = [(194.8, 1334) for _ in range(20)]
    res = dual_annealing(run_full_pipeline, bounds, x0=init_stiffness, maxiter=20, no_local_search=True)
    print("Optimization completed.")
    print("Best parameters found:", res.x)
    print("With objective function value (metabolic cost):", res.fun)

    # save optimization results to a pkl file
    with open('simulated_annealing_results.pkl', 'wb') as f:
        pickle.dump(res, f)
    print("Saved optimization results to simulated_annealing_results.pkl")

    # plot convergence
    # this is untested
    plt.plot(res.func_vals)
    plt.xlabel('Iteration')
    plt.ylabel('Metabolic Cost')
    plt.title('Simulated Annealing Convergence')
    plt.savefig('simulated_annealing_convergence.png')
    print("Saved convergence plot to simulated_annealing_convergence.png")

if __name__ == "__main__":
    # Set up logging to file
    osim.Logger.addFileSink('full_log.txt')

    run_simulated_annealing()