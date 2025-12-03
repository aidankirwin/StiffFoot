import opensim as osim

# for running simulated annealing
from generate_model import generate_model_with_segments
from convert_muscles import convert_muscles_to_degroote
from gait_simulation import solve_metabolic_tracking
from scipy.optimize import dual_annealing   # scipy simulated annealing function

# saving and plotting results
import pickle
import matplotlib.pyplot as plt

# Our input model, amputated version of the Rajagopal (2016) 3D musculoskeletal model
template_model = "original_model/GenericAmputee_r.osim"
# Define bounds for parameters to optimize
# Bounds for 20 stiffnesses are 3.4 Nm deg−1 and 23.3 Nm deg−1, from: https://link.springer.com/article/10.1186/s12984-021-00916-1#Sec2
bounds = [(3.4, 23.3) for _ in range(20)]
# initial stiffness parameters, calculated from the Young's modulus (E), area moment of inertia of each segment, and length of each segment
# formula and Young's modulus from: https://pmc.ncbi.nlm.nih.gov/articles/PMC3707817/#s4
init_stiffness = [13.71, 21.32, 23.3, 23.3, 23.3, 23.3, 23.3, 23.3, 19.19, 23.3, 
                  15.99, 10.66, 15.99, 9.14, 6.4, 21.32, 23.3, 23.3, 23.3, 21.32]

counter = 1
sim_annealing_max_iter = 20
gait_cycle_max_iter = 10

simulated_annealing_data = []
minimas = []

def run_full_pipeline(x):
    global counter
    """
    Run the full pipeline of generating the model, converting muscles,
    and solving the metabolic tracking problem for a given set of
    prosthetic segment stiffness parameters.
    """

    print('====================================================================================')
    print(f'RUNNING GAIT CYCLE OPTIMIZATION {counter} OF {sim_annealing_max_iter}')
    print('====================================================================================')

    model = osim.Model(template_model)

    # Step 1: Convert muscles to DeGrooteFregly2016Muscle
    print("Converting muscles to DeGrooteFregly2016Muscle...")
    model_degroote_fregly = convert_muscles_to_degroote(model=model)

    # Step 2: Generate modified model with prosthetic segments
    print("Generating modified model with prosthetic segments...")
    prosthesis_model = generate_model_with_segments(model=model_degroote_fregly, stiffness_array=x)

    # Step 3: Solve metabolic/tracking problem
    print("Solving tracking and metabolic cost problem...")
    met = solve_metabolic_tracking(model=prosthesis_model, iterations=gait_cycle_max_iter)

    counter += 1    # track simulated annealing iterations

    simulated_annealing_data.append({'iteration': counter-1, 'x': x, 'objective': met})
    print('====================================================================================')
    print(f'SAVING DATA FROM ITERATION {counter}')
    print('====================================================================================')

    return met

def store_iteration_data(x, f, context):
    minimas.append({'iteration': counter-1, 'x': x, 'objective': f, 'context': context})
    print('====================================================================================')
    print(f'MINIMA DETECTED - SAVING DATA FROM ITERATION {counter} TO MINIMA ARRAY')
    print('====================================================================================')

def run_simulated_annealing():
    """
    Runs the whole simulated annealing pipeline to optimize prosthetic segment stiffnesses
    to minimize metabolic cost during gait simulation.
    """

    res = dual_annealing(run_full_pipeline, bounds, x0=init_stiffness, callback=store_iteration_data, maxfun=sim_annealing_max_iter, maxiter=sim_annealing_max_iter, no_local_search=True)
    print("Optimization completed.")
    print("Best parameters found:", res.x)
    print("With objective function value (metabolic cost):", res.fun)

    # save optimization results to a pkl file
    with open('annealing_result_object.pkl', 'wb') as f:
        pickle.dump(res, f)

    # save minima data
    with open('minima_data.pkl', 'wb') as f:
        pickle.dump(minimas, f)

    # save individual iteration data
    with open('iteration_data.pkl', 'wb') as f:
        pickle.dump(simulated_annealing_data, f)

if __name__ == "__main__":
    # Set up logging to file
    osim.Logger.addFileSink('full_log.txt')
    run_simulated_annealing()