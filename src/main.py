import opensim as osim
from simulated_annealing import run_simulated_annealing

if __name__ == "__main__":
    # Set up logging to file
    osim.Logger.addFileSink('full_log.txt')
    run_simulated_annealing()