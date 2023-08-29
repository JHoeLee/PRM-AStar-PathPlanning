import numpy as np
import pylab as pl
import sys
sys.path.append('osr_examples/scripts/')
import environment_2d
from AStar import AStar
from PRM import PRM
import time
import argparse

def main(seed, num_samples, connection_radius):
    
    # Set up the environment
    pl.ion()
    np.random.seed(seed) # Change this to try different maps
    env = environment_2d.Environment(10, 6, 5)
    pl.clf()
    env.plot()

    # Get a random query
    q = env.random_query()
    if q is not None:
        start_time = time.time()
        x_start, y_start, x_goal, y_goal = q
        start = (x_start, y_start)
        goal = (x_goal, y_goal)

        # Create and build PRM
        prm = PRM(env,num_samples,connection_radius, start, goal)
        prm.generate_samples()
        prm.build_roadmap()

        # Conduct A* search
        a_star = AStar(prm, start, goal)
        a_star.run()

        # Plot query points on top of path
        env.plot_query(x_start, y_start, x_goal, y_goal)
        end_time = time.time()
        print("Total time taken =", end_time - start_time)

    pl.ioff()
    pl.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PRM and A* Path Planning")
    parser.add_argument("--seed", type=int, default = 4, help = "Random seed")
    parser.add_argument("--num_samples", type=int, default=3000, help="Number of samples for PRM")
    parser.add_argument("--connection_radius", type=float, default=0.5, help="Connection radius for PRM")
    args = parser.parse_args()

    main(args.seed,args.num_samples, args.connection_radius)