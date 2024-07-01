import numpy as np
from bayes_opt import BayesianOptimization
import matplotlib.pyplot as plt
from scipy.optimize import NonlinearConstraint

def target_function(x):
    # Gardner is looking for the minimum, but this packages looks for maxima, thus the sign switch
    return np.cos(2*x) + np.sin(x)

def constraint_function(x):
    return np.cos(x)  - np.sin(x)

if __name__ == '__main__':
    constraint_limit = 0.5
    constraint = NonlinearConstraint(constraint_function, -np.inf, constraint_limit)
    # Bounded region of parameter space
    pbounds = {'x': (0, 6)}

    optimizer = BayesianOptimization(
        f=target_function,
        constraint=constraint,
        pbounds=pbounds,
        verbose=0,  # verbose = 1 prints only when a maximum is observed, verbose = 0 is silent
        random_state=1,
    )

    optimizer.maximize(
        init_points=2,
        n_iter=10,
    )

    print(optimizer.max)