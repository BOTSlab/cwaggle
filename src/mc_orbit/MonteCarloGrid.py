#! /usr/bin/env python

"""
NOT PART OF A5 IN 2020: Experimenting with Monte Carlo policy evaluation.
"""

import numpy as np
from numpy import array, ones, zeros
from random import randint
from PolicyGrid_solution import PolicyGrid

class MonteCarloGrid(PolicyGrid):

    def __init__(self, like_part_a=True, width=5, height=5, obstacle_prob=0):
        super().__init__(like_part_a, width, height, obstacle_prob)

        self.gamma = 0.9

    def evaluate_policy(self, every_visit=False, max_iterations=100):
        """Monte Carlo (MC) policy evaluation.  If every_visit == True then its
        Every-Visit MC.  Otherwise its First-Visit MC."""
        self.values = zeros(self.size)

        # Use these arrays to keep track of the number of times each state is
        # visited (N) and the total return (G).
        N = zeros(self.size)
        G = zeros(self.size)
        for k in range(max_iterations):
            (episode, returns) = self.mc_simulate_episode()
            state_visit_count = zeros(self.size)
            for t in range(len(episode)):
                state = episode[t][0]
                state_visit_count[state] += 1 # Needed for First-Visit
                if every_visit or state_visit_count[state] == 1:
                    total_return = returns[t]
                    N[state] = N[state] + 1
                    G[state] = G[state] + total_return
                    self.values[state] = G[state] / N[state]

        print(self.values)

    def mc_simulate_episode(self):

        # The length of the episode is the sum of the grid's height and width.
        episode_length = self.size[0] + self.size[1]

        # Pick a random start position that is not within an obstacle.
        position_okay = False
        while not position_okay:
            y = randint(0, self.size[0]-1)
            x = randint(0, self.size[1]-1)
            position_okay = self.occupancy[y, x] == 0
        state = (y, x)

        # Episode will be captured as a list of (state, action, reward) tuples
        episode = []

        for t in range(episode_length):
            # Consult the policy.
            action = (self.policy_y_array[y,x], self.policy_x_array[y,x])

            episode.append((state, action, self.rewards[state]))
        
            # Now follow the policy, but don't move through walls or the border.
            new_y = y + action[0]
            new_x = x + action[1]
            if not (new_y < 0 or new_y >= self.size[0] or \
                    new_x < 0 or new_x >= self.size[1] or \
                    self.occupancy[new_y, new_x] == 1):
                y = new_y
                x = new_x

            state = (y, x)

        # For each episode we can compute the returns at each time step.
        returns = []
        for t in range(episode_length):
            G = float(episode[t][2])
            for t_limit in range(t+1, episode_length):
                G += self.gamma**(t_limit - t) * episode[t_limit][2]
            returns.append(G)
        
        print("Episode:")
        for e in episode:
            print("\t{}".format(e))
        print("Returns:")
        for r in returns:
            print("\t{}".format(r))

        return (episode, returns)

grid = MonteCarloGrid(like_part_a=False)
grid.evaluate_policy(every_visit=False)
grid.draw()
