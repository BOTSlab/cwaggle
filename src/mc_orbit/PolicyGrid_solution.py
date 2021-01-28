#! /usr/bin/env python

"""
Used to compute policy evaluation and iteration within a gridworld where the
only movements possible are up, down and right.

STUDENTS: Your job is to fill in the code for the following functions below.
    evaluate_policy()
    policy_improvement()
    policy_iteration()
"""

import matplotlib.pyplot as plt
import numpy as np
from numpy import array, ones, zeros
from numpy.ma import masked_array
from random import randint

class PolicyGrid:

    def __init__(self, like_part_a=True, width=3, height=4, obstacle_prob=0):

        if like_part_a:
            # Initialize grid in the same form as A5, Part A
            self.size = (3, 4)
            self.occupancy = zeros(self.size)
            self.occupancy[0,2] = 1
            self.occupancy[1,1] = 1
            self.occupancy[1,2] = 1

            # Set policy 0 as in Part A
            self.policy_y_array = array([[1, 1, 0, -1],
                                         [1, 0, 0, -1],
                                         [1, 1, -1, -1]])
            self.policy_x_array = array([[0, 0, 0, 0],
                                         [0, 0, 0, 0],
                                         [0, 0, 0, 0]])

            # Set reward function.  As a slight variation from the notes, we'll
            # assign rewards directly to states.  This works fine for matching
            # with A5, Part A since the reward of +10 is applied for taking any
            # action in state c.
            self.rewards = array([[0, 0, 0, 10],
                                  [0, 0, 0, 0],
                                  [0, 0, 0, 0]])
        else:
            self.size = (height, width)
            # Initialize with with each cell having obstacle_prob probability
            # of being occupied.  0 = free.  1 = occupied
            self.occupancy = np.random.binomial(1, obstacle_prob, width*height).reshape(self.size)

            self.set_random_policy()

            # See comment above about reward function.  Here we just pick the
            # upper-right corner as the only rewarded state.
            self.rewards = zeros(self.size)
            self.rewards[0,-1] = 10

        # Discount factor for policy evaluation and iteration.
        self.gamma = 0.9
        self.values = zeros(self.size)

        #print("self.occupancy:")
        #print(self.occupancy)
        #print("self.policy_y_array:")
        #print(self.policy_y_array)
        #print("self.policy_x_array:")
        #print(self.policy_x_array)
        #print("self.rewards:")
        #print(self.rewards)

    def set_random_policy(self):
        self.policy_y_array = zeros(self.size, dtype=np.int)
        self.policy_x_array = zeros(self.size, dtype=np.int)
        for j in range(self.size[0]):
            for i in range(self.size[1]):
                if self.occupancy[j,i] == 1:
                    continue

                r = randint(0, 2)
                if r == 0:
                    # Up
                    self.policy_y_array[j,i] = -1
                    self.policy_x_array[j,i] = 0
                elif r == 1:
                    # Down
                    self.policy_y_array[j,i] = 1
                    self.policy_x_array[j,i] = 0
                else:
                    # Right
                    self.policy_y_array[j,i] = 0
                    self.policy_x_array[j,i] = 1

    def draw_policy(self, ax):
        # First mask out zero actions.
        mask = zeros(self.size)
        for j in range(self.size[0]):
            for i in range(self.size[1]):
                dy = self.policy_y_array[j,i]
                dx = self.policy_x_array[j,i]
                if dy == 0 and dx == 0:
                    mask[j,i] = 1

        masked_y_array = masked_array(self.policy_y_array, mask)
        masked_x_array = masked_array(self.policy_x_array, mask)
        ax.quiver(masked_x_array, -masked_y_array)

    def draw(self):
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, sharex=True)

        ax1.set_title("Occupancy", y=-10.1)
        ax1.matshow(self.occupancy)

        ax2.set_title("Reward")
        ax2.matshow(self.rewards)

        #ax2.matshow(zeros(self.size))
        #ax2.set_title("Policy")
        #self.draw_policy(ax2)

        ax3.set_title("Value Function + Policy")
        value_image = ax3.matshow(self.values)
        #fig.colorbar(value_image, ax=ax3)
        self.draw_policy(ax3)

        plt.show()

    def evaluate_policy(self, max_iterations=100, convergence_threshold=0.1):
        ################################################################
        # STUDENTS: Remove the following line and provide an implementation
        # for policy evaluation.  The idea is to evaluate the current policy
        # which exists in self.policy_y_array and self.policy_x_array.
        # pass

        # We could initialize the value function every time, but it is faster
        # if we just lose the values from the last evaluation (if any).
        #self.values = zeros(self.size)

        iterations = 0
        for iterations in range(max_iterations):
            new_values = zeros(self.size)

            max_diff = 0
            for j in range(self.size[0]):
                for i in range(self.size[1]):
                    new_value = self.expected_value(j, i, self.values, use_policy=True)
                    
                    # Check how much difference this makes
                    max_diff = max(max_diff, abs(new_value - self.values[j,i]))
                    #print("max_diff: {}".format(max_diff))

                    new_values[j, i] = new_value

            self.values = new_values

            if max_diff < convergence_threshold:
                break

            #print(self.values)

        print("policy evaluation iterations: {}".format(iterations))

    def policy_improvement(self):
        """Complete one step of policy improvement based on the existing value
        function in self.values (must be computed prior to this)."""

        ################################################################
        # STUDENTS: Remove the following line and provide an implementation
        # for policy improvement.
        # pass


        self.policy_modified = False
        
        # First we need to compute the state-action value, Q for the policy.
        # For simplicity, we choose to define three different Q's, one for each
        # action.  Another strategy would be to form a 3-dimensional structure
        # (height x width x action) or a 4-dimensional structure (height x
        # width x action_y x action_x).
        Q_up = zeros(self.size)
        Q_down = zeros(self.size)
        Q_right = zeros(self.size)

        for j in range(self.size[0]):
            for i in range(self.size[1]):
                if self.occupancy[j,i] == 1:
                    continue

                q_up = self.expected_value(j, i, self.values, use_policy=False, action_y=-1, action_x=0)
                q_down = self.expected_value(j, i, self.values, use_policy=False, action_y=1, action_x=0)
                q_right = self.expected_value(j, i, self.values, use_policy=False, action_y=0, action_x=1)

                # Adjust the policy to be the action with maximum q-value
                y_before = self.policy_y_array[j, i]
                x_before = self.policy_x_array[j, i]
                if q_up >= q_down and q_up >= q_right: # Up wins
                    self.policy_y_array[j, i] = -1
                    self.policy_x_array[j, i] = 0
                    if y_before != -1 and x_before != 0:
                        self.policy_modified = True
                elif q_down >= q_up and q_down >= q_right: # Down wins
                    self.policy_y_array[j, i] = 1
                    self.policy_x_array[j, i] = 0
                    if y_before != 1 and x_before != 0:
                        self.policy_modified = True
                else: # Right wins
                    self.policy_y_array[j, i] = 0
                    self.policy_x_array[j, i] = 1
                    if y_before != 0 and x_before != 1:
                        self.policy_modified = True

                # Actually, its not important to retain the Q arrays for the
                # sake of policy improvement.  But we do update them here for
                # debugging and analysis purposes.
                Q_up[j,i] = q_up
                Q_down[j,i] = q_down
                Q_right[j,i] = q_right
                
        #print(Q_up)
        #print(Q_down)
        #print(Q_right)

    def policy_iteration(self):
        ################################################################
        # STUDENTS: Remove the following line and provide an implementation
        # for policy improvement.
        # pass

        # Random policy already set in constructor.
        self.policy_modified = True
        while self.policy_modified:
            self.evaluate_policy()
            self.policy_improvement()

        # We may have states from which the goal is not reachable.  In this
        # case, the value function will be zero.  Set the policy to the null
        # action in such states to make it clear that there is no solution.
        for j in range(self.size[0]):
            for i in range(self.size[1]):
                if self.occupancy[j,i] == 0 and self.values[j,i] == 0:
                    self.policy_y_array[j, i] = 0
                    self.policy_x_array[j, i] = 0

    def expected_value(self, j, i, old_values, use_policy=False, action_y=None, action_x=None):
        """This is a bit over-complicated, but allows computing both the value
        function and the state-action value (Q) using the same code.  If
        'use_policy' is True then the current policy is used, otherwise the action is
        given by action_y and action_x."""

        # Occupied cells are not valid states.
        if self.occupancy[j,i] == 1:
            return 0

        # Initialize the value with the reward for this state.
        total = self.rewards[j, i]

        # A general implementation would loop through all possible
        # successor states, computing the products probabilities and
        # values.  Here we're assuming a deterministic policy so we
        # just have to consult the successor state for the current
        # policy.  

        # First we compute the successor state, but if this is either
        # off the grid or within an occupied cell, then it amounts
        # to a null movement action---which will still effect the
        # computation of the value function.
        if use_policy:
            succ_y = j + self.policy_y_array[j,i]
            succ_x = i + self.policy_x_array[j,i]
        else:
            succ_y = j + action_y
            succ_x = i + action_x

        if succ_y < 0 or succ_y >= self.size[0] or \
           succ_x < 0 or succ_x >= self.size[1] or \
           self.occupancy[succ_y, succ_x] == 1:
            # The policy's movement will cause a self-transition
            # (e.g. bumping into a wall and staying in the same
            # state).
            succ_y = j
            succ_x = i

        total += self.gamma * old_values[succ_y, succ_x]

        return total

    def set_policy_same(self, set_all_up=False, set_all_down=True, set_all_right=False):

        all_y = 0
        all_x = 0
        if set_all_up:
            all_y = -1
            all_x = 0
        elif set_all_down:
            all_y = 1
            all_x = 0
        elif set_all_right:
            all_y = 0
            all_x = 1

        for j in range(self.size[0]):
            for i in range(self.size[1]):
                if self.occupancy[j,i] == 1:
                    continue
                self.policy_y_array[j,i] = all_y
                self.policy_x_array[j,i] = all_x

# STUDENTS: To test that you get the same results as A5, Part A.
grid = PolicyGrid(like_part_a=True)
#grid.evaluate_policy(max_iterations=3)
#grid.policy_improvement()
# STUDENTS: Uncomment this when you are ready to test full policy iteration
#grid.policy_iteration()
#grid.draw()

# STUDENTS: Uncomment the following for a larger test environment.
grid = PolicyGrid(like_part_a=False, width=20, height=15, obstacle_prob=0.1)
grid.policy_iteration()
grid.draw()

# EXPERIMENTING (POST A5 POSTING)
#grid = PolicyGrid(like_part_a=False, width=20, height=15, obstacle_prob=0.05)
#grid.set_policy_same(set_all_right=True)
#grid.evaluate_policy()
#grid.draw()
