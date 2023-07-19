#!/usr/bin/env python
# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The "ISAAC - Integrated System for Autonomous and Adaptive Caretaking
# platform" software is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

# https://towardsdatascience.com/linear-programming-using-python-priyansh-22b5ee888fe0
import numpy as np
from pulp import *


class EMDGMM:
    def __init__(self, gmm1_weights, gmm2_weights):
        self.warehouse_supply = gmm1_weights  # Supply Matrix
        self.cust_demands = gmm2_weights  # Demand Matrix
        self.n_warehouses = gmm1_weights.size
        self.n_customers = gmm2_weights.size
        self.weight_sum1 = np.sum(self.warehouse_supply)
        self.weight_sum2 = np.sum(self.cust_demands)
        self.distances = None
        self.emd = None

    def get_distance(self, means1, means2):
        """Given two GMMs, generate a distance matrix between all cluster
        representatives (means) of GMM1 and GMM2. Output: K1 x K2 matrix"""

        distances = np.zeros((means1.shape[0], means2.shape[0]))
        for i, row1 in enumerate(means1):
            for j, row2 in enumerate(means2):
                distances[i][j] = np.linalg.norm(row1 - row2)
        self.distances = distances

    def calculate_emd(self):
        """Optimize the cost-distance (weight-distance) flow between the
        two GMMs and use the optimized distance as the EMD distance metric."""

        # Cost Matrix
        cost_matrix = self.distances

        # Initialize Model
        model = LpProblem("Supply-Demand-Problem", LpMinimize)

        # Define Variable Names
        variable_names = [
            str(i) + "_" + str(j)
            for j in range(1, self.n_customers + 1)
            for i in range(1, self.n_warehouses + 1)
        ]
        variable_names.sort()

        # Decision Variables
        DV_variables = LpVariable.matrix(
            "X", variable_names, cat="Continuous", lowBound=0
        )
        allocation = np.array(DV_variables).reshape(self.n_warehouses, self.n_customers)

        # Objective Function
        obj_func = lpSum(allocation * cost_matrix)
        model += obj_func

        # Constraints
        for i in range(self.n_warehouses):
            # print(lpSum(allocation[i][j] for j in range(self.n_customers)) <= warehouse_supply[i])
            model += lpSum(
                allocation[i][j] for j in range(self.n_customers)
            ) <= self.warehouse_supply[i], "Supply Constraints " + str(i)

        for j in range(self.n_customers):
            # print(lpSum(allocation[i][j] for i in range(self.n_warehouses)) >= cust_demands[j])
            model += lpSum(
                allocation[i][j] for i in range(self.n_warehouses)
            ) >= self.cust_demands[j], "Demand Constraints " + str(j)

        model.solve(GLPK_CMD(msg=0))
        status = LpStatus[model.status]
        # print(status)

        # print("Total Cost:", model.objective.value())
        # for v in model.variables():
        #    try:
        #        print(v.name, "=", v.value())
        #    except:
        #        print("error couldn't find value")

        # for i in range(self.n_warehouses):
        #    print("Warehouse ", str(i+1))
        #    print(lpSum(allocation[i][j].value() for j in range(self.n_customers)))

        total_flow = min(self.weight_sum1, self.weight_sum2)
        self.emd = model.objective.value() / total_flow
