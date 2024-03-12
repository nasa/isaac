# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The Astrobee platform is licensed under the Apache License, Version 2.0
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


class Model_Params:
    def __init__(self):
        self.hyperparams = {
            "learning_rate": 0.0001,
            "momentum": 0.9,
            "weight_decay": 0.0005,
            "burn_in": 1000,
            "lr_steps": 4,
            "batch_size": 2,
            "num_workers": 4,
        }
        self.optimizer = {"step_size": 3, "gamma": 0.1}

        self.checkpoint_interval = 10
