#!/bin/bash

# Copyright (c) 2023, United States Government, as represented by the
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

# DESCRIPTION
# Regenerate outputs from *_config.yaml test cases.

set -eu -o pipefail

THIS_FILE=$(readlink -f "$0")
THIS_DIR=$(dirname "${THIS_FILE}")

SURVEY_PLANNER_DIR=$(readlink -f "${THIS_DIR}/../..")
PDDL_DIR="${SURVEY_PLANNER_DIR}/pddl"
DATA_DIR="${SURVEY_PLANNER_DIR}/data"
CASES_DIR="${DATA_DIR}/test_cases"
cd "${THIS_DIR}"

TEST_CASES=(completed1 completed2 proactive1 proactive2)

set -x
for i in "${TEST_CASES[@]}"; do
    { echo -e "\n=== $i ==="; } 2> /dev/null
    rosrun survey_planner problem_generator.py "--config=${DATA_DIR}/survey_static.yaml,${CASES_DIR}/${i}_config.yaml" "--output=${CASES_DIR}/${i}_problem.pddl"
    # Try to eliminate variation in problem.pddl due to embedding different local install paths in the output.
    perl -i -ple "s:${SURVEY_PLANNER_DIR}/::g;" "${CASES_DIR}/${i}_problem.pddl"
    rosrun survey_planner plan_survey.py "${PDDL_DIR}/domain_survey.pddl" "${CASES_DIR}/${i}_problem.pddl" > "${CASES_DIR}/${i}_plan.pddl"
    { echo "[plan_survey.py stdout redirected to ${CASES_DIR}/${i}_plan.pddl]"; } 2> /dev/null
    rosrun survey_planner plan_interpreter.py "--plan=${CASES_DIR}/${i}_plan.pddl" "--plot=${CASES_DIR}/${i}_plot.png" "--output=${CASES_DIR}/${i}_plan.yaml"
done
