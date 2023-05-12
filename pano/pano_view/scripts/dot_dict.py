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

"""
Convenience wrapper for dict so you can access members with dot notation.
"""


class DotDict(dict):
    """
    Convenience wrapper for dict so you can access members with dot notation.
    """

    def __getattr__(self, attr):
        if attr in self:
            return self[attr]
        return super(DotDict, self).__getattribute__(attr)
