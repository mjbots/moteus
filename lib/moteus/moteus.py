# Copyright 2020 Josh Pieper, jjp@pobox.com.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import enum

from . import multiplex


class Controller:
    """Operates a single moteus controller across some communication
    medium.

    Attributes:
      id: bus ID of the controller
      query_resolution: an instance of moteus.QueryResolution
      position_resolution: an instance of moteus.PositionResolution
      router: something modeling moteus.Router to send commands through
    """

    def __init__(self, id=1,
                 query_resolution=QueryResolution(),
                 position_resolution=PositionResolution(),
                 router=None):
        self.id = id
        self.query_resolution = query_resolution
        self.position_resolution = position_resolution
        self.router = router

    def write_position(self,
                       position=0.0,
                       velocity=0.0,
                       feedforward_torque=0.0,
                       kp_scale=1.0,
                       kd_scale=1.0,
                       maximum_torque=0.0,
                       stop_position=None,
                       watchdog_timeout=0.0,
                       query=False):
        result = moteus.Command()

        result.destination = self.id
        result.source = 0
        result.reply_required = query
