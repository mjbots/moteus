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

import asyncio

class Transport:
    """This is an object which can dispatch commands to one or more
    controllers across potentially multiple CAN-FD busses.
    """

    async def cycle(self, commands):
        '''The 'cycle' method accepts a list of 'commands'.  Each command must
         model moteus.Command.

        It should return a list of results from Command.parse()
        '''
        return []
