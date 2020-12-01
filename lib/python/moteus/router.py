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

class Router:
    """This dispatches multiplex commands and responses to multiple
    destinations depending upon id.
    """

    def __init__(self, destinations):
        """Args:

          destinations: A list of tuples, (target, [list, of, ids])
        """
        self._destinations = destinations

    async def cycle(self, commands):
        arguments = [
            (target, [x for x in commands if x.destination in these_ids])
            for target, these_ids in self._destinations
        ]
        arguments = [x for x in arguments if len(x[1])]
        tasks = [x[0].cycle(x[1]) for x in arguments]
        results = await asyncio.gather(*tasks)
        return sum(results, [])
