# Copyright 2020-2021 Josh Pieper, jjp@pobox.com.
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


class Command():
    destination = 1
    source = 0
    reply_required = False
    data = b''
    can_prefix = 0x0000  # a 13 bit CAN prefix

    # If True, then the following parameters are used directly instead
    # of being calculated from destination and source (i.e. for
    # non-moteus devices).
    raw = False
    arbitration_id = 0  # this is the name python-can gives
    bus = None  # Only valid for pi3hat

    def parse(self, message):
        # By default, we just return the message as is.
        return message
