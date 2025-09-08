# Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
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


from .device_info import DeviceAddress


class Command():
    destination = DeviceAddress(can_id=1)
    source = 0
    reply_required = False
    data = b''
    can_prefix = 0x0000  # a 13 bit CAN prefix
    expected_reply_size = 0

    # If True, then the following parameters are used directly instead
    # of being calculated from destination and source (i.e. for
    # non-moteus devices).
    raw = False
    arbitration_id = 0  # this is the name python-can gives

    # The channel can be specified to direct this to a particular
    # transport device.
    channel = None

    def parse(self, message):
        # By default, we just return the message as is.
        return message
