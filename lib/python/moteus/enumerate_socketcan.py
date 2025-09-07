# Copyright 2025 mjbots Robotic Systems, LLC.  info@mjbots.com
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

import os
import typing


ARPHRD_CAN = 280
SYS_NET = '/sys/class/net'
IFF_UP = 0x01

def enumerate_socketcan_up() -> typing.List[str]:
    """Return a list of socketcan interfaces that are marked as up."""

    # To find the interfaces, we'll just spleunk in /sys/class do our
    # best to verify that the interfaces match what we need.

    results = []
    for ifname in os.listdir(SYS_NET):
        path = os.path.join(SYS_NET, ifname)

        try:
            with open(os.path.join(path, "type")) as f:
                if int(f.read().strip()) != ARPHRD_CAN:
                    continue
        except RuntimeError:
            # For any error, just assume this is not a useful
            # interface.
            continue

        # Now check to see if it is marked as up.
        try:
            with open(os.path.join(path, "flags")) as f:
                flags = int(f.read().strip(), 16)
        except RuntimeError:
            # Missing file or malformed sys entry all means we won't
            # enumerate this one either.
            continue

        is_up = bool(flags & IFF_UP)

        if is_up:
            results.append(ifname)

    return sorted(results)


if __name__ == '__main__':
    print(enumerate_socketcan_up())
