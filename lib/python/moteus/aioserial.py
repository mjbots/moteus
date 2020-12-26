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

'''This is a low-performance implementation of an asyncio serial
wrapper that is compatible with windows which does not allow
non-blocking serial connections.  It uses threads and busy-looping to
emulate non-blocking operations while still supporting cancellation.

'''

import os
import sys

if (sys.platform == 'win32' or
    os.environ.get('MOTEUS_FORCE_WIN32_SERIAL', False)):
    from moteus.win32_aioserial import *
else:
    from moteus.posix_aioserial import *
