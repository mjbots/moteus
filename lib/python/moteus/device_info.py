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

from dataclasses import dataclass
import typing
import uuid

from .transport_device import TransportDevice

@dataclass(frozen=True)
class DeviceAddress:
    """The minimal set of information necessary to communicate with a
    device in a system.  It may be just a CAN ID if that is unique, or
    it may be a UUID prefix.  It may also include a transport device,
    although that is not required."""
    can_id: typing.Optional[int] = None
    uuid: typing.Optional[bytes] = None
    transport_device: typing.Optional[TransportDevice] = None

    def __repr__(self):
        if self.can_id:
            return f'DeviceAddress(can_id={self.can_id}, td={self.transport_device})'
        uuid_bytes = self.uuid.hex() if self.uuid else None
        return f'DeviceAddress(uuid={uuid_bytes}, td={self.transport_device})'


@dataclass
class DeviceInfo:
    """This describes a device that was discovered on the CAN bus.  It
    includes the full available addressing information, as well as the
    minimal DeviceAddress structure necessary to address it in the
    current system."""

    can_id: int = 1
    uuid: typing.Optional[bytes] = None
    transport_device: typing.Optional[TransportDevice] = None
    address: typing.Optional[DeviceAddress] = None

    def __repr__(self):
        uuid_bytes = uuid.UUID(bytes=self.uuid) if self.uuid else None
        return f'DeviceInfo(can_id={self.can_id}, uuid={uuid_bytes}, td={self.transport_device})'

    def _cmp_key(self):
        return (self.can_id, self.uuid or b'')

    def __lt__(self, other):
        if not isinstance(other, DeviceInfo):
            return NotImplemented

        return self._cmp_key() < other._cmp_key()
