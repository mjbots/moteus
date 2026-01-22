// Copyright 2026 mjbots Robotic Systems, LLC.  info@mjbots.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Common SocketCAN types and helpers shared between sync and async implementations.
//!
//! This module contains the raw FFI declarations, kernel frame types, and
//! utility functions used by both [`super::socketcan`] and
//! [`super::async_socketcan`].

#[cfg(target_os = "linux")]
pub(crate) mod linux {
    use crate::error::{Error, Result};
    use std::io;
    use std::os::raw::{c_int, c_void};

    pub const AF_CAN: c_int = 29;
    pub const PF_CAN: c_int = AF_CAN;
    pub const SOCK_RAW: c_int = 3;
    pub const CAN_RAW: c_int = 1;
    pub const CAN_RAW_FD_FRAMES: c_int = 5;
    pub const SOL_CAN_RAW: c_int = 101;
    pub const F_GETFL: c_int = 3;
    pub const F_SETFL: c_int = 4;
    pub const O_NONBLOCK: c_int = 2048;
    pub const SIOCGIFINDEX: u64 = 0x8933;

    pub const CANFD_MTU: usize = 72;
    pub const CANFD_BRS: u8 = 0x01;

    /// Extended frame format flag in SocketCAN can_id.
    /// Must be set for CAN IDs that don't fit in 11 bits.
    pub const CAN_EFF_FLAG: u32 = 0x80000000;
    /// Mask for the 29-bit extended CAN ID.
    pub const CAN_EFF_MASK: u32 = 0x1FFFFFFF;

    #[repr(C)]
    pub struct Ifreq {
        pub ifr_name: [i8; 16],
        pub ifr_ifindex: c_int,
        pub _padding: [u8; 20],
    }

    /// Socket address structure for CAN.
    #[repr(C)]
    pub struct SockAddrCan {
        pub can_family: u16,
        pub can_ifindex: i32,
        pub rx_id: u32,
        pub tx_id: u32,
    }

    /// CAN FD frame structure for kernel interface.
    #[repr(C)]
    pub struct CanFdFrameRaw {
        pub can_id: u32,
        pub len: u8,
        pub flags: u8,
        pub _res0: u8,
        pub _res1: u8,
        pub data: [u8; 64],
    }

    impl Default for CanFdFrameRaw {
        fn default() -> Self {
            CanFdFrameRaw {
                can_id: 0,
                len: 0,
                flags: 0,
                _res0: 0,
                _res1: 0,
                data: [0; 64],
            }
        }
    }

    extern "C" {
        pub fn socket(domain: c_int, ty: c_int, protocol: c_int) -> c_int;
        pub fn bind(sockfd: c_int, addr: *const c_void, addrlen: u32) -> c_int;
        pub fn close(fd: c_int) -> c_int;
        pub fn read(fd: c_int, buf: *mut c_void, count: usize) -> isize;
        pub fn write(fd: c_int, buf: *const c_void, count: usize) -> isize;
        pub fn setsockopt(
            sockfd: c_int,
            level: c_int,
            optname: c_int,
            optval: *const c_void,
            optlen: u32,
        ) -> c_int;
        pub fn fcntl(fd: c_int, cmd: c_int, ...) -> c_int;
        pub fn ioctl(fd: c_int, request: u64, ...) -> c_int;
    }

    /// Gets the interface index for a given CAN interface name.
    pub fn get_ifindex(interface: &str) -> Result<i32> {
        let mut ifr = Ifreq {
            ifr_name: [0; 16],
            ifr_ifindex: 0,
            _padding: [0; 20],
        };

        let name_bytes = interface.as_bytes();
        let copy_len = std::cmp::min(name_bytes.len(), 15);
        for (i, &b) in name_bytes[..copy_len].iter().enumerate() {
            ifr.ifr_name[i] = b as i8;
        }

        let sock = unsafe { socket(AF_CAN, SOCK_RAW, CAN_RAW) };
        if sock < 0 {
            return Err(Error::Io(io::Error::last_os_error()));
        }

        let ret = unsafe { ioctl(sock, SIOCGIFINDEX, &mut ifr) };
        unsafe { close(sock) };

        if ret < 0 {
            return Err(Error::DeviceNotFound(interface.to_string()));
        }

        Ok(ifr.ifr_ifindex)
    }

    /// Converts a `CanFdFrame` to a raw kernel `CanFdFrameRaw`, with DLC padding.
    pub fn frame_to_raw(frame: &moteus_protocol::CanFdFrame, disable_brs: bool) -> CanFdFrameRaw {
        let mut padded = frame.clone();
        padded.pad_to_dlc();

        let mut raw = CanFdFrameRaw::default();
        raw.can_id = padded.arbitration_id;
        if padded.arbitration_id > 0x7FF {
            raw.can_id |= CAN_EFF_FLAG;
        }
        raw.len = padded.size;

        if padded.brs_enabled() && !disable_brs {
            raw.flags |= CANFD_BRS;
        }

        raw.data[..padded.size as usize]
            .copy_from_slice(&padded.data[..padded.size as usize]);
        raw
    }

    /// Converts a raw kernel `CanFdFrameRaw` to a `CanFdFrame`.
    pub fn frame_from_raw(raw: &CanFdFrameRaw) -> moteus_protocol::CanFdFrame {
        let mut frame = moteus_protocol::CanFdFrame::new();
        frame.arbitration_id = raw.can_id & CAN_EFF_MASK;
        frame.size = raw.len;
        frame.data[..raw.len as usize]
            .copy_from_slice(&raw.data[..raw.len as usize]);

        if raw.flags & CANFD_BRS != 0 {
            frame.set_brs(true);
        }
        frame.set_fdcan(true);

        frame
    }
}
