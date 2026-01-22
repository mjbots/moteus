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

//! Device discovery for fdcanusb and socketcan interfaces.
//!
//! This module provides functions to detect available CAN-FD interfaces
//! on the system.


/// Information about a detected fdcanusb device.
#[derive(Debug, Clone)]
pub struct FdcanusbInfo {
    /// Filesystem path to the serial device (e.g., "/dev/ttyACM0").
    pub path: String,
    /// USB serial number, if available.
    pub serial_number: Option<String>,
}

/// Information about a detected socketcan interface.
#[derive(Debug, Clone)]
pub struct SocketCanInfo {
    /// Network interface name (e.g., "can0").
    pub interface: String,
    /// If this interface is backed by an fdcanusb, its serial number.
    /// Used for deduplication with CDC fdcanusb devices.
    pub fdcanusb_serial: Option<String>,
}

/// All detected devices on the system.
#[derive(Debug, Clone, Default)]
pub struct DetectedDevices {
    /// Detected fdcanusb devices (CDC serial).
    pub fdcanusb: Vec<FdcanusbInfo>,
    /// Detected socketcan interfaces.
    pub socketcan: Vec<SocketCanInfo>,
}

/// USB Vendor/Product IDs for fdcanusb devices.
const FDCANUSB_IDS: &[(u16, u16)] = &[
    (0x0483, 0x5740), // Legacy fdcanusb (STM32 VCP)
    (0x1209, 0x2323), // New fdcanusb with gs_usb
];

/// Detect all fdcanusb devices on Linux.
///
/// On Linux, this first checks `/dev/serial/by-id/*fdcanusb*` for symlinks,
/// then falls back to USB VID/PID enumeration.
#[cfg(target_os = "linux")]
pub fn detect_fdcanusbs() -> Vec<FdcanusbInfo> {
    use std::fs;

    let mut result = Vec::new();

    // First, check /dev/serial/by-id for fdcanusb symlinks
    if let Ok(entries) = fs::read_dir("/dev/serial/by-id") {
        for entry in entries.filter_map(|e| e.ok()) {
            let file_name = entry.file_name();
            let name = file_name.to_string_lossy();

            if name.to_lowercase().contains("fdcanusb") {
                let path = entry.path();
                let resolved = fs::canonicalize(&path)
                    .unwrap_or_else(|_| path.clone());

                // Extract serial number from the symlink name
                // Format: usb-mjbots_fdcanusb_SERIALNUM-ifNN
                // where NN is 00 for CDC serial or 01 for gs_usb
                let serial_number = name
                    .rsplit('_')
                    .next()
                    .and_then(|s| {
                        s.strip_suffix("-if00")
                            .or_else(|| s.strip_suffix("-if01"))
                    })
                    .map(String::from);

                result.push(FdcanusbInfo {
                    path: resolved.to_string_lossy().to_string(),
                    serial_number,
                });
            }
        }
    }

    // If we found devices via /dev/serial/by-id, return those
    if !result.is_empty() {
        result.sort_by(|a, b| a.path.cmp(&b.path));
        return result;
    }

    // Fall back to USB VID/PID enumeration
    detect_fdcanusbs_by_vid_pid()
}

/// Detect fdcanusb devices on non-Linux platforms using USB VID/PID.
#[cfg(not(target_os = "linux"))]
pub fn detect_fdcanusbs() -> Vec<FdcanusbInfo> {
    detect_fdcanusbs_by_vid_pid()
}

/// Detect fdcanusb devices by enumerating serial ports and checking VID/PID.
fn detect_fdcanusbs_by_vid_pid() -> Vec<FdcanusbInfo> {
    #[cfg(feature = "serialport")]
    {
        use serialport::available_ports;

        let mut result = Vec::new();

        if let Ok(ports) = available_ports() {
            for port in ports {
                if let serialport::SerialPortType::UsbPort(usb_info) = port.port_type {
                    let is_fdcanusb = FDCANUSB_IDS
                        .iter()
                        .any(|(vid, pid)| usb_info.vid == *vid && usb_info.pid == *pid);

                    if is_fdcanusb {
                        result.push(FdcanusbInfo {
                            path: port.port_name,
                            serial_number: usb_info.serial_number,
                        });
                    }
                }
            }
        }

        result.sort_by(|a, b| a.path.cmp(&b.path));
        result
    }

    #[cfg(not(feature = "serialport"))]
    {
        Vec::new()
    }
}

/// Detect all socketcan interfaces on Linux.
///
/// Enumerates network interfaces in `/sys/class/net` that support CAN.
#[cfg(target_os = "linux")]
pub fn detect_socketcan_interfaces() -> Vec<SocketCanInfo> {
    use std::fs;

    let mut result = Vec::new();

    if let Ok(entries) = fs::read_dir("/sys/class/net") {
        for entry in entries.filter_map(|e| e.ok()) {
            let ifname = entry.file_name().to_string_lossy().to_string();

            // Check if this interface supports CAN by looking for the
            // protocol file or type file
            let type_path = entry.path().join("type");
            if let Ok(type_str) = fs::read_to_string(&type_path) {
                // ARPHRD_CAN = 280
                if type_str.trim() == "280" {
                    let fdcanusb_serial = detect_fdcanusb_serial_linux(&ifname);
                    result.push(SocketCanInfo {
                        interface: ifname,
                        fdcanusb_serial,
                    });
                }
            }
        }
    }

    result.sort_by(|a, b| a.interface.cmp(&b.interface));
    result
}

/// Detect if a socketcan interface is backed by an fdcanusb device.
///
/// Returns the USB serial number if it is, None otherwise.
#[cfg(target_os = "linux")]
fn detect_fdcanusb_serial_linux(ifname: &str) -> Option<String> {
    use std::fs;
    use std::path::PathBuf;

    // Navigate through sysfs to find the USB device
    let device_path = PathBuf::from(format!("/sys/class/net/{}/device", ifname));
    let device_path = fs::canonicalize(&device_path).ok()?;

    // Go up to find the USB device directory (contains idVendor, idProduct, serial)
    let mut current = device_path.as_path();

    // Walk up the directory tree looking for USB device info
    for _ in 0..10 {
        current = current.parent()?;

        let vid_path = current.join("idVendor");
        let pid_path = current.join("idProduct");
        let serial_path = current.join("serial");

        if vid_path.exists() && pid_path.exists() {
            let vid_str = fs::read_to_string(&vid_path).ok()?;
            let pid_str = fs::read_to_string(&pid_path).ok()?;

            let vid = u16::from_str_radix(vid_str.trim(), 16).ok()?;
            let pid = u16::from_str_radix(pid_str.trim(), 16).ok()?;

            if FDCANUSB_IDS.iter().any(|(v, p)| *v == vid && *p == pid) {
                return fs::read_to_string(&serial_path)
                    .ok()
                    .map(|s| s.trim().to_string());
            }

            // Found a USB device but it's not an fdcanusb
            return None;
        }
    }

    None
}

/// SocketCAN is not available on non-Linux platforms.
#[cfg(not(target_os = "linux"))]
pub fn detect_socketcan_interfaces() -> Vec<SocketCanInfo> {
    Vec::new()
}

/// Detect all available CAN-FD devices on the system.
///
/// This detects both fdcanusb (CDC serial) and socketcan interfaces.
pub fn detect_all_devices() -> DetectedDevices {
    DetectedDevices {
        fdcanusb: detect_fdcanusbs(),
        socketcan: detect_socketcan_interfaces(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_detect_all_devices_runs() {
        // This test just verifies the detection functions don't panic
        let devices = detect_all_devices();
        // On most systems without hardware, this will be empty
        let _ = devices;
    }

    #[test]
    fn test_fdcanusb_ids() {
        // Verify the VID/PID constants are correct
        assert!(FDCANUSB_IDS.contains(&(0x0483, 0x5740)));
        assert!(FDCANUSB_IDS.contains(&(0x1209, 0x2323)));
    }
}
