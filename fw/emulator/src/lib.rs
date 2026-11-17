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

//! Runs the moteus firmware image on the host using an instruction
//! set simulator, in order to measure properties of the control ISR
//! without real hardware.

pub mod elf;
pub mod machine;
pub mod motor;
pub mod soc;
pub mod timing;

pub use machine::{IsrSample, Machine};
pub use soc::Board;
