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

//! A minimal reader for 32 bit little endian ELF firmware images.

const PT_LOAD: u32 = 1;
const SHT_SYMTAB: u32 = 2;
const STT_FUNC: u8 = 2;

/// A loadable region of the image.
pub struct Segment {
    pub paddr: u32,
    pub data: Vec<u8>,
}

/// A function symbol.
pub struct Function {
    pub addr: u32,
    pub size: u32,
    pub name: String,
}

pub struct Elf {
    pub segments: Vec<Segment>,
    /// Sorted by address.
    pub functions: Vec<Function>,
}

fn u16_at(d: &[u8], o: usize) -> Result<usize, String> {
    let b = d.get(o..o + 2).ok_or("truncated ELF")?;
    Ok(u16::from_le_bytes([b[0], b[1]]) as usize)
}

fn u32_at(d: &[u8], o: usize) -> Result<u32, String> {
    let b = d.get(o..o + 4).ok_or("truncated ELF")?;
    Ok(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
}

fn slice(d: &[u8], offset: u32, size: u32) -> Result<&[u8], String> {
    let start = offset as usize;
    d.get(start..start + size as usize)
        .ok_or_else(|| "truncated ELF".to_string())
}

fn cstr(d: &[u8], o: usize) -> String {
    let tail = d.get(o..).unwrap_or(&[]);
    let end = tail.iter().position(|&c| c == 0).unwrap_or(tail.len());
    String::from_utf8_lossy(&tail[..end]).into_owned()
}

impl Elf {
    /// An image with nothing in it.
    pub fn empty() -> Self {
        Self {
            segments: Vec::new(),
            functions: Vec::new(),
        }
    }

    pub fn parse(d: &[u8]) -> Result<Self, String> {
        if d.len() < 0x34 || &d[0..4] != b"\x7fELF" || d[4] != 1 || d[5] != 1 {
            return Err("not a 32 bit little endian ELF file".to_string());
        }
        let phoff = u32_at(d, 0x1c)? as usize;
        let shoff = u32_at(d, 0x20)? as usize;
        let phentsize = u16_at(d, 0x2a)?;
        let phnum = u16_at(d, 0x2c)?;
        let shentsize = u16_at(d, 0x2e)?;
        let shnum = u16_at(d, 0x30)?;

        let mut segments = Vec::new();
        for i in 0..phnum {
            let ph = phoff + i * phentsize;
            let filesz = u32_at(d, ph + 16)?;
            if u32_at(d, ph)? != PT_LOAD || filesz == 0 {
                continue;
            }
            segments.push(Segment {
                paddr: u32_at(d, ph + 12)?,
                data: slice(d, u32_at(d, ph + 4)?, filesz)?.to_vec(),
            });
        }

        let mut functions = Vec::new();
        for i in 0..shnum {
            let sh = shoff + i * shentsize;
            if u32_at(d, sh + 4)? != SHT_SYMTAB {
                continue;
            }
            let symtab = slice(d, u32_at(d, sh + 16)?, u32_at(d, sh + 20)?)?;
            let strsh = shoff + u32_at(d, sh + 24)? as usize * shentsize;
            let strtab = slice(d, u32_at(d, strsh + 16)?, u32_at(d, strsh + 20)?)?;
            for sym in symtab.chunks_exact(16) {
                if sym[12] & 0xf != STT_FUNC {
                    continue;
                }
                functions.push(Function {
                    // Strip the thumb bit.
                    addr: u32_at(sym, 4)? & !1,
                    size: u32_at(sym, 8)?,
                    name: cstr(strtab, u32_at(sym, 0)? as usize),
                });
            }
        }
        functions.sort_by_key(|f| f.addr);

        Ok(Self {
            segments,
            functions,
        })
    }

    /// The address of the named function, if present.
    pub fn function(&self, name: &str) -> Option<u32> {
        self.functions
            .iter()
            .find(|f| f.name == name)
            .map(|f| f.addr)
    }

    /// The index into `functions` of the one containing `pc`.
    pub fn function_index(&self, pc: u32) -> Option<usize> {
        let i = self.functions.partition_point(|f| f.addr <= pc);
        let f = self.functions.get(i.checked_sub(1)?)?;
        (pc < f.addr + f.size.max(2)).then_some(i - 1)
    }

    /// Describe an address as "function+offset".
    pub fn symbolize(&self, pc: u32) -> String {
        let i = self.functions.partition_point(|f| f.addr <= pc);
        if i > 0 {
            let f = &self.functions[i - 1];
            if pc < f.addr + f.size.max(2) {
                return format!("{}+{:#x}", f.name, pc - f.addr);
            }
        }
        format!("{pc:#010x}")
    }
}
