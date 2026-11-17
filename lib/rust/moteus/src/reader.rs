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

//! Reader for the self-describing binary telemetry format.
//!
//! A moteus controller describes each of its telemetry channels with a
//! binary *schema*, and then sends the contents of that channel as
//! binary *data* which can only be interpreted using that schema.  On
//! the diagnostic stream, `tel schema NAME` returns the former, and
//! `tel get NAME` the latter once `tel fmt NAME 0` has selected binary
//! output.  Log files written by `tview` hold the same two things.
//!
//! This module mirrors the Python `moteus.reader` module.  A schema is
//! parsed into a [`Type`] with [`Type::from_binary`], and that type
//! then turns any number of data records into [`Value`]s with
//! [`Type::read`].
//!
//! # Example
//!
//! ```
//! use moteus::reader::Type;
//!
//! # fn main() -> Result<(), moteus::reader::ParseError> {
//! // An object with the fields "mode", an enumeration, and
//! // "position", a 32 bit float.
//! let schema = [
//!     16, 0,
//!     0, 4, b'm', b'o', b'd', b'e', 0,
//!     17, 6, 2,
//!     0, 7, b's', b't', b'o', b'p', b'p', b'e', b'd',
//!     10, 8, b'p', b'o', b's', b'i', b't', b'i', b'o', b'n',
//!     0,
//!     0, 8, b'p', b'o', b's', b'i', b't', b'i', b'o', b'n', 0, 7, 0,
//!     0, 0, 0, 0, 0,
//! ];
//! let servo_stats = Type::from_binary(&schema)?;
//!
//! let value = servo_stats.read(&[10, 0x00, 0x00, 0x20, 0x40])?;
//! assert_eq!(value.get("position").and_then(|x| x.as_f64()), Some(2.5));
//!
//! let mode = value.get("mode").and_then(|x| x.as_enum()).unwrap();
//! assert_eq!(mode.value, 10);
//! assert_eq!(mode.name.as_deref(), Some("position"));
//! # Ok(())
//! # }
//! ```

use std::collections::BTreeMap;
use std::fmt;

/// How deeply types may be nested within a schema.
const MAX_DEPTH: usize = 64;

/// The reason a schema or data could not be read.
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ParseError {
    /// The schema or data ended before what it describes did.
    UnexpectedEof,
    /// A variable length integer does not fit in 64 bits.
    InvalidVaruint,
    /// A string is not UTF-8.
    InvalidUtf8,
    /// A schema uses a type identifier which is not known.
    UnknownType(u64),
    /// A fixed size integer is not 1, 2, 4 or 8 bytes.
    InvalidFixedSize(u8),
    /// An enumeration is of something other than an integer which
    /// fits in an `i64`.
    InvalidEnum,
    /// A union selects an alternative which its schema does not have.
    InvalidUnionIndex(u64),
    /// Data was to be read as the type which only marks the end of a
    /// list in a schema.
    FinalType,
    /// A schema nests types too deeply.
    TooDeep,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ParseError::UnexpectedEof => write!(f, "unexpected end of data"),
            ParseError::InvalidVaruint => write!(f, "invalid varuint"),
            ParseError::InvalidUtf8 => write!(f, "string is not UTF-8"),
            ParseError::UnknownType(index) => write!(f, "unknown type: {index}"),
            ParseError::InvalidFixedSize(size) => write!(f, "invalid fixed integer size: {size}"),
            ParseError::InvalidEnum => write!(f, "enumeration is not of a 64 bit integer"),
            ParseError::InvalidUnionIndex(index) => write!(f, "invalid union index: {index}"),
            ParseError::FinalType => write!(f, "data of the final type"),
            ParseError::TooDeep => write!(f, "schema is nested too deeply"),
        }
    }
}

impl std::error::Error for ParseError {}

impl From<ParseError> for crate::Error {
    fn from(err: ParseError) -> Self {
        crate::Error::Parse(err)
    }
}

/// Result type for reading schemas and data.
pub type Result<T> = std::result::Result<T, ParseError>;

/// A cursor over the bytes of a schema or of data, from which the
/// primitive encodings can be read.
///
/// [`Type::from_binary`] and [`Type::read`] are all that is needed to
/// read one schema or one record.  A `Stream` is for when several
/// follow one another, or are embedded in something else.
#[derive(Debug, Clone)]
pub struct Stream<'a> {
    data: &'a [u8],
}

macro_rules! read_fixed {
    ($(#[$doc:meta] $name:ident -> $type:ty;)*) => {
        $(
            #[$doc]
            pub fn $name(&mut self) -> Result<$type> {
                let bytes = self.read_exact(std::mem::size_of::<$type>())?;
                Ok(<$type>::from_le_bytes(bytes.try_into().unwrap()))
            }
        )*
    };
}

impl<'a> Stream<'a> {
    /// Create a stream which reads from the start of `data`.
    pub fn new(data: &'a [u8]) -> Self {
        Self { data }
    }

    /// The bytes which have not yet been read.
    pub fn remaining(&self) -> &'a [u8] {
        self.data
    }

    /// Whether everything has been read.
    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    /// Read exactly `size` bytes.
    pub fn read_exact(&mut self, size: usize) -> Result<&'a [u8]> {
        if size > self.data.len() {
            return Err(ParseError::UnexpectedEof);
        }
        let (result, rest) = self.data.split_at(size);
        self.data = rest;
        Ok(result)
    }

    /// Skip `size` bytes.
    pub fn ignore(&mut self, size: usize) -> Result<()> {
        self.read_exact(size).map(|_| ())
    }

    read_fixed! {
        /// Read an unsigned 8 bit integer.
        read_u8 -> u8;
        /// Read a little endian unsigned 16 bit integer.
        read_u16 -> u16;
        /// Read a little endian unsigned 32 bit integer.
        read_u32 -> u32;
        /// Read a little endian unsigned 64 bit integer.
        read_u64 -> u64;
        /// Read a signed 8 bit integer.
        read_i8 -> i8;
        /// Read a little endian signed 16 bit integer.
        read_i16 -> i16;
        /// Read a little endian signed 32 bit integer.
        read_i32 -> i32;
        /// Read a little endian signed 64 bit integer.
        read_i64 -> i64;
        /// Read a little endian 32 bit float.
        read_f32 -> f32;
        /// Read a little endian 64 bit float.
        read_f64 -> f64;
    }

    /// Read a variable length unsigned integer: 7 bits per byte, least
    /// significant first, with the top bit of each byte set when
    /// another follows.
    pub fn read_varuint(&mut self) -> Result<u64> {
        let mut result = 0u64;
        for shift in (0..64).step_by(7) {
            let byte = self.read_u8()?;
            let bits = u64::from(byte & 0x7f);
            if shift == 63 && bits > 1 {
                return Err(ParseError::InvalidVaruint);
            }
            result |= bits << shift;
            if byte < 0x80 {
                return Ok(result);
            }
        }
        Err(ParseError::InvalidVaruint)
    }

    /// Read a variable length signed integer, which is a varuint with
    /// the sign in its least significant bit.
    pub fn read_varint(&mut self) -> Result<i64> {
        let encoded = self.read_varuint()?;
        Ok((encoded >> 1) as i64 ^ -((encoded & 1) as i64))
    }

    /// Read a varuint length followed by that many bytes.
    pub fn read_bytes(&mut self) -> Result<&'a [u8]> {
        let size = self.read_count()?;
        self.read_exact(size)
    }

    /// Read a varuint length followed by that many bytes of UTF-8.
    pub fn read_string(&mut self) -> Result<&'a str> {
        std::str::from_utf8(self.read_bytes()?).map_err(|_| ParseError::InvalidUtf8)
    }

    /// Read a varuint which says how many of something follow.  Every
    /// such thing which moteus sends takes at least a byte, so more
    /// than there are bytes left is treated as the data ending early
    /// instead of attempting to read them.
    fn read_count(&mut self) -> Result<usize> {
        let count = self.read_varuint()?;
        self.check_count(count)
    }

    fn check_count(&self, count: u64) -> Result<usize> {
        match usize::try_from(count) {
            Ok(count) if count <= self.data.len() => Ok(count),
            _ => Err(ParseError::UnexpectedEof),
        }
    }
}

/// A named member of an [`ObjectType`].
#[derive(Debug, Clone, PartialEq)]
pub struct Field {
    /// Flags, none of which are currently defined.
    pub flags: u64,
    /// The name of the field.
    pub name: String,
    /// Names which the field has previously had.
    pub aliases: Vec<String>,
    /// The type of the field's value.
    pub field_type: Type,
    /// The value the field has when it is not otherwise set, if the
    /// schema gives one.
    pub default_value: Option<Value>,
}

/// The schema of a structure: an ordered list of named fields.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ObjectType {
    /// Flags, none of which are currently defined.
    pub flags: u64,
    /// The fields, in the order in which their values are sent.
    pub fields: Vec<Field>,
}

impl ObjectType {
    /// The field with the given name.
    pub fn field(&self, name: &str) -> Option<&Field> {
        self.fields.iter().find(|field| field.name == name)
    }
}

/// The schema of an enumeration: an integer type, and names for some
/// of its values.
#[derive(Debug, Clone, PartialEq)]
pub struct EnumType {
    /// The type which values are sent as.
    pub underlying: Box<Type>,
    /// Each named value, in the order the schema gives them.
    pub items: Vec<(i64, String)>,
}

impl EnumType {
    /// The name of `value`, if it has one.
    pub fn name(&self, value: i64) -> Option<&str> {
        self.items
            .iter()
            .find(|item| item.0 == value)
            .map(|item| item.1.as_str())
    }

    /// The value which has the given name.
    pub fn value(&self, name: &str) -> Option<i64> {
        self.items
            .iter()
            .find(|item| item.1 == name)
            .map(|item| item.0)
    }
}

/// A parsed schema, which describes how to read data.
///
/// The variants correspond to the classes of the Python
/// `moteus.reader` module: `Type::FixedInt` to `FixedIntType`, and so
/// on.
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum Type {
    /// Marks the end of a list of types or fields in a schema.  There
    /// is no data of this type.
    Final,
    /// Nothing, which takes no space.
    Null,
    /// A boolean, as one byte.
    Boolean,
    /// A little endian signed integer of 1, 2, 4 or 8 bytes.
    FixedInt {
        /// The size in bytes.
        size: u8,
    },
    /// A little endian unsigned integer of 1, 2, 4 or 8 bytes.
    FixedUInt {
        /// The size in bytes.
        size: u8,
    },
    /// A variable length signed integer.
    Varint,
    /// A variable length unsigned integer.
    Varuint,
    /// A 32 bit float.
    Float32,
    /// A 64 bit float.
    Float64,
    /// A length, and that many bytes.
    Bytes,
    /// A length, and that many bytes of UTF-8.
    String,
    /// A structure.
    Object(ObjectType),
    /// An enumeration.
    Enum(EnumType),
    /// A count, and that many of the element type.
    Array(Box<Type>),
    /// A number of the element type which is part of the schema.
    FixedArray {
        /// How many elements there are.
        size: u64,
        /// The type of each.
        element: Box<Type>,
    },
    /// A count, and that many pairs of a string and the element type.
    Map(Box<Type>),
    /// An index into the alternatives, and data of that type.
    Union(Vec<Type>),
    /// Microseconds since the Unix epoch, as a signed 64 bit integer.
    Timestamp,
    /// Microseconds, as a signed 64 bit integer.
    Duration,
}

impl Type {
    /// Parse a binary schema.
    ///
    /// Anything which follows the schema in `schema` is ignored.
    pub fn from_binary(schema: &[u8]) -> Result<Type> {
        Self::from_stream(&mut Stream::new(schema))
    }

    /// Parse a binary schema from a stream, leaving the stream after
    /// it.
    pub fn from_stream(stream: &mut Stream<'_>) -> Result<Type> {
        Self::parse(stream, 0)
    }

    fn parse(stream: &mut Stream<'_>, depth: usize) -> Result<Type> {
        if depth > MAX_DEPTH {
            return Err(ParseError::TooDeep);
        }
        let fixed_size = |stream: &mut Stream<'_>| match stream.read_u8()? {
            size @ (1 | 2 | 4 | 8) => Ok(size),
            size => Err(ParseError::InvalidFixedSize(size)),
        };
        Ok(match stream.read_varuint()? {
            0 => Type::Final,
            1 => Type::Null,
            2 => Type::Boolean,
            3 => Type::FixedInt {
                size: fixed_size(stream)?,
            },
            4 => Type::FixedUInt {
                size: fixed_size(stream)?,
            },
            5 => Type::Varint,
            6 => Type::Varuint,
            7 => Type::Float32,
            8 => Type::Float64,
            9 => Type::Bytes,
            10 => Type::String,
            16 => {
                let flags = stream.read_varuint()?;
                let mut fields = Vec::new();
                loop {
                    let flags = stream.read_varuint()?;
                    let name = stream.read_string()?.to_owned();
                    let mut aliases = Vec::new();
                    for _ in 0..stream.read_count()? {
                        aliases.push(stream.read_string()?.to_owned());
                    }
                    let field_type = Self::parse(stream, depth + 1)?;
                    let default_value = match stream.read_u8()? {
                        0 => None,
                        _ => Some(field_type.read_stream(stream)?),
                    };
                    if field_type == Type::Final {
                        break;
                    }
                    fields.push(Field {
                        flags,
                        name,
                        aliases,
                        field_type,
                        default_value,
                    });
                }
                Type::Object(ObjectType { flags, fields })
            }
            17 => {
                let underlying = Self::parse(stream, depth + 1)?;
                let mut items = Vec::new();
                for _ in 0..stream.read_count()? {
                    let value = underlying
                        .read_stream(stream)?
                        .as_i64()
                        .ok_or(ParseError::InvalidEnum)?;
                    items.push((value, stream.read_string()?.to_owned()));
                }
                Type::Enum(EnumType {
                    underlying: Box::new(underlying),
                    items,
                })
            }
            18 => Type::Array(Box::new(Self::parse(stream, depth + 1)?)),
            19 => Type::FixedArray {
                size: stream.read_varuint()?,
                element: Box::new(Self::parse(stream, depth + 1)?),
            },
            20 => Type::Map(Box::new(Self::parse(stream, depth + 1)?)),
            21 => {
                let mut alternatives = Vec::new();
                loop {
                    let alternative = Self::parse(stream, depth + 1)?;
                    if alternative == Type::Final {
                        break;
                    }
                    alternatives.push(alternative);
                }
                Type::Union(alternatives)
            }
            22 => Type::Timestamp,
            23 => Type::Duration,
            other => return Err(ParseError::UnknownType(other)),
        })
    }

    /// Read one record of this type.
    ///
    /// Anything which follows the record in `data` is ignored.
    pub fn read(&self, data: &[u8]) -> Result<Value> {
        self.read_stream(&mut Stream::new(data))
    }

    /// Read one record of this type from a stream, leaving the stream
    /// after it.
    pub fn read_stream(&self, stream: &mut Stream<'_>) -> Result<Value> {
        Ok(match self {
            Type::Final => return Err(ParseError::FinalType),
            Type::Null => Value::Null,
            Type::Boolean => Value::Boolean(stream.read_u8()? != 0),
            Type::FixedInt { size } => Value::Int(match size {
                1 => stream.read_i8()?.into(),
                2 => stream.read_i16()?.into(),
                4 => stream.read_i32()?.into(),
                8 => stream.read_i64()?,
                _ => return Err(ParseError::InvalidFixedSize(*size)),
            }),
            Type::FixedUInt { size } => Value::UInt(match size {
                1 => stream.read_u8()?.into(),
                2 => stream.read_u16()?.into(),
                4 => stream.read_u32()?.into(),
                8 => stream.read_u64()?,
                _ => return Err(ParseError::InvalidFixedSize(*size)),
            }),
            Type::Varint => Value::Int(stream.read_varint()?),
            Type::Varuint => Value::UInt(stream.read_varuint()?),
            Type::Float32 => Value::Float(stream.read_f32()?.into()),
            Type::Float64 => Value::Float(stream.read_f64()?),
            Type::Bytes => Value::Bytes(stream.read_bytes()?.to_vec()),
            Type::String => Value::String(stream.read_string()?.to_owned()),
            Type::Object(object) => {
                let mut fields = Vec::with_capacity(object.fields.len());
                for field in &object.fields {
                    let value = field.field_type.read_stream(stream)?;
                    fields.push((field.name.clone(), value));
                }
                Value::Object(Object { fields })
            }
            Type::Enum(enum_type) => {
                let value = enum_type
                    .underlying
                    .read_stream(stream)?
                    .as_i64()
                    .ok_or(ParseError::InvalidEnum)?;
                Value::Enum(EnumValue {
                    value,
                    name: enum_type.name(value).map(str::to_owned),
                })
            }
            Type::Array(element) => {
                let count = stream.read_count()?;
                Value::Array(Self::read_elements(element, count, stream)?)
            }
            Type::FixedArray { size, element } => {
                let count = stream.check_count(*size)?;
                Value::Array(Self::read_elements(element, count, stream)?)
            }
            Type::Map(element) => {
                let mut items = BTreeMap::new();
                for _ in 0..stream.read_count()? {
                    let key = stream.read_string()?.to_owned();
                    items.insert(key, element.read_stream(stream)?);
                }
                Value::Map(items)
            }
            Type::Union(alternatives) => {
                let index = stream.read_varuint()?;
                usize::try_from(index)
                    .ok()
                    .and_then(|index| alternatives.get(index))
                    .ok_or(ParseError::InvalidUnionIndex(index))?
                    .read_stream(stream)?
            }
            Type::Timestamp => Value::Timestamp(stream.read_i64()?),
            Type::Duration => Value::Duration(stream.read_i64()?),
        })
    }

    fn read_elements(element: &Type, count: usize, stream: &mut Stream<'_>) -> Result<Vec<Value>> {
        (0..count).map(|_| element.read_stream(stream)).collect()
    }
}

/// The value of an enumeration.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EnumValue {
    /// The numeric value.
    pub value: i64,
    /// The name the schema gives that value, if it gives one.
    pub name: Option<String>,
}

impl fmt::Display for EnumValue {
    /// The name, or the number when there is no name for it.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match &self.name {
            Some(name) => f.write_str(name),
            None => write!(f, "{}", self.value),
        }
    }
}

/// The value of a structure: its fields, in the order of the schema.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Object {
    fields: Vec<(String, Value)>,
}

impl Object {
    /// The value of the field with the given name.
    pub fn get(&self, name: &str) -> Option<&Value> {
        self.fields
            .iter()
            .find(|field| field.0 == name)
            .map(|field| &field.1)
    }

    /// The name and value of each field, in order.
    pub fn iter(&self) -> impl ExactSizeIterator<Item = (&str, &Value)> + '_ {
        self.fields
            .iter()
            .map(|(name, value)| (name.as_str(), value))
    }

    /// How many fields there are.
    pub fn len(&self) -> usize {
        self.fields.len()
    }

    /// Whether there are no fields.
    pub fn is_empty(&self) -> bool {
        self.fields.is_empty()
    }
}

impl FromIterator<(String, Value)> for Object {
    fn from_iter<I: IntoIterator<Item = (String, Value)>>(iter: I) -> Self {
        Self {
            fields: iter.into_iter().collect(),
        }
    }
}

impl IntoIterator for Object {
    type Item = (String, Value);
    type IntoIter = std::vec::IntoIter<(String, Value)>;

    fn into_iter(self) -> Self::IntoIter {
        self.fields.into_iter()
    }
}

/// Data, as read according to a [`Type`].
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    /// Nothing.
    Null,
    /// A boolean.
    Boolean(bool),
    /// A signed integer of any encoding.
    Int(i64),
    /// An unsigned integer of any encoding.
    UInt(u64),
    /// A float of either size.
    Float(f64),
    /// Bytes.
    Bytes(Vec<u8>),
    /// A string.
    String(String),
    /// A structure.
    Object(Object),
    /// An enumeration.
    Enum(EnumValue),
    /// An array, whether or not of a fixed size.
    Array(Vec<Value>),
    /// A map from strings.
    Map(BTreeMap<String, Value>),
    /// Microseconds since the Unix epoch.
    Timestamp(i64),
    /// Microseconds.
    Duration(i64),
}

impl Value {
    /// Whether this is [`Value::Null`].
    pub fn is_null(&self) -> bool {
        matches!(self, Value::Null)
    }

    /// The value of a boolean.
    pub fn as_bool(&self) -> Option<bool> {
        match *self {
            Value::Boolean(value) => Some(value),
            _ => None,
        }
    }

    /// The value of an integer or an enumeration, if it fits.
    pub fn as_i64(&self) -> Option<i64> {
        match *self {
            Value::Int(value) => Some(value),
            Value::UInt(value) => i64::try_from(value).ok(),
            Value::Enum(EnumValue { value, .. }) => Some(value),
            _ => None,
        }
    }

    /// The value of an integer or an enumeration, if it is not
    /// negative.
    pub fn as_u64(&self) -> Option<u64> {
        match *self {
            Value::Int(value) => u64::try_from(value).ok(),
            Value::UInt(value) => Some(value),
            Value::Enum(EnumValue { value, .. }) => u64::try_from(value).ok(),
            _ => None,
        }
    }

    /// The value of a float, an integer or an enumeration.
    pub fn as_f64(&self) -> Option<f64> {
        match *self {
            Value::Int(value) => Some(value as f64),
            Value::UInt(value) => Some(value as f64),
            Value::Float(value) => Some(value),
            Value::Enum(EnumValue { value, .. }) => Some(value as f64),
            _ => None,
        }
    }

    /// A timestamp or a duration, in seconds, which is what the Python
    /// reader gives for them.
    pub fn as_seconds(&self) -> Option<f64> {
        match *self {
            Value::Timestamp(us) | Value::Duration(us) => Some(us as f64 / 1e6),
            _ => None,
        }
    }

    /// The contents of a string.
    pub fn as_str(&self) -> Option<&str> {
        match self {
            Value::String(value) => Some(value),
            _ => None,
        }
    }

    /// The contents of bytes.
    pub fn as_bytes(&self) -> Option<&[u8]> {
        match self {
            Value::Bytes(value) => Some(value),
            _ => None,
        }
    }

    /// An enumeration, with the name of its value.
    pub fn as_enum(&self) -> Option<&EnumValue> {
        match self {
            Value::Enum(value) => Some(value),
            _ => None,
        }
    }

    /// The elements of an array.
    pub fn as_array(&self) -> Option<&[Value]> {
        match self {
            Value::Array(value) => Some(value),
            _ => None,
        }
    }

    /// The fields of a structure.
    pub fn as_object(&self) -> Option<&Object> {
        match self {
            Value::Object(value) => Some(value),
            _ => None,
        }
    }

    /// The contents of a map.
    pub fn as_map(&self) -> Option<&BTreeMap<String, Value>> {
        match self {
            Value::Map(value) => Some(value),
            _ => None,
        }
    }

    /// A member of this value: the named field of a structure, the
    /// entry of a map with that key, or the element of an array at
    /// that decimal index.
    pub fn get(&self, name: &str) -> Option<&Value> {
        match self {
            Value::Object(object) => object.get(name),
            Value::Map(map) => map.get(name),
            Value::Array(array) => array.get(name.parse::<usize>().ok()?),
            _ => None,
        }
    }

    /// A member any number of levels down, named as `tview` and the
    /// controller's text output do: each level as for [`Value::get`],
    /// separated by periods.
    ///
    /// ```
    /// # use moteus::reader::Type;
    /// // An object "fw", holding a float "id_A".
    /// let schema = [
    ///     16, 0,
    ///     0, 2, b'f', b'w', 0,
    ///     16, 0, 0, 4, b'i', b'd', b'_', b'A', 0, 7, 0, 0, 0, 0, 0, 0,
    ///     0,
    ///     0, 0, 0, 0, 0,
    /// ];
    /// let value = Type::from_binary(&schema)
    ///     .unwrap()
    ///     .read(&(-1.5f32).to_le_bytes())
    ///     .unwrap();
    /// assert_eq!(value.get_path("fw.id_A").unwrap().as_f64(), Some(-1.5));
    /// ```
    pub fn get_path(&self, path: &str) -> Option<&Value> {
        path.split('.')
            .try_fold(self, |value, name| value.get(name))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn read(schema: &[u8], data: &[u8]) -> (Type, Value) {
        let parsed = Type::from_binary(schema).unwrap();
        let value = parsed.read(data).unwrap();
        (parsed, value)
    }

    /// The same schemas and data as the Python reader's tests.
    #[test]
    fn basic() {
        assert_eq!(read(&[1], &[]), (Type::Null, Value::Null));
        assert_eq!(read(&[2], &[0]), (Type::Boolean, Value::Boolean(false)));
        assert_eq!(
            read(&[3, 2], &[1, 2]),
            (Type::FixedInt { size: 2 }, Value::Int(0x201))
        );
        assert_eq!(
            read(&[4, 4], &[1, 2, 3, 4]),
            (Type::FixedUInt { size: 4 }, Value::UInt(0x0403_0201))
        );
        assert_eq!(read(&[6], &[3]), (Type::Varuint, Value::UInt(3)));
        assert_eq!(read(&[7], &[0; 4]), (Type::Float32, Value::Float(0.0)));
        assert_eq!(read(&[8], &[0; 8]), (Type::Float64, Value::Float(0.0)));
        assert_eq!(
            read(&[9], &[2, 4, 5]),
            (Type::Bytes, Value::Bytes(vec![4, 5]))
        );
        assert_eq!(
            read(&[10], &[3, 97, 98, 99]),
            (Type::String, Value::String("abc".to_owned()))
        );
        assert_eq!(
            read(&[18, 4, 1], &[3, 7, 8, 9]).1,
            Value::Array(vec![Value::UInt(7), Value::UInt(8), Value::UInt(9)])
        );
        assert_eq!(
            read(&[19, 2, 4, 1], &[4, 5]).1,
            Value::Array(vec![Value::UInt(4), Value::UInt(5)])
        );
        assert_eq!(read(&[23], &[0; 8]), (Type::Duration, Value::Duration(0)));
        assert_eq!(
            read(&[22], &1_500_000i64.to_le_bytes()).1.as_seconds(),
            Some(1.5)
        );
    }

    #[test]
    fn object() {
        let schema = [16, 0, 0, 2, 102, 49, 0, 2, 0, 0, 0, 0, 0, 0];
        let (parsed, value) = read(&schema, &[0]);
        let Type::Object(object_type) = &parsed else {
            panic!("{parsed:?}");
        };
        assert_eq!(object_type.fields.len(), 1);
        assert_eq!(object_type.field("f1").unwrap().field_type, Type::Boolean);
        assert_eq!(object_type.field("f1").unwrap().default_value, None);

        let object = value.as_object().unwrap();
        assert_eq!(object.len(), 1);
        assert_eq!(
            object.iter().collect::<Vec<_>>(),
            vec![("f1", &Value::Boolean(false))]
        );
        assert_eq!(value.get("f1").and_then(Value::as_bool), Some(false));
        assert_eq!(value.get("f2"), None);
    }

    #[test]
    fn object_with_aliases_and_default() {
        // One field "a", once called "b", a u8 which defaults to 7.
        let schema = [16, 0, 0, 1, 97, 1, 1, 98, 4, 1, 1, 7, 0, 0, 0, 0, 0];
        let (parsed, value) = read(&schema, &[9]);
        let Type::Object(object_type) = &parsed else {
            panic!("{parsed:?}");
        };
        let field = object_type.field("a").unwrap();
        assert_eq!(field.aliases, vec!["b".to_owned()]);
        assert_eq!(field.default_value, Some(Value::UInt(7)));
        assert_eq!(value.get("a").and_then(Value::as_u64), Some(9));
    }

    #[test]
    fn enumeration() {
        let schema = [17, 4, 1, 2, 1, 3, 101, 110, 49, 2, 3, 101, 110, 50];
        let (parsed, value) = read(&schema, &[1]);
        let Type::Enum(enum_type) = &parsed else {
            panic!("{parsed:?}");
        };
        assert_eq!(enum_type.name(2), Some("en2"));
        assert_eq!(enum_type.value("en2"), Some(2));
        assert_eq!(enum_type.value("en3"), None);

        let enum_value = value.as_enum().unwrap();
        assert_eq!(enum_value.value, 1);
        assert_eq!(enum_value.name.as_deref(), Some("en1"));
        assert_eq!(enum_value.to_string(), "en1");
        assert_eq!(value.as_i64(), Some(1));
        assert_eq!(value.as_f64(), Some(1.0));

        // A value with no name is still a value.
        let unnamed = parsed.read(&[10]).unwrap();
        assert_eq!(unnamed.as_enum().unwrap().name, None);
        assert_eq!(unnamed.as_enum().unwrap().to_string(), "10");

        // As is any value of an enumeration with no names at all.
        assert_eq!(read(&[17, 4, 1, 0], &[10]).1.as_i64(), Some(10));

        // An enumeration of floats is not one.
        assert_eq!(
            Type::from_binary(&[17, 7, 0]).unwrap().read(&[0; 4]),
            Err(ParseError::InvalidEnum)
        );
    }

    #[test]
    fn map() {
        let (_, value) = read(&[20, 4, 1], &[2, 2, 105, 49, 10, 2, 105, 50, 11]);
        assert_eq!(value.as_map().unwrap().len(), 2);
        assert_eq!(value.get("i1").and_then(Value::as_u64), Some(10));
        assert_eq!(value.get("i2").and_then(Value::as_u64), Some(11));
    }

    #[test]
    fn union() {
        let schema = [21, 4, 1, 10, 0];
        let (_, value) = read(&schema, &[1, 3, 116, 115, 49]);
        assert_eq!(value.as_str(), Some("ts1"));
        assert_eq!(
            Type::from_binary(&schema).unwrap().read(&[2]),
            Err(ParseError::InvalidUnionIndex(2))
        );
    }

    #[test]
    fn varints() {
        let mut stream = Stream::new(&[0x00, 0x01, 0x02, 0x03, 0x80, 0x01]);
        assert_eq!(stream.read_varint(), Ok(0));
        assert_eq!(stream.read_varint(), Ok(-1));
        assert_eq!(stream.read_varint(), Ok(1));
        assert_eq!(stream.read_varint(), Ok(-2));
        assert_eq!(stream.read_varint(), Ok(64));
        assert!(stream.is_empty());

        let largest = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01];
        assert_eq!(Stream::new(&largest).read_varuint(), Ok(u64::MAX));
        assert_eq!(Stream::new(&largest).read_varint(), Ok(i64::MIN));
        assert_eq!(read(&[5], &largest).1, Value::Int(i64::MIN));

        let mut too_large = largest;
        too_large[9] = 0x02;
        assert_eq!(
            Stream::new(&too_large).read_varuint(),
            Err(ParseError::InvalidVaruint)
        );
        assert_eq!(
            Stream::new(&[0x80; 11]).read_varuint(),
            Err(ParseError::InvalidVaruint)
        );
        assert_eq!(
            Stream::new(&[0x80]).read_varuint(),
            Err(ParseError::UnexpectedEof)
        );
    }

    #[test]
    fn streams_of_several() {
        // Two schemas, one after the other, and then two records.
        let mut schemas = Stream::new(&[4, 2, 10, 99]);
        let first = Type::from_stream(&mut schemas).unwrap();
        let second = Type::from_stream(&mut schemas).unwrap();
        assert_eq!(schemas.remaining(), &[99]);

        let mut data = Stream::new(&[1, 2, 1, 97, 5, 0]);
        assert_eq!(first.read_stream(&mut data), Ok(Value::UInt(0x201)));
        assert_eq!(second.read_stream(&mut data).unwrap().as_str(), Some("a"));
        assert_eq!(first.read_stream(&mut data), Ok(Value::UInt(5)));
        assert_eq!(first.read_stream(&mut data), Err(ParseError::UnexpectedEof));
    }

    #[test]
    fn nested_members() {
        // An object holding an array of objects.
        let inner = [16, 0, 0, 1, 120, 0, 3, 1, 0, 0, 0, 0, 0, 0];
        let mut schema = vec![16, 0, 0, 1, 97, 0, 18];
        schema.extend(inner);
        schema.extend([0, 0, 0, 0, 0, 0]);
        let (_, value) = read(&schema, &[2, 5, 0xff]);
        assert_eq!(value.get_path("a.0.x").and_then(Value::as_i64), Some(5));
        assert_eq!(value.get_path("a.1.x").and_then(Value::as_i64), Some(-1));
        assert_eq!(value.get_path("a.1.x").and_then(Value::as_u64), None);
        assert_eq!(value.get_path("a.2.x"), None);
        assert_eq!(value.get_path("a.x"), None);
        assert_eq!(value.get_path("a").unwrap().as_array().unwrap().len(), 2);
    }

    #[test]
    fn invalid() {
        assert_eq!(Type::from_binary(&[]), Err(ParseError::UnexpectedEof));
        assert_eq!(Type::from_binary(&[11]), Err(ParseError::UnknownType(11)));
        assert_eq!(
            Type::from_binary(&[3, 3]),
            Err(ParseError::InvalidFixedSize(3))
        );
        assert_eq!(
            Type::from_binary(&[10]).unwrap().read(&[2, 0xff, 0xfe]),
            Err(ParseError::InvalidUtf8)
        );
        assert_eq!(Type::Final.read(&[]), Err(ParseError::FinalType));
        assert_eq!(
            Type::FixedInt { size: 3 }.read(&[0; 8]),
            Err(ParseError::InvalidFixedSize(3))
        );
        // More elements than there is data for is not attempted.
        assert_eq!(
            Type::from_binary(&[18, 1])
                .unwrap()
                .read(&[0xff, 0xff, 0xff, 0x7f]),
            Err(ParseError::UnexpectedEof)
        );
        // Every prefix of a valid schema and of valid data is an
        // error, not a panic.
        let schema = [16, 0, 0, 1, 97, 1, 1, 98, 4, 1, 1, 7, 0, 0, 0, 0, 0];
        for length in 0..schema.len() {
            assert_eq!(
                Type::from_binary(&schema[..length]),
                Err(ParseError::UnexpectedEof)
            );
        }
        assert_eq!(Type::from_binary(&[18; 100]), Err(ParseError::TooDeep));

        let error: crate::Error = ParseError::UnexpectedEof.into();
        assert_eq!(error.to_string(), "Telemetry error: unexpected end of data");
    }
}
