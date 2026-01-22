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

//! Extension traits for command types to support query format overrides.
//!
//! This module provides a generic mechanism to attach a query format override
//! to any command type, allowing per-call customization of the query resolution.

use moteus_protocol::query::QueryFormat;

/// Wrapper that pairs any command with a query format override.
///
/// This allows overriding the controller's default query format for a specific call.
#[derive(Debug, Clone)]
pub struct WithQuery<C> {
    /// The wrapped command
    pub command: C,
    /// The query format to use instead of the controller's default
    pub format: QueryFormat,
}

/// Extension trait that adds `.with_query()` to any command type.
///
/// This trait has a blanket implementation for all types, so any command
/// can use `.with_query()` to attach a query format override.
///
/// # Example
///
/// ```ignore
/// use moteus::CommandExt;
/// use moteus_protocol::command::PositionCommand;
/// use moteus_protocol::query::QueryFormat;
///
/// let cmd = PositionCommand::new()
///     .position(0.5)
///     .with_query(QueryFormat::comprehensive());
/// ```
pub trait CommandExt: Sized {
    /// Attaches a query format override to this command.
    fn with_query(self, format: QueryFormat) -> WithQuery<Self> {
        WithQuery {
            command: self,
            format,
        }
    }
}

// Blanket implementation for all types
impl<T> CommandExt for T {}

/// Represents a command that may or may not have a query format override.
///
/// This enum is used internally to handle both bare commands and commands
/// with query overrides in a uniform way.
#[derive(Debug, Clone)]
pub enum MaybeQuery<C> {
    /// Use the controller's default query format
    Default(C),
    /// Override with the specified query format
    Override(C, QueryFormat),
}

impl<C> MaybeQuery<C> {
    /// Returns a reference to the underlying command.
    pub fn command(&self) -> &C {
        match self {
            MaybeQuery::Default(c) => c,
            MaybeQuery::Override(c, _) => c,
        }
    }

    /// Returns the query format override, if any.
    pub fn query_override(&self) -> Option<&QueryFormat> {
        match self {
            MaybeQuery::Default(_) => None,
            MaybeQuery::Override(_, f) => Some(f),
        }
    }

    /// Consumes self and returns the command and optional query format.
    pub fn into_parts(self) -> (C, Option<QueryFormat>) {
        match self {
            MaybeQuery::Default(c) => (c, None),
            MaybeQuery::Override(c, f) => (c, Some(f)),
        }
    }
}

/// Allows bare commands to be passed where `MaybeQuery<C>` is expected.
impl<C> From<C> for MaybeQuery<C> {
    fn from(cmd: C) -> Self {
        MaybeQuery::Default(cmd)
    }
}

/// Allows `WithQuery<C>` to be passed where `MaybeQuery<C>` is expected.
impl<C> From<WithQuery<C>> for MaybeQuery<C> {
    fn from(wq: WithQuery<C>) -> Self {
        MaybeQuery::Override(wq.command, wq.format)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use moteus_protocol::command::PositionCommand;

    #[test]
    fn test_with_query() {
        let cmd = PositionCommand::new().position(0.5);
        let with_query = cmd.with_query(QueryFormat::default());

        assert!(with_query.command.position.is_some());
    }

    #[test]
    fn test_maybe_query_from_command() {
        let cmd = PositionCommand::new().position(0.5);
        let maybe: MaybeQuery<PositionCommand> = cmd.into();

        assert!(maybe.query_override().is_none());
    }

    #[test]
    fn test_maybe_query_from_with_query() {
        let cmd = PositionCommand::new()
            .position(0.5)
            .with_query(QueryFormat::comprehensive());
        let maybe: MaybeQuery<PositionCommand> = cmd.into();

        assert!(maybe.query_override().is_some());
    }

    #[test]
    fn test_into_parts() {
        let cmd = PositionCommand::new().position(0.5);
        let maybe: MaybeQuery<PositionCommand> = cmd.into();
        let (cmd, format) = maybe.into_parts();

        assert!(cmd.position.is_some());
        assert!(format.is_none());
    }
}
