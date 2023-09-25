// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
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

#pragma once

#include <memory>
#include <optional>

#include <boost/asio/awaitable.hpp>

#include "mjlib/io/deadline_timer.h"

namespace moteus {
namespace tool {

/// Run the coroutine @f, but call 'cancel' on @p object if it does
/// not return before @p expires_at.
template <typename Object, typename Functor>
auto RunFor(const boost::asio::any_io_executor& executor,
            Object& object,
            Functor f,
            boost::posix_time::ptime expires_at)
    -> boost::asio::awaitable<std::optional<typename decltype(f())::value_type>> {

  struct Context {
    bool done = false;
  };

  auto ctx = std::make_shared<Context>();

  boost::asio::co_spawn(
      executor,
      [ctx, &executor, &object, expires_at]() -> boost::asio::awaitable<void> {
        boost::asio::deadline_timer timer{executor};
        timer.expires_at(expires_at);
        co_await timer.async_wait(boost::asio::use_awaitable);
        if (ctx->done) { co_return; }

        ctx->done = true;
        object.cancel();
      },
      boost::asio::detached);

  auto result = co_await f();
  ctx->done = true;
  co_return result;
}

}
}
