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

#include <cstring>
#include <string>

namespace mjbots {
namespace moteus {
namespace detail {

class Tokenizer {
 public:
  Tokenizer(const std::string& source, const char* delimiters)
      : source_(source),
        delimiters_(delimiters),
        position_(source_.cbegin()) {}

  std::string next() {
    if (position_ == source_.end()) { return std::string(); }

    const auto start = position_;
    auto my_next = position_;
    bool found = false;
    for (; my_next != source_.end(); ++my_next) {
      if (std::strchr(delimiters_, *my_next) != nullptr) {
        position_ = my_next;
        ++position_;
        found = true;
        break;
      }
    }
    if (!found) { position_ = my_next; }
    return std::string(&*start, my_next - start);
  }

  std::string remaining() const {
    return std::string(&*position_, source_.end() - position_);
  }

 private:
  const std::string source_;
  const char* const delimiters_;
  std::string::const_iterator position_;
};


}  // namespace detail
}  // namespace moteus
}  // namespace mjbots
