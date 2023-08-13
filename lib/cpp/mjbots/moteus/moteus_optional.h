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

namespace mjbots {
namespace moteus {

// A stripped down optional class for C++11 environments.
template <typename T>
class Optional {
 public:
  Optional() : dummy_(0), engaged_(false) {}
  Optional(const T& t) : val_(t), engaged_(true) {}

  ~Optional() {
    if (engaged_) { val_.~T(); }
  }

  Optional& operator=(const T& val) {
    engaged_ = true;
    val_ = val;
    return *this;
  }

  bool has_value() const { return engaged_; }

  T& operator*() noexcept { return val_; }
  T* operator->() noexcept { return &val_; }
  const T& operator*() const noexcept { return val_; }
  const T* operator->() const noexcept { return &val_; }

  explicit operator bool() const noexcept { return engaged_; }
  bool operator!() const noexcept { return !engaged_; }

 private:
  union { char dummy_; T val_; };
  bool engaged_;
};

}  // namespace moteus
}  // namespace mjbots
