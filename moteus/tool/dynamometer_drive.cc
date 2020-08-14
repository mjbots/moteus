// Copyright 2020 Josh Pieper, jjp@pobox.com.
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

/// @file
///
/// Drive a dynamometer test fixture.

#include "mjlib/base/clipp_archive.h"
#include "mjlib/base/clipp.h"
#include "mjlib/base/visitor.h"

namespace moteus {
namespace tool {
namespace {

struct Options {
  int fixture_id = 32;
  int dut_id = 1;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(fixture_id));
    a->Visit(MJ_NVP(dut_id));
  }
};

int do_main(int argc, char** argv) {
  Options options;

  auto group = mjlib::base::ClippArchive().Accept(&options).group();
  mjlib::base::ClippParse(argc, argv, group);

  return 0;
}

}
}
}

int main(int argc, char** argv) {
  return ::moteus::tool::do_main(argc, argv);
}
