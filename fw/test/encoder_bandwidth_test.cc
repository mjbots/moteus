// Copyright 2025 mjbots Robotic Systems, LLC.  info@mjbots.com
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

#include "fw/motor_position.h"

#include <fstream>
#include <sstream>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "mjlib/micro/test/persistent_config_fixture.h"

#include "mjlib/base/clipp_archive.h"
#include "mjlib/base/clipp.h"
#include "mjlib/base/fail.h"
#include "mjlib/base/visitor.h"

using namespace moteus;

namespace {

struct Options {
  double dt = 1.0f / 30000.0f;
  bool hall = false;
  double magnitude = 0.10;
  bool commutation = false;

  double frequency_min_hz = 1.0;
  double frequency_max_hz = 400.0;
  double frequency_step = 1.2;

  double min_time_s = 2.0;

  double filter_hz = 100.0;
  bool single = false;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(dt));
    a->Visit(MJ_NVP(hall));
    a->Visit(MJ_NVP(magnitude));
    a->Visit(MJ_NVP(commutation));
    a->Visit(MJ_NVP(frequency_min_hz));
    a->Visit(MJ_NVP(frequency_max_hz));
    a->Visit(MJ_NVP(frequency_step));

    a->Visit(MJ_NVP(min_time_s));

    a->Visit(MJ_NVP(filter_hz));
    a->Visit(MJ_NVP(single));
  }
};

struct Context {
  mjlib::micro::test::PersistentConfigFixture pcf;
  mjlib::micro::TelemetryManager telemetry_manager{
    &pcf.pool, &pcf.command_manager, &pcf.write_stream, pcf.output_buffer};
  aux::AuxStatus aux1_status;
  aux::AuxStatus aux2_status;
  aux::AuxConfig aux1_config;
  aux::AuxConfig aux2_config;

  MotorPosition dut{&pcf.persistent_config, &telemetry_manager,
                    &aux1_status, &aux2_status,
                    &aux1_config, &aux2_config};

  Context() {
    dut.motor()->poles = 60;
    dut.config()->sources[1].type = MotorPosition::SourceConfig::kSpi;

    aux1_status.spi.active = true;

    pcf.persistent_config.Load();
  }
};

struct Application {
  Options options;

  Application(int argc, char** argv) {
    auto group = mjlib::base::ClippArchive().Accept(&options).group();
    mjlib::base::ClippParse(argc, argv, group);

  }

  void Run() {
    if (options.single) {
      RunFrequency(options.frequency_min_hz, kPrint);
    } else {
      for (double frequency_hz = options.frequency_min_hz;
           frequency_hz <= options.frequency_max_hz;
           frequency_hz *= options.frequency_step) {
        const double result = RunFrequency(frequency_hz, kSilent);
        fmt::print("{} {}\n", frequency_hz, result);
      }
    }
  }

  enum RunMode {
    kSilent,
    kPrint,
  };

  double RunFrequency(double freq_hz, RunMode run_mode) {
    Context ctx;
    ctx.dut.SetRate(options.dt);


    if (options.commutation) {
      ctx.dut.config()->commutation_source = 0;
    } else {
      ctx.dut.config()->commutation_source = 1;
    }
    if (options.hall) {
      ctx.dut.config()->sources[0].type = MotorPosition::SourceConfig::kHall;
      ctx.aux1_status.hall.active = true;
    } else {
      ctx.dut.config()->sources[0].type = MotorPosition::SourceConfig::kSpi;
      ctx.dut.config()->sources[0].cpr = 16384.0f;
    }

    ctx.dut.config()->sources[0].pll_filter_hz = options.filter_hz;
    ctx.pcf.persistent_config.Load();

    const double final_time =
        std::max(options.min_time_s, 50.0 / options.filter_hz);
    double sqsum_inp = 0.0;
    double sqsum = 0.0;
    int count = 0;

    for (double t = 0; t < final_time; t += options.dt) {
      const auto cpr = ctx.dut.config()->sources[0].cpr;
      const double raw = std::sin(freq_hz * t * 2.0 * M_PI) *
          options.magnitude * cpr;
      sqsum_inp += std::pow(raw, 2.0);

      int raw_value = 0;

      if (options.hall) {
        const auto old_count = ctx.aux1_status.hall.count;
        const auto new_count = ((static_cast<int>(raw) % 6) + 6) % 6;
        ctx.aux1_status.hall.count = new_count;
        if (old_count != new_count) {
          ctx.aux1_status.hall.nonce += 1;
        }
        raw_value = ctx.aux1_status.hall.count;
      } else {
        ctx.aux1_status.spi.value = (static_cast<int>(raw) + cpr) % cpr;
        raw_value = ctx.aux1_status.spi.value;
      }

      ctx.aux1_status.spi.nonce += 1;

      ctx.dut.ISR_Update();

      const auto status = ctx.dut.status();
      const auto value = status.sources[0].filtered_value;
      const auto wrapped = MotorPosition::WrapBalancedCpr(
          value, ctx.dut.config()->sources[0].cpr);

      sqsum += std::pow(wrapped, 2);
      count++;

      if (run_mode == kPrint) {
        fmt::print("{} {} {} {} {}\n", t, raw, raw_value, status.sources[0].compensated_value, wrapped);
      }
    }

    const auto input_rms = std::sqrt(sqsum_inp / count);
    const auto output_rms = std::sqrt(sqsum / count);
    return output_rms / input_rms;
  }
};
}

int main(int argc, char** argv) {
  Application app(argc, argv);

  try {
    app.Run();
    return 0;
  } catch (std::runtime_error& e) {
    fmt::print(stderr, "Error: {}\n", e.what());
    return 1;
  }

  return 2;
}
