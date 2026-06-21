// Copyright 2026 goober
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

#ifndef PICK_N_PLACE_BOT__ARM_KINEMATICS_HPP_
#define PICK_N_PLACE_BOT__ARM_KINEMATICS_HPP_

#include <cmath>
#include <optional>

namespace arm_kinematics
{

struct Params
{
  double link_base{0.06};
  double link1{0.065};
  double link2{0.065};
  double ee_offset_rad{0.0};
  bool prefer_elbow_up{true};
};

struct ServoAngles
{
  int base_x;
  int base_y;
  int shoulder;
  int wrist;
};

inline int clamp_servo(double deg)
{
  const int v = static_cast<int>(std::lround(deg));
  if (v < 0) {return 0;}
  if (v > 180) {return 180;}
  return v;
}

inline std::optional<ServoAngles> solve_ik(
  double x, double y, double z, const Params & p)
{
  const double r = std::hypot(x, y);
  const double q_bx = std::atan2(y, x);

  const double px = r;
  const double pz = z - p.link_base;
  const double dist = std::hypot(px, pz);
  const double L1 = p.link1;
  const double L2 = p.link2;

  if (dist > L1 + L2 + 1e-9 || dist < std::abs(L1 - L2) - 1e-9) {
    return std::nullopt;
  }

  const double cos_el =
    (dist * dist - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
  if (cos_el < -1.0 || cos_el > 1.0) {
    return std::nullopt;
  }

  const double el = std::acos(cos_el);
  double q_s = p.prefer_elbow_up ? el : -el;
  if (std::abs(cos_el) < 1e-2) {
    q_s = p.prefer_elbow_up ? M_PI_2 : -M_PI_2;
  }

  const double q_by =
    std::atan2(px, pz) - std::atan2(L2 * std::sin(q_s), L1 + L2 * std::cos(q_s));

  const double q_w = M_PI_2 - q_by - q_s - p.ee_offset_rad;

  const double k = 180.0 / M_PI;
  const double bx = 90.0 + q_bx * k;
  const double by = 90.0 - q_by * k;
  const double sh = 90.0 - q_s * k;
  const double wr = 90.0 + q_w * k;

  if (bx < 0.0 || bx > 180.0 || by < 0.0 || by > 180.0 || sh < 0.0 || sh > 180.0 ||
    wr < 0.0 || wr > 180.0)
  {
    return std::nullopt;
  }

  ServoAngles out;
  out.base_x = clamp_servo(bx);
  out.base_y = clamp_servo(by);
  out.shoulder = clamp_servo(sh);
  out.wrist = clamp_servo(wr);

  return out;
}

}  // namespace arm_kinematics

#endif  // PICK_N_PLACE_BOT__ARM_KINEMATICS_HPP_
