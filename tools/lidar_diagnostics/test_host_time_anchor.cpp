#include "host_time_anchor.hpp"
#include <cassert>
#include <iostream>
int main() {
  using namespace lidar_timing;
  const ClockPair pair{1000,1792900000000000000LL,1010};
  const auto anchor=host_anchor(900,pair);
  assert(anchor.lower_ns==1792899999999999890LL && anchor.upper_ns==1792899999999999900LL);
  const auto shifted=shift_model(anchor,250000.25);
  assert(shifted.lower_ns==anchor.lower_ns+250000 && shifted.upper_ns==anchor.upper_ns+250001);
  assert(bracket_gap(pair,{1100,pair.ros_ns+100,1110})==0);
  assert(bracket_gap(pair,{1100,pair.ros_ns+120,1110})==10);
  // Preserve nanoseconds at Unix-epoch magnitudes; no floating-point epoch conversion.
  assert(shift_model(anchor,1).lower_ns-anchor.lower_ns==1);
  const auto fraction=shift_sample_fraction(anchor,1,100,3);
  assert(fraction.lower_ns==anchor.lower_ns+33 && fraction.upper_ns==anchor.upper_ns+34);
  const auto exact=shift_sample_fraction(anchor,2,90,3);
  assert(exact.lower_ns==anchor.lower_ns+60 && exact.upper_ns==anchor.upper_ns+60);
  bool bad=false;
  try {host_anchor(1011,pair);}catch(const std::invalid_argument&){bad=true;}assert(bad);
  bad=false;
  try {host_anchor(0,{2,5,1});}catch(const std::invalid_argument&){bad=true;}assert(bad);
  bad=false;
  try {shift_model(anchor,NAN);}catch(const std::invalid_argument&){bad=true;}assert(bad);
  bad=false;
  try {shift_model({INT64_MAX,INT64_MAX},1);}catch(const std::overflow_error&){bad=true;}assert(bad);
  bad=false;
  try {checked_sub(INT64_MIN,1);}catch(const std::overflow_error&){bad=true;}assert(bad);
  std::cout << "PASS: clock brackets, conditional host anchoring, rounding, epoch precision, invalid input, overflow\n";
}
