#include "sample_time_model.hpp"
#include <cassert>
#include <iostream>
int main() {
  using lidar_timing::model_times;
  // Angular-bin order crosses the seam; source time must remain in its original ring.
  const auto times=model_times(400,100000000,{1,0,399,-1,397});
  assert(times[0].nominal_ns==250000 && times[1].nominal_ns==0);
  assert(times[2].nominal_ns==99750000 && times[2].period_scaled_ns==99750000);
  assert(times[2].nominal_ns-times[1].nominal_ns==99750000);
  assert(std::isnan(times[3].nominal_ns) && times[3].source_index==-1);
  assert(times[4].nominal_ns==99250000); // gap of two source samples, not one bin step
  const auto varied=model_times(401,100000000,{0,400});
  assert(varied[1].nominal_ns==100000000);
  assert(varied[1].period_scaled_ns<100000000);
  const auto one=model_times(1,100000000,{0});
  assert(one[0].nominal_ns==0 && one[0].period_scaled_ns==0);
  for (const int64_t invalid:{-2,400}) {
    bool rejected=false;
    try { model_times(400,100000000,{invalid}); } catch(const std::invalid_argument&) {rejected=true;}
    assert(rejected);
  }
  bool rejected=false;
  try { model_times(0,100000000,{}); } catch(const std::invalid_argument&) {rejected=true;}
  assert(rejected);
  rejected=false;
  try { model_times(400,NAN,{0}); } catch(const std::invalid_argument&) {rejected=true;}
  assert(rejected);
  std::cout << "PASS: relative offsets, seam preservation, empty bins, gaps, variable count, invalid input\n";
}
