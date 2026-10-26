#ifndef PROJECT2_HOST_TIME_ANCHOR_HPP
#define PROJECT2_HOST_TIME_ANCHOR_HPP
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
namespace lidar_timing {
struct ClockPair { int64_t before_ns, ros_ns, after_ns; };
// Conditional envelope from clock-read bracketing ONLY. Not acquisition uncertainty.
struct HostEnvelope { int64_t lower_ns, upper_ns; };
inline int64_t checked_add(int64_t a,int64_t b) {
  if ((b>0 && a>INT64_MAX-b) || (b<0 && a<INT64_MIN-b))
    throw std::overflow_error("Timestamp addition overflow");
  return a+b;
}
inline int64_t checked_sub(int64_t a,int64_t b) {
  if ((b>0 && a<INT64_MIN+b) || (b<0 && a>INT64_MAX+b))
    throw std::overflow_error("Timestamp subtraction overflow");
  return a-b;
}
inline HostEnvelope offset_bracket(const ClockPair& pair) {
  if (pair.before_ns<0 || pair.after_ns<pair.before_ns || pair.ros_ns<0)
    throw std::invalid_argument("Invalid clock bracket");
  return {checked_sub(pair.ros_ns,pair.after_ns),checked_sub(pair.ros_ns,pair.before_ns)};
}
inline HostEnvelope host_anchor(int64_t event_ns,const ClockPair& pair) {
  if (event_ns<0 || event_ns>pair.before_ns)
    throw std::invalid_argument("Host anchor must precede clock bracket");
  const auto offset=offset_bracket(pair);
  return {checked_add(event_ns,offset.lower_ns),checked_add(event_ns,offset.upper_ns)};
}
inline HostEnvelope shift_model(const HostEnvelope& anchor,double relative_ns) {
  // A diagnostic scan offset must be finite, nonnegative, and below 1000 seconds.
  if (!std::isfinite(relative_ns) || relative_ns<0 || relative_ns>=1e12 ||
      anchor.lower_ns>anchor.upper_ns) throw std::invalid_argument("Invalid relative time or envelope");
  return {checked_add(anchor.lower_ns,static_cast<int64_t>(std::floor(relative_ns))),
          checked_add(anchor.upper_ns,static_cast<int64_t>(std::ceil(relative_ns)))};
}
inline HostEnvelope shift_sample_fraction(const HostEnvelope& anchor,int64_t index,
    int64_t period_ns,int64_t count) {
  if (index<0 || period_ns<=0 || count<=0 || index>=count || anchor.lower_ns>anchor.upper_ns)
    throw std::invalid_argument("Invalid sample fraction");
  if (index>INT64_MAX/period_ns) throw std::overflow_error("Sample fraction overflow");
  const auto numerator=index*period_ns;
  const auto floor=numerator/count;
  const auto ceil=floor+(numerator%count!=0);
  return {checked_add(anchor.lower_ns,floor),checked_add(anchor.upper_ns,ceil)};
}
inline int64_t bracket_gap(const ClockPair& a,const ClockPair& b) {
  const auto x=offset_bracket(a),y=offset_bracket(b);
  if (x.lower_ns>y.upper_ns) return checked_sub(x.lower_ns,y.upper_ns);
  if (y.lower_ns>x.upper_ns) return checked_sub(y.lower_ns,x.upper_ns);
  return 0;
}
} // namespace lidar_timing
#endif
