#include "planning/approach.hpp"

#include <algorithm>
#include <cmath>

namespace planning
{

Pose2D backed_off(const Pose2D & pose, double back_m)
{
  return {pose.x - std::cos(pose.yaw) * back_m, pose.y - std::sin(pose.yaw) * back_m, pose.yaw};
}

void approach_observe(ApproachState & st, double face_m, double front_m, double travel_m)
{
  if (std::isfinite(face_m)) {
    st.have_face = true;
    st.face_m = face_m;
    st.front_m = front_m;
    st.travel_m = travel_m;
  } else if (st.have_face && std::isfinite(front_m)) {
    // 면은 사라졌지만 앞에 뭔가 보임 → 마지막 면 관측 때 기준으로 바꿔 저장 (approach_step 이 그 뒤 직진만큼 뺌)
    const double front_ref = front_m + (travel_m - st.travel_m);
    st.front_m = std::isfinite(st.front_m) ? std::min(st.front_m, front_ref) : front_ref;
  }
}

ApproachCmd approach_step(const ApproachState & st, const ApproachConfig & c, double travel_m)
{
  ApproachCmd out;
  if (travel_m > c.max_travel_m) {
    out.fail = true;
    out.why = "박스 접근: 예상보다 많이 감";
    return out;
  }
  if (!st.have_face) {   // 면을 본 적 없음 → 움직이지 않음 (시간 초과는 부르는 쪽)
    out.waiting = true;
    return out;
  }
  const double moved = travel_m - st.travel_m;   // 마지막 관측 뒤 직진
  out.remaining = st.face_m - moved - c.target_face_m;
  if (std::isfinite(st.front_m)) {
    const double front = st.front_m - moved;
    if (front < st.face_m - moved - c.obstacle_margin_m || front < c.min_clear_m) {
      out.fail = true;
      out.why = "박스 접근: 앞에 박스보다 가까운 물체";
      return out;
    }
  }
  if (out.remaining <= c.tol_m) {
    out.done = true;
    return out;
  }
  out.vx = std::clamp(c.gain * out.remaining, c.min_speed, c.speed);
  return out;
}

}  // namespace planning
