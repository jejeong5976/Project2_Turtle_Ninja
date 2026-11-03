#include "planning/approach.hpp"

#include <algorithm>
#include <cmath>

namespace planning
{

Pose2D backed_off(const Pose2D & pose, double back_m)
{
  return {pose.x - std::cos(pose.yaw) * back_m, pose.y - std::sin(pose.yaw) * back_m, pose.yaw};
}

void approach_observe(ApproachState & st, const ApproachConfig & c, double face_m, double front_m, double travel_m)
{
  // 관측 (ref_face, ref_travel) 에서 예측한 지금 면과의 차 (+ = 예측보다 멂)
  auto off = [&](double ref_face, double ref_travel) {return face_m - (ref_face - (travel_m - ref_travel));};
  if (!st.have_face) {
    // 첫 면은 바로 앞 스캔과 맞아야 받음 (한 스캔 튄 값을 기준으로 삼지 않게). 안 맞으면 이번 것을 다음 비교 대상으로,
    //   면 없는 스캔이 끼면 처음부터 (띄엄띄엄 두 번 보인 반사로 움직이지 않게)
    if (!std::isfinite(face_m)) {
      st.have_prev = false;
      return;
    }
    if (!(st.have_prev && std::fabs(off(st.prev_face_m, st.prev_travel_m)) <= c.face_jump_m)) {
      st.have_prev = true;
      st.prev_face_m = face_m;
      st.prev_travel_m = travel_m;
      return;
    }
  } else if (std::isfinite(face_m) &&
    (off(st.face_m, st.travel_m) <= c.face_jump_m ||
    (st.have_prev && std::fabs(off(st.prev_face_m, st.prev_travel_m)) <= c.face_jump_m)))
  {
    // 가까워진 쪽은 일찍 멈출 뿐이라 그대로 받음. 먼 쪽으로 뛴 면은 그 앞 면 예측에 맞을 때 (± face_jump_m) 만 받음
    //   (바로 전 면이 한 스캔 튄 가까운 값이었던 것 → 실제 면을 계속 버려 일찍 끝내지 않게)
    st.prev_face_m = st.face_m;
    st.prev_travel_m = st.travel_m;
  } else {
    // 면이 안 보이거나 먼 배경만 (front_m 도 먼 값으로 덮지 않음). 앞에 뭔가 보이면 마지막 면 관측 때 기준으로 바꿔
    //   더 가까운 것만 저장 (approach_step 이 그 뒤 직진만큼 뺌)
    if (std::isfinite(front_m)) {
      const double front_ref = front_m + (travel_m - st.travel_m);
      st.front_m = std::isfinite(st.front_m) ? std::min(st.front_m, front_ref) : front_ref;
    }
    return;
  }
  st.have_face = true;
  st.face_m = face_m;
  st.front_m = front_m;
  st.travel_m = travel_m;
}

ApproachCmd approach_step(const ApproachState & st, const ApproachConfig & c, double travel_m, double dist_m)
{
  ApproachCmd out;
  if (dist_m > c.max_travel_m) {
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
