// rl_core.cpp — RL 지역 계획기 계산 구현, 설명은 rl_core.hpp

#include "navigation/rl_core.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <numeric>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace navigation::rl
{

double wrap(double a)
{
  return a - 2.0 * M_PI * std::floor((a + M_PI) / (2.0 * M_PI));
}

Vec2 apply(const Pose2 & T, const Vec2 & p)
{
  const double c = std::cos(T.yaw), s = std::sin(T.yaw);
  return {T.x + c * p.x - s * p.y, T.y + s * p.x + c * p.y};
}

Pose2 compose(const Pose2 & T, const Pose2 & p)
{
  const Vec2 xy = apply(T, {p.x, p.y});
  return {xy.x, xy.y, wrap(p.yaw + T.yaw)};
}

Pose2 inverse(const Pose2 & T)
{
  const double c = std::cos(T.yaw), s = std::sin(T.yaw);
  return {-(c * T.x + s * T.y), -(-s * T.x + c * T.y), -T.yaw};
}

// ================================================================ 경로

RlPath::RlPath(std::vector<Vec2> pts)
: p_(std::move(pts))
{
  if (p_.size() < 2) {throw std::runtime_error("경로 점은 2 개 이상");}
  s_.assign(p_.size(), 0.0);
  for (size_t i = 1; i < p_.size(); ++i) {
    s_[i] = s_[i - 1] + std::hypot(p_[i].x - p_[i - 1].x, p_[i].y - p_[i - 1].y);
  }
  L_ = s_.back();
}

// np.interp 와 같음: xp[j] ≤ s < xp[j + 1] 인 마지막 j (길이 0 구간은 건너뜀)
Vec2 RlPath::point_at(double s) const
{
  s = std::clamp(s, 0.0, L_);
  const auto it = std::upper_bound(s_.begin(), s_.end(), s);
  const long j = static_cast<long>(it - s_.begin()) - 1;
  if (j >= static_cast<long>(p_.size()) - 1) {return p_.back();}
  if (j < 0) {return p_.front();}
  const double t = (s - s_[j]) / (s_[j + 1] - s_[j]);
  return {p_[j].x + t * (p_[j + 1].x - p_[j].x), p_[j].y + t * (p_[j + 1].y - p_[j].y)};
}

std::pair<double, double> RlPath::project(const Vec2 & xy, std::optional<double> s_prev, double back, double fwd) const
{
  double best_d = std::numeric_limits<double>::infinity(), best_s = 0.0;
  for (size_t i = 0; i + 1 < p_.size(); ++i) {
    const double dx = p_[i + 1].x - p_[i].x, dy = p_[i + 1].y - p_[i].y;
    const double L2 = std::max(dx * dx + dy * dy, 1e-12), Ls = std::sqrt(L2);
    double t = std::clamp(((xy.x - p_[i].x) * dx + (xy.y - p_[i].y) * dy) / L2, 0.0, 1.0);
    double s = s_[i] + t * Ls;
    if (s_prev) {
      s = std::clamp(s, *s_prev - back, *s_prev + fwd);
      t = std::clamp((s - s_[i]) / Ls, 0.0, 1.0);
    }
    const double d = std::hypot(xy.x - (p_[i].x + dx * t), xy.y - (p_[i].y + dy * t));
    if (d < best_d) {best_d = d; best_s = s;}   // 같으면 앞 구간 (argmin)
  }
  return {std::max(0.0, best_s), best_d};
}

// ================================================================ npz 정책

namespace
{
struct NpyArray
{
  std::string descr;
  std::vector<size_t> shape;
  std::string data;                                     // 원시 바이트
};

uint32_t u32(const std::string & b, size_t o) {uint32_t v; std::memcpy(&v, b.data() + o, 4); return v;}
uint16_t u16(const std::string & b, size_t o) {uint16_t v; std::memcpy(&v, b.data() + o, 2); return v;}

// npz = 압축 안 한 zip (np.savez). 중앙 목록에서 이름 · 크기 · 위치 → 지역 머리 뒤 데이터
std::map<std::string, std::string> read_zip_stored(const std::string & path)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {throw std::runtime_error("정책 파일 없음: " + path);}
  const std::string b((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  if (b.size() < 22) {throw std::runtime_error("zip 아님: " + path);}
  size_t eocd = std::string::npos;
  for (size_t o = b.size() - 22 + 1; o-- > 0;) {
    if (u32(b, o) == 0x06054b50) {eocd = o; break;}
  }
  if (eocd == std::string::npos) {throw std::runtime_error("zip 끝 목록 없음: " + path);}
  const size_t n = u16(b, eocd + 10);
  size_t o = u32(b, eocd + 16);
  std::map<std::string, std::string> out;
  for (size_t k = 0; k < n; ++k) {
    if (o + 46 > b.size() || u32(b, o) != 0x02014b50) {throw std::runtime_error("zip 중앙 목록 깨짐: " + path);}
    const uint16_t method = u16(b, o + 10);
    const uint32_t usize = u32(b, o + 24);
    const uint16_t nlen = u16(b, o + 28), xlen = u16(b, o + 30), clen = u16(b, o + 32);
    const uint32_t loff = u32(b, o + 42);
    const std::string name = b.substr(o + 46, nlen);
    if (method != 0) {throw std::runtime_error("압축된 npz 는 못 읽음 (np.savez 로 저장): " + name);}
    if (usize == 0xFFFFFFFFu || loff == 0xFFFFFFFFu) {throw std::runtime_error("zip64 큰 항목은 못 읽음: " + name);}
    if (loff + 30 > b.size() || u32(b, loff) != 0x04034b50) {throw std::runtime_error("zip 지역 머리 깨짐: " + name);}
    const size_t data = loff + 30 + u16(b, loff + 26) + u16(b, loff + 28);
    if (data + usize > b.size()) {throw std::runtime_error("zip 데이터 잘림: " + name);}
    out[name] = b.substr(data, usize);
    o += 46 + nlen + xlen + clen;
  }
  return out;
}

NpyArray parse_npy(const std::string & name, const std::string & b)
{
  if (b.size() < 10 || b.compare(0, 6, "\x93NUMPY") != 0) {throw std::runtime_error("npy 아님: " + name);}
  const int major = static_cast<unsigned char>(b[6]);
  size_t hlen, ho;
  if (major == 1) {hlen = u16(b, 8); ho = 10;} else {hlen = u32(b, 8); ho = 12;}
  const std::string h = b.substr(ho, hlen);
  NpyArray a;
  const auto q = [&](const std::string & key) {
      const size_t k = h.find("'" + key + "'");
      if (k == std::string::npos) {throw std::runtime_error("npy 머리에 " + key + " 없음: " + name);}
      return h.find(':', k) + 1;
    };
  size_t i = h.find('\'', q("descr"));
  a.descr = h.substr(i + 1, h.find('\'', i + 1) - i - 1);
  if (h.find("True", q("fortran_order")) == h.find_first_not_of(' ', q("fortran_order"))) {
    throw std::runtime_error("fortran 순서 npy 는 못 읽음: " + name);
  }
  i = h.find('(', q("shape"));
  const std::string sh = h.substr(i + 1, h.find(')', i) - i - 1);
  size_t p = 0;
  while (p < sh.size()) {
    const size_t c = sh.find(',', p);
    const std::string tok = sh.substr(p, c == std::string::npos ? std::string::npos : c - p);
    if (tok.find_first_of("0123456789") != std::string::npos) {a.shape.push_back(std::stoul(tok));}
    if (c == std::string::npos) {break;}
    p = c + 1;
  }
  a.data = b.substr(ho + hlen);
  return a;
}

size_t count(const NpyArray & a) {return std::accumulate(a.shape.begin(), a.shape.end(), size_t{1}, std::multiplies<>());}

std::vector<double> as_doubles(const std::string & name, const NpyArray & a)
{
  const size_t n = count(a);
  std::vector<double> v(n);
  if (a.descr == "<f4") {
    if (a.data.size() < n * 4) {throw std::runtime_error("npy 데이터 잘림: " + name);}
    for (size_t i = 0; i < n; ++i) {float x; std::memcpy(&x, a.data.data() + 4 * i, 4); v[i] = x;}
  } else if (a.descr == "<f8") {
    if (a.data.size() < n * 8) {throw std::runtime_error("npy 데이터 잘림: " + name);}
    std::memcpy(v.data(), a.data.data(), n * 8);
  } else if (a.descr == "<i4") {
    for (size_t i = 0; i < n; ++i) {int32_t x; std::memcpy(&x, a.data.data() + 4 * i, 4); v[i] = x;}
  } else if (a.descr == "<i8") {
    for (size_t i = 0; i < n; ++i) {int64_t x; std::memcpy(&x, a.data.data() + 8 * i, 8); v[i] = static_cast<double>(x);}
  } else {
    throw std::runtime_error("npy 형식 " + a.descr + " 못 읽음: " + name);
  }
  return v;
}

std::string as_text(const std::string & name, const NpyArray & a)   // '<U?' = UTF-32 (ASCII 만)
{
  if (a.descr.rfind("<U", 0) != 0) {throw std::runtime_error("npy 글자 아님: " + name);}
  std::string s;
  for (size_t i = 0; i + 4 <= a.data.size(); i += 4) {
    const uint32_t c = u32(a.data, i);
    if (c == 0) {break;}
    s.push_back(static_cast<char>(c));
  }
  return s;
}
}  // namespace

NpzPolicy::NpzPolicy(const std::string & path)
{
  const auto files = read_zip_stored(path);
  const auto get = [&](const std::string & k) {
      const auto it = files.find(k + ".npy");
      if (it == files.end()) {throw std::runtime_error("정책 npz 에 " + k + " 없음: " + path);}
      return parse_npy(k, it->second);
    };
  mean_ = as_doubles("mean", get("mean"));
  std_ = as_doubles("std", get("std"));
  eps_ = as_doubles("eps", get("eps")).at(0);
  if (const auto act = as_text("activation", get("activation")); act != "elu") {
    throw std::runtime_error("활성 함수 " + act + " (elu 만): " + path);
  }
  const int n = static_cast<int>(as_doubles("n_layers", get("n_layers")).at(0));
  int in = static_cast<int>(mean_.size());
  for (int i = 0; i < n; ++i) {
    const auto W = get("W" + std::to_string(i));
    const auto b = get("b" + std::to_string(i));
    if (W.shape.size() != 2 || static_cast<int>(W.shape[1]) != in || b.shape.size() != 1 || b.shape[0] != W.shape[0]) {
      throw std::runtime_error("정책 층 " + std::to_string(i) + " 크기 안 맞음: " + path);
    }
    Layer L;
    L.out = static_cast<int>(W.shape[0]);
    L.in = in;
    L.W = as_doubles("W", W);
    L.b = as_doubles("b", b);
    layers_.push_back(std::move(L));
    in = layers_.back().out;
  }
  if (in != 3) {throw std::runtime_error("정책 출력이 3 개가 아님: " + path);}
  if (std_.size() != mean_.size()) {throw std::runtime_error("mean · std 크기 다름: " + path);}
}

std::array<double, 3> NpzPolicy::operator()(const std::vector<float> & obs) const
{
  if (obs.size() != mean_.size()) {
    throw std::runtime_error("관측 크기 " + std::to_string(obs.size()) + " ≠ 정책 입력 " + std::to_string(mean_.size()));
  }
  std::vector<double> x(obs.size());
  for (size_t i = 0; i < x.size(); ++i) {x[i] = (static_cast<double>(obs[i]) - mean_[i]) / (std_[i] + eps_);}
  for (size_t l = 0; l < layers_.size(); ++l) {
    const auto & L = layers_[l];
    std::vector<double> y(L.out);
    for (int r = 0; r < L.out; ++r) {
      const double * w = L.W.data() + static_cast<size_t>(r) * L.in;
      double acc = 0.0;
      for (int k = 0; k < L.in; ++k) {acc += w[k] * x[k];}
      y[r] = acc + L.b[r];
      if (l + 1 < layers_.size() && y[r] <= 0.0) {y[r] = std::expm1(y[r]);}   // ELU (alpha 1)
    }
    x = std::move(y);
  }
  return {x[0], x[1], x[2]};
}

// ================================================================ 박스 맞춤

BoxTracker::BoxTracker(Vec2 goal_xy, double goal_yaw, double box_dist, double half, double max_shift, double max_yaw_deg,
  double win, int min_pts, double freeze_dist, int med_n, double min_len)
: yaw0(goal_yaw), h(half), freeze(freeze_dist), dist_(box_dist), max_shift_(max_shift),
  max_yaw_(max_yaw_deg * M_PI / 180.0), win_(win), min_len_(min_len), min_pts_(min_pts), med_n_(med_n)
{
  c0 = {goal_xy.x + box_dist * std::cos(goal_yaw), goal_xy.y + box_dist * std::sin(goal_yaw)};   // 지도 박스 중심
  c = c0;
}

Pose2 BoxTracker::goal() const
{
  const double y = yaw0 + th;
  return {c.x - dist_ * std::cos(y), c.y - dist_ * std::sin(y), wrap(y)};
}

namespace
{
double median(std::vector<double> v)                    // numpy 와 같음 (짝수 개 = 가운데 둘 평균)
{
  std::sort(v.begin(), v.end());
  const size_t n = v.size();
  return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}
}  // namespace

bool BoxTracker::update(const std::vector<Vec2> & pts, Vec2 robot_xy)
{
  const Pose2 g = goal();
  if (std::hypot(robot_xy.x - g.x, robot_xy.y - g.y) < freeze) {
    last_why = "freeze";
    return false;
  }
  const auto m = measure(pts, robot_xy);
  if (!m) {return false;}
  hist_.push_back(*m);
  if (static_cast<int>(hist_.size()) > med_n_) {hist_.erase(hist_.begin());}
  std::vector<double> xs, ys, ts;
  for (const auto & e : hist_) {xs.push_back(e[0]); ys.push_back(e[1]); ts.push_back(e[2]);}
  c = {median(xs), median(ys)};
  th = median(ts);
  ++n_ok;
  return true;
}

std::optional<std::array<double, 3>> BoxTracker::measure(const std::vector<Vec2> & pts, Vec2 robot_xy)
{
  const double y = yaw0 + th;
  const Vec2 n{-std::cos(y), -std::sin(y)};             // 박스 → 로봇 쪽 면 법선
  const Vec2 t{-n.y, n.x};
  const Vec2 f{c.x + h * n.x, c.y + h * n.y};           // 지금 추정 앞면 가운데
  std::vector<Vec2> P;
  for (const auto & p : pts) {
    const double rx = p.x - f.x, ry = p.y - f.y;
    if (std::abs(rx * n.x + ry * n.y) < win_ && std::abs(rx * t.x + ry * t.y) < h + 0.06) {P.push_back(p);}
  }
  if (static_cast<int>(P.size()) < min_pts_) {
    last_why = "점 " + std::to_string(P.size());
    return std::nullopt;
  }
  Vec2 mu, d;
  for (int it = 0; it < 2; ++it) {                      // 직선 맞춤 → 1.5 cm 넘는 점 빼고 다시
    mu = {0.0, 0.0};
    for (const auto & p : P) {mu.x += p.x; mu.y += p.y;}
    mu.x /= P.size(); mu.y /= P.size();
    double sxx = 0, sxy = 0, syy = 0;
    for (const auto & p : P) {
      const double dx = p.x - mu.x, dy = p.y - mu.y;
      sxx += dx * dx; sxy += dx * dy; syy += dy * dy;
    }
    const double a = 0.5 * std::atan2(2.0 * sxy, sxx - syy);   // 가장 큰 고유벡터 방향 (SVD vt[0] 와 부호만 다를 수 있음)
    d = {std::cos(a), std::sin(a)};
    const Vec2 v1{-d.y, d.x};
    std::vector<Vec2> keep;
    for (const auto & p : P) {
      if (std::abs((p.x - mu.x) * v1.x + (p.y - mu.y) * v1.y) < 0.015) {keep.push_back(p);}
    }
    if (static_cast<int>(keep.size()) < min_pts_) {
      last_why = "직선 안 맞음";
      return std::nullopt;
    }
    P = std::move(keep);
  }
  mu = {0.0, 0.0};
  for (const auto & p : P) {mu.x += p.x; mu.y += p.y;}
  mu.x /= P.size(); mu.y /= P.size();
  Vec2 nn{-d.y, d.x};
  if (nn.x * (robot_xy.x - mu.x) + nn.y * (robot_xy.y - mu.y) < 0) {nn = {-nn.x, -nn.y};}
  const double thm = wrap(std::atan2(-nn.y, -nn.x) - yaw0);   // 면 법선 반대 = 로봇이 볼 방향
  char buf[96];
  if (std::abs(thm) > max_yaw_) {
    std::snprintf(buf, sizeof(buf), "기울기 %.0f°", thm * 180.0 / M_PI);
    last_why = buf;
    return std::nullopt;
  }
  double smin = std::numeric_limits<double>::infinity(), smax = -smin;
  for (const auto & p : P) {
    const double s = (p.x - mu.x) * d.x + (p.y - mu.y) * d.y;
    smin = std::min(smin, s); smax = std::max(smax, s);
  }
  const double span = smax - smin;
  if (span > 2 * h + 0.03) {
    ++n_wall;
    std::snprintf(buf, sizeof(buf), "면 길이 %.0f cm (벽)", span * 100);
    last_why = buf;
    return std::nullopt;
  }
  if (span < min_len_) {
    std::snprintf(buf, sizeof(buf), "면 길이 %.0f cm (조각)", span * 100);
    last_why = buf;
    return std::nullopt;
  }
  int beyond = 0;                                       // 창 밖까지 같은 직선 위 점 = 벽이 박스 끝에서 안 끊김
  for (const auto & p : pts) {
    const double rx = p.x - mu.x, ry = p.y - mu.y;
    if (std::abs(rx * nn.x + ry * nn.y) >= 0.015) {continue;}
    const double sa = rx * d.x + ry * d.y;
    if ((sa > smax + 0.02 && sa < smax + 0.14) || (sa < smin - 0.02 && sa > smin - 0.14)) {++beyond;}
  }
  if (beyond >= min_pts_) {
    ++n_wall;
    last_why = "면 밖으로 이어짐 " + std::to_string(beyond) + " 점 (벽)";
    return std::nullopt;
  }
  // 모서리 둘 다 보임 → 옆 위치도 잼, 한쪽만 → 이전 추정을 이 면에 투영
  const double s_mid = span >= 2 * h - 0.03 ? 0.5 * (smax + smin) : (c.x - mu.x) * d.x + (c.y - mu.y) * d.y;
  const Vec2 cm{mu.x + s_mid * d.x - h * nn.x, mu.y + s_mid * d.y - h * nn.y};
  const double shift = std::hypot(cm.x - c0.x, cm.y - c0.y);
  if (shift > max_shift_) {
    std::snprintf(buf, sizeof(buf), "지도에서 %.0f cm", shift * 100);
    last_why = buf;
    return std::nullopt;
  }
  last_why = "ok";
  return std::array<double, 3>{cm.x, cm.y, thm};
}

// ================================================================ 설정

EnvCfg load_env_cfg(const std::string & cfg_json)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(cfg_json);                    // JSON = YAML 부분집합
  } catch (const std::exception & e) {
    throw std::runtime_error("cfg 못 읽음: " + cfg_json + " (" + e.what() + ")");
  }
  const YAML::Node e = root["env"];
  if (!e) {throw std::runtime_error("cfg 에 env 없음: " + cfg_json);}
  EnvCfg c;
  const auto need = [&](const char * k) {
      if (!e[k]) {throw std::runtime_error(std::string("cfg env 에 ") + k + " 없음: " + cfg_json);}
      return e[k];
    };
  c.vmax = need("vmax").as<double>();
  c.wmax = need("wmax").as<double>();
  c.ctrl_dt = need("ctrl_dt").as<double>();
  c.decimation = need("decimation").as<int>();
  c.path_obs_points = need("path_obs_points").as<int>();
  c.path_obs_spacing = need("path_obs_spacing").as<double>();
  c.lidar_rays = need("lidar_rays").as<int>();
  c.lidar_max = need("lidar_max").as<double>();
  c.lidar_obs_clip = need("lidar_obs_clip").as<double>();
  c.history = need("history").as<int>();
  c.goal_hold_s = need("goal_hold_s").as<double>();
  c.goal_lin_vel = need("goal_lin_vel").as<double>();
  c.goal_ang_vel = need("goal_ang_vel").as<double>();
  if (e["lidar_body_half"]) {
    c.lidar_body_x = e["lidar_body_half"][0].as<double>();
    c.lidar_body_y = e["lidar_body_half"][1].as<double>();
  }
  if (e["approach_limit"]) {c.approach_limit = e["approach_limit"].as<bool>();}
  if (c.approach_limit) {
    c.approach_v0 = need("approach_v0").as<double>();
    c.approach_k = need("approach_k").as<double>();
  }
  if (e["rot_limit"]) {c.rot_limit = e["rot_limit"].as<bool>();}
  if (c.rot_limit) {
    c.rot_clear_lo = need("rot_clear_lo").as<double>();
    c.rot_clear_hi = need("rot_clear_hi").as<double>();
    c.rot_wz_lo = need("rot_wz_lo").as<double>();
  }
  if (e["swerve_yaml"]) {c.swerve_yaml = e["swerve_yaml"].as<std::string>();}
  return c;
}

SwerveCfg load_swerve(const std::string & yaml)
{
  YAML::Node y;
  try {
    y = YAML::LoadFile(yaml);
  } catch (const std::exception & e) {
    throw std::runtime_error("스워브 yaml 못 읽음: " + yaml + " (" + e.what() + ")");
  }
  SwerveCfg s;
  s.steer = y["joints"]["steer"].as<std::vector<std::string>>();
  s.drive = y["joints"]["drive"].as<std::vector<std::string>>();
  const auto mn = y["steer_range_rad"]["min"].as<std::vector<double>>();
  const auto mx = y["steer_range_rad"]["max"].as<std::vector<double>>();
  if (s.steer.size() != 4 || s.drive.size() != 4 || mn.size() != 4 || mx.size() != 4) {
    throw std::runtime_error("스워브 yaml 은 바퀴 4 개: " + yaml);
  }
  for (int i = 0; i < 4; ++i) {s.rmin[i] = mn[i]; s.rmax[i] = mx[i];}
  s.max_wheel = y["hardware_max"]["wheel_speed"].as<double>();
  return s;
}

// ================================================================ 관측 · 명령

std::vector<double> lidar_bins(const std::vector<Vec2> & body_pts, const EnvCfg & c)
{
  const int n = c.lidar_rays;
  std::vector<double> out(n, c.lidar_max);
  const double step = 2.0 * M_PI / n;
  for (const auto & p : body_pts) {
    if (std::abs(p.x) <= c.lidar_body_x && std::abs(p.y) <= c.lidar_body_y) {continue;}   // scan_merger 와 같은 몸 상자
    long k = static_cast<long>(std::nearbyint(std::atan2(p.y, p.x) / step));             // numpy round (짝수 쪽)
    k = ((k % n) + n) % n;
    out[k] = std::min(out[k], std::hypot(p.x, p.y));
  }
  return out;
}

std::vector<float> build_obs(const EnvCfg & c, const SwerveCfg & sw, const RlPath & path, const Pose2 & pose, double s,
  const Pose2 & goal, const std::vector<double> & scan, const JointObs & j, const std::array<double, 3> & last_cmd,
  std::vector<std::vector<double>> & hist)
{
  const double cs = std::cos(pose.yaw), sn = std::sin(pose.yaw);
  const auto body = [&](const Vec2 & p) {
      return Vec2{cs * (p.x - pose.x) + sn * (p.y - pose.y), -sn * (p.x - pose.x) + cs * (p.y - pose.y)};
    };
  std::vector<double> o;
  o.reserve(256);
  for (int k = 0; k < c.path_obs_points; ++k) {
    const Vec2 b = body(path.point_at(s + c.path_obs_spacing * k));
    o.push_back(b.x);
    o.push_back(b.y);
  }
  const Vec2 gb = body({goal.x, goal.y});
  const double dy = wrap(goal.yaw - pose.yaw);
  const double rem = path.length() - s;
  o.insert(o.end(), {gb.x / 2, gb.y / 2, std::sin(dy), std::cos(dy), std::min(rem / 3, 2.0)});
  for (const double r : scan) {o.push_back(std::clamp(r, 0.0, c.lidar_obs_clip) / c.lidar_obs_clip);}
  const std::array<double, 3> lim{c.vmax, c.vmax, c.wmax};
  std::vector<double> fr;
  for (int i = 0; i < 4; ++i) {fr.push_back(std::sin(j.q[i]));}
  for (int i = 0; i < 4; ++i) {fr.push_back(std::cos(j.q[i]));}
  for (int i = 0; i < 4; ++i) {fr.push_back((j.q[i] - sw.rmin[i]) / (sw.rmax[i] - sw.rmin[i]) * 2 - 1);}
  for (int i = 0; i < 4; ++i) {fr.push_back(j.w[i] / sw.max_wheel);}
  for (int i = 0; i < 3; ++i) {fr.push_back(j.odom[i] / lim[i]);}
  for (int i = 0; i < 3; ++i) {fr.push_back(last_cmd[i] / lim[i]);}
  if (hist.empty()) {
    hist.assign(c.history, fr);
  } else {
    hist.insert(hist.begin(), fr);
    hist.resize(c.history);
  }
  for (const auto & h : hist) {o.insert(o.end(), h.begin(), h.end());}
  return std::vector<float>(o.begin(), o.end());        // 학습과 같게 float32
}

std::array<double, 3> command(const EnvCfg & c, std::array<double, 3> a, std::optional<double> goal_dist,
  double extra_vlim, std::optional<double> scan_min)
{
  for (auto & v : a) {v = std::clamp(v, -1.0, 1.0);}
  double vx = a[0] * c.vmax, vy = a[1] * c.vmax;
  double vlim = c.vmax;
  if (c.approach_limit && goal_dist) {vlim = std::min(vlim, c.approach_v0 + c.approach_k * *goal_dist);}
  if (extra_vlim > 0) {vlim = std::min(vlim, extra_vlim);}
  const double n = std::hypot(vx, vy);
  if (n > vlim) {vx *= vlim / n; vy *= vlim / n;}
  double wz = a[2] * c.wmax;
  if (c.rot_limit) {                                    // 벽 가까우면 회전 상한 (라이다 최솟값)
    const double dmin = scan_min.value_or(0.0);
    const double t = std::clamp((dmin - c.rot_clear_lo) / (c.rot_clear_hi - c.rot_clear_lo), 0.0, 1.0);
    const double lim = c.rot_wz_lo + (c.wmax - c.rot_wz_lo) * t;
    wz = std::clamp(wz, -lim, lim);
  }
  return {vx, vy, wz};
}

std::pair<int, double> straight_blocked(const Pose2 & pose, const Pose2 & goal, const BoxTracker & tr,
  const std::vector<Vec2> & pts, Vec2 body_half)
{
  if (pts.empty()) {return {0, -1.0};}
  const double y = tr.yaw0 + tr.th;
  const Vec2 n{-std::cos(y), -std::sin(y)}, t{-n.y, n.x};
  const Vec2 f{tr.c.x + tr.h * n.x, tr.c.y + tr.h * n.y};
  std::vector<Vec2> P;
  for (const auto & p : pts) {
    const double rx = p.x - f.x, ry = p.y - f.y;
    const bool box = rx * n.x + ry * n.y < 0.03 && std::abs(rx * t.x + ry * t.y) < tr.h + 0.08;
    if (!box) {P.push_back(p);}
  }
  const double L = std::hypot(goal.x - pose.x, goal.y - pose.y);
  if (P.empty() || L < 1e-3) {return {0, -1.0};}
  const double hx = body_half.x + 0.01, hy = body_half.y + 0.01;
  const double dyaw = wrap(goal.yaw - pose.yaw);
  const int N = std::max(2, static_cast<int>(L / 0.02) + 1);
  for (int i = 0; i < N; ++i) {
    const double k = static_cast<double>(i) / (N - 1);
    const double cx = pose.x + k * (goal.x - pose.x), cy = pose.y + k * (goal.y - pose.y);
    const double yw = pose.yaw + k * dyaw, cc = std::cos(yw), ss = std::sin(yw);
    int hit = 0;
    for (const auto & p : P) {
      const double dx = p.x - cx, dy = p.y - cy;
      if (std::abs(cc * dx + ss * dy) <= hx && std::abs(-ss * dx + cc * dy) <= hy) {++hit;}
    }
    if (hit) {return {hit, k * L};}
  }
  return {0, -1.0};
}

}  // namespace navigation::rl
