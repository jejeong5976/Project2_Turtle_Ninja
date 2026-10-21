// Arducam 캘리브레이션 사진을 안내에 따라 모으는 프로그램 (레포 밖에서 사용, 커밋 안 함).
// capture_guided.py 를 C++로 옮긴 것. 동작은 같음.
//
// 빌드 (Ubuntu 24.04, 처음 한 번: sudo apt install libopencv-dev):
//     g++ -O2 -std=c++17 capture_guided.cpp -o capture_guided $(pkg-config --cflags --libs opencv4)
// 실행:
//     ./capture_guided --dev /dev/video8 --out ~/calib_wide2
//
// 화면 보는 법:
//   - 3x3 칸: 빨강 = 아직 없음, 노랑 = 조금 있음, 초록 = 충분함. 빨강/노랑 칸에 보드를 가져가세요
//   - 오른쪽 위 TILT 목록: 보드를 그 방향으로 기울인 사진. [ ] 인 것을 채우세요
//   - 보드가 전부 보이고, 잠깐 멈춰 있고, 이전 사진과 충분히 다르면 자동 저장
//   - 아래쪽 NEXT 줄이 다음에 찍을 곳을 알려줌
// 키: q / ESC 종료,  s 지금 프레임 강제 저장

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <set>
#include <string>
#include <vector>

namespace fs = std::filesystem;

static const cv::Size CB(9, 6);        // 내부 코너
static const int GRID = 3;             // 화면을 3x3 칸으로 나눔
static const int ZONE_OK = 60;         // 칸에 코너가 이만큼 쌓이면 초록
static const double STABLE_PX = 1.5;   // 프레임 사이 코너 평균 이동이 이보다 작으면 "멈춤"
static const int STABLE_N = 4;         // 연속으로 멈춘 프레임 수
static const double NOVEL_TH = 0.12;   // 저장된 사진들과의 최소 차이
static const double COOLDOWN = 0.8;    // 저장 간격 (초)
static const double TILT_RATIO = 1.25; // 마주 보는 변 길이 비율이 이보다 크면 기울인 것으로 봄
static const int TARGET = 25;          // 최소 저장 장수

static const char* ZONE_NAMES[GRID][GRID] = {{"top-left", "top", "top-right"},
                                             {"left", "center", "right"},
                                             {"bottom-left", "bottom", "bottom-right"}};
static const std::array<std::string, 4> TILTS = {"tilt-left", "tilt-right", "tilt-up", "tilt-down"};

using Corners = std::vector<cv::Point2f>;
using Desc = std::array<double, 5>;

static double now_s() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

static std::string expand_home(const std::string& p) {
  if (!p.empty() && p[0] == '~') {
    const char* h = std::getenv("HOME");
    return std::string(h ? h : "") + p.substr(1);
  }
  return p;
}

// 반 크기에서 빠르게 찾고, 원본에서 서브픽셀로 다듬음. 못 찾으면 false.
static bool detect(const cv::Mat& gray, Corners& c) {
  cv::Mat small;
  cv::resize(gray, small, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
  int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK;
  if (!cv::findChessboardCorners(small, CB, c, flags)) return false;
  for (auto& p : c) p *= 2.0f;
  cv::cornerSubPix(gray, c, cv::Size(5, 5), cv::Size(-1, -1),
                   cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 40, 1e-3));
  return true;
}

static double dist(const cv::Point2f& a, const cv::Point2f& b) { return cv::norm(a - b); }

// 기울기 판정. lr = 왼변/오른변, tb = 윗변/아랫변 길이 비율
static std::set<std::string> tilt_of(const Corners& c, double* lr_out = nullptr, double* tb_out = nullptr) {
  const int W = CB.width, H = CB.height;
  cv::Point2f tl = c[0], tr = c[W - 1], br = c[(H - 1) * W + W - 1], bl = c[(H - 1) * W];
  double left = dist(bl, tl), right = dist(br, tr), top = dist(tr, tl), bottom = dist(br, bl);
  double lr = left / right, tb = top / bottom;
  if (lr_out) *lr_out = lr;
  if (tb_out) *tb_out = tb;
  std::set<std::string> found;
  if (lr > TILT_RATIO) found.insert("tilt-right");        // 왼쪽이 더 크게 보임 = 오른쪽이 멀어짐
  if (1.0 / lr > TILT_RATIO) found.insert("tilt-left");
  if (tb > TILT_RATIO) found.insert("tilt-down");
  if (1.0 / tb > TILT_RATIO) found.insert("tilt-up");
  return found;
}

// 사진끼리 얼마나 다른지 비교용: 중심 위치, 크기, 기울기
static Desc descriptor(const Corners& c, int w, int h) {
  Corners hull;
  cv::convexHull(c, hull);
  double area = cv::contourArea(hull) / (double(w) * h);
  cv::Point2f m(0, 0);
  for (auto& p : c) m += p;
  m *= 1.0f / c.size();
  double lr, tb;
  tilt_of(c, &lr, &tb);
  return {m.x / w, m.y / h, std::sqrt(area), std::log(lr), std::log(tb)};
}

static double desc_dist(const Desc& a, const Desc& b) {
  double s = 0;
  for (size_t i = 0; i < a.size(); ++i) s += (a[i] - b[i]) * (a[i] - b[i]);
  return std::sqrt(s);
}

static double sharpness(const cv::Mat& gray, const Corners& c) {
  cv::Rect r = cv::boundingRect(c) & cv::Rect(0, 0, gray.cols, gray.rows);
  if (r.area() == 0) return 0.0;
  cv::Mat lap;
  cv::Laplacian(gray(r), lap, CV_64F);
  cv::Scalar mean, sd;
  cv::meanStdDev(lap, mean, sd);
  return sd[0] * sd[0];
}

int main(int argc, char** argv) {
  std::string dev = "/dev/video8", out = "~/calib_wide2";
  double min_sharp = 80.0;  // 이보다 흐리면 저장 안 함
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--dev" && i + 1 < argc) dev = argv[++i];
    else if (a == "--out" && i + 1 < argc) out = argv[++i];
    else if (a == "--min-sharp" && i + 1 < argc) min_sharp = std::stod(argv[++i]);
    else { std::cerr << "사용법: " << argv[0] << " [--dev /dev/video8] [--out ~/calib_wide2] [--min-sharp 80]\n"; return 1; }
  }
  out = expand_home(out);
  fs::create_directories(out);

  cv::VideoCapture cap(dev, cv::CAP_V4L2);
  cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
  cap.set(cv::CAP_PROP_FPS, 30);
  if (!cap.isOpened()) {
    std::cerr << "카메라를 열 수 없어요: " << dev << " (다른 프로그램이 쓰는 중인지 확인)\n";
    return 1;
  }

  int zones[GRID][GRID] = {};
  std::set<std::string> tilts_done;
  std::vector<Desc> descs;
  int saved = 0, still = 0, flash = 0;
  double last_save = 0.0;
  Corners prev;
  bool have_prev = false;
  const auto font = cv::FONT_HERSHEY_SIMPLEX;

  cv::Mat frame, gray;
  while (true) {
    if (!cap.read(frame) || frame.empty()) { std::cerr << "프레임을 못 받았어요\n"; break; }
    const int w = frame.cols, h = frame.rows;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    Corners c;
    bool found = detect(gray, c);
    cv::Mat view = frame.clone();

    std::string status = "board: NOT FOUND (whole board must be visible)";
    cv::Scalar color(0, 0, 255);
    bool save_now = false;
    if (found) {
      double move = 99;
      if (have_prev && prev.size() == c.size()) {
        move = 0;
        for (size_t i = 0; i < c.size(); ++i) move += std::abs(c[i].x - prev[i].x) + std::abs(c[i].y - prev[i].y);
        move /= 2.0 * c.size();
      }
      still = move < STABLE_PX ? still + 1 : 0;
      prev = c;
      have_prev = true;
      Desc d = descriptor(c, w, h);
      double mind = 9;
      for (auto& e : descs) mind = std::min(mind, desc_dist(d, e));
      bool novel = mind > NOVEL_TH;
      double sharp = sharpness(gray, c);
      bool new_tilt = false;
      for (auto& t : tilt_of(c)) if (!tilts_done.count(t)) new_tilt = true;
      char buf[128];
      if (still < STABLE_N) { status = "board OK - hold still..."; color = {0, 200, 255}; }
      else if (sharp < min_sharp) { std::snprintf(buf, sizeof buf, "blurry (%.0f) - hold still", sharp); status = buf; color = {0, 200, 255}; }
      else if (!(novel || new_tilt)) { status = "same as before - move to a new place"; color = {0, 200, 255}; }
      else { status = "saving"; color = {0, 255, 0}; save_now = now_s() - last_save > COOLDOWN; }
      cv::drawChessboardCorners(view, CB, c, true);
    } else {
      have_prev = false;
      still = 0;
    }

    int key = cv::waitKey(1) & 0xFF;
    if (key == 'q' || key == 27) break;
    if (key == 's' && found) save_now = true;

    if (save_now && found) {
      ++saved;
      char name[64];
      std::snprintf(name, sizeof name, "frame_%03d.png", saved);
      cv::imwrite((fs::path(out) / name).string(), frame);
      descs.push_back(descriptor(c, w, h));
      for (auto& p : c) {
        int r = std::min(int(p.y / h * GRID), GRID - 1), q = std::min(int(p.x / w * GRID), GRID - 1);
        zones[std::max(r, 0)][std::max(q, 0)]++;
      }
      for (auto& t : tilt_of(c)) tilts_done.insert(t);
      last_save = now_s();
      still = 0;
      flash = 6;
      std::cout << "saved " << saved << "  tilts=";
      for (auto& t : tilts_done) std::cout << t << " ";
      std::cout << std::endl;
    }

    // --- 안내 표시 ---
    cv::Mat ov = view.clone();
    for (int r = 0; r < GRID; ++r)
      for (int q = 0; q < GRID; ++q) {
        int n = zones[r][q];
        cv::Scalar col = n == 0 ? cv::Scalar(0, 0, 255) : n < ZONE_OK ? cv::Scalar(0, 220, 255) : cv::Scalar(0, 200, 0);
        cv::rectangle(ov, cv::Point(q * w / GRID, r * h / GRID), cv::Point((q + 1) * w / GRID, (r + 1) * h / GRID), col, cv::FILLED);
      }
    cv::addWeighted(ov, 0.18, view, 0.82, 0, view);
    for (int r = 0; r < GRID; ++r)
      for (int q = 0; q < GRID; ++q) {
        cv::Point p0(q * w / GRID, r * h / GRID);
        cv::rectangle(view, p0, cv::Point((q + 1) * w / GRID, (r + 1) * h / GRID), {255, 255, 255}, 1);
        cv::putText(view, std::string(ZONE_NAMES[r][q]) + " " + std::to_string(zones[r][q]), p0 + cv::Point(8, 22),
                    font, 0.55, {255, 255, 255}, 2);
      }
    for (size_t k = 0; k < TILTS.size(); ++k) {
      bool done = tilts_done.count(TILTS[k]);
      cv::putText(view, std::string(done ? "[x] " : "[ ] ") + TILTS[k], cv::Point(w - 190, 60 + 26 * int(k)), font, 0.6,
                  done ? cv::Scalar(0, 200, 0) : cv::Scalar(0, 0, 255), 2);
    }

    // 덜 채워진 칸부터 안내
    std::vector<std::pair<int, std::string>> zl;
    for (int r = 0; r < GRID; ++r)
      for (int q = 0; q < GRID; ++q)
        if (zones[r][q] < ZONE_OK) zl.push_back({zones[r][q], ZONE_NAMES[r][q]});
    std::stable_sort(zl.begin(), zl.end(), [](auto& a, auto& b) { return a.first < b.first; });
    std::vector<std::string> todo;
    for (auto& z : zl) todo.push_back(z.second);
    for (auto& t : TILTS) if (!tilts_done.count(t)) todo.push_back(t);
    std::string nxt = "NEXT: ";
    if (todo.empty()) nxt += "all done!";
    for (size_t i = 0; i < std::min<size_t>(3, todo.size()); ++i) nxt += (i ? ", " : "") + todo[i];
    if (todo.empty() && saved >= TARGET) nxt = "DONE - press q";

    cv::rectangle(view, cv::Point(0, h - 70), cv::Point(w, h), {0, 0, 0}, cv::FILLED);
    cv::putText(view, "saved " + std::to_string(saved) + "/" + std::to_string(TARGET) + "   " + status,
                cv::Point(10, h - 42), font, 0.7, color, 2);
    cv::putText(view, nxt, cv::Point(10, h - 12), font, 0.7, {255, 255, 255}, 2);
    if (flash > 0) {
      cv::rectangle(view, cv::Point(0, 0), cv::Point(w - 1, h - 1), {0, 255, 0}, 12);
      --flash;
    }
    cv::imshow("calib capture", view);
  }

  cap.release();
  cv::destroyAllWindows();
  std::cout << "끝: " << saved << "장 저장 -> " << out << std::endl;
  return 0;
}
