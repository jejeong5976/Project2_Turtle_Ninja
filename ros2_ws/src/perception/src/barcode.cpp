// barcode.cpp — 바코드 판독기 · 노드 단계 구현, 설명은 barcode.hpp
//   PERCEPTION_HAVE_ZXING 이면 zxing-cpp, 없으면 OpenCV barcode 모듈

#include "barcode.hpp"

#include <algorithm>
#include <filesystem>

#include <opencv2/imgcodecs.hpp>

#include <opencv2/core/version.hpp>
#include <opencv2/imgproc.hpp>

#include "msg_utils.hpp"

#ifdef PERCEPTION_HAVE_ZXING

#include <ZXing/ReadBarcode.h>

namespace perception
{

struct BarcodeReader::Impl
{
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
  ZXing::ReaderOptions opts;

  Impl()
  {
    opts.setFormats(ZXing::BarcodeFormat::LinearCodes);   // 1D 전부 (Code128 · Code39 · EAN …)
    opts.setTryHarder(true);
    opts.setTryRotate(true);                              // 비스듬 · 세로 바코드
    opts.setTryInvert(false);
  }

  std::vector<BarcodeFound> decode(const cv::Mat & gray, [[maybe_unused]] double scale)
  {
    const cv::Mat g = gray.isContinuous() ? gray : gray.clone();
    const ZXing::ImageView iv(g.data, g.cols, g.rows, ZXing::ImageFormat::Lum, static_cast<int>(g.step));
    std::vector<BarcodeFound> out;
    for (const auto & r : ZXing::ReadBarcodes(iv, opts)) {
      if (!r.isValid() || r.text().empty()) {continue;}
      BarcodeFound f;   // 자리: PERCEPTION_ZXING_POSITION (CMake 가 이 zxing 판에서 빌드되는지 확인) 이면 zxing 꼭짓점, 아니면 비워 둠 → locate_barcodes
      f.text = r.text();
      f.format = ZXing::ToString(r.format());
#ifdef PERCEPTION_ZXING_POSITION
      {
        const auto & pos = r.position();
        float x1 = 1e9f, y1 = 1e9f, x2 = -1e9f, y2 = -1e9f;
        for (int i = 0; i < 4; ++i) {
          x1 = std::min(x1, static_cast<float>(pos[i].x));
          y1 = std::min(y1, static_cast<float>(pos[i].y));
          x2 = std::max(x2, static_cast<float>(pos[i].x));
          y2 = std::max(y2, static_cast<float>(pos[i].y));
        }
        const float k = static_cast<float>(1.0 / scale);
        if (x2 > x1) {f.bbox = {x1 * k, y1 * k, x2 * k, y2 * k};}
      }
#endif
      out.push_back(std::move(f));
    }
    return out;
  }
};

BarcodeReader::BarcodeReader()
: impl_(std::make_unique<Impl>()) {}

BarcodeReader::~BarcodeReader() = default;

std::vector<BarcodeFound> BarcodeReader::read_located(
  const cv::Mat & img_bgr, const std::vector<double> & scales) const
{
  cv::Mat gray0;
  if (img_bgr.channels() == 3) {
    cv::cvtColor(img_bgr, gray0, cv::COLOR_BGR2GRAY);
  } else {
    gray0 = img_bgr;
  }
  for (double s : scales) {
    cv::Mat gray;
    if (s == 1.0) {
      gray = gray0;
    } else {
      cv::resize(gray0, gray, cv::Size(), s, s, cv::INTER_AREA);
    }
    // 원본 → CLAHE (어두운 · 반사 조명). zxing 이 안에서 이진화 · 다시 시도까지 함
    auto found = impl_->decode(gray, s);
    if (!found.empty()) {return found;}
    cv::Mat eq;
    impl_->clahe->apply(gray, eq);
    found = impl_->decode(eq, s);
    if (!found.empty()) {return found;}
  }
  return {};
}

const char * BarcodeReader::backend() {return "zxing-cpp";}

}  // namespace perception

#else   // ---------------------------------------------------------------- OpenCV (zxing 없을 때)

// OpenCV 4.8 부터 barcode 가 objdetect 로 옮겨짐 (그 전은 contrib 모듈)
#define PERCEPTION_CV_AT_LEAST(maj, min) \
  (CV_VERSION_MAJOR > (maj) || (CV_VERSION_MAJOR == (maj) && CV_VERSION_MINOR >= (min)))

#if PERCEPTION_CV_AT_LEAST(4, 8)
#include <opencv2/objdetect/barcode.hpp>
#else
#include <opencv2/barcode.hpp>
#endif

namespace perception
{

struct BarcodeReader::Impl
{
  cv::barcode::BarcodeDetector det;
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));

  std::vector<BarcodeFound> decode(const cv::Mat & bgr, double scale)
  {
    std::vector<std::string> infos;
    std::vector<std::string> names;
    cv::Mat points;
#if PERCEPTION_CV_AT_LEAST(4, 8)
    const bool ok = det.detectAndDecodeWithType(bgr, infos, names, points);
#else
    std::vector<cv::barcode::BarcodeType> types;
    const bool ok = det.detectAndDecode(bgr, infos, types, points);
    for (auto t : types) {   // OpenCV 4.6~4.7 은 숫자 코드로 돌려줌
      switch (static_cast<int>(t)) {
        case 1: names.emplace_back("EAN_8"); break;
        case 2: names.emplace_back("EAN_13"); break;
        case 3: names.emplace_back("UPC_A"); break;
        case 4: names.emplace_back("UPC_E"); break;
        case 5: names.emplace_back("UPC_EAN_EXTENSION"); break;
        default: names.emplace_back(""); break;
      }
    }
#endif
    std::vector<BarcodeFound> out;
    if (!ok) {return out;}
    const cv::Mat pts = points.reshape(2, static_cast<int>(points.total() * points.channels() / 2));   // 바코드마다 꼭짓점 4 개 (Point2f)
    for (size_t i = 0; i < infos.size(); ++i) {
      if (infos[i].empty()) {continue;}
      BarcodeFound f;
      f.text = infos[i];
      f.format = i < names.size() ? names[i] : "";
      if (pts.type() == CV_32FC2 && static_cast<int>(i * 4 + 3) < pts.rows) {
        float x1 = 1e9f, y1 = 1e9f, x2 = -1e9f, y2 = -1e9f;
        for (int k = 0; k < 4; ++k) {
          const auto p = pts.at<cv::Point2f>(static_cast<int>(i * 4) + k);
          x1 = std::min(x1, p.x); y1 = std::min(y1, p.y); x2 = std::max(x2, p.x); y2 = std::max(y2, p.y);
        }
        const float k = static_cast<float>(1.0 / scale);
        f.bbox = {x1 * k, y1 * k, x2 * k, y2 * k};
      }
      out.push_back(std::move(f));
    }
    return out;
  }
};

BarcodeReader::BarcodeReader()
: impl_(std::make_unique<Impl>()) {}

BarcodeReader::~BarcodeReader() = default;

std::vector<BarcodeFound> BarcodeReader::read_located(
  const cv::Mat & img_bgr, const std::vector<double> & scales) const
{
  cv::Mat gray0;
  if (img_bgr.channels() == 3) {
    cv::cvtColor(img_bgr, gray0, cv::COLOR_BGR2GRAY);
  } else {
    gray0 = img_bgr;
  }
  for (double s : scales) {
    cv::Mat gray;
    if (s == 1.0) {
      gray = gray0;
    } else {
      cv::resize(gray0, gray, cv::Size(), s, s, cv::INTER_AREA);
    }
    for (int step = 0; step < 4; ++step) {   // 원본 → CLAHE → 샤프닝 → 이진화
      cv::Mat g;
      if (step == 0) {
        g = gray;
      } else if (step == 1) {
        impl_->clahe->apply(gray, g);
      } else if (step == 2) {
        cv::Mat blur;
        cv::GaussianBlur(gray, blur, cv::Size(0, 0), 3);
        cv::addWeighted(gray, 1.5, blur, -0.5, 0, g);
      } else {
        cv::adaptiveThreshold(gray, g, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 31, 5);
      }
      cv::Mat bgr;
      cv::cvtColor(g, bgr, cv::COLOR_GRAY2BGR);
      auto found = impl_->decode(bgr, s);
      if (!found.empty()) {return found;}
    }
  }
  return {};
}

const char * BarcodeReader::backend() {return "opencv (EAN/UPC 만)";}

}  // namespace perception

#endif

// ================================================================ 공통: read · 자리만 찾기

#if defined(__has_include)
#if __has_include(<opencv2/objdetect/barcode.hpp>)
#include <opencv2/objdetect/barcode.hpp>
#define PERCEPTION_HAVE_CV_BARCODE 1
#elif __has_include(<opencv2/barcode.hpp>)
#include <opencv2/barcode.hpp>
#define PERCEPTION_HAVE_CV_BARCODE 1
#endif
#endif

namespace perception
{

std::vector<std::pair<std::string, std::string>> BarcodeReader::read(
  const cv::Mat & img_bgr, const std::vector<double> & scales) const
{
  std::vector<std::pair<std::string, std::string>> out;
  for (const auto & f : read_located(img_bgr, scales)) {out.emplace_back(f.text, f.format);}
  return out;
}

std::vector<LabelBox> locate_labels(const cv::Mat & img_bgr, const LabelParams & p, double fx,
  const std::function<double(double, double)> & depth_at)
{
  std::vector<LabelBox> out;
  if (img_bgr.empty() || img_bgr.channels() != 3) {return out;}
  cv::Mat hsv, gray;
  cv::cvtColor(img_bgr, hsv, cv::COLOR_BGR2HSV);
  cv::cvtColor(img_bgr, gray, cv::COLOR_BGR2GRAY);
  std::vector<cv::Mat> ch;
  cv::split(hsv, ch);
  cv::Mat vbin;
  const double otsu = cv::threshold(ch[2], vbin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  const double vt = std::max(otsu, p.v_min);
  cv::Mat mask = (ch[2] > vt) & (ch[1] < p.s_max);
  // 막대 (검은 줄) 사이를 메움 → 라벨 하나의 덩어리
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9)));
  std::vector<std::vector<cv::Point>> cs;
  cv::findContours(mask, cs, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  const double img_area = static_cast<double>(img_bgr.cols) * img_bgr.rows;
  for (const auto & c : cs) {
    const double area = cv::contourArea(c);
    if (area < p.min_area_px || area > 0.6 * img_area) {continue;}
    const cv::RotatedRect rr = cv::minAreaRect(c);
    const double lw = std::max(rr.size.width, rr.size.height), sw = std::min(rr.size.width, rr.size.height);
    if (sw < 4.0) {continue;}
    const double aspect = lw / sw;
    if (aspect < p.aspect_min || aspect > p.aspect_max) {continue;}
    if (area < 0.6 * lw * sw) {continue;}   // 사각처럼 꽉 참
    const double z = depth_at ? depth_at(rr.center.x, rr.center.y) : 0.0;
    if (z > 0.05 && fx > 0.0) {   // 깊이로 본 기대 크기
      const double expect = fx * p.width_m / z;
      if (lw < p.size_lo * expect || lw > p.size_hi * expect) {continue;}
    }
    // 막대 무늬: 긴 축 따라 3 줄 (가운데 · ±1/4 짧은 변) 밝기 차 부호 바뀜
    const double ang = (rr.size.width >= rr.size.height ? rr.angle : rr.angle + 90.0) * CV_PI / 180.0;
    const cv::Point2d ax(std::cos(ang), std::sin(ang)), nx(-std::sin(ang), std::cos(ang));
    double changes = 0.0;
    for (double off : {-0.25, 0.0, 0.25}) {
      int n = 0, last = 0;
      int prev = -1;
      const int samples = static_cast<int>(lw * 0.9);
      for (int k = 0; k < samples; ++k) {
        const double t = -0.45 * lw + k;
        const cv::Point2d q = cv::Point2d(rr.center) + ax * t + nx * (off * sw);
        const int x = static_cast<int>(std::lround(q.x)), y = static_cast<int>(std::lround(q.y));
        if (x < 0 || y < 0 || x >= gray.cols || y >= gray.rows) {continue;}
        const int g = gray.at<uint8_t>(y, x);
        if (prev >= 0) {
          const int d = g - prev;
          const int sg = d > 8 ? 1 : (d < -8 ? -1 : 0);
          if (sg != 0 && last != 0 && sg != last) {++n;}
          if (sg != 0) {last = sg;}
        }
        prev = g;
      }
      changes += n / 3.0;
    }
    if (changes < p.min_changes) {continue;}
    cv::Point2f pts[4];
    rr.points(pts);
    float x1 = 1e9f, y1 = 1e9f, x2 = -1e9f, y2 = -1e9f;
    for (const auto & q : pts) {x1 = std::min(x1, q.x); y1 = std::min(y1, q.y); x2 = std::max(x2, q.x); y2 = std::max(y2, q.y);}
    out.push_back(LabelBox{{x1, y1, x2, y2}, changes});
  }
  std::sort(out.begin(), out.end(), [](const LabelBox & a, const LabelBox & b) {return a.score > b.score;});
  return out;
}

bool barcode_locator_available()
{
#ifdef PERCEPTION_HAVE_CV_BARCODE
  return true;
#else
  return false;
#endif
}

std::vector<std::array<float, 4>> locate_barcodes(const cv::Mat & img_bgr)
{
  std::vector<std::array<float, 4>> out;
#ifdef PERCEPTION_HAVE_CV_BARCODE
  try {
    static thread_local cv::barcode::BarcodeDetector det;
    cv::Mat points;
    if (!det.detect(img_bgr, points) || points.empty()) {return out;}
    const cv::Mat pts = points.reshape(2, static_cast<int>(points.total() * points.channels() / 2));
    if (pts.type() != CV_32FC2) {return out;}
    for (int i = 0; i + 3 < pts.rows; i += 4) {
      float x1 = 1e9f, y1 = 1e9f, x2 = -1e9f, y2 = -1e9f;
      for (int k = 0; k < 4; ++k) {
        const auto p = pts.at<cv::Point2f>(i + k);
        x1 = std::min(x1, p.x); y1 = std::min(y1, p.y); x2 = std::max(x2, p.x); y2 = std::max(y2, p.y);
      }
      out.push_back({x1, y1, x2, y2});
    }
  } catch (const cv::Exception &) {
    out.clear();
  }
#else
  (void)img_bgr;
#endif
  return out;
}

}  // namespace perception

// ================================================================ 노드 단계 (BarcodeStage)

namespace perception
{

BarcodeStage::BarcodeStage(rclcpp::Node & node)
: node_(node),
  voter_(static_cast<int>(node.declare_parameter<int64_t>("barcode.window", 5)),
    static_cast<int>(node.declare_parameter<int64_t>("barcode.need", 3)))
{
  camera_ = node.declare_parameter<std::string>("barcode.camera", "wrist");
  enabled_ = node.declare_parameter<bool>("barcode.enabled", false);   // planning 이 READ_BARCODE 에서 켬
  period_s_ = 1.0 / std::max(node.declare_parameter<double>("barcode.max_hz", 10.0), 0.1);
  roi_class_ = node.declare_parameter<std::string>("barcode.class", "barcode");
  roi_pad_ = node.declare_parameter<double>("barcode.roi_pad", 0.3);
  roi_scale_ = node.declare_parameter<double>("barcode.roi_scale", 2.0);
  full_frame_ = node.declare_parameter<bool>("barcode.full_frame", true);
  if (camera_ != "wide" && camera_ != "wrist") {
    RCLCPP_ERROR(node.get_logger(), "barcode.camera=%s 모름 (wide | wrist) — 바코드 판독 안 함", camera_.c_str());
  }
  pub_ = node.create_publisher<interfaces::msg::TaskText>("/perception/task_text", 10);
  // 장마다 바코드 자리 (읽음 · 자리만). "" = 안 냄
  const auto det_topic = node.declare_parameter<std::string>("barcode.detections_topic", "/perception/barcode/detections");
  locate_ = node.declare_parameter<bool>("barcode.locate", true);
  // 흰 라벨 자리 찾기 (cv::barcode 가 못 찾을 때 · 받는 쪽이 있을 때만)
  label_locate_ = node.declare_parameter<bool>("barcode.label_locate", true);
  {
    const auto snap = node.declare_parameter<std::string>("barcode.snapshot_topic", "/perception/barcode/snapshot");
    if (!snap.empty()) {
      snap_sub_ = node.create_subscription<std_msgs::msg::String>(snap, 10, [this](std_msgs::msg::String::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lk(snap_mtx_);
          snap_path_ = m->data;
        });
    }
  }
  label_.s_max = node.declare_parameter<double>("barcode.label_s_max", label_.s_max);
  label_.v_min = node.declare_parameter<double>("barcode.label_v_min", label_.v_min);
  label_.aspect_min = node.declare_parameter<double>("barcode.label_aspect_min", label_.aspect_min);
  label_.aspect_max = node.declare_parameter<double>("barcode.label_aspect_max", label_.aspect_max);
  label_.min_area_px = node.declare_parameter<double>("barcode.label_min_area_px", label_.min_area_px);
  label_.width_m = node.declare_parameter<double>("barcode.label_width_m", label_.width_m);
  label_.min_changes = node.declare_parameter<double>("barcode.label_min_changes", label_.min_changes);
  if (!det_topic.empty()) {det_pub_ = node.create_publisher<interfaces::msg::DetectionArray>(det_topic, 10);}
#if defined(PERCEPTION_HAVE_ZXING) && defined(PERCEPTION_ZXING_POSITION)
  const char * zx_pos = "읽은 자리 = zxing 꼭짓점";
#elif defined(PERCEPTION_HAVE_ZXING)
  const char * zx_pos = "읽은 자리 = OpenCV 자리 찾기 (이 zxing 판은 꼭짓점 안 씀)";
#else
  const char * zx_pos = "읽은 자리 = OpenCV 판독기";
#endif
  RCLCPP_INFO(node.get_logger(), "[barcode] 자리 출력 %s · 자리만 찾기 (cv::barcode) %s · %s", det_topic.empty() ? "끔" : det_topic.c_str(),
    barcode_locator_available() ? "있음" : "없음 — 못 읽은 바코드의 자리는 안 나옴 (미션 다시 겨눔 · 흔들기 줄어듦)", zx_pos);
  // 손목 깊이 (camera wrist 만): 판독이 켜져 있는 동안만 구독 → 바코드 상자 · 화면 가운데 3D 점 (벽 거리 1 순위)
  depth_topic_ = node.declare_parameter<std::string>("barcode.depth_topic", "/camera/camera/aligned_depth_to_color/image_raw");
  info_topic_ = node.declare_parameter<std::string>("barcode.info_topic", "/camera/camera/color/camera_info");
  depth_scale_ = node.declare_parameter<double>("barcode.depth_scale", 0.001);
  if (camera_ != "wrist") {depth_topic_.clear();}
  RCLCPP_INFO(node.get_logger(), "[barcode] %s 컬러 → /perception/task_text (%s, ~/barcode/enable, 판독 %s)",
    camera_.c_str(), enabled_ ? "켜짐" : "꺼짐", BarcodeReader::backend());
}

void BarcodeStage::set_enabled(bool on)
{
  enabled_ = on;
  ++reset_req_;
  if (depth_topic_.empty() || !det_pub_) {return;}
  std::lock_guard<std::mutex> lk(depth_mtx_);
  if (on && !depth_sub_) {
    depth_sub_ = node_.create_subscription<sensor_msgs::msg::Image>(depth_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr m) {std::lock_guard<std::mutex> l(depth_mtx_); depth_ = m;});
    info_sub_ = node_.create_subscription<sensor_msgs::msg::CameraInfo>(info_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {std::lock_guard<std::mutex> l(depth_mtx_); info_ = m;});
  } else if (!on) {
    depth_sub_.reset();
    info_sub_.reset();
    depth_.reset();
  }
}

// 상자 (x1 y1 x2 y2) 가운데 절반 안 깊이 가운데값 → 광학 프레임 점. 깊이 · 내부값 없으면 false
bool BarcodeStage::depth_point(const std::array<float, 4> & bb, geometry_msgs::msg::Point & out) const
{
  sensor_msgs::msg::Image::ConstSharedPtr d;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr k;
  {
    std::lock_guard<std::mutex> lk(depth_mtx_);
    d = depth_;
    k = info_;
  }
  if (!d || !k || d->encoding != "16UC1" || k->k[0] <= 0.0 || k->k[4] <= 0.0) {return false;}
  const double cx = 0.5 * (bb[0] + bb[2]), cy = 0.5 * (bb[1] + bb[3]);
  const double hw = std::max(2.0, 0.25 * (bb[2] - bb[0])), hh = std::max(2.0, 0.25 * (bb[3] - bb[1]));
  const int x0 = std::max(0, static_cast<int>(cx - hw)), x1 = std::min(static_cast<int>(d->width) - 1, static_cast<int>(cx + hw));
  const int y0 = std::max(0, static_cast<int>(cy - hh)), y1 = std::min(static_cast<int>(d->height) - 1, static_cast<int>(cy + hh));
  std::vector<uint16_t> v;
  for (int y = y0; y <= y1; ++y) {
    const auto * row = reinterpret_cast<const uint16_t *>(d->data.data() + static_cast<std::size_t>(y) * d->step);
    for (int x = x0; x <= x1; ++x) {if (row[x] > 0) {v.push_back(row[x]);}}
  }
  if (v.size() < 5) {return false;}
  std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
  const double z = v[v.size() / 2] * depth_scale_;
  out.x = (cx - k->k[2]) / k->k[0] * z;
  out.y = (cy - k->k[5]) / k->k[4] * z;
  out.z = z;
  return true;
}

bool BarcodeStage::due() const
{
  // 10 ms 여유: 영상 도착 간격 흔들림으로 한 장 더 건너뛰지 않게 (30 Hz 카메라 + 10 Hz → 실제 7.5 Hz 방지)
  return enabled_ && std::chrono::duration<double>(Clock::now() - last_).count() >= period_s_ - 0.010;
}

void BarcodeStage::process(const cv::Mat & bgr, const std_msgs::msg::Header & header, const std::vector<wide::Box> & boxes)
{
  last_ = Clock::now();
  {   // 사진 요청 (판독이 켜져 있을 때 다음 장)
    std::string path;
    {
      std::lock_guard<std::mutex> lk(snap_mtx_);
      path.swap(snap_path_);
    }
    if (!path.empty()) {
      std::error_code ec;
      std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ec);
      try {
        if (!cv::imwrite(path, bgr)) {RCLCPP_WARN(node_.get_logger(), "[barcode] 사진 못 씀: %s", path.c_str());}
      } catch (const cv::Exception & e) {
        RCLCPP_WARN(node_.get_logger(), "[barcode] 사진 못 씀: %s (%s)", path.c_str(), e.what());
      }
    }
  }
  if (reset_req_ != reset_done_) {
    reset_done_ = reset_req_;
    voter_.clear();
    last_text_.clear();
  }
  std::vector<BarcodeFound> found;
  for (const auto & b : boxes) {   // 광각 barcode 상자를 먼저 확대해서
    found = reader_.read_located(crop(bgr, b));
    if (!found.empty()) {
      for (auto & f : found) {f.bbox = {b.x1, b.y1, b.x2, b.y2};}   // 자리 = 광각 상자 (잘라 키운 화소 아님)
      break;
    }
  }
  if (found.empty() && full_frame_) {found = reader_.read_located(bgr);}
  if (det_pub_) {   // 자리 출력: 읽었으면 그 상자, 아니면 (locate) 검출기 상자
    interfaces::msg::DetectionArray da;
    da.header = header;
    auto add = [&](const std::string & text, const std::array<float, 4> & bb, float conf) {
        interfaces::msg::Detection d;
        d.header = header;
        d.kind = "barcode";
        d.label = text;
        d.confidence = conf;
        d.bbox = {bb[0], bb[1], bb[2], bb[3]};
        d.yaw_known = false;
        set_unknown_pose(d.pose);
        da.detections.push_back(std::move(d));
      };
    std::vector<std::array<float, 4>> located;
    const bool need_loc = found.empty() || found.front().bbox[2] <= found.front().bbox[0];   // 못 읽음 · 읽었지만 자리 모름 (zxing)
    // 자리만 찾기는 받는 쪽이 있을 때만 (미션 barcode_search 가 구독 — 꺼져 있으면 CPU 안 씀)
    if (locate_ && need_loc && det_pub_->get_subscription_count() > 0) {located = locate_barcodes(bgr);}
    for (auto f : found) {
      if (f.bbox[2] <= f.bbox[0] && !located.empty()) {f.bbox = located.front();}
      add(f.text, f.bbox, 1.0f);
    }
    if (found.empty()) {
      for (const auto & bb : located) {add("", bb, 0.5f);}
    }
    if (found.empty() && located.empty() && label_locate_ && det_pub_->get_subscription_count() > 0) {   // 흰 라벨
      double fx = 606.32;
      {
        std::lock_guard<std::mutex> lk(depth_mtx_);
        if (info_ && info_->k[0] > 0.0) {fx = info_->k[0];}
      }
      auto depth_at = [this](double u, double v) {
          geometry_msgs::msg::Point q;
          const float fu = static_cast<float>(u), fv = static_cast<float>(v);
          return depth_point({fu - 6.f, fv - 6.f, fu + 6.f, fv + 6.f}, q) ? q.z : 0.0;
        };
      const auto labels = locate_labels(bgr, label_, fx, depth_at);
      if (!labels.empty()) {add("", labels.front().bbox, 0.4f);}
    }
    for (auto & d : da.detections) {   // 손목 깊이 3D 점 (광학 프레임) — 없으면 NaN 그대로
      geometry_msgs::msg::Point p;
      if (d.bbox[2] > d.bbox[0] && depth_point({d.bbox[0], d.bbox[1], d.bbox[2], d.bbox[3]}, p)) {d.pose.position = p;}
    }
    {   // 화면 가운데 40×40 깊이 = 벽 자리 (kind "wall")
      geometry_msgs::msg::Point p;
      const float cx = 0.5f * bgr.cols, cy = 0.5f * bgr.rows;
      if (depth_point({cx - 40.f, cy - 40.f, cx + 40.f, cy + 40.f}, p)) {
        add("", {cx - 20.f, cy - 20.f, cx + 20.f, cy + 20.f}, 0.0f);
        da.detections.back().kind = "wall";
        da.detections.back().pose.position = p;
      }
    }
    det_pub_->publish(da);
  }
  // 못 읽은 장도 빈 값으로 넣음 → 바코드가 안 보이면 지난 표가 밀려나감
  const auto read = found.empty() ? std::pair<std::string, std::string>{} :
    std::pair<std::string, std::string>{found.front().text, found.front().format};
  const auto value = voter_.push(read).first;
  // 이번 장에서 읽은 값이 확정값과 같을 때만 보냄 (A 확정 뒤 B 를 읽은 장에 A 를 내지 않음)
  if (!value || value->first.empty() || read != *value) {return;}

  interfaces::msg::TaskText out;
  out.header = header;
  out.raw_text = value->first;
  set_unknown_pose(out.pose);   // 바코드 위치는 아직 안 구함
  pub_->publish(out);
  if (value->first != last_text_) {
    last_text_ = value->first;
    RCLCPP_INFO(node_.get_logger(), "[barcode] %s (%s)", value->first.c_str(), value->second.c_str());
  }
}

// 상자를 roi_pad 만큼 넓혀 자르고 roi_scale 배 키움 (광각에서는 바코드가 작아서 확대가 중요)
cv::Mat BarcodeStage::crop(const cv::Mat & img, const wide::Box & b) const
{
  const double cx = (b.x1 + b.x2) / 2, cy = (b.y1 + b.y2) / 2;
  const double w = (b.x2 - b.x1) * (1 + 2 * roi_pad_), h = (b.y2 - b.y1) * (1 + 2 * roi_pad_);
  const int x1 = static_cast<int>(std::max(0.0, cx - w / 2)), y1 = static_cast<int>(std::max(0.0, cy - h / 2));
  const int x2 = static_cast<int>(std::min(static_cast<double>(img.cols), cx + w / 2));
  const int y2 = static_cast<int>(std::min(static_cast<double>(img.rows), cy + h / 2));
  if (x2 <= x1 || y2 <= y1) {return img;}
  cv::Mat out;
  cv::resize(img(cv::Range(y1, y2), cv::Range(x1, x2)), out, cv::Size(), roi_scale_, roi_scale_, cv::INTER_CUBIC);
  return out;
}

}  // namespace perception
