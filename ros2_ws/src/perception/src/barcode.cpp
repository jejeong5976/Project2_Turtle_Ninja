// barcode.cpp — 바코드 판독기 · 노드 단계 구현, 설명은 barcode.hpp
//   PERCEPTION_HAVE_ZXING 이면 zxing-cpp, 없으면 OpenCV barcode 모듈

#include "barcode.hpp"

#include <algorithm>

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

  std::vector<std::pair<std::string, std::string>> decode(const cv::Mat & gray)
  {
    const cv::Mat g = gray.isContinuous() ? gray : gray.clone();
    const ZXing::ImageView iv(g.data, g.cols, g.rows, ZXing::ImageFormat::Lum, static_cast<int>(g.step));
    std::vector<std::pair<std::string, std::string>> out;
    for (const auto & r : ZXing::ReadBarcodes(iv, opts)) {
      if (r.isValid() && !r.text().empty()) {out.emplace_back(r.text(), ZXing::ToString(r.format()));}
    }
    return out;
  }
};

BarcodeReader::BarcodeReader()
: impl_(std::make_unique<Impl>()) {}

BarcodeReader::~BarcodeReader() = default;

std::vector<std::pair<std::string, std::string>> BarcodeReader::read(
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
    auto found = impl_->decode(gray);
    if (!found.empty()) {return found;}
    cv::Mat eq;
    impl_->clahe->apply(gray, eq);
    found = impl_->decode(eq);
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

  std::vector<std::pair<std::string, std::string>> decode(const cv::Mat & bgr)
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
    std::vector<std::pair<std::string, std::string>> out;
    if (!ok) {return out;}
    for (size_t i = 0; i < infos.size(); ++i) {
      if (!infos[i].empty()) {out.emplace_back(infos[i], i < names.size() ? names[i] : "");}
    }
    return out;
  }
};

BarcodeReader::BarcodeReader()
: impl_(std::make_unique<Impl>()) {}

BarcodeReader::~BarcodeReader() = default;

std::vector<std::pair<std::string, std::string>> BarcodeReader::read(
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
      auto found = impl_->decode(bgr);
      if (!found.empty()) {return found;}
    }
  }
  return {};
}

const char * BarcodeReader::backend() {return "opencv (EAN/UPC 만)";}

}  // namespace perception

#endif

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
  RCLCPP_INFO(node.get_logger(), "[barcode] %s 컬러 → /perception/task_text (%s, ~/barcode/enable, 판독 %s)",
    camera_.c_str(), enabled_ ? "켜짐" : "꺼짐", BarcodeReader::backend());
}

bool BarcodeStage::due() const
{
  return enabled_ && std::chrono::duration<double>(Clock::now() - last_).count() >= period_s_;
}

void BarcodeStage::process(const cv::Mat & bgr, const std_msgs::msg::Header & header, const std::vector<wide::Box> & boxes)
{
  last_ = Clock::now();
  if (reset_req_ != reset_done_) {
    reset_done_ = reset_req_;
    voter_.clear();
    last_text_.clear();
  }
  std::vector<std::pair<std::string, std::string>> found;
  for (const auto & b : boxes) {   // 광각 barcode 상자를 먼저 확대해서
    found = reader_.read(crop(bgr, b));
    if (!found.empty()) {break;}
  }
  if (found.empty() && full_frame_) {found = reader_.read(bgr);}
  // 못 읽은 장도 빈 값으로 넣음 → 바코드가 안 보이면 예전 표가 밀려나감
  const auto read = found.empty() ? std::pair<std::string, std::string>{} : found.front();
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
