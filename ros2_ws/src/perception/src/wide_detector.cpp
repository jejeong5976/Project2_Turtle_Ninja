// wide_detector.cpp — 광각 상자 검출 구현

#include "perception/wide_detector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

#ifdef PERCEPTION_HAVE_TRT
#include "perception/trt_engine.hpp"
#endif

namespace perception::wide
{

namespace
{
float box_iou(const Box & a, const Box & b)
{
  const float ix = std::max(0.f, std::min(a.x2, b.x2) - std::max(a.x1, b.x1));
  const float iy = std::max(0.f, std::min(a.y2, b.y2) - std::max(a.y1, b.y1));
  const float inter = ix * iy;
  const float ua = (a.x2 - a.x1) * (a.y2 - a.y1) + (b.x2 - b.x1) * (b.y2 - b.y1) - inter;
  return ua > 0 ? inter / ua : 0.f;
}
}  // namespace

cv::Mat letterbox(const cv::Mat & img, cv::Size size, LetterboxInfo & info)
{
  const double r = std::min(static_cast<double>(size.height) / img.rows, static_cast<double>(size.width) / img.cols);
  const int nw = static_cast<int>(std::round(img.cols * r)), nh = static_cast<int>(std::round(img.rows * r));
  const double dw = (size.width - nw) / 2.0, dh = (size.height - nh) / 2.0;
  cv::Mat resized;
  if (nw != img.cols || nh != img.rows) {
    cv::resize(img, resized, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
  } else {
    resized = img;
  }
  const int top = static_cast<int>(std::round(dh - 0.1)), bottom = static_cast<int>(std::round(dh + 0.1));
  const int left = static_cast<int>(std::round(dw - 0.1)), right = static_cast<int>(std::round(dw + 0.1));
  cv::Mat out;
  cv::copyMakeBorder(resized, out, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
  info.r = r;
  info.pad_x = left;
  info.pad_y = top;
  return out;
}

void to_chw(const cv::Mat & bgr, std::vector<float> & chw)
{
  const int H = bgr.rows, W = bgr.cols, plane = H * W;
  chw.resize(static_cast<size_t>(3 * plane));
  for (int y = 0; y < H; ++y) {
    const auto * p = bgr.ptr<cv::Vec3b>(y);
    for (int x = 0; x < W; ++x) {
      const int i = y * W + x;
      chw[i] = p[x][2] / 255.f;
      chw[plane + i] = p[x][1] / 255.f;
      chw[2 * plane + i] = p[x][0] / 255.f;
    }
  }
}

std::vector<BoxDetection> nms(std::vector<BoxDetection> dets, float iou)
{
  std::sort(dets.begin(), dets.end(), [](const BoxDetection & a, const BoxDetection & b) {return a.score > b.score;});
  std::vector<BoxDetection> keep;
  std::vector<bool> removed(dets.size(), false);
  for (size_t i = 0; i < dets.size(); ++i) {
    if (removed[i]) {continue;}
    keep.push_back(dets[i]);
    for (size_t j = i + 1; j < dets.size(); ++j) {
      if (!removed[j] && dets[j].cls == dets[i].cls && box_iou(dets[i].box, dets[j].box) > iou) {
        removed[j] = true;
      }
    }
  }
  return keep;
}

std::vector<BoxDetection> decode(
  const std::vector<float> & out, const std::vector<int64_t> & shape, const LetterboxInfo & lb, cv::Size orig,
  float conf, float iou)
{
  auto to_orig = [&](float x1, float y1, float x2, float y2) {
      Box b;
      b.x1 = std::clamp(static_cast<float>((x1 - lb.pad_x) / lb.r), 0.f, static_cast<float>(orig.width));
      b.y1 = std::clamp(static_cast<float>((y1 - lb.pad_y) / lb.r), 0.f, static_cast<float>(orig.height));
      b.x2 = std::clamp(static_cast<float>((x2 - lb.pad_x) / lb.r), 0.f, static_cast<float>(orig.width));
      b.y2 = std::clamp(static_cast<float>((y2 - lb.pad_y) / lb.r), 0.f, static_cast<float>(orig.height));
      return b;
    };

  std::vector<BoxDetection> dets;
  if (shape.size() == 3 && shape[2] == 6) {            // [1, 300, 6] end-to-end (NMS 없음)
    if (out.size() < static_cast<size_t>(shape[1] * 6)) {throw std::runtime_error("출력 크기가 모양과 다름");}
    for (int64_t i = 0; i < shape[1]; ++i) {
      const float * d = &out[static_cast<size_t>(i * 6)];
      if (d[4] < conf) {continue;}
      dets.push_back({to_orig(d[0], d[1], d[2], d[3]), d[4], static_cast<int>(d[5])});
    }
    return dets;
  }
  if (shape.size() != 3 || shape[1] < 5) {throw std::runtime_error("모르는 출력 모양");}
  const int64_t C = shape[1], N = shape[2];             // [1, 4+클래스, 후보]
  if (out.size() < static_cast<size_t>(C * N)) {throw std::runtime_error("출력 크기가 모양과 다름");}
  for (int64_t j = 0; j < N; ++j) {
    int best = -1;
    float s = conf;
    for (int64_t c = 4; c < C; ++c) {
      const float v = out[static_cast<size_t>(c * N + j)];
      if (v >= s) {s = v; best = static_cast<int>(c - 4);}
    }
    if (best < 0) {continue;}
    const float cx = out[j], cy = out[N + j], w = out[2 * N + j], h = out[3 * N + j];
    dets.push_back({to_orig(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2), s, best});
  }
  return nms(std::move(dets), iou);
}

#ifdef PERCEPTION_HAVE_TRT
BoxDetector::BoxDetector(const std::string & engine_path)
: engine_(std::make_unique<TrtEngine>(engine_path))
{
  if (engine_->outputs().size() != 1) {throw std::runtime_error("출력이 1 개인 상자 모델이 아님: " + engine_path);}
}

BoxDetector::~BoxDetector() = default;

const std::vector<std::string> & BoxDetector::names() const {return engine_->names();}

std::string BoxDetector::name(int cls) const
{
  const auto & n = engine_->names();
  return (cls >= 0 && cls < static_cast<int>(n.size())) ? n[cls] : std::to_string(cls);
}

cv::Size BoxDetector::input_size() const {return engine_->input_size();}

std::vector<BoxDetection> BoxDetector::detect(const cv::Mat & bgr, float conf, float iou)
{
  LetterboxInfo lb;
  to_chw(letterbox(bgr, engine_->input_size(), lb), chw_);
  engine_->infer(chw_, outs_);
  return decode(outs_[0], engine_->outputs()[0].shape, lb, bgr.size(), conf, iou);
}
#endif  // PERCEPTION_HAVE_TRT

}  // namespace perception::wide
