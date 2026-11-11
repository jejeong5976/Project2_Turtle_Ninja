// wrist_seg.cpp — 손목 퍽 분할 구현, 설명은 wrist_seg.hpp

#include "perception/wrist_seg.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

#ifdef PERCEPTION_HAVE_TRT
#include "perception/trt_engine.hpp"
#endif

namespace perception::wrist
{

namespace
{
// Python round (짝수 쪽 반올림) 과 같음
int pyround(double x) {return static_cast<int>(std::nearbyint(x));}
}  // namespace

Letterbox letterbox(const cv::Mat & bgr, cv::Size in, std::vector<float> & chw)
{
  const int h0 = bgr.rows, w0 = bgr.cols;
  const double r = std::min(static_cast<double>(in.height) / h0, static_cast<double>(in.width) / w0);
  Letterbox lb;
  lb.new_w = std::max(pyround(w0 * r), 1);
  lb.new_h = std::max(pyround(h0 * r), 1);
  const double dw = (in.width - lb.new_w) / 2.0, dh = (in.height - lb.new_h) / 2.0;
  lb.pad_x = pyround(dw - 0.1);
  lb.pad_y = pyround(dh - 0.1);
  lb.gain_x = static_cast<double>(lb.new_w) / w0;
  lb.gain_y = static_cast<double>(lb.new_h) / h0;
  cv::Mat img = bgr;
  if (lb.new_w != w0 || lb.new_h != h0) {cv::resize(bgr, img, cv::Size(lb.new_w, lb.new_h), 0, 0, cv::INTER_LINEAR);}
  cv::Mat padded;
  cv::copyMakeBorder(img, padded, lb.pad_y, in.height - lb.new_h - lb.pad_y, lb.pad_x, in.width - lb.new_w - lb.pad_x,
    cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
  // HWC BGR uint8 → CHW RGB float /255
  const size_t plane = static_cast<size_t>(in.width) * in.height;
  chw.resize(3 * plane);
  for (int y = 0; y < in.height; ++y) {
    const cv::Vec3b * row = padded.ptr<cv::Vec3b>(y);
    for (int x = 0; x < in.width; ++x) {
      const size_t i = static_cast<size_t>(y) * in.width + x;
      chw[i] = row[x][2] / 255.0f;
      chw[plane + i] = row[x][1] / 255.0f;
      chw[2 * plane + i] = row[x][0] / 255.0f;
    }
  }
  return lb;
}

std::vector<SegInstance> postprocess(
  const SegOutputs & o, const Letterbox & lb, cv::Size orig, const std::vector<std::string> & names,
  float conf, float iou, int max_det)
{
  const int nc = o.n_ch - 4 - o.nm;
  if (nc <= 0) {throw std::runtime_error("output0 채널 수가 4 + nc + nm 이 아님");}
  // 앵커 a 의 채널 k 값 (두 배치 모두)
  auto at = [&o](int a, int k) {
      return o.channels_first ? o.det[static_cast<size_t>(k) * o.n_anchor + a] : o.det[static_cast<size_t>(a) * o.n_ch + k];
    };
  struct Cand {float s; int a, c; std::array<float, 4> b;};   // b = 입력 픽셀 xyxy
  std::vector<Cand> cand;
  if (o.channels_first) {
    // one2many: 앵커마다 최고 클래스 → conf 거르기 → 점수 순 → 클래스별 NMS (상자에 클래스 × 7680 더해 섞이지 않게)
    for (int a = 0; a < o.n_anchor; ++a) {
      int bc = 0;
      float bs = at(a, 4);
      for (int c = 1; c < nc; ++c) {
        if (at(a, 4 + c) > bs) {bs = at(a, 4 + c); bc = c;}
      }
      if (!(bs > conf)) {continue;}
      const float cx = at(a, 0), cy = at(a, 1), w = at(a, 2), h = at(a, 3);
      cand.push_back({bs, a, bc, {cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2}});
    }
    std::stable_sort(cand.begin(), cand.end(), [](const Cand & x, const Cand & y) {return x.s > y.s;});
    if (cand.size() > 30000) {cand.resize(30000);}
    std::vector<Cand> kept;
    std::vector<bool> gone(cand.size(), false);
    for (size_t i = 0; i < cand.size() && static_cast<int>(kept.size()) < max_det; ++i) {
      if (gone[i]) {continue;}
      kept.push_back(cand[i]);
      const auto & A = cand[i];
      const float area_a = (A.b[2] - A.b[0]) * (A.b[3] - A.b[1]);
      for (size_t j = i + 1; j < cand.size(); ++j) {
        if (gone[j] || cand[j].c != A.c) {continue;}           // 다른 클래스는 offset 때문에 안 겹침
        const auto & B = cand[j];
        const float iw = std::max(0.0f, std::min(A.b[2], B.b[2]) - std::max(A.b[0], B.b[0]));
        const float ih = std::max(0.0f, std::min(A.b[3], B.b[3]) - std::max(A.b[1], B.b[1]));
        const float inter = iw * ih;
        const float uni = area_a + (B.b[2] - B.b[0]) * (B.b[3] - B.b[1]) - inter;
        if (inter / uni > iou) {gone[j] = true;}
      }
    }
    cand = std::move(kept);
  } else {
    // one2one: 점수 > conf 인 (앵커, 클래스), 점수 순 (NMS 없음)
    for (int a = 0; a < o.n_anchor; ++a) {
      for (int c = 0; c < nc; ++c) {
        if (at(a, 4 + c) > conf) {cand.push_back({at(a, 4 + c), a, c, {at(a, 0), at(a, 1), at(a, 2), at(a, 3)}});}
      }
    }
    std::stable_sort(cand.begin(), cand.end(), [](const Cand & x, const Cand & y) {return x.s > y.s;});
    if (static_cast<int>(cand.size()) > max_det) {cand.resize(max_det);}
  }

  // 마스크 원형 → 원본 크기 변환 준비 (ultralytics process_mask_native · scale_masks)
  const int h0 = orig.height, w0 = orig.width;
  const double g4 = std::min(4.0 * o.mh / h0, 4.0 * o.mw / w0);
  const int pad4 = (4 * o.mh - std::max(pyround(h0 * g4), 1)) | (4 * o.mw - std::max(pyround(w0 * g4), 1));
  const int f = 4 / std::gcd(4, pad4 | (pad4 / 2));          // 여백을 정수 화소로 자르는 가장 작은 확대 (1, 2, 4)
  const int ih = o.mh * f, iw = o.mw * f;
  const double gm = std::min(static_cast<double>(ih) / h0, static_cast<double>(iw) / w0);
  const int top = pyround((ih - std::max(pyround(h0 * gm), 1)) / 2.0 - 0.1);
  const int left = pyround((iw - std::max(pyround(w0 * gm), 1)) / 2.0 - 0.1);
  const cv::Rect content(left, top, std::max(pyround(w0 * gm), 1), std::max(pyround(h0 * gm), 1));
  const cv::Mat1f protos(o.nm, o.mh * o.mw, const_cast<float *>(o.proto.data()));

  std::vector<SegInstance> out;
  for (const auto & c : cand) {
    SegInstance s;
    s.cls = c.c;
    s.kind = c.c < static_cast<int>(names.size()) ? names[c.c] : std::to_string(c.c);
    s.conf = c.s;
    // 3) 상자 → 원본 좌표
    const float xs[2] = {c.b[0], c.b[2]}, ys[2] = {c.b[1], c.b[3]};
    s.box = {
      std::clamp(static_cast<float>((xs[0] - lb.pad_x) / lb.gain_x), 0.0f, static_cast<float>(w0)),
      std::clamp(static_cast<float>((ys[0] - lb.pad_y) / lb.gain_y), 0.0f, static_cast<float>(h0)),
      std::clamp(static_cast<float>((xs[1] - lb.pad_x) / lb.gain_x), 0.0f, static_cast<float>(w0)),
      std::clamp(static_cast<float>((ys[1] - lb.pad_y) / lb.gain_y), 0.0f, static_cast<float>(h0))};
    // 4) 마스크 로짓 = 계수 · 원형
    cv::Mat1f coef(1, o.nm);
    for (int k = 0; k < o.nm; ++k) {coef(0, k) = at(c.a, 4 + nc + k);}
    cv::Mat1f logit = cv::Mat1f(coef * protos).reshape(1, o.mh);
    if (f > 1) {cv::resize(logit, logit, cv::Size(iw, ih), 0, 0, cv::INTER_LINEAR);}
    cv::Mat1f big = logit(content);
    if (big.rows != h0 || big.cols != w0) {cv::resize(big, big, cv::Size(w0, h0), 0, 0, cv::INTER_LINEAR);}
    s.mask = big > 0.0f;
    // 상자 밖 0 (열 x1 ≤ c < x2, 행 y1 ≤ r < y2)
    const int cx0 = std::clamp(static_cast<int>(std::ceil(s.box[0])), 0, w0);
    const int cy0 = std::clamp(static_cast<int>(std::ceil(s.box[1])), 0, h0);
    const int cx1 = std::clamp(static_cast<int>(std::ceil(s.box[2])), 0, w0);
    const int cy1 = std::clamp(static_cast<int>(std::ceil(s.box[3])), 0, h0);
    cv::Mat1b keep = cv::Mat1b::zeros(h0, w0);
    if (cx1 > cx0 && cy1 > cy0) {keep(cv::Rect(cx0, cy0, cx1 - cx0, cy1 - cy0)).setTo(255);}
    s.mask &= keep;
    if (cv::countNonZero(s.mask) == 0) {continue;}           // 마스크 없는 검출은 버림 (ultralytics 와 같음)
    out.push_back(std::move(s));
  }
  return out;
}

void shape_outputs(SegOutputs & o, const std::vector<int64_t> & det, const std::vector<int64_t> & proto)
{
  if (det.size() != 3 || proto.size() != 4) {throw std::runtime_error("모델 출력 모양이 [1,A,C] · [1,nm,mh,mw] 가 아님");}
  o.channels_first = det[1] < det[2];                        // [1, C, A] = one2many (C = 4 + nc + 32 < 앵커 수)
  o.n_anchor = static_cast<int>(o.channels_first ? det[2] : det[1]);
  o.n_ch = static_cast<int>(o.channels_first ? det[1] : det[2]);
  o.nm = static_cast<int>(proto[1]);
  o.mh = static_cast<int>(proto[2]);
  o.mw = static_cast<int>(proto[3]);
}

#ifdef PERCEPTION_HAVE_TRT
SegModel::SegModel(const std::string & engine_path, float conf, float iou)
: engine_(std::make_unique<TrtEngine>(engine_path)), conf_(conf), iou_(iou)
{
  const auto & outs = engine_->outputs();
  for (size_t i = 0; i < outs.size(); ++i) {
    if (outs[i].shape.size() == 3) {idet_ = static_cast<int>(i);}
    if (outs[i].shape.size() == 4) {iproto_ = static_cast<int>(i);}
  }
  if (outs.size() != 2 || idet_ < 0 || iproto_ < 0) {throw std::runtime_error("엔진 출력이 seg 모델 모양이 아님: " + engine_path);}
  shape_outputs(out_, outs[idet_].shape, outs[iproto_].shape);
}

SegModel::~SegModel() = default;

const std::vector<std::string> & SegModel::names() const {return engine_->names();}

cv::Size SegModel::input_size() const {return engine_->input_size();}

std::vector<SegInstance> SegModel::operator()(const cv::Mat & bgr)
{
  using clk = std::chrono::steady_clock;
  const auto ms = [](clk::time_point a, clk::time_point b) {return std::chrono::duration<double, std::milli>(b - a).count();};
  const auto t0 = clk::now();
  const Letterbox lb = letterbox(bgr, engine_->input_size(), chw_);
  const auto t1 = clk::now();
  engine_->infer(chw_, outs_);
  const auto t2 = clk::now();
  out_.det.swap(outs_[idet_]);       // swap = 버퍼 재사용 (move 면 다음 장마다 수 MB 를 다시 할당)
  out_.proto.swap(outs_[iproto_]);
  auto res = postprocess(out_, lb, bgr.size(), engine_->names(), conf_, iou_);
  timing_ = {ms(t0, t1), ms(t1, t2), engine_->last_gpu_ms(), ms(t2, clk::now())};
  return res;
}
#endif  // PERCEPTION_HAVE_TRT

}  // namespace perception::wrist
