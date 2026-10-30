#include "yolo.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <regex>
#include <stdexcept>

#include <onnxruntime_cxx_api.h>
#include <opencv2/imgproc.hpp>

namespace perception
{

namespace
{

Ort::Env & ort_env()
{
  static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "perception");   // 프로세스에 하나 (두 탐지 노드가 같이 씀)
  return env;
}

// ultralytics 메타데이터 names: "{0: 'red_cyl', 1: 'green_cyl', ...}"
std::vector<std::string> parse_names(const std::string & s)
{
  std::vector<std::pair<int, std::string>> kv;
  const std::regex re(R"((\d+)\s*:\s*'([^']*)')");
  for (auto it = std::sregex_iterator(s.begin(), s.end(), re); it != std::sregex_iterator(); ++it) {
    kv.emplace_back(std::stoi((*it)[1]), (*it)[2]);
  }
  std::sort(kv.begin(), kv.end());
  std::vector<std::string> out;
  for (auto & p : kv) {out.push_back(p.second);}
  return out;
}

void append_provider(Ort::SessionOptions & so, const std::string & provider, const std::string & cache_dir)
{
  const OrtApi & api = Ort::GetApi();
  if (provider == "tensorrt") {
    // FP16 엔진을 만들어 cache_dir 에 저장 (처음 한 번 수 분 걸림). TensorRT 가 못 하는 연산은 CUDA 로
    OrtTensorRTProviderOptionsV2 * trt = nullptr;
    Ort::ThrowOnError(api.CreateTensorRTProviderOptions(&trt));
    const char * keys[] = {"device_id", "trt_fp16_enable", "trt_engine_cache_enable", "trt_engine_cache_path"};
    const char * vals[] = {"0", "1", "1", cache_dir.c_str()};
    Ort::ThrowOnError(api.UpdateTensorRTProviderOptions(trt, keys, vals, 4));
    try {
      so.AppendExecutionProvider_TensorRT_V2(*trt);
    } catch (...) {
      api.ReleaseTensorRTProviderOptions(trt);
      throw;
    }
    api.ReleaseTensorRTProviderOptions(trt);
  }
  if (provider == "tensorrt" || provider == "cuda") {
    OrtCUDAProviderOptionsV2 * cuda = nullptr;
    Ort::ThrowOnError(api.CreateCUDAProviderOptions(&cuda));
    const char * keys[] = {"device_id"};
    const char * vals[] = {"0"};
    Ort::ThrowOnError(api.UpdateCUDAProviderOptions(cuda, keys, vals, 1));
    try {
      so.AppendExecutionProvider_CUDA_V2(*cuda);
    } catch (...) {
      api.ReleaseCUDAProviderOptions(cuda);
      throw;
    }
    api.ReleaseCUDAProviderOptions(cuda);
  }
}

}  // namespace

struct Yolo::Impl
{
  Ort::Session session{nullptr};
  std::string in_name, out_name;
  std::vector<int64_t> in_shape;
  bool fp16_in{false};
  std::vector<std::string> names;
  std::string provider;
  cv::Size size;
  std::vector<float> blob;
  std::vector<Ort::Float16_t> blob16;
};

Yolo::Yolo(const std::string & model_path, const std::string & provider, const std::vector<std::string> & fallback_names)
: impl_(std::make_unique<Impl>())
{
  const std::string cache_dir = model_path.substr(0, model_path.find_last_of('/') + 1);
  auto make = [&](const std::string & prov) {
      Ort::SessionOptions so;
      so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
      append_provider(so, prov, cache_dir.empty() ? std::string("./") : cache_dir);
      impl_->session = Ort::Session(ort_env(), model_path.c_str(), so);
      impl_->provider = prov;
    };
  try {
    make(provider);
  } catch (const Ort::Exception & e) {
    if (provider == "cpu") {throw;}
    std::cerr << "[perception] provider '" << provider << "' 사용 불가 → CPU 로 실행: " << e.what() << std::endl;
    make("cpu");
  }

  Ort::AllocatorWithDefaultOptions alloc;
  impl_->in_name = impl_->session.GetInputNameAllocated(0, alloc).get();
  impl_->out_name = impl_->session.GetOutputNameAllocated(0, alloc).get();
  const auto in_type = impl_->session.GetInputTypeInfo(0);   // TypeInfo 를 살려 둬야 아래 정보가 유효함
  const auto in_info = in_type.GetTensorTypeAndShapeInfo();
  impl_->in_shape = in_info.GetShape();
  impl_->fp16_in = in_info.GetElementType() == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16;
  if (impl_->in_shape.size() != 4) {throw std::runtime_error("입력이 [1, 3, H, W] 가 아님: " + model_path);}
  impl_->in_shape[0] = 1;
  if (impl_->in_shape[2] <= 0 || impl_->in_shape[3] <= 0) {   // 동적 크기 모델이면 960
    impl_->in_shape[2] = impl_->in_shape[3] = 960;
  }
  impl_->size = cv::Size(static_cast<int>(impl_->in_shape[3]), static_cast<int>(impl_->in_shape[2]));

  auto md = impl_->session.GetModelMetadata();
  auto names = md.LookupCustomMetadataMapAllocated("names", alloc);
  if (names) {impl_->names = parse_names(names.get());}
  if (impl_->names.empty()) {impl_->names = fallback_names;}

  infer(cv::Mat(impl_->size, CV_8UC3, cv::Scalar(114, 114, 114)), 1.0f);   // 첫 추론은 초기화로 느려서 미리 한 번
}

Yolo::~Yolo() = default;

const std::vector<std::string> & Yolo::names() const {return impl_->names;}

std::string Yolo::name(int cls) const
{
  return (cls >= 0 && cls < static_cast<int>(impl_->names.size())) ? impl_->names[cls] : std::to_string(cls);
}

cv::Size Yolo::input_size() const {return impl_->size;}

const std::string & Yolo::provider() const {return impl_->provider;}

std::vector<Detection> Yolo::infer(const cv::Mat & bgr, float conf, float iou)
{
  auto & I = *impl_;
  double r;
  int px, py;
  const cv::Mat lb = letterbox(bgr, I.size, r, px, py);

  // BGR uint8 HWC → RGB float CHW (0~1)
  const int H = I.size.height, W = I.size.width, plane = H * W;
  I.blob.resize(static_cast<size_t>(3 * plane));
  for (int y = 0; y < H; ++y) {
    const auto * p = lb.ptr<cv::Vec3b>(y);
    for (int x = 0; x < W; ++x) {
      const int i = y * W + x;
      I.blob[i] = p[x][2] / 255.f;
      I.blob[plane + i] = p[x][1] / 255.f;
      I.blob[2 * plane + i] = p[x][0] / 255.f;
    }
  }
  auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  Ort::Value input{nullptr};
  if (I.fp16_in) {
    I.blob16.resize(I.blob.size());
    for (size_t i = 0; i < I.blob.size(); ++i) {I.blob16[i] = Ort::Float16_t(I.blob[i]);}
    input = Ort::Value::CreateTensor<Ort::Float16_t>(mem, I.blob16.data(), I.blob16.size(), I.in_shape.data(), I.in_shape.size());
  } else {
    input = Ort::Value::CreateTensor<float>(mem, I.blob.data(), I.blob.size(), I.in_shape.data(), I.in_shape.size());
  }
  const char * in_names[] = {I.in_name.c_str()};
  const char * out_names[] = {I.out_name.c_str()};
  auto outs = I.session.Run(Ort::RunOptions{nullptr}, in_names, &input, 1, out_names, 1);

  auto info = outs[0].GetTensorTypeAndShapeInfo();
  const auto shape = info.GetShape();
  const size_t n = info.GetElementCount();
  std::vector<float> out(n);
  if (info.GetElementType() == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16) {
    const auto * d = outs[0].GetTensorData<Ort::Float16_t>();
    for (size_t i = 0; i < n; ++i) {out[i] = d[i].ToFloat();}
  } else {
    const auto * d = outs[0].GetTensorData<float>();
    std::copy(d, d + n, out.begin());
  }

  auto to_orig = [&](float x1, float y1, float x2, float y2) {
      Box b;
      b.x1 = std::clamp(static_cast<float>((x1 - px) / r), 0.f, static_cast<float>(bgr.cols));
      b.y1 = std::clamp(static_cast<float>((y1 - py) / r), 0.f, static_cast<float>(bgr.rows));
      b.x2 = std::clamp(static_cast<float>((x2 - px) / r), 0.f, static_cast<float>(bgr.cols));
      b.y2 = std::clamp(static_cast<float>((y2 - py) / r), 0.f, static_cast<float>(bgr.rows));
      return b;
    };

  std::vector<Detection> dets;
  if (shape.size() == 3 && shape[2] == 6) {            // [1, 300, 6] end-to-end (NMS 없음)
    for (int64_t i = 0; i < shape[1]; ++i) {
      const float * d = &out[static_cast<size_t>(i * 6)];
      if (d[4] < conf) {continue;}
      dets.push_back({to_orig(d[0], d[1], d[2], d[3]), d[4], static_cast<int>(d[5])});
    }
    return dets;
  }
  if (shape.size() != 3 || shape[1] < 5) {throw std::runtime_error("모르는 출력 모양");}
  const int64_t C = shape[1], N = shape[2];             // [1, 4+클래스, 후보]
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

}  // namespace perception
