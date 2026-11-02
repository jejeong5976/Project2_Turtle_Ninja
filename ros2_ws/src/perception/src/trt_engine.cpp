// trt_engine.cpp — TensorRT 엔진 실행 구현, 설명은 trt_engine.hpp
//    TensorRT 10 API: IO 텐서 이름 · setTensorAddress · enqueueV3

#include "perception/trt_engine.hpp"

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>

namespace perception
{

namespace
{
class TrtLogger : public nvinfer1::ILogger
{
  void log(Severity s, const char * msg) noexcept override
  {
    if (s <= Severity::kWARNING) {std::cerr << "[TensorRT] " << msg << std::endl;}
  }
};

bool ends_with(const std::string & s, const std::string & e)
{
  return s.size() >= e.size() && s.compare(s.size() - e.size(), e.size(), e) == 0;
}

// <이름>.engine → <이름>.names.txt. 없거나 비면 예외
std::vector<std::string> read_names(const std::string & engine_path)
{
  const auto dot = engine_path.find_last_of('.');
  const std::string path = engine_path.substr(0, dot) + ".names.txt";
  std::ifstream f(path);
  if (!f) {throw std::runtime_error("클래스 이름 파일 없음 (export_onnx.py 가 .onnx 옆에 만듦): " + path);}
  std::vector<std::string> out;
  for (std::string line; std::getline(f, line); ) {
    while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) {line.pop_back();}
    if (!line.empty()) {out.push_back(line);}
  }
  if (out.empty()) {throw std::runtime_error("클래스 이름 파일이 비어 있음: " + path);}
  return out;
}

void cuda_ok(cudaError_t e, const char * what)
{
  if (e != cudaSuccess) {throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(e));}
}
}  // namespace

struct TrtEngine::Impl
{
  TrtLogger logger;
  std::unique_ptr<nvinfer1::IRuntime> runtime;
  std::unique_ptr<nvinfer1::ICudaEngine> engine;
  std::unique_ptr<nvinfer1::IExecutionContext> ctx;
  cudaStream_t stream{nullptr};
  TrtTensor in;
  std::vector<TrtTensor> outs;
  void * in_ptr{nullptr};
  std::vector<void *> out_ptrs;
  std::vector<void *> bufs;   // 해제용 (입력 포함 전부)
  std::vector<std::string> names;

  ~Impl()
  {
    for (void * p : bufs) {cudaFree(p);}
    if (stream) {cudaStreamDestroy(stream);}
    ctx.reset();      // 만든 역순으로 해제
    engine.reset();
    runtime.reset();
  }
};

TrtEngine::TrtEngine(const std::string & path)
: impl_(std::make_unique<Impl>())
{
  auto & I = *impl_;
  if (!ends_with(path, ".engine") && !ends_with(path, ".plan")) {
    throw std::runtime_error("모델 확장자는 .engine / .plan (.onnx 는 Jetson 에서 trtexec 로 변환): " + path);
  }
  I.names = read_names(path);
  std::ifstream f(path, std::ios::binary);
  if (!f) {throw std::runtime_error("엔진 파일 없음: " + path);}
  const std::vector<char> blob((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  I.runtime.reset(nvinfer1::createInferRuntime(I.logger));
  if (!I.runtime) {throw std::runtime_error("TensorRT runtime 생성 실패");}
  I.engine.reset(I.runtime->deserializeCudaEngine(blob.data(), blob.size()));
  if (!I.engine) {throw std::runtime_error("엔진 읽기 실패 (다른 TensorRT 판 · 다른 장비에서 만든 엔진?): " + path);}
  I.ctx.reset(I.engine->createExecutionContext());
  if (!I.ctx) {throw std::runtime_error("TensorRT 실행 문맥 생성 실패");}
  cuda_ok(cudaStreamCreate(&I.stream), "cudaStreamCreate");

  bool have_in = false;
  for (int i = 0; i < I.engine->getNbIOTensors(); ++i) {
    const char * name = I.engine->getIOTensorName(i);
    if (I.engine->getTensorDataType(name) != nvinfer1::DataType::kFLOAT) {
      throw std::runtime_error(std::string("float32 아닌 입출력: ") + name + " (trtexec 에 --inputIOFormats 주지 말 것)");
    }
    const auto d = I.engine->getTensorShape(name);
    TrtTensor t;
    t.name = name;
    t.count = 1;
    for (int k = 0; k < d.nbDims; ++k) {
      if (d.d[k] <= 0) {throw std::runtime_error(std::string("동적 크기 텐서는 안 받음: ") + name + " (고정 크기로 export)");}
      t.shape.push_back(d.d[k]);
      t.count *= static_cast<size_t>(d.d[k]);
    }
    void * ptr = nullptr;
    cuda_ok(cudaMalloc(&ptr, t.count * sizeof(float)), "cudaMalloc");
    I.bufs.push_back(ptr);
    if (!I.ctx->setTensorAddress(name, ptr)) {throw std::runtime_error(std::string("setTensorAddress 실패: ") + name);}
    if (I.engine->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
      if (have_in) {throw std::runtime_error("입력이 2 개 이상인 엔진은 안 받음");}
      if (t.shape.size() != 4 || t.shape[0] != 1 || t.shape[1] != 3) {
        throw std::runtime_error("입력이 [1, 3, H, W] 가 아님: " + t.name);
      }
      I.in = t;
      I.in_ptr = ptr;
      have_in = true;
    } else {
      I.outs.push_back(t);
      I.out_ptrs.push_back(ptr);
    }
  }
  if (!have_in || I.outs.empty()) {throw std::runtime_error("엔진에 입력 · 출력이 없음: " + path);}

  std::vector<std::vector<float>> outs;
  infer(std::vector<float>(I.in.count, 0.0f), outs);   // 첫 추론은 초기화로 느려서 미리 한 번
}

TrtEngine::~TrtEngine() = default;

cv::Size TrtEngine::input_size() const
{
  return cv::Size(static_cast<int>(impl_->in.shape[3]), static_cast<int>(impl_->in.shape[2]));
}

const TrtTensor & TrtEngine::input() const {return impl_->in;}

const std::vector<TrtTensor> & TrtEngine::outputs() const {return impl_->outs;}

const std::vector<std::string> & TrtEngine::names() const {return impl_->names;}

void TrtEngine::infer(const std::vector<float> & input, std::vector<std::vector<float>> & outs)
{
  auto & I = *impl_;
  if (input.size() != I.in.count) {throw std::runtime_error("입력 크기가 엔진과 다름");}
  cuda_ok(cudaMemcpyAsync(I.in_ptr, input.data(), input.size() * sizeof(float), cudaMemcpyHostToDevice, I.stream),
    "입력 복사");
  if (!I.ctx->enqueueV3(I.stream)) {throw std::runtime_error("TensorRT enqueueV3 실패");}
  outs.resize(I.outs.size());
  for (size_t i = 0; i < I.outs.size(); ++i) {
    outs[i].resize(I.outs[i].count);
    cuda_ok(cudaMemcpyAsync(outs[i].data(), I.out_ptrs[i], I.outs[i].count * sizeof(float), cudaMemcpyDeviceToHost,
      I.stream), "출력 복사");
  }
  cuda_ok(cudaStreamSynchronize(I.stream), "cudaStreamSynchronize");
}

}  // namespace perception
