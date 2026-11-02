// trt_engine.hpp — TensorRT 엔진 실행, 광각 BoxDetector · 손목 SegModel 공용
//
// 역할
//   - 엔진 파일을 읽어 추론
//   - 생성할 때 0 입력으로 한 번 미리 추론해 첫 추론 지연을 없앰
// 입력
//   - 엔진 <이름>.engine 또는 <이름>.plan + 같은 폴더의 <이름>.names.txt, 한 줄에 클래스 하나
//   - infer 입력: 1 × 3 × H × W float32 CHW
// 출력
//   - 출력 텐서별 float32 배열, 엔진에 적힌 출력 순서
//   - 입력 크기, 출력 텐서 이름 · 모양, 클래스 이름
// 실행
//   - perception_core 라이브러리, TensorRT 가 있을 때만 빌드됨
//   - 엔진 만들기
//       1. PC: yolo/{wide,wrist}/scripts/export_onnx.py → <이름>.onnx + <이름>.names.txt
//       2. Jetson: trtexec --onnx=<이름>.onnx --saveEngine=<이름>.engine --fp16
//       3. <이름>.engine 옆에 <이름>.names.txt 를 같이 둠
// note
//   - 엔진은 만든 Jetson · 같은 TensorRT 판에서만 읽힘
//   - ultralytics export format=engine 결과는 앞에 메타데이터가 붙어 못 읽음
//   - 입출력은 float32 만 받음 → trtexec 에 --inputIOFormats · --outputIOFormats 를 주지 말 것
//   - names.txt 가 없으면 예외
//   - NvInfer.h 는 .cpp 안에만 있어 이 헤더는 TensorRT 없이도 include 됨
//   - infer 는 내부 버퍼를 써서 한 번에 한 스레드만 부를 것

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace perception
{

struct TrtTensor
{
  std::string name;
  std::vector<int64_t> shape;
  size_t count{0};                          // 원소 수
};

class TrtEngine
{
public:
  explicit TrtEngine(const std::string & path);
  ~TrtEngine();

  cv::Size input_size() const;                         // 입력 [1, 3, H, W] 의 W × H
  const TrtTensor & input() const;
  const std::vector<TrtTensor> & outputs() const;
  const std::vector<std::string> & names() const;      // <이름>.names.txt (한 줄에 하나)

  // input: 1 × 3 × H × W float32 (CHW). outs[i] = outputs()[i] 의 값
  void infer(const std::vector<float> & input, std::vector<std::vector<float>> & outs);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace perception
