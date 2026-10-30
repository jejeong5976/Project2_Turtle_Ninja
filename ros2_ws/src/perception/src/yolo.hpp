// YOLO 추론 (ONNX Runtime C++). ultralytics 로 내보낸 .onnx 를 그대로 씀
//
// - 입력 크기는 모델에서 읽음 (예: 1×3×960×960). letterbox 로 맞추고 결과를 원본 좌표로 되돌림
// - 출력 두 형식 모두 처리
//     [1, 4+클래스, 후보]  : 상자 cx, cy, w, h + 클래스 점수 → conf 필터 + 클래스별 NMS
//     [1, 300, 6]          : NMS 없는 end-to-end (x1, y1, x2, y2, score, class)
// - 입력이 float16 인 모델(FP16 export)도 받음
// - provider: "cuda" | "tensorrt" | "cpu". 못 쓰면 CPU 로 돌고 경고
// - 클래스 이름은 모델 메타데이터(names)에서 읽고, 없으면 생성자에 준 이름을 씀
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "postproc.hpp"

namespace perception
{

class Yolo
{
public:
  Yolo(const std::string & model_path, const std::string & provider,
    const std::vector<std::string> & fallback_names = {});
  ~Yolo();

  std::vector<Detection> infer(const cv::Mat & bgr, float conf, float iou = 0.7f);

  const std::vector<std::string> & names() const;
  std::string name(int cls) const;
  cv::Size input_size() const;          // (w, h)
  const std::string & provider() const; // 실제로 붙은 provider

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace perception
