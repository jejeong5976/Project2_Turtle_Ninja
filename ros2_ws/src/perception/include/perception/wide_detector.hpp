// wide_detector.hpp — 광각 상자 검출 YOLO: 전처리 · 후처리 + TensorRT 모델
//
// 역할
//   - letterbox: 비율 유지 리사이즈 + 회색 114 채움, ultralytics LetterBox auto=False · center=True 와 같음
//   - to_chw: BGR HWC → RGB float CHW 0–1
//   - decode: 엔진 출력 → 원본 좌표 상자, 두 모양을 받음
//       [1, 4+클래스, 후보] = cx · cy · w · h + 클래스 점수 → conf 거르기 + 클래스별 NMS
//       [1, 300, 6] = NMS 가 끝난 end-to-end 출력 x1 · y1 · x2 · y2 · 점수 · 클래스
//   - BoxDetector: 엔진 + 위 과정을 묶음
// 입력
//   - BGR 영상 한 장
//   - 엔진 <이름>.engine + <이름>.names.txt
// 출력
//   - BoxDetection 목록: 원본 픽셀 xyxy · 점수 · 클래스 번호
// 실행
//   - perception_core 라이브러리, WidePipeline 이 부름. 시험 test_wide
//   - 엔진 만들기는 trt_engine.hpp, yolo/wide/scripts/export_onnx.py
// note
//   - 클래스 6 개: red_cyl · green_cyl · red_box · green_box · barcode · traffic_light
//   - 전처리 · 후처리는 TensorRT 없이 빌드 · 시험됨. BoxDetector 구현은 PERCEPTION_HAVE_TRT 일 때만 들어감
//   - 손목 wrist_seg 의 letterbox 와 반올림 방식이 달라 따로 둠
//   - BoxDetector 하나는 한 스레드에서만 씀

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace perception
{
class TrtEngine;
}

namespace perception::wide
{

// 원본 이미지 픽셀 좌표 상자 (왼쪽 위 x1, y1 / 오른쪽 아래 x2, y2)
struct Box
{
  float x1{0}, y1{0}, x2{0}, y2{0};
};

// 상자 하나
struct BoxDetection
{
  Box box;          // 원본 이미지 좌표
  float score{0};
  int cls{0};
};

// letterbox 정보 (원본 → 모델 입력)
struct LetterboxInfo
{
  double r{1.0};    // 배율
  int pad_x{0}, pad_y{0};
};

// → 모델 입력 크기 이미지 (BGR uint8)
cv::Mat letterbox(const cv::Mat & img, cv::Size size, LetterboxInfo & info);

// BGR uint8 HWC → RGB float CHW (0~1)
void to_chw(const cv::Mat & bgr, std::vector<float> & chw);

// 클래스별 NMS (IoU 가 iou 를 넘는 낮은 점수 상자 제거)
std::vector<BoxDetection> nms(std::vector<BoxDetection> dets, float iou);

// 엔진 출력 하나 (shape 포함) → 원본 좌표 상자. 모르는 모양이면 예외
std::vector<BoxDetection> decode(
  const std::vector<float> & out, const std::vector<int64_t> & shape, const LetterboxInfo & lb, cv::Size orig,
  float conf, float iou);

// 엔진 + 전처리 · 후처리. 출력 1 개인 상자 모델이 아니면 예외
class BoxDetector
{
public:
  explicit BoxDetector(const std::string & engine_path);
  ~BoxDetector();

  std::vector<BoxDetection> detect(const cv::Mat & bgr, float conf, float iou = 0.7f);

  const std::vector<std::string> & names() const;
  std::string name(int cls) const;
  cv::Size input_size() const;   // (w, h)

private:
  std::unique_ptr<TrtEngine> engine_;
  std::vector<float> chw_;
  std::vector<std::vector<float>> outs_;
};

}  // namespace perception::wide
