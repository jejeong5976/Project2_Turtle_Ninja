// wrist_seg.hpp — 손목 퍽 분할 YOLO26-seg: 전처리 · 후처리 + TensorRT 모델
//
// 역할
//   - letterbox: 비율 유지 리사이즈 + 회색 114 여백, RGB, /255, CHW
//   - postprocess: 엔진 출력 → 인스턴스
//       one2many: 앵커마다 가장 높은 클래스 점수 > conf → 클래스별 NMS
//       one2one: 점수 > conf 인 앵커 · 클래스 쌍을 점수 순으로
//       점수 순 max_det 개 → 상자를 원본 좌표로 → 마스크 계수 × 원형 = 로짓 → 원본 크기로 bilinear → > 0 → 상자 밖 0
//   - SegModel: 엔진 + 위 과정을 묶음
// 입력
//   - BGR 영상 한 장
//   - 엔진 <이름>.engine + <이름>.names.txt, 출력 2 개
//       output0 검출: one2many = [1, 4+nc+32, A] 상자 xywh, one2one = [1, A, 4+nc+32] 상자 xyxy
//       output1 마스크 원형: [1, 32, mh, mw]
// 출력
//   - SegInstance 목록: 클래스 · 신뢰도 · 원본 상자 xyxy · 원본 크기 마스크, 255 = 퍽
// 실행
//   - perception_core 라이브러리, WristPipeline 이 부름. 시험 test_wrist_seg
//   - 엔진 만들기는 trt_engine.hpp, yolo/wrist/scripts/export_onnx.py
// note
//   - 결과는 ultralytics 8.4 predict retina_masks=True 와 같게 맞춤
//   - head 종류는 출력 모양으로 구분. 학습 체크포인트 기본 = one2many, export_onnx.py --head o2o = one2one
//   - 클래스 점수는 sigmoid 가 끝난 값, 뒤 32 개 = 마스크 계수
//   - 전처리 · 후처리는 TensorRT 없이 빌드 · 시험됨. SegModel 구현은 PERCEPTION_HAVE_TRT 일 때만 들어감
//   - OpenCV 4.6 DNN 은 YOLO26 ONNX 를 못 읽어 안 씀
//   - SegModel 하나는 한 스레드에서만 씀

#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace perception
{
class TrtEngine;
}

namespace perception::wrist
{

struct SegInstance
{
  int cls{-1};
  std::string kind;                         // 클래스 이름 (red_cyl_up 등)
  float conf{0.0f};
  std::array<float, 4> box{};               // 원본 픽셀 xyxy
  cv::Mat1b mask;                           // 원본 크기, 255 = 퍽
};

// 모델 원시 출력 (배치 1)
struct SegOutputs
{
  std::vector<float> det;                   // channels_first 면 C × A (one2many), 아니면 A × C (one2one)
  int n_anchor{0}, n_ch{0};
  bool channels_first{false};
  std::vector<float> proto;                 // nm × mh × mw
  int nm{0}, mh{0}, mw{0};
};

// 엔진 출력 모양 output0 [1, A, C] 또는 [1, C, A] · output1 [1, nm, mh, mw] → SegOutputs 크기 (값은 안 채움). 아니면 예외
void shape_outputs(SegOutputs & o, const std::vector<int64_t> & det, const std::vector<int64_t> & proto);

struct Letterbox
{
  double gain_x{1.0}, gain_y{1.0};          // 새 크기 / 원본 크기 (각 변 반올림 반영)
  int pad_x{0}, pad_y{0};                   // 왼쪽 · 위 여백
  int new_w{0}, new_h{0};
};

// BGR → 모델 입력 (RGB, /255, CHW float). ultralytics LetterBox(center=True, 114)
Letterbox letterbox(const cv::Mat & bgr, cv::Size in, std::vector<float> & chw);

// 원시 출력 → 인스턴스
std::vector<SegInstance> postprocess(
  const SegOutputs & o, const Letterbox & lb, cv::Size orig, const std::vector<std::string> & names,
  float conf, float iou = 0.7f, int max_det = 300);

// 엔진 + 전처리 · 후처리. 출력이 검출 (3 차원) · 마스크 원형 (4 차원) 2 개가 아니면 예외
class SegModel
{
public:
  explicit SegModel(const std::string & engine_path, float conf = 0.5f, float iou = 0.7f);
  ~SegModel();

  std::vector<SegInstance> operator()(const cv::Mat & bgr);

  const std::vector<std::string> & names() const;
  cv::Size input_size() const;   // (w, h)

private:
  std::unique_ptr<TrtEngine> engine_;
  int idet_{-1}, iproto_{-1};
  float conf_, iou_;
  std::vector<float> chw_;
  std::vector<std::vector<float>> outs_;
  SegOutputs out_;
};

}  // namespace perception::wrist
