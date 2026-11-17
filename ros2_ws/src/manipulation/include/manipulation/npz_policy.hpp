// npz_policy.hpp — numpy .npz 읽기 + MLP 정책 (순수 C++, test/test_nudge.cpp)
// 팀 RL 정책 형식 (sim/robot_sim/rl/tasks/local_planner/export.py · deploy/numpy_policy.py 와 같음):
//   npz = zip 안 .npy 들: mean · std · eps (관측 정규화), n_layers, activation ("elu"), W0..W{n−1} (출력 × 입력), b0..b{n−1}
//   x = (obs − mean) / (std + eps) → [W x + b → ELU] × (n − 1) → W x + b (= 평균 행동, [−1, 1] 자르기는 부르는 쪽)
// zip: 저장 (stored) 은 늘 읽음. deflate 는 zlib 이 빌드에 있을 때만 (MANIP_HAVE_ZLIB). np.savez 는 stored + zip64 머리 (force_zip64)
// .npy: 버전 1 · 2 · 3, '<f4' · '<f8' · '<i4' · '<i8' · '|b1' · '<U<n>' (글), C 순서만 (fortran_order True 는 거절)

#pragma once

#include <cstddef>
#include <map>
#include <string>
#include <vector>

namespace manipulation
{

struct NpyArray
{
  std::string descr;                 // '<f4' …
  std::vector<std::size_t> shape;    // () = 스칼라
  std::vector<double> data;          // 숫자 (글이면 비어 있음)
  std::string text;                  // '<U..' 글 (스칼라 하나)
  std::size_t size() const;          // 원소 수 (스칼라 = 1)
};

// .npy 한 개 (바이트) → 배열. 실패면 false + why
bool parse_npy(const std::string & bytes, NpyArray & out, std::string & why);
// .npz (zip) 파일 → 이름 ("mean" — 끝의 .npy 뺌) → 배열
bool read_npz(const std::string & path, std::map<std::string, NpyArray> & out, std::string & why);

class MlpPolicy
{
public:
  // npz 읽기 + 모양 확인 (mean · std 길이 = W0 입력, 층마다 이어짐). 실패면 false + why (정책 안 바뀜)
  bool load(const std::string & path, std::string & why);
  bool loaded() const {return !W_.empty();}
  std::size_t obs_dim() const {return mean_.size();}
  std::size_t act_dim() const {return b_.empty() ? 0 : b_.back().size();}
  // obs 길이가 obs_dim 이 아니면 빈 벡터
  std::vector<double> act(const std::vector<double> & obs) const;

private:
  std::vector<double> mean_, std_;
  double eps_{0.0};
  std::vector<std::vector<double>> W_;   // 층마다 [출력 × 입력] 행 우선
  std::vector<std::size_t> rows_, cols_;
  std::vector<std::vector<double>> b_;
};

}  // namespace manipulation
