// npz_policy.cpp — numpy .npz 읽기 + MLP 정책 (설명은 include/manipulation/npz_policy.hpp)

#include "manipulation/npz_policy.hpp"

#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>

#ifdef MANIP_HAVE_ZLIB
#include <zlib.h>
#endif

namespace manipulation
{

namespace
{
uint16_t rd16(const std::string & s, std::size_t o) {return static_cast<uint16_t>(static_cast<uint8_t>(s[o]) | (static_cast<uint8_t>(s[o + 1]) << 8));}
uint32_t rd32(const std::string & s, std::size_t o) {return static_cast<uint32_t>(rd16(s, o)) | (static_cast<uint32_t>(rd16(s, o + 2)) << 16);}
uint64_t rd64(const std::string & s, std::size_t o) {return static_cast<uint64_t>(rd32(s, o)) | (static_cast<uint64_t>(rd32(s, o + 4)) << 32);}

// 머리 글 'key': 값 → 값 글 (괄호 · 따옴표 포함, 다음 ',' 까지 — shape 는 괄호 끝까지)
bool header_value(const std::string & h, const std::string & key, std::string & v)
{
  const auto k = h.find("'" + key + "'");
  if (k == std::string::npos) {return false;}
  auto p = h.find(':', k);
  if (p == std::string::npos) {return false;}
  ++p;
  while (p < h.size() && h[p] == ' ') {++p;}
  if (p >= h.size()) {return false;}
  std::size_t e;
  if (h[p] == '(') {
    e = h.find(')', p);
    if (e == std::string::npos) {return false;}
    ++e;
  } else if (h[p] == '\'') {
    e = h.find('\'', p + 1);
    if (e == std::string::npos) {return false;}
    ++e;
  } else {
    e = h.find_first_of(",}", p);
    if (e == std::string::npos) {return false;}
  }
  v = h.substr(p, e - p);
  return true;
}

#ifdef MANIP_HAVE_ZLIB
bool inflate_raw(const std::string & in, std::size_t out_size, std::string & out)
{
  out.assign(out_size, '\0');
  z_stream zs{};
  if (inflateInit2(&zs, -MAX_WBITS) != Z_OK) {return false;}
  zs.next_in = reinterpret_cast<Bytef *>(const_cast<char *>(in.data()));
  zs.avail_in = static_cast<uInt>(in.size());
  zs.next_out = reinterpret_cast<Bytef *>(out.data());
  zs.avail_out = static_cast<uInt>(out.size());
  const int r = inflate(&zs, Z_FINISH);
  const bool ok = r == Z_STREAM_END && zs.total_out == out_size;
  inflateEnd(&zs);
  return ok;
}
#endif
}  // namespace

std::size_t NpyArray::size() const
{
  std::size_t n = 1;
  for (auto s : shape) {n *= s;}
  return n;
}

bool parse_npy(const std::string & b, NpyArray & out, std::string & why)
{
  if (b.size() < 10 || b.compare(0, 6, "\x93NUMPY") != 0) {why = ".npy 머리 (\\x93NUMPY) 아님"; return false;}
  const int major = static_cast<uint8_t>(b[6]);
  std::size_t hlen = 0, off = 0;
  if (major == 1) {hlen = rd16(b, 8); off = 10;}
  else if (major == 2 || major == 3) {
    if (b.size() < 12) {why = ".npy 머리 짧음"; return false;}
    hlen = rd32(b, 8);
    off = 12;
  } else {why = ".npy 버전 " + std::to_string(major) + " 모름"; return false;}
  if (off + hlen > b.size()) {why = ".npy 머리 길이 이상"; return false;}
  const std::string h = b.substr(off, hlen);
  std::string descr, fo, shape;
  if (!header_value(h, "descr", descr) || !header_value(h, "fortran_order", fo) || !header_value(h, "shape", shape)) {
    why = ".npy 머리 글 이상: " + h;
    return false;
  }
  if (fo.find("True") != std::string::npos) {why = "fortran_order True 는 안 읽음"; return false;}
  if (descr.size() < 3) {why = "descr 이상"; return false;}
  out = NpyArray{};
  out.descr = descr.substr(1, descr.size() - 2);
  for (std::size_t i = 0; i < shape.size();) {   // (256, 86) · (86,) · ()
    if (std::isdigit(static_cast<unsigned char>(shape[i]))) {
      std::size_t j = i;
      while (j < shape.size() && std::isdigit(static_cast<unsigned char>(shape[j]))) {++j;}
      out.shape.push_back(static_cast<std::size_t>(std::stoull(shape.substr(i, j - i))));
      i = j;
    } else {
      ++i;
    }
  }
  const std::size_t n = out.size();
  const std::string & d = out.descr;
  const std::size_t data0 = off + hlen;
  auto need = [&](std::size_t bytes) {
      if (data0 + bytes > b.size()) {why = ".npy 자료가 모자람 (" + d + ")"; return false;}
      return true;
    };
  if (d.size() >= 3 && (d[0] == '<' || d[0] == '|') && d[1] == 'U') {   // UTF-32 글 (스칼라 하나만)
    if (d.size() < 3 || !std::isdigit(static_cast<unsigned char>(d[2]))) {why = "글 dtype 이상 " + d; return false;}
    const std::size_t chars = static_cast<std::size_t>(std::stoul(d.substr(2)));
    if (!need(4 * chars * n)) {return false;}
    for (std::size_t i = 0; i < chars; ++i) {
      const uint32_t c = rd32(b, data0 + 4 * i);
      if (c == 0) {break;}
      out.text.push_back(c < 128 ? static_cast<char>(c) : '?');
    }
    return true;
  }
  out.data.resize(n);
  if (d == "<f4") {
    if (!need(4 * n)) {return false;}
    for (std::size_t i = 0; i < n; ++i) {
      const uint32_t u = rd32(b, data0 + 4 * i);
      float f;
      std::memcpy(&f, &u, 4);
      out.data[i] = f;
    }
  } else if (d == "<f8") {
    if (!need(8 * n)) {return false;}
    for (std::size_t i = 0; i < n; ++i) {
      const uint64_t u = rd64(b, data0 + 8 * i);
      double f;
      std::memcpy(&f, &u, 8);
      out.data[i] = f;
    }
  } else if (d == "<i4") {
    if (!need(4 * n)) {return false;}
    for (std::size_t i = 0; i < n; ++i) {out.data[i] = static_cast<int32_t>(rd32(b, data0 + 4 * i));}
  } else if (d == "<i8") {
    if (!need(8 * n)) {return false;}
    for (std::size_t i = 0; i < n; ++i) {out.data[i] = static_cast<double>(static_cast<int64_t>(rd64(b, data0 + 8 * i)));}
  } else if (d == "|b1" || d == "|u1" || d == "|i1") {
    if (!need(n)) {return false;}
    for (std::size_t i = 0; i < n; ++i) {
      out.data[i] = d == "|i1" ? static_cast<double>(static_cast<int8_t>(b[data0 + i])) : static_cast<double>(static_cast<uint8_t>(b[data0 + i]));
    }
  } else {
    why = "모르는 dtype " + d;
    return false;
  }
  return true;
}

bool read_npz(const std::string & path, std::map<std::string, NpyArray> & out, std::string & why)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {why = "파일 못 엶: " + path; return false;}
  std::stringstream ss;
  ss << f.rdbuf();
  const std::string z = ss.str();
  // 중앙 목록 끝 (EOCD, 0x06054b50) 을 뒤에서 찾음 (주석 최대 64 KB)
  if (z.size() < 22) {why = "zip 이 너무 짧음"; return false;}
  std::size_t eocd = std::string::npos;
  const std::size_t lo = z.size() > 22 + 65535 ? z.size() - 22 - 65535 : 0;
  for (std::size_t p = z.size() - 22 + 1; p-- > lo;) {
    if (rd32(z, p) == 0x06054b50u) {eocd = p; break;}
  }
  if (eocd == std::string::npos) {why = "zip 끝 목록 (EOCD) 없음 — npz 아님"; return false;}
  uint64_t entries = rd16(z, eocd + 10);
  uint64_t cd_off = rd32(z, eocd + 16);
  if (cd_off == 0xFFFFFFFFu || entries == 0xFFFFu) {   // zip64 끝 목록
    if (eocd < 20 || rd32(z, eocd - 20) != 0x07064b50u) {why = "zip64 위치 기록 없음"; return false;}
    const uint64_t z64 = rd64(z, eocd - 20 + 8);
    if (z64 > z.size() || z.size() - z64 < 56 || rd32(z, z64) != 0x06064b50u) {why = "zip64 끝 목록 이상"; return false;}
    entries = rd64(z, z64 + 32);
    cd_off = rd64(z, z64 + 48);
  }
  std::size_t p = cd_off;
  out.clear();
  for (uint64_t e = 0; e < entries; ++e) {
    if (p > z.size() || z.size() - p < 46 || rd32(z, p) != 0x02014b50u) {why = "zip 중앙 목록 이상"; return false;}
    const uint16_t method = rd16(z, p + 10);
    uint64_t csize = rd32(z, p + 20), usize = rd32(z, p + 24);
    const uint16_t nlen = rd16(z, p + 28), xlen = rd16(z, p + 30), clen = rd16(z, p + 32);
    uint64_t loc = rd32(z, p + 42);
    if (p + 46 + nlen + xlen + clen > z.size()) {why = "zip 중앙 목록 넘침"; return false;}
    const std::string name = z.substr(p + 46, nlen);
    // zip64 덧붙임 (id 1): 0xFFFFFFFF 인 칸만 차례로 (원래 크기 · 압축 크기 · 위치)
    for (std::size_t x = p + 46 + nlen; x + 4 <= p + 46 + nlen + xlen;) {
      const uint16_t id = rd16(z, x), sz = rd16(z, x + 2);
      if (id == 1) {
        std::size_t q = x + 4;
        if (usize == 0xFFFFFFFFu && q + 8 <= x + 4 + sz) {usize = rd64(z, q); q += 8;}
        if (csize == 0xFFFFFFFFu && q + 8 <= x + 4 + sz) {csize = rd64(z, q); q += 8;}
        if (loc == 0xFFFFFFFFu && q + 8 <= x + 4 + sz) {loc = rd64(z, q);}
      }
      x += 4 + sz;
    }
    p += 46 + nlen + xlen + clen;
    if (loc > z.size() || z.size() - loc < 30 || rd32(z, loc) != 0x04034b50u) {why = "zip 파일 머리 이상: " + name; return false;}
    const std::size_t data = loc + 30 + rd16(z, loc + 26) + rd16(z, loc + 28);
    if (data > z.size() || csize > z.size() - data || usize > (1ull << 31)) {why = "zip 자료 넘침: " + name; return false;}
    std::string bytes;
    if (method == 0) {
      bytes = z.substr(data, csize);
    } else if (method == 8) {
#ifdef MANIP_HAVE_ZLIB
      if (!inflate_raw(z.substr(data, csize), usize, bytes)) {why = "deflate 풀기 실패: " + name; return false;}
#else
      why = "deflate 압축 npz (np.savez_compressed) — 이 빌드엔 zlib 없음, np.savez 로 다시 저장";
      return false;
#endif
    } else {
      why = "zip 압축 방식 " + std::to_string(method) + " 모름: " + name;
      return false;
    }
    std::string key = name;
    if (key.size() > 4 && key.compare(key.size() - 4, 4, ".npy") == 0) {key.resize(key.size() - 4);}
    NpyArray a;
    std::string w;
    if (!parse_npy(bytes, a, w)) {why = name + ": " + w; return false;}
    out[key] = a;
  }
  if (out.empty()) {why = "npz 가 비어 있음"; return false;}
  return true;
}

bool MlpPolicy::load(const std::string & path, std::string & why)
{
  std::map<std::string, NpyArray> z;
  try {   // 깨진 파일이 예외 (stoul 등) 로 노드를 죽이지 않게
    if (!read_npz(path, z, why)) {return false;}
  } catch (const std::exception & e) {
    why = std::string("npz 읽기 예외: ") + e.what();
    return false;
  }
  for (const char * k : {"mean", "std", "eps", "n_layers"}) {
    if (!z.count(k)) {why = std::string("npz 에 ") + k + " 없음"; return false;}
  }
  if (z.count("activation") && z["activation"].text != "elu") {why = "활성화 " + z["activation"].text + " (elu 만)"; return false;}
  const int n = static_cast<int>(std::lround(z["n_layers"].data.empty() ? 0.0 : z["n_layers"].data[0]));
  if (n < 1 || n > 16) {why = "n_layers 이상"; return false;}
  std::vector<std::vector<double>> W, B;
  std::vector<std::size_t> rows, cols;
  std::size_t in = z["mean"].size();
  if (z["std"].size() != in || z["eps"].data.empty()) {why = "mean · std · eps 모양 이상"; return false;}
  for (int i = 0; i < n; ++i) {
    const std::string wk = "W" + std::to_string(i), bk = "b" + std::to_string(i);
    if (!z.count(wk) || !z.count(bk)) {why = "npz 에 " + wk + " · " + bk + " 없음"; return false;}
    const auto & w = z[wk];
    const auto & b = z[bk];
    if (w.shape.size() != 2 || w.shape[1] != in || b.size() != w.shape[0]) {
      why = wk + " 모양이 앞 층과 안 맞음";
      return false;
    }
    W.push_back(w.data);
    B.push_back(b.data);
    rows.push_back(w.shape[0]);
    cols.push_back(w.shape[1]);
    in = w.shape[0];
  }
  mean_ = z["mean"].data;
  std_ = z["std"].data;
  eps_ = z["eps"].data[0];
  W_ = W;
  b_ = B;
  rows_ = rows;
  cols_ = cols;
  return true;
}

std::vector<double> MlpPolicy::act(const std::vector<double> & obs) const
{
  if (W_.empty() || obs.size() != mean_.size()) {return {};}
  std::vector<double> x(obs.size());
  for (std::size_t i = 0; i < x.size(); ++i) {x[i] = (obs[i] - mean_[i]) / (std_[i] + eps_);}   // numpy_policy.py 와 같음 (std + eps)
  for (std::size_t l = 0; l < W_.size(); ++l) {
    std::vector<double> y(rows_[l]);
    const double * w = W_[l].data();
    for (std::size_t r = 0; r < rows_[l]; ++r) {
      double s = b_[l][r];
      const double * wr = w + r * cols_[l];
      for (std::size_t c = 0; c < cols_[l]; ++c) {s += wr[c] * x[c];}
      y[r] = l + 1 < W_.size() ? (s > 0.0 ? s : std::expm1(s)) : s;   // ELU (alpha 1), 마지막 층 선형
    }
    x.swap(y);
  }
  return x;
}

}  // namespace manipulation
