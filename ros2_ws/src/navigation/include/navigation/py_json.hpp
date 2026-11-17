// py_json.hpp — 파이썬 json.dumps(ensure_ascii=False) 와 같은 글자로 쓰는 작은 도구
//   구분자 ", " · ": ", float = 파이썬 repr (가장 짧은 왕복 자릿수, 1e-05 · 1e+16 · 1.0), 제어 문자만 escape
//   startup 상태 JSON (/startup/pose_match_status · /startup/lifecycle_status) 을 파이썬 판과 같은 문자열로 냄
#pragma once

#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <string_view>

namespace navigation
{
namespace pyjson
{

inline void append_string(std::string & out, std::string_view s)
{
  out += '"';
  for (const char ch : s) {
    const auto c = static_cast<unsigned char>(ch);
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      default:
        if (c < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          out += ch;
        }
    }
  }
  out += '"';
}

inline void append_bool(std::string & out, bool b)
{
  out += b ? "true" : "false";
}

inline void append_int(std::string & out, int64_t v)
{
  char buf[32];
  auto r = std::to_chars(buf, buf + sizeof(buf), v);
  out.append(buf, r.ptr);
}

// 파이썬 float.__repr__ ('r' 형식): 소수점 위치 decpt 가 -4 이하 또는 16 초과면 지수 표기
inline void append_float(std::string & out, double v)
{
  if (!std::isfinite(v)) {
    // json.dumps(allow_nan=False) 와 같음
    throw std::invalid_argument("Out of range float values are not JSON compliant");
  }
  char buf[64];
  auto r = std::to_chars(buf, buf + sizeof(buf), v, std::chars_format::scientific);
  std::string_view sv(buf, static_cast<std::size_t>(r.ptr - buf));
  bool neg = false;
  if (!sv.empty() && sv[0] == '-') {
    neg = true;
    sv.remove_prefix(1);
  }
  const std::size_t e = sv.find('e');
  std::string digits;
  for (const char c : sv.substr(0, e)) {
    if (c != '.') {
      digits += c;
    }
  }
  const int exp10 = std::atoi(std::string(sv.substr(e + 1)).c_str());
  if (neg) {
    out += '-';
  }
  if (digits == "0") {
    out += "0.0";
    return;
  }
  const int n = static_cast<int>(digits.size());
  const int decpt = exp10 + 1;
  if (decpt <= -4 || decpt > 16) {
    out += digits[0];
    if (n > 1) {
      out += '.';
      out.append(digits, 1, std::string::npos);
    }
    out += 'e';
    int x = decpt - 1;
    out += x < 0 ? '-' : '+';
    x = std::abs(x);
    if (x < 10) {
      out += '0';
    }
    out += std::to_string(x);
  } else if (decpt <= 0) {
    out += "0.";
    out.append(static_cast<std::size_t>(-decpt), '0');
    out += digits;
  } else if (decpt >= n) {
    out += digits;
    out.append(static_cast<std::size_t>(decpt - n), '0');
    out += ".0";
  } else {
    out.append(digits, 0, static_cast<std::size_t>(decpt));
    out += '.';
    out.append(digits, static_cast<std::size_t>(decpt), std::string::npos);
  }
}

// 파이썬 float(str) 과 같게 앞뒤 공백 허용, 전체가 수여야 함 (nan · inf 포함). 16진·밑줄 표기는 안 받음
inline bool parse_python_float(const std::string & text, double & out)
{
  const char * ws = " \t\n\r\f\v";
  const auto b = text.find_first_not_of(ws);
  if (b == std::string::npos) {
    return false;
  }
  const auto e = text.find_last_not_of(ws);
  const std::string t = text.substr(b, e - b + 1);
  if (t.find_first_of("xX(") != std::string::npos) {
    return false;
  }
  char * end = nullptr;
  out = std::strtod(t.c_str(), &end);
  return end == t.c_str() + t.size();
}

}  // namespace pyjson
}  // namespace navigation
