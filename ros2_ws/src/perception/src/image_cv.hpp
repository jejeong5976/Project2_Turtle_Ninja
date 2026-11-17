// sensor_msgs/Image ↔ cv::Mat 변환 (cv_bridge 대신 씀)
//   이유: apt cv_bridge 는 OpenCV 4.6 에 묶여 있음 → 젯슨 (JetPack OpenCV 4.8) 에서 한 프로세스에 두 판이 섞임
//   to_bgr8 · to_mono8: 형식이 같으면 메시지 버퍼를 그대로 가리킴 (복사 없음 = toCvShare 와 같음, 메시지가 살아 있는 동안만 씀)
//   모르는 형식 · 크기 안 맞음 = std::runtime_error
//   같은 파일이 manipulation/src/image_cv.hpp 에도 있음 (패키지끼리 의존 안 두려고 복사)
#pragma once

#include <cstring>
#include <stdexcept>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

namespace image_cv
{
namespace enc = sensor_msgs::image_encodings;

// 메시지 버퍼를 그대로 가리키는 Mat (색 변환 없음)
inline cv::Mat wrap(const sensor_msgs::msg::Image & m)
{
  const std::string & e = m.encoding;
  int type = -1;
  if (e == enc::MONO8 || e == enc::TYPE_8UC1) {
    type = CV_8UC1;
  } else if (e == enc::BGR8 || e == enc::RGB8 || e == enc::TYPE_8UC3) {
    type = CV_8UC3;
  } else if (e == enc::BGRA8 || e == enc::RGBA8 || e == enc::TYPE_8UC4) {
    type = CV_8UC4;
  } else if (e == enc::MONO16 || e == enc::TYPE_16UC1) {
    type = CV_16UC1;
  } else if (e == enc::TYPE_32FC1) {
    type = CV_32FC1;
  } else {
    throw std::runtime_error("영상 형식 " + e + " 모름");
  }
  const size_t row = static_cast<size_t>(m.width) * CV_ELEM_SIZE(type);
  if (m.width == 0 || m.height == 0 || m.step < row ||
    m.data.size() < static_cast<size_t>(m.step) * m.height)
  {
    throw std::runtime_error("영상 크기 안 맞음 (" + e + ")");
  }
  if (CV_ELEM_SIZE1(type) > 1 && m.is_bigendian) {
    throw std::runtime_error("big-endian 영상 안 받음 (" + e + ")");
  }
  return cv::Mat(static_cast<int>(m.height), static_cast<int>(m.width), type,
    const_cast<uint8_t *>(m.data.data()), m.step);
}

inline cv::Mat to_bgr8(const sensor_msgs::msg::Image & m)
{
  const cv::Mat src = wrap(m);
  const std::string & e = m.encoding;
  if (e == enc::BGR8 || e == enc::TYPE_8UC3) {return src;}
  cv::Mat out;
  if (e == enc::RGB8) {
    cv::cvtColor(src, out, cv::COLOR_RGB2BGR);
  } else if (e == enc::BGRA8 || e == enc::TYPE_8UC4) {
    cv::cvtColor(src, out, cv::COLOR_BGRA2BGR);
  } else if (e == enc::RGBA8) {
    cv::cvtColor(src, out, cv::COLOR_RGBA2BGR);
  } else if (e == enc::MONO8 || e == enc::TYPE_8UC1) {
    cv::cvtColor(src, out, cv::COLOR_GRAY2BGR);
  } else {
    throw std::runtime_error("영상 형식 " + e + " 를 bgr8 로 못 바꿈");
  }
  return out;
}

inline cv::Mat to_mono8(const sensor_msgs::msg::Image & m)
{
  const cv::Mat src = wrap(m);
  const std::string & e = m.encoding;
  if (e == enc::MONO8 || e == enc::TYPE_8UC1) {return src;}
  cv::Mat out;
  if (e == enc::BGR8 || e == enc::TYPE_8UC3) {
    cv::cvtColor(src, out, cv::COLOR_BGR2GRAY);
  } else if (e == enc::RGB8) {
    cv::cvtColor(src, out, cv::COLOR_RGB2GRAY);
  } else if (e == enc::BGRA8 || e == enc::TYPE_8UC4) {
    cv::cvtColor(src, out, cv::COLOR_BGRA2GRAY);
  } else if (e == enc::RGBA8) {
    cv::cvtColor(src, out, cv::COLOR_RGBA2GRAY);
  } else {
    throw std::runtime_error("영상 형식 " + e + " 를 mono8 로 못 바꿈");
  }
  return out;
}

// bgr8 Mat → 메시지 (복사)
inline sensor_msgs::msg::Image::UniquePtr bgr8_msg(const std_msgs::msg::Header & header, const cv::Mat & bgr)
{
  if (bgr.type() != CV_8UC3) {throw std::runtime_error("bgr8_msg: CV_8UC3 아님");}
  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->header = header;
  msg->height = static_cast<uint32_t>(bgr.rows);
  msg->width = static_cast<uint32_t>(bgr.cols);
  msg->encoding = enc::BGR8;
  msg->is_bigendian = false;
  msg->step = static_cast<uint32_t>(bgr.cols * 3);
  msg->data.resize(static_cast<size_t>(msg->step) * msg->height);
  for (int r = 0; r < bgr.rows; ++r) {
    std::memcpy(msg->data.data() + static_cast<size_t>(r) * msg->step, bgr.ptr(r), msg->step);
  }
  return msg;
}

}  // namespace image_cv
