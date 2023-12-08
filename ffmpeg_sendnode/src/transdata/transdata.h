#ifndef VERSION1_0_TRANSDATA_H
#define VERSION1_0_TRANSDATA_H

#include <iostream>
#include <thread>
#include <mutex>
extern "C" {
#include "libavformat/avformat.h"
#include <libavutil/mathematics.h>
#include <libavutil/time.h>
#include <libavutil/samplefmt.h>
#include <libavcodec/avcodec.h>
#include <libavfilter/buffersink.h>
#include <libavfilter/buffersrc.h>
};
#include "opencv2/core.hpp"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class Transdata
{
public:
  Transdata();
  ~Transdata();
  void setUrl(const std::string url);
  void setUsername(const std::string username);
  void setPassword(const std::string password);
  cv::Mat image_out_;
  int transdataInit();
  int transdataRecdata();
  int transdataFree();

private:
  std::string getFilename();
  void avFrame2Img(AVFrame* pFrame, cv::Mat& img);
  void yuv420p2Rgb32(const uchar* yuvBuffer_in, uchar* rgbBuffer_out, int width, int height);
  std::string in_url_;
  std::string in_username_;
  std::string in_password_;
  AVFormatContext* ifmt_ctx_ = nullptr;
  AVPacket pkt_;
  AVFrame* pframe_ = nullptr;
  int ret_;
  int videoindex_ = -1;
  AVCodecContext* pCodecCtx_;
  AVCodec* pCodec_;
  const AVBitStreamFilter* buffersrc_ = nullptr;
  AVBSFContext* bsf_ctx_;
  AVCodecParameters* codecpar_ = nullptr;
  mutex mImage_buf_;
};

#endif  // VERSION1_0_TRANSDATA_H
