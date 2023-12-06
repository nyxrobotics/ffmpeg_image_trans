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

  void avFrame2Img(AVFrame* pFrame, cv::Mat& img);
  void yuv420p2Rgb32(const uchar* yuvBuffer_in, const uchar* rgbBuffer_out, int width, int height);
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
  // const char *in_filename  = "rtmp://localhost:1935/rtmplive";   // Mango channel rtmp address
  // const char *in_filename  = "rtmp://58.200.131.2:1935/livetv/hunantv";   // Mango channel rtmp address
  const char* in_filename_ = "rtmp://183.62.75.39:6030/livertmp/test";
  const char* out_filename_v_ = "test1.h264";  // Output file URL

  cv::Mat image_test_;
  mutex mImage_buf_;
  int transdataInit();
  int transdataRecdata();
  int transdataFree();
};

#endif  // VERSION1_0_TRANSDATA_H
