#include "transdata.h"

Transdata::Transdata()
{
}
Transdata::~Transdata()
{
}

int Transdata::transdataFree()
{
  av_bsf_free(&bsf_ctx_);
  avformat_close_input(&ifmt_ctx_);
  if (ret_ < 0 && ret_ != AVERROR_EOF)
  {
    printf("Error occurred.\n");
    return -1;
  }
  return 0;
}

int Transdata::transdataRecdata()
{
  if (av_read_frame(ifmt_ctx_, &pkt_) < 0)
  {
    return -1;
  }
  if (pkt_.stream_index == videoindex_)
  {
    // H.264 Filter
    if (av_bsf_send_packet(bsf_ctx_, &pkt_) < 0)
    {
      cout << " bsg_send_packet is error! " << endl;
      return -1;
    }
    if (av_bsf_receive_packet(bsf_ctx_, &pkt_) < 0)
    {
      cout << " bsg_receive_packet is error! " << endl;
      return -1;
    }
    printf("Write Video Packet. size:%d\tpts:%ld\n", pkt_.size, pkt_.pts);
    // Decode AVPacket
    if (pkt_.size)
    {
      ret_ = avcodec_send_packet(pCodecCtx_, &pkt_);
      if (ret_ < 0 || ret_ == AVERROR(EAGAIN) || ret_ == AVERROR_EOF)
      {
        std::cout << "avcodec_send_packet: " << ret_ << std::endl;
        return -1;
      }
      // Get AVframe
      ret_ = avcodec_receive_frame(pCodecCtx_, pframe_);
      if (ret_ == AVERROR(EAGAIN) || ret_ == AVERROR_EOF)
      {
        std::cout << "avcodec_receive_frame: " << ret_ << std::endl;
        return -1;
      }

      // AVframe to rgb This step requires operating image_test, so lock it
      mImage_buf_.lock();
      avFrame2Img(pframe_, image_test_);
      mImage_buf_.unlock();
    }
  }
  // Free AvPacket
  av_packet_unref(&pkt_);
  return 0;
}

int Transdata::transdataInit()
{
  // Register
  av_register_all();
  // Network
  avformat_network_init();
  // Input
  if ((ret_ = avformat_open_input(&ifmt_ctx_, in_filename_, nullptr, nullptr)) < 0)
  {
    printf("Could not open input file.");
    return -1;
  }
  if ((ret_ = avformat_find_stream_info(ifmt_ctx_, nullptr)) < 0)
  {
    printf("Failed to retrieve input stream information");
    return -1;
  }
  videoindex_ = -1;
  for (int i = 0; i < ifmt_ctx_->nb_streams; i++)
  {
    if (ifmt_ctx_->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO)
    {
      videoindex_ = i;
      codecpar_ = ifmt_ctx_->streams[i]->codecpar;
    }
  }
  // Find H.264 Decoder
  pCodec_ = avcodec_find_decoder(AV_CODEC_ID_H264);
  if (pCodec_ == nullptr)
  {
    printf("Couldn't find Codec.\n");
    return -1;
  }
  pCodecCtx_ = avcodec_alloc_context3(pCodec_);
  if (!pCodecCtx_)
  {
    fprintf(stderr, "Could not allocate video codec context\n");
    exit(1);
  }
  if (avcodec_open2(pCodecCtx_, pCodec_, nullptr) < 0)
  {
    printf("Couldn't open codec.\n");
    return -1;
  }
  pframe_ = av_frame_alloc();
  if (!pframe_)
  {
    printf("Could not allocate video frame\n");
    exit(1);
  }
  buffersrc_ = av_bsf_get_by_name("h264_mp4toannexb");

  if (av_bsf_alloc(buffersrc_, &bsf_ctx_) < 0)
    return -1;
  if (avcodec_parameters_copy(bsf_ctx_->par_in, codecpar_) < 0)
    return -1;
  if (av_bsf_init(bsf_ctx_) < 0)
    return -1;
}

void Transdata::yuv420p2Rgb32(const uchar* yuvBuffer_in, const uchar* rgbBuffer_out, int width, int height)
{
  uchar* yuv_buffer = (uchar*)yuvBuffer_in;
  uchar* rgb32_buffer = (uchar*)rgbBuffer_out;

  int channels = 3;

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      int index = y * width + x;

      int index_y = y * width + x;
      int index_u = width * height + y / 2 * width / 2 + x / 2;
      int index_v = width * height + width * height / 4 + y / 2 * width / 2 + x / 2;

      uchar y = yuv_buffer[index_y];
      uchar u = yuv_buffer[index_u];
      uchar v = yuv_buffer[index_v];

      int r = y + 1.402 * (v - 128);
      int g = y - 0.34413 * (u - 128) - 0.71414 * (v - 128);
      int b = y + 1.772 * (u - 128);
      r = (r < 0) ? 0 : r;
      g = (g < 0) ? 0 : g;
      b = (b < 0) ? 0 : b;
      r = (r > 255) ? 255 : r;
      g = (g > 255) ? 255 : g;
      b = (b > 255) ? 255 : b;

      rgb32_buffer[(y * width + x) * channels + 2] = uchar(r);
      rgb32_buffer[(y * width + x) * channels + 1] = uchar(g);
      rgb32_buffer[(y * width + x) * channels + 0] = uchar(b);
    }
  }
}

void Transdata::avFrame2Img(AVFrame* pFrame, cv::Mat& img)
{
  int frame_height = pFrame->height;
  int frame_width = pFrame->width;
  int channels = 3;
  // Output image allocation memory
  img = cv::Mat::zeros(frame_height, frame_width, CV_8UC3);
  Mat output = cv::Mat::zeros(frame_height, frame_width, CV_8U);

  // Create a buffer to save yuv data
  uchar* p_decoded_buffer = (uchar*)malloc(frame_height * frame_width * sizeof(uchar) * channels);

  // Get yuv420p data from AVFrame and save it to buffer
  int i, j, k;
  // copy y component
  for (i = 0; i < frame_height; i++)
  {
    memcpy(p_decoded_buffer + frame_width * i, pFrame->data[0] + pFrame->linesize[0] * i, frame_width);
  }
  // Copy u component
  for (j = 0; j < frame_height / 2; j++)
  {
    memcpy(p_decoded_buffer + frame_width * i + frame_width / 2 * j, pFrame->data[1] + pFrame->linesize[1] * j,
           frame_width / 2);
  }
  // Copy v component
  for (k = 0; k < frame_height / 2; k++)
  {
    memcpy(p_decoded_buffer + frame_width * i + frame_width / 2 * j + frame_width / 2 * k,
           pFrame->data[2] + pFrame->linesize[2] * k, frame_width / 2);
  }

  // Convert the yuv420p data in the buffer to RGB;
  yuv420p2Rgb32(p_decoded_buffer, img.data, frame_width, frame_height);

  // Simple processing, canny is used here for binarization
  //    cvtColor(img, output, CV_RGB2GRAY);
  //    waitKey(2);
  //    Canny(img, output, 50, 50*2);
  //    waitKey(2);
  namedWindow("test", WINDOW_NORMAL);
  imshow("test", img);
  waitKey(1);
  // test function
  // imwrite("test.jpg",img);
  // Release buffer
  free(p_decoded_buffer);
  // img.release();
  output.release();
}
