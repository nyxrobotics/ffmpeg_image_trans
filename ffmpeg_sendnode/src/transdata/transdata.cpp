#include "transdata.h"

Transdata::Transdata()
  : ifmt_ctx_(nullptr)
  , pCodec_(nullptr)
  , pCodecCtx_(nullptr)
  , pframe_(nullptr)
  , bsf_ctx_(nullptr)
  , videoindex_(-1)
  , ret_(0)
{
}

Transdata::~Transdata()
{
  transdataFree();
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
    if (av_bsf_send_packet(bsf_ctx_, &pkt_) < 0 || av_bsf_receive_packet(bsf_ctx_, &pkt_) < 0)
    {
      cout << "Error in bitstream filter processing." << endl;
      return -1;
    }

    // printf("Write Video Packet. size:%d\tpts:%ld\n", pkt_.size, pkt_.pts);

    // Decode AVPacket
    if (pkt_.size)
    {
      if ((ret_ = avcodec_send_packet(pCodecCtx_, &pkt_)) < 0 ||
          (ret_ = avcodec_receive_frame(pCodecCtx_, pframe_)) < 0)
      {
        std::cout << "Error in AVCodec processing: " << ret_ << std::endl;
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
  // Network
  avformat_network_init();

  // Set options
  AVDictionary* options = nullptr;
  av_dict_set(&options, "probesize", "32", 0);               // probesize 32
  av_dict_set(&options, "flags", "low_delay", 0);            // flags low_delay
  av_dict_set(&options, "rtsp_transport", "tcp", 0);         // or "udp" based on preference
  av_dict_set(&options, "rtsp_flags", "prefer_tcp", 0);      // Try TCP for RTP transport first
  av_dict_set(&options, "tcp_nodelay", "1", 0);              // Set TCP_NODELAY to disable Nagle’s algorithm
  av_dict_set(&options, "max_delay", "0", 0);                // the maximum demuxing delay [us]
  av_dict_set(&options, "allowed_media_types", "video", 0);  // Only accept video

  // Input
  if ((ret_ = avformat_open_input(&ifmt_ctx_, in_filename_, nullptr, &options)) < 0)
  {
    printf("Could not open input file.");
    av_dict_free(&options);
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

  if (!pCodec_)
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

  if (av_bsf_alloc(buffersrc_, &bsf_ctx_) < 0 || avcodec_parameters_copy(bsf_ctx_->par_in, codecpar_) < 0 ||
      av_bsf_init(bsf_ctx_) < 0)
  {
    return -1;
  }

  return 0;
}

void Transdata::yuv420p2Rgb32(const uchar* yuvBuffer_in, uchar* rgbBuffer_out, int width, int height)
{
  int channels = 3;

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      int index_y = y * width + x;
      int index_u = width * height + y / 2 * width / 2 + x / 2;
      int index_v = width * height + width * height / 4 + y / 2 * width / 2 + x / 2;

      uchar y_val = yuvBuffer_in[index_y];
      uchar u_val = yuvBuffer_in[index_u];
      uchar v_val = yuvBuffer_in[index_v];

      int r = y_val + 1.402 * (v_val - 128);
      int g = y_val - 0.34413 * (u_val - 128) - 0.71414 * (v_val - 128);
      int b = y_val + 1.772 * (u_val - 128);

      r = (r < 0) ? 0 : (r > 255) ? 255 : r;
      g = (g < 0) ? 0 : (g > 255) ? 255 : g;
      b = (b < 0) ? 0 : (b > 255) ? 255 : b;

      rgbBuffer_out[(y * width + x) * channels + 2] = static_cast<uchar>(r);
      rgbBuffer_out[(y * width + x) * channels + 1] = static_cast<uchar>(g);
      rgbBuffer_out[(y * width + x) * channels + 0] = static_cast<uchar>(b);
    }
  }
}

void Transdata::avFrame2Img(AVFrame* pFrame, cv::Mat& img)
{
  int frame_height = pFrame->height;
  int frame_width = pFrame->width;

  // Output image allocation memory
  img = cv::Mat::zeros(frame_height, frame_width, CV_8UC3);

  // Create a buffer to save yuv data
  uchar* p_decoded_buffer = (uchar*)malloc(frame_height * frame_width * sizeof(uchar) * 3);

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

  // Release buffer
  free(p_decoded_buffer);
}
