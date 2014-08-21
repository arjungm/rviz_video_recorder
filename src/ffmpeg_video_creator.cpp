#include <boost/make_shared.hpp>
#include "rviz_video_recorder/ffmpeg_video_creator.h"

#define STREAM_DURATION   5.0
#define STREAM_FRAME_RATE 25
#define STREAM_NB_FRAMES  ((int)(STREAM_DURATION * STREAM_FRAME_RATE))
#define STREAM_PIX_FMT PIX_FMT_YUV420P

FFMPEGImageToVideoCreator::FFMPEGImageToVideoCreator() :
write_frame_count_(0),
image_frame_(),
video_frame_(),
image_format_context_(NULL),
video_format_context_(NULL),
image_codec_context_(NULL),
video_codec_context_(NULL),
video_stream_(NULL),
video_output_format_(NULL)
{
  av_register_all();
}

FFMPEGImageToVideoCreator::~FFMPEGImageToVideoCreator() {}

void FFMPEGImageToVideoCreator::loadImages( const std::string& directory, const std::string& prefix )
{
}

void FFMPEGImageToVideoCreator::createVideo( const std::string& video_filename )
{
}

void FFMPEGImageToVideoCreator::testFun()
{
  read_image("/home/amenon/Pictures/text_slide.jpg");
  std::string output_filename = "test.mp4";
  video_output_format_ = av_guess_format(NULL, output_filename.c_str(), NULL);
  if (!video_output_format_) {
    printf("Could not deduce output format from file extension: using MPEG.\n");
    video_output_format_ = av_guess_format("mpeg", NULL, NULL);
  }
  if (!video_output_format_) {
    fprintf(stderr, "Could not find suitable output format\n");
    exit(1);
  }
  
  video_format_context_ = avformat_alloc_context();
  video_format_context_->oformat = video_output_format_;
  snprintf(video_format_context_->filename, sizeof(video_format_context_->filename), "%s", output_filename.c_str());
  /* open the output file, if needed */
  if (!(video_output_format_->flags & AVFMT_NOFILE)) {
    //if (url_fopen(&video_format_context_->pb, output_filename.c_str(), URL_WRONLY) < 0)
    if( avio_open(&video_format_context_->pb, output_filename.c_str(), AVIO_FLAG_WRITE) < 0)
    {
      fprintf(stderr, "Could not open '%s'\n", output_filename.c_str());
      exit(1);
    }
  }
  video_format_context_->flags = AVFMT_FLAG_CUSTOM_IO;

  std::cout << "Filename:" << video_format_context_->filename << std::endl;
  std::cout << "========" << std::endl;
  
  // open stream, open video
  if (video_output_format_->video_codec != CODEC_ID_NONE)
    add_video_stream(video_output_format_->video_codec);
  av_dump_format(video_format_context_, 0, output_filename.c_str(), 1);
  if (video_stream_)
    open_video();

  avformat_write_header(video_format_context_, NULL);

  double video_pts = 0.0;
  for(;;) {
    /* compute current audio and video time */
    if (video_stream_)
      video_pts = (double)video_stream_->pts.val * video_stream_->time_base.num / video_stream_->time_base.den;
    else
      video_pts = 0.0;

    if (!video_stream_ || video_pts >= STREAM_DURATION)
      break;

    /* write interleaved audio and video frames */
    write_video_frame();
  }

  /* write the trailer, if any.  the trailer must be written
   * before you close the CodecContexts open when you wrote the
   * header; otherwise write_trailer may try to use memory that
   * was freed on av_codec_close() */
  std::cout << "Trailer!" << std::endl;
  av_write_trailer(video_format_context_);

  std::cout << "Close video" << std::endl;
  /* close each codec */
  if (video_stream_)
    close_video();

  // avformat_close_input(&video_format_context_); // NOT OPENED WITH AVFORMAT_OPEN_INPUT!
  
  if (!(video_output_format_->flags & AVFMT_NOFILE))
    avio_close(video_format_context_->pb);
  avformat_free_context(video_format_context_);
  avformat_free_context(image_format_context_);

  //std::cout << "Close streams" << std::endl;
  ///* free the streams */
  //for(int i = 0; i < video_format_context_->nb_streams; i++) {
  //  av_freep(&video_format_context_->streams[i]->codec);
  //  av_freep(&video_format_context_->streams[i]);
  //}
  //std::cout << "Close file" << std::endl;
  //std::cout << "Close contexts" << std::endl;
  ///* free the stream */
  //av_free(video_format_context_);
  
  std::cout << "Close image" << std::endl;
  av_free(image_frame_.frame);
  av_free(image_frame_.buffer);
}

void FFMPEGImageToVideoCreator::alloc_frame(PixelFormat pix_fmt, int width, int height, FFMPEGFrame& fframe)
{
  if(fframe.frame)
    av_free(fframe.frame);
  // allocate frame
  fframe.frame = avcodec_alloc_frame();

  if(!fframe.frame)
    std::cout << "Unable to allocate frame!" << std::endl;
  
  // make buffer
  fframe.buffer_size = avpicture_get_size( pix_fmt, width, height);
  fframe.buffer = (uint8_t *) av_malloc( fframe.buffer_size * sizeof(uint8_t));
  
  // connect buffer to frame
  avpicture_fill((AVPicture *) fframe.frame, fframe.buffer, pix_fmt, width, height);
}

void FFMPEGImageToVideoCreator::read_image( const std::string& filename )
{
  if(image_format_context_)
    av_free(image_format_context_);
  image_format_context_ = NULL;
  image_format_context_ = avformat_alloc_context();

  if(avformat_open_input( &image_format_context_ , filename.c_str(), NULL, NULL)!=0)
    std::cout << "Can't open image" << std::endl;    

  // get codec
  AVCodec* image_codec;
  image_codec_context_ = image_format_context_->streams[0]->codec;
  image_codec_context_->width = 1600;
  image_codec_context_->height = 1200;

    // (*codec_context)->pix_fmt = PIX_FMT_YUV420P;
  image_codec = avcodec_find_decoder(image_codec_context_->codec_id);
  if(!image_codec)
    std::cout << "Failed to find codec" << std::endl;
  if(avcodec_open2( image_codec_context_, image_codec, NULL)<0)
    std::cout << "Failed to open codec" << std::endl;
  
  // get frame
  alloc_frame(image_codec_context_->pix_fmt, image_codec_context_->width, image_codec_context_->height, image_frame_);

  // read image into the frame
  AVPacket packet;
  int frame_number = 0;
  while (av_read_frame(image_format_context_, &packet) >= 0)
  {
    // image stream (only 1)
    if(packet.stream_index != 0)
      continue;

    int frame_finished; 
    int ret = avcodec_decode_video2(image_codec_context_, image_frame_.frame, &frame_finished, &packet);
    if (ret > 0)
      std::cout << "Decoded" << std::endl;
    else 
      std::cout << "Failed to decode" << std::endl;

    av_free_packet(&packet);
  }
}

void FFMPEGImageToVideoCreator::open_video()
{
  AVCodec* video_codec;

  // locate codec
  video_codec = avcodec_find_encoder( video_codec_context_->codec_id );
  if(!video_codec)
    std::cout << "Can't find video codec" << std::endl;

  // open codec
  if( avcodec_open2( video_codec_context_, video_codec, NULL)<0 )
    std::cout << "Can't open the video codec" << std::endl;

  // create video buffer
  if (!(video_format_context_->oformat->flags & AVFMT_RAWPICTURE))
  {
    video_buffer_size_ = 200000+video_codec_context_->width*video_codec_context_->height;
    video_buffer_ = (uint8_t *)av_malloc(video_buffer_size_);
  }

  // create video frame
  alloc_frame(video_codec_context_->pix_fmt, video_codec_context_->width, video_codec_context_->height, video_frame_);
}

void FFMPEGImageToVideoCreator::close_video()
{
  avcodec_close(video_codec_context_);
  avcodec_close(image_codec_context_);
  av_free(video_frame_.frame);
  av_free(video_frame_.buffer);
  av_free(video_buffer_);
}

void FFMPEGImageToVideoCreator::add_video_stream(enum CodecID codec_id)
{
  video_stream_ = avformat_new_stream(video_format_context_, NULL);
  video_stream_->id = 0;

  if (!video_stream_) {
    fprintf(stderr, "Could not alloc stream\n");
    exit(1);
  }

  video_codec_context_ = video_stream_->codec;
  video_codec_context_->codec_id = codec_id;
  video_codec_context_->codec_type = AVMEDIA_TYPE_VIDEO;

  /* put sample parameters */
  video_codec_context_->bit_rate = 400000;
  /* resolution must be a multiple of two */
  video_codec_context_->width = 1600;
  video_codec_context_->height = 1200;
  /* time base: this is the fundamental unit of time (in seconds) in terms
     of which frame timestamps are represented. for fixed-fps content,
     timebase should be 1/framerate and timestamp increments should be
     identically 1. */
  video_codec_context_->time_base.den = STREAM_FRAME_RATE;
  video_codec_context_->time_base.num = 1;
  video_codec_context_->gop_size = 12; /* emit one intra frame every twelve frames at most */
  video_codec_context_->pix_fmt = STREAM_PIX_FMT;
  if (video_codec_context_->codec_id == CODEC_ID_MPEG2VIDEO) {
    /* just for testing, we also add B frames */
    video_codec_context_->max_b_frames = 2;
  }
  if (video_codec_context_->codec_id == CODEC_ID_MPEG1VIDEO){
    /* Needed to avoid using macroblocks in which some coeffs overflow.
       This does not happen with normal video, it just happens here as
       the motion of the chroma plane does not match the luma plane. */
    video_codec_context_->mb_decision=2;
  }
  // some formats want stream headers to be separate
  if(video_format_context_->oformat->flags & AVFMT_GLOBALHEADER)
    video_codec_context_->flags |= CODEC_FLAG_GLOBAL_HEADER;
}

void FFMPEGImageToVideoCreator::write_video_frame()
{
  int out_size, ret;
  static struct SwsContext *img_convert_ctx;

  if (write_frame_count_ >= STREAM_NB_FRAMES) 
  {
    /* no more frame to compress. The codec has a latency of a few
       frames if using B frames, so we get the last frames by
       passing the same picture again */
  } 
  else 
  {
    img_convert_ctx = NULL;
    img_convert_ctx = sws_getContext( video_codec_context_->width, 
                                      video_codec_context_->height, 
                                      PIX_FMT_YUV420P, 
                                      video_codec_context_->width, 
                                      video_codec_context_->height, 
                                      video_codec_context_->pix_fmt, 
                                      SWS_BICUBIC, NULL, NULL, NULL);
    if (img_convert_ctx == NULL) {
      fprintf(stderr, "Cannot initialize the conversion context\n");
      exit(1);
    }
    sws_scale(img_convert_ctx, image_frame_.frame->data, image_frame_.frame->linesize, 0, 
                video_codec_context_->height, video_frame_.frame->data, video_frame_.frame->linesize);
    sws_freeContext(img_convert_ctx);
  }

  if (video_format_context_->oformat->flags & AVFMT_RAWPICTURE) 
  {
    /* raw video case. The API will change slightly in the near
       futur for that */
    AVPacket pkt;
    av_init_packet(&pkt);

    pkt.flags |= AV_PKT_FLAG_KEY;
    pkt.stream_index= video_stream_->index;
    pkt.data= (uint8_t *)video_frame_.frame;
    pkt.size= sizeof(AVPicture);

    ret = av_interleaved_write_frame(video_format_context_, &pkt);
  } 
  else
  {
    /* encode the image */
    out_size = avcodec_encode_video(video_codec_context_, video_buffer_, video_buffer_size_, video_frame_.frame);
    /* if zero size, it means the image was buffered */
    if (out_size > 0) {
      AVPacket pkt;
      av_init_packet(&pkt);

      if (video_codec_context_->coded_frame->pts != AV_NOPTS_VALUE)
        pkt.pts= av_rescale_q(video_codec_context_->coded_frame->pts, 
                              video_codec_context_->time_base, 
                              video_stream_->time_base);
      if(video_codec_context_->coded_frame->key_frame)
        pkt.flags |= AV_PKT_FLAG_KEY;
      pkt.stream_index= video_stream_->index;
      pkt.data= video_buffer_;
      pkt.size= out_size;

      /* write the compressed frame in the media file */
      ret = av_interleaved_write_frame(video_format_context_, &pkt);
    } 
    else
    {
      ret = 0;
    }
  }
  if (ret != 0) {
    fprintf(stderr, "Error while writing video frame\n");
    exit(1);
  }
  write_frame_count_++;
}
