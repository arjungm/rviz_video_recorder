#ifdef __cplusplus
#define __STDC_CONSTANT_MACROS
#ifdef _STDINT_H
#undef _STDINT_H
#endif
extern "C" {
#include <stdint.h>
#include <math.h>
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/mathematics.h>
#include <libswscale/swscale.h>
}
#endif

#include <iostream>
#include <string>

#define STREAM_DURATION   5.0
#define STREAM_FRAME_RATE 25
#define STREAM_NB_FRAMES  ((int)(STREAM_DURATION * STREAM_FRAME_RATE))
#define STREAM_PIX_FMT PIX_FMT_YUV420P

AVFrame *image_frame;
AVCodecContext *image_codec_context;

AVFrame *picture, *tmp_picture;
uint8_t *video_outbuf;
int frame_count, video_outbuf_size;

uint8_t* picbuf;
long picbuf_size;

static int sws_flags = SWS_BICUBIC;

static AVStream *add_video_stream(AVFormatContext *oc, enum CodecID codec_id)
{
  AVCodecContext *c;
  AVStream *st;

  st = av_new_stream(oc, 0);
  if (!st) {
    fprintf(stderr, "Could not alloc stream\n");
    exit(1);
  }

  c = st->codec;
  c->codec_id = codec_id;
  c->codec_type = AVMEDIA_TYPE_VIDEO;

  /* put sample parameters */
  c->bit_rate = 400000;
  /* resolution must be a multiple of two */
  c->width = 1600;
  c->height = 1200;
  /* time base: this is the fundamental unit of time (in seconds) in terms
     of which frame timestamps are represented. for fixed-fps content,
     timebase should be 1/framerate and timestamp increments should be
     identically 1. */
  c->time_base.den = STREAM_FRAME_RATE;
  c->time_base.num = 1;
  c->gop_size = 12; /* emit one intra frame every twelve frames at most */
  c->pix_fmt = STREAM_PIX_FMT;
  if (c->codec_id == CODEC_ID_MPEG2VIDEO) {
    /* just for testing, we also add B frames */
    c->max_b_frames = 2;
  }
  if (c->codec_id == CODEC_ID_MPEG1VIDEO){
    /* Needed to avoid using macroblocks in which some coeffs overflow.
       This does not happen with normal video, it just happens here as
       the motion of the chroma plane does not match the luma plane. */
    c->mb_decision=2;
  }
  // some formats want stream headers to be separate
  if(oc->oformat->flags & AVFMT_GLOBALHEADER)
    c->flags |= CODEC_FLAG_GLOBAL_HEADER;

  return st;
}

static AVFrame *alloc_picture(enum PixelFormat pix_fmt, int width, int height)
{
  AVFrame *picture;
  uint8_t *picture_buf;
  int size;

  picture = avcodec_alloc_frame();
  if (!picture)
    return NULL;
  size = avpicture_get_size(pix_fmt, width, height);
  picture_buf = (uint8_t *)av_malloc(size);
  if (!picture_buf) {
    av_free(picture);
    return NULL;
  }
  avpicture_fill((AVPicture *)picture, picture_buf,
      pix_fmt, width, height);
  return picture;
}

static void open_video(AVFormatContext *oc, AVStream *st)
{
  AVCodec *codec;
  AVCodecContext *c;

  c = st->codec;

  /* find the video encoder */
  codec = avcodec_find_encoder(c->codec_id);
  if (!codec) {
    fprintf(stderr, "codec not found\n");
    exit(1);
  }

  /* open the codec */
  if (avcodec_open(c, codec) < 0) {
    fprintf(stderr, "could not open codec\n");
    exit(1);
  }

  video_outbuf = NULL;
  if (!(oc->oformat->flags & AVFMT_RAWPICTURE)) {
    /* allocate output buffer */
    /* XXX: API change will be done */
    /* buffers passed into lav* can be allocated any way you prefer,
       as long as they're aligned enough for the architecture, and
       they're freed appropriately (such as using av_free for buffers
       allocated with av_malloc) */
    video_outbuf_size = 200000+c->width*c->height;
    video_outbuf = (uint8_t *)av_malloc(video_outbuf_size);
  }

  /* allocate the encoded raw picture */
  picture = alloc_picture(c->pix_fmt, c->width, c->height);
  if (!picture) {
    fprintf(stderr, "Could not allocate picture\n");
    exit(1);
  }

  /* if the output format is not YUV420P, then a temporary YUV420P
     picture is needed too. It is then converted to the required
     output format */
  tmp_picture = NULL;
  if (c->pix_fmt != PIX_FMT_YUV420P) {
    tmp_picture = alloc_picture(PIX_FMT_YUV420P, c->width, c->height);
    if (!tmp_picture) {
      fprintf(stderr, "Could not allocate temporary picture\n");
      exit(1);
    }
  }
}

/* prepare a dummy image */
static void fill_yuv_image(AVFrame *pict, int frame_index, int width, int height)
{
  int x, y, i;

  i = frame_index;

  /* Y */
  for(y=0;y<height;y++) {
    for(x=0;x<width;x++) {
      pict->data[0][y * pict->linesize[0] + x] = x + y + i * 3;
    }
  }

  /* Cb and Cr */
  for(y=0;y<height/2;y++) {
    for(x=0;x<width/2;x++) {
      pict->data[1][y * pict->linesize[1] + x] = 128 + y + i * 2;
      pict->data[2][y * pict->linesize[2] + x] = 64 + x + i * 5;
    }
  }
}

static void write_video_frame(AVFormatContext *oc, AVStream *st)
{
  int out_size, ret;
  AVCodecContext *c;
  static struct SwsContext *img_convert_ctx;

  c = st->codec;

  if (frame_count >= STREAM_NB_FRAMES) 
  {
    /* no more frame to compress. The codec has a latency of a few
       frames if using B frames, so we get the last frames by
       passing the same picture again */
  } 
  else 
  {
    if (c->pix_fmt != PIX_FMT_YUV420P)
    {
      /* as we only generate a YUV420P picture, we must convert it
         to the codec pixel format if needed */
      if (img_convert_ctx == NULL)
      {
        img_convert_ctx = sws_getContext(c->width, c->height, PIX_FMT_YUV420P, c->width, c->height, c->pix_fmt, sws_flags, NULL, NULL, NULL);
        if (img_convert_ctx == NULL) {
          fprintf(stderr, "Cannot initialize the conversion context\n");
          exit(1);
        }
      }
      fill_yuv_image(tmp_picture, frame_count, c->width, c->height);
      // sws_scale(img_convert_ctx, tmp_picture->data, tmp_picture->linesize, 0, c->height, picture->data, picture->linesize);
    } 
    else 
    {
      // fill_yuv_image(picture, frame_count, c->width, c->height);
    }
    std::cout << image_codec_context->width << "x" << image_codec_context->height << std::endl;
    img_convert_ctx = sws_getContext(c->width, c->height, PIX_FMT_YUV420P, c->width, c->height, c->pix_fmt, sws_flags, NULL, NULL, NULL);
    if (img_convert_ctx == NULL) {
      fprintf(stderr, "Cannot initialize the conversion context\n");
      exit(1);
    }
    sws_scale(img_convert_ctx, image_frame->data, image_frame->linesize, 0, c->height, picture->data, picture->linesize);
    std::cin.get();
  }

  if (oc->oformat->flags & AVFMT_RAWPICTURE) {
    /* raw video case. The API will change slightly in the near
       futur for that */
    AVPacket pkt;
    av_init_packet(&pkt);

    pkt.flags |= AV_PKT_FLAG_KEY;
    pkt.stream_index= st->index;
    pkt.data= (uint8_t *)picture;
    pkt.size= sizeof(AVPicture);

    ret = av_interleaved_write_frame(oc, &pkt);
  } else {
    /* encode the image */
    out_size = avcodec_encode_video(c, video_outbuf, video_outbuf_size, picture);
    /* if zero size, it means the image was buffered */
    if (out_size > 0) {
      AVPacket pkt;
      av_init_packet(&pkt);

      if (c->coded_frame->pts != AV_NOPTS_VALUE)
        pkt.pts= av_rescale_q(c->coded_frame->pts, c->time_base, st->time_base);
      if(c->coded_frame->key_frame)
        pkt.flags |= AV_PKT_FLAG_KEY;
      pkt.stream_index= st->index;
      pkt.data= video_outbuf;
      pkt.size= out_size;

      /* write the compressed frame in the media file */
      ret = av_interleaved_write_frame(oc, &pkt);
    } else {
      ret = 0;
    }
  }
  if (ret != 0) {
    fprintf(stderr, "Error while writing video frame\n");
    exit(1);
  }
  frame_count++;
}

static void close_video(AVFormatContext *oc, AVStream *st)
{
  avcodec_close(st->codec);
  av_free(picture->data[0]);
  av_free(picture);
  if (tmp_picture) {
    av_free(tmp_picture->data[0]);
    av_free(tmp_picture);
  }
  av_free(video_outbuf);
}

static void read_image(const std::string& filename, AVFormatContext** format_context, AVCodecContext** codec_context)
{
  *format_context=NULL;
  *format_context=avformat_alloc_context();
  // open an image
  if(avformat_open_input( format_context, filename.c_str(), NULL, NULL)!=0)
    std::cout << "Can't open image" << std::endl;    

  // get codec
  AVCodec* codec;
  *codec_context = (*format_context)->streams[0]->codec;
  (*codec_context)->width = 1600;
  (*codec_context)->height = 1200;
  // (*codec_context)->pix_fmt = PIX_FMT_YUV420P;
  codec = avcodec_find_decoder((*codec_context)->codec_id);
  if(!codec)
    std::cout << "Failed to find codec" << std::endl;
  if(avcodec_open2(*codec_context, codec, NULL)<0)
    std::cout << "Failed to open codec" << std::endl;

  // get frame
  image_frame = avcodec_alloc_frame();
  if(!image_frame)
    std::cout << "Unable to allocate frame!" << std::endl;
  int num_bytes = avpicture_get_size((*codec_context)->pix_fmt, (*codec_context)->width, (*codec_context)->height);
  picbuf_size = (long)num_bytes;
  picbuf = (uint8_t *) av_malloc(num_bytes * sizeof(uint8_t));
  avpicture_fill((AVPicture *) image_frame, picbuf, (*codec_context)->pix_fmt, (*codec_context)->width, (*codec_context)->height);
  
  // read image into the frame
  AVPacket packet;
  int frame_number = 0;
  while (av_read_frame(*format_context, &packet) >= 0)
  {
    // image stream (only 1)
    if(packet.stream_index != 0)
      continue;

    int frame_finished; 
    int ret = avcodec_decode_video2(*codec_context, image_frame, &frame_finished, &packet);
    if (ret > 0)
      std::cout << "Decoded" << std::endl;
    else 
      std::cout << "Failed to decode" << std::endl;
  }
}

int main(int argc, char** argv)
{
  av_register_all();
  std::string filename = "/home/amenon/Pictures/text_slide.jpg";

  AVFormatContext* format_context;

  read_image(filename, &format_context, &image_codec_context);
  std::cout << "(press enter to continue)" << std::endl;
  std::cin.get();

  AVOutputFormat *fmt;
  AVFormatContext *oc;
  AVStream *video_st;
  double video_pts, audio_pts=0.0;
  int i;

  std::string output_filename = "test.mp4";
  fmt = av_guess_format(NULL, output_filename.c_str(), NULL);

  if (!fmt) {
    printf("Could not deduce output format from file extension: using MPEG.\n");
    fmt = av_guess_format("mpeg", NULL, NULL);
  }
  if (!fmt) {
    fprintf(stderr, "Could not find suitable output format\n");
    exit(1);
  }

  /* allocate the output media context */
  oc = avformat_alloc_context();
  if (!oc) {
    fprintf(stderr, "Memory error\n");
    exit(1);
  }
  oc->oformat = fmt;
  snprintf(oc->filename, sizeof(oc->filename), "%s", output_filename.c_str());

  /* add the audio and video streams using the default format codecs
     and initialize the codecs */
  video_st = NULL;
  if (fmt->video_codec != CODEC_ID_NONE) {
    video_st = add_video_stream(oc, fmt->video_codec);
  }
  /* set the output parameters (must be done even if no
     parameters). */
  if (av_set_parameters(oc, NULL) < 0) {
    fprintf(stderr, "Invalid output format parameters\n");
    exit(1);
  }

  dump_format(oc, 0, output_filename.c_str(), 1);

  /* now that all the parameters are set, we can open the audio and
     video codecs and allocate the necessary encode buffers */
  if (video_st)
    open_video(oc, video_st);

  /* open the output file, if needed */
  if (!(fmt->flags & AVFMT_NOFILE)) {
    if (url_fopen(&oc->pb, output_filename.c_str(), URL_WRONLY) < 0) {
      fprintf(stderr, "Could not open '%s'\n", output_filename.c_str());
      exit(1);
    }
  }

  /* write the stream header, if any */
  av_write_header(oc);

  for(;;) {
    /* compute current audio and video time */
    if (video_st)
      video_pts = (double)video_st->pts.val * video_st->time_base.num / video_st->time_base.den;
    else
      video_pts = 0.0;

    if (!video_st || video_pts >= STREAM_DURATION)
      break;

    /* write interleaved audio and video frames */
    std::cout << "Writing!" << std::endl;
    write_video_frame(oc, video_st);
  }

  /* write the trailer, if any.  the trailer must be written
   * before you close the CodecContexts open when you wrote the
   * header; otherwise write_trailer may try to use memory that
   * was freed on av_codec_close() */
  av_write_trailer(oc);

  /* close each codec */
  if (video_st)
    close_video(oc, video_st);

  /* free the streams */
  for(i = 0; i < oc->nb_streams; i++) {
    av_freep(&oc->streams[i]->codec);
    av_freep(&oc->streams[i]);
  }

  if (!(fmt->flags & AVFMT_NOFILE)) {
    /* close the output file */
    url_fclose(oc->pb);
  }

  /* free the stream */
  av_free(oc);
  
  av_free(image_frame);
  return 0;
}
