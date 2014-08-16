#ifdef __cplusplus
#define __STDC_CONSTANT_MACROS
#ifdef _STDINT_H
#undef _STDINT_H
#endif
extern "C" {
#include <stdint.h>
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libswscale/swscale.h>
}
#endif

#include <iostream>
#include <string>

int main(int argc, char** argv)
{
  av_register_all();
  AVFormatContext* format_context=NULL;
  std::string filename = "/home/amenon/Pictures/title_slide.jpg";
  
  // open an image
  if(avformat_open_input( &format_context, filename.c_str(), NULL, NULL)!=0)
  {
    std::cout << "Can't open image" << std::endl;    
  }
  
  // some meta information
  std::cout << "Number of streams in image: " << format_context->nb_streams << std::endl;

  // create a frame out of the image
  AVCodecContext *codec_context;
  codec_context = format_context->streams[0]->codec;
  codec_context->width = 1600;
  codec_context->height = 1200;
  codec_context->pix_fmt = PIX_FMT_YUV420P;
  
  // get codec
  AVCodec* codec = avcodec_find_decoder(codec_context->codec_id);
  if(!codec)
    std::cout << "Failed to find codec" << std::endl;
  if(avcodec_open2(codec_context, codec, NULL)<0)
    std::cout << "Failed to open codec" << std::endl;
  
  // initialize frame
  AVFrame* frame = avcodec_alloc_frame();
  if(!frame)
    std::cout << "Unable to allocate frame!" << std::endl;
  int num_bytes = avpicture_get_size(PIX_FMT_YUV420P, codec_context->width, codec_context->height);
  long buff_size = num_bytes;
  uint8_t* buffer = (uint8_t *) av_malloc(num_bytes * sizeof(uint8_t));
  avpicture_fill((AVPicture *) frame, buffer, PIX_FMT_YUVJ420P, codec_context->width, codec_context->height);

  // read image into the frame
  AVPacket packet;
  int frame_number = 0;
  while (av_read_frame(format_context, &packet) >= 0)
  {
    // image stream (only 1)
    if(packet.stream_index != 0)
      continue;

    int frame_finished; 
    int ret = avcodec_decode_video2(codec_context, frame, &frame_finished, &packet);
    if (ret > 0)
    {
      std::cout << "Decoded" << std::endl;
    }
    else {
      std::cout << "Failed to decode" << std::endl;
    }
  }

  // prepare output file
  const char* output_file = "test.mp4";
  AVOutputFormat* output_format = av_guess_format(NULL, output_file, NULL);
  if(!output_format)
  {
    printf("Could not deduce output format from file extension: using MPEG.\n");
    output_format = av_guess_format("mpeg", NULL, NULL);
  }
  if(!output_format)
  {
    fprintf(stderr, "Could not find suitable output format\n");
    exit(1);
  }
  fprintf(stdout,"Format Name: %s\n", output_format->name);
  // output_format->video_codec = CODEC_ID_H264;
  fprintf(stdout,"Format Video Codec: %d\n", output_format->video_codec);

  // prepare output format
  AVFormatContext* output_format_context;
  output_format_context = avformat_alloc_context();
  if(!output_format_context){
    fprintf(stderr, "Could not find suitable output format\n");
    exit(1);
  }
  output_format_context->oformat = output_format;
  snprintf(output_format_context->filename, sizeof(output_format_context->filename), "%s", output_file);

  // find video encoder
  AVCodec* video_codec = avcodec_find_encoder(output_format->video_codec);
  if (!video_codec) {
    fprintf(stderr, "Codec not found\n");
    exit(1);
  }
  
  // create video stream
  AVStream *video_avstream;
  video_avstream = avformat_new_stream(output_format_context, video_codec);
  if (!video_avstream) {
    fprintf(stderr, "Could not alloc stream\n");
    exit(1);
  }
  if (video_avstream->codec == NULL) {
    fprintf(stderr, "AVStream codec is NULL\n");
    exit(1);
  }

  // open the codec for the stream
  AVCodecContext* video_codec_context = NULL;
  video_codec_context = video_avstream->codec;
  if (!video_codec_context) {
    fprintf(stderr, "Could not allocate video codec context\n");
    exit(1);
  }
  video_codec_context->bit_rate = 400000;
  video_codec_context->width = 1600;
  video_codec_context->height = 1200;
  video_codec_context->time_base= (AVRational){1,25};
  video_codec_context->gop_size = 10; /* emit one intra frame every ten frames */
  video_codec_context->max_b_frames=1;
  video_codec_context->pix_fmt = PIX_FMT_YUV420P;
  if (avcodec_open2(video_codec_context, video_codec, NULL) < 0) {
    fprintf(stderr, "Could not open codec\n");
    exit(1);
  }

  // open the file
  if (!(output_format->flags & AVFMT_NOFILE)) {
    if (avio_open(&output_format_context->pb, output_file, AVIO_FLAG_WRITE) <
        0) {
      fprintf(stderr, "Could not open '%s'\n", output_file);
      return 1;
    }
  }
  // some formats want stream headers to be separate
  if(output_format_context->oformat->flags & AVFMT_GLOBALHEADER)
    video_codec_context->flags |= CODEC_FLAG_GLOBAL_HEADER;

  avformat_write_header(output_format_context, NULL);

  /* encode 1 second of video */
  uint8_t* pic_buffer = (uint8_t*)malloc(buff_size);

  AVPacket output_packet;
  for(int i=0;i<100;i++)
  {
    av_init_packet(&output_packet);
    output_packet.data=NULL;
    output_packet.size=0;

    fflush(stdout);
    frame->pts = i;
    /* encode the image */
    int got_output;
    // int ret = avcodec_encode_video2(video_codec_context, &packet, frame, &got_output);
    int out_size = avcodec_encode_video(video_codec_context, pic_buffer, buff_size, frame);
    if (out_size < 0) {
      fprintf(stderr, "Error encoding frame\n");
      exit(1);
    }

// Image -> AVFormatContext --(decode)--> AVPacket -> AVFrame
// AVFrame --(encode)--> uint8_t* buffer --> ??? --> AVFormatContext
    //if (video_codec_context->coded_frame->pts != AV_NOPTS_VALUE)
    //  output_packet.pts= av_rescale_q(video_codec_context->coded_frame->pts, video_codec_context->time_base, video_avstream->time_base);
    if(video_codec_context->coded_frame->key_frame)
      output_packet.flags |= AV_PKT_FLAG_KEY;
    output_packet.stream_index= video_avstream->index;
    output_packet.data= pic_buffer;
    output_packet.size= out_size;
    int ret = av_interleaved_write_frame( output_format_context, &output_packet);

    //av_free_packet(&output_packet);
  }

  std::cout << "End" << std::endl;
  
  avcodec_close( video_codec_context );
  av_free( video_codec_context );
  for(int i = 0; i < output_format_context->nb_streams; i++) {
    av_freep(&output_format_context->streams[i]);
  }
  if (!(output_format_context->flags & AVFMT_NOFILE)) {
    avio_close(output_format_context->pb);
  }

  /* free the stream */
  av_free(output_format_context);

  av_free(buffer);
  av_free(frame);
  avcodec_close(codec_context);
  avformat_close_input(&format_context);
  return 0;
}
