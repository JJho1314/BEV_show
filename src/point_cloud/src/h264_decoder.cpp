#include "h264_decoder.h"

CH264Decoder::CH264Decoder()
{
    initial();
}

CH264Decoder::~CH264Decoder()
{
    unInitial();
}




int CH264Decoder::initial()
{
    avcodec_register_all();//新版本应该不需要这句话
    av_init_packet(&packet);

    codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec)
    {
        printf("avcodec_find_encoder failed");
        return -1;
    }

    context = avcodec_alloc_context3(codec);
    if (!context)
    {
        printf("avcodec_alloc_context3 failed");
        return -2;
    }

    context->codec_type = AVMEDIA_TYPE_VIDEO;
    context->pix_fmt = AV_PIX_FMT_YUV420P;

    if (avcodec_open2(context, codec, NULL) < 0)
    {
        printf("avcodec_open2 failed");
        return -3;
    }

    frame = av_frame_alloc();
    if (!frame)
    {
        return -4;
    }

    return 0;
}

void CH264Decoder::unInitial()
{
    avcodec_close(context);
    av_free(context);
    av_frame_free(&frame);
}

int CH264Decoder::decode(unsigned char *pDataIn, int nInSize, cv::Mat& res)
{
//    av_init_packet(&packet);
    packet.size = nInSize;
    packet.data = pDataIn;

    if (packet.size > 0)
    {
        int got_picture=0;
        //int ret= avcodec_decode_video2(context, frame, &got_picture, &packet);
        //新版用法
        int ret = avcodec_send_packet(context, &packet);
        if (ret == 0) got_picture = avcodec_receive_frame(context, frame); //got_picture = 0 success, a frame was returned
        if (ret < 0)
        {
            printf("avcodec_encode_video2 failed");
            return -2;
        }

        if (got_picture==0)//采用avcodec_decode_video2时,此处为if (got_picture)
        {
             avframe_to_cvmat(frame,res);
        }
    }
    else
    {
        printf("no data to decode");
        return -1;
    }

    return 0;
}

int CH264Decoder::avframe_to_cvmat(AVFrame *frame,cv::Mat& res)
{
    int width = frame->width, height = frame->height;
    res.create(height*3/2, width, CV_8UC1);
    memcpy( res.data, frame->data[0], width*height );
    memcpy( res.data + width*height, frame->data[1], width*height/4 );
    memcpy( res.data + width*height*5/4, frame->data[2], width*height/4 );

    //cv::imshow( "yuv_show", res );//yuv格式
    cv::cvtColor( res, res, cv::COLOR_YUV2BGR_I420 );//bgr格式
    //cv::imshow( "bgr_show", bgr );
    return 0;

}
AVFrame * CH264Decoder::cvmat2avframe(cv::Mat mat) {

    // alloc avframe
    AVFrame *avframe = av_frame_alloc();
    if (avframe && !mat.empty()) {

        avframe->format = AV_PIX_FMT_YUV420P;
        avframe->width = mat.cols;
        avframe->height = mat.rows;
        av_frame_get_buffer(avframe, 0);
        av_frame_make_writable(avframe);
        cv::Mat yuv; // convert to yuv420p first
        cv::cvtColor(mat, yuv, cv::COLOR_BGR2YUV_I420);
        // calc frame size
        int frame_size = mat.cols * mat.rows;
        unsigned char *pdata = yuv.data;
        // fill yuv420
        // yyy yyy yyy yyy
        // uuu
        // vvv
        avframe->data[0] = pdata; // fill y
        avframe->data[1] = pdata + frame_size; // fill u
        avframe->data[2] = pdata + frame_size * 5 / 4; // fill v
    }
    return avframe;
}