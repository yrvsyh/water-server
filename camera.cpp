#include "camera.hpp"

#include <cstdio>
#include <mutex>
#include <unistd.h>

#define __STDC_CONSTANT_MACROS

#ifdef __cplusplus
extern "C" {
#endif
#include <libavdevice/avdevice.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/mathematics.h>
#include <libavutil/time.h>
#include <libswscale/swscale.h>
#ifdef __cplusplus
};
#endif

struct Camera::M {
    std::string device_path;
    std::string input_format;
    std::string framerate;
    std::string video_size;
    int videoindex;
    AVFormatContext *ifmt_ctx;
    std::mutex mtx;
    bool quitStream;
    bool isStreaming;
    bool isOpen;
};

Camera::Camera(std::string device_path, std::string input_format, std::string framerate,
               std::string video_size)
{
    m = new M{};
    m->device_path = device_path;
    m->input_format = input_format;
    m->framerate = framerate;
    m->video_size = video_size;
    m->quitStream = false;
    m->isStreaming = false;
    m->isOpen = false;
}

Camera::~Camera()
{
    close();
    delete m;
}

void Camera::open()
{
    if (m->isOpen)
        return;
    // ffmpeg初始化
    avdevice_register_all();
    avformat_network_init();
    // 摄像头输入fmt_ctx
    m->ifmt_ctx = avformat_alloc_context();
    AVDictionary *option = nullptr;
    av_dict_set(&option, "input_format", m->input_format.c_str(), 0);
    av_dict_set(&option, "framerate", m->framerate.c_str(), 0);
    av_dict_set(&option, "video_size", m->video_size.c_str(), 0);
    // 打开摄像头，v4l2格式，input_format为mjpeg
    avformat_open_input(&m->ifmt_ctx, m->device_path.c_str(),
                        av_find_input_format("v4l2"), &option);
    av_dict_free(&option);
    // 寻找fmt_ctx内的stream
    if (avformat_find_stream_info(m->ifmt_ctx, 0) < 0) {
        puts("Failed to retrieve input stream information");
        exit(-1);
    }
    for (uint i = 0; i < m->ifmt_ctx->nb_streams; i++) {
        if (m->ifmt_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            // 找到视频流
            m->videoindex = i;
            break;
        }
    }
    // 打印输入数据
    //    av_dump_format(m->ifmt_ctx, m->videoindex, m->device_path, 0);
    m->isOpen = true;
}

void Camera::close()
{
    stopPush();
    while (m->isStreaming)
        sleep(1);
    if (m->isOpen)
        avformat_close_input(&m->ifmt_ctx);
    m->isOpen = false;
    m->quitStream = false;
}

std::shared_ptr<std::vector<char>> Camera::capOneFrame(std::string video_size,
                                                       std::string framerate)
{
    if (!m->isOpen)
        open();
    std::lock_guard<std::mutex> lock{m->mtx};

    auto old_size = m->video_size;
    auto old_rate = m->framerate;

    if (framerate.empty())
        framerate = old_rate;
    if (!video_size.empty())
        // 调整分辨率
        changeVideoOption("mjpeg", framerate, video_size);

    auto pkt = av_packet_alloc();
    av_read_frame(m->ifmt_ctx, pkt);
    // 多读几次，稳定图像
    for (int i = 0; i < 5; i++) {
        av_packet_unref(pkt);
        av_read_frame(m->ifmt_ctx, pkt);
    }

    auto data = new std::vector<char>(pkt->size);
    data->insert(data->end(), pkt->data, pkt->data + pkt->size);

    // 写入文件
    FILE *fp = fopen("cap.jpg", "w");
    fwrite(pkt->data, 1, pkt->size, fp);

    printf("cap %d Kb data\n", pkt->size / 1000);
    av_packet_unref(pkt);
    fclose(fp);

    // 恢复分辨率
    changeVideoOption("mjpeg", old_rate, old_size);
    return std::shared_ptr<std::vector<char>>{data};
}

void Camera::changeVideoOption(std::string input_format, std::string framerate,
                               std::string video_size)
{
    avformat_close_input(&m->ifmt_ctx);
    AVDictionary *option = nullptr;
    av_dict_set(&option, "input_format", input_format.c_str(), 0);
    av_dict_set(&option, "framerate", framerate.c_str(), 0);
    av_dict_set(&option, "video_size", video_size.c_str(), 0);
    if (0 == avformat_open_input(&m->ifmt_ctx, m->device_path.c_str(),
                                 av_find_input_format("v4l2"), &option)) {
        m->input_format = input_format;
        m->framerate = framerate;
        m->video_size = video_size;
    }
    av_dict_free(&option);
}

void Camera::pushStream(std::string url)
{
    if (!m->isOpen)
        open();
    if (m->isStreaming)
        return;
    m->isStreaming = true;
    int ret = 0;

    // 初始化解码器，MJPEG，YUV422P
    auto icodec =
        avcodec_find_decoder(m->ifmt_ctx->streams[m->videoindex]->codecpar->codec_id);
    auto icodec_ctx = avcodec_alloc_context3(icodec);
    avcodec_parameters_to_context(icodec_ctx,
                                  m->ifmt_ctx->streams[m->videoindex]->codecpar);
    icodec_ctx->pix_fmt = AV_PIX_FMT_YUV422P;
    // 画质
    icodec_ctx->qmin = 1;
    icodec_ctx->qmax = 10;
    icodec_ctx->framerate = {30, 1};
    //    ifmt_ctx->streams[m->videoindex]->r_frame_rate = {30, 1};
    icodec_ctx->pkt_timebase = m->ifmt_ctx->streams[m->videoindex]->time_base;
    avcodec_open2(icodec_ctx, icodec, nullptr);

    // 新建输出Contex
    AVFormatContext *ofmt_ctx = nullptr;
    avformat_alloc_output_context2(&ofmt_ctx, nullptr, "flv", url.c_str());

    // 初始化编码器，FLV，YUV420P
    auto ocodec = avcodec_find_encoder(AV_CODEC_ID_FLV1);
    auto ocodec_ctx = avcodec_alloc_context3(ocodec);
    ocodec_ctx->codec_id = AV_CODEC_ID_FLV1;
    ocodec_ctx->codec_type = AVMEDIA_TYPE_VIDEO;
    ocodec_ctx->pix_fmt = AV_PIX_FMT_YUV420P;
    ocodec_ctx->width = m->ifmt_ctx->streams[m->videoindex]->codecpar->width;
    ocodec_ctx->height = m->ifmt_ctx->streams[m->videoindex]->codecpar->height;
    ocodec_ctx->time_base = AV_TIME_BASE_Q;
    ocodec_ctx->pkt_timebase = AV_TIME_BASE_Q;
    //    ocodec_ctx->bit_rate = 10000000;
    //    ocodec_ctx->gop_size = 250;
    // 画质
    ocodec_ctx->qmin = 1;
    ocodec_ctx->qmax = 10;
    ocodec_ctx->framerate = {30, 1};
    avcodec_open2(ocodec_ctx, ocodec, nullptr);

    //新建输出流
    auto *stream = avformat_new_stream(ofmt_ctx, ocodec);
    // 复制编码器数据
    avcodec_parameters_from_context(stream->codecpar, ocodec_ctx);
    stream->codec = ocodec_ctx;
    //    avcodec_parameters_to_context(stream->codec, stream->codecpar);
    stream->time_base = AV_TIME_BASE_Q;
    stream->r_frame_rate = {30, 1};
    stream->codecpar->codec_tag = 0;

    // 网络流需要这个
    avio_open(&ofmt_ctx->pb, url.c_str(), AVIO_FLAG_WRITE);

    AVDictionary *option = nullptr;
    av_dict_set(&option, "framerate", "30", 0);
    av_dict_set(&option, "fps", "30", 0);
    // 写入头部数据
    ret = avformat_write_header(ofmt_ctx, &option);
    if (ret < 0) {
        puts("avformat_write_header");
        return;
    }
    av_dict_free(&option);
    //打印输出信息
    //    av_dump_format(ofmt_ctx, 0, url, 1);

    // 存放read出来的packet
    auto pkt = av_packet_alloc();
    // 解码后的帧
    auto frame = av_frame_alloc();
    // 编码后的帧
    auto frameYUV = av_frame_alloc();
    // YUV422P转YUV420P
    auto outBuffer = (unsigned char *)av_malloc(av_image_get_buffer_size(
        ocodec_ctx->pix_fmt, ocodec_ctx->width, ocodec_ctx->height, 1));
    av_image_fill_arrays(frameYUV->data, frameYUV->linesize, outBuffer,
                         ocodec_ctx->pix_fmt, ocodec_ctx->width, ocodec_ctx->height, 1);
    auto imgConvertCtx = sws_getContext(
        icodec_ctx->width, icodec_ctx->height, icodec_ctx->pix_fmt, ocodec_ctx->width,
        ocodec_ctx->height, ocodec_ctx->pix_fmt, SWS_BICUBIC, nullptr, nullptr, nullptr);

    while (!m->quitStream) {
        int64_t t1 = 0;
        {
            std::lock_guard<std::mutex> lock{m->mtx};

            t1 = av_gettime();
            //读一帧
            av_read_frame(m->ifmt_ctx, pkt);
        }
        auto dts = pkt->dts;
        // 解码
        avcodec_send_packet(icodec_ctx, pkt);
        avcodec_receive_frame(icodec_ctx, frame);
        // 释放packet
        av_packet_unref(pkt);
        // 格式转换
        sws_scale(imgConvertCtx, (const unsigned char *const *)frame->data,
                  frame->linesize, 0, icodec_ctx->height, frameYUV->data,
                  frameYUV->linesize);
        // 释放frame
        av_frame_unref(frame);
        frameYUV->format = ocodec_ctx->pix_fmt;
        frameYUV->width = ocodec_ctx->width;
        frameYUV->height = ocodec_ctx->height;
        // 编码，frameYUV不用释放，重复使用
        avcodec_send_frame(ocodec_ctx, frameYUV);
        avcodec_receive_packet(ocodec_ctx, pkt);
        pkt->dts = pkt->pts = dts / 1000;

        //        printf("%ld\n", pkt->pts);
        // 发送数据
        ret = av_interleaved_write_frame(ofmt_ctx, pkt);
        if (ret < 0) {
            puts("av_interleaved_write_frame");
        }
        // 释放packet
        av_packet_unref(pkt);
        // 延时，控制帧率
        auto sleep_time = 1000000.0 / 30 - av_gettime() + t1;
        if (sleep_time > 0)
            av_usleep(sleep_time);
    }
    //    av_write_trailer(ofmt_ctx);
    avio_close(ofmt_ctx->pb);
    // 释放资源
    avcodec_free_context(&icodec_ctx);
    avformat_free_context(ofmt_ctx);
    av_frame_unref(frameYUV);
    av_free(outBuffer);
    sws_freeContext(imgConvertCtx);
    m->quitStream = false;
    m->isStreaming = false;
}

void Camera::stopPush() { m->quitStream = true; }
