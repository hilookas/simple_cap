#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <errno.h>
#include <fcntl.h> /* low-level i/o */
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <linux/videodev2.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rcpputils/endian.hpp"

extern "C" {
#define __STDC_CONSTANT_MACROS  // Required for libavutil
#include "libavutil/imgutils.h"
#include "libavformat/avformat.h"
#include "libavutil/error.h"
#include "libavutil/log.h"
#include "linux/videodev2.h"
#include "libswscale/swscale.h"
}

#define CLEAR(x) memset(&(x), 0, sizeof(x))

static int xioctl(int fh, uint32_t request, void *arg) {
    int r;
    do {
        r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno);
    return r;
}

#define CAP_PROP_BUFFERSIZE 2

void cap_thread(
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub, 
    std::string device, 
    int image_width, int image_height, double framerate, 
    int resize_image_width, int resize_image_height
) {
    if (resize_image_width == 0 || resize_image_height == 0) {
        resize_image_width = image_width;
        resize_image_height = image_height;
    }

    const char *dev_name = device.c_str();

    // open_device
    struct stat st;
    int ret = stat(dev_name, &st);
    assert(ret != -1); // cannot identify
    assert(S_ISCHR(st.st_mode)); // not a device file

    int fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);
    assert(fd != -1); // cannot open

    // init_device
    struct v4l2_capability cap;
    ret = xioctl(fd, VIDIOC_QUERYCAP, &cap);
    assert(ret != -1); // errno == EINVAL ? no V4L2 device : errno_exit("VIDIOC_QUERYCAP");
    assert(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE); // no video capture device

    // using mmap as io method
    assert(cap.capabilities & V4L2_CAP_STREAMING); // not support streaming i/o

    /* Select video input, video standard and tune here. */
    struct v4l2_cropcap cropcap;
    CLEAR(cropcap);
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    struct v4l2_crop crop;
    ret = xioctl(fd, VIDIOC_CROPCAP, &cropcap);
    if (ret == 0) {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */
        ret = xioctl(fd, VIDIOC_S_CROP, &crop);
        if (ret == -1) {
            switch (errno) {
            case EINVAL:
                /* Cropping not supported. */
                break;
            default:
                /* Errors ignored. */
                break;
            }
        }
    } else {
        /* Errors ignored. */
    }

    struct v4l2_format fmt;
    CLEAR(fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    /* Preserve original settings as set by v4l2-ctl for example */
    ret = xioctl(fd, VIDIOC_G_FMT, &fmt);
    assert(ret != -1); // errno_exit("VIDIOC_G_FMT");

    // set custom image meta
    fmt.fmt.pix.width = image_width;
    fmt.fmt.pix.height = image_height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    ret = xioctl(fd, VIDIOC_S_FMT, &fmt);
    assert(ret != -1); // errno_exit("VIDIOC_S_FMT");
    ret = xioctl(fd, VIDIOC_G_FMT, &fmt);
    assert(ret != -1); // errno_exit("VIDIOC_G_FMT");
    assert(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_MJPEG);

    /* Buggy driver paranoia. */
    unsigned int min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
        fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
        fmt.fmt.pix.sizeimage = min;

    // ref: https://gist.github.com/TIS-Edgar/e2bd764e5594897d1835
    struct v4l2_streamparm parm;
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = framerate;
    ret = xioctl(fd, VIDIOC_S_PARM, &parm);
    assert(ret != -1);

    // init_mmap
    struct v4l2_requestbuffers req;
    CLEAR(req);
    req.count = CAP_PROP_BUFFERSIZE;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    ret = xioctl(fd, VIDIOC_REQBUFS, &req);
    assert(ret != -1); // EINVAL == errno ? not support memory mapping : errno_exit("VIDIOC_REQBUFS");
    assert(req.count >= CAP_PROP_BUFFERSIZE); // Insufficient buffer memory

    struct buffer_s {
        void *start;
        size_t length;
    };
    struct buffer_s *buffers = (struct buffer_s *)calloc(req.count, sizeof(*buffers));
    assert(buffers); // Out of memory

    unsigned int n_buffers;
    for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
        struct v4l2_buffer buf;
        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = n_buffers;

        ret = xioctl(fd, VIDIOC_QUERYBUF, &buf);
        assert(ret != -1); // errno_exit("VIDIOC_QUERYBUF");

        buffers[n_buffers].length = buf.length;
        buffers[n_buffers].start = mmap(
            NULL /* start anywhere */,
            buf.length,
            PROT_READ | PROT_WRITE /* required */,
            MAP_SHARED /* recommended */,
            fd,
            buf.m.offset
        );
        assert(buffers[n_buffers].start != MAP_FAILED); // errno_exit("mmap")
    }

    // start_capturing
    for (unsigned int i = 0; i < n_buffers; ++i) {
        struct v4l2_buffer buf;
        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        ret = xioctl(fd, VIDIOC_QBUF, &buf);
        assert(ret != -1); // errno_exit("VIDIOC_QBUF");
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ret = xioctl(fd, VIDIOC_STREAMON, &type);
    assert(ret != -1); // errno_exit("VIDIOC_STREAMON");



    // ffmpeg
    // Suppress warnings from ffmpeg libraries to avoid spamming the console
    av_log_set_level(AV_LOG_FATAL);
    av_log_set_flags(AV_LOG_SKIP_REPEATED);
    // av_log_set_flags(AV_LOG_PRINT_LEVEL);

    int m_result = 0;
    char *m_averror_str = reinterpret_cast<char *>(malloc(AV_ERROR_MAX_STRING_SIZE));

    const int m_align = 32;
    
    // device frame
    AVFrame *m_avframe_device = av_frame_alloc();
    m_avframe_device->width = image_width;
    m_avframe_device->height = image_height;
    m_avframe_device->format = AV_PIX_FMT_YUV422P;

    size_t m_avframe_device_size = static_cast<size_t>(
        av_image_get_buffer_size(
            (AVPixelFormat)m_avframe_device->format,
            m_avframe_device->width,
            m_avframe_device->height,
            m_align
        )
    );

    m_result = av_frame_get_buffer(m_avframe_device, m_align);
    if (m_result != 0) {
        av_make_error_string(m_averror_str, AV_ERROR_MAX_STRING_SIZE, m_result);
        std::cerr << m_averror_str << std::endl;
    }
    
    // rgb frame
    AVFrame *m_avframe_rgb = av_frame_alloc();
    m_avframe_rgb->width = resize_image_width;
    m_avframe_rgb->height = resize_image_height;
    m_avframe_rgb->format = AV_PIX_FMT_RGB24;
    
    size_t m_avframe_rgb_size = static_cast<size_t>(
        av_image_get_buffer_size(
            (AVPixelFormat)m_avframe_rgb->format,
            m_avframe_rgb->width,
            m_avframe_rgb->height,
            m_align
        )
    );

    m_result = av_frame_get_buffer(m_avframe_rgb, m_align);
    if (m_result != 0) {
        av_make_error_string(m_averror_str, AV_ERROR_MAX_STRING_SIZE, m_result);
        std::cerr << m_averror_str << std::endl;
    }
    
    // decoding
    AVCodec *m_avcodec = avcodec_find_decoder(AVCodecID::AV_CODEC_ID_MJPEG);
    if (!m_avcodec) {
        throw std::runtime_error("Could not find MJPEG decoder");
    }
    
    AVCodecParserContext *m_avparser = av_parser_init(AVCodecID::AV_CODEC_ID_MJPEG);
    if (!m_avparser) {
        throw std::runtime_error("Could not find MJPEG parser");
    }

    AVCodecContext *m_avcodec_context = avcodec_alloc_context3(m_avcodec);
    m_avcodec_context->width = image_width;
    m_avcodec_context->height = image_height;
    m_avcodec_context->pix_fmt = (AVPixelFormat)m_avframe_device->format;
    m_avcodec_context->codec_type = AVMEDIA_TYPE_VIDEO;

    AVDictionary *m_avoptions = NULL;

    // Initialize AVCodecContext
    if (avcodec_open2(m_avcodec_context, m_avcodec, &m_avoptions) < 0) {
        throw std::runtime_error("Could not open decoder");
        return;
    }

    SwsContext *m_sws_context = sws_getContext(
        image_width, image_height, (AVPixelFormat)m_avframe_device->format,
        resize_image_width, resize_image_height, (AVPixelFormat)m_avframe_rgb->format,
        SWS_FAST_BILINEAR,
        NULL, NULL, NULL
    );

    // mainloop
    for (;;) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        struct timeval tv;
        /* Timeout. */
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        // block until new image
        int r = select(fd + 1, &fds, NULL, NULL, &tv);
        assert(r != 0); // select timeout
        if (r == -1) {
            if (EINTR == errno)
                continue;
            else
                assert(false); // errno_exit("select");
        }

        // read_image
        struct v4l2_buffer buf;
        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        ret = xioctl(fd, VIDIOC_DQBUF, &buf);
        if (ret == -1) {
            if (errno == EAGAIN) { // errno is thread-safe, ref: https://stackoverflow.com/questions/1694164/is-errno-thread-safe
                // nanosleep((const struct timespec[]){{0, 6000000L}}, NULL); // sleep 6ms
                // will cause buffer block up
                continue; /* EAGAIN - continue select loop. */
            } else if (errno == EIO) {
                /* Could ignore EIO, see spec. */
                /* fall through */
            } else {
                assert(false); // errno_exit("VIDIOC_DQBUF"); // 这里出问题一般是摄像头掉了
            }
        }
        assert(buf.index < n_buffers);

        // prepare msg
        sensor_msgs::msg::Image::SharedPtr msg = std::make_shared<sensor_msgs::msg::Image>();
        msg->header.stamp.sec = buf.timestamp.tv_sec;
        msg->header.stamp.nanosec = buf.timestamp.tv_usec * 1000;
        msg->header.frame_id = "default_cam";
        msg->height = resize_image_height;
        msg->width = resize_image_width;
        msg->encoding = "rgb8";
        msg->is_bigendian = (rcpputils::endian::native == rcpputils::endian::big);
        msg->step = resize_image_width * 3;
        msg->data.resize(m_avframe_rgb_size);
        memset(reinterpret_cast<char *>(&msg->data[0]), 0, m_avframe_rgb_size);

        // decode
        auto avpacket = av_packet_alloc();
        av_new_packet(avpacket, buf.bytesused);
        memcpy(avpacket->data, (uint8_t *)buffers[buf.index].start, buf.bytesused);

        // Pass src MJPEG image to decoder
        m_result = avcodec_send_packet(m_avcodec_context, avpacket);

        av_packet_free(&avpacket);

        // If result is not 0, report what went wrong
        if (m_result != 0) {
            std::cerr << "Failed to send AVPacket to decode: ";            
            av_make_error_string(m_averror_str, AV_ERROR_MAX_STRING_SIZE, m_result);
            std::cerr << m_averror_str << std::endl;
            return;
        }

        m_result = avcodec_receive_frame(m_avcodec_context, m_avframe_device);

        if (m_result == AVERROR(EAGAIN) || m_result == AVERROR_EOF) {
            return;
        } else if (m_result < 0) {
            std::cerr << "Failed to recieve decoded frame from codec: ";
            av_make_error_string(m_averror_str, AV_ERROR_MAX_STRING_SIZE, m_result);
            std::cerr << m_averror_str << std::endl;
            return;
        }

        sws_scale(
            m_sws_context, m_avframe_device->data,
            m_avframe_device->linesize, 0, m_avframe_device->height,
            m_avframe_rgb->data, m_avframe_rgb->linesize
        );

        av_image_copy_to_buffer(
            const_cast<uint8_t *>(reinterpret_cast<const uint8_t *>(reinterpret_cast<char *>(&msg->data[0]))),
            m_avframe_rgb_size, m_avframe_rgb->data,
            m_avframe_rgb->linesize, (AVPixelFormat)m_avframe_rgb->format,
            m_avframe_rgb->width, m_avframe_rgb->height, m_align
        );

        pub->publish(*msg);

        // give back buffer
        ret = xioctl(fd, VIDIOC_QBUF, &buf);
        assert(ret != -1); // errno_exit("VIDIOC_QBUF");
    }

    // stop_capturing
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ret = xioctl(fd, VIDIOC_STREAMOFF, &type);
    assert(ret != -1); // errno_exit("VIDIOC_STREAMOFF");

    // uninit_device
    unsigned int i;
    for (i = 0; i < n_buffers; ++i) {
        ret = munmap(buffers[i].start, buffers[i].length);
        assert(ret != -1); // errno_exit("munmap");
    }

    free(buffers);

    // close_device
    ret = close(fd);
    assert(ret != -1); // errno_exit("close");

    fd = -1;



    // ffmpeg
    if (m_averror_str) {
        free(m_averror_str);
    }
    if (m_avoptions) {
        free(m_avoptions);
    }
    if (m_avcodec_context) {
        avcodec_close(m_avcodec_context);
        avcodec_free_context(&m_avcodec_context);
    }
    if (m_avframe_device) {
        av_frame_free(&m_avframe_device);
    }
    if (m_avframe_rgb) {
        av_frame_free(&m_avframe_rgb);
    }
    if (m_avparser) {
        av_parser_close(m_avparser);
    }
    if (m_sws_context) {
        sws_freeContext(m_sws_context);
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("simple_cap_node");
    
    node->declare_parameter("device", "/dev/video_head");
    node->declare_parameter("image_width", 640);
    node->declare_parameter("image_height", 360);
    node->declare_parameter("framerate", 30.0);
    node->declare_parameter("resize_image_width", 0);
    node->declare_parameter("resize_image_height", 0);

    std::string device = node->get_parameter("device").as_string();
    int image_width = node->get_parameter("image_width").as_int();
    int image_height = node->get_parameter("image_height").as_int();
    double framerate = node->get_parameter("framerate").as_double();
    int resize_image_width = node->get_parameter("resize_image_width").as_int();
    int resize_image_height = node->get_parameter("resize_image_height").as_int();

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher = node->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);

    std::thread th(cap_thread, 
        publisher, 
        device, 
        image_width, image_height, framerate, 
        resize_image_width, resize_image_height
    );
    rclcpp::spin(node);
    rclcpp::shutdown();
    th.join();
    return 0;
}