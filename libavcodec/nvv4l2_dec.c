/*
 * Copyright (c) 2021-2023, CTCaer <ctcaer@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include "internal.h"
#include "decode.h"
#include "thread.h"
#include "libavutil/log.h"
#include "libavutil/pixdesc.h"
#include "libavutil/opt.h"

#include "nvv4l2.h"

extern AVCodec ff_h264_decoder;
extern AVCodec ff_hevc_decoder;
extern AVCodec ff_vp9_decoder;

/*
 ** Output plane format support:
 **  S264 (H264  Encoded Slice bitstream)
 **  VP8F (VP8   Encoded Slice bitstream)
 **  H264 (H264  Encoded bitstream)
 **  H265 (H265  Encoded bitstream)
 **  VP80 (VP8   Encoded bitstream)
 **  VP90 (VP9   Encoded bitstream)
 **  MPG2 (MPEG2 Encoded bitstream)
 **  MPG4 (MPEG4 Encoded bitstream)
 **  JPEG (JPEG  Encoded bitstream)
 **  MJPG (MJPEG Encoded bitstream)
 **  DVX4 (divx  Encoded bitstream)
 **  DVX5 (divx  Encoded bitstream)
 **
 ** Capture plane format support:
 **  NM12 (YUV 4:2:0)
 */

/*
 ** Output plane memory type support:
 **  V4L2_MEMORY_MMAP
 **  V4L2_MEMORY_USERPTR
 ** Capture plane memory type support:
 **  V4L2_MEMORY_MMAP
 **  V4L2_MEMORY_DMABUF
 */

#define DECODER_DEV "/dev/nvhost-nvdec"
#define OP_PLANE_REQ_SIZEIMAGE 4000000

typedef struct {
    char eos_reached;
    nvv4l2_ctx_t *ctx;
    AVClass *av_class;
} nvv4l2DecodeContext;

static int
set_output_plane_format(nvv4l2_ctx_t *ctx, uint32_t pixfmt,
                        uint32_t sizeimage)
{
    int ret;
    struct v4l2_format format;

    memset(&format, 0, sizeof(struct v4l2_format));
    format.type = ctx->op_buf_type;
    format.fmt.pix_mp.pixelformat = pixfmt;
    format.fmt.pix_mp.num_planes = 1;
    format.fmt.pix_mp.plane_fmt[0].sizeimage = sizeimage;

    ret = v4l2_ioctl(ctx->fd, VIDIOC_S_FMT, &format);

    if (ret == 0) {
        ctx->op_num_planes = format.fmt.pix_mp.num_planes;
        for (uint32_t i = 0; i < ctx->op_num_planes; i++) {
            ctx->op_planefmts[i].stride =
                format.fmt.pix_mp.plane_fmt[i].bytesperline;
            ctx->op_planefmts[i].sizeimage =
                format.fmt.pix_mp.plane_fmt[i].sizeimage;
        }
    }

    return ret;
}

static int
set_capture_plane_format(AVCodecContext *avctx, nvv4l2_ctx_t *ctx,
                         uint32_t pixfmt, uint32_t width, uint32_t height)
{
    int ret;
    struct v4l2_format format;
    uint32_t num_bufferplanes;
    NvBufferPlaneFormat planefmts[NV_MAX_PLANES];

    nvv4l2_fill_buffer_plane_format(ctx, &num_bufferplanes, planefmts, width,
                             height, pixfmt);
    ctx->cp_num_planes = num_bufferplanes;
    for (uint32_t i = 0; i < num_bufferplanes; i++) {
        ctx->cp_planefmts[i] = planefmts[i];
    }
    memset(&format, 0, sizeof(struct v4l2_format));
    format.type = ctx->cp_buf_type;
    format.fmt.pix_mp.width = width;
    format.fmt.pix_mp.height = height;
    format.fmt.pix_mp.pixelformat = pixfmt;
    format.fmt.pix_mp.num_planes = num_bufferplanes;

    ret = v4l2_ioctl(ctx->fd, VIDIOC_S_FMT, &format);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR, "Error in VIDIOC_S_FMT!\n");
        ctx->in_error = true;
    } else {
        ctx->cp_num_planes = format.fmt.pix_mp.num_planes;
        for (uint32_t i = 0; i < ctx->cp_num_planes; i++) {
            ctx->cp_planefmts[i].stride =
                format.fmt.pix_mp.plane_fmt[i].bytesperline;
            ctx->cp_planefmts[i].sizeimage =
                format.fmt.pix_mp.plane_fmt[i].sizeimage;
        }
    }

    return ret;
}

static void query_set_capture(AVCodecContext *avctx, nvv4l2_ctx_t *ctx)
{
    struct v4l2_format format;
    struct v4l2_crop crop;
    struct v4l2_control ctl;
    int ret, cp_num_old_buffers;
    int32_t min_cap_buffers;
    NvBufferCreateParams input_params = { 0 };
    NvBufferCreateParams cap_params = { 0 };

    if (ctx->in_error || ctx->eos)
        return;

    /* Get format on capture plane set by device.
     ** This may change after an resolution change event.
     */
    format.type = ctx->cp_buf_type;
    ret = v4l2_ioctl(ctx->fd, VIDIOC_G_FMT, &format);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR,
               "Could not get format from decoder capture plane!\n");
        ctx->in_error = true;
        return;
    }

    /* Query cropping size and position. */
    crop.type = ctx->cp_buf_type;
    ret = v4l2_ioctl(ctx->fd, VIDIOC_G_CROP, &crop);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR,
               "Could not get crop from decoder capture plane!\n");
        ctx->in_error = true;
        return;
    }

    /* Set codec resolution */
    ctx->codec_width = crop.c.width;
    ctx->codec_height = crop.c.height;

    av_log(avctx, AV_LOG_VERBOSE, "Resolution changed to: %dx%d\n",
           ctx->codec_width, ctx->codec_height);

    /* Destroy all allocated transform/export DMA buffers. */
    for (uint32_t i = 0; i < NV_MAX_BUFFERS; i++) {
        if (ctx->plane_dma_fd[i] != -1) {
            ret = NvBufferDestroy(ctx->plane_dma_fd[i]);
            if (ret < 0) {
                av_log(avctx, AV_LOG_ERROR,
                       "Failed to destroy plane buffer!\n");
            } else {
                ctx->plane_dma_fd[i] = -1;
            }
        }
    }

    /*
     ** Due to VIC constrains the transformation from Block Linear to Pitch
     ** must have aligned widths to 64B. Otherwise the frame might be produced
     ** as scrambled.
     */
    ctx->plane_width_aligned = NVALIGN(crop.c.width, 64);
    if (ctx->plane_width_aligned != crop.c.width)
        av_log(avctx, AV_LOG_VERBOSE, "Linesize got aligned: %d -> %d\n",
           crop.c.width, ctx->plane_width_aligned);
    crop.c.width = ctx->plane_width_aligned;

    /* Create transform/export DMA buffers. */
    input_params.width = crop.c.width;
    input_params.height = crop.c.height;
    input_params.layout = NvBufferLayout_Pitch;
    input_params.payloadType = NvBufferPayload_SurfArray;
    input_params.nvbuf_tag = NvBufferTag_VIDEO_CONVERT;

    switch (ctx->cp_pixfmt) {
    case V4L2_PIX_FMT_YUV420M:
        input_params.colorFormat = NvBufferColorFormat_YUV420;
        break;
    case V4L2_PIX_FMT_NV12M:
        input_params.colorFormat = NvBufferColorFormat_NV12;
        if (ctx->pixfmt_list_ver == NvBufferPixFmtVersion_New)
            input_params.colorFormat++;
        break;
    }

    for (uint32_t i = 0; i < NV_MAX_BUFFERS; i++) {
        ret = NvBufferCreateEx(&ctx->plane_dma_fd[i], &input_params);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR, "Creation of dmabuf failed!\n");
            ctx->in_error = true;
        }
    }

    /* Stop streaming. */
    pthread_mutex_lock(&ctx->queue_lock);
    ret = v4l2_ioctl(ctx->fd, VIDIOC_STREAMOFF, &ctx->cp_buf_type);
    if (ret) {
        ctx->in_error = true;
    } else {
        pthread_cond_broadcast(&ctx->queue_cond);
    }
    pthread_mutex_unlock(&ctx->queue_lock);

    /* Request buffers with count 0 and destroy all
     ** previously allocated buffers.
     */
    cp_num_old_buffers = ctx->cp_num_buffers;
    ret = nvv4l2_req_buffers_on_capture_plane(ctx,
                                              ctx->cp_buf_type,
                                              ctx->cp_mem_type, 0);
    if (ret) {
        av_log(avctx, AV_LOG_WARNING,
               "Error in requesting 0 capture plane buffers!\n");
    }

    /* Destroy previous DMA buffers. */
    for (uint32_t i = 0; i < cp_num_old_buffers; i++) {
        if (ctx->dmabuff_fd[i] != -1) {
            ret = NvBufferDestroy(ctx->dmabuff_fd[i]);
            if (ret) {
                av_log(avctx, AV_LOG_ERROR,
                       "Failed to Destroy NvBuffer!\n");
                ctx->in_error = true;
            } else {
                ctx->dmabuff_fd[i] = -1;
            }
        }
    }
    if (ctx->in_error)
        return;

    /* Set capture plane format to update vars. */
    ret = set_capture_plane_format(avctx, ctx,
                                   format.fmt.pix_mp.pixelformat,
                                   format.fmt.pix_mp.width,
                                   format.fmt.pix_mp.height);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR,
               "Error in setting capture plane format!\n");
        ctx->in_error = true;
        return;
    }

    /* Get control value for min buffers which have to
     ** be requested on capture plane.
     */
    ctl.id = V4L2_CID_MIN_BUFFERS_FOR_CAPTURE;

    ret = v4l2_ioctl(ctx->fd, VIDIOC_G_CTRL, &ctl);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR, "Error getting value of control!\n");
        ctx->in_error = true;
        return;
    } else {
        min_cap_buffers = ctl.value;
    }

    /* Set color format based on colorspace and quantization type. */
    switch(format.fmt.pix_mp.colorspace)
    {
    case V4L2_COLORSPACE_REC709:
        if (format.fmt.pix_mp.quantization == V4L2_QUANTIZATION_DEFAULT)
        {
            av_log(avctx, AV_LOG_VERBOSE,
                "Colorspace ITU-R BT.709 with standard range luma (16-235)\n");
            cap_params.colorFormat = NvBufferColorFormat_NV12_709;
        }
        else
        {
            av_log(avctx, AV_LOG_VERBOSE,
                "Colorspace ITU-R BT.709 with extended range luma (0-255)\n");
            cap_params.colorFormat = NvBufferColorFormat_NV12_709_ER;
        }
        break;
    case V4L2_COLORSPACE_BT2020:
        av_log(avctx, AV_LOG_VERBOSE,
               "Colorspace ITU-R BT.2020\n");
        cap_params.colorFormat = NvBufferColorFormat_NV12_2020;
        break;
    default:
        av_log(avctx, AV_LOG_VERBOSE,
               "Colorspace details are missing, using default\n");
    case V4L2_COLORSPACE_SMPTE170M:
        if (format.fmt.pix_mp.quantization == V4L2_QUANTIZATION_DEFAULT)
        {
            av_log(avctx, AV_LOG_VERBOSE,
                "Colorspace ITU-R BT.601 with standard range luma (16-235)\n");
            cap_params.colorFormat = NvBufferColorFormat_NV12;
        }
        else
        {
            av_log(avctx, AV_LOG_VERBOSE,
                "Colorspace ITU-R BT.601 with extended range luma (0-255)\n");
            cap_params.colorFormat = NvBufferColorFormat_NV12_ER;
        }
        break;
    }

    /* Increment color format if NvBuffer is newer. */
    if (ctx->pixfmt_list_ver == NvBufferPixFmtVersion_New)
        cap_params.colorFormat++;

    /* Request number of buffers returned by ctrl, plus 10 more. */
    ctx->cp_num_buffers = min_cap_buffers + 10;

    /* Create DMA Buffers by defining the parameters for the HW Buffer. */
    cap_params.width = crop.c.width;
    cap_params.height = crop.c.height;
    cap_params.layout = NvBufferLayout_BlockLinear;
    cap_params.payloadType = NvBufferPayload_SurfArray;
    cap_params.nvbuf_tag = NvBufferTag_VIDEO_DEC;

    for (uint32_t i = 0; i < ctx->cp_num_buffers; i++) {
        ret = NvBufferCreateEx(&ctx->dmabuff_fd[i], &cap_params);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR, "Failed to create buffers!\n");
            ctx->in_error = true;
            return;
        }
    }

    /* Request buffers on capture plane. */
    ret = nvv4l2_req_buffers_on_capture_plane(ctx,
                                              ctx->cp_buf_type,
                                              ctx->cp_mem_type,
                                              ctx->cp_num_buffers);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR,
               "Error in requesting capture plane buffers!\n");
        ctx->in_error = true;
        return;
    }

    /* Set max performance mode if low latency is requested. */
    if (ctx->low_latency) {
        ret = nvv4l2_set_ext_controls(ctx->fd,
                                V4L2_CID_MPEG_VIDEO_MAX_PERFORMANCE, 0, 1);
        if (ret) {
            av_log(avctx, AV_LOG_WARNING,
                   "Failed to set control max performance!\n");
        }
    }

    /* Set streaming status ON on capture plane. */
    ret = v4l2_ioctl(ctx->fd, VIDIOC_STREAMON, &ctx->cp_buf_type);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR, "Streaming error on capture plane!\n");
        ctx->in_error = true;
        return;
    }

    /* Enqueue all empty buffers on capture plane. */
    for (uint32_t i = 0; i < ctx->cp_num_buffers; i++) {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[NV_MAX_PLANES];

        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, sizeof(planes));

        v4l2_buf.index = i;
        v4l2_buf.m.planes = planes;
        v4l2_buf.type = ctx->cp_buf_type;
        v4l2_buf.memory = ctx->cp_mem_type;
        v4l2_buf.length = ctx->cp_num_planes;
        /* Set DMA plane handle. */
        v4l2_buf.m.planes[0].m.fd = ctx->dmabuff_fd[i];
        v4l2_buf.m.planes[1].m.fd = ctx->dmabuff_fd[i];

        ret = nvv4l2_q_buffer(ctx, &v4l2_buf, ctx->cp_buffers[i],
                              ctx->cp_buf_type, ctx->cp_mem_type,
                              ctx->cp_num_planes);

        if (ret) {
            av_log(avctx, AV_LOG_ERROR, "Qing failed on capture plane!\n");
            ctx->cp_streamon = true;
            ctx->in_error = true;
            return;
        }
    }

    ctx->cp_streamon = true;

    av_log(avctx, AV_LOG_VERBOSE, "Query and set capture successful\n");

    return;
}

static void *dec_capture_thread(void *arg)
{
    nvv4l2_ctx_t *ctx = (nvv4l2_ctx_t *)arg;
    struct v4l2_event event;
    int buf_index;
    int ret;

    av_log(ctx->avctx, AV_LOG_VERBOSE, "Starting capture thread\n");

    /* Need to wait for the first Resolution change event, so that
     ** the decoder knows the stream resolution and can allocate
     ** appropriate buffers when REQBUFS is called.
     */
    do {
        /* Dequeue the subscribed event. */
        ret = nvv4l2_dq_event(ctx, &event, 50000);
        if (ret) {
            if (errno == EAGAIN) {
                av_log(ctx->avctx, AV_LOG_VERBOSE,
                       "Timeout waiting for first resolution event!\n");
            } else {
                av_log(ctx->avctx, AV_LOG_ERROR,
                       "Error in dequeuing decoder event!\n");
            }
            ctx->in_error = true;
            break;
        }
    }
    while ((event.type != V4L2_EVENT_RESOLUTION_CHANGE) &&
           !ctx->in_error && !ctx->eos);

    /* Received first resolution change event
     ** Format and buffers are now set on capture.
     */
    query_set_capture(ctx->avctx, ctx);

    /* Check for resolution event to again
     ** set format and buffers on capture plane.
     */
    while (!ctx->in_error && !ctx->eos) {
        ret = nvv4l2_dq_event(ctx, &event, 0);
        if (ret == 0) {
            switch (event.type) {
            case V4L2_EVENT_RESOLUTION_CHANGE:
                query_set_capture(ctx->avctx, ctx);
                continue;
            }
        }

        /* Main Capture loop for DQ and Q. */
        while (!ctx->eos) {
            struct v4l2_buffer v4l2_cp_buf;
            struct v4l2_plane capture_planes[NV_MAX_PLANES];
            NvBufferRect src_rect, dest_rect;
            NvBufferParams buf_params;
            NvBufferTransformParams transform_params;

            memset(&v4l2_cp_buf, 0, sizeof(v4l2_cp_buf));
            memset(capture_planes, 0, sizeof(capture_planes));
            v4l2_cp_buf.m.planes = capture_planes;

            /* Dequeue the filled buffer. */
            if (nvv4l2_dq_buffer(ctx, &v4l2_cp_buf, NULL,
                 ctx->cp_buf_type, ctx->cp_mem_type, 0)) {
                if (errno == EAGAIN) {
                    usleep(1000);
                }
                break;
            }

            /* Transformation parameters are defined
             ** which are passed to the NvBufferTransform
             ** for required conversion.
             */
            src_rect.top = 0;
            src_rect.left = 0;
            src_rect.width = ctx->codec_width;
            src_rect.height = ctx->codec_height;
            dest_rect.top = 0;
            dest_rect.left = 0;
            dest_rect.width = ctx->codec_width;
            dest_rect.height = ctx->codec_height;

            /* @transform_flag defines the flags for enabling the
             ** valid transforms. All the valid parameters are
             **  present in the nvv4l2_ext_utils header.
             */
            transform_params.transform_flag = NVBUFFER_TRANSFORM_FILTER;
            transform_params.transform_flip = NvBufferTransform_None;
            transform_params.transform_filter =
                                            NvBufferTransform_Filter_Smart;
            transform_params.src_rect = src_rect;
            transform_params.dst_rect = dest_rect;
            transform_params.session = ctx->buf_session;

            pthread_mutex_lock(&ctx->queue_lock);

            buf_index = nvv4l2_pool_idx_next(ctx, ctx->export_pool);

            /* Blocklinear to Pitch transformation is required
             ** to dump the raw decoded buffer data.
             */
            if (buf_index >= 0) {
                ret = NvBufferTransform(ctx->dmabuff_fd[v4l2_cp_buf.index],
                                        ctx->plane_dma_fd[buf_index],
                                        &transform_params);
                if (ret == -1) {
                    ctx->in_error = true;
                    av_log(ctx->avctx, AV_LOG_ERROR, "Transform failed!\n");
                    pthread_mutex_unlock(&ctx->queue_lock);
                    break;
                }

                ret = NvBufferGetParams(ctx->plane_dma_fd[buf_index],
                                        &buf_params);
                if (ret) {
                    ctx->in_error = true;
                    av_log(ctx->avctx, AV_LOG_ERROR, "GetParams failed!\n");
                    pthread_mutex_unlock(&ctx->queue_lock);
                    break;
                }
            }

            ctx->plane_width[0] = buf_params.width[0];
            ctx->plane_height[0] = buf_params.height[0];
            ctx->plane_width[1] = buf_params.width[1];
            ctx->plane_height[1] = buf_params.height[1];
            if (ctx->cp_pixfmt == V4L2_PIX_FMT_YUV420M) {
                ctx->plane_width[2] = buf_params.width[2];
                ctx->plane_height[2] = buf_params.height[2];
            }

            /* Set timestamp based on origin pts flags */
            if (buf_index >= 0) {
                if (v4l2_cp_buf.timestamp.tv_usec == 0 &&
                     v4l2_cp_buf.timestamp.tv_sec == NV_V4L2_NOPTS_VALUE) {
                    /* Origin packet had no pts and user pts values. */
                    ctx->frame_pts[buf_index] = AV_NOPTS_VALUE;
                    ctx->frame_user_pts[buf_index] = AV_NOPTS_VALUE;
                } else if (v4l2_cp_buf.timestamp.tv_sec &
                           NV_V4L2_REORDERED_OPAQUE_FLAG) {
                    /* Origin packet had only user pts value. */
                    v4l2_cp_buf.timestamp.tv_sec &=
                                   (~NV_V4L2_REORDERED_OPAQUE_FLAG);
                    ctx->frame_pts[buf_index] = AV_NOPTS_VALUE;
                    ctx->frame_user_pts[buf_index] =
                                v4l2_cp_buf.timestamp.tv_usec +
                                (v4l2_cp_buf.timestamp.tv_sec * AV_TIME_BASE);
                } else {
                    /* Origin packet had pts value. */
                    ctx->frame_pts[buf_index] =
                                v4l2_cp_buf.timestamp.tv_usec +
                                (v4l2_cp_buf.timestamp.tv_sec * AV_TIME_BASE);
                   ctx->frame_user_pts[buf_index] = AV_NOPTS_VALUE;
                }
            }

            nvv4l2_pool_push(ctx, ctx->export_pool);
            pthread_mutex_unlock(&ctx->queue_lock);

            if (ctx->low_latency) {
                pthread_mutex_lock(&ctx->frame_lock);
                pthread_cond_signal(&ctx->frame_cond);
                pthread_mutex_unlock(&ctx->frame_lock);
            }

            /* Set DMA plane handle. */
            v4l2_cp_buf.m.planes[0].m.fd = ctx->dmabuff_fd[v4l2_cp_buf.index];

            /* Queue the buffer. */
            ret = nvv4l2_q_buffer(ctx, &v4l2_cp_buf, NULL, ctx->cp_buf_type,
                                  ctx->cp_mem_type, ctx->cp_num_planes);

            if (ret) {
                av_log(ctx->avctx, AV_LOG_ERROR,
                       "Qing failed on capture plane!\n");
                if (ctx->draining_event) {
                    ctx->draining_event = false;
                    av_log(ctx->avctx, AV_LOG_ERROR,
                           "Draining event, rejecting error\n");
                } else {
                    ctx->in_error = true;
                }
                break;
            }
        }
    }

    if (ctx->low_latency) {
        pthread_mutex_lock(&ctx->frame_lock);
        pthread_cond_broadcast(&ctx->frame_cond);
        pthread_mutex_unlock(&ctx->frame_lock);
    }

    av_log(ctx->avctx, AV_LOG_VERBOSE,
           "Exiting decoder capture loop thread\n");
    return NULL;
}

int
nvv4l2_decoder_get_frame(AVCodecContext *avctx, nvv4l2_ctx_t *ctx,
                         int *buf_index, NvFrame *frame)
{
    struct timespec timeout;
    struct timeval now;
    int _buf_index;
    int ret = 0;

    /* In low latency mode, block until a decoded frame is ready. */
    if (ctx->low_latency) {
        pthread_mutex_lock(&ctx->frame_lock);
        while (atomic_load(&ctx->export_pool->capacity) == 0 &&
               !ctx->eos && !ctx->in_error && ret != ETIMEDOUT) {
            /* 500ms timeout */
            gettimeofday(&now, NULL);
            timeout.tv_nsec = (now.tv_usec + 500000L) * 1000L;
            timeout.tv_sec = now.tv_sec + timeout.tv_nsec / 1000000000L;
            timeout.tv_nsec = timeout.tv_nsec % 1000000000L;

            ret = pthread_cond_timedwait(&ctx->frame_cond,
                                         &ctx->frame_lock, &timeout);
        }
        pthread_mutex_unlock(&ctx->frame_lock);
    }

    if (ctx->export_pool->capacity == 0)
        return 1;

    _buf_index = nvv4l2_pool_pop(ctx, ctx->export_pool);

    frame->width = ctx->codec_width;
    frame->height = ctx->codec_height;
    frame->pts = ctx->frame_pts[_buf_index];
    frame->user_pts = ctx->frame_user_pts[_buf_index];

    *buf_index = _buf_index;

    return 0;

}

int
nvv4l2_decoder_put_packet(AVCodecContext *avctx, nvv4l2_ctx_t *ctx,
                          NvPacket *packet)
{
    int ret;
    /* Read the encoded data and Enqueue the output
     ** plane buffers. Exit loop in case file read is complete.
     */
    struct v4l2_buffer v4l2_buf_op;
    struct v4l2_plane queue_op_planes[NV_MAX_PLANES];
    NvBuffer *buffer;
    memset(&v4l2_buf_op, 0, sizeof(v4l2_buf_op));
    memset(queue_op_planes, 0, sizeof(queue_op_planes));
    v4l2_buf_op.m.planes = queue_op_planes;

    if (ctx->num_active_op_buffers < ctx->op_num_buffers) {
        /* Get an unused buffer to add to the queue. */
        buffer = ctx->op_buffers[ctx->num_active_op_buffers];
        v4l2_buf_op.index = ctx->num_active_op_buffers;
    } else {
        /* Dequeue a finished buffer and reuse it. */
        ret = nvv4l2_dq_buffer(ctx, &v4l2_buf_op, &buffer,
                               ctx->op_buf_type, ctx->op_mem_type, -1);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR,
                   "Error DQing buffer at output plane!\n");
            ctx->in_error = true;
            return -1;
        }
    }

    /* Copy packet data. */
    memcpy(buffer->planes[0].data, packet->payload, packet->payload_size);
    buffer->planes[0].bytesused = packet->payload_size;

    v4l2_buf_op.m.planes[0].bytesused = buffer->planes[0].bytesused;

    /* Set timestamp based on packet flags. */
    v4l2_buf_op.flags |= V4L2_BUF_FLAG_TIMESTAMP_COPY;
    if (packet->pts != AV_NOPTS_VALUE) {
        /* Packet pts is valid */
        v4l2_buf_op.timestamp.tv_sec = packet->pts / AV_TIME_BASE;
        v4l2_buf_op.timestamp.tv_usec = packet->pts % AV_TIME_BASE;
    } else if (packet->user_pts != AV_NOPTS_VALUE) {
        /* User pts is valid */
        v4l2_buf_op.timestamp.tv_sec = packet->user_pts / AV_TIME_BASE;
        v4l2_buf_op.timestamp.tv_usec = packet->user_pts % AV_TIME_BASE;
        v4l2_buf_op.timestamp.tv_sec |= NV_V4L2_REORDERED_OPAQUE_FLAG;
    } else {
        /* No valid pts or user pts */
        v4l2_buf_op.timestamp.tv_sec = NV_V4L2_NOPTS_VALUE;
        v4l2_buf_op.timestamp.tv_usec = 0;
    }

    /* Queue packet on output plane. */
    ret = nvv4l2_q_buffer(ctx, &v4l2_buf_op, buffer,
                   ctx->op_buf_type, ctx->op_mem_type, ctx->op_num_planes);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR, "Error Qing buffer at output plane!\n");
        ctx->in_error = true;
        return -1;
    }

    if (ctx->num_active_op_buffers < ctx->op_num_buffers) {
        ctx->num_active_op_buffers++;
    }

    if (v4l2_buf_op.m.planes[0].bytesused == 0) {
        ctx->eos = true;
        av_log(avctx, AV_LOG_VERBOSE, "Input file read complete\n");
    }

    return 0;
}

nvv4l2_ctx_t *nvv4l2_create_decoder(AVCodecContext *avctx,
                                    NvCodingType nv_codec_type,
                                    int pix_fmt)
{
    nvv4l2_ctx_t *ctx = (nvv4l2_ctx_t *)NVCALLOC(1, sizeof(nvv4l2_ctx_t));
    int ret = 0;
    int flags = 0;
    ctx->avctx = avctx;

    /* The call creates a new V4L2 Video Decoder object
     ** on the device node "/dev/nvhost-nvdec"
     ** Additional flags can also be given with which the device
     ** should be opened.
     ** This opens the device in Blocking mode.
     */
    ctx->fd = v4l2_open(DECODER_DEV, flags | O_RDWR);
    if (ctx->fd == -1) {
        av_log(avctx, AV_LOG_ERROR, "Could not open device!\n");
        ctx->in_error = true;
        return ctx;
    }

    /* Initialization. */
    ctx->cp_pixfmt = pix_fmt;
    ctx->op_pixfmt = nvv4l2_map_nvcodec_type(nv_codec_type);

    /* Get NvBuffer pixel format list version */
    ctx->pixfmt_list_ver = nvv4l2_get_pixfmt_list_version(ctx);

    /* Get a NvBuffer session for interprocess transforms */
    ctx->buf_session = NvBufferSessionCreate();

    /* Decoder code assumes that the following do not change.
     ** If another memory type is wanted, relevant changes should be done
     ** to the rest of the code.
     */
    ctx->op_mem_type = V4L2_MEMORY_USERPTR;
    ctx->cp_mem_type = V4L2_MEMORY_DMABUF;

    ctx->op_buf_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    ctx->cp_buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

    for (uint32_t i = 0; i < NV_MAX_BUFFERS; i++) {
        ctx->dmabuff_fd[i] = -1;
        ctx->plane_dma_fd[i] = -1;
    }

    /* Allocate packet pool. */
    ctx->export_pool = (NvQueues *)NVCALLOC(1, sizeof(NvQueues));

    /* Initialize mutexes */
    pthread_mutex_init(&ctx->queue_lock, NULL);
    pthread_mutex_init(&ctx->frame_lock, NULL);
    pthread_cond_init(&ctx->queue_cond, NULL);
    pthread_cond_init(&ctx->frame_cond, NULL);

    /* Subscribe to Resolution change event.
     ** This is required to catch whenever resolution change event
     ** is triggered to set the format on capture plane.
     */
    ret = nvv4l2_subscribe_event(ctx->fd,
                                 V4L2_EVENT_RESOLUTION_CHANGE,
                                 0, 0);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR,
               "Failed to subscribe for resolution change!\n");
        ctx->in_error = true;
    }

    /* Set format on output plane.
     ** The format of the encoded bitstream is set.
     */
    ret = set_output_plane_format(ctx, ctx->op_pixfmt, OP_PLANE_REQ_SIZEIMAGE);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR,
               "Error in setting output plane format!\n");
        ctx->in_error = true;
    }

    /* Set appropriate controls.
     ** V4L2_CID_MPEG_VIDEO_DISABLE_COMPLETE_FRAME_INPUT control is
     ** set to false as the application always sends NALUs.
     ** Also, mandatory when V4L2_BUF_FLAG_TIMESTAMP_COPY is used.
     */
    ret =
        nvv4l2_set_ext_controls(ctx->fd,
                         V4L2_CID_MPEG_VIDEO_DISABLE_COMPLETE_FRAME_INPUT,
                         0, 0);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR,
               "Failed to set control enable complete frame!\n");
        ctx->in_error = true;
    }

    /* Request buffers on output plane to fill
     ** the input bitstream.
     */
    ret = nvv4l2_req_buffers_on_output_plane(ctx,
                                             ctx->op_buf_type,
                                             ctx->op_mem_type, 10);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR,
               "Error in requesting buffers on output plane!\n");
        ctx->in_error = true;
    }

    for (uint32_t i = 0; i < ctx->op_num_buffers; i++) {
        if (nvv4l2_allocate_memory(ctx, ctx->op_buffers[i])) {
            av_log(avctx, AV_LOG_ERROR,
                   "Buffer mapping error on output plane!\n");
            ctx->in_error = true;
        }
    }

    /* Start stream processing on output plane
     ** by setting the streaming status ON.
     */
    ret = v4l2_ioctl(ctx->fd, VIDIOC_STREAMON, &ctx->op_buf_type);
    if (ret != 0) {
        av_log(avctx, AV_LOG_ERROR, "Streaming error on output plane!\n");
        ctx->in_error = true;
    }

    ctx->op_streamon = true;

    /* Create and start capture loop thread. */
    pthread_create(&ctx->capture_thread, NULL, dec_capture_thread, ctx);

    return ctx;
}

int nvv4l2_decoder_close(AVCodecContext *avctx, nvv4l2_ctx_t *ctx)
{
    int ret, cp_num_old_buffers;

    if (!ctx)
        return 0;

    pthread_mutex_lock(&ctx->queue_lock);
    ctx->eos = true;
    pthread_mutex_unlock(&ctx->queue_lock);
    if (ctx->fd != -1) {
        /* Stop streaming on both planes. */
        ret = v4l2_ioctl(ctx->fd, VIDIOC_STREAMOFF, &ctx->op_buf_type);
        ret = v4l2_ioctl(ctx->fd, VIDIOC_STREAMOFF, &ctx->cp_buf_type);
        ctx->op_streamon = false;
        ctx->cp_streamon = false;

        /* Wait for capture thread to exit. */
        if (ctx->capture_thread) {
            pthread_join(ctx->capture_thread, NULL);
        }

        /* Request 0 buffers on both planes. */
        nvv4l2_req_buffers_on_output_plane(ctx,
                                           ctx->op_buf_type,
                                           ctx->op_mem_type, 0);

        cp_num_old_buffers = ctx->cp_num_buffers;
        nvv4l2_req_buffers_on_capture_plane(ctx,
                                            ctx->cp_buf_type,
                                            ctx->cp_mem_type, 0);

        /* All allocated DMA buffers must be destroyed. */
        for (uint32_t i = 0; i < cp_num_old_buffers; i++) {
            if (ctx->dmabuff_fd[i] != -1) {
                ret = NvBufferDestroy(ctx->dmabuff_fd[i]);
                if (ret < 0) {
                    av_log(avctx, AV_LOG_ERROR,
                           "Failed to destroy dma buffer!\n");
                }
                ctx->dmabuff_fd[i] = -1;
            }
        }

        /* Destroy all allocated transform/export DMA buffers. */
        for (uint32_t i = 0; i < NV_MAX_BUFFERS; i++) {
            if (ctx->plane_dma_fd[i] != -1) {
                ret = NvBufferDestroy(ctx->plane_dma_fd[i]);
                if (ret < 0) {
                    av_log(avctx, AV_LOG_ERROR,
                           "Failed to destroy plane buffer!\n");
                }
                ctx->plane_dma_fd[i] = -1;
            }
        }

        /* Destroy NvBuffer session. */
        if (ctx->buf_session)
            NvBufferSessionDestroy(ctx->buf_session);

        NVFREE(ctx->export_pool);

        /* Close the opened V4L2 device. */
        ret = v4l2_close(ctx->fd);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR, "Unable to close the device!\n");
        }

        /* Free mutexes */
        pthread_mutex_destroy(&ctx->queue_lock);
        pthread_mutex_destroy(&ctx->frame_lock);
        pthread_cond_destroy(&ctx->queue_cond);
        pthread_cond_destroy(&ctx->frame_cond);
    }

    /* Report application run status on exit. */
    if (ctx->in_error) {
        av_log(avctx, AV_LOG_VERBOSE, "Decoder Run failed\n");
    } else {
        av_log(avctx, AV_LOG_VERBOSE, "Decoder Run was successful\n");
    }

    NVFREE(ctx);

    return ret;
}

static NvCodingType map_avcodec_id(enum AVCodecID id)
{
    switch (id) {
    case AV_CODEC_ID_H264:
        return NvVideoCodec_H264;
    case AV_CODEC_ID_HEVC:
        return NvVideoCodec_HEVC;
    case AV_CODEC_ID_MPEG2VIDEO:
        return NvVideoCodec_MPEG2;
    case AV_CODEC_ID_MPEG4:
        return NvVideoCodec_MPEG4;
    case AV_CODEC_ID_VP8:
        return NvVideoCodec_VP8;
    case AV_CODEC_ID_VP9:
        return NvVideoCodec_VP9;
    }
    return NvVideoCodec_UNDEFINED;
}

static int nvv4l2dec_codec_fallback(AVCodecContext *avctx)
{
    av_log(avctx, AV_LOG_WARNING, "Falling back to software decoding.\n");

    switch (avctx->codec_id) {
    case AV_CODEC_ID_H264:
        avctx->codec = &ff_h264_decoder;
        break;
    case AV_CODEC_ID_HEVC:
        avctx->codec = &ff_hevc_decoder;
        break;
    case AV_CODEC_ID_VP9:
        avctx->codec = &ff_vp9_decoder;
        break;
    default:
        av_log(avctx, AV_LOG_ERROR, "Unsupported codec fallback!\n");
        return AVERROR_BUG;
    }

    av_opt_free(avctx->priv_data);

    if (avctx->codec->priv_data_size > 0) {
        avctx->priv_data = av_mallocz(avctx->codec->priv_data_size);
        if (!avctx->priv_data)
            return AVERROR(ENOMEM);
    }

    if (HAVE_THREADS
        && !(avctx->internal->frame_thread_encoder && (avctx->active_thread_type&FF_THREAD_FRAME))) {
        ff_thread_init(avctx);
    }
    if (!HAVE_THREADS && !(avctx->codec->caps_internal & FF_CODEC_CAP_AUTO_THREADS))
        avctx->thread_count = 1;

    if (avctx->codec->priv_class) {
        *(const AVClass **)avctx->priv_data = avctx->codec->priv_class;
        av_opt_set_defaults(avctx->priv_data);
    }

    return avctx->codec->init(avctx);
}

static int nvv4l2dec_init(AVCodecContext *avctx)
{
    nvv4l2DecodeContext *nvv4l2_ctx = avctx->priv_data;
    NvCodingType nv_codec_type = map_avcodec_id(avctx->codec_id);
    int pix_fmt;

    if (nv_codec_type == NvVideoCodec_UNDEFINED) {
        av_log(avctx, AV_LOG_ERROR, "Unsupported codec ID %d!\n",
               avctx->codec_id);
        return AVERROR_BUG;
    }

    switch (avctx->pix_fmt) {
    case AV_PIX_FMT_NONE:
    case AV_PIX_FMT_YUV420P:
        avctx->pix_fmt = AV_PIX_FMT_YUV420P;
        pix_fmt = V4L2_PIX_FMT_YUV420M;
        break;
    case AV_PIX_FMT_NV12:
        pix_fmt = V4L2_PIX_FMT_NV12M;
        break;
    case AV_PIX_FMT_YUV420P10LE:
    case AV_PIX_FMT_P010:
        if (avctx->codec_id == AV_CODEC_ID_HEVC) {
            avctx->pix_fmt = AV_PIX_FMT_YUV420P;
            pix_fmt = V4L2_PIX_FMT_YUV420M;
            break;
        }
    default:
        av_log(avctx, AV_LOG_WARNING, "Unsupported pixel format %s!\n",
               av_get_pix_fmt_name(avctx->pix_fmt));
        return nvv4l2dec_codec_fallback(avctx);
    }

    nvv4l2_ctx->ctx = nvv4l2_create_decoder(avctx, nv_codec_type, pix_fmt);

    if (!nvv4l2_ctx->ctx || nvv4l2_ctx->ctx->in_error) {
        av_log(avctx, AV_LOG_ERROR, "Failed to create nvv4l2 decoder!\n");

        if (nvv4l2_ctx->ctx && nvv4l2_ctx->ctx->in_error) {
            nvv4l2_decoder_close(avctx, nvv4l2_ctx->ctx);
        }

        return AVERROR_UNKNOWN;
    }

    /*
     ** Check if low latency is needed.
     ** Depends on whole frames received instead of slices.
     ** Otherwise the decoder only starts streaming after a
     ** required amount of packets received.
     */
    nvv4l2_ctx->ctx->low_latency =
                (avctx->flags & AV_CODEC_FLAG_LOW_DELAY) ? true : false;

    return 0;
}

static void nvv4l2dec_flush(AVCodecContext *avctx)
{
    nvv4l2DecodeContext *nvv4l2_ctx = avctx->priv_data;
    nvv4l2_ctx_t *ctx = nvv4l2_ctx->ctx;
    int ret = 0;

    pthread_mutex_lock(&ctx->queue_lock);
    /* Flush all queued buffers from output and capture plane. */
    if (ctx->op_streamon && ctx->cp_streamon &&
       (ctx->num_queued_op_buffers || ctx->num_active_op_buffers)) {
        /* Stop streaming on both planes. */
        v4l2_ioctl(ctx->fd, VIDIOC_STREAMOFF, &ctx->op_buf_type);
        v4l2_ioctl(ctx->fd, VIDIOC_STREAMOFF, &ctx->cp_buf_type);
        ctx->op_streamon = false;
        ctx->cp_streamon = false;

        /* Turn on output plane streaming. */
        ret = v4l2_ioctl(ctx->fd, VIDIOC_STREAMON, &ctx->op_buf_type);
        if (ret != 0) {
            av_log(avctx, AV_LOG_ERROR, "Streaming error on output plane!\n");
            ctx->in_error = true;
        } else {
            ctx->op_streamon = true;
        }

        ctx->draining_event = true;
        ctx->num_active_op_buffers = 0;
        ctx->num_queued_op_buffers = 0;
        ctx->num_queued_cp_buffers = 0;

        /* Turn on capture plane streaming. */
        ret = v4l2_ioctl(ctx->fd, VIDIOC_STREAMON, &ctx->cp_buf_type);
        if (ret != 0) {
            av_log(avctx, AV_LOG_ERROR, "Streaming error on capture plane!\n");
            ctx->in_error = true;
        } else {
            /* Re-enqueue all now empty buffers on capture plane. */
            for (uint32_t i = 0; i < ctx->cp_num_buffers; i++) {
                struct v4l2_buffer v4l2_buf;
                struct v4l2_plane planes[NV_MAX_PLANES];

                memset(&v4l2_buf, 0, sizeof(v4l2_buf));
                memset(planes, 0, sizeof(planes));

                v4l2_buf.index = i;
                v4l2_buf.m.planes = planes;
                v4l2_buf.type = ctx->cp_buf_type;
                v4l2_buf.memory = ctx->cp_mem_type;
                v4l2_buf.length = ctx->cp_num_planes;
                /* Set DMA plane handle */
                v4l2_buf.m.planes[0].m.fd = ctx->dmabuff_fd[i];
                v4l2_buf.m.planes[1].m.fd = ctx->dmabuff_fd[i];

                pthread_mutex_unlock(&ctx->queue_lock);
                ret = nvv4l2_q_buffer(ctx, &v4l2_buf, ctx->cp_buffers[i],
                                      ctx->cp_buf_type, ctx->cp_mem_type,
                                      ctx->cp_num_planes);
                pthread_mutex_lock(&ctx->queue_lock);

                if (ret) {
                    av_log(avctx, AV_LOG_WARNING,
                           "Qing empty failed on capture plane!\n");
                }
            }

            ctx->cp_streamon = true;
        }
    }

    /* Flush all decoded frames from frame pool */
    while (ctx->export_pool->capacity != 0) {
        nvv4l2_pool_pop(ctx, ctx->export_pool);
    }
    ctx->export_pool->front = 0;
    ctx->export_pool->back = 0;
    pthread_mutex_unlock(&ctx->queue_lock);
}

static int nvv4l2dec_close(AVCodecContext *avctx)
{
    nvv4l2DecodeContext *nvv4l2_ctx = avctx->priv_data;
    return nvv4l2_decoder_close(avctx, nvv4l2_ctx->ctx);
}

static int
nvv4l2dec_decode(AVCodecContext *avctx, void *data, int *got_frame,
                 AVPacket *avpkt)
{
    nvv4l2DecodeContext *nvv4l2_ctx = avctx->priv_data;
    nvv4l2_ctx_t *ctx = nvv4l2_ctx->ctx;
    AVFrame *avframe = (AVFrame *)data;
    NvFrame _nvframe = { 0 };
    int processed_size = 0;
    int buf_index = -1;

    if (ctx->in_error) {
        return AVERROR_UNKNOWN;
    }

    if (avpkt->size) {
        NvPacket packet;
        packet.payload_size = avpkt->size;
        packet.payload = avpkt->data;
        packet.pts = avpkt->pts;
        packet.user_pts = avctx->reordered_opaque;

        if (!nvv4l2_decoder_put_packet(avctx, ctx, &packet)) {
            processed_size = avpkt->size;
        } else {
            return AVERROR_UNKNOWN;
        }
    }

    /* Get a decoded frame if any. */
    if (nvv4l2_decoder_get_frame(avctx, ctx, &buf_index, &_nvframe))
        return processed_size;

    /* Set coded width to aligned size to fit the transformation.
     ** It gets restored after transformation by default.
     */
    avctx->coded_width = ctx->plane_width_aligned;

    /* Get frame data buffers. */
    if (ff_get_buffer(avctx, avframe, 0) < 0)
        return AVERROR(ENOMEM);

    /* Export decoded frame data. */
    if (buf_index >= 0 && avframe->data[0]) {
        NvBuffer2Raw(ctx->plane_dma_fd[buf_index], 0,
                     ctx->plane_width[0], ctx->plane_height[0],
                     avframe->data[0]);
        NvBuffer2Raw(ctx->plane_dma_fd[buf_index], 1,
                     ctx->plane_width[1], ctx->plane_height[1],
                     avframe->data[1]);
        if (ctx->cp_pixfmt == V4L2_PIX_FMT_YUV420M) {
            NvBuffer2Raw(ctx->plane_dma_fd[buf_index], 2,
                         ctx->plane_width[2], ctx->plane_height[2],
                         avframe->data[2]);
        }
    }

    avframe->width = _nvframe.width;
    avframe->height = _nvframe.height;

    avframe->format = avctx->pix_fmt;
    avframe->pkt_dts = AV_NOPTS_VALUE;

    /* Decide which timestamps to set. */
    if (_nvframe.pts != AV_NOPTS_VALUE) {
        avframe->pts = _nvframe.pts;
    } else {
        avframe->pts = _nvframe.pts;
        avframe->reordered_opaque = _nvframe.user_pts;
    }

    avframe->key_frame = 0;

    avctx->coded_width = _nvframe.width;
    avctx->coded_height = _nvframe.height;
    avctx->width = _nvframe.width;
    avctx->height = _nvframe.height;

    *got_frame = 1;

    return processed_size;
}

#define NVV4L2_DEC_CLASS(NAME)                         \
    static const AVClass nvv4l2_##NAME##_dec_class = { \
        .class_name = "nvv4l2_" #NAME "_dec",          \
        .version    = LIBAVUTIL_VERSION_INT,           \
    };

#define NVV4L2_DEC(NAME, ID, BSFS)                                                    \
    NVV4L2_DEC_CLASS(NAME)                                                            \
    AVCodec ff_##NAME##_nvv4l2_decoder = {                                            \
        .name           = #NAME "_nvv4l2",                                            \
        .long_name      = NULL_IF_CONFIG_SMALL(#NAME " NVV4L2 HW decoder for Tegra"), \
        .type           = AVMEDIA_TYPE_VIDEO,                                         \
        .id             = ID,                                                         \
        .priv_data_size = sizeof(nvv4l2DecodeContext),                                \
        .init           = nvv4l2dec_init,                                             \
        .close          = nvv4l2dec_close,                                            \
        .decode         = nvv4l2dec_decode,                                           \
        .flush          = nvv4l2dec_flush,                                            \
        .priv_class     = &nvv4l2_##NAME##_dec_class,                                 \
        .capabilities   = AV_CODEC_CAP_DELAY | AV_CODEC_CAP_HARDWARE |                \
                          AV_CODEC_CAP_AVOID_PROBING,                                 \
        .bsfs           = BSFS,                                                       \
        .wrapper_name   = "nvv4l2",                                                   \
        .pix_fmts       =(const enum AVPixelFormat[]){ AV_PIX_FMT_YUV420P,            \
                                                       AV_PIX_FMT_NV12,               \
                                                       AV_PIX_FMT_NONE },             \
    };

NVV4L2_DEC(h264,  AV_CODEC_ID_H264,       "h264_mp4toannexb");
NVV4L2_DEC(hevc,  AV_CODEC_ID_HEVC,       "hevc_mp4toannexb");
NVV4L2_DEC(mpeg2, AV_CODEC_ID_MPEG2VIDEO, NULL);
NVV4L2_DEC(mpeg4, AV_CODEC_ID_MPEG4,      NULL);
NVV4L2_DEC(vp9,   AV_CODEC_ID_VP9,        NULL);
NVV4L2_DEC(vp8,   AV_CODEC_ID_VP8,        NULL);
