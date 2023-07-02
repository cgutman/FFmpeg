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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include "internal.h"
#include "libavutil/imgutils.h"
#include "libavutil/log.h"
#include "libavutil/opt.h"

#include "nvv4l2.h"

#define ENCODER_DEV "/dev/nvhost-msenc"
#define PACKET_DEFAULT_SIZE (2*1024*1024)

/*
 ** Output plane format support:
 **  YM12 (YUV 4:2:0)
 **  NM12 (YUV 4:2:0 interleaved)
 **  YM24 (YUV 4:4:4)
 **  PM10 (YUV 4:2:0 10-bit interleaved)
 **
 ** Capture plane format support:
 **  H264 (H264 Encoded bitstream)
 **  H265 (H265 Encoded bitstream)
 **  VP80 (VP8  Encoded bitstream)
 */

/*
 ** Output plane memory type support:
 **  V4L2_MEMORY_MMAP
 **  V4L2_MEMORY_DMABUF
 ** Capture plane memory type support:
 **  V4L2_MEMORY_MMAP
 */

typedef struct {
    const AVClass *class;
    nvv4l2_ctx_t *ctx;
    int num_capture_buffers;
    int profile;
    int level;
    int tier;
    int rc;
    int preset;
    int lossless;
    int twopass;
} nvv4l2EncodeContext;

static int
set_output_plane_format(AVCodecContext *avctx, nvv4l2_ctx_t *ctx,
                        uint32_t pixfmt, uint32_t width, uint32_t height)
{
    int ret;
    struct v4l2_format format;
    uint32_t num_bufferplanes;
    NvBufferPlaneFormat planefmts[NV_MAX_PLANES];

    /* Get plane format */
    ret = nvv4l2_fill_buffer_plane_format(ctx, &num_bufferplanes, planefmts,
                                          width, height, pixfmt);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR,
               "Error getting output plane format!\n");
        ctx->in_error = true;
        return ret;
    }
    ctx->op_num_planes = num_bufferplanes;

    /* Set plane format. */
    for (uint32_t j = 0; j < num_bufferplanes; ++j) {
        ctx->op_planefmts[j] = planefmts[j];
    }
    memset(&format, 0, sizeof(struct v4l2_format));
    format.type = ctx->op_buf_type;
    format.fmt.pix_mp.width = width;
    format.fmt.pix_mp.height = height;
    format.fmt.pix_mp.pixelformat = pixfmt;
    format.fmt.pix_mp.num_planes = num_bufferplanes;

    ret = v4l2_ioctl(ctx->fd, VIDIOC_S_FMT, &format);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR,
               "Error in setting output plane format!\n");
        ctx->in_error = true;
    } else {
        ctx->op_num_planes = format.fmt.pix_mp.num_planes;
        for (uint32_t j = 0; j < ctx->op_num_planes; j++) {
            ctx->op_planefmts[j].stride =
                format.fmt.pix_mp.plane_fmt[j].bytesperline;
            ctx->op_planefmts[j].sizeimage =
                format.fmt.pix_mp.plane_fmt[j].sizeimage;
        }
    }

    return ret;
}

static int
set_capture_plane_format(AVCodecContext *avctx, nvv4l2_ctx_t *ctx,
                         uint32_t pixfmt, uint32_t width,
                         uint32_t height, uint32_t sizeimage)
{
    int ret;
    struct v4l2_format format;

    memset(&format, 0, sizeof(struct v4l2_format));
    format.type = ctx->cp_buf_type;
    format.fmt.pix_mp.pixelformat = pixfmt;
    format.fmt.pix_mp.width = width;
    format.fmt.pix_mp.height = height;
    format.fmt.pix_mp.num_planes = 1;
    format.fmt.pix_mp.plane_fmt[0].sizeimage = sizeimage;

    ret = v4l2_ioctl(ctx->fd, VIDIOC_S_FMT, &format);

    if (ret == 0) {
        ctx->cp_num_planes = format.fmt.pix_mp.num_planes;
        for (uint32_t i = 0; i < ctx->cp_num_planes; ++i) {
            ctx->cp_planefmts[i].stride =
                format.fmt.pix_mp.plane_fmt[i].bytesperline;
            ctx->cp_planefmts[i].sizeimage =
                format.fmt.pix_mp.plane_fmt[i].sizeimage;
        }
    } else {
        av_log(avctx, AV_LOG_ERROR,
               "Error in setting capture plane format!\n");
        ctx->in_error = true;
    }

    return ret;
}

static void *enc_capture_thread(void *arg)
{
    nvv4l2_ctx_t *ctx = (nvv4l2_ctx_t *)arg;
    uint32_t packet_size;
    int buf_index;
    int ret;

    /* Check for EOS event in case stream finished. */
    while (!ctx->in_error && !ctx->eos) {
        /* Main Capture loop for DQ and Q. */
        struct v4l2_buffer v4l2_cp_buf;
        struct v4l2_plane capture_planes[NV_MAX_PLANES];
        v4l2_ctrl_videoenc_outputbuf_metadata enc_metadata;
        NvBuffer *cp_buffer = NULL;

        memset(&v4l2_cp_buf, 0, sizeof(v4l2_cp_buf));
        memset(capture_planes, 0, sizeof(capture_planes));
        v4l2_cp_buf.m.planes = capture_planes;
        v4l2_cp_buf.length = 1;

        /* Dequeue the filled buffer. */
        if (nvv4l2_dq_buffer(ctx, &v4l2_cp_buf, &cp_buffer,
             ctx->cp_buf_type, ctx->cp_mem_type, 0)) {
            if (errno == EAGAIN) {
                usleep(1000);
            }
            continue;
        }

        packet_size = cp_buffer->planes[0].bytesused;

        if (packet_size == 0) {
            av_log(ctx->avctx, AV_LOG_ERROR,
                   "Got 0 size buffer in capture!\n");
            ctx->in_error = true;
            break;
        }

        buf_index = nvv4l2_pool_idx_next(ctx, ctx->export_pool);

        if (buf_index >= 0) {
            /* Ensure packet buffer fits new packet */
            if (ctx->packet_buf_size[buf_index] < packet_size) {
                NVFREE(ctx->packet[buf_index]);
                ctx->packet[buf_index] = (unsigned char *)NVMALLOC(packet_size);
                ctx->packet_buf_size[buf_index] = packet_size;
            }

            ctx->packet_size[buf_index] = packet_size;
            memcpy(ctx->packet[buf_index], cp_buffer->planes[0].data,
                   packet_size);

            ctx->frame_pts[buf_index] = v4l2_cp_buf.timestamp.tv_usec +
                                        (v4l2_cp_buf.timestamp.tv_sec *
                                         AV_TIME_BASE);

            ret = nvv4l2_get_ext_control_metadata(ctx->fd,
                                                  v4l2_cp_buf.index,
                                                  &enc_metadata);
            if (ret) {
                av_log(ctx->avctx, AV_LOG_ERROR,
                       "Failed getting metadata!\n");
                ctx->in_error = true;
                break;
            }
            ctx->packet_keyflag[buf_index] =
                                    enc_metadata.KeyFrame ? true : false;
        }

        nvv4l2_pool_push(ctx, ctx->export_pool);

        /* Queue the buffer. */
        ret = nvv4l2_q_buffer(ctx, &v4l2_cp_buf, cp_buffer, ctx->cp_buf_type,
                              ctx->cp_mem_type, ctx->cp_num_planes);

        if (ret) {
            av_log(ctx->avctx, AV_LOG_ERROR,
                   "Qing failed on capture plane!\n");
            ctx->in_error = true;
            break;
        }
    }

    av_log(ctx->avctx, AV_LOG_VERBOSE,
           "Exiting encoder capture loop thread\n");

    return NULL;
}

nvv4l2_ctx_t *nvv4l2_create_encoder(AVCodecContext *avctx,
                                    NvEncoder *encoder,
                                    NvCodingType nv_codec_type,
                                    int pix_fmt)
{
    nvv4l2EncodeContext *nvv4l2_ctx = avctx->priv_data;

    int ret;
    int flags = 0;
    nvv4l2_ctx_t *ctx = (nvv4l2_ctx_t *)NVCALLOC(1, sizeof(nvv4l2_ctx_t));
    ctx->avctx = avctx;
    ctx->enc = encoder;

    /* The call creates a new V4L2 Video Decoder object
     ** on the device node "/dev/nvhost-msenc"
     ** Additional flags can also be given with which the device
     ** should be opened.
     ** This opens the device in Blocking mode.
     */
    ctx->fd = v4l2_open(ENCODER_DEV, flags | O_RDWR);
    if (ctx->fd == -1) {
        av_log(avctx, AV_LOG_ERROR, "Could not open device!\n");
        ctx->in_error = true;
        return ctx;
    }

    /* Initialization. */
    ctx->codec_width = encoder->width;
    ctx->codec_height = encoder->height;
    ctx->low_latency = encoder->low_latency;
    ctx->op_pixfmt = pix_fmt;
    ctx->cp_pixfmt = nvv4l2_map_nvcodec_type(nv_codec_type);

    /* Get NvBuffer pixel format list version */
    ctx->pixfmt_list_ver = nvv4l2_get_pixfmt_list_version(ctx);

    /* Encoder code assumes that the following do not change.
     ** If another memory type is wanted, relevant changes should be done
     ** to the rest of the code.
     */
    ctx->op_mem_type = V4L2_MEMORY_DMABUF;
    ctx->cp_mem_type = V4L2_MEMORY_MMAP;

    ctx->op_buf_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    ctx->cp_buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

    for (uint32_t i = 0; i < NV_MAX_BUFFERS; i++)
        ctx->plane_dma_fd[i] = -1;

    /* Allocate packet pool. */
    ctx->export_pool = (NvQueues *)NVCALLOC(1, sizeof(NvQueues));
    for(int index = 0; index < NV_MAX_BUFFERS; index++) {
        ctx->packet[index] = (unsigned char *)NVMALLOC(PACKET_DEFAULT_SIZE);
        ctx->packet_buf_size[index] = PACKET_DEFAULT_SIZE;
    }

    /* Initialize mutexes */
    pthread_mutex_init(&ctx->queue_lock, NULL);
    pthread_cond_init(&ctx->queue_cond, NULL);

    /* Set format on capture plane. */
    ret = set_capture_plane_format(avctx, ctx, ctx->cp_pixfmt,
                                   ctx->codec_width, ctx->codec_height,
                                   PACKET_DEFAULT_SIZE);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR,
               "Error in setting capture plane format!\n");
        ctx->in_error = true;
    }

    /* Set format on output plane. */
    ret = set_output_plane_format(avctx, ctx, ctx->op_pixfmt,
                                  ctx->codec_width, ctx->codec_height);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR,
               "Error in setting output plane format!\n");
        ctx->in_error = true;
    }

    /* Set max performance mode if low latency is requested. */
    if (ctx->low_latency) {
        ret =
        nvv4l2_set_ext_controls(ctx->fd,
                                V4L2_CID_MPEG_VIDEO_MAX_PERFORMANCE, 0, 1);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR,
                   "Failed to set control max performance!\n");
            ctx->in_error = true;
        }
    }

    /* Set encoder bitrate. */
    ret = nvv4l2_set_ext_controls(ctx->fd, V4L2_CID_MPEG_VIDEO_BITRATE,
                                  V4L2_CTRL_CLASS_MPEG,
                                  ctx->enc->bitrate);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR,
               "Failed to set encoder bitrate!\n");
        ctx->in_error = true;
    }

    /* Set encoder HW Preset Type. */
    ret = nvv4l2_set_ext_controls(ctx->fd,
                                  V4L2_CID_MPEG_VIDEOENC_HW_PRESET_TYPE_PARAM,
                                  V4L2_CTRL_CLASS_MPEG,
                                  ctx->enc->preset_type);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR,
               "Failed to set encoder HW Preset Type!\n");
        ctx->in_error = true;
    }

    /* Set number of reference frames. */
    if (ctx->enc->num_ref) {
        ret = nvv4l2_set_ext_controls(ctx->fd,
                                V4L2_CID_MPEG_VIDEOENC_NUM_REFERENCE_FRAMES,
                                V4L2_CTRL_CLASS_MPEG,
                                ctx->enc->num_ref);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR,
                   "Failed to set num reference frames!\n");
            ctx->in_error = true;
        }
    }

    /* Set number of B Frames. */
    if (ctx->enc->num_b_frames && nv_codec_type == NvVideoCodec_H264) {
        ret = nvv4l2_set_ext_controls(ctx->fd,
                                      V4L2_CID_MPEG_VIDEOENC_NUM_BFRAMES,
                                      V4L2_CTRL_CLASS_MPEG,
                                      ctx->enc->num_b_frames);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR,
                   "Failed to set number of B Frames!\n");
            ctx->in_error = true;
        }
    }

    /* Set encoder profile. */
    ret = nvv4l2_set_ext_controls(ctx->fd, nv_codec_type == NvVideoCodec_H264 ?
                                  V4L2_CID_MPEG_VIDEO_H264_PROFILE :
                                  V4L2_CID_MPEG_VIDEO_H265_PROFILE,
                                  V4L2_CTRL_CLASS_MPEG,
                                  ctx->enc->profile);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR,
               "Failed to set encoder profile!\n");
        ctx->in_error = true;
    }

    /* Set encoder level. */
    ret = nvv4l2_set_ext_controls(ctx->fd, nv_codec_type == NvVideoCodec_H264 ?
                                  V4L2_CID_MPEG_VIDEO_H264_LEVEL :
                                  V4L2_CID_MPEG_VIDEOENC_H265_LEVEL,
                                  V4L2_CTRL_CLASS_MPEG,
                                  ctx->enc->level);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR,
               "Failed to set encoder level!\n");
        ctx->in_error = true;
    }

    if (!ctx->enc->lossless) {
        /* Set encoder rate control mode. */
        ret = nvv4l2_set_ext_controls(ctx->fd, V4L2_CID_MPEG_VIDEO_BITRATE_MODE,
                                      V4L2_CTRL_CLASS_MPEG,
                                      ctx->enc->ratecontrol);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR,
                   "Failed to set encoder rate control mode!\n");
            ctx->in_error = true;
        }

        /* Set encoder max bitrate for VBR. */
        if (ctx->enc->ratecontrol == V4L2_MPEG_VIDEO_BITRATE_MODE_VBR) {

            uint32_t max_bitrate = 1.2f * ctx->enc->bitrate;
            ret = nvv4l2_set_ext_controls(ctx->fd,
                                          V4L2_CID_MPEG_VIDEO_BITRATE_PEAK,
                                          V4L2_CTRL_CLASS_MPEG,
                                          max_bitrate);
            if (ret) {
                av_log(avctx, AV_LOG_ERROR,
                       "Failed to set encoder max bitrate for VBR!\n");
                ctx->in_error = true;
            }
        }
    } else {
        /* Set constant qp configuration for lossless encoding enabled */
        ret = nvv4l2_set_ext_control_constant_qp(ctx->fd, 0);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR,
                   "Failed to set encoder qp to 0 for lossless encoding!\n");
            ctx->in_error = true;
        }
    }

    /* Set Two-pass CBR mode. */
    if (ctx->enc->twopass) {
        /* Set encoder IDR interval. */
        ret = nvv4l2_set_ext_controls(ctx->fd,
                                      V4L2_CID_MPEG_VIDEOENC_TWO_PASS_CBR,
                                      V4L2_CTRL_CLASS_MPEG,
                                      1);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR,
                   "Failed to set encoder 2-pass cbr!\n");
            ctx->in_error = true;
        }
    }

    /* Set encoder IDR interval. */
    ret = nvv4l2_set_ext_controls(ctx->fd, V4L2_CID_MPEG_VIDEO_IDR_INTERVAL,
                                  V4L2_CTRL_CLASS_MPEG,
                                  ctx->enc->idr_interval);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR,
               "Failed to set encoder IDR interval!\n");
        ctx->in_error = true;
    }

    /* Set encoder quantization parameters. */
    if (ctx->enc->qmin != -1 || ctx->enc->qmax != -1) {
        ret = nvv4l2_set_ext_control_qp_range(ctx->fd,
                                              ctx->enc->qmin, ctx->enc->qmax);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR,
                   "Failed to set encoder quantization parameters!\n");
            ctx->in_error = true;
        }
    }

    /* Set encoder I-Frame interval. */
    ret = nvv4l2_set_ext_controls(ctx->fd, V4L2_CID_MPEG_VIDEO_GOP_SIZE,
                                  V4L2_CTRL_CLASS_MPEG,
                                  ctx->enc->iframe_interval);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR,
               "Failed to set encoder I-Frame interval!\n");
        ctx->in_error = true;
    }

    /* Set insertSPSPPSAtIDR. */
    if (ctx->enc->sps_pps_at_idr) {
        ret = nvv4l2_set_ext_controls(ctx->fd,
                                V4L2_CID_MPEG_VIDEOENC_INSERT_SPS_PPS_AT_IDR,
                                V4L2_CTRL_CLASS_MPEG,
                                1);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR,
                   "Failed to set insertSPSPPSAtIDR!\n");
            ctx->in_error = true;
        }
    }

    /* Set encoder framerate. */
    ret = nvv4l2_set_stream_control_framerate(ctx->fd,
                                              ctx->op_mem_type,
                                              ctx->enc->fps_n,
                                              ctx->enc->fps_d);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR,
               "Failed to set framerate!\n");
        ctx->in_error = true;
    }

    /* Request max 10 buffers on output plane.
     ** Number of received buffers normally is lower (6). */
    ret = nvv4l2_req_buffers_on_output_plane(ctx,
                                             ctx->op_buf_type,
                                             ctx->op_mem_type,
                                             10);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR,
               "Error in requesting buffers on output plane!\n");
        ctx->in_error = true;
    }

    /* Create import DMA buffers. */
    for (uint32_t i = 0; i < ctx->op_num_buffers; i++) {
        NvBufferCreateParams iParams;
        memset(&iParams, 0, sizeof(NvBufferCreateParams));
        iParams.width = ctx->codec_width;
        iParams.height = ctx->codec_height;
        iParams.layout = NvBufferLayout_Pitch;
        iParams.payloadType = NvBufferPayload_SurfArray;
        iParams.nvbuf_tag = NvBufferTag_VIDEO_ENC;
        switch (ctx->op_pixfmt) {
        case V4L2_PIX_FMT_YUV444M:
            iParams.colorFormat = NvBufferColorFormat_YUV444;
            break;
        case V4L2_PIX_FMT_P010M:
            iParams.layout = NvBufferLayout_BlockLinear;
            iParams.colorFormat = NvBufferColorFormat_NV12_10LE;
            break;
        case V4L2_PIX_FMT_NV12M:
            iParams.colorFormat = NvBufferColorFormat_NV12;
            break;
        default:
            iParams.colorFormat = NvBufferColorFormat_YUV420;
            break;
        }

        if (ctx->enc->profile == V4L2_MPEG_VIDEO_H265_PROFILE_MAIN10) {
            iParams.layout = NvBufferLayout_BlockLinear;
            iParams.colorFormat = NvBufferColorFormat_NV12_10LE;
        }

        /* Increment color format if NvBuffer is newer. */
        if (ctx->pixfmt_list_ver == NvBufferPixFmtVersion_New &&
             iParams.colorFormat > NvBufferColorFormat_YUV420) {
            iParams.colorFormat++;
        }

        ret = NvBufferCreateEx(&ctx->plane_dma_fd[i], &iParams);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR, "Creation of dmabuf failed!\n");
            ctx->in_error = true;
        }
    }

    /* Request buffers on capture plane. */
    ret = nvv4l2_req_buffers_on_capture_plane(ctx,
                                         ctx->cp_buf_type,
                                         ctx->cp_mem_type,
                                         nvv4l2_ctx->num_capture_buffers);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR,
               "Error in requesting buffers on capture plane!\n");
        ctx->in_error = true;
    }

    /* Map buffers on capture plane */
    for (uint32_t i = 0; i < ctx->cp_num_buffers; i++) {
        ret = nvv4l2_query_buffer(ctx, ctx->cp_buf_type,
                                  ctx->cp_mem_type, ctx->cp_num_planes, i);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR,
                   "Failed to query buffer on capture plane!\n");
            ctx->in_error = true;
        }
        ret = nvv4l2_export_buffer(ctx, ctx->cp_buf_type,
                                   ctx->cp_num_planes, i);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR,
                   "Failed to export buffer on capture plane!\n");
            ctx->in_error = true;
        }
        ret = nvv4l2_map(ctx, ctx->cp_buffers[i]);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR,
                   "Failed to map buffer on capture plane!\n");
            ctx->in_error = true;
        }
    }

    /* Start stream processing on output plane. */
    ret = v4l2_ioctl(ctx->fd, VIDIOC_STREAMON, &ctx->op_buf_type);
    if (ret != 0) {
        av_log(avctx, AV_LOG_ERROR, "Streaming error on output plane!\n");
        ctx->in_error = true;
    }
    ctx->op_streamon = true;

     /* Set streaming status ON on capture plane. */
    ret = v4l2_ioctl(ctx->fd, VIDIOC_STREAMON, &ctx->cp_buf_type);
    if (ret != 0) {
        av_log(avctx, AV_LOG_ERROR, "Streaming error on capture plane!\n");
        ctx->in_error = true;
    }
    ctx->cp_streamon = true;

    /* Create and start capture loop thread. */
    pthread_create(&ctx->capture_thread, NULL, enc_capture_thread, ctx);

    /* Enqueue all the empty capture plane buffers. */
    for (uint32_t i = 0; i < ctx->cp_num_buffers; i++){
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[NV_MAX_PLANES];
        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, NV_MAX_PLANES * sizeof(struct v4l2_plane));

        v4l2_buf.index = i;
        v4l2_buf.m.planes = planes;

        ret = nvv4l2_q_buffer(ctx, &v4l2_buf, ctx->cp_buffers[i],
                              ctx->cp_buf_type, ctx->cp_mem_type,
                              ctx->cp_num_planes);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR, "Qing failed on capture plane!\n");
            ctx->in_error = true;
        }
    }

    return ctx;
}

int nvv4l2_encoder_put_frame(AVCodecContext *avctx, nvv4l2_ctx_t *ctx,
                             NvFrame *frame)
{
    int ret;
    int alignment;
    struct v4l2_buffer v4l2_buf_op;
    struct v4l2_plane queue_op_planes[NV_MAX_PLANES];
    NvBuffer *buffer;
    memset(&v4l2_buf_op, 0, sizeof(v4l2_buf_op));
    memset(queue_op_planes, 0, sizeof(queue_op_planes));
    v4l2_buf_op.m.planes = queue_op_planes;

    if (ctx->in_error)
        return -1;

    if (ctx->num_active_op_buffers < ctx->op_num_buffers) {
        /* Get an unused buffer to add to the queue. */
        buffer = ctx->op_buffers[ctx->num_active_op_buffers];
        v4l2_buf_op.index = ctx->num_active_op_buffers;

        /* Map new plane buffer for memory type DMABUF. */
        v4l2_buf_op.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        v4l2_buf_op.memory = ctx->op_mem_type;
        ret = nvv4l2_map_out(ctx, &v4l2_buf_op, ctx->op_buf_type,
                             ctx->op_mem_type,
                             ctx->plane_dma_fd[v4l2_buf_op.index]);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR,
                   "Error while mapping buffer at output plane!\n");
            ctx->in_error = true;
            return -1;
        }
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

    /*
     ** Due to NvMap/VIC stride conversion constrains, the transformation
     ** must be aligned per format/plane. Otherwise the frame might be
     ** produced as scrambled.
     **
     ** !TODO: Have a proper algorithm to calculate needed alignements.
     */
    switch (ctx->op_pixfmt) {
    case V4L2_PIX_FMT_NV12M:
        alignment = 16;
        break;
    case V4L2_PIX_FMT_YUV420M:
        alignment = 64;
        break;
    case V4L2_PIX_FMT_YUV444M:
        alignment = 32;
        break;
    case V4L2_PIX_FMT_P010M:
    default:
        alignment = 1;
        break;
    }

    /* Import frame into output plane */
    for (uint32_t i = 0; i < buffer->n_planes; i++) {
        int aligned_plane_width = NVALIGN(ctx->op_planefmts[i].width, alignment);

        /* If plane is reduced, use alignment of main plane */
        if (ctx->op_planefmts[i].width == ctx->op_planefmts[0].width / 2)
            aligned_plane_width = NVALIGN(ctx->op_planefmts[0].width, alignment) / 2;

        av_log(avctx, AV_LOG_VERBOSE, "Plane %d: width %d -> %d\n",
               i, ctx->op_planefmts[i].width, aligned_plane_width);

        Raw2NvBuffer(frame->payload[i], i, aligned_plane_width,
                     ctx->op_planefmts[i].height, buffer->planes[i].fd);
        buffer->planes[i].bytesused = ctx->op_planefmts[i].width *
                                      ctx->op_planefmts[i].height *
                                      ctx->op_planefmts[i].bytesperpixel;
        v4l2_buf_op.m.planes[i].bytesused = buffer->planes[i].bytesused;
    }

    /* Set timestamp */
    v4l2_buf_op.flags |= V4L2_BUF_FLAG_TIMESTAMP_COPY;
    v4l2_buf_op.timestamp.tv_usec = frame->pts % AV_TIME_BASE;
    v4l2_buf_op.timestamp.tv_sec = frame->pts / AV_TIME_BASE;

    /* Queue frame on output plane. */
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

    return 0;
}

int nvv4l2_encoder_get_packet(AVCodecContext *avctx,
                              nvv4l2_ctx_t *ctx,
                              NvPacket *packet)
{
    int packet_index;

    if (ctx->export_pool->capacity == 0)
        return 1;

    packet_index = nvv4l2_pool_pop(ctx, ctx->export_pool);

    packet->payload = ctx->packet[packet_index];
    packet->payload_size = ctx->packet_size[packet_index];
    packet->pts = ctx->frame_pts[packet_index];

    if (ctx->packet_keyflag[packet_index])
        packet->flags |= AV_PKT_FLAG_KEY;

    return 0;
}

int nvv4l2_encoder_close(AVCodecContext *avctx, nvv4l2_ctx_t *ctx)
{
    int ret, op_num_old_buffers;

    if (!ctx)
        return 0;

    pthread_mutex_lock(&ctx->queue_lock);
    ctx->eos = true;
    pthread_mutex_unlock(&ctx->queue_lock);
    if (ctx->fd != -1) {
        /* Stop streaming on both planes. */
        ret = v4l2_ioctl(ctx->fd, VIDIOC_STREAMOFF, &ctx->op_buf_type);
        ret = v4l2_ioctl(ctx->fd, VIDIOC_STREAMOFF, &ctx->cp_buf_type);

        /* Wait for capture thread to exit. */
        if (ctx->capture_thread) {
            pthread_join(ctx->capture_thread, NULL);
        }

        /* Unmap MMAPed buffers. */
        for (uint32_t i = 0; i < ctx->cp_num_buffers; ++i) {
            nvv4l2_destroyBuffer(ctx, ctx->cp_buffers[i]);
        }

        for (uint32_t i = 0; i < ctx->op_num_buffers; i++) {
            if (ctx->plane_dma_fd[i] != -1) {
                nvv4l2_unmap_out(ctx, i, ctx->op_buf_type,
                                 ctx->op_mem_type, ctx->plane_dma_fd[i]);
            }
        }

        /* Request 0 buffers on both planes. */
        op_num_old_buffers = ctx->op_num_buffers;
        ret = nvv4l2_req_buffers_on_output_plane(ctx,
                                                 ctx->op_buf_type,
                                                 ctx->op_mem_type, 0);

        ret = nvv4l2_req_buffers_on_capture_plane(ctx,
                                                  ctx->cp_buf_type,
                                                  ctx->cp_mem_type, 0);

        /* Unmap and destroy all allocated DMA buffers. */
        for (uint32_t i = 0; i < op_num_old_buffers; i++) {
            if (ctx->plane_dma_fd[i] != -1) {
                ret = NvBufferDestroy(ctx->plane_dma_fd[i]);
                ctx->plane_dma_fd[i] = -1;
                if (ret) {
                    av_log(avctx, AV_LOG_ERROR,
                           "Failed to destroy output plane dma buffer!\n");
                }
            }
        }

        /* Free packet pool */
        for (int index = 0; index < NV_MAX_BUFFERS; index++) {
            NVFREE(ctx->packet[index]);
        }
        NVFREE(ctx->export_pool);

        /* Close the opened V4L2 device. */
        ret = v4l2_close(ctx->fd);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR, "Unable to close the device!\n");
        }

        /* Free mutexes */
        pthread_mutex_destroy(&ctx->queue_lock);
        pthread_cond_destroy(&ctx->queue_cond);
    }

    /* Free encoder parameters */
    NVFREE(ctx->enc);

    /* Report application run status on exit. */
    if (ctx->in_error) {
        av_log(avctx, AV_LOG_ERROR, "Encoder Run failed\n");
    } else {
        av_log(avctx, AV_LOG_VERBOSE, "Encoder Run is successful\n");
    }

    NVFREE(ctx);

    return ret;
}

static void
nvv4l2_set_h264_profile_params(nvv4l2EncodeContext *nvv4l2_ctx,
                               NvEncoder *enc,
                               int *pix_fmt)
{
    switch (nvv4l2_ctx->profile & ~FF_PROFILE_H264_INTRA) {
    case FF_PROFILE_H264_MAIN:
        enc->profile = V4L2_MPEG_VIDEO_H264_PROFILE_MAIN;
        break;
    case FF_PROFILE_H264_BASELINE:
        enc->profile = V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE;
        break;
    case FF_PROFILE_H264_HIGH:
        enc->profile = V4L2_MPEG_VIDEO_H264_PROFILE_HIGH;
        break;
    case FF_PROFILE_H264_HIGH_444_PREDICTIVE:
        enc->profile = V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_PREDICTIVE;
        break;

    default:
        enc->profile = V4L2_MPEG_VIDEO_H264_PROFILE_MAIN;
        break;
    }

    if (enc->lossless && *pix_fmt == V4L2_PIX_FMT_YUV444M)
        enc->profile = V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_PREDICTIVE;

    switch (nvv4l2_ctx->level) {
    case 9:
        enc->level = V4L2_MPEG_VIDEO_H264_LEVEL_1B;
        break;
    case 10:
        enc->level = V4L2_MPEG_VIDEO_H264_LEVEL_1_0;
        break;
    case 11:
        if (nvv4l2_ctx->profile & FF_PROFILE_H264_INTRA)
            enc->level = V4L2_MPEG_VIDEO_H264_LEVEL_1B;
        else
            enc->level = V4L2_MPEG_VIDEO_H264_LEVEL_1_1;
        break;
    case 12:
        enc->level = V4L2_MPEG_VIDEO_H264_LEVEL_1_2;
        break;
    case 13:
        enc->level = V4L2_MPEG_VIDEO_H264_LEVEL_1_3;
        break;
    case 20:
        enc->level = V4L2_MPEG_VIDEO_H264_LEVEL_2_0;
        break;
    case 21:
        enc->level = V4L2_MPEG_VIDEO_H264_LEVEL_2_1;
        break;
    case 22:
        enc->level = V4L2_MPEG_VIDEO_H264_LEVEL_2_2;
        break;
    case 30:
        enc->level = V4L2_MPEG_VIDEO_H264_LEVEL_3_0;
        break;
    case 31:
        enc->level = V4L2_MPEG_VIDEO_H264_LEVEL_3_1;
        break;
    case 32:
        enc->level = V4L2_MPEG_VIDEO_H264_LEVEL_3_2;
        break;
    case 40:
        enc->level = V4L2_MPEG_VIDEO_H264_LEVEL_4_0;
        break;
    case 41:
        enc->level = V4L2_MPEG_VIDEO_H264_LEVEL_4_1;
        break;
    case 42:
        enc->level = V4L2_MPEG_VIDEO_H264_LEVEL_4_2;
        break;
    case 50:
        enc->level = V4L2_MPEG_VIDEO_H264_LEVEL_5_0;
        break;
    case 51:
        enc->level = V4L2_MPEG_VIDEO_H264_LEVEL_5_1;
        break;
    default:
        enc->level = V4L2_MPEG_VIDEO_H264_LEVEL_5_1;
        break;
    }
}

static void
nvv4l2_set_hevc_profile_params(nvv4l2EncodeContext *nvv4l2_ctx,
                               NvEncoder *enc,
                               int *pix_fmt)
{
    switch (nvv4l2_ctx->profile & ~FF_PROFILE_H264_INTRA) {
    case FF_PROFILE_HEVC_MAIN:
        enc->profile = V4L2_MPEG_VIDEO_H265_PROFILE_MAIN;
        break;
    case FF_PROFILE_HEVC_MAIN_10:
        enc->profile = V4L2_MPEG_VIDEO_H265_PROFILE_MAIN10;
        *pix_fmt = V4L2_PIX_FMT_P010M;
        break;

    default:
        enc->profile = V4L2_MPEG_VIDEO_H265_PROFILE_MAIN;
        break;
    }

    if (*pix_fmt == V4L2_PIX_FMT_P010M)
        enc->profile = V4L2_MPEG_VIDEO_H265_PROFILE_MAIN10;

    switch (nvv4l2_ctx->tier) {
    case 0:
    case 1:
        enc->tier = nvv4l2_ctx->tier;
        break;

    default:
        enc->tier = 0;
        break;
    }

    switch (nvv4l2_ctx->level) {
    case 30:
        enc->level = V4L2_MPEG_VIDEO_H265_LEVEL_1_0_MAIN_TIER;
        break;
    case 60:
        enc->level = V4L2_MPEG_VIDEO_H265_LEVEL_2_0_MAIN_TIER;
        break;
    case 63:
        enc->level = V4L2_MPEG_VIDEO_H265_LEVEL_2_1_MAIN_TIER;
        break;
    case 90:
        enc->level = V4L2_MPEG_VIDEO_H265_LEVEL_3_0_MAIN_TIER;
        break;
    case 93:
        enc->level = V4L2_MPEG_VIDEO_H265_LEVEL_3_1_MAIN_TIER;
        break;
    case 120:
        enc->level = V4L2_MPEG_VIDEO_H265_LEVEL_4_0_MAIN_TIER;
        break;
    case 123:
        enc->level = V4L2_MPEG_VIDEO_H265_LEVEL_4_1_MAIN_TIER;
        break;
    case 150:
        enc->level = V4L2_MPEG_VIDEO_H265_LEVEL_5_0_MAIN_TIER;
        break;
    case 153:
        enc->level = V4L2_MPEG_VIDEO_H265_LEVEL_5_1_MAIN_TIER;
        break;
    case 156:
        enc->level = V4L2_MPEG_VIDEO_H265_LEVEL_5_2_MAIN_TIER;
        break;
    case 180:
        enc->level = V4L2_MPEG_VIDEO_H265_LEVEL_6_0_MAIN_TIER;
        break;
    case 183:
        enc->level = V4L2_MPEG_VIDEO_H265_LEVEL_6_1_MAIN_TIER;
        break;
    case 186:
        enc->level = V4L2_MPEG_VIDEO_H265_LEVEL_6_2_MAIN_TIER;
        break;
    default:
        enc->level = V4L2_MPEG_VIDEO_H265_LEVEL_6_2_MAIN_TIER;
        break;
    }

    enc->level += enc->tier;
}

static NvEncoder *set_encoder_parameters(AVCodecContext *avctx,
                                         nvv4l2EncodeContext *nvv4l2_ctx,
                                         NvCodingType nv_codec_type,
                                         int *pix_fmt)
{
    NvEncoder *enc = (NvEncoder *)NVCALLOC(1, sizeof(NvEncoder));

    enc->lossless = nvv4l2_ctx->lossless;
    enc->ratecontrol = nvv4l2_ctx->rc == 1 ?
                            V4L2_MPEG_VIDEO_BITRATE_MODE_VBR :
                            V4L2_MPEG_VIDEO_BITRATE_MODE_CBR;
    if (nvv4l2_ctx->twopass) {
        enc->twopass = 1;
        enc->ratecontrol = V4L2_MPEG_VIDEO_BITRATE_MODE_CBR;
    }

    enc->width = avctx->width;
    enc->height = avctx->height;
    enc->bitrate = avctx->bit_rate;

    if (nv_codec_type == NvVideoCodec_H264) {
        nvv4l2_set_h264_profile_params(nvv4l2_ctx, enc, pix_fmt);
    } else if (nv_codec_type == NvVideoCodec_HEVC) {
        nvv4l2_set_hevc_profile_params(nvv4l2_ctx, enc, pix_fmt);
    }

    switch (nvv4l2_ctx->preset) {
    case 1:
        enc->preset_type = V4L2_ENC_HW_PRESET_ULTRAFAST;
        break;
    case 2:
        enc->preset_type = V4L2_ENC_HW_PRESET_FAST;
        break;
    case 3:
        enc->preset_type = V4L2_ENC_HW_PRESET_MEDIUM;
        break;
    case 4:
        enc->preset_type = V4L2_ENC_HW_PRESET_SLOW;
        break;
    default:
        enc->preset_type = V4L2_ENC_HW_PRESET_MEDIUM;
        break;
    }

    if (avctx->gop_size > 0) {
        enc->idr_interval = avctx->gop_size;
        enc->iframe_interval = avctx->gop_size;
    } else {
        enc->idr_interval = 60;
        enc->iframe_interval = 30;
    }
    enc->fps_n = avctx->framerate.num;
    enc->fps_d = avctx->framerate.den;

    if (avctx->qmin >= 0 && avctx->qmax >= 0) {
        enc->qmin = avctx->qmin;
        enc->qmax = avctx->qmax;
    } else {
        enc->qmin = -1;
        enc->qmax = -1;
    }

    if (avctx->max_b_frames >= 0 && avctx->max_b_frames < 3)
        enc->num_b_frames = avctx->max_b_frames;

    if (avctx->refs > 0)
        enc->num_ref = avctx->refs;

    enc->sps_pps_at_idr = !(avctx->flags & AV_CODEC_FLAG_GLOBAL_HEADER);
    enc->low_latency = (avctx->flags & AV_CODEC_FLAG_LOW_DELAY) ? true : false;

    return enc;
}

static NvCodingType map_avcodec_id(enum AVCodecID id)
{
    switch (id) {
    case AV_CODEC_ID_H264:
        return NvVideoCodec_H264;
    case AV_CODEC_ID_HEVC:
        return NvVideoCodec_HEVC;
    }
    return NvVideoCodec_UNDEFINED;
}

static int nvv4l2enc_init(AVCodecContext *avctx)
{
    nvv4l2EncodeContext *nvv4l2_ctx = avctx->priv_data;
    NvCodingType nv_codec_type;
    NvEncoder *encoder;
    int pix_fmt;

    nv_codec_type = map_avcodec_id(avctx->codec_id);
    if (nv_codec_type == NvVideoCodec_UNDEFINED) {
        av_log(avctx, AV_LOG_ERROR, "Unsupported codec ID %d!\n",
               avctx->codec_id);
        return AVERROR_BUG;
    }

    /* Set output plane pixel format. */
    switch (avctx->pix_fmt) {
    case AV_PIX_FMT_YUV444P:
        pix_fmt = V4L2_PIX_FMT_YUV444M;
        break;
    case AV_PIX_FMT_NV12:
        pix_fmt = V4L2_PIX_FMT_NV12M;
        break;
    case AV_PIX_FMT_YUV420P10:
        avctx->pix_fmt = AV_PIX_FMT_P010;
    case AV_PIX_FMT_P010:
        pix_fmt = V4L2_PIX_FMT_P010M;
        break;
    case AV_PIX_FMT_NONE:
        avctx->pix_fmt = AV_PIX_FMT_YUV420P;
    case AV_PIX_FMT_YUV420P:
        pix_fmt = V4L2_PIX_FMT_YUV420M;
        break;
    default:
        av_log(avctx, AV_LOG_ERROR, "Unsupported pixel format %d!\n",
               avctx->pix_fmt);
        return AVERROR_BUG;
    }

    /* Set encoder parameters. */
    encoder = set_encoder_parameters(avctx, nvv4l2_ctx, nv_codec_type,
                                     &pix_fmt);

    /* Check if global SPS/PPS header is required and sample it. */
    if (nv_codec_type == NvVideoCodec_H264 &&
        (avctx->flags & AV_CODEC_FLAG_GLOBAL_HEADER)) {
        NvFrame _nvframe = {0};
        NvPacket packet = {0};
        uint8_t *dst[4];
        int linesize[4];
        int header_size = 0;
        int ret = 0;

        nvv4l2_ctx->ctx = nvv4l2_create_encoder(avctx, encoder,
                                                NvVideoCodec_H264,
                                                pix_fmt);
        if (!nvv4l2_ctx->ctx || nvv4l2_ctx->ctx->in_error) {
            ret = 1;
            goto out;
        }

        /* Get a blank packet to extract metadata */
        av_image_alloc(dst, linesize, avctx->width, avctx->height,
                       avctx->pix_fmt, 1);

        while (true) {
            _nvframe.payload[0] = dst[0];
            _nvframe.payload[1] = dst[1];
            _nvframe.payload[2] = dst[2];

            ret = nvv4l2_encoder_put_frame(avctx, nvv4l2_ctx->ctx, &_nvframe);
            if (ret)
                goto out;

            /* Try several times to get a packet before queuing a new one. */
            for (uint32_t i = 0; i < 100; i++) {
                ret = nvv4l2_encoder_get_packet(avctx, nvv4l2_ctx->ctx,
                                                &packet);
                if (!ret)
                    break;
                usleep(1000);
            }
            if (ret)
                continue;

            /* Find H264_NAL_IDR_SLICE */
            for (header_size = 0;
                 (header_size + 4) < packet.payload_size;
                 header_size++) {
                if (packet.payload[header_size]     == 0 &&
                    packet.payload[header_size + 1] == 0 &&
                    packet.payload[header_size + 2] == 0 &&
                    packet.payload[header_size + 3] == 1 &&
                    packet.payload[header_size + 4] == 0x65) {
                    break;
                }
            }

            if (header_size >= packet.payload_size) {
                av_log(avctx, AV_LOG_ERROR, "Header was not found!\n");
                return AVERROR_BUG;
            }

            avctx->extradata_size = header_size;
            avctx->extradata = av_mallocz(header_size +
                                          AV_INPUT_BUFFER_PADDING_SIZE);
            memcpy(avctx->extradata, packet.payload, header_size);

            break;
        }
        av_free(dst[0]);

out:
        nvv4l2_encoder_close(avctx, nvv4l2_ctx->ctx);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR, "Error in initializing!\n");
            return AVERROR_BUG;
        }

        /* Set encoder parameters again */
        encoder = set_encoder_parameters(avctx, nvv4l2_ctx, nv_codec_type,
                                         &pix_fmt);
    }

    nvv4l2_ctx->ctx = nvv4l2_create_encoder(avctx, encoder, nv_codec_type,
                                            pix_fmt);

    if (!nvv4l2_ctx->ctx || nvv4l2_ctx->ctx->in_error) {
        nvv4l2_encoder_close(avctx, nvv4l2_ctx->ctx);
        return AVERROR_BUG;
    } else
        return 0;
}

static int
nvv4l2enc_encode(AVCodecContext *avctx, AVPacket *pkt,
                 const AVFrame *frame, int *got_packet)
{
    nvv4l2EncodeContext *nvv4l2_ctx = avctx->priv_data;
    nvv4l2_ctx_t *ctx = nvv4l2_ctx->ctx;
    NvFrame _nvframe = {0};
    NvPacket packet = {0};

    if (ctx->in_error) {
        return AVERROR_UNKNOWN;
    }

    if (frame) {
        _nvframe.payload[0] = frame->data[0];
        _nvframe.payload[1] = frame->data[1];
        _nvframe.payload[2] = frame->data[2];

        _nvframe.pts = frame->pts;

        if (nvv4l2_encoder_put_frame(avctx, ctx, &_nvframe))
            return AVERROR_UNKNOWN;
    }

    if (nvv4l2_encoder_get_packet(avctx, ctx, &packet))
        return 0;

    ff_alloc_packet2(avctx, pkt, packet.payload_size, packet.payload_size);

    memcpy(pkt->data, packet.payload, packet.payload_size);
    pkt->dts = pkt->pts = packet.pts;

    if (packet.flags & AV_PKT_FLAG_KEY)
        pkt->flags = AV_PKT_FLAG_KEY;

    *got_packet = 1;

    return 0;
}

static av_cold int nvv4l2enc_close(AVCodecContext *avctx)
{
    nvv4l2EncodeContext *nvv4l2_ctx = avctx->priv_data;
    nvv4l2_encoder_close(avctx, nvv4l2_ctx->ctx);

    return 0;
}

static const AVCodecDefault defaults[] = {
    { "b",     "5M" },
    { "qmin",  "-1" },
    { "qmax",  "-1" },
    { "qdiff", "-1" },
    { "qblur", "-1" },
    { "qcomp", "-1" },
    { "g",     "50" },
    { "bf",    "0" },
    { "refs",  "0" },
    { NULL },
};

#define OFFSET(x) offsetof(nvv4l2EncodeContext, x)
#define VE AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_ENCODING_PARAM

static const AVOption options_h264[] = {
    { "num_capture_buffers", "Number of buffers in the capture context",
        OFFSET(num_capture_buffers), AV_OPT_TYPE_INT, {.i64 = 10 }, 1, 32, VE },

    { "profile",  "Set the encoding profile", OFFSET(profile), AV_OPT_TYPE_INT,
        { .i64 = FF_PROFILE_H264_MAIN }, FF_PROFILE_H264_BASELINE,
        FF_PROFILE_H264_HIGH_444_PREDICTIVE, VE, "profile" },
#define PROFILE(name, value)  name, NULL, 0, AV_OPT_TYPE_CONST, \
                              { .i64 = value }, 0, 0, VE, "profile"
    { PROFILE("baseline", FF_PROFILE_H264_BASELINE) },
    { PROFILE("main",     FF_PROFILE_H264_MAIN) },
    { PROFILE("high",     FF_PROFILE_H264_HIGH) },
    { PROFILE("high444",  FF_PROFILE_H264_HIGH_444_PREDICTIVE) },
#undef PROFILE

    { "level", "Profile Level", OFFSET(level), AV_OPT_TYPE_INT,
        { .i64 = 51 }, 9, 51, VE, "level" },
#define LEVEL(name, value) name, NULL, 0, AV_OPT_TYPE_CONST, \
                           { .i64 = value }, 0, 0, VE, "level"
    { LEVEL("1.0", 10) },
    { LEVEL("1b",  9 ) },
    { LEVEL("1.1", 11) },
    { LEVEL("1.2", 12) },
    { LEVEL("1.3", 13) },
    { LEVEL("2.0", 20) },
    { LEVEL("2.1", 21) },
    { LEVEL("2.2", 22) },
    { LEVEL("3.0", 30) },
    { LEVEL("3.1", 31) },
    { LEVEL("3.2", 32) },
    { LEVEL("4.0", 40) },
    { LEVEL("4.1", 41) },
    { LEVEL("4.2", 42) },
    { LEVEL("5.0", 50) },
    { LEVEL("5.1", 51) },
#undef LEVEL

    { "lossless", "Enable lossless encoding", OFFSET(lossless), AV_OPT_TYPE_INT,
        { .i64 = 0 }, 0, 1, VE, "lossless"},
#define LOSSLESS(name, value)  name, NULL, 0, AV_OPT_TYPE_CONST, \
                           { .i64 = value }, 0, 0, VE, "lossless"
    { LOSSLESS("off", 0) },
    { LOSSLESS("on",  1) },
#undef LOSSLESS

    { "rc",  "Override the preset rate-control",
        OFFSET(rc), AV_OPT_TYPE_INT,   { .i64 = 1 }, 0, 1, VE, "rc" },
    { "cbr", "Constant bitrate mode", 0, AV_OPT_TYPE_CONST,
        { .i64 = 0  },  0, 0, VE, "rc" },
    { "vbr", "Variable bitrate mode", 0, AV_OPT_TYPE_CONST,
        { .i64 = 1  },  0, 0, VE, "rc" },

    { "preset",    "Set the encoding preset", OFFSET(preset),
        AV_OPT_TYPE_INT,   { .i64 = 3 }, 1, 4, VE, "preset" },
    { "default",   "", 0, AV_OPT_TYPE_CONST, { .i64 = 3 }, 0, 0, VE, "preset" },
    { "slow",      "", 0, AV_OPT_TYPE_CONST, { .i64 = 4 }, 0, 0, VE, "preset" },
    { "medium",    "", 0, AV_OPT_TYPE_CONST, { .i64 = 3 }, 0, 0, VE, "preset" },
    { "fast",      "", 0, AV_OPT_TYPE_CONST, { .i64 = 2 }, 0, 0, VE, "preset" },
    { "ultrafast", "", 0, AV_OPT_TYPE_CONST, { .i64 = 1 }, 0, 0, VE, "preset" },

    { "2pass", "Enable Two-Pass CBR. (Forces CBR).",
        OFFSET(twopass), AV_OPT_TYPE_INT, {.i64 = 0 }, 0, 1, VE },
#define TWOPASS(name, value)  name, NULL, 0, AV_OPT_TYPE_CONST, \
                           { .i64 = value }, 0, 0, VE, "twopass"
    { TWOPASS("off", 0) },
    { TWOPASS("on",  1) },
#undef TWOPASS
    { NULL }
};

static const AVOption options_hevc[] = {
    { "num_capture_buffers", "Number of buffers in the capture context",
        OFFSET(num_capture_buffers), AV_OPT_TYPE_INT, {.i64 = 10 }, 1, 32, VE },

    { "profile",  "Set the encoding profile", OFFSET(profile), AV_OPT_TYPE_INT,
        { .i64 = FF_PROFILE_HEVC_MAIN }, FF_PROFILE_HEVC_MAIN,
        FF_PROFILE_HEVC_MAIN_10, VE, "profile" },
#define PROFILE(name, value)  name, NULL, 0, AV_OPT_TYPE_CONST, \
                              { .i64 = value }, 0, 0, VE, "profile"
    { PROFILE("main",   FF_PROFILE_HEVC_MAIN) },
    { PROFILE("main10", FF_PROFILE_HEVC_MAIN_10) },
#undef PROFILE

    { "tier", "Set the encoding tier", OFFSET(tier), AV_OPT_TYPE_INT,
        { .i64 = 0 }, 0, 1, VE, "tier"},
#define TIER(name, value)  name, NULL, 0, AV_OPT_TYPE_CONST, \
                           { .i64 = value }, 0, 0, VE, "tier"
    { TIER("main", 0) },
    { TIER("high", 1) },
#undef TIER

    { "level", "Profile Level", OFFSET(level), AV_OPT_TYPE_INT,
        { .i64 = 186 }, 30, 186, VE, "level" },
#define LEVEL(name, value) name, NULL, 0, AV_OPT_TYPE_CONST, \
                           { .i64 = value }, 0, 0, VE, "level"
    { LEVEL("1",    30) },
    { LEVEL("2",    60) },
    { LEVEL("2.1",  63) },
    { LEVEL("3",    90) },
    { LEVEL("3.1",  93) },
    { LEVEL("4",   120) },
    { LEVEL("4.1", 123) },
    { LEVEL("5",   150) },
    { LEVEL("5.1", 153) },
    { LEVEL("5.2", 156) },
    { LEVEL("6",   180) },
    { LEVEL("6.1", 183) },
    { LEVEL("6.2", 186) },
#undef LEVEL

    { "lossless", "Enable lossless encoding", OFFSET(lossless), AV_OPT_TYPE_INT,
        { .i64 = 0 }, 0, 1, VE, "lossless"},
#define LOSSLESS(name, value)  name, NULL, 0, AV_OPT_TYPE_CONST, \
                           { .i64 = value }, 0, 0, VE, "lossless"
    { LOSSLESS("off", 0) },
    { LOSSLESS("on",  1) },
#undef LOSSLESS

    { "rc",  "Override the preset rate-control", OFFSET(rc),
        AV_OPT_TYPE_INT,   { .i64 = 1 }, 0, 1, VE, "rc" },
    { "cbr", "Constant bitrate mode", 0, AV_OPT_TYPE_CONST,
        { .i64 = 0  },  0, 0, VE, "rc" },
    { "vbr", "Variable bitrate mode", 0, AV_OPT_TYPE_CONST,
        { .i64 = 1  },  0, 0, VE, "rc" },

    { "preset",    "Set the encoding preset", OFFSET(preset),
        AV_OPT_TYPE_INT,   { .i64 = 3 }, 3, 4, VE, "preset" },
    { "default",   "", 0, AV_OPT_TYPE_CONST, { .i64 = 3 }, 0, 0, VE, "preset" },
    { "slow",      "", 0, AV_OPT_TYPE_CONST, { .i64 = 4 }, 0, 0, VE, "preset" },
    { "medium",    "", 0, AV_OPT_TYPE_CONST, { .i64 = 3 }, 0, 0, VE, "preset" },
    { "fast",      "", 0, AV_OPT_TYPE_CONST, { .i64 = 2 }, 0, 0, VE, "preset" },
    { "ultrafast", "", 0, AV_OPT_TYPE_CONST, { .i64 = 1 }, 0, 0, VE, "preset" },

    { "2pass", "Enable Two-Pass CBR. (Forces CBR).",
        OFFSET(twopass), AV_OPT_TYPE_INT, {.i64 = 0 }, 0, 1, VE },
#define TWOPASS(name, value)  name, NULL, 0, AV_OPT_TYPE_CONST, \
                           { .i64 = value }, 0, 0, VE, "twopass"
    { TWOPASS("off", 0) },
    { TWOPASS("on",  1) },
#undef TWOPASS
    { NULL }
};

#define NVV4L2_ENC_CLASS(NAME)                         \
    static const AVClass nvv4l2_##NAME##_enc_class = { \
        .class_name = "nvv4l2_" #NAME "_enc",          \
        .item_name  = av_default_item_name,            \
        .option     = options_##NAME,                  \
        .version    = LIBAVUTIL_VERSION_INT,           \
    };

#define NVV4L2_ENC(NAME, ID)                                                          \
    NVV4L2_ENC_CLASS(NAME)                                                            \
    AVCodec ff_##NAME##_nvv4l2_encoder = {                                            \
        .name           = #NAME "_nvv4l2" ,                                           \
        .long_name      = NULL_IF_CONFIG_SMALL(#NAME " NVV4L2 HW encoder for Tegra"), \
        .type           = AVMEDIA_TYPE_VIDEO,                                         \
        .id             = ID,                                                         \
        .priv_data_size = sizeof(nvv4l2EncodeContext),                                \
        .init           = nvv4l2enc_init,                                             \
        .close          = nvv4l2enc_close,                                            \
        .encode2        = nvv4l2enc_encode,                                           \
        .priv_class     = &nvv4l2_##NAME##_enc_class,                                 \
        .capabilities   = AV_CODEC_CAP_DELAY | AV_CODEC_CAP_HARDWARE,                 \
        .defaults       = defaults,                                                   \
        .wrapper_name   = "nvv4l2",                                                   \
        .pix_fmts       = (const enum AVPixelFormat[]) { AV_PIX_FMT_YUV420P,          \
                                                         AV_PIX_FMT_YUV444P,          \
                                                         AV_PIX_FMT_NV12,             \
                                                         AV_PIX_FMT_P010,             \
                                                         AV_PIX_FMT_YUV420P10,        \
                                                         AV_PIX_FMT_NONE },           \
    };

NVV4L2_ENC(h264, AV_CODEC_ID_H264);
NVV4L2_ENC(hevc, AV_CODEC_ID_HEVC);
