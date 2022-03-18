/*
 * Copyright (c) 2021-2022, CTCaer <ctcaer@gmail.com>
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
#include <sys/mman.h>
#include <errno.h>
#include "internal.h"
#include "libavutil/log.h"

#include "nvv4l2.h"

uint32_t nvv4l2_map_nvcodec_type(NvCodingType nv_codec_type)
{
    uint32_t v4l2_pix_fmt;
    switch (nv_codec_type) {
    case NvVideoCodec_H264:
        v4l2_pix_fmt = V4L2_PIX_FMT_H264;
        break;
    case NvVideoCodec_HEVC:
        v4l2_pix_fmt = V4L2_PIX_FMT_H265;
        break;
    case NvVideoCodec_MPEG2:
        v4l2_pix_fmt = V4L2_PIX_FMT_MPEG2;
        break;
    case NvVideoCodec_MPEG4:
        v4l2_pix_fmt = V4L2_PIX_FMT_MPEG4;
        break;
    case NvVideoCodec_VP8:
        v4l2_pix_fmt = V4L2_PIX_FMT_VP8;
        break;
    case NvVideoCodec_VP9:
        v4l2_pix_fmt = V4L2_PIX_FMT_VP9;
        break;
    default:
        v4l2_pix_fmt = V4L2_PIX_FMT_H264;
        break;
    }

    return v4l2_pix_fmt;
}

int nvv4l2_pool_idx_next(nvv4l2_ctx_t *ctx, NvQueues *q)
{
    int index;
    if (q->capacity < NV_MAX_BUFFERS) {
        index = q->back;
    } else {
        index = -1;
    }
    return index;
}

void nvv4l2_pool_push(nvv4l2_ctx_t *ctx, NvQueues *q)
{
    if (q->capacity < NV_MAX_BUFFERS) {
        q->back = (q->back + 1) % NV_MAX_BUFFERS;
        atomic_fetch_add(&q->capacity, 1);
    } else {
        av_log(ctx->avctx, AV_LOG_ERROR, "Queue already full!\n");
    }
}

int nvv4l2_pool_pop(nvv4l2_ctx_t *ctx, NvQueues *q)
{
    int index = q->front;
    if (q->capacity != 0) {
        q->front = (q->front + 1) % NV_MAX_BUFFERS;
        atomic_fetch_sub(&q->capacity, 1);
    } else {
        av_log(ctx->avctx, AV_LOG_ERROR, "Queue already empty!");
    }
    return index;
}

int
nvv4l2_create_bufferfmt(NvBuffer *buffer, enum v4l2_buf_type buf_type,
                     enum v4l2_memory memory_type, uint32_t n_planes,
                     NvBufferPlaneFormat *fmt, uint32_t index)
{
    buffer->mapped = false;
    buffer->buf_type = buf_type;
    buffer->memory_type = memory_type;
    buffer->index = index;
    buffer->n_planes = n_planes;

    memset(buffer->planes, 0, sizeof(NvBufferPlane));
    for (uint32_t i = 0; i < buffer->n_planes; i++) {
        buffer->planes[i].fd = -1;
        buffer->planes[i].fmt = fmt[i];
    }
    return 0;
}

int
nvv4l2_map_out(nvv4l2_ctx_t *ctx, struct v4l2_buffer *v4l2_buf,
               enum v4l2_buf_type buf_type, enum v4l2_memory mem_type,
               int dma_fd)
{
    int ret;
    NvBuffer *buffer;
    NvBufferParams params;
    unsigned char *data;
    pthread_mutex_lock(&ctx->queue_lock);

    if (buf_type == ctx->op_buf_type)
        buffer = ctx->op_buffers[v4l2_buf->index];
    else if (buf_type == ctx->cp_buf_type)
        buffer = ctx->cp_buffers[v4l2_buf->index];

    switch (mem_type) {
    case V4L2_MEMORY_DMABUF:
        ret = NvBufferGetParams(dma_fd, &params);
        if(ret) {
            av_log(ctx->avctx, AV_LOG_ERROR, "GetParams failed!\n");
            pthread_mutex_unlock(&ctx->queue_lock);
            return ret;
        }
        for (uint32_t i = 0; i < buffer->n_planes; i++) {
            buffer->planes[i].fd = dma_fd;
            v4l2_buf->m.planes[i].m.fd = dma_fd;
            buffer->planes[i].mem_offset = params.offset[i];
            ret = NvBufferMemMap(dma_fd, i, NvBufferMem_Read_Write,
                                 (void **)&data);
            if (ret) {
                ctx->in_error = true;
                av_log(ctx->avctx, AV_LOG_ERROR,
                       "Error while Mapping buffer!\n");
                pthread_mutex_unlock(&ctx->queue_lock);
                return ret;
            }
            buffer->planes[i].data = data;
        }
        break;
    default:
        pthread_mutex_unlock(&ctx->queue_lock);
        return -1;
    }
    pthread_mutex_unlock(&ctx->queue_lock);

    return ret;
}

int
nvv4l2_unmap_out(nvv4l2_ctx_t *ctx, int index, enum v4l2_buf_type buf_type,
                 enum v4l2_memory mem_type, int dma_fd)
{
    int ret = 0;
    NvBuffer *buffer;
    pthread_mutex_lock(&ctx->queue_lock);

    if (buf_type == ctx->op_buf_type)
        buffer = ctx->op_buffers[index];
    else if (buf_type == ctx->cp_buf_type)
        buffer = ctx->cp_buffers[index];

    switch (mem_type) {
    case V4L2_MEMORY_DMABUF:
        for (uint32_t i = 0; i < buffer->n_planes; i++) {
            ret = NvBufferMemUnMap(dma_fd, i, (void **)&buffer->planes[i].data);
            if (ret) {
                ctx->in_error = true;
                av_log(ctx->avctx, AV_LOG_ERROR,
                       "Error while Unmapping buffer!\n");
                pthread_mutex_unlock(&ctx->queue_lock);
                return ret;
            }
        }
        break;
    default:
        pthread_mutex_unlock(&ctx->queue_lock);
        return -1;
    }
    pthread_mutex_unlock(&ctx->queue_lock);

    return ret;
}

int nvv4l2_allocate_memory(nvv4l2_ctx_t *ctx, NvBuffer *buffer)
{
     for (uint32_t i = 0; i < buffer->n_planes; i++) {
        buffer->planes[i].length = NVMAX(buffer->planes[i].fmt.sizeimage,
                                         buffer->planes[i].fmt.width *
                                          buffer->planes[i].fmt.bytesperpixel *
                                          buffer->planes[i].fmt.height);
        buffer->planes[i].data =
                (unsigned char *)NVMALLOC(sizeof(unsigned char) *
                                        buffer->planes[i].length);
        if (buffer->planes[i].data == NULL) {
            av_log(ctx->avctx, AV_LOG_ERROR,
                   "Could not allocate buffer %d plane %d!\n",
                   buffer->index, i);
            return -1;
        }
    }
    return 0;
}

int nvv4l2_map(nvv4l2_ctx_t *ctx, NvBuffer *buffer)
{
    if (buffer->memory_type != V4L2_MEMORY_MMAP) {
        av_log(ctx->avctx, AV_LOG_ERROR,
               "Buffer type %d can't be mapped!\n", buffer->memory_type);
        return -1;
    }

    if (buffer->mapped) {
        av_log(ctx->avctx, AV_LOG_VERBOSE, "Buffer %d already mapped!\n",
               buffer->index);
        return 0;
    }

    for (uint32_t i = 0; i < buffer->n_planes; i++) {
        if (buffer->planes[i].fd == -1) {
            return -1;
        }

        buffer->planes[i].data =
            (unsigned char *)mmap(NULL, buffer-> planes[i].length,
                                         PROT_READ | PROT_WRITE, MAP_SHARED,
                                         buffer->planes[i].fd,
                                         buffer->planes
                                         [i].mem_offset);
        if (buffer->planes[i].data == MAP_FAILED) {
            av_log(ctx->avctx, AV_LOG_ERROR,
                   "Could not map buffer %d plane %d!\n", buffer->index, i);
            return -1;
        }
    }
    buffer->mapped = true;
    return 0;
}

void nvv4l2_unmap(nvv4l2_ctx_t *ctx, NvBuffer *buffer)
{
    if (buffer->memory_type != V4L2_MEMORY_MMAP || !buffer->mapped) {
        av_log(ctx->avctx, AV_LOG_VERBOSE,
            "Cannot unmap Buffer %d Only mapped MMAP buffer can be unmapped\n",
            buffer->index);
        return;
    }

    for (uint32_t i = 0; i < buffer->n_planes; i++) {
        if (buffer->planes[i].data) {
            munmap(buffer->planes[i].data, buffer->planes[i].length);
        }
        buffer->planes[i].data = NULL;
    }
    buffer->mapped = false;
}

void nvv4l2_destroyBuffer(nvv4l2_ctx_t *ctx, NvBuffer *buffer)
{
    if (buffer->mapped) {
        nvv4l2_unmap(ctx, buffer);
    }
}

int
nvv4l2_query_buffer(nvv4l2_ctx_t *ctx, enum v4l2_buf_type buf_type,
                    enum v4l2_memory memory_type, uint32_t num_planes,
                    uint32_t index)
{
    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane planes[NV_MAX_PLANES];
    NvBuffer *buffer;
    int ret;
    uint32_t j;

    memset(&v4l2_buf, 0, sizeof(struct v4l2_buffer));
    memset(planes, 0, sizeof(planes));
    v4l2_buf.index = index;
    v4l2_buf.type = buf_type;
    v4l2_buf.memory = memory_type;
    v4l2_buf.m.planes = planes;
    v4l2_buf.length = num_planes;

    ret = v4l2_ioctl(ctx->fd, VIDIOC_QUERYBUF, &v4l2_buf);
    if (ret) {
        av_log(ctx->avctx, AV_LOG_ERROR, "Error in QueryBuf!\n");
    } else {
        if (buf_type == ctx->op_buf_type) {
            buffer = ctx->op_buffers[index];
        } else if (buf_type == ctx->cp_buf_type) {
            buffer = ctx->cp_buffers[index];
        }

        for (j = 0; j < v4l2_buf.length; j++) {
            buffer->planes[j].length = v4l2_buf.m.planes[j].length;
            buffer->planes[j].mem_offset =
                v4l2_buf.m.planes[j].m.mem_offset;
        }
    }

    return ret;
}

int
nvv4l2_export_buffer(nvv4l2_ctx_t *ctx, enum v4l2_buf_type buf_type,
                     uint32_t num_planes, uint32_t index)
{
    struct v4l2_exportbuffer expbuf;
    NvBuffer *buffer;
    int ret;

    memset(&expbuf, 0, sizeof(expbuf));
    expbuf.type = buf_type;
    expbuf.index = index;

    for (uint32_t i = 0; i < num_planes; i++) {
        expbuf.plane = i;
        ret = v4l2_ioctl(ctx->fd, VIDIOC_EXPBUF, &expbuf);
        if (ret) {
            av_log(ctx->avctx, AV_LOG_ERROR, "Error in ExportBuf!\n");
        }
        else {
            if (buf_type == ctx->op_buf_type) {
                buffer = ctx->op_buffers[index];
            } else if (buf_type == ctx->cp_buf_type) {
                buffer = ctx->cp_buffers[index];
            }
            buffer->planes[i].fd = expbuf.fd;
        }
    }
    return 0;
}

int
nvv4l2_fill_buffer_plane_format(nvv4l2_ctx_t *ctx,
                                uint32_t *num_planes,
                                NvBufferPlaneFormat *planefmts,
                                uint32_t width, uint32_t height,
                                uint32_t pixfmt)
{
    switch (pixfmt) {
    case V4L2_PIX_FMT_YUV444M:
        *num_planes = 3;

        planefmts[0].width = width;
        planefmts[1].width = width;
        planefmts[2].width = width;

        planefmts[0].height = height;
        planefmts[1].height = height;
        planefmts[2].height = height;

        planefmts[0].bytesperpixel = 1;
        planefmts[1].bytesperpixel = 1;
        planefmts[2].bytesperpixel = 1;
        break;
    case V4L2_PIX_FMT_YUV420M:
        *num_planes = 3;

        planefmts[0].width = width;
        planefmts[1].width = width / 2;
        planefmts[2].width = width / 2;

        planefmts[0].height = height;
        planefmts[1].height = height / 2;
        planefmts[2].height = height / 2;

        planefmts[0].bytesperpixel = 1;
        planefmts[1].bytesperpixel = 1;
        planefmts[2].bytesperpixel = 1;
        break;
    case V4L2_PIX_FMT_NV12M:
        *num_planes = 2;

        planefmts[0].width = width;
        planefmts[1].width = width / 2;

        planefmts[0].height = height;
        planefmts[1].height = height / 2;

        planefmts[0].bytesperpixel = 1;
        planefmts[1].bytesperpixel = 2;
        break;
    case V4L2_PIX_FMT_P010M:
        *num_planes = 2;

        planefmts[0].width = width;
        planefmts[1].width = width / 2;

        planefmts[0].height = height;
        planefmts[1].height = height / 2;

        planefmts[0].bytesperpixel = 2;
        planefmts[1].bytesperpixel = 4;
        break;
    default:
        av_log(ctx->avctx, AV_LOG_ERROR, "Unsupported pixel format!");
        return -1;
    }

    return 0;
}

int
nvv4l2_dq_event(nvv4l2_ctx_t *ctx, struct v4l2_event *event,
                uint32_t max_wait_ms)
{
    int ret;
    do {
        ret = v4l2_ioctl(ctx->fd, VIDIOC_DQEVENT, event);

        if (errno != EAGAIN) {
            break;
        } else if (max_wait_ms-- == 0) {
            break;
        } else {
            usleep(1000);
        }
    }
    while (ret && (ctx->op_streamon || ctx->cp_streamon));

    return ret;
}

int
nvv4l2_dq_buffer(nvv4l2_ctx_t *ctx, struct v4l2_buffer *v4l2_buf,
                 NvBuffer **buffer, enum v4l2_buf_type buf_type,
                 enum v4l2_memory memory_type, uint32_t num_retries)
{
    int ret;
    bool is_in_error = false;
    v4l2_buf->type = buf_type;
    v4l2_buf->memory = memory_type;
    do {
        ret = v4l2_ioctl(ctx->fd, VIDIOC_DQBUF, v4l2_buf);
        if (ret == 0) {
            pthread_mutex_lock(&ctx->queue_lock);
            switch (v4l2_buf->type) {
            case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
                if (buffer)
                    *buffer = ctx->op_buffers[v4l2_buf->index];
                for (uint32_t i = 0;
                     i < ctx->op_buffers[v4l2_buf->index]->n_planes; i++) {
                    ctx->op_buffers[v4l2_buf->index]->planes[i].bytesused =
                        v4l2_buf->m.planes[i].bytesused;
                }
                ctx->num_queued_op_buffers--;
                break;

            case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
                if (buffer)
                    *buffer = ctx->cp_buffers[v4l2_buf->index];
                for (uint32_t i = 0;
                     i < ctx->cp_buffers[v4l2_buf->index]->n_planes; i++) {
                    ctx->cp_buffers[v4l2_buf->index]->planes[i].bytesused =
                        v4l2_buf->m.planes[i].bytesused;
                }
                ctx->num_queued_cp_buffers--;
                break;

            default:
                av_log(ctx->avctx, AV_LOG_ERROR, "Invalid buffer type!\n");
            }
            pthread_cond_broadcast(&ctx->queue_cond);
            pthread_mutex_unlock(&ctx->queue_lock);
        } else if (errno == EAGAIN) {
            pthread_mutex_lock(&ctx->queue_lock);
            if (v4l2_buf->flags & V4L2_BUF_FLAG_LAST) {
                pthread_mutex_unlock(&ctx->queue_lock);
                break;
            }
            pthread_mutex_unlock(&ctx->queue_lock);

            if (num_retries-- == 0) {
                av_log(ctx->avctx, AV_LOG_VERBOSE, "Resource unavailable!\n");
                break;
            }
        } else {
            is_in_error = true;
            break;
        }
    }
    while (ret && !is_in_error);

    return ret;
}

int
nvv4l2_q_buffer(nvv4l2_ctx_t *ctx, struct v4l2_buffer *v4l2_buf,
                NvBuffer *buffer, enum v4l2_buf_type buf_type,
                enum v4l2_memory memory_type, int num_planes)
{
    int ret;

    pthread_mutex_lock(&ctx->queue_lock);

    if (buf_type == ctx->op_buf_type)
        buffer = ctx->op_buffers[v4l2_buf->index];
    else if (buf_type == ctx->cp_buf_type)
        buffer = ctx->cp_buffers[v4l2_buf->index];

    v4l2_buf->type = buf_type;
    v4l2_buf->memory = memory_type;
    v4l2_buf->length = num_planes;

    switch (memory_type) {
    case V4L2_MEMORY_USERPTR:
        for (uint32_t i = 0; i < buffer->n_planes; i++) {
            v4l2_buf->m.planes[i].m.userptr =
                (unsigned long) buffer->planes[i].data;
            v4l2_buf->m.planes[i].bytesused = buffer->planes[i].bytesused;
        }
        break;
    case V4L2_MEMORY_MMAP:
        for (uint32_t i = 0; i < buffer->n_planes; i++) {
            v4l2_buf->m.planes[i].bytesused = buffer->planes[i].bytesused;
        }
        break;

    case V4L2_MEMORY_DMABUF:
        break;

    default:
        pthread_cond_broadcast(&ctx->queue_cond);
        pthread_mutex_unlock(&ctx->queue_lock);
        return -1;
    }
    ret = v4l2_ioctl(ctx->fd, VIDIOC_QBUF, v4l2_buf);

    if (ret == 0) {
        if (v4l2_buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
            ctx->num_queued_op_buffers++;
        } else {
            ctx->num_queued_cp_buffers++;
        }
        pthread_cond_broadcast(&ctx->queue_cond);
    }
    pthread_mutex_unlock(&ctx->queue_lock);

    return ret;
}

int
nvv4l2_req_buffers_on_capture_plane(nvv4l2_ctx_t *ctx,
                                    enum v4l2_buf_type buf_type,
                                    enum v4l2_memory mem_type,
                                    int num_buffers)
{
    struct v4l2_requestbuffers reqbufs;
    int ret;
    memset(&reqbufs, 0, sizeof(struct v4l2_requestbuffers));

    reqbufs.count = num_buffers;
    reqbufs.memory = mem_type;
    reqbufs.type = buf_type;

    ret = v4l2_ioctl(ctx->fd, VIDIOC_REQBUFS, &reqbufs);
    if (ret)
        return ret;

    if (reqbufs.count) {
        ctx->cp_buffers =
            (NvBuffer **)NVMALLOC(reqbufs.count * sizeof(NvBuffer *));
        for (uint32_t i = 0; i < reqbufs.count; ++i) {
            ctx->cp_buffers[i] = (NvBuffer *)NVMALLOC(sizeof(NvBuffer));
            nvv4l2_create_bufferfmt(ctx->cp_buffers[i], buf_type, mem_type,
                             ctx->cp_num_planes, ctx->cp_planefmts, i);
        }
    } else if (ctx->cp_buffers) {
        for (uint32_t i = 0; i < ctx->cp_num_buffers; ++i) {
            for (uint32_t j = 0; j < ctx->cp_buffers[i]->n_planes &&
                 mem_type == V4L2_MEMORY_USERPTR; j++) {
                NVFREE(ctx->cp_buffers[i]->planes[j].data);
            }
            NVFREE(ctx->cp_buffers[i]);
        }
        NVFREE(ctx->cp_buffers);
        ctx->cp_buffers = NULL;
    }
    ctx->cp_num_buffers = reqbufs.count;

    return ret;
}

int
nvv4l2_req_buffers_on_output_plane(nvv4l2_ctx_t *ctx,
                                   enum v4l2_buf_type buf_type,
                                   enum v4l2_memory mem_type,
                                   int num_buffers)
{
    struct v4l2_requestbuffers reqbufs;
    int ret;
    memset(&reqbufs, 0, sizeof(struct v4l2_requestbuffers));

    reqbufs.count = num_buffers;
    reqbufs.memory = mem_type;
    reqbufs.type = buf_type;

    ret = v4l2_ioctl(ctx->fd, VIDIOC_REQBUFS, &reqbufs);
    if (ret)
        return ret;

    if (reqbufs.count) {
        ctx->op_buffers =
            (NvBuffer **)NVMALLOC(reqbufs.count * sizeof(NvBuffer *));
        for (uint32_t i = 0; i < reqbufs.count; ++i) {
            ctx->op_buffers[i] = (NvBuffer *)NVMALLOC(sizeof(NvBuffer));
            nvv4l2_create_bufferfmt(ctx->op_buffers[i], buf_type, mem_type,
                             ctx->op_num_planes, ctx->op_planefmts, i);
        }
    } else if (ctx->op_buffers) {
        for (uint32_t i = 0; i < ctx->op_num_buffers; ++i) {
            for (uint32_t j = 0; j < ctx->op_buffers[i]->n_planes &&
                 mem_type == V4L2_MEMORY_USERPTR; j++) {
                NVFREE(ctx->op_buffers[i]->planes[j].data);
            }
            NVFREE(ctx->op_buffers[i]);
        }
        NVFREE(ctx->op_buffers);
        ctx->op_buffers = NULL;
    }
    ctx->op_num_buffers = reqbufs.count;

    return ret;
}

int
nvv4l2_set_ext_controls(int fd, uint32_t id,
                        uint32_t class, uint32_t value)
{
    int ret;
    struct v4l2_ext_control ctl;
    struct v4l2_ext_controls ctrls;

    memset(&ctl, 0, sizeof(struct v4l2_ext_control));
    memset(&ctrls, 0, sizeof(struct v4l2_ext_controls));
    ctl.id = id;
    ctl.value = value;
    ctrls.count = 1;
    ctrls.controls = &ctl;
    ctrls.ctrl_class = class;

    ret = v4l2_ioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrls);

    return ret;
}

int
nvv4l2_set_ext_control_qp_range(int fd, uint32_t qpmin,
                                uint32_t qpmax)
{
    int ret;
    struct v4l2_ext_control ctl;
    struct v4l2_ext_controls ctrls;
    v4l2_ctrl_video_qp_range qprange;

    memset(&ctl, 0, sizeof(struct v4l2_ext_control));
    memset(&ctrls, 0, sizeof(struct v4l2_ext_controls));

    qprange.MinQpI = qpmin;
    qprange.MaxQpI = qpmax;
    qprange.MinQpP = qpmin;
    qprange.MaxQpP = qpmax;
    qprange.MinQpB = qpmin;
    qprange.MaxQpB = qpmax;

    ctl.id = V4L2_CID_MPEG_VIDEOENC_QP_RANGE;
    ctl.string = (char *)&qprange;

    ctrls.count = 1;
    ctrls.controls = &ctl;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    ret = v4l2_ioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrls);

    return ret;
}

int
nvv4l2_set_ext_control_constant_qp(int fd, uint32_t qpval)
{
    int ret;
    struct v4l2_ext_control ctl[3];
    struct v4l2_ext_controls ctrls;

    memset(&ctl, 0, sizeof(ctl));
    memset(&ctrls, 0, sizeof(ctrls));

    ctl[0].id = V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE;
    ctl[0].value = 0; // disable rate control

    ctl[1].id = V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP;
    ctl[1].value = qpval;

    ctl[2].id = V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP;
    ctl[2].value = qpval;

    ctrls.count = 3;
    ctrls.controls = &ctl[0];
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    ret = v4l2_ioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrls);

    return ret;
}

int
nvv4l2_get_ext_control_metadata(int fd, uint32_t buffer_index,
                    v4l2_ctrl_videoenc_outputbuf_metadata *enc_metadata)
{
    int ret;
    struct v4l2_ext_control ctl;
    struct v4l2_ext_controls ctrls;
    v4l2_ctrl_video_metadata metadata;

    memset(&ctl, 0, sizeof(struct v4l2_ext_control));
    memset(&ctrls, 0, sizeof(struct v4l2_ext_controls));

    metadata.buffer_index = buffer_index;
    metadata.VideoEncMetadata =
        (v4l2_ctrl_videoenc_outputbuf_metadata *)&enc_metadata;

    ctl.id = V4L2_CID_MPEG_VIDEOENC_METADATA;
    ctl.string = (char *)&metadata;

    ctrls.count = 1;
    ctrls.controls = &ctl;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    ret = v4l2_ioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrls);

    return ret;
}

int
nvv4l2_set_stream_control_framerate(int fd,  uint32_t buf_type,
                                    uint32_t framerate_num,
                                    uint32_t framerate_den)
{
    int ret;
    struct v4l2_streamparm parms;

    memset(&parms, 0, sizeof(parms));

    parms.parm.output.timeperframe.numerator = framerate_den;
    parms.parm.output.timeperframe.denominator = framerate_num;
    parms.type = buf_type;

    ret = v4l2_ioctl(fd, VIDIOC_S_PARM, &parms);

    return ret;
}

int
nvv4l2_subscribe_event(int fd, uint32_t type, uint32_t id,
                       uint32_t flags)
{
    struct v4l2_event_subscription sub;
    int ret;

    memset(&sub, 0, sizeof(struct v4l2_event_subscription));

    sub.type = type;
    sub.id = id;
    sub.flags = flags;

    ret = v4l2_ioctl(fd, VIDIOC_SUBSCRIBE_EVENT, &sub);

    return ret;
}

void
nvv4l2_dbg_plane_supported_formats(nvv4l2_ctx_t *ctx,
                                   uint32_t buf_type)
{
    struct v4l2_fmtdesc fdesc;
    char fourcc[5] = {0};
    int ret;

    memset(&fdesc, 0, sizeof(fdesc));
    fdesc.type = buf_type;

    av_log(ctx->avctx, AV_LOG_INFO,
           "%s plane format support:\n",
           buf_type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE ?
                                    "Output" : "Capture");

    while (true) {
        ret = v4l2_ioctl(ctx->fd, VIDIOC_ENUM_FMT, &fdesc);
        if (ret)
            break;

        memcpy(fourcc, &fdesc.pixelformat, 4);
        av_log(ctx->avctx, AV_LOG_INFO, "%d: %s (%s)\n", fdesc.index, fourcc, fdesc.description);
        fdesc.index++;
    }
}
