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

/**
 * Specifies the decoder device node.
 */
#ifndef __nvv4l2_H__
#define __nvv4l2_H__

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <libv4l2.h>
#include <linux/videodev2.h>
#include "avcodec.h"

#include "nvbuf_utils.h"
#include "v4l2_nv_extensions.h"

#define NV_MAX_BUFFERS 32

/**
 * Specifies the maximum number of planes a buffer can contain.
 */
#define NV_MAX_PLANES 3
#define NVMIN(a,b) (((a) < (b)) ? (a) : (b))
#define NVMAX(a, b) ((a) > (b) ? (a) : (b))

/* Use app malloc/free implementation */
#define NVMALLOC(size) (av_malloc((size)))
#define NVCALLOC(num, size) (av_mallocz((num) * (size)))
#define NVFREE(ptr) (av_free((ptr)))

typedef struct _queue {
    uint32_t capacity;
    uint32_t front;
    uint32_t back;
} NvQueues;

typedef enum {
    NV_PIX_NV12,
    NV_PIX_YUV420
} NvPixFormat;

typedef struct _NVPACKET {
    uint32_t flags;
    uint32_t payload_size;
    uint8_t *payload;
    uint32_t width;
    uint32_t height;
    uint64_t pts;
} NvPacket;

typedef struct _NVFRAME {
    uint32_t flags;
    uint32_t payload_size[3];
    uint8_t *payload[3];
    uint32_t width;
    uint32_t height;
    uint64_t pts;
} NvFrame;

typedef enum {
    NvVideoCodec_H264,              /**< H.264 */
    NvVideoCodec_MPEG4,             /**< MPEG-4 */
    NvVideoCodec_MPEG2,             /**< MPEG-2 */
    NvVideoCodec_VP8,               /**< VP8 */
    NvVideoCodec_VP9,               /**< VP9 */
    NvVideoCodec_HEVC,              /**< H.265/HEVC */
    NvVideoCodec_UNDEFINED,
} NvCodingType;

typedef struct {
    uint32_t width;                 /**< Holds the width of the plane in pixels. */
    uint32_t height;                /**< Holds the height of the plane in pixels. */

    uint32_t bytesperpixel;         /**< Holds the bytes used to represent one
                                         pixel in the plane. */
    uint32_t stride;                /**< Holds the stride of the plane in bytes. */
    uint32_t sizeimage;             /**< Holds the size of the plane in bytes. */
} NvBufferPlaneFormat;

    /**
     * Holds the buffer plane parameters.
     */

typedef struct {
    NvBufferPlaneFormat fmt;        /**< Holds the format of the plane. */
    uint8_t *data;                  /**< Holds a pointer to the plane memory. */
    uint32_t bytesused;             /**< Holds the number of valid bytes in the plane. */
    int fd;                         /**< Holds the file descriptor (FD) of the plane of the
                                     exported buffer, in the case of V4L2 MMAP buffers. */
    uint32_t mem_offset;            /**< Holds the offset of the first valid byte
                                         from the data pointer. */
    uint32_t length;                /**< Holds the size of the buffer in bytes. */
} NvBufferPlane;

typedef struct {
    enum v4l2_buf_type buf_type;    /**< Type of the buffer. */
    enum v4l2_memory memory_type;   /**< Type of memory associated with the buffer. */
    uint32_t index;                 /**< Holds the buffer index. */
    uint32_t n_planes;              /**< Holds the number of planes in the buffer. */
    NvBufferPlane planes[NV_MAX_PLANES];
    bool mapped;
} NvBuffer;

typedef struct {
    uint32_t width;
    uint32_t height;
    bool low_latency;
    uint32_t profile;
    uint32_t bitrate;
    uint32_t level;
    uint32_t tier;
    uint32_t preset_type;
    uint32_t lossless;
    uint32_t iframe_interval;
    uint32_t idr_interval;
    uint32_t fps_n;
    uint32_t fps_d;
    int qmin;
    int qmax;
    int num_b_frames;
    uint32_t num_ref;
    bool sps_pps_at_idr;
    uint32_t ratecontrol;
} NvEncoder;

/**
 * @brief Struct defining the decoder context.
 * The video decoder device node is `/dev/nvhost-nvdec`. The category name
 * for the decoder is \c "NVDEC".
 *
 * The context stores the information for decoding.
 * Refer to [V4L2 Video Decoder](group__V4L2Dec.html) for more information on the decoder.
 */

typedef struct {
    uint32_t codec_width;
    uint32_t codec_height;

    uint32_t op_pixfmt;
    uint32_t cp_pixfmt;
    enum v4l2_memory op_mem_type;
    enum v4l2_memory cp_mem_type;
    enum v4l2_buf_type op_buf_type;
    enum v4l2_buf_type cp_buf_type;
    NvBufferPlaneFormat op_planefmts[NV_MAX_PLANES];
    NvBufferPlaneFormat cp_planefmts[NV_MAX_PLANES];
    uint32_t cp_num_planes;
    uint32_t op_num_planes;
    uint32_t cp_num_buffers;
    uint32_t op_num_buffers;
    NvQueues *export_pool;
    NvBuffer **op_buffers;
    NvBuffer **cp_buffers;
    uint32_t num_active_op_buffers;
    uint32_t num_queued_op_buffers;
    uint32_t num_queued_cp_buffers;

    pthread_mutex_t queue_lock;
    pthread_cond_t queue_cond;
    pthread_mutex_t frame_lock;
    pthread_cond_t frame_cond;
    pthread_mutex_t pool_lock;
    pthread_t capture_thread;

    bool in_error;
    bool eos;
    bool op_streamon;
    bool cp_streamon;
    bool draining_event;
    bool low_latency;

    int fd;
    int out_dma_fd;
    int dmabuff_fd[NV_MAX_BUFFERS];

    int      plane_dma_fd[NV_MAX_BUFFERS];
    uint32_t plane_width[MAX_NUM_PLANES];
    uint32_t plane_height[MAX_NUM_PLANES];
    uint64_t frame_pts[NV_MAX_BUFFERS];

    uint8_t *packet[NV_MAX_BUFFERS];
    uint32_t packet_buf_size[NV_MAX_BUFFERS];
    uint32_t packet_size[NV_MAX_BUFFERS];
    bool packet_keyflag[NV_MAX_BUFFERS];

    NvEncoder *enc;
    AVCodecContext *avctx;
} nvv4l2_ctx_t;

/* NVV4L2 common functions */
uint32_t nvv4l2_map_nvcodec_type(NvCodingType nv_codec_type);
int
nvv4l2_pool_idx_next(nvv4l2_ctx_t *ctx, NvQueues *q);
void
nvv4l2_pool_push(nvv4l2_ctx_t *ctx, NvQueues *q);
int
nvv4l2_pool_pop(nvv4l2_ctx_t *ctx, NvQueues *q);
int
nvv4l2_create_bufferfmt(NvBuffer *buffer, enum v4l2_buf_type buf_type,
                     enum v4l2_memory memory_type, uint32_t n_planes,
                     NvBufferPlaneFormat *fmt, uint32_t index);
int
nvv4l2_map_out(nvv4l2_ctx_t *ctx, struct v4l2_buffer *v4l2_buf,
               enum v4l2_buf_type buf_type, enum v4l2_memory mem_type,
               int dma_fd);
int
nvv4l2_unmap_out(nvv4l2_ctx_t *ctx, int index, enum v4l2_buf_type buf_type,
                 enum v4l2_memory mem_type, int dma_fd);
void
nvv4l2_destroyBuffer(nvv4l2_ctx_t *ctx, NvBuffer *buffer);
int
nvv4l2_allocate_memory(nvv4l2_ctx_t *ctx, NvBuffer *buffer);
int
nvv4l2_map(nvv4l2_ctx_t *ctx, NvBuffer *buffer);
void
nvv4l2_unmap(nvv4l2_ctx_t *ctx, NvBuffer *buffer);
int
nvv4l2_query_buffer(nvv4l2_ctx_t *ctx, enum v4l2_buf_type buf_type,
                    enum v4l2_memory memory_type, uint32_t num_planes,
                    uint32_t index);
int
nvv4l2_export_buffer(nvv4l2_ctx_t *ctx, enum v4l2_buf_type buf_type,
                     uint32_t num_planes, uint32_t index);
int
nvv4l2_fill_buffer_plane_format(nvv4l2_ctx_t *ctx,
                                uint32_t *num_planes,
                                NvBufferPlaneFormat *planefmts,
                                uint32_t width, uint32_t height,
                                uint32_t pixfmt);
int
nvv4l2_dq_event(nvv4l2_ctx_t *ctx, struct v4l2_event *event,
                uint32_t max_wait_ms);
int
nvv4l2_dq_buffer(nvv4l2_ctx_t *ctx, struct v4l2_buffer *v4l2_buf,
                 NvBuffer **buffer, enum v4l2_buf_type buf_type,
                 enum v4l2_memory memory_type, uint32_t num_retries);
int
nvv4l2_q_buffer(nvv4l2_ctx_t *ctx, struct v4l2_buffer *v4l2_buf,
                NvBuffer *buffer, enum v4l2_buf_type buf_type,
                enum v4l2_memory memory_type, int num_planes);
int
nvv4l2_req_buffers_on_capture_plane(nvv4l2_ctx_t *ctx,
                                    enum v4l2_buf_type buf_type,
                                    enum v4l2_memory mem_type,
                                    int num_buffers);
int
nvv4l2_req_buffers_on_output_plane(nvv4l2_ctx_t *ctx,
                                   enum v4l2_buf_type buf_type,
                                   enum v4l2_memory mem_type,
                                   int num_buffers);
int
nvv4l2_set_ext_controls(int fd, uint32_t id,
                        uint32_t class, uint32_t value);
int
nvv4l2_set_ext_control_qp_range(int fd, uint32_t qpmin,
                                uint32_t qpmax);
int
nvv4l2_set_ext_control_constant_qp(int fd, uint32_t qpval);
int
nvv4l2_get_ext_control_metadata(int fd, uint32_t buffer_index,
                    v4l2_ctrl_videoenc_outputbuf_metadata *enc_metadata);
int
nvv4l2_set_stream_control_framerate(int fd,  uint32_t buf_type,
                                    uint32_t framerate_num,
                                    uint32_t framerate_den);
int
nvv4l2_subscribe_event(int fd, uint32_t type, uint32_t id,
                       uint32_t flags);

/* NVV4L2 debug functions */
void
nvv4l2_dbg_plane_supported_formats(nvv4l2_ctx_t *ctx,
                                   uint32_t buf_type);

/* NVV4L2 decoder functions */
nvv4l2_ctx_t *nvv4l2_create_decoder(AVCodecContext *avctx,
                                    NvCodingType nv_codec_type,
                                    int pix_fmt);
int
nvv4l2_decoder_put_packet(AVCodecContext *avctx, nvv4l2_ctx_t *ctx,
                          NvPacket *packet);
int
nvv4l2_decoder_get_frame(AVCodecContext *avctx, nvv4l2_ctx_t *ctx,
                         int *buf_index, NvFrame *frame);
int nvv4l2_decoder_close(AVCodecContext *avctx, nvv4l2_ctx_t *ctx);

/* NVV4L2 encoder functions */
nvv4l2_ctx_t *nvv4l2_create_encoder(AVCodecContext *avctx,
                                    NvEncoder *enc,
                                    NvCodingType codingType,
                                    int pix_fmt);
int
nvv4l2_encoder_put_frame(AVCodecContext *avctx, nvv4l2_ctx_t *ctx,
                         NvFrame *frame);
int
nvv4l2_encoder_get_packet(AVCodecContext *avctx,
                          nvv4l2_ctx_t *ctx,
                          NvPacket *packet);
int
nvv4l2_encoder_close(AVCodecContext *avctx, nvv4l2_ctx_t *ctx);

#endif
