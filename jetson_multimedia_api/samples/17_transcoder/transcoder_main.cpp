#include "NvApplicationProfiler.h"
#include "NvUtils.h"
#include <errno.h>
#include <fstream>
#include <iostream>
#include <linux/videodev2.h>
#include <malloc.h>
#include <pthread.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <functional>
#include <nvbuf_utils.h>
#include <cuda.h>
#include <cudaEGL.h>

#include "video_decode.h"
#include "video_encode.h"
#include "nvbuf_utils.h"



#include <queue>
#include <mutex>
#include <condition_variable>

const char dec_name[] = "dec0";

    template <typename T>
    class SharedQueue
    {
    public:
        SharedQueue();
        ~SharedQueue();

        T& front();
        void pop_front();

        void push_back(const T& item);
        void push_back(T&& item);

        int size();
        bool empty();

    private:
        std::deque<T> queue_;
        std::mutex mutex_;
        std::condition_variable cond_;
    }; 

    template <typename T>
    SharedQueue<T>::SharedQueue(){}

    template <typename T>
    SharedQueue<T>::~SharedQueue(){}

    template <typename T>
    T& SharedQueue<T>::front()
    {
        std::unique_lock<std::mutex> mlock(mutex_);
        while (queue_.empty())
        {
            cond_.wait(mlock);
        }
        return queue_.front();
    }

    template <typename T>
    void SharedQueue<T>::pop_front()
    {
        std::unique_lock<std::mutex> mlock(mutex_);
        while (queue_.empty())
        {
            cond_.wait(mlock);
        }
        queue_.pop_front();
    }     

    template <typename T>
    void SharedQueue<T>::push_back(const T& item)
    {
        std::unique_lock<std::mutex> mlock(mutex_);
        queue_.push_back(item);
        mlock.unlock();     // unlock before notificiation to minimize mutex con
        cond_.notify_one(); // notify one waiting thread

    }

    template <typename T>
    void SharedQueue<T>::push_back(T&& item)
    {
        std::unique_lock<std::mutex> mlock(mutex_);
        queue_.push_back(std::move(item));
        mlock.unlock();     // unlock before notificiation to minimize mutex con
        cond_.notify_one(); // notify one waiting thread

    }

    template <typename T>
    int SharedQueue<T>::size()
    {
        std::unique_lock<std::mutex> mlock(mutex_);
        int size = queue_.size();
        mlock.unlock();
        return size;
    }



typedef struct
{
  dec_context_t dec_ctx;
  enc_context_t enc_ctx;
  pthread_t encoder_proc;
  sem_t sema;
  int argc;
  char **argv;
  SharedQueue<struct v4l2_buffer> buffers;
  SharedQueue<struct v4l2_buffer> release_buffers;
} tc_context_t;


#define TEST_ERROR(cond, str, label) if(cond) { \
                                        cerr << str << endl; \
                                        error = 1; \
                                        goto label; }

#define TEST_PARSE_ERROR(cond, label) if(cond) { \
    cerr << "Error parsing runtime parameter changes string" << endl; \
    goto label; }


#define MICROSECOND_UNIT 1000000
#define CHUNK_SIZE 4000000
#define MIN(a,b) (((a) < (b)) ? (a) : (b))
#define IS_DIGIT(c) (c >= '0' && c <= '9')

#define IS_NAL_UNIT_START(buffer_ptr) (!buffer_ptr[0] && !buffer_ptr[1] && \
        !buffer_ptr[2] && (buffer_ptr[3] == 1))

#define IS_NAL_UNIT_START1(buffer_ptr) (!buffer_ptr[0] && !buffer_ptr[1] && \
        (buffer_ptr[2] == 1))

#define H264_NAL_UNIT_CODED_SLICE  1
#define H264_NAL_UNIT_CODED_SLICE_IDR  5

#define HEVC_NUT_TRAIL_N  0
#define HEVC_NUT_RASL_R  9
#define HEVC_NUT_BLA_W_LP  16
#define HEVC_NUT_CRA_NUT  21

#define IVF_FILE_HDR_SIZE   32
#define IVF_FRAME_HDR_SIZE  12

#define IS_H264_NAL_CODED_SLICE(buffer_ptr) ((buffer_ptr[0] & 0x1F) == H264_NAL_UNIT_CODED_SLICE)
#define IS_H264_NAL_CODED_SLICE_IDR(buffer_ptr) ((buffer_ptr[0] & 0x1F) == H264_NAL_UNIT_CODED_SLICE_IDR)

#define GET_H265_NAL_UNIT_TYPE(buffer_ptr) ((buffer_ptr[0] & 0x7E) >> 1)

using namespace std;








static void
abort(enc_context_t *ctx)
{
    ctx->got_error = true;
    ctx->enc->abort();
}

static void
abort(dec_context_t *ctx)
{
    ctx->got_error = true;
    ctx->dec->abort();
}










//
// Encoder
//

//Initialise CRC Rec and creates CRC Table based on the polynomial.
static
Crc* InitCrc(unsigned int CrcPolynomial)
{
    unsigned short int i;
    unsigned short int j;
    unsigned int tempcrc;
    Crc *phCrc;
    phCrc = (Crc*) malloc (sizeof(Crc));
    if (phCrc == NULL)
    {
        cerr << "Mem allocation failed for Init CRC" <<endl;
        return NULL;
    }

    memset (phCrc, 0, sizeof(Crc));

    for (i = 0; i <= 255; i++)
    {
        tempcrc = i;
        for (j = 8; j > 0; j--)
        {
            if (tempcrc & 1)
            {
                tempcrc = (tempcrc >> 1) ^ CrcPolynomial;
            }
            else
            {
                tempcrc >>= 1;
            }
        }
        phCrc->CRCTable[i] = tempcrc;
    }

    phCrc->CrcValue = 0;
    return phCrc;
}

//Calculates CRC of data provided in by buffer.
static
void CalculateCrc(Crc *phCrc, unsigned char *buffer, uint32_t count)
{
    unsigned char *p;
    unsigned int temp1;
    unsigned int temp2;
    unsigned int crc = phCrc->CrcValue;
    unsigned int *CRCTable = phCrc->CRCTable;

    if(!count)
        return;

    p = (unsigned char *) buffer;
    while (count-- != 0)
    {
        temp1 = (crc >> 8) & 0x00FFFFFFL;
        temp2 = CRCTable[((unsigned int) crc ^ *p++) & 0xFF];
        crc = temp1 ^ temp2;
    }

    phCrc->CrcValue = crc;
}

//Closes CRC related handles.
static
void CloseCrc(Crc **phCrc)
{
    if (*phCrc)
        free (*phCrc);
}

static int
write_encoder_output_frame(ofstream * stream, NvBuffer * buffer)
{
    stream->write((char *) buffer->planes[0].data, buffer->planes[0].bytesused);
    return 0;
}


static void
enc_set_defaults(enc_context_t * ctx)
{
    memset(ctx, 0, sizeof(enc_context_t));

    ctx->raw_pixfmt = V4L2_PIX_FMT_YUV420M;
    ctx->bitrate = 4 * 1024 * 1024;
    ctx->peak_bitrate = 0;
    ctx->profile = V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE;
    ctx->ratecontrol = V4L2_MPEG_VIDEO_BITRATE_MODE_CBR;
    ctx->iframe_interval = 30;
    ctx->externalRPS = false;
    ctx->enableGDR = false;
    ctx->enableROI = false;
    ctx->bnoIframe = false;
    ctx->bGapsInFrameNumAllowed = false;
    ctx->bReconCrc = false;
    ctx->enableLossless = false;
    ctx->nH264FrameNumBits = 0;
    ctx->nH265PocLsbBits = 0;
    ctx->idr_interval = 256;
    ctx->level = V4L2_MPEG_VIDEO_H264_LEVEL_5_1;
    ctx->fps_n = 30;
    ctx->fps_d = 1;
    ctx->gdr_start_frame_number = 0xffffffff;
    ctx->gdr_num_frames = 0xffffffff;
    ctx->gdr_out_frame_number = 0xffffffff;
    ctx->num_b_frames = (uint32_t) -1;
    ctx->nMinQpI = (uint32_t)QP_RETAIN_VAL;
    ctx->nMaxQpI = (uint32_t)QP_RETAIN_VAL;
    ctx->nMinQpP = (uint32_t)QP_RETAIN_VAL;
    ctx->nMaxQpP = (uint32_t)QP_RETAIN_VAL;
    ctx->nMinQpB = (uint32_t)QP_RETAIN_VAL;
    ctx->nMaxQpB = (uint32_t)QP_RETAIN_VAL;
    ctx->use_gold_crc = false;
    ctx->pBitStreamCrc = NULL;
    ctx->externalRCHints = false;
    ctx->input_metadata = false;
    ctx->sMaxQp = 51;
    ctx->stress_test = 1;
    
    ctx->copy_timestamp = false;
    ctx->start_ts = 0;
    ctx->max_perf = 0;
    //ctx->blocking_mode = 1;
}

static int
get_next_parsed_pair(enc_context_t *ctx, char *id, uint32_t *value)
{
    char charval;

    *ctx->runtime_params_str >> *id;
    if (ctx->runtime_params_str->eof())
    {
        return -1;
    }

    charval = ctx->runtime_params_str->peek();
    if (!IS_DIGIT(charval))
    {
        return -1;
    }

    *ctx->runtime_params_str >> *value;

    *ctx->runtime_params_str >> charval;
    if (ctx->runtime_params_str->eof())
    {
        return 0;
    }

    return charval;
}


static int
get_next_runtime_param_change_frame(enc_context_t *ctx)
{
    char charval;
    int ret;

    ret = get_next_parsed_pair(ctx, &charval, &ctx->next_param_change_frame);
    if(ret == 0)
    {
        return 0;
    }

    TEST_PARSE_ERROR((ret != ';' && ret != ',') || charval != 'f', err);

    return 0;

err:
    cerr << "Skipping further runtime parameter changes" <<endl;
    delete ctx->runtime_params_str;
    ctx->runtime_params_str = NULL;
    return -1;
}

static int
setup_output_dmabuf(enc_context_t *ctx, uint32_t num_buffers )
{
    int ret=0;
    NvBufferCreateParams cParams;
    int fd;
    ret = ctx->enc->output_plane.reqbufs(V4L2_MEMORY_DMABUF,num_buffers);
    if(ret)
    {
        cerr << "reqbufs failed for output plane V4L2_MEMORY_DMABUF" << endl;
        return ret;
    }
    for (uint32_t i = 0; i < ctx->enc->output_plane.getNumBuffers(); i++)
    {
        cParams.width = ctx->width;
        cParams.height = ctx->height;
        cParams.layout = NvBufferLayout_Pitch;
        /*if (ctx->enableLossless && ctx->encoder_pixfmt == V4L2_PIX_FMT_H264)
        {
            cParams.colorFormat = NvBufferColorFormat_YUV444;
        }
        else if (ctx->profile == V4L2_MPEG_VIDEO_H265_PROFILE_MAIN10)
        {
            cParams.colorFormat = NvBufferColorFormat_NV12_10LE;
        }
        else
        {
            cParams.colorFormat = ctx->enable_extended_colorformat ?
                 NvBufferColorFormat_YUV420_ER : NvBufferColorFormat_YUV420;
        }*/
        cParams.colorFormat = NvBufferColorFormat_NV12;
        cParams.nvbuf_tag = NvBufferTag_VIDEO_ENC;
        cParams.payloadType = NvBufferPayload_SurfArray;
        ret = NvBufferCreateEx(&fd, &cParams);
        if(ret < 0)
        {
            cerr << "Failed to create NvBuffer" << endl;
            return ret;
        }
        ctx->output_plane_fd[i]=fd;
    }
    return ret;
}

static bool
encoder_capture_plane_dq_callback(struct v4l2_buffer *v4l2_buf, NvBuffer * buffer,
                                  NvBuffer * shared_buffer, void *arg)
{
    enc_context_t *ctx = (enc_context_t *) arg;
    NvVideoEncoder *enc = ctx->enc;
    pthread_setname_np(pthread_self(), "EncCapPlane");
    uint32_t frame_num = ctx->enc->capture_plane.getTotalDequeuedBuffers() - 1;
    uint32_t ReconRef_Y_CRC = 0;
    uint32_t ReconRef_U_CRC = 0;
    uint32_t ReconRef_V_CRC = 0;
    static uint32_t num_encoded_frames = 1;
    struct v4l2_event ev;
    int ret = 0;

    if (v4l2_buf == NULL)
    {
        cout << "Error while dequeing buffer from output plane" << endl;
        abort(ctx);
        return false;
    }

    if (ctx->b_use_enc_cmd)
    {
        if(v4l2_buf->flags & V4L2_BUF_FLAG_LAST)
        {
            memset(&ev,0,sizeof(struct v4l2_event));
            ret = ctx->enc->dqEvent(ev,1000);
            if (ret < 0)
                cout << "Error in dqEvent" << endl;
            if(ev.type == V4L2_EVENT_EOS)
                return false;
        }
    }

    // GOT EOS from encoder. Stop dqthread.
    if (buffer->planes[0].bytesused == 0)
    {
        cout << "Got 0 size buffer in capture \n";
        return false;
    }

    // Computing CRC with each frame
    if(ctx->pBitStreamCrc)
        CalculateCrc (ctx->pBitStreamCrc, buffer->planes[0].data, buffer->planes[0].bytesused);

    write_encoder_output_frame(ctx->out_file, buffer);
    num_encoded_frames++;

    // Accounting for the first frame as it is only sps+pps
    if (ctx->gdr_out_frame_number != 0xFFFFFFFF)
        if ( (ctx->enableGDR) && (ctx->GDR_out_file_path) && (num_encoded_frames >= ctx->gdr_out_frame_number+1))
            write_encoder_output_frame(ctx->gdr_out_file, buffer);

    if (ctx->report_metadata)
    {
        v4l2_ctrl_videoenc_outputbuf_metadata enc_metadata;
        if (ctx->enc->getMetadata(v4l2_buf->index, enc_metadata) == 0)
        {
            if (ctx->bReconCrc && enc_metadata.bValidReconCRC) {
                // CRC for Recon frame
                cout << "Frame: " << frame_num << endl;
                cout << "ReconFrame_Y_CRC " << enc_metadata.ReconFrame_Y_CRC <<
                    " ReconFrame_U_CRC " << enc_metadata.ReconFrame_U_CRC <<
                    " ReconFrame_V_CRC " << enc_metadata.ReconFrame_V_CRC <<
                    endl;

                if (!ctx->recon_Ref_file->eof()) {
                    *ctx->recon_Ref_file >> ReconRef_Y_CRC;
                    *ctx->recon_Ref_file >> ReconRef_U_CRC;
                    *ctx->recon_Ref_file >> ReconRef_V_CRC;
                }

                if ((ReconRef_Y_CRC != enc_metadata.ReconFrame_Y_CRC) ||
                    (ReconRef_U_CRC != enc_metadata.ReconFrame_U_CRC) ||
                    (ReconRef_V_CRC != enc_metadata.ReconFrame_V_CRC))
                {
                    cout << "Recon CRC FAIL" << endl;
                    cout << "ReconRef_Y_CRC " << ReconRef_Y_CRC <<
                        " ReconRef_U_CRC " << ReconRef_U_CRC <<
                        " ReconRef_V_CRC " << ReconRef_V_CRC <<
                        endl;
                    abort(ctx);
                    return false;
                }
                cout << "Recon CRC PASS for frame : " << frame_num << endl;
            } else if (ctx->externalRPS && enc_metadata.bRPSFeedback_status) {
                // RPS Feedback
                cout << "Frame: " << frame_num << endl;
                cout << "nCurrentRefFrameId " << enc_metadata.nCurrentRefFrameId <<
                     " nActiveRefFrames " << enc_metadata.nActiveRefFrames << endl;

                for (uint32_t i = 0; i < enc_metadata.nActiveRefFrames; i++)
                {
                    cout << "FrameId " << enc_metadata.RPSList[i].nFrameId <<
                     " IdrFrame " <<  (int) enc_metadata.RPSList[i].bIdrFrame <<
                     " LTRefFrame " <<  (int) enc_metadata.RPSList[i].bLTRefFrame <<
                     " PictureOrderCnt " << enc_metadata.RPSList[i].nPictureOrderCnt <<
                     " FrameNum " << enc_metadata.RPSList[i].nFrameNum <<
                     " LTFrameIdx " <<  enc_metadata.RPSList[i].nLTRFrameIdx << endl;
                }
            } else if (ctx->externalRCHints) {
                // Rate Control Feedback
                cout << "Frame: " << frame_num << endl;
                cout << "EncodedBits " << enc_metadata.EncodedFrameBits <<
                    " MinQP " << enc_metadata.FrameMinQP <<
                    " MaxQP " << enc_metadata.FrameMaxQP <<
                    endl;
            } else {
                cout << "Frame " << frame_num <<
                    ": isKeyFrame=" << (int) enc_metadata.KeyFrame <<
                    " AvgQP=" << enc_metadata.AvgQP <<
                    " MinQP=" << enc_metadata.FrameMinQP <<
                    " MaxQP=" << enc_metadata.FrameMaxQP <<
                    " EncodedBits=" << enc_metadata.EncodedFrameBits <<
                    endl;
            }
        }
    }
    if (ctx->dump_mv)
    {
        v4l2_ctrl_videoenc_outputbuf_metadata_MV enc_mv_metadata;
        if (ctx->enc->getMotionVectors(v4l2_buf->index, enc_mv_metadata) == 0)
        {
            uint32_t numMVs = enc_mv_metadata.bufSize / sizeof(MVInfo);
            MVInfo *pInfo = enc_mv_metadata.pMVInfo;

            cout << "Frame " << frame_num << ": Num MVs=" << numMVs << endl;

            for (uint32_t i = 0; i < numMVs; i++, pInfo++)
            {
                cout << i << ": mv_x=" << pInfo->mv_x <<
                    " mv_y=" << pInfo->mv_y <<
                    " weight=" << pInfo->weight <<
                    endl;
            }
        }
    }

    if (enc->capture_plane.qBuffer(*v4l2_buf, NULL) < 0)
    {
        cerr << "Error while Qing buffer at capture plane" << endl;
        abort(ctx);
        return false;
    }

    return true;
}

static void *encoder_pollthread_fcn(void *arg)
{

    enc_context_t *ctx = (enc_context_t *) arg;
    v4l2_ctrl_video_device_poll devicepoll;

    cout << "Starting Device Poll Thread " << endl;

    memset(&devicepoll, 0, sizeof(v4l2_ctrl_video_device_poll));

    // wait here until you are signalled to issue the Poll call.
    // Check if the abort status is set , if so exit
    // Else issue the Poll on the decoder and block.
    // When the Poll returns, signal the encoder thread to continue.

    while (!ctx->got_error && !ctx->enc->isInError())
    {
        sem_wait(&ctx->pollthread_sema);

        if (ctx->got_eos)
        {
            cout << "Got eos, exiting poll thread \n";
            return NULL;
        }

        devicepoll.req_events = POLLIN | POLLOUT | POLLERR | POLLPRI;

        // This call shall wait in the v4l2 encoder library
        ctx->enc->DevicePoll(&devicepoll);

        // We can check the devicepoll.resp_events bitmask to see which events are set.
        sem_post(&ctx->encoderthread_sema);
    }
    return NULL;
}

static int
set_runtime_params(enc_context_t *ctx)
{
    char charval;
    uint32_t intval;
    int ret, next;

    cout << "Frame " << ctx->next_param_change_frame <<
        ": Changing parameters" << endl;
    while (!ctx->runtime_params_str->eof())
    {
        next = get_next_parsed_pair(ctx, &charval, &intval);
        TEST_PARSE_ERROR(next < 0, err);
        switch (charval)
        {
            case 'b':
                if (ctx->ratecontrol == V4L2_MPEG_VIDEO_BITRATE_MODE_VBR &&
                    ctx->peak_bitrate < intval) {
                    uint32_t peak_bitrate = 1.2f * intval;
                    cout << "Peak bitrate = " << peak_bitrate << endl;
                    ret = ctx->enc->setPeakBitrate(peak_bitrate);
                    if (ret < 0)
                    {
                        cerr << "Could not set encoder peakbitrate" << endl;
                        goto err;
                    }
                }
                cout << "Bitrate = " << intval << endl;
                ret = ctx->enc->setBitrate(intval);
                if (ret < 0)
                {
                    cerr << "Could not set encoder bitrate" << endl;
                    goto err;
                }
                break;
            case 'p':
                cout << "Peak bitrate = " << intval << endl;
                ret = ctx->enc->setPeakBitrate(intval);
                if (ret < 0)
                {
                    cerr << "Could not set encoder peakbitrate" << endl;
                    goto err;
                }
                break;
            case 'r':
            {
                int fps_num = intval;
                TEST_PARSE_ERROR(next != '/', err);

                ctx->runtime_params_str->seekg(-1, ios::cur);
                next = get_next_parsed_pair(ctx, &charval, &intval);
                TEST_PARSE_ERROR(next < 0, err);

                cout << "Framerate = " << fps_num << "/"  << intval << endl;

                ret = ctx->enc->setFrameRate(fps_num, intval);
                if (ret < 0)
                {
                    cerr << "Could not set framerate" << endl;
                    goto err;
                }
                break;
            }
            case 'i':
                if (intval > 0)
                {
                    ctx->enc->forceIDR();
                    cout << "Forcing IDR" << endl;
                }
                break;
            default:
                TEST_PARSE_ERROR(true, err);
        }
        switch (next)
        {
            case 0:
                delete ctx->runtime_params_str;
                ctx->runtime_params_str = NULL;
                return 0;
            case '#':
                return 0;
            case ',':
                break;
            default:
                break;
        }
    }
    return 0;
err:
    cerr << "Skipping further runtime parameter changes" <<endl;
    delete ctx->runtime_params_str;
    ctx->runtime_params_str = NULL;
    return -1;
}


static void
populate_roi_Param(std::ifstream * stream, v4l2_enc_frame_ROI_params *VEnc_ROI_params)
{
    unsigned int ROIIndex = 0;

    if (!stream->eof()) {
        *stream >> VEnc_ROI_params->num_ROI_regions;
        while (ROIIndex < VEnc_ROI_params->num_ROI_regions)
        {
            if (ROIIndex == V4L2_MAX_ROI_REGIONS) {
                string skip_str;
                getline(*stream, skip_str);

                VEnc_ROI_params->num_ROI_regions = V4L2_MAX_ROI_REGIONS;

                cout << "Maximum of " << V4L2_MAX_ROI_REGIONS <<
                        "regions can be applied for a frame" << endl;
                break;
            }

            *stream >> VEnc_ROI_params->ROI_params[ROIIndex].QPdelta;
            *stream >> VEnc_ROI_params->ROI_params[ROIIndex].ROIRect.left;
            *stream >> VEnc_ROI_params->ROI_params[ROIIndex].ROIRect.top;
            *stream >> VEnc_ROI_params->ROI_params[ROIIndex].ROIRect.width;
            *stream >> VEnc_ROI_params->ROI_params[ROIIndex].ROIRect.height;
            ROIIndex++;
        }
    } else {
        cout << "EOF of ROI_param_file & rewind" << endl;
        stream->clear();
        stream->seekg(0);
    }
}

static void
populate_ext_rps_ctrl_Param (std::ifstream * stream, v4l2_enc_frame_ext_rps_ctrl_params *VEnc_ext_rps_ctrl_params)
{
    unsigned int RPSIndex = 0;
    unsigned int temp = 0;

    stream->peek();
restart :
    if (stream->eof()) {
        cout << "EOF of rps_param_file & rewind" << endl;
        stream->clear();
        stream->seekg(0);
    }
    if (!stream->eof()) {
        *stream >> VEnc_ext_rps_ctrl_params->nFrameId;
        if (stream->eof())
            goto restart;
        *stream >> temp;
        VEnc_ext_rps_ctrl_params->bRefFrame = ((temp)?true:false);
        *stream >> temp;
        VEnc_ext_rps_ctrl_params->bLTRefFrame = ((temp)?true:false);
        *stream >> VEnc_ext_rps_ctrl_params->nMaxRefFrames;
        *stream >> VEnc_ext_rps_ctrl_params->nActiveRefFrames;
        *stream >> VEnc_ext_rps_ctrl_params->nCurrentRefFrameId;
        while (RPSIndex < VEnc_ext_rps_ctrl_params->nActiveRefFrames)
        {
            if (RPSIndex == V4L2_MAX_REF_FRAMES) {
                string skip_str;
                getline(*stream, skip_str);

                VEnc_ext_rps_ctrl_params->nActiveRefFrames = V4L2_MAX_REF_FRAMES;

                cout << "Maximum of " << V4L2_MAX_REF_FRAMES <<
                        "reference frames are valid" << endl;
                break;
            }

            *stream >> VEnc_ext_rps_ctrl_params->RPSList[RPSIndex].nFrameId;
            *stream >> temp;
            VEnc_ext_rps_ctrl_params->RPSList[RPSIndex].bLTRefFrame = ((temp)?true:false);
            RPSIndex++;
        }
    }
}


static void
populate_gdr_Param(std::ifstream * stream, uint32_t *start_frame_num, uint32_t *gdr_num_frames)
{
    if (stream->eof()) {
        *start_frame_num = 0xFFFFFFFF;
        cout << "GDR param EoF reached \n";
    }
    if (!stream->eof()) {
        *stream >> *start_frame_num;
        *stream >> *gdr_num_frames;
    }
}

static void
populate_ext_rate_ctrl_Param(std::ifstream * stream, v4l2_enc_frame_ext_rate_ctrl_params *VEnc_ext_rate_ctrl_params)
{
    stream->peek();
restart:
    if (stream->eof()) {
        cout << "EOF of hints_param_file & rewind" << endl;
        stream->clear();
        stream->seekg(0);
    }
    if (!stream->eof()) {
        *stream >> VEnc_ext_rate_ctrl_params->nTargetFrameBits;
        if (stream->eof())
            goto restart;
        *stream >> VEnc_ext_rate_ctrl_params->nFrameQP;
        *stream >> VEnc_ext_rate_ctrl_params->nFrameMinQp;
        *stream >> VEnc_ext_rate_ctrl_params->nFrameMaxQp;
        *stream >> VEnc_ext_rate_ctrl_params->nMaxQPDeviation;
    }
}

static CUresult
cuda_copy(int src_fd, int dst_fd, CUstream stream) {
  CUresult ret;

  EGLImageKHR src_image = NvEGLImageFromFd(NULL, src_fd);
  if (src_image == NULL)
    return CUDA_ERROR_ASSERT;
  EGLImageKHR dst_image = NvEGLImageFromFd(NULL, dst_fd);
  if (dst_image == NULL)
    return CUDA_ERROR_ASSERT;  

  CUgraphicsResource src_res, dst_res;  
  CUeglFrame src_frame, dst_frame;

  ret = cuGraphicsEGLRegisterImage(&src_res, src_image, CU_GRAPHICS_REGISTER_FLAGS_READ_ONLY);
  if (ret != CUDA_SUCCESS)
    return ret;

  ret = cuGraphicsEGLRegisterImage(&dst_res, dst_image, CU_GRAPHICS_REGISTER_FLAGS_WRITE_DISCARD);
  if (ret != CUDA_SUCCESS)
    return ret;
  
  ret = cuGraphicsResourceGetMappedEglFrame(&src_frame, src_res, 0, 0);
  if (ret != CUDA_SUCCESS)
    return ret;
  
  ret = cuGraphicsResourceGetMappedEglFrame(&dst_frame, dst_res, 0, 0);
  if (ret != CUDA_SUCCESS)
    return ret;

  CUDA_MEMCPY2D cpy = {0};
  cpy.srcDevice = (CUdeviceptr)src_frame.frame.pPitch[0];
  cpy.srcPitch = src_frame.pitch;
  cpy.srcMemoryType = CU_MEMORYTYPE_DEVICE;
  cpy.dstDevice = (CUdeviceptr)dst_frame.frame.pPitch[0];
  cpy.dstPitch = dst_frame.pitch;
  cpy.dstMemoryType = CU_MEMORYTYPE_DEVICE;
  cpy.WidthInBytes = src_frame.width;
  cpy.Height = src_frame.height; 

  ret = cuMemcpy2DAsync(&cpy, stream);
  if (ret != CUDA_SUCCESS)
    return ret;
  
  // NV12
  cpy.srcDevice = (CUdeviceptr)src_frame.frame.pPitch[1];
  cpy.dstDevice = (CUdeviceptr)dst_frame.frame.pPitch[1];
  cpy.Height = src_frame.height / 2;   
  ret = cuMemcpy2DAsync(&cpy, stream);
  if (ret != CUDA_SUCCESS)
      return ret;

  ret = cuStreamSynchronize(stream);
  if (ret != CUDA_SUCCESS)
    return ret;

  ret = cuCtxSynchronize();
  if (ret != CUDA_SUCCESS)
    return ret;

  ret = cuGraphicsUnregisterResource(src_res);
  if (ret != CUDA_SUCCESS)
    return ret;

  ret = cuGraphicsUnregisterResource(dst_res);
  if (ret != CUDA_SUCCESS)
    return ret;

  NvDestroyEGLImage(NULL, src_image);
  NvDestroyEGLImage(NULL, dst_image);
}




static int encoder_proc_blocking(tc_context_t *tc, bool eos)
{
    int ret = 0;
    // Keep reading input till EOS is reached
    enc_context_t *ctx = &tc->enc_ctx;
    cout << "encoder_proc_blocking" << endl;
    while (!ctx->got_error && !ctx->enc->isInError() && !eos)
    {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];
        NvBuffer *buffer;

        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, sizeof(planes));

        v4l2_buf.m.planes = planes;

        if (ctx->enc->output_plane.dqBuffer(v4l2_buf, &buffer, NULL, 10) < 0)
        {
            cerr << "ERROR while DQing buffer at output plane" << endl;
            abort(ctx);
            goto cleanup;
        }
        if (ctx->runtime_params_str &&
            (ctx->enc->output_plane.getTotalQueuedBuffers() ==
                ctx->next_param_change_frame))
        {
            set_runtime_params(ctx);
            if (ctx->runtime_params_str)
                get_next_runtime_param_change_frame(ctx);
        }

        auto v4l2_dec_buf = tc->buffers.front();
        int dec_fd = tc->dec_ctx.dmabuff_fd[v4l2_dec_buf.index];

        if (ctx->use_cuda) {
            CUresult cu_ret = cuda_copy(dec_fd, ctx->output_plane_fd[v4l2_buf.index], ctx->cu_stream);
            if (cu_ret != CUDA_SUCCESS)
            {
                cerr << "Error while cuda copy" << endl;
                abort(ctx);
                goto cleanup;
            }
        } else {
            NvBufferRect src_rect, dest_rect;
            src_rect.top = 0;
            src_rect.left = 0;
            src_rect.width = tc->dec_ctx.display_width;
            src_rect.height = tc->dec_ctx.display_height;
            dest_rect.top = 0;
            dest_rect.left = 0;
            dest_rect.width = ctx->width;
            dest_rect.height = ctx->height;

            NvBufferTransformParams transform_params;
            memset(&transform_params,0,sizeof(transform_params));
            /* Indicates which of the transform parameters are valid */
            transform_params.transform_flag = NVBUFFER_TRANSFORM_FILTER;
            transform_params.transform_flip = NvBufferTransform_None;
            transform_params.transform_filter = NvBufferTransform_Filter_Smart;
            transform_params.src_rect = src_rect;
            transform_params.dst_rect = dest_rect;
            if (ctx->use_transform_async) {
                NvBufferSyncObj obj;
                memset(&obj, 0, sizeof(NvBufferSyncObj));
                obj.use_outsyncobj = 1;
                ret = NvBufferTransformAsync(dec_fd, ctx->output_plane_fd[v4l2_buf.index], &transform_params, &obj);
                if (ret != 0) {
                    ret = NvBufferSyncObjWait(&obj.outsyncobj, 0xffff);
                }
            } else {
                ret = NvBufferTransform(dec_fd, ctx->output_plane_fd[v4l2_buf.index], &transform_params);
            }
            

            if (ret != 0) {
                cerr << "Error transform" << endl;
                abort(ctx);
                goto cleanup;
            }

        }



        //cout << "NvBufferTransform1 " << dec_fd << " " << buffer->planes[0].fd << " " << ret << endl;

        tc->buffers.pop_front();
        tc->release_buffers.push_back(v4l2_dec_buf);

        // TODO eos

        if (ctx->input_metadata)
        {
            v4l2_ctrl_videoenc_input_metadata VEnc_imeta_param;
            v4l2_enc_frame_ROI_params VEnc_ROI_params;
            v4l2_enc_frame_ReconCRC_params VEnc_ReconCRC_params;
            v4l2_enc_frame_ext_rps_ctrl_params VEnc_ext_rps_ctrl_params;
            v4l2_enc_frame_ext_rate_ctrl_params VEnc_ext_rate_ctrl_params;
            v4l2_enc_gdr_params VEnc_gdr_params;
            VEnc_imeta_param.flag = 0;

            if (ctx->ROI_Param_file_path)
            {
                if (ctx->enableROI) {
                    VEnc_imeta_param.flag |= V4L2_ENC_INPUT_ROI_PARAM_FLAG;
                    VEnc_imeta_param.VideoEncROIParams = &VEnc_ROI_params;
                    populate_roi_Param(ctx->roi_Param_file, VEnc_imeta_param.VideoEncROIParams);
                }
            }

            if (ctx->bReconCrc)
            {
                VEnc_imeta_param.flag |= V4L2_ENC_INPUT_RECONCRC_PARAM_FLAG;

                VEnc_ReconCRC_params.ReconCRCRect.left = ctx->rl;
                VEnc_ReconCRC_params.ReconCRCRect.top = ctx->rt;
                VEnc_ReconCRC_params.ReconCRCRect.width = ctx->rw;
                VEnc_ReconCRC_params.ReconCRCRect.height = ctx->rh;

                VEnc_imeta_param.VideoReconCRCParams = &VEnc_ReconCRC_params;
            }

            if (ctx->RPS_Param_file_path)
            {
                if (ctx->externalRPS) {
                    VEnc_imeta_param.flag |= V4L2_ENC_INPUT_RPS_PARAM_FLAG;
                    VEnc_imeta_param.VideoEncRPSParams = &VEnc_ext_rps_ctrl_params;
                    populate_ext_rps_ctrl_Param(ctx->rps_Param_file, VEnc_imeta_param.VideoEncRPSParams);
                }
            }

            if (ctx->GDR_Param_file_path)
            {
                if (ctx->enableGDR)
                {
                    if (ctx->gdr_start_frame_number == 0xFFFFFFFF)
                        populate_gdr_Param(ctx->gdr_Param_file, &ctx->gdr_start_frame_number,
                                    &ctx->gdr_num_frames);
                    if (ctx->input_frames_queued_count == ctx->gdr_start_frame_number)
                    {
                        ctx->gdr_out_frame_number = ctx->gdr_start_frame_number;
                        VEnc_gdr_params.nGDRFrames = ctx->gdr_num_frames;
                        VEnc_imeta_param.flag |= V4L2_ENC_INPUT_GDR_PARAM_FLAG;
                        VEnc_imeta_param.VideoEncGDRParams = &VEnc_gdr_params;
                    }
                }
            }

            if (ctx->hints_Param_file_path)
            {
                if (ctx->externalRCHints) {
                    VEnc_imeta_param.flag |= V4L2_ENC_INPUT_RC_PARAM_FLAG;
                    VEnc_imeta_param.VideoEncExtRCParams = &VEnc_ext_rate_ctrl_params;

                    populate_ext_rate_ctrl_Param(ctx->hints_Param_file, VEnc_imeta_param.VideoEncExtRCParams);

                }
            }

            if (VEnc_imeta_param.flag)
            {
                ctx->enc->SetInputMetaParams(v4l2_buf.index, VEnc_imeta_param);
                v4l2_buf.reserved2 = v4l2_buf.index;
            }
        }

        if (ctx->copy_timestamp)
        {
          v4l2_buf.flags |= V4L2_BUF_FLAG_TIMESTAMP_COPY;
          ctx->timestamp += ctx->timestampincr;
          v4l2_buf.timestamp.tv_sec = ctx->timestamp / (MICROSECOND_UNIT);
          v4l2_buf.timestamp.tv_usec = ctx->timestamp % (MICROSECOND_UNIT);
        }

        NvBufferParams params; 
        NvBufferGetParams(ctx->output_plane_fd[v4l2_buf.index], &params);
        for (uint32_t j = 0 ; j < buffer->n_planes ; j++)
        {
            v4l2_buf.m.planes[j].bytesused = params.psize[j];
            v4l2_buf.m.planes[j].m.fd = ctx->output_plane_fd[v4l2_buf.index];
        }
        ret = ctx->enc->output_plane.qBuffer(v4l2_buf, NULL);
        if (ret < 0)
        {
            cerr << "Error while queueing buffer at output plane" << endl;
            abort(ctx);
            goto cleanup;
        }

        ctx->input_frames_queued_count++;
        if (v4l2_buf.m.planes[0].bytesused == 0)
        {
            cerr << "File read complete." << endl;
            eos = true;
            ctx->got_eos = true;
            return 0;
        }
    }
cleanup:
    cout << "!encoder_proc_blocking" << endl;
    return -1;
}





static void *
encode_proc(void *arg)
{
    tc_context_t *tc = (tc_context_t *) arg;
    int argc = tc->argc;
    char **argv = tc->argv;
    enc_context_t *ctx = &tc->enc_ctx;

    int ret = 0;
    int error = 0;
    bool eos = false;

    enc_set_defaults(ctx);

    ret = enc_parse_csv_args(ctx, argc, argv);
    TEST_ERROR(ret < 0, "Error parsing commandline arguments", cleanup);

    pthread_setname_np(pthread_self(),"EncOutPlane");

    if (ctx->runtime_params_str)
    {
        get_next_runtime_param_change_frame(ctx);
    }

    if (ctx->encoder_pixfmt == V4L2_PIX_FMT_H265)
    {
        TEST_ERROR(ctx->width < 144 || ctx->height < 144, "Height/Width should be"
            " > 144 for H.265", cleanup);
    }

    if (ctx->use_gold_crc)
    {
        ctx->pBitStreamCrc = InitCrc(CRC32_POLYNOMIAL);
        TEST_ERROR(!ctx->pBitStreamCrc, "InitCrc failed", cleanup);
    }

    ctx->out_file = new ofstream(ctx->out_file_path);
    TEST_ERROR(!ctx->out_file->is_open(), "Could not open output file", cleanup);

    if (ctx->ROI_Param_file_path) {
        ctx->roi_Param_file = new ifstream(ctx->ROI_Param_file_path);
        TEST_ERROR(!ctx->roi_Param_file->is_open(), "Could not open roi param file", cleanup);
    }

    if (ctx->Recon_Ref_file_path) {
        ctx->recon_Ref_file = new ifstream(ctx->Recon_Ref_file_path);
        TEST_ERROR(!ctx->recon_Ref_file->is_open(), "Could not open recon crc reference file", cleanup);
    }

    if (ctx->RPS_Param_file_path) {
        ctx->rps_Param_file = new ifstream(ctx->RPS_Param_file_path);
        TEST_ERROR(!ctx->rps_Param_file->is_open(), "Could not open rps param file", cleanup);
    }

    if (ctx->GDR_Param_file_path) {
        ctx->gdr_Param_file = new ifstream(ctx->GDR_Param_file_path);
        TEST_ERROR(!ctx->gdr_Param_file->is_open(), "Could not open GDR param file", cleanup);
    }

    if (ctx->GDR_out_file_path) {
        ctx->gdr_out_file = new ofstream(ctx->GDR_out_file_path);
        TEST_ERROR(!ctx->gdr_out_file->is_open(), "Could not open GDR Out file", cleanup);
    }

    if (ctx->hints_Param_file_path) {
        ctx->hints_Param_file = new ifstream(ctx->hints_Param_file_path);
        TEST_ERROR(!ctx->hints_Param_file->is_open(), "Could not open hints param file", cleanup);
    }

    cout << "Creating Encoder in blocking mode \n";
    ctx->enc = NvVideoEncoder::createVideoEncoder("enc0");
    TEST_ERROR(!ctx->enc, "Could not create encoder", cleanup);

    // It is necessary that Capture Plane format be set before Output Plane
    // format.
    // Set encoder capture plane format. It is necessary to set width and
    // height on the capture plane as well
    ret =
        ctx->enc->setCapturePlaneFormat(ctx->encoder_pixfmt, ctx->width,
                                      ctx->height, 2 * 1024 * 1024);
    TEST_ERROR(ret < 0, "Could not set capture plane format", cleanup);

    // Set encoder output plane format
    switch (ctx->profile)
    {
        case V4L2_MPEG_VIDEO_H265_PROFILE_MAIN10:
            ctx->raw_pixfmt = V4L2_PIX_FMT_P010M;
            break;
        case V4L2_MPEG_VIDEO_H265_PROFILE_MAIN:
        default:
            ctx->raw_pixfmt = V4L2_PIX_FMT_NV12M;
    }
    if (ctx->enableLossless && ctx->encoder_pixfmt == V4L2_PIX_FMT_H264)
    {
        ctx->profile = V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_PREDICTIVE;
        ret =
            ctx->enc->setOutputPlaneFormat(V4L2_PIX_FMT_YUV444M, ctx->width,
                                      ctx->height);
    }
    else
    {
        ret =
            ctx->enc->setOutputPlaneFormat(ctx->raw_pixfmt, ctx->width,
                                      ctx->height);
    }
    TEST_ERROR(ret < 0, "Could not set output plane format", cleanup);

    ret = ctx->enc->setBitrate(ctx->bitrate);
    TEST_ERROR(ret < 0, "Could not set encoder bitrate", cleanup);

    if (ctx->encoder_pixfmt == V4L2_PIX_FMT_H264 || ctx->encoder_pixfmt == V4L2_PIX_FMT_H265)
    {
        ret = ctx->enc->setProfile(ctx->profile);
        TEST_ERROR(ret < 0, "Could not set encoder profile", cleanup);
    }
    if (ctx->encoder_pixfmt == V4L2_PIX_FMT_H264)
    {
        ret = ctx->enc->setLevel(ctx->level);
        TEST_ERROR(ret < 0, "Could not set encoder level", cleanup);
    }

    if (ctx->enableLossless)
    {
        ret = ctx->enc->setConstantQp(0);
        TEST_ERROR(ret < 0, "Could not set encoder constant qp=0", cleanup);
    }
    else
    {
        ret = ctx->enc->setRateControlMode(ctx->ratecontrol);
        TEST_ERROR(ret < 0, "Could not set encoder rate control mode", cleanup);
        if (ctx->ratecontrol == V4L2_MPEG_VIDEO_BITRATE_MODE_VBR) {
            uint32_t peak_bitrate;
            if (ctx->peak_bitrate < ctx->bitrate)
                peak_bitrate = 1.2f * ctx->bitrate;
            else
                peak_bitrate = ctx->peak_bitrate;
            ret = ctx->enc->setPeakBitrate(peak_bitrate);
            TEST_ERROR(ret < 0, "Could not set encoder peak bitrate", cleanup);
        }
    }

    ret = ctx->enc->setIDRInterval(ctx->idr_interval);
    TEST_ERROR(ret < 0, "Could not set encoder IDR interval", cleanup);

    ret = ctx->enc->setIFrameInterval(ctx->iframe_interval);
    TEST_ERROR(ret < 0, "Could not set encoder I-Frame interval", cleanup);

    ret = ctx->enc->setFrameRate(ctx->fps_n, ctx->fps_d);
    TEST_ERROR(ret < 0, "Could not set framerate", cleanup);

    if (ctx->temporal_tradeoff_level)
    {
        ret = ctx->enc->setTemporalTradeoff(ctx->temporal_tradeoff_level);
        TEST_ERROR(ret < 0, "Could not set temporal tradeoff level", cleanup);
    }

    if (ctx->slice_length)
    {
        ret = ctx->enc->setSliceLength(ctx->slice_length_type,
                ctx->slice_length);
        TEST_ERROR(ret < 0, "Could not set slice length params", cleanup);
    }

    if (ctx->hw_preset_type)
    {
        ret = ctx->enc->setHWPresetType(ctx->hw_preset_type);
        TEST_ERROR(ret < 0, "Could not set encoder HW Preset Type", cleanup);
    }

    if (ctx->virtual_buffer_size)
    {
        ret = ctx->enc->setVirtualBufferSize(ctx->virtual_buffer_size);
        TEST_ERROR(ret < 0, "Could not set virtual buffer size", cleanup);
    }

    if (ctx->num_reference_frames)
    {
        ret = ctx->enc->setNumReferenceFrames(ctx->num_reference_frames);
        TEST_ERROR(ret < 0, "Could not set num reference frames", cleanup);
    }

    if (ctx->slice_intrarefresh_interval)
    {
        ret = ctx->enc->setSliceIntrarefresh(ctx->slice_intrarefresh_interval);
        TEST_ERROR(ret < 0, "Could not set slice intrarefresh interval", cleanup);
    }

    if (ctx->insert_sps_pps_at_idr)
    {
        ret = ctx->enc->setInsertSpsPpsAtIdrEnabled(true);
        TEST_ERROR(ret < 0, "Could not set insertSPSPPSAtIDR", cleanup);
    }

    if (ctx->insert_vui)
    {
        ret = ctx->enc->setInsertVuiEnabled(true);
        TEST_ERROR(ret < 0, "Could not set insertVUI", cleanup);
    }

    if (ctx->enable_extended_colorformat)
    {
        ret = ctx->enc->setExtendedColorFormat(true);
        TEST_ERROR(ret < 0, "Could not set extended color format", cleanup);
    }

    if (ctx->insert_aud)
    {
        ret = ctx->enc->setInsertAudEnabled(true);
        TEST_ERROR(ret < 0, "Could not set insertAUD", cleanup);
    }

    if (ctx->alliframes)
    {
        ret = ctx->enc->setAlliFramesEncode(true);
        TEST_ERROR(ret < 0, "Could not set Alliframes encoding", cleanup);
    }

    if (ctx->num_b_frames != (uint32_t) -1)
    {
        ret = ctx->enc->setNumBFrames(ctx->num_b_frames);
        TEST_ERROR(ret < 0, "Could not set number of B Frames", cleanup);
    }

    if ((ctx->nMinQpI != (uint32_t)QP_RETAIN_VAL) ||
        (ctx->nMaxQpI != (uint32_t)QP_RETAIN_VAL) ||
        (ctx->nMinQpP != (uint32_t)QP_RETAIN_VAL) ||
        (ctx->nMaxQpP != (uint32_t)QP_RETAIN_VAL) ||
        (ctx->nMinQpB != (uint32_t)QP_RETAIN_VAL) ||
        (ctx->nMaxQpB != (uint32_t)QP_RETAIN_VAL))
    {
        ret = ctx->enc->setQpRange(ctx->nMinQpI, ctx->nMaxQpI, ctx->nMinQpP,
                ctx->nMaxQpP, ctx->nMinQpB, ctx->nMaxQpB);
        TEST_ERROR(ret < 0, "Could not set quantization parameters", cleanup);
    }

    if (ctx->max_perf)
    {
        ret = ctx->enc->setMaxPerfMode(ctx->max_perf);
        TEST_ERROR(ret < 0, "Error while setting encoder to max perf", cleanup);
    }

    if (ctx->dump_mv)
    {
        ret = ctx->enc->enableMotionVectorReporting();
        TEST_ERROR(ret < 0, "Could not enable motion vector reporting", cleanup);
    }

    if (ctx->bnoIframe) {
        ctx->iframe_interval = ((1<<31) + 1); /* TODO: how can we do this properly */
        ret = ctx->enc->setIFrameInterval(ctx->iframe_interval);
        TEST_ERROR(ret < 0, "Could not set encoder I-Frame interval", cleanup);
    }

    if (ctx->enableROI) {
        v4l2_enc_enable_roi_param VEnc_enable_ext_roi_ctrl;

        VEnc_enable_ext_roi_ctrl.bEnableROI = ctx->enableROI;
        ret = ctx->enc->enableROI(VEnc_enable_ext_roi_ctrl);
        TEST_ERROR(ret < 0, "Could not enable ROI", cleanup);
    }

    if (ctx->bReconCrc) {
        v4l2_enc_enable_reconcrc_param VEnc_enable_recon_crc_ctrl;

        VEnc_enable_recon_crc_ctrl.bEnableReconCRC = ctx->bReconCrc;
        ret = ctx->enc->enableReconCRC(VEnc_enable_recon_crc_ctrl);
        TEST_ERROR(ret < 0, "Could not enable Recon CRC", cleanup);
    }

    if (ctx->externalRPS) {
        v4l2_enc_enable_ext_rps_ctr VEnc_enable_ext_rps_ctrl;

        VEnc_enable_ext_rps_ctrl.bEnableExternalRPS = ctx->externalRPS;
        if (ctx->encoder_pixfmt == V4L2_PIX_FMT_H264) {
            VEnc_enable_ext_rps_ctrl.bGapsInFrameNumAllowed = ctx->bGapsInFrameNumAllowed;
            VEnc_enable_ext_rps_ctrl.nH264FrameNumBits = ctx->nH264FrameNumBits;
        }
        if (ctx->encoder_pixfmt == V4L2_PIX_FMT_H265) {
            VEnc_enable_ext_rps_ctrl.nH265PocLsbBits = ctx->nH265PocLsbBits;
        }
        ret = ctx->enc->enableExternalRPS(VEnc_enable_ext_rps_ctrl);
        TEST_ERROR(ret < 0, "Could not enable external RPS", cleanup);
    }

    if (ctx->externalRCHints) {
        v4l2_enc_enable_ext_rate_ctr VEnc_enable_ext_rate_ctrl;

        VEnc_enable_ext_rate_ctrl.bEnableExternalPictureRC = ctx->externalRCHints;
        VEnc_enable_ext_rate_ctrl.nsessionMaxQP = ctx->sMaxQp;

        ret = ctx->enc->enableExternalRC(VEnc_enable_ext_rate_ctrl);
        TEST_ERROR(ret < 0, "Could not enable external RC", cleanup);
    }

    if (ctx->use_cuda) {
        CUresult cu_ret;

        cu_ret = cuInit(0);
        TEST_ERROR(cu_ret != CUDA_SUCCESS, "cuInit", cleanup);

        cu_ret = cuDeviceGet(&ctx->cu_dev, 0);
        TEST_ERROR(cu_ret != CUDA_SUCCESS, "cuDeviceGet", cleanup);

        cu_ret = cuCtxCreate(&ctx->cu_ctx, CU_CTX_SCHED_AUTO, ctx->cu_dev);
        TEST_ERROR(cu_ret != CUDA_SUCCESS, "cuCtxCreate", cleanup);

        cu_ret = cuStreamCreate(&ctx->cu_stream, CU_STREAM_NON_BLOCKING);
        TEST_ERROR(cu_ret != CUDA_SUCCESS, "cuStreamCreate", cleanup);
    }


    // Query, Export and Map the output plane buffers so that we can read
    // raw data into the buffers
    
    ret = setup_output_dmabuf(ctx,10);
    TEST_ERROR(ret < 0, "Could not setup plane", cleanup);

    ret = ctx->enc->capture_plane.setupPlane(V4L2_MEMORY_MMAP, 6, true, false);
    TEST_ERROR(ret < 0, "Could not setup capture plane", cleanup);

    ret = ctx->enc->subscribeEvent(V4L2_EVENT_EOS,0,0);
    TEST_ERROR(ret < 0, "Could not subscribe EOS event", cleanup);

    if (ctx->b_use_enc_cmd)
    {
        ret = ctx->enc->setEncoderCommand(V4L2_ENC_CMD_START, 0);
        TEST_ERROR(ret < 0, "Error in start of encoder commands ", cleanup);
    }
    else
    {
        // output plane STREAMON
        ret = ctx->enc->output_plane.setStreamStatus(true);
        TEST_ERROR(ret < 0, "Error in output plane streamon", cleanup);

        // capture plane STREAMON
        ret = ctx->enc->capture_plane.setStreamStatus(true);
        TEST_ERROR(ret < 0, "Error in capture plane streamon", cleanup);
    }


    ctx->enc->capture_plane.
        setDQThreadCallback(encoder_capture_plane_dq_callback);

    // startDQThread starts a thread internally which calls the
    // encoder_capture_plane_dq_callback whenever a buffer is dequeued
    // on the plane
    ctx->enc->capture_plane.startDQThread(ctx);

    // Enqueue all the empty capture plane buffers
    for (uint32_t i = 0; i < ctx->enc->capture_plane.getNumBuffers(); i++)
    {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];

        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, MAX_PLANES * sizeof(struct v4l2_plane));

        v4l2_buf.index = i;
        v4l2_buf.m.planes = planes;

        ret = ctx->enc->capture_plane.qBuffer(v4l2_buf, NULL);
        if (ret < 0)
        {
            cerr << "Error while queueing buffer at capture plane" << endl;
            abort(ctx);
            goto cleanup;
        }
    }

    if (ctx->copy_timestamp) {
      ctx->timestamp = (ctx->start_ts * MICROSECOND_UNIT);
      ctx->timestampincr = (MICROSECOND_UNIT * 16) / ((uint32_t) (ctx->fps_n * 16));
    }

    // Read video frame and queue all the output plane buffers
    for (uint32_t i = 0; i < ctx->enc->output_plane.getNumBuffers(); i++)
    {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];
        NvBuffer *buffer = ctx->enc->output_plane.getNthBuffer(i);

        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, MAX_PLANES * sizeof(struct v4l2_plane));

        v4l2_buf.index = i;
        v4l2_buf.m.planes = planes;
        v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        v4l2_buf.memory = V4L2_MEMORY_DMABUF;


        auto v4l2_dec_buf = tc->buffers.front();
        int dec_fd = tc->dec_ctx.dmabuff_fd[v4l2_dec_buf.index];

        if (ctx->use_cuda) {
            CUresult cu_ret = cuda_copy(dec_fd, ctx->output_plane_fd[i], ctx->cu_stream);
            if (cu_ret != CUDA_SUCCESS) {
              cerr << "Error while cuda copy" << endl;
              abort(ctx);
              goto cleanup;
            }
        } else {
            NvBufferRect src_rect, dest_rect;
            src_rect.top = 0;
            src_rect.left = 0;
            src_rect.width = tc->dec_ctx.display_width;
            src_rect.height = tc->dec_ctx.display_height;
            dest_rect.top = 0;
            dest_rect.left = 0;
            dest_rect.width = ctx->width;
            dest_rect.height = ctx->height;

            NvBufferTransformParams transform_params;
            memset(&transform_params,0,sizeof(transform_params));
            /* Indicates which of the transform parameters are valid */
            transform_params.transform_flag = NVBUFFER_TRANSFORM_FILTER;
            transform_params.transform_flip = NvBufferTransform_None;
            transform_params.transform_filter = NvBufferTransform_Filter_Smart;
            transform_params.src_rect = src_rect;
            transform_params.dst_rect = dest_rect;

            if (ctx->use_transform_async) {
                NvBufferSyncObj obj;
                memset(&obj, 0, sizeof(NvBufferSyncObj));
                obj.use_outsyncobj = 1;
                ret = NvBufferTransformAsync(dec_fd, ctx->output_plane_fd[v4l2_buf.index], &transform_params, &obj);
                if (ret != 0) {
                    ret = NvBufferSyncObjWait(&obj.outsyncobj, 0);
                }
            } else {
                ret = NvBufferTransform(dec_fd, ctx->output_plane_fd[i], &transform_params);
            }

            if (ret != 0) {
              cerr << "Error transform" << endl;
              abort(ctx);
              goto cleanup;
            }
        }

        tc->buffers.pop_front();
        tc->release_buffers.push_back(v4l2_dec_buf);

        // TODO cmdstop !


        if (ctx->runtime_params_str &&
            (ctx->enc->output_plane.getTotalQueuedBuffers() ==
                ctx->next_param_change_frame))
        {
            set_runtime_params(ctx);
            if (ctx->runtime_params_str)
                get_next_runtime_param_change_frame(ctx);
        }

        if (ctx->input_metadata)
        {
            v4l2_ctrl_videoenc_input_metadata VEnc_imeta_param;
            v4l2_enc_frame_ROI_params VEnc_ROI_params;
            v4l2_enc_frame_ReconCRC_params VEnc_ReconCRC_params;
            v4l2_enc_frame_ext_rps_ctrl_params VEnc_ext_rps_ctrl_params;
            v4l2_enc_frame_ext_rate_ctrl_params VEnc_ext_rate_ctrl_params;
            v4l2_enc_gdr_params VEnc_gdr_params;
            VEnc_imeta_param.flag = 0;

            if (ctx->ROI_Param_file_path)
            {
                if (ctx->enableROI) {
                    VEnc_imeta_param.flag |= V4L2_ENC_INPUT_ROI_PARAM_FLAG;
                    VEnc_imeta_param.VideoEncROIParams = &VEnc_ROI_params;

                    populate_roi_Param(ctx->roi_Param_file, VEnc_imeta_param.VideoEncROIParams);
                }
            }

            if (ctx->bReconCrc)
            {
                VEnc_imeta_param.flag |= V4L2_ENC_INPUT_RECONCRC_PARAM_FLAG;

                VEnc_ReconCRC_params.ReconCRCRect.left = ctx->rl;
                VEnc_ReconCRC_params.ReconCRCRect.top = ctx->rt;
                VEnc_ReconCRC_params.ReconCRCRect.width = ctx->rw;
                VEnc_ReconCRC_params.ReconCRCRect.height = ctx->rh;

                VEnc_imeta_param.VideoReconCRCParams = &VEnc_ReconCRC_params;
            }

            if (ctx->RPS_Param_file_path)
            {
                if (ctx->externalRPS) {
                    VEnc_imeta_param.flag |= V4L2_ENC_INPUT_RPS_PARAM_FLAG;
                    VEnc_imeta_param.VideoEncRPSParams = &VEnc_ext_rps_ctrl_params;

                    populate_ext_rps_ctrl_Param(ctx->rps_Param_file, VEnc_imeta_param.VideoEncRPSParams);
                }
            }

            if (ctx->GDR_Param_file_path)
            {
                if (ctx->enableGDR)
                {
                    if (ctx->gdr_start_frame_number == 0xFFFFFFFF)
                        populate_gdr_Param(ctx->gdr_Param_file, &ctx->gdr_start_frame_number,
                                    &ctx->gdr_num_frames);
                    if (ctx->input_frames_queued_count == ctx->gdr_start_frame_number)
                    {
                        ctx->gdr_out_frame_number = ctx->gdr_start_frame_number;
                        VEnc_gdr_params.nGDRFrames = ctx->gdr_num_frames;
                        VEnc_imeta_param.flag |= V4L2_ENC_INPUT_GDR_PARAM_FLAG;
                        VEnc_imeta_param.VideoEncGDRParams = &VEnc_gdr_params;
                    }
                }
            }

            if (ctx->hints_Param_file_path)
            {
                if (ctx->externalRCHints) {
                    VEnc_imeta_param.flag |= V4L2_ENC_INPUT_RC_PARAM_FLAG;
                    VEnc_imeta_param.VideoEncExtRCParams = &VEnc_ext_rate_ctrl_params;

                    populate_ext_rate_ctrl_Param(ctx->hints_Param_file, VEnc_imeta_param.VideoEncExtRCParams);
                }
            }

            if (VEnc_imeta_param.flag)
            {
                ctx->enc->SetInputMetaParams(v4l2_buf.index, VEnc_imeta_param);
                v4l2_buf.reserved2 = v4l2_buf.index;
            }
        }

        if (ctx->copy_timestamp)
        {
          v4l2_buf.flags |= V4L2_BUF_FLAG_TIMESTAMP_COPY;
          ctx->timestamp += ctx->timestampincr;
          v4l2_buf.timestamp.tv_sec = ctx->timestamp / (MICROSECOND_UNIT);
          v4l2_buf.timestamp.tv_usec = ctx->timestamp % (MICROSECOND_UNIT);
        }

        NvBufferParams params; 
        NvBufferGetParams(ctx->output_plane_fd[i], &params);                    
        for (uint32_t j = 0 ; j < buffer->n_planes ; j++)
        {
            v4l2_buf.m.planes[j].bytesused = params.psize[j];
            v4l2_buf.m.planes[j].m.fd = ctx->output_plane_fd[i];
        }
        
        ret = ctx->enc->output_plane.qBuffer(v4l2_buf, NULL);        
        if (ret < 0)
        {
            cerr << "Error while queueing buffer at output plane" << endl;
            abort(ctx);
            goto cleanup;
        }

        /*if (v4l2_buf.m.planes[0].bytesused == 0)
        {
            cerr << "File read complete." << endl;
            eos = true;
            break;
        }*/
        ctx->input_frames_queued_count++;
    }    

    // Wait till capture plane DQ Thread finishes
    // i.e. all the capture plane buffers are dequeued
    if (encoder_proc_blocking(tc, eos) != 0)
        goto cleanup;
    ctx->enc->capture_plane.waitForDQThread(-1);

cleanup:
    cout << "encoder fin " << endl;
    if (ctx->enc && ctx->enc->isInError())
    {
        cerr << "Encoder is in error" << endl;
        error = 1;
    }
    if (ctx->got_error)
    {
        error = 1;
    }

    if (ctx->pBitStreamCrc)
    {
        char *pgold_crc = ctx->gold_crc;
        Crc *pout_crc= ctx->pBitStreamCrc;
        char StrCrcValue[20];
        snprintf (StrCrcValue, 20, "%u", pout_crc->CrcValue);
        // Remove CRLF from end of CRC, if present
        do {
               unsigned int len = strlen(pgold_crc);
               if (len == 0) break;
               if (pgold_crc[len-1] == '\n')
                   pgold_crc[len-1] = '\0';
               else if (pgold_crc[len-1] == '\r')
                   pgold_crc[len-1] = '\0';
               else
                   break;
        } while(1);

        if (strcmp (StrCrcValue, pgold_crc))
        {
            cout << "======================" << endl;
            cout << "video_encode: CRC FAILED" << endl;
            cout << "======================" << endl;
            cout << "Encoded CRC: " << StrCrcValue << " Gold CRC: " << pgold_crc << endl;
            error = 1;
        }
        else
        {
            cout << "======================" << endl;
            cout << "video_encode: CRC PASSED" << endl;
            cout << "======================" << endl;
        }

        CloseCrc(&ctx->pBitStreamCrc);
    }

    for (uint32_t i = 0; i < ctx->enc->output_plane.getNumBuffers(); i++)
    {
        ret = NvBufferDestroy(ctx->output_plane_fd[i]);
        if(ret < 0)
        {
            cerr << "Failed to Destroy NvBuffer\n" << endl;
            return NULL;
        }
    }

    delete ctx->enc;
    delete ctx->out_file;
    delete ctx->roi_Param_file;
    delete ctx->recon_Ref_file;
    delete ctx->rps_Param_file;
    delete ctx->hints_Param_file;
    delete ctx->gdr_Param_file;
    delete ctx->gdr_out_file;

    free(ctx->out_file_path);
    free(ctx->ROI_Param_file_path);
    free(ctx->Recon_Ref_file_path);
    free(ctx->RPS_Param_file_path);
    free(ctx->hints_Param_file_path);
    free(ctx->GDR_Param_file_path);
    free(ctx->GDR_out_file_path);
    delete ctx->runtime_params_str;

    return NULL;
}

















//
// Decoder
//

static int
read_decoder_input_nalu(ifstream * stream, NvBuffer * buffer,
        char *parse_buffer, streamsize parse_buffer_size, dec_context_t * ctx)
{
    // Length is the size of the buffer in bytes
    char *buffer_ptr = (char *) buffer->planes[0].data;
    int h265_nal_unit_type;
    char *stream_ptr;
    bool nalu_found = false;

    streamsize bytes_read;
    streamsize stream_initial_pos = stream->tellg();

    stream->read(parse_buffer, parse_buffer_size);
    bytes_read = stream->gcount();

    if (bytes_read == 0)
    {
        return buffer->planes[0].bytesused = 0;
    }

    // Find the first NAL unit in the buffer
    stream_ptr = parse_buffer;
    while ((stream_ptr - parse_buffer) < (bytes_read - 3))
    {
        nalu_found = IS_NAL_UNIT_START(stream_ptr) ||
                    IS_NAL_UNIT_START1(stream_ptr);
        if (nalu_found)
        {
            break;
        }
        stream_ptr++;
    }

    // Reached end of buffer but could not find NAL unit
    if (!nalu_found)
    {
        cerr << "Could not read nal unit from file. EOF or file corrupted"
            << endl;
        return -1;
    }

    memcpy(buffer_ptr, stream_ptr, 4);
    buffer_ptr += 4;
    buffer->planes[0].bytesused = 4;
    stream_ptr += 4;

    if (ctx->copy_timestamp)
    {
      if (ctx->decoder_pixfmt == V4L2_PIX_FMT_H264) {
        if ((IS_H264_NAL_CODED_SLICE(stream_ptr)) ||
            (IS_H264_NAL_CODED_SLICE_IDR(stream_ptr)))
          ctx->flag_copyts = true;
        else
          ctx->flag_copyts = false;
      } else if (ctx->decoder_pixfmt == V4L2_PIX_FMT_H265) {
        h265_nal_unit_type = GET_H265_NAL_UNIT_TYPE(stream_ptr);
        if ((h265_nal_unit_type >= HEVC_NUT_TRAIL_N && h265_nal_unit_type <= HEVC_NUT_RASL_R) ||
            (h265_nal_unit_type >= HEVC_NUT_BLA_W_LP && h265_nal_unit_type <= HEVC_NUT_CRA_NUT))
          ctx->flag_copyts = true;
        else
          ctx->flag_copyts = false;
      }
    }

    // Copy bytes till the next NAL unit is found
    while ((stream_ptr - parse_buffer) < (bytes_read - 3))
    {
        if (IS_NAL_UNIT_START(stream_ptr) || IS_NAL_UNIT_START1(stream_ptr))
        {
            streamsize seekto = stream_initial_pos +
                    (stream_ptr - parse_buffer);
            if(stream->eof())
            {
                stream->clear();
            }
            stream->seekg(seekto, stream->beg);
            return 0;
        }
        *buffer_ptr = *stream_ptr;
        buffer_ptr++;
        stream_ptr++;
        buffer->planes[0].bytesused++;
    }

    // Reached end of buffer but could not find NAL unit
    cerr << "Could not read nal unit from file. EOF or file corrupted"
            << endl;
    return -1;
}

static int
read_decoder_input_chunk(ifstream * stream, NvBuffer * buffer)
{
    // Length is the size of the buffer in bytes
    streamsize bytes_to_read = MIN(CHUNK_SIZE, buffer->planes[0].length);

    stream->read((char *) buffer->planes[0].data, bytes_to_read);
    // It is necessary to set bytesused properly, so that decoder knows how
    // many bytes in the buffer are valid
    buffer->planes[0].bytesused = stream->gcount();
    if(buffer->planes[0].bytesused == 0)
    {
        stream->clear();
        stream->seekg(0,stream->beg);
    }
    return 0;
}



static void
query_and_set_capture(dec_context_t * ctx)
{
    NvVideoDecoder *dec = ctx->dec;
    struct v4l2_format format;
    struct v4l2_crop crop;
    int32_t min_dec_capture_buffers;
    int ret = 0;
    int error = 0;
    uint32_t window_width;
    uint32_t window_height;
    NvBufferCreateParams input_params = {0};
    NvBufferCreateParams cParams = {0};

    // Get capture plane format from the decoder. This may change after
    // an resolution change event
    ret = dec->capture_plane.getFormat(format);
    TEST_ERROR(ret < 0,
               "Error: Could not get format from decoder capture plane", error);

    // Get the display resolution from the decoder
    ret = dec->capture_plane.getCrop(crop);
    TEST_ERROR(ret < 0,
               "Error: Could not get crop from decoder capture plane", error);

    cout << "Video Resolution: " << crop.c.width << "x" << crop.c.height
        << endl;
    ctx->display_height = crop.c.height;
    ctx->display_width = crop.c.width;

    if(ctx->dst_dma_fd != -1)
    {
        NvBufferDestroy(ctx->dst_dma_fd);
        ctx->dst_dma_fd = -1;
    }

    input_params.payloadType = NvBufferPayload_SurfArray;
    input_params.width = crop.c.width;
    input_params.height = crop.c.height;
    input_params.layout = NvBufferLayout_Pitch;
    input_params.colorFormat = ctx->out_pixfmt == 1 ? NvBufferColorFormat_NV12 : NvBufferColorFormat_YUV420;
    input_params.nvbuf_tag = NvBufferTag_VIDEO_CONVERT;

    ret = NvBufferCreateEx (&ctx->dst_dma_fd, &input_params);
    TEST_ERROR(ret == -1, "create dmabuf failed", error);

    // deinitPlane unmaps the buffers and calls REQBUFS with count 0
    dec->capture_plane.deinitPlane();
    if(ctx->capture_plane_mem_type == V4L2_MEMORY_DMABUF)
    {
        for(int index = 0 ; index < ctx->numCapBuffers ; index++)
        {
            if(ctx->dmabuff_fd[index] != 0)
            {
                ret = NvBufferDestroy (ctx->dmabuff_fd[index]);
                TEST_ERROR(ret < 0, "Failed to Destroy NvBuffer", error);
            }
        }
    }

    // Not necessary to call VIDIOC_S_FMT on decoder capture plane.
    // But decoder setCapturePlaneFormat function updates the class variables
    ret = dec->setCapturePlaneFormat(format.fmt.pix_mp.pixelformat,
                                     format.fmt.pix_mp.width,
                                     format.fmt.pix_mp.height);
    TEST_ERROR(ret < 0, "Error in setting decoder capture plane format", error);    

    ctx->video_height = format.fmt.pix_mp.height;
    ctx->video_width = format.fmt.pix_mp.width;
    // Get the minimum buffers which have to be requested on the capture plane
    ret = dec->getMinimumCapturePlaneBuffers(min_dec_capture_buffers);
    TEST_ERROR(ret < 0,
               "Error while getting value of minimum capture plane buffers",
               error);

    // Request (min + extra) buffers, export and map buffers
    if(ctx->capture_plane_mem_type == V4L2_MEMORY_MMAP)
    {
        ret =
            dec->capture_plane.setupPlane(V4L2_MEMORY_MMAP,
                                          min_dec_capture_buffers + ctx->extra_cap_plane_buffer, false,
                                          false);
        TEST_ERROR(ret < 0, "Error in decoder capture plane setup", error);
    }
    else if(ctx->capture_plane_mem_type == V4L2_MEMORY_DMABUF)
    {
        switch(format.fmt.pix_mp.colorspace)
        {
            case V4L2_COLORSPACE_SMPTE170M:
                if (format.fmt.pix_mp.quantization == V4L2_QUANTIZATION_DEFAULT)
                {
                    cout << "Decoder colorspace ITU-R BT.601 with standard range luma (16-235)" << endl;
                    cParams.colorFormat = NvBufferColorFormat_NV12;
                }
                else
                {
                    cout << "Decoder colorspace ITU-R BT.601 with extended range luma (0-255)" << endl;
                    cParams.colorFormat = NvBufferColorFormat_NV12_ER;
                }
                break;
            case V4L2_COLORSPACE_REC709:
                if (format.fmt.pix_mp.quantization == V4L2_QUANTIZATION_DEFAULT)
                {
                    cout << "Decoder colorspace ITU-R BT.709 with standard range luma (16-235)" << endl;
                    cParams.colorFormat = NvBufferColorFormat_NV12_709;
                }
                else
                {
                    cout << "Decoder colorspace ITU-R BT.709 with extended range luma (0-255)" << endl;
                    cParams.colorFormat = NvBufferColorFormat_NV12_709_ER;
                }
                break;
            case V4L2_COLORSPACE_BT2020:
                {
                    cout << "Decoder colorspace ITU-R BT.2020" << endl;
                    cParams.colorFormat = NvBufferColorFormat_NV12_2020;
                }
                break;
            default:
                cout << "supported colorspace details not available, use default" << endl;
                if (format.fmt.pix_mp.quantization == V4L2_QUANTIZATION_DEFAULT)
                {
                    cout << "Decoder colorspace ITU-R BT.601 with standard range luma (16-235)" << endl;
                    cParams.colorFormat = NvBufferColorFormat_NV12;
                }
                else
                {
                    cout << "Decoder colorspace ITU-R BT.601 with extended range luma (0-255)" << endl;
                    cParams.colorFormat = NvBufferColorFormat_NV12_ER;
                }
                break;
        }

        ctx->numCapBuffers = min_dec_capture_buffers + ctx->extra_cap_plane_buffer;

        for (int index = 0; index < ctx->numCapBuffers; index++)
        {
            cParams.width = crop.c.width;
            cParams.height = crop.c.height;
            cParams.layout = NvBufferLayout_Pitch;
            cParams.payloadType = NvBufferPayload_SurfArray;
            cParams.nvbuf_tag = NvBufferTag_VIDEO_DEC;
            ret = NvBufferCreateEx(&ctx->dmabuff_fd[index], &cParams);            
            TEST_ERROR(ret < 0, "Failed to create buffers", error);
        }
        ret = dec->capture_plane.reqbufs(V4L2_MEMORY_DMABUF,ctx->numCapBuffers);
            TEST_ERROR(ret, "Error in request buffers on capture plane", error);
    }

    // Capture plane STREAMON
    ret = dec->capture_plane.setStreamStatus(true);
    TEST_ERROR(ret < 0, "Error in decoder capture plane streamon", error);

    // Enqueue all the empty capture plane buffers
    for (uint32_t i = 0; i < dec->capture_plane.getNumBuffers(); i++)
    {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];

        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, sizeof(planes));

        v4l2_buf.index = i;
        v4l2_buf.m.planes = planes;
        v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l2_buf.memory = ctx->capture_plane_mem_type;
        if(ctx->capture_plane_mem_type == V4L2_MEMORY_DMABUF)
            v4l2_buf.m.planes[0].m.fd = ctx->dmabuff_fd[i];
        ret = dec->capture_plane.qBuffer(v4l2_buf, NULL);
        TEST_ERROR(ret < 0, "Error Qing buffer at output plane", error);
    }
    cout << "Query and set capture successful" << endl;
    return;

error:
    if (error)
    {
        abort(ctx);
        cerr << "Error in " << __func__ << endl;
    }
}



static void *
dec_capture_loop_fcn(void *arg)
{
    tc_context_t *tc = (tc_context_t *) arg;
    dec_context_t *ctx = &tc->dec_ctx;
    NvVideoDecoder *dec = ctx->dec;
    struct v4l2_event ev;
    int ret;

    cout << "Starting decoder capture loop thread" << endl;
    // Need to wait for the first Resolution change event, so that
    // the decoder knows the stream resolution and can allocate appropriate
    // buffers when we call REQBUFS
    do
    {
        ret = dec->dqEvent(ev, 50000);
        if (ret < 0)
        {
            if (errno == EAGAIN)
            {
                cerr <<
                    "Timed out waiting for first V4L2_EVENT_RESOLUTION_CHANGE"
                    << endl;
            }
            else
            {
                cerr << "Error in dequeueing decoder event" << endl;
            }
            abort(ctx);
            break;
        }
    }
    while ((ev.type != V4L2_EVENT_RESOLUTION_CHANGE) && !ctx->got_error);

    // query_and_set_capture acts on the resolution change event
    if (!ctx->got_error)
    {
        query_and_set_capture(ctx);
        pthread_create(&tc->encoder_proc, NULL, encode_proc, tc);
    }

    // Exit on error or EOS which is signalled in main()
    while (!(ctx->got_error || dec->isInError() || ctx->got_eos))
    {
        NvBuffer *dec_buffer;

        // Check for Resolution change again
        ret = dec->dqEvent(ev, false);
        if (ret == 0)
        {
            switch (ev.type)
            {
                case V4L2_EVENT_RESOLUTION_CHANGE:
                    query_and_set_capture(ctx);
                    continue;
            }
        }

        while (1)
        {
            struct v4l2_buffer v4l2_buf;
            struct v4l2_plane planes[MAX_PLANES];

            memset(&v4l2_buf, 0, sizeof(v4l2_buf));
            memset(planes, 0, sizeof(planes));
            v4l2_buf.m.planes = planes;

            // Dequeue a filled buffer
            if (dec->capture_plane.dqBuffer(v4l2_buf, &dec_buffer, NULL, 0))
            {
                if (errno == EAGAIN)
                {
                    //usleep(1000);
                }
                else
                {
                    abort(ctx);
                    cerr << "Error while calling dequeue at capture plane" <<
                        endl;
                }
                break;
            }

            if (ctx->copy_timestamp && ctx->input_nalu && ctx->stats)
            {
              cout << "[" << v4l2_buf.index << "]" "dec capture plane dqB timestamp [" <<
                  v4l2_buf.timestamp.tv_sec << "s" << v4l2_buf.timestamp.tv_usec << "us]" << endl;
            }
            v4l2_buf.m.planes[0].m.fd = ctx->dmabuff_fd[v4l2_buf.index];

            cout << "encode frame " << v4l2_buf.index << " " << v4l2_buf.m.planes[0].m.fd << endl;

            // Push to encoder
            tc->buffers.push_back(v4l2_buf);
            auto v4l2_buf2 = tc->release_buffers.front();

            v4l2_buf2.m.planes[0].m.fd = ctx->dmabuff_fd[v4l2_buf.index];
            

            if (dec->capture_plane.qBuffer(v4l2_buf2, NULL) < 0)
            {
                tc->release_buffers.pop_front();
                abort(ctx);
                cerr <<
                    "Error while queueing buffer at decoder capture plane"
                    << endl;
                break;
            }
            tc->release_buffers.pop_front();
        }
    }
    cout << "Exiting decoder capture loop thread" << endl;
    return NULL;
}


static bool decoder_proc_blocking(dec_context_t *ctx, bool eos, uint32_t current_file,
                                int current_loop, char *nalu_parse_buffer)
{
    // Since all the output plane buffers have been queued, we first need to
    // dequeue a buffer from output plane before we can read new data into it
    // and queue it again.
    int allow_DQ = true;
    int ret = 0;
    struct v4l2_buffer temp_buf;

    while (!eos && !ctx->got_error && !ctx->dec->isInError())
    {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];
        NvBuffer *buffer;

        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, sizeof(planes));

        v4l2_buf.m.planes = planes;

        if(allow_DQ)
        {
            ret = ctx->dec->output_plane.dqBuffer(v4l2_buf, &buffer, NULL, -1);
            if (ret < 0)
            {
                cerr << "Error DQing buffer at output plane" << endl;
                abort(ctx);
                break;
            }
        }
        else
        {
            allow_DQ = true;
            memcpy(&v4l2_buf,&temp_buf,sizeof(v4l2_buffer));
            buffer = ctx->dec->output_plane.getNthBuffer(v4l2_buf.index);
        }

        if ((ctx->decoder_pixfmt == V4L2_PIX_FMT_H264) ||
                (ctx->decoder_pixfmt == V4L2_PIX_FMT_H265) ||
                (ctx->decoder_pixfmt == V4L2_PIX_FMT_MPEG2) ||
                (ctx->decoder_pixfmt == V4L2_PIX_FMT_MPEG4))
        {
            if (ctx->input_nalu)
            {
                read_decoder_input_nalu(ctx->in_file[current_file], buffer, nalu_parse_buffer,
                        CHUNK_SIZE, ctx);
            }
            else
            {
                read_decoder_input_chunk(ctx->in_file[current_file], buffer);
            }
        }

        if (ctx->live) {
            usleep(40000);
        }

        v4l2_buf.m.planes[0].bytesused = buffer->planes[0].bytesused;

        if (ctx->input_nalu && ctx->copy_timestamp && ctx->flag_copyts)
        {
          v4l2_buf.flags |= V4L2_BUF_FLAG_TIMESTAMP_COPY;
          ctx->timestamp += ctx->timestampincr;
          v4l2_buf.timestamp.tv_sec = ctx->timestamp / (MICROSECOND_UNIT);
          v4l2_buf.timestamp.tv_usec = ctx->timestamp % (MICROSECOND_UNIT);
        }

        if (v4l2_buf.m.planes[0].bytesused == 0)
        {
            if(ctx->bLoop)
            {
                current_file = current_file % ctx->file_count;
                allow_DQ = false;
                memcpy(&temp_buf,&v4l2_buf,sizeof(v4l2_buffer));
            }
        }
        ret = ctx->dec->output_plane.qBuffer(v4l2_buf, NULL);
        if (ret < 0)
        {
            cerr << "Error Qing buffer at output plane" << endl;
            abort(ctx);
            break;
        }
        if (v4l2_buf.m.planes[0].bytesused == 0)
        {
            eos = true;
            cout << "Input file read complete" << endl;
            break;
        }
    }
    return eos;
}




static void
dec_set_defaults(dec_context_t * ctx)
{
    memset(ctx, 0, sizeof(dec_context_t));
    ctx->fullscreen = false;
    ctx->window_height = 0;
    ctx->window_width = 0;
    ctx->window_x = 0;
    ctx->window_y = 0;
    ctx->out_pixfmt = 1;
    ctx->fps = 30;
    ctx->output_plane_mem_type = V4L2_MEMORY_USERPTR;
    ctx->capture_plane_mem_type = V4L2_MEMORY_DMABUF;
    ctx->vp9_file_header_flag = 0;
    ctx->vp8_file_header_flag = 0;
    ctx->stress_test = 1;
    ctx->copy_timestamp = false;
    ctx->flag_copyts = false;
    ctx->start_ts = 0;
    ctx->file_count = 1;
    ctx->dec_fps = 30;
    ctx->dst_dma_fd = -1;
    ctx->bLoop = false;
    //ctx->bQueue = false;
    ctx->loop_count = 0;
    ctx->max_perf = 0;
    ctx->extra_cap_plane_buffer = 1;//5;
    //ctx->blocking_mode = 1;
    pthread_mutex_init(&ctx->queue_lock, NULL);
    pthread_cond_init(&ctx->queue_cond, NULL);
}

static void *decoder_pollthread_fcn(void *arg)
{

    dec_context_t *ctx = (dec_context_t *) arg;
    v4l2_ctrl_video_device_poll devicepoll;

    cout << "Starting Device Poll Thread " << endl;

    memset(&devicepoll, 0, sizeof(v4l2_ctrl_video_device_poll));

    // wait here until you are signalled to issue the Poll call.
    // Check if the abort status is set , if so exit
    // Else issue the Poll on the decoder and block.
    // When the Poll returns, signal the decoder thread to continue.

    while (!ctx->got_error && !ctx->dec->isInError())
    {
        sem_wait(&ctx->pollthread_sema);

        if (ctx->got_eos)
        {
            cout << "Decoder got eos, exiting poll thread \n";
            return NULL;
        }

        devicepoll.req_events = POLLIN | POLLOUT | POLLERR | POLLPRI;

        // This call shall wait in the v4l2 decoder library
        ctx->dec->DevicePoll(&devicepoll);

        // We can check the devicepoll.resp_events bitmask to see which events are set.
        sem_post(&ctx->decoderthread_sema);
    }
    return NULL;
}






static int
transcoder_proc(tc_context_t *tc_ctx, int argc, char *argv[])
{
    int ret = 0;
    int error = 0;
    uint32_t current_file = 0;
    uint32_t i;
    bool eos = false;
    int current_loop = 0;
    char *nalu_parse_buffer = NULL;
    NvApplicationProfiler &profiler = NvApplicationProfiler::getProfilerInstance();
    
    tc_ctx->argc = argc;
    tc_ctx->argv = argv;
    sem_init(&tc_ctx->sema, 0, 0);
    dec_context_t *dec_ctx = &tc_ctx->dec_ctx;
    printf("transcoder_proc\n");

    dec_set_defaults(dec_ctx);

    pthread_setname_np(pthread_self(), "DecOutPlane");

    if (dec_parse_csv_args(dec_ctx, argc, argv))
    {
        fprintf(stderr, "Error parsing commandline arguments\n");
        return -1;
    }

    cout << "Creating decoder in blocking mode \n";
    dec_ctx->dec = NvVideoDecoder::createVideoDecoder(dec_name);

    TEST_ERROR(!dec_ctx->dec, "Could not create decoder", cleanup);

    dec_ctx->in_file = (std::ifstream **)malloc(sizeof(std::ifstream *)*dec_ctx->file_count);
    for (uint32_t i = 0 ; i < dec_ctx->file_count ; i++)
    {
        dec_ctx->in_file[i] = new ifstream(dec_ctx->in_file_path[i]);
        TEST_ERROR(!dec_ctx->in_file[i]->is_open(), "Error opening input file", cleanup);
    }


    if (dec_ctx->stats)
    {
        cout << "hi profiler" << dec_ctx->stats << endl;
        profiler.start(NvApplicationProfiler::DefaultSamplingInterval);
        dec_ctx->dec->enableProfiling();
    }

    // Subscribe to Resolution change event
    ret = dec_ctx->dec->subscribeEvent(V4L2_EVENT_RESOLUTION_CHANGE, 0, 0);
    TEST_ERROR(ret < 0, "Could not subscribe to V4L2_EVENT_RESOLUTION_CHANGE",
               cleanup);

    // Set format on the output plane
    ret = dec_ctx->dec->setOutputPlaneFormat(dec_ctx->decoder_pixfmt, CHUNK_SIZE);
    TEST_ERROR(ret < 0, "Could not set output plane format", cleanup);

    if (dec_ctx->input_nalu)
    {
        nalu_parse_buffer = new char[CHUNK_SIZE];
        printf("Setting frame input mode to 0 \n");
        ret = dec_ctx->dec->setFrameInputMode(0);
        TEST_ERROR(ret < 0,
                "Error in decoder setFrameInputMode", cleanup);
    }
    else
    {
        // Set V4L2_CID_MPEG_VIDEO_DISABLE_COMPLETE_FRAME_INPUT control to false
        // so that application can send chunks of encoded data instead of forming
        // complete frames.
        printf("Setting frame input mode to 1 \n");
        ret = dec_ctx->dec->setFrameInputMode(1);
        TEST_ERROR(ret < 0,
                "Error in decoder setFrameInputMode", cleanup);
    }

    // V4L2_CID_MPEG_VIDEO_DISABLE_DPB should be set after output plane
    // set format
    if (dec_ctx->disable_dpb)
    {
        ret = dec_ctx->dec->disableDPB();
        TEST_ERROR(ret < 0, "Error in decoder disableDPB", cleanup);
    }

    if (dec_ctx->enable_metadata || dec_ctx->enable_input_metadata)
    {
        ret = dec_ctx->dec->enableMetadataReporting();
        TEST_ERROR(ret < 0, "Error while enabling metadata reporting", cleanup);
    }

    if (dec_ctx->max_perf)
    {
        ret = dec_ctx->dec->setMaxPerfMode(dec_ctx->max_perf);
        TEST_ERROR(ret < 0, "Error while setting decoder to max perf", cleanup);
    }

    if (dec_ctx->skip_frames)
    {
        ret = dec_ctx->dec->setSkipFrames(dec_ctx->skip_frames);
        TEST_ERROR(ret < 0, "Error while setting skip frames param", cleanup);
    }

    // Query, Export and Map the output plane buffers so that we can read
    // encoded data into the buffers    
    if (dec_ctx->output_plane_mem_type == V4L2_MEMORY_MMAP) {
        ret = dec_ctx->dec->output_plane.setupPlane(V4L2_MEMORY_MMAP, 2, true, false);
    } else if (dec_ctx->output_plane_mem_type == V4L2_MEMORY_USERPTR) {
        ret = dec_ctx->dec->output_plane.setupPlane(V4L2_MEMORY_USERPTR, 10, false, true);
    } 

    TEST_ERROR(ret < 0, "Error while setting up output plane", cleanup);


    ret = dec_ctx->dec->output_plane.setStreamStatus(true);
    TEST_ERROR(ret < 0, "Error in output plane stream on", cleanup);


    if (dec_ctx->copy_timestamp && dec_ctx->input_nalu) {
      dec_ctx->timestamp = (dec_ctx->start_ts * MICROSECOND_UNIT);
      dec_ctx->timestampincr = (MICROSECOND_UNIT * 16) / ((uint32_t) (dec_ctx->dec_fps * 16));
    }

    // Read encoded data and enqueue all the output plane buffers.
    // Exit loop in case file read is complete.
    i = 0;
    current_loop = 1;
    while (!eos && !dec_ctx->got_error && !dec_ctx->dec->isInError() &&
           i < dec_ctx->dec->output_plane.getNumBuffers())
    {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];
        NvBuffer *buffer;

        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, sizeof(planes));

        buffer = dec_ctx->dec->output_plane.getNthBuffer(i);
        if ((dec_ctx->decoder_pixfmt == V4L2_PIX_FMT_H264) ||
                (dec_ctx->decoder_pixfmt == V4L2_PIX_FMT_H265) ||
                (dec_ctx->decoder_pixfmt == V4L2_PIX_FMT_MPEG2) ||
                (dec_ctx->decoder_pixfmt == V4L2_PIX_FMT_MPEG4))
        {
            if (dec_ctx->input_nalu)
            {
                read_decoder_input_nalu(dec_ctx->in_file[current_file], buffer, nalu_parse_buffer,
                        CHUNK_SIZE, dec_ctx);
            }
            else
            {
                read_decoder_input_chunk(dec_ctx->in_file[current_file], buffer);
            }
        }

        v4l2_buf.index = i;
        v4l2_buf.m.planes = planes;
        v4l2_buf.m.planes[0].bytesused = buffer->planes[0].bytesused;

        if (dec_ctx->input_nalu && dec_ctx->copy_timestamp && dec_ctx->flag_copyts)
        {
          v4l2_buf.flags |= V4L2_BUF_FLAG_TIMESTAMP_COPY;
          dec_ctx->timestamp += dec_ctx->timestampincr;
          v4l2_buf.timestamp.tv_sec = dec_ctx->timestamp / (MICROSECOND_UNIT);
          v4l2_buf.timestamp.tv_usec = dec_ctx->timestamp % (MICROSECOND_UNIT);
        }

        if (v4l2_buf.m.planes[0].bytesused == 0)
        {
            if(dec_ctx->bLoop)
            {
                current_file = current_file % dec_ctx->file_count;
                if(dec_ctx->loop_count == 0 || current_loop < dec_ctx->loop_count )
                {
                    current_loop++;
                    continue;
                }
            }
        }
        // It is necessary to queue an empty buffer to signal EOS to the decoder
        // i.e. set v4l2_buf.m.planes[0].bytesused = 0 and queue the buffer
        ret = dec_ctx->dec->output_plane.qBuffer(v4l2_buf, NULL);
        if (ret < 0)
        {
            cerr << "Error Qing buffer at output plane" << endl;
            abort(dec_ctx);
            break;
        }
        if (v4l2_buf.m.planes[0].bytesused == 0)
        {
            eos = true;
            cout << "Input file read complete" << endl;
            break;
        }
        i++;
    }

    pthread_create(&dec_ctx->dec_capture_loop, NULL, dec_capture_loop_fcn, tc_ctx);
    pthread_setname_np(dec_ctx->dec_capture_loop, "DecCapPlane");
    
    eos = decoder_proc_blocking(dec_ctx, eos, current_file, current_loop, nalu_parse_buffer);
    // After sending EOS, all the buffers from output plane should be dequeued.
    // and after that capture plane loop should be signalled to stop.
    while (dec_ctx->dec->output_plane.getNumQueuedBuffers() > 0 &&
            !dec_ctx->got_error && !dec_ctx->dec->isInError())
    {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];

        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, sizeof(planes));

        v4l2_buf.m.planes = planes;
        ret = dec_ctx->dec->output_plane.dqBuffer(v4l2_buf, NULL, NULL, -1);
        if (ret < 0)
        {
            cerr << "Error DQing buffer at output plane" << endl;
            abort(dec_ctx);
            break;
        }

    }

    // Signal EOS to the decoder capture loop
    dec_ctx->got_eos = true;

    if (dec_ctx->stats)
    {
        profiler.stop();
        dec_ctx->dec->printProfilingStats(cout);
        profiler.printProfilerData(cout);
    }

cleanup:
    if (dec_ctx->dec_capture_loop)
    {
        pthread_join(dec_ctx->dec_capture_loop, NULL);
    }
    if (dec_ctx->dec && dec_ctx->dec->isInError())
    {
        cerr << "Decoder is in error" << endl;
        error = 1;
    }

    if (dec_ctx->got_error)
    {
        error = 1;
    }

    // The decoder destructor does all the cleanup i.e set streamoff on output and capture planes,
    // unmap buffers, tell decoder to deallocate buffer (reqbufs ioctl with counnt = 0),
    // and finally call v4l2_close on the fd.
    delete dec_ctx->dec;
    for (uint32_t i = 0 ; i < dec_ctx->file_count ; i++)
      delete dec_ctx->in_file[i];
    
    if(dec_ctx->dst_dma_fd != -1)
    {
        NvBufferDestroy(dec_ctx->dst_dma_fd);
        dec_ctx->dst_dma_fd = -1;
    }
    delete[] nalu_parse_buffer;

    free (dec_ctx->in_file);
    for (uint32_t i = 0 ; i < dec_ctx->file_count ; i++)
      free (dec_ctx->in_file_path[i]);
    free (dec_ctx->in_file_path);

    return -error;
}

int
main(int argc, char *argv[])
{    
    int ret = 0;
    tc_context_t tc_ctx;
    //memset(&tc_ctx, 0, sizeof(tc_ctx));
 
    ret = transcoder_proc(&tc_ctx, argc, argv);
 
    if (ret)
    {
        cout << "App run failed" << endl;
    }
    else
    {
        cout << "App run was successful" << endl;
    }

    return ret;
}
