/*
 * Copyright (c) 2016-2019, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <cstdlib>
#include <cstring>

#include "video_decode.h"

#define CHECK_OPTION_VALUE(argp) if(!*argp || (*argp)[0] == '-') \
                                { \
                                    cerr << "Error: value not specified for option " << arg << endl; \
                                    goto error; \
                                }

#define CSV_PARSE_CHECK_ERROR(condition, str) \
    if (condition) {\
    cerr << "Error: " << str << endl; \
    goto error; \
    }

using namespace std;

static void
print_help(void)
{
    cout << "\ntranscoder <in-format> <in-file> <encoder-type> <out-file> [OPTIONS]\n\n"
            "Supported formats:\n"
            "\tVP9\n"
            "\tVP8\n"
            "\tH264\n"
            "\tH265\n"
            "\tMPEG2\n"
            "\tMPEG4\n\n"
            "Encoder Types:\n"
            "\tH264\n"
            "\tH265\n"
            "\tVP8\n"
            "\tVP9\n\n"
            "DECODER OPTIONS:\n"
            "\t-h,--help            Prints this text\n"
            "\t--dbg-level <level>  Sets the debug level [Values 0-3]\n\n"
            "\t--stats              Report profiling data for the app\n\n"
            "\tNOTE: this should not be used alongside -o option as it decreases the FPS value shown in --stats\n"
            "\t--disable-rendering  Disable rendering\n"
            "\t--max-perf           Enable maximum Performance \n"
            "\tNOTE: this should be set only for platform T194 or above\n"
            "\t--fullscreen         Fullscreen playback [Default = disabled]\n"
            "\t-ww <width>          Window width in pixels [Default = video-width]\n"
            "\t-wh <height>         Window height in pixels [Default = video-height]\n"
            "\t-loop <count>        Playback in a loop.[count = 1,2,...,n times looping , 0 = infinite looping]\n"
            "\t-queue [<file1> <file2> ....] Files are played in a queue manner\n"
            "\tNOTE: -queue should be the last option mentioned in the command line no other option should be mentioned after that.\n"
            "\t-wx <x-offset>       Horizontal window offset [Default = 0]\n"
            "\t-wy <y-offset>       Vertical window offset [Default = 0]\n\n"
            "\t-fps <fps>           Display rate in frames per second [Default = 30]\n\n"
            "\t-o <out-file>        Write to output file\n\n"
            "\tNOTE: Not to be used along-side -loop and -queue option.\n"
            "\t-f <out_pixfmt>      1 NV12, 2 I420 [Default = 1]\n\n"
            "\t-sf <value>          Skip frames while decoding [Default = 0]\n"
            "\tAllowed values for the skip-frames parameter:\n"
            "\t0 = Decode all frames\n"
            "\t1 = Skip non-reference frames\n"
            "\t2 = Decode only key frames\n\n"
            "\t--input-nalu         Input to the decoder will be nal units\n"
            "\t--input-chunks       Input to the decoder will be a chunk of bytes [Default]\n\n"
            "\t--copy-timestamp <st> <fps> Enable copy timestamp with start timestamp(st) in seconds for decode fps(fps) (for input-nalu mode)\n"
            "\tNOTE: copy-timestamp used to demonstrate how timestamp can be associated with an individual H264/H265 frame to achieve video-synchronization.\n"
            "\t      currenly only supported for H264 & H265 video encode using MM APIs and is only for demonstration purpose.\n"
            "\t--report-metadata    Enable metadata reporting\n\n"
            "\t--blocking-mode <val> Set blocking mode, 0 is non-blocking, 1 for blocking (Default) \n\n"
            "\t--report-input-metadata  Enable metadata reporting for input header parsing error\n\n"
            "\t-v4l2-memory-out-plane <num>       Specify memory type to be used on Output Plane [1 = V4L2_MEMORY_MMAP, 2 = V4L2_MEMORY_USERPTR], Default = V4L2_MEMORY_MMAP\n\n"
            "\t-v4l2-memory-cap-plane <num>       Specify memory type to be used on Capture Plane [1 = V4L2_MEMORY_MMAP, 2 = V4L2_MEMORY_DMABUF], Default = V4L2_MEMORY_DMABUF\n\n"
            "\t-s <loop-count>      Stress test [Default = 1]\n\n"
            "\t-extra_cap_plane_buffer <num>      Specify extra capture plane buffers (Default=1, MAX=32) to be allocated\n"
#ifndef USE_NVBUF_TRANSFORM_API
            "\t--do-yuv-rescale     Rescale decoded YUV from full range to limited range\n\n"
#endif
            "ENCODER OPTIONS:\n"
            "\t-h,--help             Prints this text\n"
            "\t--dbg-level <level>   Sets the debug level [Values 0-3]\n\n"
            "\r-ew <width>            Width encoded picture"
            "\r-eh <height>            Width encoded picture"
            "\t-br <bitrate>         Bitrate [Default = 4000000]\n"
            "\t-pbr <peak_bitrate>   Peak bitrate [Default = 1.2*bitrate]\n\n"
            "NOTE: Peak bitrate takes effect in VBR more; must be >= bitrate\n\n"
            "\t-p <profile>          Encoding Profile [Default = baseline]\n"
            "\t-rc <rate-control>    Ratecontrol mode [Default = cbr]\n"
            "\t--elossless           Enable Lossless encoding [Default = disabled,"
                                     "Option applicable only with YUV444 input and H264 encoder]\n"
            "\t--max-perf            Enable maximum Performance \n"
            "\t-ifi <interval>       I-frame Interval [Default = 30]\n"
            "\t-idri <interval>      IDR Interval [Default = 256]\n"
            "\t--insert-spspps-idr   Insert SPS PPS at every IDR [Default = disabled]\n"
            "\t--insert-vui          Insert VUI [Default = disabled]\n"
            "\t--enable-extcolorfmt  Set Extended ColorFormat (Only works with insert-vui) [Default = disabled]\n"
            "\t--insert-aud          Insert AUD [Default = disabled]\n"
            "\t--alliframes          Enable all I-frame encoding [Default = disabled]\n"
            "\t-fps <num> <den>      Encoding fps in num/den [Default = 30/1]\n\n"
            "\t-tt <level>           Temporal Tradeoff level [Default = 0]\n"
            "\t-vbs <size>           Virtual buffer size [Default = 0]\n"
            "\t-nrf <num>            Number of reference frames [Default = 1]\n\n"
            "\t-slt <type>           Slice length type (1 = Number of MBs, 2 = Bytes) [Default = 1]\n"
            "\t-hpt <type>           HW preset type (1 = ultrafast, 2 = fast, 3 = medium,  4 = slow)\n"
            "\t-slen <length>        Slice length [Default = 0]\n"
            "\t-sir <interval>       Slice intrarefresh interval [Default = 0]\n\n"
            "\t-nbf <num>            Number of B frames [Default = 0]\n\n"
            "\t-rpc <string>         Change configurable parameters at runtime\n\n"
            "\t-goldcrc <string>     GOLD CRC\n\n"
            "\t--rcrc                Reconstructed surface CRC\n\n"
            "\t-rl <cordinate>       Reconstructed surface Left cordinate [Default = 0]\n\n"
            "\t-rt <cordinate>       Reconstructed surface Top cordinate [Default = 0]\n\n"
            "\t-rw <val>             Reconstructed surface width\n\n"
            "\t-rh <val>             Reconstructed surface height\n\n"
            "\t-rcrcf <reconref_file_path> Specify recon crc reference param file\n\n"
            "\t--report-metadata     Print encoder output metadata\n"
            "\t--blocking-mode <val> Set blocking mode, 0 is non-blocking, 1 for blocking (Default) \n\n"
            "\t--input-metadata      Enable encoder input metadata\n"
            "\t--copy-timestamp <st> Enable copy timestamp with start timestamp(st) in seconds\n"
            "\t--mvdump              Dump encoded motion vectors\n\n"
            "\t--eroi                Enable ROI [Default = disabled]\n\n"
            "\t-roi <roi_file_path>  Specify roi param file\n\n"
            "\t--erps                Enable External RPS [Default = disabled]\n\n"
            "\t--egdr                Enable GDR [Default = disabled]\n\n"
            "\t--gif                 Enable Gaps in FrameNum [Default = disabled]\n\n"
            "\t-fnb <num_bits>       H264 FrameNum bits [Default = 0]\n\n"
            "\t-plb <num_bits>       H265 poc lsb bits [Default = 0]\n\n"
            "\t--ni                  No I-frames [Default = disabled]\n\n"
            "\t-rpsf <rps_file_path> Specify external rps param file\n\n"
            "\t--erh                 Enable External picture RC [Default = disabled]\n\n"
            "\t-mem_type_oplane <num> Specify memory type for the output plane to be used [1 = V4L2_MEMORY_MMAP, 2 = V4L2_MEMORY_USERPTR, 3 = V4L2_MEMORY_DMABUF]\n\n"
            "\t-gdrf <gdr_file_path> Specify GDR Parameters filename \n\n"
            "\t-gdrof <gdr_out_file_path> Specify GDR Out filename \n\n"
            "\t-smq <max_qp_value>   Max QP per session when external picture RC enabled\n\n"
            "\t-hf <hint_file_path>  Specify external rate control param file\n\n"
            "\t-MinQpI               Specify minimum Qp Value for I frame\n\n"
            "\t-MaxQpI               Specify maximum Qp Value for I frame\n\n"
            "\t-MinQpP               Specify minimum Qp Value for P frame\n\n"
            "\t-MaxQpP               Specify maximum Qp Value for P frame\n\n"
            "\t-MinQpB               Specify minimum Qp Value for B frame\n\n"
            "\t-MaxQpB               Specify maximum Qp Value for B frame\n\n"
            "\t-s <loop-count>       Stress test [Default = 1]\n\n"
            "NOTE: roi parameters need to be feed per frame in following format\n"
            "      <no. of roi regions> <Qpdelta> <left> <top> <width> <height> ...\n"
            "      e.g. [Each line corresponds roi parameters for one frame] \n"
            "      2   -2   34  33  16  19  -3  68  68  16  16\n"
            "      1   -5   40  40  40  40\n"
            "      3   -4   34  34  16  16  -5  70  70  18  18  -3  100  100  34  34\n"
            "Supported Encoding profiles for H.264:\n"
            "\tbaseline\tmain\thigh\n"
            "Supported Encoding profiles for H.265:\n"
            "\tmain\n"
            "\tmain10\n"
            "Supported Encoding rate control modes:\n"
            "\tcbr\tvbr\n\n"
            "Supported Temporal Tradeoff levels:\n"
            "0:Drop None       1:Drop 1 in 5      2:Drop 1 in 3\n"
            "3:Drop 1 in 2     4:Drop 2 in 3\n\n"
            "Runtime configurable parameter string should be of the form:\n"
            "\"f<frame_num1>,<prop_id1><val>,<prop_id2><val>#f<frame_num1>,<prop_id1><val>,<prop_id2><val>#...\"\n"
            "e.g. \"f20,b8000000,i1#f300,b6000000,r40/1\"\n\n"
            "Property ids:\n"
            "\tb<bitrate>  Bitrate\n"
            "\tp<peak_bitrate>  Peak Bitrate\n"
            "\tr<num/den>  Framerate\n"
            "\ti1          Force I-frame\n\n"
            "NOTE: These encoding parameters are slightly imprecisely updated depending upon the number of\n"
            "frames in queue and/or processed already.\n";
}

static uint32_t
get_decoder_type(char *arg)
{
    if (!strcmp(arg, "H264"))
        return V4L2_PIX_FMT_H264;
    if (!strcmp(arg, "H265"))
        return V4L2_PIX_FMT_H265;
    if (!strcmp(arg, "VP9"))
        return V4L2_PIX_FMT_VP9;
    if (!strcmp(arg, "VP8"))
        return V4L2_PIX_FMT_VP8;
    if (!strcmp(arg, "MPEG2"))
        return V4L2_PIX_FMT_MPEG2;
    if (!strcmp(arg, "MPEG4"))
        return V4L2_PIX_FMT_MPEG4;
    return 0;
}

static int32_t
get_dbg_level(char *arg)
{
    int32_t log_level = atoi(arg);

    if (log_level < 0)
    {
        cout << "Warning: invalid log level input, defaulting to setting 0" << endl;
        return 0;
    }

    if (log_level > 3)
    {
        cout << "Warning: invalid log level input, defaulting to setting 3" << endl;
        return 3;
    }

    return log_level;
}

int
dec_parse_csv_args(dec_context_t * ctx, int argc, char *argv[])
{
    char **argp = argv;
    char *arg = *(++argp);

    if (argc == 1 || (arg && (!strcmp(arg, "-h") || !strcmp(arg, "--help"))))
    {
        print_help();
        exit(EXIT_SUCCESS);
    }    

    CSV_PARSE_CHECK_ERROR(argc < 4, "Insufficient arguments");

    ctx->decoder_pixfmt = get_decoder_type(argv[1]);
    CSV_PARSE_CHECK_ERROR(ctx->decoder_pixfmt == 0,
                          "Incorrect input format");

    ctx->in_file_path = (char **)malloc(sizeof(char *)*1);
    ctx->in_file_path[0] = strdup(argv[2]);
    CSV_PARSE_CHECK_ERROR(!ctx->in_file_path, "Input file not specified");

    argp = &argv[4];    

    while ((arg = *(++argp)))
    {
        /*if (!strcmp(arg, "-o"))
        {
            argp++;
            CHECK_OPTION_VALUE(argp);
            ctx->out_file_path = strdup(*argp);
            CSV_PARSE_CHECK_ERROR(!ctx->out_file_path,
                                  "Output file not specified");
        }
        else*/ if (!strcmp(arg, "-f"))
        {
            argp++;
            CHECK_OPTION_VALUE(argp);
            ctx->out_pixfmt = atoi(*argp);
            CSV_PARSE_CHECK_ERROR((ctx->out_pixfmt < 1 || ctx->out_pixfmt > 2),
                                    "format shoud be 1(NV12), 2(I420)");
        }
        else if (!strcmp(arg, "--stats"))
        {
            ctx->stats = true;
        }
        /*else if (!strcmp(arg, "--disable-rendering"))
        {
            ctx->disable_rendering = true;
        }*/
        else if (!strcmp(arg, "--disable-dpb"))
        {
            ctx->disable_dpb = true;
        }
        else if (!strcmp(arg, "--fullscreen"))
        {
            ctx->fullscreen = true;
        }
        else if (!strcmp(arg, "-wh"))
        {
            argp++;
            CHECK_OPTION_VALUE(argp);
            ctx->window_height = atoi(*argp);
            CSV_PARSE_CHECK_ERROR(ctx->window_height == 0,
                                  "Window height should be > 0");
        }
        else if (!strcmp(arg, "-ww"))
        {
            argp++;
            CHECK_OPTION_VALUE(argp);
            ctx->window_width = atoi(*argp);
            CSV_PARSE_CHECK_ERROR(ctx->window_width == 0,
                                  "Window width should be > 0");
        }
        else if (!strcmp(arg, "-wx"))
        {
            argp++;
            CHECK_OPTION_VALUE(argp);
            ctx->window_x = atoi(*argp);
        }
        else if (!strcmp(arg, "-wy"))
        {
            argp++;
            CHECK_OPTION_VALUE(argp);
            ctx->window_y = atoi(*argp);
        }
        else if (!strcmp(arg, "-fps"))
        {
            argp++;
            CHECK_OPTION_VALUE(argp);
            ctx->fps = atof(*argp);
            CSV_PARSE_CHECK_ERROR(ctx->fps == 0, "FPS should be > 0");
        }
        else if (!strcmp(arg, "--input-nalu"))
        {
            ctx->input_nalu = true;
        }
        else if (!strcmp(arg, "--input-chunks"))
        {
            ctx->input_nalu = false;
        }
        else if (!strcmp(arg, "--copy-timestamp"))
        {
            argp++;
            CHECK_OPTION_VALUE(argp);
            ctx->start_ts = atoi(*argp);
            CSV_PARSE_CHECK_ERROR(ctx->start_ts < 0, "start timestamp should be >= 0");
            argp++;
            CHECK_OPTION_VALUE(argp);
            ctx->dec_fps = atof(*argp);
            CSV_PARSE_CHECK_ERROR(ctx->dec_fps <= 0, "decode fps should be > 0");
            ctx->copy_timestamp = true;
        }
        else if (!strcmp(arg, "--report-metadata"))
        {
            ctx->enable_metadata = true;
        }
        else if (!strcmp(arg, "--report-input-metadata"))
        {
            ctx->enable_input_metadata = true;
        }
        else if (!strcmp(arg, "-s"))
        {
            argp++;
            CHECK_OPTION_VALUE(argp);
            ctx->stress_test = atoi(*argp);
            CSV_PARSE_CHECK_ERROR(ctx->stress_test <= 0,
                    "stress times should be bigger than 0");
        }
        else if(!strcmp(arg, "-v4l2-memory-out-plane"))
        {
            argp++;
            CHECK_OPTION_VALUE(argp);
            ctx->output_plane_mem_type = (enum v4l2_memory) atoi(*argp);
            CSV_PARSE_CHECK_ERROR(
                    (ctx->output_plane_mem_type > V4L2_MEMORY_USERPTR ||
                     ctx->output_plane_mem_type < V4L2_MEMORY_MMAP),
                    "Unsupported v4l2 memory type: " << *argp);
        }
        else if(!strcmp(arg, "--max-perf"))
        {
            ctx->max_perf = 1;
        }
        else if(!strcmp(arg, "-extra_cap_plane_buffer"))
        {
            argp++;
            CHECK_OPTION_VALUE(argp);
            ctx->extra_cap_plane_buffer = atoi(*argp);;
            CSV_PARSE_CHECK_ERROR((ctx->extra_cap_plane_buffer < 1 || ctx->extra_cap_plane_buffer > 32),
                                    "extra capture plane buffer should be greater than 0 & less than 33");
        }
        else if (!strcmp(arg, "-sf"))
        {
            argp++;
            CHECK_OPTION_VALUE(argp);
            ctx->skip_frames = (enum v4l2_skip_frames_type) atoi(*argp);
            CSV_PARSE_CHECK_ERROR(
                    (ctx->skip_frames > V4L2_SKIP_FRAMES_TYPE_DECODE_IDR_ONLY ||
                     ctx->skip_frames < V4L2_SKIP_FRAMES_TYPE_NONE),
                    "Unsupported values for skip frames: " << *argp);
        }
        else if (!strcmp(arg, "--dbg-level"))
        {
            argp++;
            CHECK_OPTION_VALUE(argp);
            log_level = get_dbg_level(*argp);
        }
        else if (!strcmp(arg, "-h") || !strcmp(arg, "--help"))
        {
            print_help();
            exit(EXIT_SUCCESS);
        }
        else if (!strcmp(arg, "-loop"))
        {
            ctx->bLoop = true;
            argp++;
            CHECK_OPTION_VALUE(argp);
            ctx->loop_count = atoi(*argp);
        }
        /*else if (!strcmp(arg, "-queue"))
        {
            ctx->bQueue = true;
            break;
        }*/
        else if (!strcmp(arg, "-v4l2-memory-cap-plane"))
        {
            argp++;
            CHECK_OPTION_VALUE(argp);
            int num = (uint32_t) atoi(*argp);
            switch(num)
            {
                case 1  :ctx->capture_plane_mem_type = V4L2_MEMORY_MMAP;
                         break;
                case 2  :ctx->capture_plane_mem_type = V4L2_MEMORY_DMABUF;
                         break;
            }
            CSV_PARSE_CHECK_ERROR(!(num > 0 && num < 3),
                    "Memory type selection should be > 0 and < 3");
        }
        /*else if (!strcmp(arg, "--blocking-mode"))
        {
            argp++;
            CHECK_OPTION_VALUE(argp);
            ctx->blocking_mode = atoi(*argp);
        }*/
#ifndef USE_NVBUF_TRANSFORM_API
        else if (!strcmp(arg, "--do-yuv-rescale"))
        {
            ctx->rescale_method = V4L2_YUV_RESCALE_EXT_TO_STD;
        }
#endif
        /*else
        {
            CSV_PARSE_CHECK_ERROR(ctx->in_file_path, "Unknown option " << arg);
        }*/
    }

    return 0;

error:
    //print_help();
    return -1;
}
