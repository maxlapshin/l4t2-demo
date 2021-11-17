## Usage

* jetson tx2/xavier nx
* cd jetson_multimedia_api/samples/17_transcoder
* make
* works well
  * ./transcoder H264 ../../../video_samples/sample.h264 H264 out.h264 -ew 1920 -eh 1080 --input-nalu 
  * ./transcoder H264 ../../../video_samples/sample.h264 H264 out_cuda.h264 -ew 1920 -eh 1080 --input-nalu --cuda
* works bad
  * ./transcoder H264 ../../../video_samples/sample.h264 H264 out.h264 -ew 1920 -eh 1080 --input-nalu --live
  * ./transcoder H264 ../../../video_samples/sample.h264 H264 out_cuda.h264 -ew 1920 -eh 1080 --input-nalu --cuda --live

