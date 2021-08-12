## Install

* jetson tx2/xavier nx
* apt install nvidia-l4t-jetson-multimedia-api 32.5.1-20210519111140
* cd /usr/src/jetson_multimedia_api/samples/
* git clone https://github.com/maxlapshin/l4t2-demo.git 17_transcoder
* cd 17_transcoder
* make
* ./transcoder H264 sample.h264 H264 out.h264 -ew 1920 -eh 1080 --input-nalu

