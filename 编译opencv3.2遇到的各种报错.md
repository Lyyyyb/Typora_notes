# 编译opencv3.2遇到的各种报错

/usr/include/c++/9/cstdlib:75:15: fatal error: stdlib.h: 没有那个文件或目录

```bash

In file included from /home/lyb/opencv_builds/opencv-3.2.0/modules/videoio/src/cap_ffmpeg.cpp:47:
/home/lyb/opencv_builds/opencv-3.2.0/modules/videoio/src/cap_ffmpeg_impl.hpp: In function ‘AVStream* icv_add_video_stream_FFMPEG(AVFormatContext*, AVCodecID, int, int, int, double, int)’:
/home/lyb/opencv_builds/opencv-3.2.0/modules/videoio/src/cap_ffmpeg_impl.hpp:1541:21: error: ‘CODEC_FLAG_GLOBAL_HEADER’ was not declared in this scope; did you mean ‘AV_CODEC_FLAG_GLOBAL_HEADER’?
 1541 |         c->flags |= CODEC_FLAG_GLOBAL_HEADER;
      |                     ^~~~~~~~~~~~~~~~~~~~~~~~
      |                     AV_CODEC_FLAG_GLOBAL_HEADER
/home/lyb/opencv_builds/opencv-3.2.0/modules/videoio/src/cap_ffmpeg_impl.hpp: In function ‘int icv_av_write_frame_FFMPEG(AVFormatContext*, AVStream*, uint8_t*, uint32_t, AVFrame*)’:
/home/lyb/opencv_builds/opencv-3.2.0/modules/videoio/src/cap_ffmpeg_impl.hpp:1569:30: error: ‘AVFMT_RAWPICTURE’ was not declared in this scope
 1569 |     if (oc->oformat->flags & AVFMT_RAWPICTURE) {
      |                              ^~~~~~~~~~~~~~~~
/home/lyb/opencv_builds/opencv-3.2.0/modules/videoio/src/cap_ffmpeg_impl.hpp: In member function ‘void CvVideoWriter_FFMPEG::close()’:
/home/lyb/opencv_builds/opencv-3.2.0/modules/videoio/src/cap_ffmpeg_impl.hpp:1743:35: error: ‘AVFMT_RAWPICTURE’ was not declared in this scope
 1743 |         if( (oc->oformat->flags & AVFMT_RAWPICTURE) == 0 )
      |                                   ^~~~~~~~~~~~~~~~
/home/lyb/opencv_builds/opencv-3.2.0/modules/videoio/src/cap_ffmpeg_impl.hpp: In member function ‘bool CvVideoWriter_FFMPEG::open(const char*, int, double, int, int, bool)’:
/home/lyb/opencv_builds/opencv-3.2.0/modules/videoio/src/cap_ffmpeg_impl.hpp:2039:32: error: ‘AVFMT_RAWPICTURE’ was not declared in this scope
 2039 |     if (!(oc->oformat->flags & AVFMT_RAWPICTURE)) {
      |                                ^~~~~~~~~~~~~~~~
/home/lyb/opencv_builds/opencv-3.2.0/modules/videoio/src/cap_ffmpeg_impl.hpp: In static member function ‘static AVStream* OutputMediaStream_FFMPEG::addVideoStream(AVFormatContext*, AVCodecID, int, int, int, double, AVPixelFormat)’:
/home/lyb/opencv_builds/opencv-3.2.0/modules/videoio/src/cap_ffmpeg_impl.hpp:2338:25: error: ‘CODEC_FLAG_GLOBAL_HEADER’ was not declared in this scope; did you mean ‘AV_CODEC_FLAG_GLOBAL_HEADER’?
 2338 |             c->flags |= CODEC_FLAG_GLOBAL_HEADER;
      |                         ^~~~~~~~~~~~~~~~~~~~~~~~

```



```bash
make[2]: *** [modules/python3/CMakeFiles/opencv_python3.dir/build.make:329：modules/python3/CMakeFiles/opencv_python3.dir/__/src2/cv2.cpp.o] 错误 1
make[1]: *** [CMakeFiles/Makefile2:11856：modules/python3/CMakeFiles/opencv_python3.dir/all] 错误 2
make: *** [Makefile:163：all] 错误 2

```

```bash
bool pyopencv_to(PyObject* obj, String& value, const char* name)
{
    (void)name;
    if(!obj || obj == Py_None)
        return true;
    char* str = (char *)yString_AsString(obj);
    if(!str)
        return false;
    value = String(str);
    return true;
}
```

