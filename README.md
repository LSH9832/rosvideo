# rosvideo: build bridge between ros Image topic and rtmp/rtsp stream

## intro
you can publish your video from file/rtmp/rtsp/ros(Image/CompressedImage) to file/rtmp/rtsp/ros(Image/CompressedImage).

## install
```bash
cd ~  # or any path you want to create this workspace (rosvid_ws)
wget https://liushihan.site/download/rosvideo.bash && bash rosvideo.bash
```
**if you need a rtsp/rtmp server, it is recomended to use this**

```bash
wget https://liushihan.site/download/mediaserver.bash && bash mediaserver.bash
```

just modify the function "void process_image()" in src/publish.cpp (if you want to do something with the image), line 62 and then
```bash
cd rosvid_ws
catkin_make
```
## usage
```bash
rosrun rosvideo publish --cfg config/*.yaml

# help
rosrun rosvideo publish -?
```
