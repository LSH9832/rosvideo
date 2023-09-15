# rosvideo: build bridge between ros Image topic and rtmp/rtsp stream

## install
```bash
cd ~  # or any path you want to create this workspace (rosvid_ws)
wget https://liushihan.site/download/rosvideo.bash && bash rosvideo.bash
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
