# Drone flight control

This is a drone control flight for my Tello drone and Logitech RumblePad2 controller.

The `logitech.json` is a `gobot` compatible descriptor of the controller.

There is a mjpeg streaming server available on http://localhost:8080

You can do further detection by using [yolov5](https://github.com/ultralytics/yolov5) like so
```
python detect.py --source http://localhost:8080/mjpeg --view-im
```