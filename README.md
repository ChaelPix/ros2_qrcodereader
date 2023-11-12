# ros2_qrcodereader
Read QrCode from Images.

- Install dependencies :
<pre>sudo apt-get install libopencv-dev ros-$ROS_DISTRO-cv-bridge libzbar-dev</pre>
- Clone the package
<pre>sgit clone git@github.com:ChaelPix/ros2_qrcodereader.git</pre>

<br>
<br>
  
# This package has 2 nodes :
1. `QRCodeReader() : Node("qr_code_reader")`

It's the main node which output qr code content as a string from an input image.
- Get Image from subscribed topic : `/camera/image_raw` 
- Try to read a QrCode
-  If successful, publish the QrCode content as a `std_msgs::msg::String` on the `/qr_code_content` topic.

+ To launch this node :
<pre>ros2 launch ros2_qrcodereader launch_qr_reader.launch.py</pre>

#
2. `QRImagePublisher() : Node("qr_image_publisher")`

It's a secondary node which pub images from files to `/camera/image_raw`. Useful if to test some images.
- Get the path from `image_path:=` argument, if no argument it'll load "bonjourros2.png".
- Publish the image every 1 second on `/camera/image_raw` topic.

+ To launch this node :
<pre>ros2 launch ros2_qrcodereader launch_qr_image_reader.launch.py</pre>
or
<pre>ros2 launch ros2_qrcodereader launch_qr_image_reader.launch.py image_path:=$HOME/path/to/image.png</pre>
