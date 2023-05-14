# ROS2_camera_pubsub

Just reading frames from the camera, publishing them to the camera_feed topic and showing them in the subscriber node has a an average frame rate of 3.999845FPS. I wanted to see what effect compressing the images before publishing had on the performance.

I used jpeg compression in the publisher node before publishing. The resulting frame rate was 3.680342FPS when compression was used. The additional steps of processing meant a reduction in frame rate.

I then used a Haar Cascading CLassifier to detect faces.

