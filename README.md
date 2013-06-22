group-perception
================

Repository for the perception group homework

Installation Instructions
-------------------------

1. Checkout the sourcecode to your workspace and update:
``rosws set group_perception "https://github.com/ai-seminar/group-perception.git" --git && rosws update group_perception``
2. Build the stack: ``roscd group_perception && rosmake``
3. You can now start the perception_server and client chain using ``roslaunch $(rospack find perception_client_pkg)/run.launch``
4. Play a .bag file containing rgbd camera data or get data from a real rgbd camera.
5. Watch the console output of client and server.
