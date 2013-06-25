group-perception
================

Repository for the perception group homework

Installation Instructions
-------------------------

1. Checkout the sourcecode to your workspace and update:
  1. ``rosws set group_perception "https://github.com/ai-seminar/group-perception.git" --git`` 
  2. ``rosws update group_perception``
  3. ``source ~/.bashrc``
2. Build the stack: ``roscd group_perception && rosmake``
3. You can now start the perception_server chain using ``roslaunch perception_client_pkg run.launch``
4. Play a .bag file containing rgbd camera data or get data from a real rgbd camera.
5. Launch the perception client using ``rosrun perception_client_pkg perception_client``
6. Watch the console output of client and server.


The console output for the server should look similar to this:


    The console output for the server should look similar to this:
    
        Loop start: 2013-Jun-25 14:16:25.015375
    
    Convex hull of 956 points in 3-d:
    
      Number of vertices: 65
      Number of facets: 126
    
    Statistics for:  | qhull FA
    
      Number of points processed: 74
      Number of hyperplanes created: 329
      Number of distance tests for qhull: 13111
      CPU seconds to compute hull (after input): 0.01
      Total facet area:   0.12716812
      Total volume:       0.0012292913
    
    The centroid vector is: 0.0987291,-0.120297,0.838274,0
    
    Convex hull of 769 points in 3-d:
    
      Number of vertices: 75
      Number of facets: 146
    
    Statistics for:  | qhull FA
    
      Number of points processed: 85
      Number of hyperplanes created: 372
      Number of distance tests for qhull: 10114
      CPU seconds to compute hull (after input):  0
      Total facet area:   0.08954625
      Total volume:       0.0014527954
    
    The centroid vector is: -0.276305,-0.0695283,0.718886,0
    
    Convex hull of 251 points in 3-d:
    
      Number of vertices: 74
      Number of facets: 144
    
    Statistics for:  | qhull FA
    
      Number of points processed: 78
      Number of hyperplanes created: 329
      Number of distance tests for qhull: 3450
      CPU seconds to compute hull (after input):  0
      Total facet area:   0.031462079
      Total volume:       0.00041752768
    
    The centroid vector is: 0.166426,0.113451,0.638591,0
    [ INFO] [1372169785.020197814]: Wrote a new point cloud: size = 307200
    


The console output of the perception_client shows a list of the perceived objects sorted by their volume:


    [ INFO] [1372169785.041005090]: Cluster Service call successful
    [ INFO] [1372169785.041054682]: List size: 3
    [ INFO] [1372169785.041071255]: ID of perceived object is: 16
    [ INFO] [1372169785.041093589]: Volume of perceived object is: 0.001453
    [ INFO] [1372169785.041123240]: Centroid(x) of perceived object is: -0.276305 , -0.069528 , 0.718886 
    [ INFO] [1372169785.041201537]: ID of perceived object is: 15
    [ INFO] [1372169785.041236152]: Volume of perceived object is: 0.001229
    [ INFO] [1372169785.041268426]: Centroid(x) of perceived object is: 0.098729 , -0.120297 , 0.838274 
    [ INFO] [1372169785.041300384]: ID of perceived object is: 17
    [ INFO] [1372169785.041379761]: Volume of perceived object is: 0.000418
    [ INFO] [1372169785.041449909]: Centroid(x) of perceived object is: 0.166426 , 0.113451 , 0.638591 
    [ INFO] [1372169785.041525761]: ------------------------------------------------------------



