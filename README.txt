This program simulates the scanning of a 3D object using a laser-camera-laser system and reconstructs said object in point-cloud format via a triangulation technique.

Compiling LaserScanner:

1. open the terminal in the folder where CMakeLists.txt is located;
2. type "cmake ." (without quotation marks) and press enter;
3. type "make" (without quotation marks) and press enter. The executable can be found in the same folder.


Usage of LaserScanner:

1. program input is done via an XML file (input.xml as default - you can also make your own, syntax must be consistent with input.xml). Refer to the example input file and the next paragraph of this readme for additional information as to how to configure the input file;
2. launch the program, specifying the path to the XML file as an input parameter (e.g.: ./main data/my_input_file.xml);
3. once the reconstruction is complete, the resulting point-cloud will be displayed. Pressing Q will exit the program. You will find the output point-cloud (if the save_point_cloud flag in the input file was set to 1) in the folder where the executable is located.


Input file syntax (threshold values in parentheses):

model_filename: path to the STL model to scan and reconstruct 
scanning_speed: movement speed (in mm/s) along the Y axis of the laser-camera-laser system (100-1000)
fps: camera framerate (100-500)
intrinsics_filename: path to the file storing the intrinsic parameters of the camera
x_camera_coord: starting X coordinate of the camera (relative to the centroid of the model) in the world frame of reference
y_camera_coord: starting Y coordinate of the camera (relative to the centroid of the model) in the world frame of reference
z_camera_coord: starting Z coordinate of the camera (relative to the centroid of the model) in the world frame of reference
camera_width: width (in pixel) of the images captured by the camera
camera_height: height (in pixel) of the images captured by the camera
pixel_size: length of a side (in μm) of the square pixels of the images captured by the camera
focal_length: focal length (in mm) of the camera
roi_height: height (in pixel) of the region of interests
roi_top_start: Y coordinate of the topmost row of the topmost roi in the image frame of reference
baseline: distance (in mm) between the camera and each of the lasers (500-800)
laser_incline: angle (in degrees) between the lasers and the plane the camera lays on (60-70)
laser_aperture: aperture angle (in degrees) of the laser (30-45)

factory_limitations: if set to 0, this flag allows the user to use arbitrary input values for the aforementioned parameters without being constrained by factory thresholds. When using "custom" input parameters, the correct functioning of the scanning/reconstruction process is NOT guaranteed
save_point_cloud: if set to 1, the point_cloud is saved to an output file before the program terminates execution


Make sure that a ROI fits in a single half (height-wise) of the image: that is to say, roi_height + roi_top_start should be less than camera_height/2.
The bottom roi is computed automatically using roi_height and roi_top_start as a reference. The rois are symmetric with respect to the middle (height-wise) of the image.

Please note that, when using the camera intrinsics provided by the company (stored in data/camera.xml), the camera-related parameters should be set as follows:

camera_width: 2024
camera_height: 1088
pixel_size: 5.5e-3
focal_length: 25

