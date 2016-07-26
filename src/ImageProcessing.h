#ifndef IMAGEPROCESSING_H
#define IMAGEPROCESSING_H

#include <iostream>
#include <sstream>
#include <string>

#include <osg/Matrixf>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace cv;

using std::string;
using std::cout;
using std::endl;
using std::vector;

// Contains the input parameters specified by the user.
struct InputParameters {
  // Filename of the model.
  string model_filename;

  // Speed at which the system moves along the y axis (mm/s) and framerate.
  float scanning_speed;
  float fps;

  // Filename of the file detailing the intrinsics matrix.
  string intrinsics_filename;

  // Starting coordinates of the camera, relative to the model.
  float x_camera_coord;
  float y_camera_coord;
  float z_camera_coord;

  // World coordinates of the camera.
  float x_camera_absolute;
  float y_camera_absolute;
  float z_camera_absolute;

  // Camera resolution, pixel size and focal length.
  int camera_width;
  int camera_height;
  float pixel_size;
  float focal_length;

  // Roi height.
  float roi_height;
  // Topmost y coordinate of each roi.
  float roi_top_start;
  float roi_bottom_start;

  // Distance between the camera and each laser.
  float laser_distance;
  // Laser angle with respect to the horizon.
  float laser_incline;
  // Aperture angle of the laser.
  float laser_aperture;

  // Optional flag that dictates whether the user is allowed to use arbitrary
  // system parameters (false) or must abide by the factory thresholds (true).
  bool factory_limitations;

  // Optional flag that determines whether to save the final point cloud (true)
  // or not.
  bool save_point_cloud;
};

int ImageProcessing(Mat& source, osg::Matrixf intrinsics_matrix,
                    InputParameters* input_parameters,
                    vector<Point3f>& point_cloud_points);
void LoadIntersectionPoints(Mat intersections,
                            vector<Point3f>& intersection_points);
void InsertPoints(vector<Point3f> intersection_points, Mat intrinsics,
                  InputParameters* input_parameters, bool roi,
                  vector<Point3f>& point_cloud_points);
void BuildPointCloud(vector<Point3f> point_cloud_points,
                     InputParameters* input_parameters);
void ConvertCoordinates(Point3f& point, Mat intrinsics,
                       InputParameters* input_parameters);

#endif /* IMAGEPROCESSING_H */
