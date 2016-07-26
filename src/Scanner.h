#ifndef SCANNER_H
#define SCANNER_H

#include "ImageProcessing.h"

#include <osg/Group>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineSegment>
#include <osg/io_utils>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>
#include <osg/LineWidth>
#include <osg/Point>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/Node>
#include <osgDB/ReadFile>
#include <osgDB/Registry>
#include <osgDB/XmlParser>

#include <math.h>


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

int Scanner(InputParameters* input_parameters);
void ComputeIntersections(
    osg::Vec3d start, float step_x, float threshold, osg::Node* model,
    float laser_incline, float laser_aperture, bool side,
    std::vector<osg::ref_ptr<osg::Vec3Array> >* intersections);
void ShowIntersections(std::vector<osg::ref_ptr<osg::Vec3Array> > intersections,
                       osg::Geode* intersection_line_geode);
void ProjectToImagePlane(
    std::vector<osg::ref_ptr<osg::Vec3Array> > intersections, Mat intrinsics,
    InputParameters* input_parameters, Mat& output);
float EuclideanDistance(osg::Vec3d point1, osg::Vec3d point2);
bool IntrinsicsParser(std::string filename, osg::Matrixf& intrinsics_matrix,
                      std::vector<double>& distortion_matrix);

#endif /* SCANNER_H */