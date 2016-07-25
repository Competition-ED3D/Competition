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

struct InputParameters {
  string model_filename;

  float scanning_speed;
  float fps;

  string intrinsics_filename;

  float x_camera_coord;
  float y_camera_coord;
  float z_camera_coord;

  float x_camera_absolute;
  float y_camera_absolute;
  float z_camera_absolute;

  int camera_width;
  int camera_height;
  float pixel_size;
  float focal_length;

  float roi_height;
  float roi_top_start;
  float roi_bottom_start;

  float laser_distance;
  float laser_incline;
  float laser_aperture;
};

int Scanner(InputParameters* input_parameters);
void ComputeIntersections(
    osg::Vec3d start, float step_x, float threshold, osg::Node* model,
    float laser_incline, float laser_aperture, bool side,
    std::vector<osg::ref_ptr<osg::Vec3Array> >* intersections);
void ShowIntersections(std::vector<osg::ref_ptr<osg::Vec3Array> > intersections,
                       osg::Geode* intersection_line_geode);
float EuclideanDistance(osg::Vec3d point1, osg::Vec3d point2);
bool IntrinsicsParser(std::string filename, osg::Matrixf& intrinsics_matrix,
                      std::vector<double>& distortion_matrix);
void ProjectToImagePlane(std::vector<osg::ref_ptr<osg::Vec3Array> > intersections,
                         Mat intrinsics, struct InputParameters* input_parameters, 
                         Mat& output);

#endif /* SCANNER_H */