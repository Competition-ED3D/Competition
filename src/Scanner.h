#ifndef SCANNER_H
#define SCANNER_H

#include "ScreenCapture.h"

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
  
  int camera_width;
  int camera_height;
  float pixel_size;
  float focal_length;
  
  float roi_height;
  float left_roi_start;
  float right_roi_start;
  
  float laser_distance;
  float laser_incline;
  float laser_aperture;
};

float EuclideanDistance(osg::Vec3d point1, osg::Vec3d point2);
int Scanner(InputParameters *input_parameters);
void ComputeIntersections(osg::Vec3d start, float step_x, float step_y, float threshold,
                          osg::Node* model, float laser_incline, float laser_aperture, bool side,
                          std::vector<osg::ref_ptr<osg::Vec3Array> > *intersections);
void ShowIntersections (std::vector<osg::ref_ptr<osg::Vec3Array> > intersections,
                        osg::Geode* intersection_line_geode);
bool IntrinsicsParser(std::string filename, osg::Matrixf &intrinsics_matrix, std::vector<double> &distortion_matrix);

struct MyReadCallback : public osgUtil::IntersectionVisitor::ReadCallback {
  virtual osg::Node* readNodeFile(const std::string& filename) {
    return osgDB::readNodeFile(filename);
  }
};

#endif /* SCANNER_H */