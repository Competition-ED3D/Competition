#ifndef SCANNER_H
#define SCANNER_H

#include "ScreenCapture.h"

#include <osgDB/Registry>
#include <osgDB/XmlParser>

#include <math.h>

struct InputParameters {
  string model_filename;
    
  double scanning_speed;
  double fps;
  
  string intrinsics_filename;
  
  double x_camera_coord;
  double y_camera_coord;
  double z_camera_coord;
  
  double camera_width;
  double camera_height;
  double pixel_size;
  double focal_length;
  
  double roi_height;
  double left_roi_start;
  double right_roi_start;
  
  double laser_distance;
  double laser_incline;
  double laser_aperture;
};

double EuclideanDistance(osg::Vec3d point1, osg::Vec3d point2);
bool nomeTantoCarino(osgViewer::Viewer *viewer, osg::ref_ptr<osg::GraphicsContext> pbuffer, 
                     osg::ref_ptr<osg::Camera> camera, unsigned int width, unsigned int height);
int Scanner(InputParameters *input_parameters);
void ComputeIntersections(osg::Vec3d start, double step_x, double step_y, double threshold,
                          osg::Node* model, double laser_incline, double laser_aperture, bool side,
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