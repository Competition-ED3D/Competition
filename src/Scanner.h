#ifndef SCANNER_H
#define SCANNER_H

#include "ScreenCapture.h"

#include <osgDB/Registry>
#include <osgDB/XmlParser>

#include <math.h>

double EuclideanDistance(osg::Vec3d point1, osg::Vec3d point2);
bool nomeTantoCarino(osgViewer::Viewer *viewer, osg::ref_ptr<osg::GraphicsContext> pbuffer, 
                     osg::ref_ptr<osg::Camera> camera, unsigned int width, unsigned int height);
int Scanner();
void ComputeIntersections(osg::Vec3d start, double step_x, double step_y, double threshold,
                          osg::Node* model, double laser_incline_first, double laser_aperture_first, bool side,
                          std::vector<osg::ref_ptr<osg::Vec3Array> > *intersections, osg::Geode* intersection_line_geode);
void ShowIntersections (std::vector<osg::ref_ptr<osg::Vec3Array> > intersections,
                        osg::Vec4Array* intersection_line_color,
                        osg::Geode* intersection_line_geode);
bool IntrinsicsParser(std::string filename, osg::Matrixd &intrinsics_matrix, std::vector<double> &distortion_matrix);

struct MyReadCallback : public osgUtil::IntersectionVisitor::ReadCallback {
  virtual osg::Node* readNodeFile(const std::string& filename) {
    return osgDB::readNodeFile(filename);
  }
};

#endif /* SCANNER_H */

