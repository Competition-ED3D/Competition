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