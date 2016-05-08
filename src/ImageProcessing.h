#ifndef IMAGEPROCESSING_H
#define IMAGEPROCESSING_H

#include <iostream>
#include <sstream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <osg/Image>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>



using namespace cv;

using std::string;
using std::cout;
using std::endl;
using std::vector;


int ImageProcessing(osg::Image* source, osg::Matrixd intrinsics_matrix, float y_offset, vector<Point3f>& point_cloud_points);
void InsertPoints(vector<Point3f> intersection_points, Mat intrinsics, float y_offset, vector<Point3f>& point_cloud_points);
void BuildPointCloud(vector<Point3f> point_cloud_points);
void ConvertCoordinates(Point3f& point, Mat intrinsics);


#endif /* IMAGEPROCESSING_H */

