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

int ImageProcessing(Mat& source, osg::Matrixf intrinsics_matrix,
                    struct InputParameters* input_parameters, 
                    vector<Point3f>& point_cloud_points);
void LoadIntersectionPoints(Mat intersections,
                            vector<Point3f>& intersection_points);
void InsertPoints(vector<Point3f> intersection_points, Mat intrinsics,
                  struct InputParameters* input_parameters, bool roi, 
                  vector<Point3f>& point_cloud_points);
void BuildPointCloud(vector<Point3f> point_cloud_points);
void ConvertCoordinates(Point3f& point, Mat intrinsics,
                        struct InputParameters* input_parameters);

#endif /* IMAGEPROCESSING_H */
