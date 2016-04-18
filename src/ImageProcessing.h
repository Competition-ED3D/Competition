#ifndef IMAGEPROCESSING_H
#define IMAGEPROCESSING_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <sstream>
#include <string>

#include <osg/Image>


using namespace cv;

using std::string;
using std::cout;
using std::endl;


int ImageProcessing(osg::Image* source);

#endif /* IMAGEPROCESSING_H */

