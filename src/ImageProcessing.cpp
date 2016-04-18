#include "ImageProcessing.h"

int ImageProcessing(osg::Image* source) {
  
  //Mat image = imread(filename);
  Mat image(source->t(), source->s(), CV_8UC3); 
  image.data = (uchar*)source->data();
  flip(image,image,0);
  int roi_height = 400;
  int y_start = 100;
  Rect region_of_interest = Rect(0, y_start, image.cols, roi_height);
  Mat image_roi_left = image(region_of_interest);
  cout<<"roi_left size "<<image_roi_left.size()<<endl;
  region_of_interest = Rect(0, image.rows - y_start - roi_height, image.cols, roi_height);
  Mat image_roi_right = image(region_of_interest);
  cout<<"roi_right size "<<image_roi_right.size()<<endl;
  cout<<"image.rows - y_start - roi_height "<<image.rows - y_start - roi_height<<endl;
  
  //imwrite( "roi_left.png", image_roi_left );
  //imwrite( "roi_right.png", image_roi_right );
  //Mat image_roi_left = imread(filename);
  /*Scalar mean;
  Mat scan_region;
  for(int i=0; i<image_roi_left.cols; i=i+2) {
    for(int j=0; j<image_roi_left.rows; j=j+2) {
      scan_region = image_roi_left( Rect(i,j,2,2) );
      mean = mean(scan_region);
      if(mean[2])
    }
  }*/
  
  Scalar lbound = Scalar(0,0,255);
  Scalar ubound = Scalar(0,0,255);
  Mat intersections(2024,1088,CV_8U);
  
  inRange(image_roi_left,lbound,ubound,intersections);
  
  /*Mat scan_region;
  for(int i=0; i<intersections.cols; i=i+2) {
    for(int j=0; j<intersections.rows; j=j+2) {
      scan_region = intersections( Rect(i,j,2,2) );
      mean = mean(scan_region);
      if(mean[0] = 255/2){
        
      }
    }
  }*/
  
  imwrite( "intersections.png", intersections );
  

  return 0;
}


