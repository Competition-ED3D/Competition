#include "ImageProcessing.h"

int ImageProcessing(osg::Image* source) {

  //Mat image = imread(filename);
  Mat image(source->t(), source->s(), CV_8UC3);
  image.data = (uchar*) source->data();
  flip(image, image, 0);
  int roi_height = 400;
  int y_start = 100;
  Rect region_of_interest = Rect(0, y_start, image.cols, roi_height);
  Mat image_roi_left = image(region_of_interest);
  cout << "roi_left size " << image_roi_left.size() << endl;
  region_of_interest = Rect(0, image.rows - y_start - roi_height, image.cols, roi_height);
  Mat image_roi_right = image(region_of_interest);
  cout << "roi_right size " << image_roi_right.size() << endl;
  cout << "image.rows - y_start - roi_height " << image.rows - y_start - roi_height << endl;

  imwrite("roi_left.png", image_roi_left);
  imwrite("roi_right.png", image_roi_right);

  Scalar lbound = Scalar(0, 0, 200);
  Scalar ubound = Scalar(0, 0, 255);
  Mat intersections(2024, 1088, CV_8U);

  inRange(image_roi_left, lbound, ubound, intersections);

  imwrite("intersections.jpg", intersections);
  vector<cv::Point> intersection_points;
  for (int i = 0; i < intersections.cols; i++) {
    for (int j = 0; j < intersections.rows; j++) {
      Scalar intensity = intersections.at<uchar>(j, i);
      if (intensity.val[0] != 0)
        cout << "intensity.val[0] di (" << j << "), (" << i << "): " << intensity.val[0] << endl;
      if (intensity.val[0] == 255) {
        cout << "(j,i) (" << j << "), (" << i << ")" << endl;
        intersection_points.push_back(intersections.at<cv::Point>(j, i));
        continue;
      }
    }
  }
  cout << "intersection_points.size() " << intersection_points.size() << endl;

  InsertPoints(intersection_points);

  return 0;
}

void InsertPoints(vector<Point> intersection_points) {
  vector<Point3f> point_cloud_points;
  Point3f point;
  Point first, second;
  double laser_incline = 65;
  laser_incline = 2 * M_PI * laser_incline / 360;
  cout << "intersection_points.size() " << intersection_points.size() << endl;
  if (intersection_points.size() == 0)
    return;
  for (int i = 0; i < intersection_points.size() - 1; i++) {
    first = intersection_points.at(i);
    second = intersection_points.at(i + 1);
    if (second.x - first.x < 2) {
      if (abs(first.y - second.y) > 1) {
        float z_coord = abs(first.y - second.y) * tan(laser_incline);
        if (first.y < second.y) {
          point.x = first.x;
          point.y = first.y;
          point.z = z_coord;
        } else {
          point.x = second.x;
          point.y = second.y;
          point.z = z_coord;
        }
        point_cloud_points.push_back(point);
      }
    }
  }
}

void BuildPointCloud(vector<Point3f> point_cloud_points){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(
          new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointXYZRGB output_point;

  // Builds the pointcloud
  for (int i = 0; i < point_cloud_points.size(); i++) {
    cout<<"i: "<<i<<endl;
    // Sets RGB values and XYZ coordinates of the point
    output_point.r = 255;
    output_point.g = 0;
    output_point.b = 0;
    output_point.x = -point_cloud_points.at(i).x;
    output_point.y = -point_cloud_points.at(i).y;
    output_point.z = point_cloud_points.at(i).z;

    output->points.push_back(output_point);
  }
    
  cout << "Visualization... Press Q to exit." << endl;
  pcl::visualization::PCLVisualizer viewer("Reconstructed scene");
  viewer.setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
      output);
  viewer.addPointCloud<pcl::PointXYZRGB> (output, rgb, "cloud");
  viewer.addCoordinateSystem(1.0, "cloud");
  viewer.initCameraParameters();
  viewer.spin();
}

