#include "ImageProcessing.h"

int ImageProcessing(osg::Image* source, osg::Matrixd intrinsics_matrix) {

  Mat intrinsics =Mat::eye(3, 3,  CV_64F);
  intrinsics.at<double>(0,0) = intrinsics_matrix(0,0);
  intrinsics.at<double>(0,2) = intrinsics_matrix(0,2);
  intrinsics.at<double>(1,1) = intrinsics_matrix(1,1);
  intrinsics.at<double>(1,2) = intrinsics_matrix(1,2);
  intrinsics.at<double>(2,2) = intrinsics_matrix(2,2);
    
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
  vector<cv::Point3f> intersection_points;
  for (int i = 0; i < intersections.cols; i++) {
    for (int j = 0; j < intersections.rows; j++) {
      Scalar intensity = intersections.at<uchar>(j, i);
      if (intensity.val[0] != 0)
        //cout << "intensity.val[0] di (" << j << "), (" << i << "): " << intensity.val[0] << endl;
      if (intensity.val[0] == 255) {
        //cout << "(j,i) (" << j << "), (" << i << ")" << endl;
        //cv::Point3f p = intersections.at<cv::Point3f>(j,i);
        Point3f p;
        p.x = i;
        p.y = j;
        cout << "p.y" << p.y<<endl;
        intersection_points.push_back(p);
        break;
      }
    }
  }

  InsertPoints(intersection_points, intrinsics);

  return 0;
}

void InsertPoints(vector<Point3f> intersection_points, Mat intrinsics) {
  vector<Point3f> point_cloud_points;
  Point3f point;
  Point3f first, second;
  double laser_incline = 65;
  laser_incline = 2 * M_PI * laser_incline / 360;
  cout << "intersection_points.size() " << intersection_points.size() << endl;
  if (intersection_points.size() == 0)
    return;
  for (int i = 0; i < intersection_points.size() - 1; i++) {
    first = intersection_points.at(i);
    second = intersection_points.at(i + 1);
    cout<<"second.y - first.y "<<abs(second.y-first.y)<<endl;
    float z_coord;
    int offset = 0;
    if (abs(second.y - first.y) >= 1) { // sono su righe diverse
      cout<<"second.x - first.x "<<abs(second.x-first.x)<<endl;
      if (abs(first.x - second.x) <= 1) { // sono su colonne adiacenti
        z_coord = abs(first.y - second.y) * tan(laser_incline);
        cout<<"z coord " << z_coord<<endl;
        if (first.y > second.y) {
          point.x = first.x;
          point.y = first.y;
          point.z = z_coord + offset;
          offset = offset-z_coord;
        } else {
          point.x = second.x;
          point.y = second.y;
          point.z = z_coord + offset;
          offset = offset+z_coord;
        }
      }
    } else if(z_coord!=0) { // sono sulla stessa riga 
      point.x = second.x; // do la coordinata del secondo, first e second sono adiacenti
      point.y = second.y; // la y è uguale sia per first che per second (stessa riga)
      point.z = z_coord;
    }
    point_cloud_points.push_back(point);
  }
  BuildPointCloud(point_cloud_points, intrinsics);
}

void BuildPointCloud(vector<Point3f> point_cloud_points, Mat intrinsics){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(
          new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointXYZRGB output_point;
    cout<<"point_cloud_size(): "<<point_cloud_points.size()<<endl;
    
    ConvertCoordinates(point_cloud_points,intrinsics);
        cout<<"point_cloud_size() dopo: "<<point_cloud_points.size()<<endl;

    
  // Builds the pointcloud
  for (int i = 0; i < point_cloud_points.size(); i++) {
    // Sets RGB values and XYZ coordinates of the point
    output_point.r = 255;
    output_point.g = 0;
    output_point.b = 0;
    output_point.x = -point_cloud_points.at(i).x;
    output_point.y = point_cloud_points.at(i).y;
    output_point.z = point_cloud_points.at(i).z;   
    cout<<"output_point.z "<<output_point.z<<endl;
    if(output_point.z < 3)
      output->points.push_back(output_point);
  }
    
  cout << "Visualization... Press Q to exit." << endl;
  pcl::visualization::PCLVisualizer viewer("Reconstructed scene");
  viewer.setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
      output);
  viewer.addPointCloud<pcl::PointXYZRGB> (output, rgb, "cloud");
  viewer.addCoordinateSystem(0.1, "cloud");
  //viewer.initCameraParameters();
  viewer.spin();
}

void ConvertCoordinates(vector<Point3f>& point_cloud_points, Mat intrinsics){
  vector<Point3f> output;
  for(int i=0; i<point_cloud_points.size(); i++) {
    Point3f p = point_cloud_points.at(i);
    cout << "p.x " << p.x << " p.y " << p.y << " p.z " << p.z << endl;
    double point_coord[] = {p.x, p.y, p.z};
    Mat point = Mat(3, 1, CV_64F, point_coord);
    //Mat out = intrinsics * point;
    Mat inverted_intrinsics = intrinsics.inv();
    Mat out = inverted_intrinsics * point;
    p.x = out.at<double>(0,0);
    p.y = out.at<double>(0,1);
    p.z = out.at<double>(0,2);
    cout << "Coordinate dopo: p.x " << p.x << " p.y " << p.y << " p.z " << p.z << endl;
    output.push_back(p);
  }
  //cv::Mat v = intrinsics * cv::Mat(point_cloud_points, false); /* Vec is just wrapped, no copying is performed */
  //v.copyTo( point_cloud_points );
  // P now contains the result
  for(int i=0; i<output.size(); i++) {
    point_cloud_points[i] = output[i];
  }
}

