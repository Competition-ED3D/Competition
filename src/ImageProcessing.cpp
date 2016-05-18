#include "ImageProcessing.h"

int image_counter = 0;

int ImageProcessing(osg::Image* source, osg::Matrixd intrinsics_matrix, float y_offset, float laser_incline, vector<Point3f>& point_cloud_points) {

  Mat intrinsics = Mat::eye(3, 3, CV_64F);
  intrinsics.at<double>(0, 0) = intrinsics_matrix(0, 0);
  intrinsics.at<double>(0, 2) = intrinsics_matrix(0, 2);
  intrinsics.at<double>(1, 1) = intrinsics_matrix(1, 1);
  intrinsics.at<double>(1, 2) = intrinsics_matrix(1, 2);
  intrinsics.at<double>(2, 2) = intrinsics_matrix(2, 2);

  Mat image(source->t(), source->s(), CV_8UC3);
  image.data = (uchar*) source->data();
  flip(image, image, 0);
  //int roi_height = 400;
  int roi_height = 600;
  int y_start = 100;
  Rect region_of_interest = Rect(0, y_start, image.cols, roi_height);
  Mat image_roi_left = image(region_of_interest);
  //cout << "roi_left size " << image_roi_left.size() << endl;
  region_of_interest = Rect(0, image.rows - y_start - roi_height, image.cols, roi_height);
  //Mat image_roi_right = image(region_of_interest);
  //cout << "roi_right size " << image_roi_right.size() << endl;
  //cout << "image.rows - y_start - roi_height " << image.rows - y_start - roi_height << endl;

  string name = "roi_left_" + std::to_string(image_counter) + ".png";
  imwrite(name, image_roi_left);
  //name = "roi_right_" + std::to_string(image_counter) + ".png";
  //imwrite(name, image_roi_right);
  image_counter++;
  
  Scalar lbound = Scalar(0, 0, 200);
  Scalar ubound = Scalar(0, 0, 255);
  Mat intersections(2024, 1088, CV_8U);

  inRange(image_roi_left, lbound, ubound, intersections);
  //inRange(image_roi_right, lbound, ubound, intersections);

  imwrite("intersections.jpg", intersections);
  vector<cv::Point3f> intersection_points;
  for (int i = 0; i < intersections.cols; i++) {
    for (int j = 0; j < intersections.rows; j++) {
      Scalar intensity = intersections.at<uchar>(j, i);
      if (intensity.val[0] == 255) {
        Point3f p;
        p.x = i;
        p.y = j;
        intersection_points.push_back(p);
        break;
      }
    }
  }

  //InsertPoints(intersection_points, intrinsics, y_offset, laser_incline, point_cloud_points);
  Prova(intersection_points, intrinsics, y_offset, laser_incline, point_cloud_points);

  return 0;
}

void Prova (vector<Point3f> intersection_points, Mat intrinsics, float y_offset, float laser_incline, vector<Point3f>& point_cloud_points) {
    float baseline = 500/5.5e-3; //Converto in pixel
    float focal_length = 25/5.5e-3; //Converto in pixel
    float pixel_position;
    float absolute_pixel_position;
    float alfa = 90-laser_incline;
    float z_coord;
    float alfa_rad = 2 * M_PI * alfa / 360;
    float roi_height = 600;
    Point3f point;
    if (intersection_points.size() == 0)
        return;
    for (int i = 0; i < intersection_points.size() - 1; i++) {
        pixel_position = intersection_points.at(i).y;
        absolute_pixel_position = (roi_height - pixel_position) + (544-roi_height);
        z_coord = baseline*focal_length/(focal_length*tan(alfa_rad)-pixel_position);
        z_coord = z_coord*5.5e-3; // converto in mm
        point.x = intersection_points.at(i).x;
        point.y = intersection_points.at(i).y;
        point.z = z_coord;
        ConvertCoordinates(point, intrinsics);
        point.y = point.y + y_offset;
        cout << "Coordinate punto che sta per essere inserito: (" << point.x << ", " << point.y << ", " << point.z << ")" << endl;
        point_cloud_points.push_back(point);
    }

}

void InsertPoints(vector<Point3f> intersection_points, Mat intrinsics, float y_offset, float laser_incline, vector<Point3f>& point_cloud_points) {
  Point3f point;
  Point3f first, second;
  double laser_incline_rad = 2 * M_PI * laser_incline / 360;
  if (intersection_points.size() == 0)
    return;
  //float offset = 0;
  Point3f off;
  
  //y_offset = y_offset/950;
  
  Point3f conv;
  conv.x = 0;
  conv.y = 600;
  //ConvertCoordinates(conv, intrinsics);
  
  off.x = intersection_points.at(0).x;
  off.y = intersection_points.at(0).y;
  //ConvertCoordinates(off, intrinsics);
  /*Point3f prova;
  prova.x = 0;
  prova.y = fabs(height.y - off.y);
  ConvertCoordinates(prova,intrinsics);*/
  //float offset = off.y;
  float offset = fabs(conv.y - off.y) * tan(laser_incline_rad); //setto l'offset triangolando il punto in
                                                              //basso a sinistra nella roi con il primo dell'intersezione
  //float offset = prova.y * tan(laser_incline_rad);
  //cout << "offset iniziale: " << offset << endl;
  for (int i = 0; i < intersection_points.size() - 1; i++) {
    first = intersection_points.at(i);
    second = intersection_points.at(i + 1);
    //cout << "first: (" << first.x << ", " << first.y << ")" << endl;
    //cout << "second: (" << second.x << ", " << second.y << ")" << endl;
    float z_coord;
    
    if (fabs(second.y - first.y) >= 1) { // sono su righe diverse
      if (fabs(first.x - second.x) <= 1) { // sono su colonne adiacenti
        //ConvertCoordinates(first, intrinsics);
        //ConvertCoordinates(second, intrinsics);
        point.x = second.x; // do la coordinata del secondo, first e second sono adiacenti
        point.y = second.y; // la y è uguale sia per first che per second (stessa riga)
        z_coord = fabs(first.y - second.y) * tan(laser_incline_rad);
        
        if (first.y > second.y) { // y indica la riga, in questo caso il primo è più in basso del secondo      
          point.z = z_coord + offset;
          offset = z_coord + offset;
        } else { 
          point.z = offset - z_coord;
          offset = offset - z_coord;
        }
      } else { // sono su righe diverse ma colonne non adiacenti
      continue;
      }
    } else if (z_coord != 0) { // sono sulla stessa riga 
      //ConvertCoordinates(first, intrinsics);
      //ConvertCoordinates(second, intrinsics);
      point.x = second.x; // do la coordinata del secondo, first e second sono adiacenti
      point.y = second.y; // la y è uguale sia per first che per second (stessa riga)
      point.z = offset;
    } else {// se i primi due punti della roi sono sulla stessa riga non ho informazioni sull'altezza per nessuno dei
            // dei due, non posso inserirli
      continue;
    }
    point.z = point.z*5.5e-3;
    ConvertCoordinates(point, intrinsics);
    //cout << "point.y: " << point.y << endl;
    point.y = point.y + (y_offset/11.0);
    //cout << "y_offset dopo: " << y_offset/11.0 << endl;
    //cout << "point.y - dopo: " << point.y << endl;
    
    cout << "Coordinate punto che sta per essere inserito: (" << point.x << ", " << point.y << ", " << point.z << ")" << endl;
    //cout << "z punto: " << point.z << endl;
    point_cloud_points.push_back(point);
  }
}

void BuildPointCloud(vector<Point3f> point_cloud_points) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(
          new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointXYZRGB output_point;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr piano(
          new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointXYZRGB punto_piano;
  /*for (double i = -0.75; i < 1.00; i = i + 0.1) {
    for (double j = -0.75; j < 1.00; j = j + 0.1) {
      punto_piano.r = 0;
      punto_piano.g = 0;
      punto_piano.b = 255;
      punto_piano.x = -i;
      punto_piano.y = -j;
      punto_piano.z = 0;
      piano->points.push_back(punto_piano);
    }
  }*/
  // Builds the pointcloud
  for (int i = 0; i < point_cloud_points.size(); i++) {
    // Sets RGB values and XYZ coordinates of the point
    output_point.r = 255;
    output_point.g = 0;
    output_point.b = 0;
    output_point.x = point_cloud_points.at(i).x;
    output_point.y = point_cloud_points.at(i).y;
    output_point.z = point_cloud_points.at(i).z;
    output->points.push_back(output_point);
  }
  
  output->width = 1;
  output->height = output->points.size();
  
  cout << "Visualization... Press Q to exit." << endl;
  pcl::visualization::PCLVisualizer viewer("Reconstructed scene");
  viewer.setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
          output);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(
          piano);
  viewer.addPointCloud<pcl::PointXYZRGB> (output, rgb, "cloud");
  //viewer.addPointCloud<pcl::PointXYZRGB> (piano, rgb2, "piano");
  viewer.addCoordinateSystem(100.0, "cloud");
  //viewer.initCameraParameters();
  viewer.spin();
  
  pcl::io::savePCDFileASCII("output.pcd", *output);
}

void ConvertCoordinates(Point3f& point, Mat intrinsics) {
  double point_coord[] = {point.x, point.y, point.z};
  //double point_coord[] = {x, y, point.z};
  //Mat converted_point = Mat(3, 1, CV_64F, point_coord);
  //Mat inverted_intrinsics = intrinsics.inv();
  
  double cx = intrinsics.at<double>(2, 0);
  double cy = intrinsics.at<double>(2, 1);
  double fx = intrinsics.at<double>(0, 0);
  double fy = intrinsics.at<double>(1, 1);
  
  point.x = (point.x - cx) * point.z / fx;
  point.y = (point.y - cy) * point.z / fy;
  
  //double point_coord[] = {point.x - cx, point.y - cy, fx};
  Mat converted_point = Mat(3, 1, CV_64F, point_coord);
  /*for(int i=0;i<inverted_intrinsics.rows;i++)
      for(int j=0;j<inverted_intrinsics.cols;j++)
          cout<<"mat("<<i<<")("<<j<<"): "<<inverted_intrinsics.at<double>(i,j)<<endl;*/
 /* cout<<"point x y z "<<point.x<<" "<<point.y<<" "<<point.z<<endl;
  cout<<"conv_point(0)(0)"<<converted_point.at<double>(0,0)<<endl;
  cout<<"conv_point(1)(0)"<<converted_point.at<double>(1,0)<<endl;
  cout<<"conv_point(2)(0)"<<converted_point.at<double>(2,0)<<endl;*/

  //Mat out = inverted_intrinsics * converted_point;
  
  //out = extrinsics.inv() * out;
  
  /*point.x = out.at<double>(0, 0)/point.z;
  point.y = out.at<double>(0, 1)/point.z;
  point.z = out.at<double>(0, 2)/point.z;*/
  //cout<<"point x y z dopo "<<point.x<<" "<<point.y<<" "<<point.z<<endl;
  //  getchar();

}

