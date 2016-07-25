#include "ImageProcessing.h"
#include "Scanner.h"

#define LASER_1 true
#define LASER_2 false

int image_counter = 0;

// Processes a screenshot retrieving the points of intersection between the
// laser and the model and transforming them in 3D points.
//
// Input parameters:
// source: the screenshot.
// intrinsics_matrix: the camera intrinsic matrix.
// input_parameters: the struct with all the input parameters.
// y_offset: the offset describing how much the two lasers and the camera have
// moved on the y-axis.
//
// Output parameters:
// point_cloud_points: the vector to fill with the 3D points.
int ImageProcessing(Mat& source, osg::Matrixf intrinsics_matrix,
                    struct InputParameters* input_parameters, 
                    vector<Point3f>& point_cloud_points) {
  // Defines the Mat with intrinsics parameters.
  Mat intrinsics = Mat::eye(3, 3, CV_32F);
  intrinsics.at<float>(0, 0) = intrinsics_matrix(0, 0);
  intrinsics.at<float>(0, 2) = intrinsics_matrix(0, 2);
  intrinsics.at<float>(1, 1) = intrinsics_matrix(1, 1);
  intrinsics.at<float>(1, 2) = intrinsics_matrix(1, 2);
  intrinsics.at<float>(2, 2) = intrinsics_matrix(2, 2);
  
  float roi_height = input_parameters->roi_height;
  float y_start_1 = input_parameters->roi_1_start;
  float y_start_2 = input_parameters->roi_2_start;

  // Extracts the left roi from the image.
  Rect region_of_interest = Rect(0, y_start_1, source.cols, roi_height);
  Mat image_roi_1(source, region_of_interest);

  // Extracts the right roi from the image.
  region_of_interest = Rect(0, y_start_2, source.cols, roi_height);
  Mat image_roi_2(source, region_of_interest);
  // Writes the rois to file.
  string name = "roi_left_" + std::to_string(image_counter) + ".png";
  imwrite(name, image_roi_1);
  name = "roi_right_" + std::to_string(image_counter) + ".png";
  imwrite(name, image_roi_2);
  image_counter++;

  vector<cv::Point3f> intersection_points_1;
  vector<cv::Point3f> intersection_points_2;

  // Stores the white pixels in a vector.
  LoadIntersectionPoints(image_roi_1, intersection_points_1);
  LoadIntersectionPoints(image_roi_2, intersection_points_2);
  
  // Converts the 2D pixel to 3D points.
  InsertPoints(intersection_points_1, intrinsics, input_parameters, LASER_1, 
               point_cloud_points);
  InsertPoints(intersection_points_2, intrinsics, input_parameters, LASER_2, 
               point_cloud_points);

  return 0;
}

// Stores points in a vector using the ones of the white pixels of an image as
// their coordinates.
//
// Input parameters:
// intersections: the image.
//
// Output parameters:
// intersection_points: the output vector.
void LoadIntersectionPoints(Mat intersections,
                            vector<Point3f>& intersection_points) {
  Point3f p;
  for (int i = 0; i < intersections.cols; i++) {
    for (int j = 0; j < intersections.rows; j++) {
      Scalar intensity = intersections.at<uchar>(j, i);
      // Checks if the pixel is white.
      if (intensity.val[0] == 255) {
        p.x = i;
        p.y = j;
        intersection_points.push_back(p);
        break;
      }
    }
  }
}

// Transforms 2D points (pixels) into 3D points using triangulation.
//
// Input paramaters:
// intersection_points: a vector containing the points of intersection between
// the laser and the model.
// intrinsics: the camera intrinsics matrix.
// input_parameters: the struct with all the input parameters.
// y_offset: the offset describing how much the two lasers and the camera has
// moved on the y-axis.
// roi: a boolean indicating the roi (left or right).
//
// Output parameters:
// point_cloud_points: the vector with the points in world coordinates.
void InsertPoints(vector<cv::Point3f> intersection_points, Mat intrinsics,
                  struct InputParameters* input_parameters, bool roi, 
                  vector<Point3f>& point_cloud_points) {
  float pixel_size = input_parameters->pixel_size;
  float baseline = input_parameters->laser_distance;
  float focal_length = input_parameters->focal_length;
  float y_start_1 = input_parameters->roi_1_start;
  float y_start_2 = input_parameters->roi_2_start;
  // The y coordinate of the principal point.
  int y_middle = intrinsics.at<float>(1, 2);
  float alpha = 90 - input_parameters->laser_incline;
  // Converts the angle from degrees to radians.
  float alpha_rad = 2 * M_PI * alpha / 360;

  float absolute_pixel_position, relative_pixel_position, pixel_position;
  Point3f point;

  // If no intersection points are found, then return.
  if (intersection_points.size() == 0) {
      return;
  }

  for (int i = 0; i < intersection_points.size(); i++) {
    // The y coordinate of the pixel relative to the roi.
    pixel_position = intersection_points.at(i).y;
    if (roi) {  // Left roi. TODO: sopra - LASER 1
      // The position in the y-axis of the current pixel relative to the entire screenshot.
      absolute_pixel_position = y_start_1 + pixel_position;
      // The distance in the y-axis from the current pixel and the principal point.
      relative_pixel_position = y_middle - absolute_pixel_position;
    } else {  // Right roi. LASER 2
      // The position in the y-axis of the current pixel relative to the entire screenshot.
      absolute_pixel_position = y_start_2 + pixel_position;
      // The distance in the y-axis from the current pixel and the principal point.
      relative_pixel_position = absolute_pixel_position - y_middle; 
    }
    point.x = intersection_points.at(i).x;
    point.y = absolute_pixel_position;
    // Computes the Z coordinate using triangulation.
    point.z = baseline * focal_length / (focal_length * tan(alpha_rad) -
                                         relative_pixel_position * pixel_size);
    
    // Converts from pixel to world coordinates.
    ConvertCoordinates(point, intrinsics, input_parameters);

    point_cloud_points.push_back(point);
  }
}

// Builds the point cloud.
//
// Input parameters:
// point_cloud_points: a vector containing the points to insert in the point
// cloud.
void BuildPointCloud(vector<Point3f> point_cloud_points) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointXYZRGB output_point;

  // Builds the point cloud.
  for (int i = 0; i < point_cloud_points.size(); i++) {
    // Sets RGB values of the point.
    output_point.r = 255;
    output_point.g = 0;
    output_point.b = 0;
    // Sets XYZ coordinates of the point.
    output_point.x = point_cloud_points.at(i).x;
    output_point.y = point_cloud_points.at(i).y;
    output_point.z = point_cloud_points.at(i).z;
    output->points.push_back(output_point);
  }

  output->width = 1;
  output->height = output->points.size();
  
  // Visualizes the point cloud if it is not empty.
  if(output->points.size() > 0) {
    cout << "Visualization... Press Q to exit." << endl;
    pcl::visualization::PCLVisualizer viewer("Reconstructed scene");
    viewer.setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
        output);
    viewer.addPointCloud<pcl::PointXYZRGB>(output, rgb, "cloud");
    // viewer.addCoordinateSystem(100.0, "cloud");
    // viewer.initCameraParameters();
    viewer.spin();

    // Saves the point cloud to file.
    pcl::io::savePCDFileASCII("output.pcd", *output);
  }
}

// Converts the coordinates of a point from screen to world coordinates.
//
// Input parameters:
// point: the point to convert.
// intrinsics: the camera intrinsics matrix.
// input_parameters: the struct with all the input parameters.
// y_offset: the offset describing how much the two lasers and the camera has
// moved on the y-axis.
void ConvertCoordinates(Point3f& point, Mat intrinsics,
                        struct InputParameters* input_parameters) {
  float point_coord[] = {point.x, point.y, 1};
  // The current absolute position of the camera.
  float camera_coord[] = {input_parameters->x_camera_absolute,
                          input_parameters->y_camera_absolute,
                          input_parameters->z_camera_absolute};

  // Defines the necessary matrices.
  Mat point_2D = Mat(3, 1, CV_32F, point_coord);
  Mat camera_center = Mat(3, 1, CV_32F, camera_coord);

  Mat inverted_intrinsics = intrinsics.inv();

  // Converts the point coordinates using the camera intrinsics and extrinsics 
  // parameters.
  Mat out = point.z * inverted_intrinsics * point_2D + camera_center;
  point.x = out.at<float>(0, 0);
  point.y = out.at<float>(0, 1);
  point.z = out.at<float>(0, 2);
}