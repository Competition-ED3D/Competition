#include "ImageProcessing.h"
#include "Scanner.h"

#define LASER_1 true
#define LASER_2 false

// Processes an image retrieving the white pixels (that is the points of
// intersection between the laser and the model) from it and transforming them
// in 3D points.
//
// Input parameters:
// source: the image.
// intrinsics_matrix: the camera intrinsic matrix.
// input_parameters: struct containing the input parameters specified by the
// user.
//
// Output parameters:
// point_cloud_points: the vector to fill with the 3D points.
int ImageProcessing(const Mat& source, osg::Matrixf intrinsics_matrix,
                    InputParameters* input_parameters,
                    vector<Point3f>& point_cloud_points) {
  // Defines the Mat with intrinsics parameters.
  Mat intrinsics = Mat::eye(3, 3, CV_32F);
  intrinsics.at<float>(0, 0) = intrinsics_matrix(0, 0);
  intrinsics.at<float>(0, 2) = intrinsics_matrix(0, 2);
  intrinsics.at<float>(1, 1) = intrinsics_matrix(1, 1);
  intrinsics.at<float>(1, 2) = intrinsics_matrix(1, 2);
  intrinsics.at<float>(2, 2) = intrinsics_matrix(2, 2);

  float roi_height = input_parameters->roi_height;
  float y_start_top = input_parameters->roi_top_start;
  float y_start_bottom = input_parameters->roi_bottom_start;

  // Extracts the top roi from the image.
  Rect region_of_interest = Rect(0, y_start_top, source.cols, roi_height);
  Mat image_roi_top(source, region_of_interest);

  // Extracts the bottom roi from the image.
  region_of_interest = Rect(0, y_start_bottom, source.cols, roi_height);
  Mat image_roi_bottom(source, region_of_interest);

  vector<Point3f> intersection_points_top;
  vector<Point3f> intersection_points_bottom;

  // Stores the white pixels (that is the intersection points) in a vector.
  LoadIntersectionPoints(image_roi_top, intersection_points_top);
  LoadIntersectionPoints(image_roi_bottom, intersection_points_bottom);

  // Converts the 2D pixel to 3D points.
  InsertPoints(intersection_points_top, intrinsics, input_parameters, LASER_1,
               point_cloud_points);
  InsertPoints(intersection_points_bottom, intrinsics, input_parameters,
               LASER_2, point_cloud_points);

  return 0;
}

// Stores points in a vector using the ones of the white pixels of the image as
// their coordinates.
//
// Input parameters:
// intersections: the image.
//
// Output parameters:
// intersection_points: the output vector.
void LoadIntersectionPoints(const Mat& intersections,
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
// intersection_points: vector containing the points of intersection between
// the laser and the model.
// intrinsics: the camera intrinsics matrix.
// input_parameters: struct containing the input parameters specified by the
// user.
// roi: boolean indicating the roi (top or bottom).
//
// Output parameters:
// point_cloud_points: the vector with the points in world coordinates.
void InsertPoints(const vector<Point3f>& intersection_points, Mat intrinsics,
                  InputParameters* input_parameters, bool roi,
                  vector<Point3f>& point_cloud_points) {
  // Returns if no intersection points are found.
  if (intersection_points.size() == 0)
    return;

  float pixel_size = input_parameters->pixel_size;
  float baseline = input_parameters->baseline;
  float focal_length = input_parameters->focal_length;
  float y_start_top = input_parameters->roi_top_start;
  float y_start_bottom = input_parameters->roi_bottom_start;

  // The y coordinate of the principal point.
  int c_y = intrinsics.at<float>(1, 2);

  float alpha = 90 - input_parameters->laser_incline;
  // Converts the angle from degrees to radians.
  float alpha_rad = 2 * M_PI * alpha / 360;

  float absolute_pixel_position, relative_pixel_position, pixel_position;
  Point3f point;

  for (int i = 0; i < intersection_points.size(); i++) {
    // The Y coordinate of the pixel relative to the roi.
    pixel_position = intersection_points.at(i).y;
    if (roi) {  // Top roi.
      // The position on the Y axis of the current pixel relative to the entire
      // image.
      absolute_pixel_position = y_start_top + pixel_position;
      // The distance on the Y axis from the current pixel and the principal
      // point.
      relative_pixel_position = c_y - absolute_pixel_position;
    } else {  // Bottom roi.
      // The position on the Y axis of the current pixel relative to the entire
      // image.
      absolute_pixel_position = y_start_bottom + pixel_position;
      // The distance on the Y axis from the current pixel and the principal
      // point.
      relative_pixel_position = absolute_pixel_position - c_y;
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

// Converts the coordinates of a point from pixel to world coordinates.
//
// Input parameters:
// point: the point to convert.
// intrinsics: the camera intrinsics matrix.
// input_parameters: struct containing the input parameters specified by the
// user.
void ConvertCoordinates(Point3f& point, Mat intrinsics,
                        InputParameters* input_parameters) {
  float point_coord[] = {point.x, point.y, 1};
  // The current absolute position of the camera.
  float camera_coord[] = {input_parameters->x_camera_absolute,
                          input_parameters->y_camera_absolute,
                          input_parameters->z_camera_absolute};

  // Defines the necessary matrices.
  Mat point_2D = Mat(3, 1, CV_32F, point_coord);
  Mat camera_center = Mat(3, 1, CV_32F, camera_coord);

  Mat inverted_intrinsics = intrinsics.inv();

  // Rotation matrix. The camera has the Z axis pointing downward, which
  // requires a rotation of 180 degrees around the X axis to adjust it so that
  // it points upwards.
  Mat rotation = Mat::eye(3, 3, CV_32F);
  rotation.at<float>(0, 0) = 1;
  rotation.at<float>(1, 1) = cos(M_PI);
  rotation.at<float>(2, 2) = cos(M_PI);
  Mat inverted_rotation = rotation.inv();

  // Converts the point coordinates using the camera intrinsics and extrinsics
  // parameters.
  Mat out = point.z * inverted_intrinsics * point_2D + camera_center;
  out = inverted_rotation * out;
  point.x = out.at<float>(0, 0);
  point.y = out.at<float>(0, 1);
  point.z = out.at<float>(0, 2);
}