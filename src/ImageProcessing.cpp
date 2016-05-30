#include "ImageProcessing.h"
#include "Scanner.h"

#define LEFT true
#define RIGHT false

int image_counter = 0;

// Processes a screenshot retrieving the points of intersection between the laser and 
// the model and transforming them in 3D points.
//
// Input parameters: 
// source: the screenshot.
// intrinsics_matrix: the camera intrinsic matrix.
// input_parameters: the struct with all the input parameters.
// y_offset: the offset describing how much the two lasers and the camera has moved on the y-axis.
// point_cloud_points: the vector to fill with the 3D points.
int ImageProcessing(osg::Image* source, osg::Matrixf intrinsics_matrix, struct InputParameters *input_parameters, float y_offset, vector<Point3f>& point_cloud_points) {
    // Defines the Mat with intrinsics parameters.
    Mat intrinsics = Mat::eye(3, 3, CV_32F);
    intrinsics.at<float>(0, 0) = intrinsics_matrix(0, 0);
    intrinsics.at<float>(0, 2) = intrinsics_matrix(0, 2);
    intrinsics.at<float>(1, 1) = intrinsics_matrix(1, 1);
    intrinsics.at<float>(1, 2) = intrinsics_matrix(1, 2);
    intrinsics.at<float>(2, 2) = intrinsics_matrix(2, 2);
    
    // Defines a Mat representing the source screenshot.
    Mat image(source->t(), source->s(), CV_8UC3);
    image.data = (uchar*) source->data();
    // Flips the image vertically.
    flip(image, image, 0);
    
    float roi_height = input_parameters->roi_height;
    float y_start_left = input_parameters->left_roi_start;
    float y_start_right = input_parameters->right_roi_start;
    
    // Extracts the left roi from the image.
    Rect region_of_interest = Rect(0, y_start_left, image.cols, roi_height);
    Mat image_roi_left(image, region_of_interest);
    // Converts the roi in black and white.
    cv::cvtColor(image_roi_left, image_roi_left, CV_BGR2GRAY);
    
    // Extracts the right roi from the image.
    region_of_interest = Rect(0, y_start_right, image.cols, roi_height);
    Mat image_roi_right(image, region_of_interest);
    // Converts the roi in black and white.
    cv::cvtColor(image_roi_right, image_roi_right, CV_BGR2GRAY);

    // Writes the rois to file.
    string name = "roi_left_" + std::to_string(image_counter) + ".png";
    imwrite(name, image_roi_left);
    name = "roi_right_" + std::to_string(image_counter) + ".png";
    imwrite(name, image_roi_right);
    image_counter++;

    Mat intersections_left(image_roi_left.cols, image_roi_left.rows, CV_8UC1);
    Mat intersections_right(image_roi_right.cols, image_roi_right.rows, CV_8UC1);
    vector<cv::Point3f> intersection_points_left;
    vector<cv::Point3f> intersection_points_right;

    // Applies a threshold to extract the intersection points, so if a pixel intensity is less 
    // than or equal to 254 it is set to 0.
    threshold(image_roi_left, intersections_left, 254, 255, 3);
    threshold(image_roi_right, intersections_right, 254, 255, 3);

    // Stores the white pixels in a vector.
    LoadIntersectionPoints(intersections_left, intersection_points_left);
    LoadIntersectionPoints(intersections_right, intersection_points_right);
    
    // Converts the 2D pixel to 3D points.
    InsertPoints(intersection_points_left, intrinsics, input_parameters, y_offset, LEFT, point_cloud_points);
    InsertPoints(intersection_points_right, intrinsics, input_parameters, y_offset, RIGHT, point_cloud_points);

    return 0;
}

// Stores points in a vector using as their coordinates the ones of the white pixels of an image.
// 
// Input parameters:
// intersections: the image.
// intersection_points: the output vector.
void LoadIntersectionPoints(Mat intersections, vector<Point3f>& intersection_points) {
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

// Transforms 2D points (pixel) in 3D points using triangulation.
//
// Input paramaters:
// intersection_points: a vector containing the points of intersection between the laser and the model.
// intrinsics: the camera intrinsics matrix.
// input_parameters: the struct with all the input parameters.
// y_offset: the offset describing how much the two lasers and the camera has moved on the y-axis. 
// roi: a boolean indicating the roi (left or right).
// point_cloud_points: the output vector with the points in world coordinates.
void InsertPoints(vector<cv::Point3f> intersection_points, Mat intrinsics, struct InputParameters *input_parameters, float y_offset, bool roi, vector<Point3f>& point_cloud_points) {
    float pixel_size = input_parameters->pixel_size;
    float baseline = input_parameters->laser_distance;
    float focal_length = input_parameters->focal_length;
    float y_start_left = input_parameters->left_roi_start;
    float y_start_right = input_parameters->right_roi_start;
    float height = input_parameters->camera_height;
    // The y coordinate of the middle pixel in the image.
    int y_middle = height/2;
    float alfa = 90 - input_parameters->laser_incline;
    // Converts the angle from degree to radians.
    float alfa_rad = 2 * M_PI * alfa / 360;
    
    float absolute_pixel_position, relative_pixel_position, pixel_position, z_coord;
    Point3f point;
    
    // If no intersection points are found, then return.
    if (intersection_points.size() == 0)
        return;
    
    for (int i = 0; i < intersection_points.size() - 1; i++) {
        // The y coordinate of the pixel relative to the roi.
        pixel_position = intersection_points.at(i).y;
        if (roi) { // Left roi.
            // The position of the current pixel relative to the entire screenshot.
            absolute_pixel_position = y_start_left + pixel_position;
            // The distance from the current pixel and the middle pixel.
            relative_pixel_position = y_middle - absolute_pixel_position;
        } else { // Right roi.
            // The position of the current pixel relative to the entire screenshot.
            absolute_pixel_position = y_start_right + pixel_position;
            // The distance from the current pixel and the middle pixel.
            relative_pixel_position = absolute_pixel_position - y_middle;
        }
        point.x = intersection_points.at(i).x;
        point.y = absolute_pixel_position;
        // Computes the z coordinate using triangulation.
        z_coord = baseline * focal_length / (focal_length * tan(alfa_rad) - relative_pixel_position*pixel_size);
        point.z = z_coord;
        //cout << "Coordinate punto che sta per essere convertito: (" << point.x << ", " << point.y << ", " << point.z << ")" << endl;
        // Converts from pixel to world coordinates.
        ConvertCoordinates(point, intrinsics, input_parameters, y_offset);
        point.z = -point.z;
        //cout << "Coordinate punto che sta per essere inserito: (" << point.x << ", " << point.y << ", " << point.z << ")" << endl;
        point_cloud_points.push_back(point);
    }
}

// Builds the point cloud.
//
// Input parameters:
// point_cloud_points: a vector containing the points to insert in the point cloud.
void BuildPointCloud(vector<Point3f> point_cloud_points) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(
            new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB output_point;

    // Builds the pointcloud.
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

    // Visualizes the point cloud.
    cout << "Visualization... Press Q to exit." << endl;
    pcl::visualization::PCLVisualizer viewer("Reconstructed scene");
    viewer.setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
            output);
    viewer.addPointCloud<pcl::PointXYZRGB> (output, rgb, "cloud");
    //viewer.addCoordinateSystem(100.0, "cloud");
    //viewer.initCameraParameters();
    viewer.spin();
    
    // Saves the point cloud to file.
    pcl::io::savePCDFileASCII("output.pcd", *output);
}

// Converts the coordinates of a point from screen to world coordinates.
//
// Input parameters:
// point: the point to convert.
// intrinsics: the camera intrinsics matrix.
// input_parameters: the struct with all the input parameters.
// y_offset: the offset describing how much the two lasers and the camera has moved on the y-axis.
void ConvertCoordinates(Point3f& point, Mat intrinsics, struct InputParameters *input_parameters, float y_offset) {
    float point_coord[] = {point.x, point.y, 1};
    // The actual absolute position of the camera.
    float camera_coord[] = {input_parameters->x_camera_absolute, input_parameters->y_camera_absolute - y_offset, input_parameters->z_camera_absolute};
        
    // Defines the necessary matrices.
    Mat point_2D = Mat(3, 1, CV_32F, point_coord);
    Mat camera_center = Mat(3, 1, CV_32F, camera_coord);
    
    float lambda = point.z - (input_parameters->z_camera_absolute);
    Mat inverted_intrinsics = intrinsics.inv();

    // Converts the point coordinates using the camera intrinsics parameters and position.
    Mat out = lambda * inverted_intrinsics * point_2D + camera_center;
    point.x = out.at<float>(0, 0);
    point.y = out.at<float>(0, 1);
    point.z = out.at<float>(0, 2);
}