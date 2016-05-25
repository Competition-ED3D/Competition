#include "ImageProcessing.h"
#include "Scanner.h"

#define LEFT true
#define RIGHT false

int image_counter = 0;

int ImageProcessing(osg::Image* source, osg::Matrixf intrinsics_matrix, struct InputParameters *input_parameters, float y_offset, vector<Point3f>& point_cloud_points) {

    Mat intrinsics = Mat::eye(3, 3, CV_32F);
    intrinsics.at<float>(0, 0) = intrinsics_matrix(0, 0);
    intrinsics.at<float>(0, 2) = intrinsics_matrix(0, 2);
    intrinsics.at<float>(1, 1) = intrinsics_matrix(1, 1);
    intrinsics.at<float>(1, 2) = intrinsics_matrix(1, 2);
    intrinsics.at<float>(2, 2) = intrinsics_matrix(2, 2);

    Mat image(source->t(), source->s(), CV_8UC3);
    image.data = (uchar*) source->data();
    flip(image, image, 0);
    float roi_height = input_parameters->roi_height;
    float y_start_left = input_parameters->left_roi_start;
    //int y_start_right = image.rows - y_start_left - roi_height;
    float y_start_right = input_parameters->right_roi_start;
    
    Rect region_of_interest = Rect(0, y_start_left, image.cols, roi_height);
    Mat image_roi_left(image, region_of_interest);
    cv::cvtColor(image_roi_left, image_roi_left, CV_BGR2GRAY);
    
    region_of_interest = Rect(0, y_start_right, image.cols, roi_height);
    Mat image_roi_right(image, region_of_interest);
    cv::cvtColor(image_roi_right, image_roi_right, CV_BGR2GRAY);

    string name = "roi_left_" + std::to_string(image_counter) + ".png";
    imwrite(name, image_roi_left);
    name = "roi_right_" + std::to_string(image_counter) + ".png";
    imwrite(name, image_roi_right);
    image_counter++;

    Mat intersections_left(image_roi_left.cols, image_roi_left.rows, CV_8UC1);
    Mat intersections_right(image_roi_right.cols, image_roi_right.rows, CV_8UC1);
    vector<cv::Point3f> intersection_points_left;
    vector<cv::Point3f> intersection_points_right;

    threshold(image_roi_left, intersections_left, 254, 255, 3 );
    LoadIntersectionPoints(intersections_left, intersection_points_left);

    threshold(image_roi_right, intersections_right, 254, 255, 3 );
    LoadIntersectionPoints(intersections_right, intersection_points_right);
    
    InsertPoints(intersection_points_left, intrinsics, input_parameters, y_offset, LEFT, point_cloud_points);
    InsertPoints(intersection_points_right, intrinsics, input_parameters, y_offset, RIGHT, point_cloud_points);

    return 0;
}

void LoadIntersectionPoints(Mat intersections, vector<Point3f>& intersection_points) {
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
}

void InsertPoints(vector<cv::Point3f> intersection_points, Mat intrinsics, struct InputParameters *input_parameters, float y_offset, bool roi, vector<Point3f>& point_cloud_points) {
    float pixel_size = input_parameters->pixel_size;
    float baseline = input_parameters->laser_distance / pixel_size; //Converto in pixel
    float focal_length = input_parameters->focal_length / pixel_size; //Converto in pixel
    float alfa = 90 - input_parameters->laser_incline;
    float alfa_rad = 2 * M_PI * alfa / 360;
    float height = input_parameters->camera_height;
    int y_middle = height/2;
    float y_start_left = input_parameters->left_roi_start;
    float y_start_right = input_parameters->right_roi_start;
    
    float absolute_pixel_position, relative_pixel_position, pixel_position, z_coord;
    Point3f point;
    
    if (intersection_points.size() == 0)
        return;
    
    for (int i = 0; i < intersection_points.size() - 1; i++) {
        pixel_position = intersection_points.at(i).y;
        if (roi) { // left roi
            absolute_pixel_position = y_start_left + pixel_position;
            relative_pixel_position = y_middle- absolute_pixel_position;
        } else { // right:y_ start = image.cols - y_start_left - roi_height           
            //absolute_pixel_position = height - 100 - roi_height + pixel_position;
            absolute_pixel_position = y_start_right + pixel_position;
            relative_pixel_position = absolute_pixel_position - y_middle;
        }
        point.x = intersection_points.at(i).x;
        point.y = absolute_pixel_position;
        z_coord = baseline * focal_length / (focal_length * tan(alfa_rad) - relative_pixel_position);
        z_coord = z_coord * pixel_size; // converto in mm
        point.z = z_coord;
        //cout << "Coordinate punto che sta per essere convertito: (" << point.x << ", " << point.y << ", " << point.z << ")" << endl;
        ConvertCoordinates(point, intrinsics);
        point.y = point.y - y_offset*0.55;
        //point.y = point.y - y_offset;
        //cout<<"y_offset "<<y_offset<<endl;
        //point.z = -point.z;
        //cout << "Coordinate punto che sta per essere inserito: (" << point.x << ", " << point.y << ", " << point.z << ")" << endl;
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
    //viewer.addCoordinateSystem(100.0, "cloud");
    //viewer.initCameraParameters();
    viewer.spin();

    pcl::io::savePCDFileASCII("output.pcd", *output);
}

void ConvertCoordinates(Point3f& point, Mat intrinsics) {
    float point_coord[] = {point.x*point.z, point.y*point.z, point.z};
    
    Mat converted_point = Mat(3, 1, CV_32F, point_coord);
    Mat inverted_intrinsics = intrinsics.inv();

    Mat out = inverted_intrinsics * converted_point;
    point.x = out.at<float>(0, 0);
    point.y = out.at<float>(0, 1);
    point.z = out.at<float>(0, 2);
    //cout<<"point x y z dopo "<<point.x<<" "<<point.y<<" "<<point.z<<endl;
    //  getchar();*/
}