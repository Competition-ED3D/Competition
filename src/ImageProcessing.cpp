#include "ImageProcessing.h"

#define LEFT true
#define RIGHT false

int image_counter = 0;

int ImageProcessing(osg::Image* source, osg::Matrixf intrinsics_matrix, struct InputParameters *input_parameters, float y_offset, float laser_incline, vector<Point3f>& point_cloud_points) {

    Mat intrinsics = Mat::eye(3, 3, CV_32F);
    intrinsics.at<float>(0, 0) = intrinsics_matrix(0, 0);
    intrinsics.at<float>(0, 2) = intrinsics_matrix(0, 2);
    intrinsics.at<float>(1, 1) = intrinsics_matrix(1, 1);
    intrinsics.at<float>(1, 2) = intrinsics_matrix(1, 2);
    intrinsics.at<float>(2, 2) = intrinsics_matrix(2, 2);

    Mat image(source->t(), source->s(), CV_8UC3);
    image.data = (uchar*) source->data();
    flip(image, image, 0);
    int roi_height = 400;
    //int roi_height = 600;
    int y_start_left = 200;
    int y_start_right = image.rows - y_start_left - roi_height;
    Rect region_of_interest = Rect(0, y_start_left, image.cols, roi_height);
    Mat image_roi_left(image, region_of_interest);
    cv::cvtColor(image_roi_left, image_roi_left, CV_BGR2GRAY);
    //cout << "roi_left size " << image_roi_left.size() << endl;
    region_of_interest = Rect(0, y_start_right, image.cols, roi_height);
    Mat image_roi_right(image, region_of_interest);
    cv::cvtColor(image_roi_right, image_roi_right, CV_BGR2GRAY);

    //cout << "roi_right size " << image_roi_right.size() << endl;
    //cout << "image.rows - y_start - roi_height " << image.rows - y_start - roi_height << endl;

    string name = "roi_left_" + std::to_string(image_counter) + ".png";
    imwrite(name, image_roi_left);
    name = "roi_right_" + std::to_string(image_counter) + ".png";
    imwrite(name, image_roi_right);
    image_counter++;

    Scalar lbound = Scalar(0, 0, 200);
    Scalar ubound = Scalar(0, 0, 255);

    Mat intersections_left(image_roi_left.cols, image_roi_left.rows, CV_8UC1);
    Mat intersections_right(image_roi_right.cols, image_roi_right.rows, CV_8UC1);
    vector<cv::Point3f> intersection_points_left;
    vector<cv::Point3f> intersection_points_right;

    //inRange(image_roi_left, lbound, ubound, intersections);
    //threshold(image_roi_left, intersections, 100, 255, 4 );
      //  imwrite("intersections_primo_threshold.jpg", intersections);

    threshold(image_roi_left, intersections_left, 254, 255, 3 );
    
    LoadIntersectionPoints(intersections_left, intersection_points_left);
    //getchar();
    
    threshold(image_roi_right, intersections_right, 254, 255, 3 );
    //inRange(image_roi_right, lbound, ubound, intersections);
    LoadIntersectionPoints(intersections_right, intersection_points_right);
    //findNonZero(intersections_right, intersection_points_right);
    name = "intersections_right_" + std::to_string(image_counter) + ".jpg";
    imwrite(name, intersections_right);
    //getchar();

    //InsertPoints(intersection_points, intrinsics, y_offset, laser_incline, point_cloud_points);
    InsertPoints(intersection_points_left, intrinsics, input_parameters, y_offset, laser_incline, y_start_left, LEFT, point_cloud_points);
    InsertPoints(intersection_points_right, intrinsics, input_parameters, y_offset, laser_incline, y_start_right, RIGHT, point_cloud_points);

    return 0;
}

void LoadIntersectionPoints(Mat intersections, vector<Point3f>& intersection_points) {
        for (int i = 0; i < intersections.cols; i++) {
        for (int j = 0; j < intersections.rows; j++) {
            Scalar intensity = intersections.at<uchar>(j, i);
            if (intensity.val[0] == 255) {
            //if(intersections.at<uchar>(j,i) == 255){
                Point3f p;
                p.x = i;
                p.y = j;
                intersection_points.push_back(p);
                break;
            }
        }
    }
}

void InsertPoints(vector<cv::Point3f> intersection_points, Mat intrinsics, struct InputParameters *input_parameters, float y_offset, float laser_incline, int y_start, bool roi, vector<Point3f>& point_cloud_points) {
    float baseline = 500 / 5.5e-3; //Converto in pixel
    float focal_length = 25 / 5.5e-3; //Converto in pixel
    float absolute_pixel_position, relative_pixel_position, pixel_position;
    float alfa = 90 - laser_incline;
    float z_coord;
    float alfa_rad = 2 * M_PI * alfa / 360;
    float roi_height = 400;
    Point3f point;
    
    if (intersection_points.size() == 0)
        return;
    
    for (int i = 0; i < intersection_points.size() - 1; i++) {
        pixel_position = intersection_points.at(i).y;
        if (roi) { // left roi
            absolute_pixel_position = 100 + pixel_position;
            relative_pixel_position = 544- absolute_pixel_position;
        } else { // right:y_ start = image.cols - y_start_left - roi_height
            absolute_pixel_position = 1088 - 100 - roi_height + pixel_position;
            relative_pixel_position = absolute_pixel_position - 544;
        }
        point.x = intersection_points.at(i).x;
        //point.y = absolute_pixel_position - (y_offset/5.5e-3);
        point.y = absolute_pixel_position;
        //point.y = relative_pixel_position;
        //if(roi)
        //    point.y = -point.y;
        z_coord = baseline * focal_length / (focal_length * tan(alfa_rad) - relative_pixel_position);
        z_coord = z_coord * 5.5e-3; // converto in mm
        //point.y = intersection_points.at(i).y;
        point.z = z_coord;
        //cout << "Coordinate punto che sta per essere convertito: (" << point.x << ", " << point.y << ", " << point.z << ")" << endl;
        ConvertCoordinates(point, intrinsics);
        //if(roi) // left
        point.y = point.y - y_offset*0.5;
        //point.y = point.y - y_offset;
        //cout<<"y_offset "<<y_offset<<endl;
        //else
        //    point.y = point.y + 2*baseline*5.5e-3+y_offset;
        point.z = -point.z;
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
    //double point_coord[] = {x, y, point.z};
    //Mat converted_point = Mat(3, 1, CV_64F, point_coord);
    //Mat inverted_intrinsics = intrinsics.inv();

    //double cx = intrinsics.at<double>(2, 0);
    //double cy = intrinsics.at<double>(2, 1);
    /*float cx = intrinsics.at<float>(0, 2);
    float cy = intrinsics.at<float>(1, 2);
    
    float fx = intrinsics.at<float>(0, 0);
    float fy = intrinsics.at<float>(1, 1);
    point.x = (point.x*point.z - cx * point.z) / fx;
    point.y = (point.y*point.z - cy * point.z) / fy;*/

    //cout<<"point x y z dopo "<<point.x<<" "<<point.y<<" "<<point.z<<endl;
    //  getchar();*/

}

