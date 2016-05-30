#include "Scanner.h"

// Reads input values from the XML input file.
// Returns true if data could successfully be read.
//
// Input parameters:
// filename: the XML filename.
// input_parameters: the struct to fill with all the input parameters.
bool InputParser(string filename, InputParameters *input_parameters) {
  cv::FileStorage fs(filename, cv::FileStorage::READ);

  if (!fs.isOpened()) {
    cout << "Failed to open file" << endl;
    return false;
  }

  // Terminates execution if any of the needed information is missing.
  if (fs["model_filename"].empty() || fs["scanning_speed"].empty() ||
      fs["fps"].empty() || fs["intrinsics_filename"].empty() ||
      fs["x_camera_coord"].empty() || fs["y_camera_coord"].empty() ||
      fs["z_camera_coord"].empty() || fs["camera_width"].empty() ||
      fs["camera_height"].empty() || fs["pixel_size"].empty() ||
      fs["focal_length"].empty() || fs["roi_height"].empty() ||
      fs["left_roi_start"].empty() || fs["right_roi_start"].empty() ||
      fs["laser_distance"].empty() || fs["laser_incline"].empty() ||
      fs["laser_aperture"].empty()) {
    cout << "Syntax error in input file" << endl;
    fs.release();
    return false;
  }

  if (fs["model_filename"].isNone() || fs["scanning_speed"].isNone() ||
      fs["fps"].isNone() || fs["intrinsics_filename"].isNone() ||
      fs["x_camera_coord"].isNone() || fs["y_camera_coord"].isNone() ||
      fs["z_camera_coord"].isNone() || fs["camera_width"].isNone() ||
      fs["camera_height"].isNone() || fs["pixel_size"].isNone() ||
      fs["focal_length"].isNone() || fs["roi_height"].isNone() ||
      fs["left_roi_start"].isNone() || fs["right_roi_start"].isNone() ||
      fs["laser_distance"].isNone() || fs["laser_incline"].isNone() ||
      fs["laser_aperture"].isNone()) {
    cout << "Missing value in input file" << endl;
    fs.release();
    return false;
  }

  // The filename of the model to scan.
  fs["model_filename"] >> input_parameters->model_filename;

  fs["scanning_speed"] >> input_parameters->scanning_speed;
  fs["fps"] >> input_parameters->fps;

  // The filename of the file containing the camera intrinsic parameters.
  fs["intrinsics_filename"] >> input_parameters->intrinsics_filename;

  // The position of the camera relative to the model.
  fs["x_camera_coord"] >> input_parameters->x_camera_coord;
  fs["y_camera_coord"] >> input_parameters->y_camera_coord;
  fs["z_camera_coord"] >> input_parameters->z_camera_coord;

  // The camera resolution.
  fs["camera_width"] >> input_parameters->camera_width;
  fs["camera_height"] >> input_parameters->camera_height;

  fs["pixel_size"] >> input_parameters->pixel_size;
  fs["focal_length"] >> input_parameters->focal_length;
  fs["roi_height"] >> input_parameters->roi_height;

  // The y coordinates where the two rois start.
  fs["left_roi_start"] >> input_parameters->left_roi_start;
  fs["right_roi_start"] >> input_parameters->right_roi_start;

  // The distance between the camera and the laser (baseline).
  fs["laser_distance"] >> input_parameters->laser_distance;
  // The angle between the laser and the horizon.
  fs["laser_incline"] >> input_parameters->laser_incline;

  fs["laser_aperture"] >> input_parameters->laser_aperture;

  return true;
}

int main(int argc, char *argv[]) {
  struct InputParameters *input_parameters = new InputParameters();

  // Reads the data from file and stores them in input_parameters.
  if (!InputParser(argv[1], input_parameters)) {
    cout << "Error in input file" << endl;
  }

  Scanner(input_parameters);

  return 0;
}
