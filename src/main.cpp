#include "Scanner.h"

bool InputParser(string filename, InputParameters *input_parameters);
bool InputCheck(InputParameters *input_parameters);

int main(int argc, char *argv[]) {
  struct InputParameters *input_parameters = new InputParameters();

  // Reads input data from file and stores it in input_parameters.
  // Executes model scanning  successful.
  if (InputParser(argv[1], input_parameters))
    Scanner(input_parameters);
  else
    cout << "Error in input file." << endl;

  delete input_parameters;

  return 0;
}

// Reads input values from the XML input file.
// Returns true if data could successfully be read.
//
// Input parameters:
// filename: the XML filename.
// input_parameters: the struct to fill with the input parameters specified by
// the user.
bool InputParser(string filename, InputParameters *input_parameters) {
  FileStorage fs(filename, FileStorage::READ);

  if (!fs.isOpened()) {
    cout << "Failed to open file." << endl;
    return false;
  }

  // Terminates execution if any of the needed information is missing.
  if (fs["model_filename"].empty() || fs["scanning_speed"].empty() ||
      fs["fps"].empty() || fs["intrinsics_filename"].empty() ||
      fs["x_camera_coord"].empty() || fs["y_camera_coord"].empty() ||
      fs["z_camera_coord"].empty() || fs["camera_width"].empty() ||
      fs["camera_height"].empty() || fs["pixel_size"].empty() ||
      fs["focal_length"].empty() || fs["roi_height"].empty() ||
      fs["roi_top_start"].empty() || fs["roi_bottom_start"].empty() ||
      fs["laser_distance"].empty() || fs["laser_incline"].empty() ||
      fs["laser_aperture"].empty()) {
    cout << "Syntax error in input file." << endl;
    fs.release();
    return false;
  }

  if (fs["model_filename"].isNone() || fs["scanning_speed"].isNone() ||
      fs["fps"].isNone() || fs["intrinsics_filename"].isNone() ||
      fs["x_camera_coord"].isNone() || fs["y_camera_coord"].isNone() ||
      fs["z_camera_coord"].isNone() || fs["camera_width"].isNone() ||
      fs["camera_height"].isNone() || fs["pixel_size"].isNone() ||
      fs["focal_length"].isNone() || fs["roi_height"].isNone() ||
      fs["roi_top_start"].isNone() || fs["roi_bottom_start"].isNone() ||
      fs["laser_distance"].isNone() || fs["laser_incline"].isNone() ||
      fs["laser_aperture"].isNone()) {
    cout << "Missing value in input file." << endl;
    fs.release();
    return false;
  }

  // Optional flag that allows the user to specify arbitrary system parameters
  // (when set to false). If the factory_limitations tag is missing or its value
  // is not specified, it is set to true by default.
  if (fs["factory_limitations"].isNone() || fs["factory_limitations"].empty())
    input_parameters->factory_limitations = true;
  else
    fs["factory_limitations"] >> input_parameters->factory_limitations;
  // Optional flag that allows the user to save the point cloud (when set to
  // true). If the save_point_cloud tag is missing or its value is not
  // specified, it is set to true by default.
  if (fs["save_point_cloud"].isNone() || fs["save_point_cloud"].empty())
    input_parameters->save_point_cloud = true;
  else
    fs["save_point_cloud"] >> input_parameters->save_point_cloud;

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

  // Pixel size, focal length, height of the rois.
  fs["pixel_size"] >> input_parameters->pixel_size;
  fs["focal_length"] >> input_parameters->focal_length;
  fs["roi_height"] >> input_parameters->roi_height;

  // The Y coordinates where the two rois start.
  fs["roi_top_start"] >> input_parameters->roi_top_start;
  fs["roi_bottom_start"] >> input_parameters->roi_bottom_start;

  // The distance between the camera and the laser (baseline).
  fs["laser_distance"] >> input_parameters->laser_distance;
  // The angle between the laser and the horizon.
  fs["laser_incline"] >> input_parameters->laser_incline;
  // Aperture angle of the laser.
  fs["laser_aperture"] >> input_parameters->laser_aperture;

  fs.release();

  // Terminates execution if the user is not allowed to use arbitrary parameters
  // and some of said parameters are not within factory thresholds.
  if (input_parameters->factory_limitations && !InputCheck(input_parameters)) {
    cout << "Parameters not within factory thresholds." << endl;
    return false;
  }

  return true;
}
// Verifies whether any of the input parameters are not within factory specified
// boundaries, returning false if that is the case.
//
// Input parameters:
// input_parameters: the struct to fill with the input parameters specified by
// the user.
bool InputCheck(InputParameters *input_parameters) {
  bool check = true;
  if (input_parameters->scanning_speed < 100 ||
      input_parameters->scanning_speed > 1000) {
    cout << "Scanning speed not within factory range." << endl;
    check = false;
  }
  if (input_parameters->fps < 100 || input_parameters->fps > 500) {
    cout << "FPS not within factory range." << endl;
    check = false;
  }
  if (input_parameters->laser_distance < 500 ||
      input_parameters->laser_distance > 800) {
    cout << "Laser distance not within factory range." << endl;
    check = false;
  }
  if (input_parameters->laser_incline < 60 ||
      input_parameters->laser_incline > 70) {
    cout << "Laser incline not within factory range." << endl;
    check = false;
  }
  if (input_parameters->laser_aperture < 30 ||
      input_parameters->laser_aperture > 45) {
    cout << "Laser aperture not within factory range." << endl;
    check = false;
  }
  return check;
}