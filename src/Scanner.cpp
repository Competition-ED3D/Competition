#include "Scanner.h"

#define LASER_1 true
#define LASER_2 false

// This function simulates the laser scanning process of the given model: the
// system (two lasers and a camera) scans the object moving along the Y axis and
// reconstructs a three-dimensional point cloud.
// Returns 0 if successful.
//
// Input parameters:
// input_parameters: struct containing the input parameters specified by the
// user.
int Scanner(InputParameters* input_parameters) {
  // Initializes OpenSceneGraph nodes.
  osg::Group* root = new osg::Group();
  osg::Geometry* intersection_line_geometry = new osg::Geometry();
  osg::Geode* intersection_line_geode = new osg::Geode();
  osg::Node* model = NULL;

  // Input model to be scanned.
  model = osgDB::readNodeFile(input_parameters->model_filename);

  // Geode used to store intersection lines.
  intersection_line_geode->addDrawable(intersection_line_geometry);

  root->addChild(intersection_line_geode);
  root->addChild(model);

  // Defines point size and line width.
  osg::Point* point = new osg::Point;
  point->setSize(2.0f);
  root->getOrCreateStateSet()->setAttribute(point);
  osg::LineWidth* linewidth = new osg::LineWidth();
  linewidth->setWidth(2.0f);
  root->getOrCreateStateSet()->setAttributeAndModes(linewidth,
                                                    osg::StateAttribute::ON);

  // Enables model-only lighting, ensuring the color of the laser will not be
  // affected by scene lighting.
  root->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
  model->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);

  // Initializes the viewer.
  osgViewer::Viewer viewer;
  viewer.setSceneData(root);

  // Initializes camera position according to input parameters and stores them
  // for future use.
  osg::Vec3 posInWorld =
      model->getBound().center() *
      osg::computeLocalToWorld(model->getParentalNodePaths()[0]);
  float camera_x = -posInWorld[0] + input_parameters->x_camera_coord;
  float camera_y = -posInWorld[1] + input_parameters->y_camera_coord;
  float camera_z = posInWorld[2] + input_parameters->z_camera_coord;
  input_parameters->x_camera_absolute = camera_x;
  input_parameters->y_camera_absolute = camera_y;
  input_parameters->z_camera_absolute = camera_z;

  // Translates the viewer camera to the starting position.
  osg::Matrixd cameraTrans;
  cameraTrans.makeTranslate(camera_x, camera_y, camera_z);
  viewer.getCamera()->setViewMatrix(cameraTrans);
  viewer.getCamera()->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
  viewer.realize();

  // Reads the camera resolution and matrix from the input files.
  osg::Matrixf intrinsics_matrix;
  vector<double> distortion_matrix;
  string filename = input_parameters->intrinsics_filename;
  IntrinsicsParser(filename, intrinsics_matrix, distortion_matrix);

  // Defines the distance (step-y) the system will cover along the y-axis
  // between each frame of the camera.
  float scanning_speed = input_parameters->scanning_speed;
  float fps = input_parameters->fps;
  float step_y = scanning_speed / fps;

  // Distance between the single intersection lines forming the laser plane that
  // intersects the model.
  float step_x = 1;
  // Maximum distance threshold for intersection points belonging to the same
  // line. If two points are further apart than the threshold, they are part of
  // different lines.
  float threshold = EuclideanDistance(osg::Vec3d(0, 0, 0),
                                      osg::Vec3d(step_x, step_x, step_x));

  // Vector to store the point cloud points in.
  vector<Point3f> point_cloud_points;

  // Number of consecutive iterations where no intersections have been found.
  int failed_intersections = 0;

  // Sets up the first frame of the viewer.
  viewer.frame();

  // Scans the object, moving along the y-axis, analyzing the intersections
  // lines at each frame and processing it in order to build the point cloud.
  // Stops after no intersections have been found by both lasers for ten
  // consecutive frames.
  for (int i = 0; failed_intersections < 10; i++) {
    cout << "Iteration #" << i << endl;
    // Translates the system by number of iterations * step_y millimeters along
    // the Y axis.
    cameraTrans.makeTranslate(camera_x, camera_y - i * step_y, camera_z);
    viewer.getCamera()->setViewMatrix(cameraTrans);
    // Updates camera position.
    input_parameters->y_camera_absolute = camera_y - i * step_y;

    // Initializes laser parameters.
    float laser_distance = input_parameters->laser_distance;
    float laser_incline = input_parameters->laser_incline;
    float laser_aperture = input_parameters->laser_aperture;

    // Origin coordinates of the lines forming the left laser plane.
    osg::Vec3d start_laser_1 = osg::Vec3d(
        -camera_x, laser_distance + i * step_y - camera_y, -camera_z);
    // Array used to store left laser's intersection points.
    vector<osg::ref_ptr<osg::Vec3Array> > intersections_laser_1;

    // Computes the intersections of the left laser plane with the model.
    ComputeIntersections(start_laser_1, step_x, threshold, model, laser_incline,
                         laser_aperture, LASER_1, &intersections_laser_1);

    // Origin coordinates of the lines forming the right laser plane.
    osg::Vec3d start_laser_2 = osg::Vec3d(
        -camera_x, -laser_distance + i * step_y - camera_y, -camera_z);
    // Array used to store right laser's intersection points.
    vector<osg::ref_ptr<osg::Vec3Array> > intersections_laser_2;

    // Computes the intersections of the right laser plane with the model.
    ComputeIntersections(start_laser_2, step_x, threshold, model, laser_incline,
                         laser_aperture, LASER_2, &intersections_laser_2);

    // Increases the counter if no intersections have been found, resets it
    // otherwise.
    if (intersections_laser_1.size() == 0 &&
        intersections_laser_2.size() == 0) {
      failed_intersections++;
    } else {
      failed_intersections = 0;
    }

    // Displays the intersections on the model.
    ShowIntersections(intersections_laser_1, intersection_line_geode);
    ShowIntersections(intersections_laser_2, intersection_line_geode);

    // Loads intrinsics into an OpenCV matrix.
    Mat intrinsics = Mat::eye(3, 3, CV_32F);
    intrinsics.at<float>(0, 0) = intrinsics_matrix(0, 0);
    intrinsics.at<float>(0, 2) = intrinsics_matrix(0, 2);
    intrinsics.at<float>(1, 1) = intrinsics_matrix(1, 1);
    intrinsics.at<float>(1, 2) = intrinsics_matrix(1, 2);
    intrinsics.at<float>(2, 2) = intrinsics_matrix(2, 2);

    // Projects the single intersection lines on one image plane each.
    Mat scene_laser_1 = Mat::zeros(input_parameters->camera_height,
                                   input_parameters->camera_width, CV_8UC1);
    if (intersections_laser_1.size() != 0)
      ProjectToImagePlane(intersections_laser_1, intrinsics, input_parameters,
                          scene_laser_1);

    Mat scene_laser_2 = Mat::zeros(input_parameters->camera_height,
                                   input_parameters->camera_width, CV_8UC1);
    if (intersections_laser_2.size() != 0)
      ProjectToImagePlane(intersections_laser_2, intrinsics, input_parameters,
                          scene_laser_2);

    // Merges the two image planes into a single one.
    Mat output = Mat::zeros(input_parameters->camera_height,
                            input_parameters->camera_width, CV_8UC1);
    bitwise_or(scene_laser_1, scene_laser_2, output);

    // Advances to the next frame.
    viewer.frame();

    // Processes the image, extracting the 3D points that will form the point
    // cloud.
    if (intersections_laser_2.size() != 0 || intersections_laser_1.size() != 0)
      ImageProcessing(output, intrinsics_matrix, input_parameters,
                      point_cloud_points);
    cout << "Point cloud size: " << point_cloud_points.size() << " points."
         << endl;

    // Removes the previously found intersection points from the intersection
    // geode so that they will not be displayed in the following iterations.
    intersection_line_geode->removeDrawables(
        1, intersections_laser_1.size() + intersections_laser_2.size());
  }

  // Builds and shows the point cloud using the previously computed points.
  BuildPointCloud(point_cloud_points, input_parameters);

  return 0;
}

// Computes the intersections of the laser plane with the model. The laser plane
// is not continuous, but is rather formed by a number of lines separated by a
// set distance.
//
// Input parameters:
// start: origin coordinates of the lines forming the laser plane.
// step_x: distance between two consecutive lines of the laser plane.
// threshold: maximum distance between two points that belong to the same
// intersection line.
// model: the model whose intersection with the laser plane will be computed.
// laser_incline: laser angle with respect to the horizon.
// laser_aperture: aperture angle of the laser.
// side: left or right laser.
//
// Output parameters:
// intersections: vector storing the coordinates of the intersection points.
void ComputeIntersections(
    osg::Vec3d start, float step_x, float threshold, osg::Node* model,
    float laser_incline, float laser_aperture, bool side,
    vector<osg::ref_ptr<osg::Vec3Array> >* intersections) {
  // Converts laser incline and aperture from degrees to radians.
  laser_incline = 2 * M_PI * laser_incline / 360;
  laser_aperture = 2 * M_PI * laser_aperture / 360;

  // Initializes laser height and length and halves the laser width.
  float laser_height =
      EuclideanDistance(osg::Vec3d(start.x(), start.y(), -1000), start);
  float laser_length = laser_height / cos(M_PI / 2 - laser_incline);
  float laser_width_half = tan(laser_aperture / 2) * laser_length;

  // Initializes end coordinates of the line connecting the camera center to the
  // center (with respect to the X coordinate) of the laser plane.
  float end_x_coord = start.x();
  float end_y_coord = laser_length * sin(M_PI / 2 - laser_incline);
  float end_z_coord = -1000;

  // Initializes the first and last X coordinates of the laser plane.
  float x_start = start.x() + laser_width_half;
  float x_end = start.x() - laser_width_half;

  // If the y coordinate of origin of the lines is lower than the end's, the end
  // coordinate is flipped.
  if (start.y() < end_y_coord) {
    end_y_coord = -end_y_coord;
  }

  // Array that will store the intersection points.
  osg::ref_ptr<osg::Vec3Array> vertices(new osg::Vec3Array());
  // Computes the intersection of each line of the laser plane and the model.
  for (int i = 0; i < (abs(x_end - x_start) / step_x); i++) {
    // Defines the end coordinates of the line, taking which laser is being
    // considered into account.
    osg::Vec3d end;
    if (side) {  // Laser 1.
      end = osg::Vec3d(end_x_coord - i * step_x + abs(x_end - x_start) / 2,
                       end_y_coord + start.y(), end_z_coord);
    } else {  // Laser 2.
      end = osg::Vec3d(end_x_coord - i * step_x + abs(x_end - x_start) / 2,
                       -end_y_coord + start.y(), end_z_coord);
    }

    // Initializes OpenSceneGraph intersector and links it to the model.
    osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector =
        new osgUtil::LineSegmentIntersector(start, end);
    osgUtil::IntersectionVisitor intersect_visitor(intersector.get());
    model->accept(intersect_visitor);

    if (intersector->containsIntersections()) {
      // If the current intersection point is farther than the threshold
      // distance from the last inserted point in the vertices array, it means
      // it belongs to a different line and the previous line ended and can be
      // pushed into the intersection points vector. The vertices vector is then
      // emptied.
      if (vertices->size() != 0 &&
          (EuclideanDistance(
               vertices->at(vertices->size() - 1),
               intersector->getFirstIntersection().localIntersectionPoint) >=
           threshold)) {
        intersections->push_back(vertices);
        vertices = new osg::Vec3Array();
      }
      // Pushes the intersection point into the vertices vector.
      vertices->push_back(
          intersector->getFirstIntersection().localIntersectionPoint);
    }
  }
  // If the vertices vector is not empty, its content is stored in the
  // intersections
  // array. This is necessary to ensure the last line or point found by the
  // intersector is inserted into the intersection vector.
  if (vertices->size() != 0) {
    intersections->push_back(vertices);
  }
}

// Displays the intersection points on the model.
//
// Input parameters:
// intersections: array storing the intersection points.
// intersection_line_geode: geode used to store the drawables representing the
// intersection points.
void ShowIntersections(
    const vector<osg::ref_ptr<osg::Vec3Array> >& intersections,
    osg::Geode* intersection_line_geode) {
  for (int i = 0; i < intersections.size(); i++) {
    // Initializes the geometry representing a line.
    osg::Geometry* line = new osg::Geometry();

    // Inserts the intersection points into the geometry.
    line->setVertexArray(intersections.at(i));

    // Initializes and sets the color of the intersection line (white).
    osg::Vec4Array* intersections_line_color = new osg::Vec4Array;
    for (int j = 0; j < intersections.at(i)->size(); j++) {
      intersections_line_color->push_back(osg::Vec4(1, 1, 1, 1));
    }
    line->setColorArray(intersections_line_color);
    line->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    // Draws either a line or a point, depending on the number of intersection
    // points.
    if (intersections.at(i)->size() > 1) {
      line->addPrimitiveSet(new osg::DrawArrays(
          osg::PrimitiveSet::LINE_STRIP, 0, intersections.at(i)->size()));
    } else {
      line->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0,
                                                intersections.at(i)->size()));
    }

    // Adds the line or the point to the geode.
    intersection_line_geode->addDrawable(line);
  }
}

// Projects the intersection points onto an image plane whose parameters are
// specified by the intrinsics matrix.
//
// Input parameters:
// intersections: array storing the intersection points of each intersection
// line.
// intrinsics: intrinsics matrix.
// input_parameters: struct containing the input parameters specified by the
// user.
//
// Output parameters:
// output: the image onto which the intersection points have been projected.
void ProjectToImagePlane(
    const vector<osg::ref_ptr<osg::Vec3Array> >& intersections, Mat intrinsics,
    InputParameters* input_parameters, Mat& output) {
  // Intersection line color (white).
  const int kWhite = 255;

  // Initializes translation and rotation vectors.
  Mat translation_vector(3, 1, DataType<float>::type);
  Mat rotation_vector(3, 1, DataType<float>::type);

  translation_vector.at<float>(0) = input_parameters->x_camera_absolute;
  translation_vector.at<float>(1) = input_parameters->y_camera_absolute;
  translation_vector.at<float>(2) = input_parameters->z_camera_absolute;
  rotation_vector.at<float>(0) = 0;
  rotation_vector.at<float>(1) = 0;
  rotation_vector.at<float>(2) = 0;

  // Initializes the distortion vector to 0: distortion is ignored.
  Mat distortion_coefficients(5, 1, DataType<float>::type);
  distortion_coefficients.at<float>(0) = 0;
  distortion_coefficients.at<float>(1) = 0;
  distortion_coefficients.at<float>(2) = 0;
  distortion_coefficients.at<float>(3) = 0;
  distortion_coefficients.at<float>(4) = 0;

  // Grayscale image to store a single projected intersection segment or point
  // of the intersection line that is being computed.
  Mat image = Mat::zeros(input_parameters->camera_height,
                         input_parameters->camera_width, CV_8UC1);

  // Projects all the intersections segments and points onto the image plane and
  // connects them with eachother to form a line.
  for (int i = 0; i < intersections.size(); i++) {
    // Array storing the points of a single segment.
    vector<Point3f> points;
    // Array storing the projected points of a single segment.
    vector<Point2f> projected_points;

    // If the current segment is made up of more than a single point then it is
    // a segment, otherwise it is a point.
    if (intersections.at(i)->size() > 1) {
      // Inserts all the points of the segment (the polyline vertices) into the
      // points array.
      for (int k = 0; k < intersections.at(i)->size(); k++) {
        Point3f p;
        p.x = intersections.at(i)->at(k).x();
        p.y = intersections.at(i)->at(k).y();
        p.z = intersections.at(i)->at(k).z();
        points.push_back(p);
      }
      // Projects the points onto the image plane.
      projectPoints(points, rotation_vector, translation_vector, intrinsics,
                    distortion_coefficients, projected_points);

      // Array storing the projected points with integer coordinates.
      vector<Point2i> projected_points_integer;
      // Converts the projected points' coordinates to integer, pushing the new
      // points into the array.
      for (int i = 0; i < projected_points.size(); i++) {
        Point2i p;
        p.x = (int)projected_points.at(i).x;
        p.y = (int)projected_points.at(i).y;
        projected_points_integer.push_back(p);
      }
      // Computes the polyline using the projected points as vertices.
      polylines(image, projected_points_integer, 0,
                Scalar(kWhite, kWhite, kWhite));
      // Merges the polyline with the output image.
      bitwise_or(image, output, output);
    } else {
      // Projects the single point onto the image plane and adds it to the
      // output image.
      Point3f point;
      point.x = intersections.at(i)->at(0).x();
      point.y = intersections.at(i)->at(0).y();
      point.z = intersections.at(i)->at(0).z();
      points.push_back(point);
      projectPoints(points, rotation_vector, translation_vector, intrinsics,
                    distortion_coefficients, projected_points);
      Point2f proj_point = projected_points.at(0);
      output.at<uchar>(Point((int)proj_point.x, (int)proj_point.y)) = kWhite;
    }
  }
}

// Returns the euclidean distance between two points.
//
// Input parameters:
// point1: the first point.
// point2: the second point.
float EuclideanDistance(osg::Vec3d point1, osg::Vec3d point2) {
  return sqrt(pow(point1.x() - point2.x(), 2) +
              pow(point1.y() - point2.y(), 2) +
              pow(point1.z() - point2.z(), 2));
}

// Parses the input XML file containing the camera intrinsics and distortion
// vector. Returns true if successful.
//
// Input parameters:
// filename: filename of the input XML file.
//
// Output parameters:
// intrinsics_matrix: matrix storing the intrinsics parameters.
// distortion_matrix: matrix storing the distortion parameters.
bool IntrinsicsParser(string filename, osg::Matrixf& intrinsics_matrix,
                      vector<double>& distortion_matrix) {
  // Reads the input XML file.
  osgDB::XmlNode* node = osgDB::readXmlFile(filename);

  // Reads intrinsics.
  if (node->children.at(0)->properties["Rows"] != std::to_string(3) ||
      node->children.at(0)->properties["Cols"] != std::to_string(3)) {
    cout << "Error in intrinsics matrix." << endl;
    return false;
  }
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      intrinsics_matrix(i, j) = std::stod(
          node->children.at(0)->children.at(i)->children.at(j)->contents);
    }
  }

  // Reads distortion parameters.
  if (node->children.at(1)->properties["Rows"] != std::to_string(1) ||
      node->children.at(1)->properties["Cols"] != std::to_string(5)) {
    cout << "Error in distortion vector." << endl;
    return false;
  }
  for (int i = 0; i < 5; i++) {
    distortion_matrix.push_back(std::stod(
        node->children.at(1)->children.at(0)->children.at(i)->contents));
  }

  node->unref();

  return true;
}