#include "Scanner.h"

// This function simulates the laser scanning process of the given model: the
// system (two lasers and a camera) scans the object moving along the Y-axis and
// reconstructs a three-dimensional point cloud.
// Returns 0 if successful.
//
// Input parameters:
// input_parameters: struct containing the input parameters.
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
  unsigned int width = input_parameters->camera_width;
  unsigned int height = input_parameters->camera_height;
  osg::ref_ptr<osg::Camera> camera = new osg::Camera;
  osg::Matrixf intrinsics_matrix;
  std::vector<double> distortion_matrix;
  string filename = input_parameters->intrinsics_filename;
  IntrinsicsParser(filename, intrinsics_matrix, distortion_matrix);

  // Initializes the camera used to take snapshots of the scene with the
  // specified resolution and intrinsics.
  InitializeCamera(camera, width, height, intrinsics_matrix);

  // Adds the camera to the viewer.
  viewer.addSlave(camera.get(), osg::Matrixd(), osg::Matrixd());

  // Defines the distance (step-y) the system will cover along the y-axis
  // between each frame of the camera.
  float scanning_speed = input_parameters->scanning_speed;
  float fps = input_parameters->fps;
  float step_y = scanning_speed / fps;

  // Distance between the intersection lines forming the laser plane that
  // intersects the model.
  float step_x = 1;
  // Maximum distance threshold for intersection points belonging to the same
  // line. If two points are further apart than the threshold, they are part of
  // different lines.
  float threshold = EuclideanDistance(osg::Vec3d(0, 0, 0),
                                      osg::Vec3d(step_x, step_x, step_x));

  viewer.frame();

  // Sets the camera callback so that the screenshot of the scene is taken after
  // the all the scene drawing and post render operations have been completed.
  ScreenCapture* sc = (ScreenCapture*)camera.get()->getFinalDrawCallback();
  // Data structure to store the screenshots in.
  osg::Image* screenshot =
      (sc->getContextData(camera.get()->getGraphicsContext()))->getImage();

  // Vector to store the point cloud points in.
  vector<Point3f> point_cloud_points;

  // Number of iterations where no intersections have been found.
  int failed_intersections = 0;
  bool check_laser_1 = false;
  bool check_laser_2 = false;
  int laser_1_counter = 0;
  int laser_2_counter = 0;
  // Scans the object, moving along the y-axis, taking a screenshot of the scene
  // at each frame and processing it in order to build the point cloud. Stops
  // after no intersections have been found by both lasers for 10 consecutives
  // frames.
  // for(int k=0; k < 200; k++) {
  // NOTA: left è ora laser_1, right è laser_2
  for (int k = 0; failed_intersections < 10; k++) {
    cout << "K: " << k << endl;
    // Translates the system by step_y millimeters along the Y axis.
    cameraTrans.makeTranslate(camera_x, camera_y - k * step_y, camera_z);
    viewer.getCamera()->setViewMatrix(cameraTrans);

    // Initializes laser parameters.
    float laser_distance = input_parameters->laser_distance;
    float laser_incline = input_parameters->laser_incline;
    float laser_aperture = input_parameters->laser_aperture;

    // Origin coordinates of the lines forming the left laser plane.
    osg::Vec3d start_laser_1 = osg::Vec3d(
        -camera_x, laser_distance + k * step_y - camera_y, -camera_z);
    // Array used to store left laser's intersection points.
    std::vector<osg::ref_ptr<osg::Vec3Array> > intersections_laser_1;

    // Computes the intersections of the left laser plane with the model.
    ComputeIntersections(start_laser_1, step_x, threshold, model, laser_incline,
                         laser_aperture, false, &intersections_laser_1);

    // Origin coordinates of the lines forming the right laser plane.
    osg::Vec3d start_laser_2 = osg::Vec3d(
        -camera_x, -laser_distance + k * step_y - camera_y, -camera_z);
    // Array used to store right laser's intersection points.
    std::vector<osg::ref_ptr<osg::Vec3Array> > intersections_laser_2;

    // Computes the intersections of the right laser plane with the model.
    ComputeIntersections(start_laser_2, step_x, threshold, model, laser_incline,
                         laser_aperture, true, &intersections_laser_2);
    // cout<<"intersections_left.size() "<<intersections_right.size()<<"
    // intersections_right.size() "<<intersections_right.size()<<endl;

    if (!check_laser_1 && intersections_laser_1.size() != 0){
        check_laser_1 = true;
    }
    if (!check_laser_2 && intersections_laser_2.size() != 0){
        check_laser_2 = true;
    }
    
    // Increases the counter if no intersections have been found, resets it
    // otherwise.
    if (intersections_laser_1.size() == 0 && intersections_laser_2.size() == 0) {
      // if(intersections_left.size() == 0 )
      // if(intersections_right.size() == 0 )
      failed_intersections++;
    } else {
      failed_intersections = 0;
    }

    // Displays the intersections on the model.
    ShowIntersections(intersections_laser_1, intersection_line_geode);
    ShowIntersections(intersections_laser_2, intersection_line_geode);
    
    Mat intrinsics = Mat::eye(3, 3, CV_32F);
    intrinsics.at<float>(0, 0) = intrinsics_matrix(0, 0);
    intrinsics.at<float>(0, 2) = intrinsics_matrix(0, 2);
    intrinsics.at<float>(1, 1) = intrinsics_matrix(1, 1);
    intrinsics.at<float>(1, 2) = intrinsics_matrix(1, 2);
    intrinsics.at<float>(2, 2) = intrinsics_matrix(2, 2);
    Mat scene_laser_2 = Mat::zeros(input_parameters->camera_height,input_parameters->camera_width,CV_8UC1) ;
    if(intersections_laser_2.size()!=0)
        ProjectToImagePlane(intersections_laser_2, intrinsics, input_parameters, -k*step_y, scene_laser_2);
    Mat scene_laser_1 = Mat::zeros(input_parameters->camera_height,input_parameters->camera_width,CV_8UC1);
    if(intersections_laser_1.size()!=0)
        ProjectToImagePlane(intersections_laser_1, intrinsics, input_parameters, -k*step_y, scene_laser_1);
    
    // Advances to the next frame and takes the screenshot of the scene using
    // the previously defined callback.
    viewer.frame();
    
    Mat output = Mat::zeros(input_parameters->camera_height,input_parameters->camera_width,CV_8UC1);
    bitwise_or(scene_laser_2, scene_laser_1, output);
    
    // Processes the screenshot, extracting the points that will form the point
    // cloud.
    if(intersections_laser_2.size()!=0 || intersections_laser_1.size()!=0)
        ImageProcessing(output, intrinsics_matrix, input_parameters, 
                laser_1_counter * step_y, laser_2_counter * step_y, point_cloud_points);
    //if(k==2) 
    //  getchar();
    cout << "point_cloud_points.size(): " << point_cloud_points.size() << endl;

    // Removes the previously found intersection points from the intersection
    // geode so that they will not be displayed in the following iterations.
    intersection_line_geode->removeDrawables(
        1, intersections_laser_1.size() + intersections_laser_2.size());
    // intersection_line_geode->removeDrawables (1, intersections_left.size());
    // intersection_line_geode->removeDrawables (1, intersections_right.size());
    if(check_laser_1)
        laser_1_counter++;
    if(check_laser_2)
        laser_2_counter++;
    //cout<<"laser_1_counter "<<laser_1_counter<<endl;
    //cout<<"laser_2_counter "<<laser_2_counter<<endl;
  }

  // Builds and shows the point cloud using the points found.
  BuildPointCloud(point_cloud_points);

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
// intersection line-
// model: the model whose intersection with the laser plane will be computed.
// laser_incline: laser angle with respect to the horizon.
// laser_aperture: aperture angle of the laser.
// side: left or right laser
//
// Output parameters:
// intersections: vector storing the coordinates of the intersection points
void ComputeIntersections(
    osg::Vec3d start, float step_x, float threshold, osg::Node* model,
    float laser_incline, float laser_aperture, bool side,
    std::vector<osg::ref_ptr<osg::Vec3Array> >* intersections) {
  // Converts laser incline and aperture from degrees to radians.
  laser_incline = 2 * M_PI * laser_incline / 360;
  laser_aperture = 2 * M_PI * laser_aperture / 360;

  // Initializes laser height, length and half the size of laser width.
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

  // Array storing
  osg::ref_ptr<osg::Vec3Array> vertices(new osg::Vec3Array());
  // Computes the intersection of each line of the laser plane and the model.
  for (int i = 0; i < (abs(x_end - x_start) / step_x); i++) {
    // Defines the end coordinates of the line, taking the side (left or right
    // laser) into account.
    osg::Vec3d end;
    if (side) {  // Right laser.
      end = osg::Vec3d(end_x_coord - i * step_x + abs(x_end - x_start) / 2,
                       -end_y_coord + start.y(), end_z_coord);
    } else {  // Left laser.
      end = osg::Vec3d(end_x_coord - i * step_x + abs(x_end - x_start) / 2,
                       end_y_coord + start.y(), end_z_coord);
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
      // pushed into the intersection points vector, and the vertices vector is
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
  // If the vertices is not empty, its content is stored in the intersections
  // array. This is necessary to ensure the last line/point found by the
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
void ShowIntersections(std::vector<osg::ref_ptr<osg::Vec3Array> > intersections,
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

    // Adds the line (or the point) to the geode.
    intersection_line_geode->addDrawable(line);
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
bool IntrinsicsParser(std::string filename, osg::Matrixf& intrinsics_matrix,
                      std::vector<double>& distortion_matrix) {
  // Reads the input XML file.
  osgDB::XmlNode* node = osgDB::readXmlFile(filename);

  // Reads intrinsics.
  if (node->children.at(0)->properties["Rows"] != std::to_string(3) ||
      node->children.at(0)->properties["Cols"] != std::to_string(3)) {
    std::cout << "Error in intrinsics matrix." << std::endl;
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
    std::cout << "Error in distortion vector." << std::endl;
    return false;
  }
  for (int i = 0; i < 5; i++) {
    distortion_matrix.push_back(std::stod(
        node->children.at(1)->children.at(0)->children.at(i)->contents));
  }

  return true;
}

void ProjectToImagePlane(std::vector<osg::ref_ptr<osg::Vec3Array> > intersections,
 Mat intrinsics, struct InputParameters* input_parameters, float step, Mat& output) {
    Mat tVec(3, 1, DataType<float>::type); // Translation vector
    Mat rVec(3, 1, DataType<float>::type); // Rotation vector
     
    tVec.at<float>(0) = input_parameters->x_camera_absolute;
    tVec.at<float>(1) = input_parameters->y_camera_absolute + step;
    tVec.at<float>(2) = input_parameters->z_camera_absolute;
    rVec.at<float>(0) = 0;
    rVec.at<float>(1) = 0;
    rVec.at<float>(2) = 0;
        
    cv::Mat distCoeffs(5, 1, cv::DataType<float>::type);
    distCoeffs.at<float>(0) = 0;
    distCoeffs.at<float>(1) = 0;
    distCoeffs.at<float>(2) = 0;
    distCoeffs.at<float>(3) = 0;
    distCoeffs.at<float>(4) = 0;  
    
    Vec3i color;
    color[0] = 255;
    color[1] = 255;
    color[2] = 255;
    
    Mat image = Mat::zeros(input_parameters->camera_height,input_parameters->camera_width,CV_8UC1);
     
    for (int i = 0; i < intersections.size(); i++) {
    std::vector<cv::Point3f> points;
    std::vector<cv::Point2f> projectedPoints;
    
    if (intersections.at(i)->size() > 1) { // Ho una linea
      for(int k=0; k<intersections.at(i)->size(); k++) { // Intersections.at(i) 
                                                        // contiene più punti,
                                                        // i vertici della polilinea
        cv::Point3f p;
        p.x = intersections.at(i)->at(k).x();
        p.y = intersections.at(i)->at(k).y();
        p.z = intersections.at(i)->at(k).z();
        points.push_back(p);
      }    
      projectPoints(points, rVec, tVec, intrinsics, distCoeffs, projectedPoints);
      std::vector<cv::Point2i> projectedPoints_integer;
      for(int i = 0; i < projectedPoints.size(); i++){
          Point2i p;
          p.x = (int) projectedPoints.at(i).x;
          p.y = (int) projectedPoints.at(i).y;
          projectedPoints_integer.push_back(p);
      }
      polylines(image, projectedPoints_integer, 0, Scalar(255,255,255));     
      bitwise_or(image, output, output);
    } else { // Ho un punto
      Point3f point;
      point.x = intersections.at(i)->at(0).x();
      point.y = intersections.at(i)->at(0).y();
      point.z = intersections.at(i)->at(0).z();
      points.push_back(point);
      projectPoints(points, rVec, tVec, intrinsics, distCoeffs, projectedPoints); 
      Point2f proj_point = projectedPoints.at(0);
      output.at<uchar>(Point((int) proj_point.x, (int) proj_point.y)) = 255;
    }  
  }    
    
  //imwrite("test.png", output);
}