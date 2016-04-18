#include "Scanner.h"

double EuclideanDistance(osg::Vec3d point1, osg::Vec3d point2) {
  return sqrt(pow(point1.x() - point2.x(), 2) +
              pow(point1.y() - point2.y(), 2) +
              pow(point1.z() - point2.z(), 2));
}

int Scanner(InputParameters *input_parameters) {
  osg::Group* root = new osg::Group();
  osg::Geode* plane_geode = new osg::Geode();
  osg::Geometry* plane_left = new osg::Geometry();
  osg::Geometry* plane_right = new osg::Geometry();
  osg::Geometry* intersection_line_geometry = new osg::Geometry();
  osg::Geode* intersection_line_geode = new osg::Geode();

  osg::Node* model = NULL;

  model = osgDB::readNodeFile("data/prodotto.stl");

  plane_geode->addDrawable(plane_left);
  plane_geode->addDrawable(plane_right);
  intersection_line_geode->addDrawable(intersection_line_geometry);

  root->addChild(intersection_line_geode);
  root->addChild(model);

  osg::Vec4Array* planes_color = new osg::Vec4Array;
  planes_color->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 0.2f));  // index 0 red
  planes_color->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 0.2f));  // index 1 green
  planes_color->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 0.2f));  // index 2 blue
  planes_color->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 0.2f));  // index 3 white
  planes_color->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 0.2f));  // index 4 red

  plane_left->setColorArray(planes_color);
  plane_left->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

  /*osg::Vec4Array* intersection_line_color = new osg::Vec4Array;
  intersection_line_color->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));  // index 0 red
  intersection_line_color->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));  // index 1 green
  intersection_line_color->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));  // index 2 blue
  intersection_line_color->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));  // index 3 white
  intersection_line_color->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));  // index 4 red

  plane_right->setColorArray(intersection_line_color);
  plane_right->setColorBinding(osg::Geometry::BIND_PER_VERTEX);*/
  
/*
  osg::Vec3Array* planeVertices1 = new osg::Vec3Array;
  planeVertices1->push_back(osg::Vec3(5, 6.5, 10));       // front left
  planeVertices1->push_back(osg::Vec3(-50, -10, -10));  // front right
  planeVertices1->push_back(osg::Vec3(10, -10, -10));   // back left
  plane_left->setVertexArray(planeVertices1);

  osg::DrawElementsUInt* plane1 =
      new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
  plane1->push_back(0);
  plane1->push_back(1);
  plane1->push_back(2);
  plane_left->addPrimitiveSet(plane1);

  osg::Vec3Array* planeVertices2 = new osg::Vec3Array;
  planeVertices2->push_back(osg::Vec3(-5, -6.5, 10));  // front left
  planeVertices2->push_back(osg::Vec3(-50, -10+16.5, -10));   // front right
  planeVertices2->push_back(osg::Vec3(10, -10+16.5, -10));    // back left
  plane_right->setVertexArray(planeVertices2);

  osg::DrawElementsUInt* plane2 =
      new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
  plane2->push_back(0);
  plane2->push_back(1);
  plane2->push_back(2);
  plane_right->addPrimitiveSet(plane2);
*/

  osg::Point* point = new osg::Point;
  point->setSize(2.0f);
  root->getOrCreateStateSet()->setAttribute(point);

  osg::LineWidth* linewidth = new osg::LineWidth();
  linewidth->setWidth(2.0f);
  root->getOrCreateStateSet()->setAttributeAndModes(linewidth,
                                                    osg::StateAttribute::ON);

  //////////////////////////////////////////////////////////////////////////////
  plane_geode->getOrCreateStateSet()->setRenderingHint(
      osg::StateSet::TRANSPARENT_BIN);
  plane_geode->getOrCreateStateSet()->setAttributeAndModes(
      new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA),
      osg::StateAttribute::ON);
  //////////////////////////////////////////////////////////////////////////////

  root->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
  osgViewer::Viewer viewer;
  
	unsigned int width=2024;
	unsigned int height=1088;
  osg::ref_ptr<osg::Camera> camera = new osg::Camera;
  
  osg::Matrixd intrinsics_matrix;
  std::vector<double> distortion_matrix;  
  IntrinsicsParser("src/camera.xml", intrinsics_matrix, distortion_matrix);
  
  InitializeCamera(&viewer, camera, width, height, intrinsics_matrix);

  std::cout << "Dopo nomeTantoCarino" << std::endl;
  
	osg::Matrixd cameraTrans;
  /*cameraRotation.makeRotate(
  osg::DegreesToRadians(-20.0), osg::Vec3(0,1,0), // roll
  osg::DegreesToRadians(-15.0), osg::Vec3(1,0,0) , // pitch
  osg::DegreesToRadians( 10.0), osg::Vec3(0,0,1) ); // heading */

  // 60 meters behind and 7 meters above the tank model
  viewer.setSceneData(root);
  
  double camera_x = 300;
  double camera_y = 0;
  double camera_z = -750;

  //viewer.setCameraManipulator(new osgGA::TrackballManipulator());

  cameraTrans.makeTranslate(camera_x, camera_y, camera_z );
  viewer.getCamera()->setViewMatrix(cameraTrans);

  viewer.realize();
  
  viewer.addSlave(camera.get(), osg::Matrixd(), osg::Matrixd());
  
  double scanning_speed = 100;
  double fps = 100;
  
  //double step_x = 0.02;
  double step_x = 1;
  //double threshold = 0.35;
  double threshold = EuclideanDistance(osg::Vec3d(0,0,0),osg::Vec3d(step_x,step_x,step_x));
  //double step_y = 0.1;
  double step_y = scanning_speed/fps;
  
  std::cout << "Prima del ciclo" << std::endl;

  viewer.frame();
  for(int k=0; k<15; k++) {
  //while(!viewer.done()){
    k=1;
    cameraTrans.makeTranslate(camera_x, camera_y-k*step_y, camera_z);
    viewer.getCamera()->setViewMatrix(cameraTrans);

    double laser_distance = 500;
    
    //double laser_incline = 68.1301;
    double laser_incline = 65;
    //double laser_incline = 50.1301;
    //double laser_aperture = 22.62;
    double laser_aperture = 45;
    
    osg::Vec3d start_left = osg::Vec3d(-camera_x, laser_distance + k * step_y-camera_y, -camera_z);
    std::vector<osg::ref_ptr<osg::Vec3Array> > intersections_left;

    std::cout << "intersezioni sinistra" << std::endl;
    ComputeIntersections(start_left, step_x, step_y, threshold, model, laser_incline, laser_aperture, false, &intersections_left);

    osg::Vec3d start_right = osg::Vec3d(-camera_x, -laser_distance + k * step_y-camera_y, -camera_z);
    std::vector<osg::ref_ptr<osg::Vec3Array> > intersections_right;

    std::cout << "intersezioni destra" << std::endl;
    ComputeIntersections(start_right, step_x, step_y, threshold, model, laser_incline, laser_aperture, true, &intersections_right);
    
   	ShowIntersections (intersections_right, intersection_line_geode);
   	ShowIntersections (intersections_left, intersection_line_geode);
    
    viewer.frame();
    /////////////////////////////////////////////////////////
    sleep(2);
    intersection_line_geode->removeDrawables (1, intersections_left.size() + intersections_right.size());
    k=15;
    //intersection_line_geode->removeDrawables (1, intersections_right.size());
  }

  

    
  return 0;
}

void ComputeIntersections(osg::Vec3d start, double step_x, double step_y, double threshold,
                          osg::Node* model, double laser_incline, double laser_aperture, bool side,
                          std::vector<osg::ref_ptr<osg::Vec3Array> > *intersections) {
    osg::ref_ptr<osg::Vec3Array> vertices(new osg::Vec3Array());   
    laser_incline = 2*M_PI*laser_incline/360;
    laser_aperture = 2*M_PI*laser_aperture/360;    
    
    double laser_height = EuclideanDistance(osg::Vec3d(start.x(),start.y(),-1000),start);
    double laser_length = laser_height/cos(M_PI/2-laser_incline);
    double laser_width_half = tan(laser_aperture/2)*laser_length;
    
    double end_x_coord = start.x();
    double end_y_coord = laser_length*sin(M_PI/2-laser_incline); //coordinata y
    double end_z_coord = -1000;
    
    double x_start = start.x() + laser_width_half;
    double x_end = start.x() - laser_width_half;
    
    if(start.y() < end_y_coord){
      end_y_coord = - end_y_coord;
    }
    
    for (int i = 0; i < (abs(x_end - x_start) / step_x); i++) {
      osg::Vec3d end;
      if(side) {//destra 
          //end = osg::Vec3d(x_start + i * step_x, -10+16.5, -10);
        end = osg::Vec3d(end_x_coord - i * step_x + abs(x_end - x_start)/2 , -end_y_coord+start.y(), end_z_coord);
        /*std::cout << "x_start: " << x_start << std::endl;
        std::cout << "y_start: " << start.y()<< std::endl;
        std::cout << "x_end right: " << end_x_coord + abs(x_end - x_start)/2 << std::endl; 
        std::cout << "y_end right: " << end_y_coord << std::endl;*/
      }
      else {//sinistra
          //end = osg::Vec3d(x_start + i * step_x, -10, -10);
        end = osg::Vec3d(end_x_coord - i * step_x + abs(x_end - x_start)/2, end_y_coord + start.y(), end_z_coord);
        /*std::cout << "x_start: " << x_start << std::end_x_coord + i * step_x + abs(x_end - x_start)/2ndl;
        std::cout << "y_start: " << start.y() << std::endl;
        std::cout << "x_end left: " << end_x_coord + abs(x_end - x_start)/2 << std::endl; 
        std::cout << "y_end left: " << end_y_coord + start.y() << std::endl;*/
      }
      osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector =
          new osgUtil::LineSegmentIntersector(start, end);
      osgUtil::IntersectionVisitor intersect_visitor(intersector.get());
      model->accept(intersect_visitor);
      if (intersector->containsIntersections()) {
         if(vertices->size()!=0 &&
         (EuclideanDistance(vertices->at(vertices->size()-1),
          intersector->getFirstIntersection().localIntersectionPoint) >=
          threshold)) {
          intersections->push_back(vertices);
          std::cout << "number of points: " << vertices->size() << std::endl;
          vertices = new osg::Vec3Array();
        }
        vertices->push_back(
            intersector->getFirstIntersection().localIntersectionPoint);
      }
    }
    if (vertices->size() != 0) intersections->push_back(vertices);
    std::cout << "number of points (finale): " << vertices->size() << std::endl;
}

void ShowIntersections (std::vector<osg::ref_ptr<osg::Vec3Array> > intersections,
                        osg::Geode* intersection_line_geode) {
  for (int i = 0; i < intersections.size(); i++) {
        osg::Geometry* line = new osg::Geometry();
        line->setVertexArray(intersections.at(i));
        //line->setColorArray(intersection_line_color);
        osg::Vec4Array* intersections_line_color = new osg::Vec4Array;
        for(int j = 0; j<intersections.at(i)->size(); j++){
          intersections_line_color->push_back(osg::Vec4(1,0,0,1));
        }
        line->setColorArray(intersections_line_color);
        line->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
        if (intersections.at(i)->size() > 1) {
          line->addPrimitiveSet(new osg::DrawArrays(
              osg::PrimitiveSet::LINE_STRIP, 0, intersections.at(i)->size()));
        } else {
          line->addPrimitiveSet(new osg::DrawArrays(
              osg::PrimitiveSet::POINTS, 0, intersections.at(i)->size()));
        }
        intersection_line_geode->addDrawable(line);
   }
}

bool IntrinsicsParser(std::string filename, osg::Matrixd &intrinsics_matrix, std::vector<double> &distortion_matrix) {
  osgDB::XmlNode *node = osgDB::readXmlFile (filename);
  
  if(node->children.at(0)->properties["Rows"] != std::to_string(3) || node->children.at(0)->properties["Cols"] != std::to_string(3)) {
    std::cout << "Matrice dei parametri intrinseci errata" << std::endl;
		return false;
	}  
  for(int i=0; i<3; i++) {
	  for(int j=0; j<3; j++) {
        intrinsics_matrix(i,j) = std::stod(node->children.at(0)->children.at(i)->children.at(j)->contents);
	  }
	}

  if(node->children.at(1)->properties["Rows"] != std::to_string(1) || node->children.at(1)->properties["Cols"] != std::to_string(5)) {
  std::cout << "Matrice dei coefficienti di distorsione errata" << std::endl;
		return false;
	} 
	for(int i=0; i<5; i++) {
    distortion_matrix.push_back(std::stod(node->children.at(1)->children.at(0)->children.at(i)->contents));
	}
  
  return true;
}

