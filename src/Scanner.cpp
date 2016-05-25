#include "Scanner.h"

float EuclideanDistance(osg::Vec3d point1, osg::Vec3d point2) {
  return sqrt(pow(point1.x() - point2.x(), 2) +
              pow(point1.y() - point2.y(), 2) +
              pow(point1.z() - point2.z(), 2));
}

int Scanner(InputParameters *input_parameters) {
  osg::Group* root = new osg::Group();
  osg::Geometry* intersection_line_geometry = new osg::Geometry();
  osg::Geode* intersection_line_geode = new osg::Geode();
  osg::Node* model = NULL;

  model = osgDB::readNodeFile(input_parameters->model_filename);

  intersection_line_geode->addDrawable(intersection_line_geometry);

  root->addChild(intersection_line_geode);
  root->addChild(model);

  osg::Point* point = new osg::Point;
  point->setSize(2.0f);
  root->getOrCreateStateSet()->setAttribute(point);

  osg::LineWidth* linewidth = new osg::LineWidth();
  linewidth->setWidth(2.0f);
  root->getOrCreateStateSet()->setAttributeAndModes(linewidth,
                                                    osg::StateAttribute::ON);

  root->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
  model->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);
  osgViewer::Viewer viewer;
  
  unsigned int width= input_parameters->camera_width;
  unsigned int height = input_parameters->camera_height;
  osg::ref_ptr<osg::Camera> camera = new osg::Camera;
  
  osg::Matrixf intrinsics_matrix;
  std::vector<double> distortion_matrix;  
  string filename = input_parameters->intrinsics_filename;
  IntrinsicsParser(filename, intrinsics_matrix, distortion_matrix);

  InitializeCamera(camera, width, height, intrinsics_matrix);
  
  osg::Matrixd cameraTrans;

  viewer.setSceneData(root);

  //viewer.setCameraManipulator(new osgGA::TrackballManipulator());
  osg::Vec3 posInWorld = model->getBound().center() * osg::computeLocalToWorld(model->getParentalNodePaths()[0]);
  cout<<"posInWorld: "<<posInWorld[0]<<" "<<posInWorld[1]<<" "<<posInWorld[2]<<endl;
  
  //float camera_x = 300; //valore usato prima di introdurre posInWorld
  float camera_x = -posInWorld[0] + input_parameters->x_camera_coord;
  //double camera_y = 340; // positivo: spostamento verso l'alto
  //float camera_y = 0; //valore usato prima di introdurre posInWorld
  float camera_y = -posInWorld[1] + input_parameters->y_camera_coord;
  //double camera_z = -750; // negativo: "alzare" la telecamera
  //double camera_z = -900; //valore usato prima di introdurre posInWorld
  float camera_z = posInWorld[2] + input_parameters->z_camera_coord;
  cout<<"camera_x "<<camera_x<<" "<<"camera_y "<<camera_y<<" "<<"camera_z "<<camera_z<<endl;

  cameraTrans.makeTranslate(camera_x, camera_y, camera_z );
  viewer.getCamera()->setViewMatrix(cameraTrans);
  viewer.getCamera()->setClearColor( osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f) ); 
  viewer.realize();
  
  viewer.addSlave(camera.get(), osg::Matrixd(), osg::Matrixd());
    
  float scanning_speed = input_parameters->scanning_speed;
  float fps = input_parameters->fps;
  
  //double step_x = 0.02;
  float step_x = 1;
  float threshold = EuclideanDistance(osg::Vec3d(0,0,0),osg::Vec3d(step_x,step_x,step_x));
  //double step_y = 0.1;
  float step_y = scanning_speed/fps;
  
  viewer.frame();
  
  ScreenCapture *sc = (ScreenCapture*) camera.get()->getFinalDrawCallback();
  osg::Image *screenshot = (sc->getContextData(camera.get()->getGraphicsContext()))->getImage();
  vector<Point3f> point_cloud_points;
  int failed_intersections = 0;
  //for(int k=0; k < 200; k++) {   
  for(int k=0; failed_intersections < 10; k++) {
    cout << "K: " << k << endl;
    cameraTrans.makeTranslate(camera_x, camera_y-k*step_y, camera_z);
    viewer.getCamera()->setViewMatrix(cameraTrans);

    float laser_distance = input_parameters->laser_distance;    
    //double laser_incline = 68.1301;
    //double laser_incline = 65;
    float laser_incline = input_parameters->laser_incline;
    //double laser_incline = 50.1301;
    //double laser_aperture = 22.62;
    float laser_aperture = input_parameters->laser_aperture;
    
    osg::Vec3d start_left = osg::Vec3d(-camera_x, laser_distance + k * step_y-camera_y, -camera_z);
    std::vector<osg::ref_ptr<osg::Vec3Array> > intersections_left;

    //std::cout << "intersezioni sinistra" << std::endl;
    ComputeIntersections(start_left, step_x, step_y, threshold, model, laser_incline, laser_aperture, false, &intersections_left);

    osg::Vec3d start_right = osg::Vec3d(-camera_x, -laser_distance + k * step_y-camera_y, -camera_z);
    std::vector<osg::ref_ptr<osg::Vec3Array> > intersections_right;

    //std::cout << "intersezioni destra" << std::endl;
    ComputeIntersections(start_right, step_x, step_y, threshold, model, laser_incline, laser_aperture, true, &intersections_right);
    //cout<<"intersections_left.size() "<<intersections_right.size()<<" intersections_right.size() "<<intersections_right.size()<<endl;
    
    if(intersections_left.size() == 0 && intersections_right.size() == 0 )
    //if(intersections_left.size() == 0 )
    //if(intersections_right.size() == 0 )
      failed_intersections++;
    else
      failed_intersections = 0;
            
    ShowIntersections (intersections_left, intersection_line_geode);
    ShowIntersections (intersections_right, intersection_line_geode);   
    
    viewer.frame();
    
    ImageProcessing(screenshot, intrinsics_matrix, input_parameters, k*step_y, point_cloud_points);
    cout << "point_cloud_points.size(): " << point_cloud_points.size() << endl;
    intersection_line_geode->removeDrawables (1, intersections_left.size() + intersections_right.size());
    //intersection_line_geode->removeDrawables (1, intersections_left.size());
    //intersection_line_geode->removeDrawables (1, intersections_right.size());
  }

  BuildPointCloud(point_cloud_points);
    
  return 0;
}

void ComputeIntersections(osg::Vec3d start, float step_x, float step_y, float threshold,
                          osg::Node* model, float laser_incline, float laser_aperture, bool side,
                          std::vector<osg::ref_ptr<osg::Vec3Array> > *intersections) {
    osg::ref_ptr<osg::Vec3Array> vertices(new osg::Vec3Array());   
    laser_incline = 2*M_PI*laser_incline/360;
    laser_aperture = 2*M_PI*laser_aperture/360;    
    
    float laser_height = EuclideanDistance(osg::Vec3d(start.x(),start.y(),-1000),start);
    float laser_length = laser_height/cos(M_PI/2-laser_incline);
    float laser_width_half = tan(laser_aperture/2)*laser_length;
    
    float end_x_coord = start.x();
    float end_y_coord = laser_length*sin(M_PI/2-laser_incline); //coordinata y
    float end_z_coord = -1000;
    
    float x_start = start.x() + laser_width_half;
    float x_end = start.x() - laser_width_half;
    
    if(start.y() < end_y_coord){
      end_y_coord = - end_y_coord;
    }
    
    for (int i = 0; i < (abs(x_end - x_start) / step_x); i++) {
      osg::Vec3d end;
      if(side) {//destra 
        end = osg::Vec3d(end_x_coord - i * step_x + abs(x_end - x_start)/2 , -end_y_coord+start.y(), end_z_coord);
      }
      else {//sinistra
        end = osg::Vec3d(end_x_coord - i * step_x + abs(x_end - x_start)/2, end_y_coord + start.y(), end_z_coord);
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
          vertices = new osg::Vec3Array();
        }
        vertices->push_back(
            intersector->getFirstIntersection().localIntersectionPoint);
      }
    }
    if (vertices->size() != 0) intersections->push_back(vertices);
}

void ShowIntersections (std::vector<osg::ref_ptr<osg::Vec3Array> > intersections,
                        osg::Geode* intersection_line_geode) {
  for (int i = 0; i < intersections.size(); i++) {
        osg::Geometry* line = new osg::Geometry();
        line->setVertexArray(intersections.at(i));
        //line->setColorArray(intersection_line_color);
        osg::Vec4Array* intersections_line_color = new osg::Vec4Array;
        for(int j = 0; j<intersections.at(i)->size(); j++){
          intersections_line_color->push_back(osg::Vec4(1,1,1,1));
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

bool IntrinsicsParser(std::string filename, osg::Matrixf &intrinsics_matrix, std::vector<double> &distortion_matrix) {
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