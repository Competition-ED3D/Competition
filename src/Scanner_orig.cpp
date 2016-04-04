#include "Scanner.h"

double EuclideanDistance(osg::Vec3d point1, osg::Vec3d point2) {
  return sqrt(pow(point1.x() - point2.x(), 2) +
              pow(point1.y() - point2.y(), 2) +
              pow(point1.z() - point2.z(), 2));
}

int Scanner() {
  osg::Group* root = new osg::Group();
  osg::Geode* plane_geode = new osg::Geode();
  osg::Geometry* plane_left = new osg::Geometry();
  osg::Geometry* plane_right = new osg::Geometry();
  osg::Geometry* intersection_line_geometry = new osg::Geometry();
  osg::Geode* intersection_line_geode = new osg::Geode();

  osg::Node* model = NULL;

  model = osgDB::readNodeFile("cow.osg");

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

  osg::Vec4Array* intersection_line_color = new osg::Vec4Array;
  intersection_line_color->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 0.2f));  // index 0 red
  intersection_line_color->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 0.2f));  // index 1 green
  intersection_line_color->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 0.2f));  // index 2 blue
  intersection_line_color->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 0.2f));  // index 3 white
  intersection_line_color->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 0.2f));  // index 4 red

  plane_right->setColorArray(intersection_line_color);
  plane_right->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

  
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
  point->setSize(7);
  root->getOrCreateStateSet()->setAttribute(point);

  osg::LineWidth* linewidth = new osg::LineWidth();
  linewidth->setWidth(7.0f);
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
  viewer.setSceneData(root);
  
	unsigned int width=2500;
	unsigned int height=1500;
  osg::ref_ptr<osg::Camera> camera = new osg::Camera;
  
  InitializeCamera(&viewer, camera, width, height);

  std::cout << "Dopo nomeTantoCarino" << std::endl;
  
  viewer.setCameraManipulator(new osgGA::TrackballManipulator());
  viewer.realize();
  
  viewer.addSlave(camera.get(), osg::Matrixd(), osg::Matrixd());
  
  double step_x = 0.02;
  double threshold = 0.35;
  double step_y = 0.1;

  std::cout << "Prima del ciclo" << std::endl;

  
  for(int k=0; k<5; k++) {
    double x_start = -50;
    double x_end = 10;
    
    osg::Vec3d start_left = osg::Vec3d(5, 5 + k * step_y, 10);
    std::vector<osg::ref_ptr<osg::Vec3Array> > intersections_left;

    std::cout << "intersezioni sinistra" << std::endl;
    ComputeIntersections(start_left, step_x, step_y, threshold, model, x_start, 
                         x_end, false, &intersections_left);

    osg::Vec3d start_right = osg::Vec3d(-5, -(5 + k * step_y), 10);
    std::vector<osg::ref_ptr<osg::Vec3Array> > intersections_right;

    std::cout << "intersezioni destra" << std::endl;
    ComputeIntersections(start_right, step_x, step_y, threshold, model, x_start, 
                         x_end, true, &intersections_right);
    
   	ShowIntersections (intersections_right, intersection_line_color, intersection_line_geode);
   	ShowIntersections (intersections_left, intersection_line_color, intersection_line_geode);    
    
    viewer.frame();
    /////////////////////////////////////////////////////////
    sleep(1);
    intersection_line_geode->removeDrawables (1, intersections_left.size() + intersections_right.size());
  }

  return 0;
}

// TODO: aggiungere come parametro l'angolo alfa
void ComputeIntersections(osg::Vec3d start, double step_x, double step_y, double threshold,
                          osg::Node* model, double x_start, double x_end, bool side,
                          std::vector<osg::ref_ptr<osg::Vec3Array> > *intersections) {
    osg::ref_ptr<osg::Vec3Array> vertices(new osg::Vec3Array());   
    double const kPi = 3.14159265;
    double alpha = 2*kPi*19.98/360;
    
    
    double laser_height = EuclideanDistance(osg::Vec3d(start.x(),start.y(),-10),start);
    double laser_length = laser_height/sin(alpha);
    double end_x_coord = start.x();
    double end_y_coord = laser_length*sin(90-alpha); //coordinata y
    double end_z_coord = -10;
    
   
    
    for (int i = 0; i < (abs(x_end - x_start) / step_x); i++) {
      osg::Vec3d end;
      if(side) {//destra 
          //end = osg::Vec3d(x_start + i * step_x, -10+16.5, -10);
        end = osg::Vec3d(end_x_coord + i * step_x + abs(x_end - x_start)/2 , end_y_coord+16.5, end_z_coord);
         std::cout << "x_start: " << x_start << std::endl;
    std::cout << "y_start: -10" << std::endl;
       std::cout << "x_end right: " << end_x_coord + abs(x_end - x_start)/2 << std::endl; 
       std::cout << "y_end right: " << end_y_coord << std::endl; 
      }
      else {//sinistra
          //end = osg::Vec3d(x_start + i * step_x, -10, -10);
        end = osg::Vec3d(end_x_coord + i * step_x + abs(x_end - x_start)/2, end_y_coord, end_z_coord);
         std::cout << "x_start: " << x_start << std::endl;
    std::cout << "y_start: -10" << std::endl;
        std::cout << "x_end left: " << end_x_coord + abs(x_end - x_start)/2 << std::endl; 
       std::cout << "y_end left: " << end_y_coord << std::endl; 
      }
      osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector =
          new osgUtil::LineSegmentIntersector(start, end);
      osgUtil::IntersectionVisitor intersect_visitor(intersector.get(),
                                                    new MyReadCallback);
      model->accept(intersect_visitor);
      if (intersector->containsIntersections()) {
         if(vertices->size()!=0 &&
         (EuclideanDistance(vertices->at(vertices->size()-1),
          intersector->getFirstIntersection().localIntersectionPoint) >
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
                        osg::Vec4Array* intersection_line_color,
                        osg::Geode* intersection_line_geode) {
  for (int i = 0; i < intersections.size(); i++) {
        osg::Geometry* line = new osg::Geometry();
        line->setVertexArray(intersections.at(i));
        line->setColorArray(intersection_line_color);
        line->setColorBinding(osg::Geometry::BIND_OVERALL);
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

