#ifndef SCREENCAPTURE_H
#define SCREENCAPTURE_H

#include <iostream>
#include <sstream>
#include <string>

#include <osg/Group>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Plane>
#include <osg/ShapeDrawable>
#include <osg/BlendFunc>
#include <osg/PositionAttitudeTransform>
#include <osg/LineSegment>
#include <osg/io_utils>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>
#include <osgUtil/PlaneIntersector>
#include <osgUtil/Optimizer>
#include <osg/LineWidth>
#include <osg/Point>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/Node>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <unistd.h>
#include <stdio.h>

class ScreenCapture : public osg::Camera::DrawCallback
{
    public:
    
        struct ContextData : public osg::Referenced
        {
        
            ContextData(osg::GraphicsContext* gc, GLenum readBuffer, const std::string& name);           
            void getSize(osg::GraphicsContext* gc, int& width, int& height);          
            void readPixels();
        
            typedef std::vector< osg::ref_ptr<osg::Image> >             ImageBuffer;
            typedef std::vector< GLuint > PBOBuffer;
        
            osg::GraphicsContext*   _gc;
            GLenum                  _readBuffer;
            std::string             _fileName;
            
            GLenum                  _pixelFormat;
            GLenum                  _type;
            int                     _width;
            int                     _height;
            
            unsigned int            _currentImageIndex;
            ImageBuffer             _imageBuffer;
            
            unsigned int            _currentPboIndex;
            PBOBuffer               _pboBuffer;

            unsigned int            _imgCount;

        };
    
        ScreenCapture(GLenum readBuffer);
          
        ContextData* createContextData(osg::GraphicsContext* gc) const;
        
        ContextData* getContextData(osg::GraphicsContext* gc) const;

        virtual void operator () (osg::RenderInfo& renderInfo) const {
            glReadBuffer(_readBuffer);

            osg::GraphicsContext* gc = renderInfo.getState()->getGraphicsContext();
            osg::ref_ptr<ContextData> cd = getContextData(gc);
            cd->readPixels();
        }
        
        typedef std::map<osg::GraphicsContext*, osg::ref_ptr<ContextData> > ContextDataMap;

        GLenum                      _readBuffer;
        mutable OpenThreads::Mutex  _mutex;
        mutable ContextDataMap      _contextDataMap;        
        
};

bool InitializeCamera(osgViewer::Viewer *viewer, osg::ref_ptr<osg::Camera> camera, 
                     unsigned int width, unsigned int height,  osg::Matrixd intrinsics_matrix);

#endif /* SCREENCAPTURE_H */

