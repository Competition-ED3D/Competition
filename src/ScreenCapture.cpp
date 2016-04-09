#include "ScreenCapture.h"

ScreenCapture::ContextData::ContextData(osg::GraphicsContext* gc, GLenum readBuffer, const std::string& name):
                _gc(gc),
                _readBuffer(readBuffer),
                _fileName(name),
                _pixelFormat(GL_RGB),
                _type(GL_UNSIGNED_BYTE),
                _width(0),
                _height(0),
                _imgCount(0)

            {

                getSize(gc, _width, _height);
                
                std::cout<<"Window size "<<_width<<", "<<_height<<std::endl;
            
                // single buffered image
                _imageBuffer.push_back(new osg::Image);
                
        
            }
            
void ScreenCapture::ContextData::getSize(osg::GraphicsContext* gc, int& width, int& height)
            {
                if (gc->getTraits())
                {
                    width = gc->getTraits()->width;
                    height = gc->getTraits()->height;
                }
            }
                    
void ScreenCapture::ContextData::readPixels()
{
    int width=0, height=0;
    getSize(_gc, width, height);
    if (width!=_width || _height!=height)
    {
        std::cout<<"   Window resized "<<width<<", "<<height<<std::endl;
        _width = width;
        _height = height;
    }

    osg::Image* image = _imageBuffer[0].get();

    image->readPixels(0,0,_width,_height,
                      _pixelFormat,_type);

    _fileName = "test_" + std::to_string(_imgCount) + ".jpg";

    if (!_fileName.empty())
    {
        osgDB::writeImageFile(*image, _fileName);
    }

    _imgCount++;
}

ScreenCapture::ScreenCapture(GLenum readBuffer):
            _readBuffer(readBuffer)
        {
        }

ScreenCapture::ContextData* ScreenCapture::createContextData(osg::GraphicsContext* gc) const
        {
            std::stringstream filename;
            filename << "test_"<<_contextDataMap.size()<<".jpg";
            return new ContextData(gc, _readBuffer, filename.str());
        }
        
ScreenCapture::ContextData* ScreenCapture::getContextData(osg::GraphicsContext* gc) const
        {
            OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
            osg::ref_ptr<ContextData>& data = _contextDataMap[gc];
            if (!data) data = createContextData(gc);
            
            return data.get();
        }

bool InitializeCamera(osgViewer::Viewer *viewer, 
                    osg::ref_ptr<osg::Camera> camera, unsigned int width, unsigned int height, osg::Matrixd intrinsics_matrix) {
  GLenum readBuffer = GL_BACK;
  osg::ref_ptr<osg::GraphicsContext> pbuffer;
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	traits->x = 0;
	traits->y = 0;
	traits->width = width;
	traits->height = height;
	traits->red = 8;
	traits->green = 8;
	traits->blue = 8;
	traits->alpha = 0;
	traits->windowDecoration = false;
	traits->pbuffer = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;

	pbuffer = osg::GraphicsContext::createGraphicsContext(traits.get());
	if (!pbuffer.valid())
	{
    osg::notify(osg::NOTICE)<< "Pixel buffer has not been created successfully."<<std::endl;
    return false;
	}
  else
  {
    osg::notify(osg::NOTICE)<< "Pixel buffer has been created successfully."<<std::endl;
  }

  camera->setGraphicsContext(pbuffer.get());
  camera->setViewport(new osg::Viewport(0,0,width,height));
  GLenum buffer = pbuffer->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
  camera->setDrawBuffer(buffer);
  camera->setReadBuffer(buffer);
  camera->setFinalDrawCallback(new ScreenCapture(readBuffer));
  //camera->setProjectionMatrix(intrinsics_matrix);
  
  pbuffer->realize();
  
  return true;
}
