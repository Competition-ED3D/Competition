#include "ScreenCapture.h"

// Camera callback functions.

// Initializes the graphics context.
ScreenCapture::ContextData::ContextData(osg::GraphicsContext* gc,
                                        GLenum readBuffer, const string& name)
    : _gc(gc),
      _readBuffer(readBuffer),
      _pixelFormat(GL_BGR),
      _type(GL_UNSIGNED_BYTE),
      _width(0),
      _height(0),
      _imgCount(0) {
  getSize(gc, _width, _height);

  // single buffered image
  _imageBuffer.push_back(new osg::Image);
}

// Sets width and height of the image according to the values specified in the
// graphics context.
//
// Input parameters:
// gc: the graphics context.
//
// Output parameters:
// width: image width.
// height: image height.
void ScreenCapture::ContextData::getSize(osg::GraphicsContext* gc, int& width,
                                         int& height) {
  if (gc->getTraits()) {
    width = gc->getTraits()->width;
    height = gc->getTraits()->height;
  }
}

// Callback function that reads the image currently visualized by the camera.
void ScreenCapture::ContextData::readPixels() {
  int width = 0, height = 0;
  getSize(_gc, width, height);
  if (width != _width || _height != height) {
    cout << "   Window resized " << width << ", " << height << endl;
    _width = width;
    _height = height;
  }

  // Reads the image.
  osg::Image* image = _imageBuffer[0].get();
  image->readPixels(0, 0, _width, _height, _pixelFormat, _type);

  _imgCount++;
}

// Returns the image stored in the buffer.
osg::Image* ScreenCapture::ContextData::getImage() {
  return _imageBuffer[0].get();
}

ScreenCapture::ScreenCapture(GLenum readBuffer) : _readBuffer(readBuffer) {}

// Creates context data.
ScreenCapture::ContextData* ScreenCapture::createContextData(
    osg::GraphicsContext* gc) const {
  std::stringstream filename;
  filename << "test_" << _contextDataMap.size() << ".jpg";
  return new ContextData(gc, _readBuffer, filename.str());
}

// Returns context data.
ScreenCapture::ContextData* ScreenCapture::getContextData(
    osg::GraphicsContext* gc) const {
  OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
  osg::ref_ptr<ContextData>& data = _contextDataMap[gc];
  if (!data) data = createContextData(gc);
  return data.get();
}

// Sets parameters and intrinsics of the camera used to take screenshots of the
// scene. Returns true if successful.
//
// Input parameters:
// camera: the camera whose parameters are to be set.
// width: resolution width of the camera.
// height: resolution height of the camera.
// intrinsics_matrix: camera intrinsics.
bool InitializeCamera(osg::ref_ptr<osg::Camera> camera, unsigned int width,
                      unsigned int height, osg::Matrixf intrinsics_matrix) {
  GLenum readBuffer = GL_BACK;

  // Buffer for pixel storage.
  osg::ref_ptr<osg::GraphicsContext> pbuffer;

  // Initialzies the graphics context.
  osg::ref_ptr<osg::GraphicsContext::Traits> traits =
      new osg::GraphicsContext::Traits;
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
  if (!pbuffer.valid()) {
    osg::notify(osg::NOTICE)
        << "Pixel buffer has not been created successfully." << endl;
    return false;
  }

  // Sets the graphics context, viewport and buffers.
  camera->setGraphicsContext(pbuffer.get());
  camera->setViewport(new osg::Viewport(0, 0, width, height));
  GLenum buffer = pbuffer->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
  camera->setDrawBuffer(buffer);
  camera->setReadBuffer(buffer);

  // Sets the screencapture callback.
  camera->setFinalDrawCallback(new ScreenCapture(readBuffer));

  // Sets the intrinsics.
  camera->setProjectionMatrix(intrinsics_matrix);

  pbuffer->realize();

  return true;
}