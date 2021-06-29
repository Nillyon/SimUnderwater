#include "equations.h"
#include "water_model.h"
#include "camera_model.h"
#include "light_model.h"
#include "box_visitor.h"
#include <cmath>
#include <osg/Camera>
#include <osg/Node>
#include <osgViewer/Viewer>

#include <osgGA/TrackballManipulator>
#if defined(_WIN32) || defined(__APPLE__)
#include "gdal_priv.h"
#include "cpl_conv.h"
#include "ogr_spatialref.h"
#else
#include "gdal/gdal_priv.h"
#include "gdal/cpl_conv.h"
#include "gdal/ogr_spatialref.h"
#endif

Equations::Equations() {
}

void Equations::setLightsNumber(int _lightsnumber) {
    m_lightsnumber=_lightsnumber;
}

void Equations::setWaterModel(WaterModel _watermodel) {
    m_watermodel=_watermodel;
}

void Equations::addLightModel(LightModel _lightmodel) {
    m_lightmodels.push_back(_lightmodel);
}

void Equations::setCameraModel(CameraModel _cameramodel) {
    m_cameramodel=_cameramodel;
}

bool Equations::simulate(osg::ref_ptr<osg::Node> _node, const double _pixel_size, double _refLatitude,
                         double _refLongitude, std::string fileName, osg::Vec3d _eye, osg::Vec3d _target,
                         osg::Vec3d _up) {

}

std::vector<std::vector<double>> Equations::getTheta(osg::ref_ptr<osg::Node> _node, const double _pixel_size,
                                                     osg::Vec3d _eye, osg::Vec3d _target, osg::Vec3d _up) {

    BoxVisitor boxVisitor;
    _node->accept(boxVisitor);

    osg::BoundingBox box = boxVisitor.getBoundingBox();

    // Create the edge of our picture
    double x_max = box.xMax();
    double x_min = box.xMin();
    double y_max = box.yMax();
    double y_min = box.yMin();
    double width_pixel = ceil((x_max-x_min)/_pixel_size);
    double height_pixel = ceil((y_max-y_min)/_pixel_size);
    double width_meter = _pixel_size*width_pixel;
    double height_meter = _pixel_size*height_pixel;

    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->x = 0;
    traits->y = 0;
    traits->width = width_pixel;
    traits->height = height_pixel;
    traits->pbuffer = true;
    traits->alpha =  8;
    traits->depth = 24; //32; does not work in Windows
    traits->sharedContext = 0;
    traits->doubleBuffer = false;
    traits->samples = 0;
    traits->readDISPLAY();
    if(traits->displayNum < 0)
        traits->displayNum  = 0;
    traits->screenNum = 0;
    osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());

    osg::ref_ptr< osg::Group > root( new osg::Group );
    root->addChild( _node );

    // Create the viewer
    osgViewer::Viewer viewer;
    viewer.setThreadingModel( osgViewer::Viewer::SingleThreaded );
    viewer.setRunFrameScheme( osgViewer::ViewerBase::ON_DEMAND );

    osg::ref_ptr<osg::Camera> camera = new osg::Camera;
    camera->setGraphicsContext(gc);
    camera->setClearMask( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    camera->setViewport( 0, 0, (int)width_pixel, (int)height_pixel );
    camera->setClearColor(osg::Vec4(0., 0., 0., 0.));
    viewer.setCamera( camera.get() );

    osg::ref_ptr<osg::Image> zImage = new osg::Image;
    zImage->allocateImage((int)width_pixel, (int)height_pixel, 1, GL_DEPTH_COMPONENT , GL_FLOAT);
    camera->attach(osg::Camera::DEPTH_BUFFER, zImage.get()); /* */

    // put our model in the center of our viewer
    viewer.setCameraManipulator(new osgGA::TrackballManipulator());

    viewer.getCamera()->setProjectionMatrixAsOrtho2D(-width_meter/2,width_meter/2,-height_meter/2,height_meter/2);
    viewer.getCameraManipulator()->setHomePosition(_eye,_target,_up);

    viewer.setSceneData( root.get() );
    viewer.realize();

    viewer.frame();

    float zmin = box.zMin();
    float zmax = box.zMax();

    float delta = zmax - zmin;
    int width = width_pixel;
    int height = height_pixel;

    zImage->readPixels(0,0,width,height, GL_DEPTH_COMPONENT, GL_FLOAT);
    float *buffer= new float[width];
    const float no_data = -9999.0f;

    std::vector<std::vector<double>> Theta;
    for(int i=0; i<height; i++) {

        float *data = (float *) zImage->data();

        for (int j = 0; j < width; j++) {
            float val = data[(height - i - 1) * width + j];
            if (val == 1.0f) {
                buffer[j] = no_data;
                Theta[j][i] = M_PI/2;
            }
            else{
                buffer[j] = (1.0f - val) * delta + zmin;
                Theta[j][i] = asin(sqrt(pow(j*_pixel_size - width_meter / 2, 2) + pow(i*_pixel_size - height_meter / 2, 2)) / buffer[j]);
            }
        }
    }



    delete [] buffer;

    viewer.done();
    viewer.setSceneData(nullptr);

    return Theta;

}

std::vector<std::vector<double>> Equations::getGamma(osg::ref_ptr<osg::Node> _node, const double _pixel_size,
                                                     osg::Vec3d _eye, osg::Vec3d _target, osg::Vec3d _up) {
    BoxVisitor boxVisitor;
    _node->accept(boxVisitor);

    osg::BoundingBox box = boxVisitor.getBoundingBox();

    // Create the edge of our picture
    double x_max = box.xMax();
    double x_min = box.xMin();
    double y_max = box.yMax();
    double y_min = box.yMin();
    double width_pixel = ceil((x_max - x_min) / _pixel_size);
    double height_pixel = ceil((y_max - y_min) / _pixel_size);
    double width_meter = _pixel_size * width_pixel;
    double height_meter = _pixel_size * height_pixel;

    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->x = 0;
    traits->y = 0;
    traits->width = width_pixel;
    traits->height = height_pixel;
    traits->pbuffer = true;
    traits->alpha = 8;
    traits->depth = 24; //32; does not work in Windows
    traits->sharedContext = 0;
    traits->doubleBuffer = false;
    traits->samples = 0;
    traits->readDISPLAY();
    if (traits->displayNum < 0)
        traits->displayNum = 0;
    traits->screenNum = 0;
    osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());

    osg::ref_ptr<osg::Group> root(new osg::Group);
    root->addChild(_node);

    // Create the viewer
    osgViewer::Viewer viewer;
    viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
    viewer.setRunFrameScheme(osgViewer::ViewerBase::ON_DEMAND);

    osg::ref_ptr<osg::Camera> camera = new osg::Camera;
    camera->setGraphicsContext(gc);
    camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    camera->setViewport(0, 0, (int) width_pixel, (int) height_pixel);
    camera->setClearColor(osg::Vec4(0., 0., 0., 0.));
    viewer.setCamera(camera.get());

    osg::ref_ptr<osg::Image> zImage = new osg::Image;
    zImage->allocateImage((int) width_pixel, (int) height_pixel, 1, GL_DEPTH_COMPONENT, GL_FLOAT);
    camera->attach(osg::Camera::DEPTH_BUFFER, zImage.get()); /* */

    // put our model in the center of our viewer
    viewer.setCameraManipulator(new osgGA::TrackballManipulator());

    viewer.getCamera()->setProjectionMatrixAsOrtho2D(-width_meter / 2, width_meter / 2, -height_meter / 2,
                                                     height_meter / 2);
    viewer.getCameraManipulator()->setHomePosition(_eye, _target, _up);

    viewer.setSceneData(root.get());
    viewer.realize();

    viewer.frame();

    float zmin = box.zMin();
    float zmax = box.zMax();

    float delta = zmax - zmin;
    int width = width_pixel;
    int height = height_pixel;

    zImage->readPixels(0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT);
    float *buffer = new float[width];
    const float no_data = -9999.0f;

    osg::Matrixd InverseMatrix = viewer.getCameraManipulator()->getInverseMatrix();
    std::vector<std::vector<double>> Gamma;
    for (int i = 0; i < height; i++) {

        float *data = (float *) zImage->data();

        for (int j = 0; j < width; j++) {
            float val = data[(height - i - 1) * width + j];
            if (val == 1.0f) {
                buffer[j] = no_data;

                Gamma[j][i] = M_PI / 2;
            } else {
                buffer[j] = (1.0f - val) * delta + zmin;
                Gamma[j][i] = asin(
                        sqrt(pow(j * _pixel_size - width_meter / 2, 2) + pow(i * _pixel_size - height_meter / 2, 2)) /
                        buffer[j]);
            }
        }
    }
    delete[] buffer;

    viewer.done();
    viewer.setSceneData(nullptr);

    return Gamma;
}

std::vector<std::vector<double>> Equations::getRc(osg::ref_ptr<osg::Node> _node, const double _pixel_size,
                                                  osg::Vec3d _eye, osg::Vec3d _target, osg::Vec3d _up) {
    BoxVisitor boxVisitor;
    _node->accept(boxVisitor);

    osg::BoundingBox box = boxVisitor.getBoundingBox();

    // Create the edge of our picture
    double x_max = box.xMax();
    double x_min = box.xMin();
    double y_max = box.yMax();
    double y_min = box.yMin();
    double width_pixel = ceil((x_max - x_min) / _pixel_size);
    double height_pixel = ceil((y_max - y_min) / _pixel_size);
    double width_meter = _pixel_size * width_pixel;
    double height_meter = _pixel_size * height_pixel;

    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->x = 0;
    traits->y = 0;
    traits->width = width_pixel;
    traits->height = height_pixel;
    traits->pbuffer = true;
    traits->alpha = 8;
    traits->depth = 24; //32; does not work in Windows
    traits->sharedContext = 0;
    traits->doubleBuffer = false;
    traits->samples = 0;
    traits->readDISPLAY();
    if (traits->displayNum < 0)
        traits->displayNum = 0;
    traits->screenNum = 0;
    osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());

    osg::ref_ptr<osg::Group> root(new osg::Group);
    root->addChild(_node);

    // Create the viewer
    osgViewer::Viewer viewer;
    viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
    viewer.setRunFrameScheme(osgViewer::ViewerBase::ON_DEMAND);

    osg::ref_ptr<osg::Camera> camera = new osg::Camera;
    camera->setGraphicsContext(gc);
    camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    camera->setViewport(0, 0, (int) width_pixel, (int) height_pixel);
    camera->setClearColor(osg::Vec4(0., 0., 0., 0.));
    viewer.setCamera(camera.get());

    osg::ref_ptr<osg::Image> zImage = new osg::Image;
    zImage->allocateImage((int) width_pixel, (int) height_pixel, 1, GL_DEPTH_COMPONENT, GL_FLOAT);
    camera->attach(osg::Camera::DEPTH_BUFFER, zImage.get()); /* */

    // put our model in the center of our viewer
    viewer.setCameraManipulator(new osgGA::TrackballManipulator());

    viewer.getCamera()->setProjectionMatrixAsOrtho2D(-width_meter / 2, width_meter / 2, -height_meter / 2,
                                                     height_meter / 2);
    viewer.getCameraManipulator()->setHomePosition(_eye, _target, _up);

    viewer.setSceneData(root.get());
    viewer.realize();

    viewer.frame();

    float zmin = box.zMin();
    float zmax = box.zMax();

    float delta = zmax - zmin;
    int width = width_pixel;
    int height = height_pixel;

    zImage->readPixels(0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT);
    float *buffer = new float[width];
    const float no_data = -9999.0f;

    std::vector<std::vector<double>> Rc;
    for (int i = 0; i < height; i++) {

        float *data = (float *) zImage->data();

        for (int j = 0; j < width; j++) {
            float val = data[(height - i - 1) * width + j];
            if (val == 1.0f)
                buffer[j] = no_data;
            else
                buffer[j] = (1.0f - val) * delta + zmin;
            Rc[j][i]=buffer[j];

        }
    }
    delete [] buffer;

    viewer.done();
    viewer.setSceneData(nullptr);

    return Rc;
}

std::vector<std::vector<double>> Equations::getRs(osg::ref_ptr<osg::Node> _node, const double _pixel_size,
                                                  osg::Vec3d _eye, osg::Vec3d _target, osg::Vec3d _up) {
    BoxVisitor boxVisitor;
    _node->accept(boxVisitor);

    osg::BoundingBox box = boxVisitor.getBoundingBox();

    // Create the edge of our picture
    double x_max = box.xMax();
    double x_min = box.xMin();
    double y_max = box.yMax();
    double y_min = box.yMin();
    double width_pixel = ceil((x_max - x_min) / _pixel_size);
    double height_pixel = ceil((y_max - y_min) / _pixel_size);
    double width_meter = _pixel_size * width_pixel;
    double height_meter = _pixel_size * height_pixel;


    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->x = 0;
    traits->y = 0;
    traits->width = width_pixel;
    traits->height = height_pixel;
    traits->pbuffer = true;
    traits->alpha = 8;
    traits->depth = 24; //32; does not work in Windows
    traits->sharedContext = 0;
    traits->doubleBuffer = false;
    traits->samples = 0;
    traits->readDISPLAY();
    if (traits->displayNum < 0)
        traits->displayNum = 0;
    traits->screenNum = 0;
    osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());

    osg::ref_ptr<osg::Group> root(new osg::Group);
    root->addChild(_node);

    // Create the viewer
    osgViewer::Viewer viewer;
    viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
    viewer.setRunFrameScheme(osgViewer::ViewerBase::ON_DEMAND);

    osg::ref_ptr<osg::Camera> camera = new osg::Camera;
    camera->setGraphicsContext(gc);
    camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    camera->setViewport(0, 0, (int) width_pixel, (int) height_pixel);
    camera->setClearColor(osg::Vec4(0., 0., 0., 0.));
    viewer.setCamera(camera.get());

    osg::ref_ptr<osg::Image> zImage = new osg::Image;
    zImage->allocateImage((int) width_pixel, (int) height_pixel, 1, GL_DEPTH_COMPONENT, GL_FLOAT);
    camera->attach(osg::Camera::DEPTH_BUFFER, zImage.get()); /* */

    // put our model in the center of our viewer
    viewer.setCameraManipulator(new osgGA::TrackballManipulator());

    viewer.getCamera()->setProjectionMatrixAsOrtho2D(-width_meter / 2, width_meter / 2, -height_meter / 2,
                                                     height_meter / 2);
    viewer.getCameraManipulator()->setHomePosition(_eye, _target, _up);

    viewer.setSceneData(root.get());
    viewer.realize();

    viewer.frame();

    float zmin = box.zMin();
    float zmax = box.zMax();

    float delta = zmax - zmin;
    int width = width_pixel;
    int height = height_pixel;

    zImage->readPixels(0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT);
    float *buffer = new float[width];
    const float no_data = -9999.0f;

    std::vector<std::vector<double>> Rs;
    for (int i = 0; i < height; i++) {

        float *data = (float *) zImage->data();

        for (int j = 0; j < width; j++) {
            float val = data[(height - i - 1) * width + j];
            if (val == 1.0f)
                buffer[j] = no_data;
            else
                buffer[j] = (1.0f - val) * delta + zmin;
            Rs[j][i]=buffer[j];
        }
    }
    delete [] buffer;

    viewer.done();
    viewer.setSceneData(nullptr);

    return Rs;
}

osg::Vec3d Equations::getEyeVector(double _tx, double _ty, double _tz) {
    return osg::Vec3d(_tx,_ty,_tz);
}

osg::Vec3d Equations::getTargetVector(double _rx, double _ry, osg::Vec3d _eye) {
    return osg::Vec3d(_eye.x() - sin(_ry*M_PI/180), _eye.y() + sin(_rx*M_PI/180), _eye.z() - cos(_rx*M_PI/180) - cos(_ry*M_PI/180) + 1);
}

osg::Vec3d Equations::getUpVector(double _rz) {
    return osg::Vec3d(sin(_rz*M_PI/180), cos(_rz*M_PI/180), 0);
}


Equations::~Equations()
{
}
