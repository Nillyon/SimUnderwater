#include "equations.h"
#include "water_model.h"
#include "camera_model.h"
#include "light_model.h"
#include "box_visitor.h"

#include <cmath>
#include <Eigen/Dense>
using namespace Eigen;
#include <osg/Camera>
#include <osg/Node>
#include <osgViewer/Viewer>
#include <osg/Texture2D>
#include <osgGA/TrackballManipulator>
#include <osg/MatrixTransform>
#include <osg/BlendFunc>

Equations::Equations() {
}

Matrix<double,Dynamic,Dynamic> Equations::getDirectLightR() {
    return m_DirectLightR;
}

Matrix<double,Dynamic,Dynamic> Equations::getDirectLightG() {
    return m_DirectLightG;
}

Matrix<double,Dynamic,Dynamic> Equations::getDirectLightB() {
    return m_DirectLightB;
}

void Equations::computeDirectLight(osg::ref_ptr<osg::Node> _node, const double _pixel_size,
                                   osg::Vec3d _eyecam, osg::Vec3d _targetcam, osg::Vec3d _upcam, osg::Matrixd _inverse,
                                   osg::Vec3d _eyelight, osg::Vec3d _targetlight, osg::Vec3d _uplight,
                                   LightModel _lightModel, WaterModel _waterModel, CameraModel _cameraModel) {
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
    int width = (int)width_pixel;
    int height = (int)height_pixel;

    compute3DCoordinates(_node, _pixel_size, _eyecam, _targetcam, _upcam, _inverse, _cameraModel);
    computeNormals(_node,_pixel_size);
    computeCamera(_node,_pixel_size,_eyecam, _targetcam, _upcam, _cameraModel);
    computeLight(_node,_pixel_size,_eyelight, _targetlight, _uplight,_lightModel);
    double Elr;
    double Elg;
    double Elb;
    double Lr=_lightModel.getLr();
    double Lg=_lightModel.getLg();
    double Lb=_lightModel.getLb();
    double cr=_waterModel.getcr();
    double cg=_waterModel.getcg();
    double cb=_waterModel.getcb();
    double Mr=_waterModel.getMr();
    double Mg=_waterModel.getMg();
    double Mb=_waterModel.getMb();
    double Tl=_cameraModel.getLensTransmittance();
    double focal=_cameraModel.getFocal();
    double fnumber=_cameraModel.getFnumber();

    m_DirectLightR.resize(height,width);
    m_DirectLightG.resize(height,width);
    m_DirectLightB.resize(height,width);

    for(int i=0; i<height; i++) {
        for (int j = 0; j < width; j++) {
            Elr=Lr*cos(m_Gamma(i,j))*exp(-cr*m_Rs(i,j))/(m_Rs(i,j)*m_Rs(i,j));
            Elg=Lg*cos(m_Gamma(i,j))*exp(-cg*m_Rs(i,j))/(m_Rs(i,j)*m_Rs(i,j));
            Elb=Lb*cos(m_Gamma(i,j))*exp(-cb*m_Rs(i,j))/(m_Rs(i,j)*m_Rs(i,j));
            m_DirectLightR(i,j)=Elr*exp(-cr*m_Rc(i,j))*Mr/M_PI*pow(cos(m_Theta(i,j)),4)*Tl*pow((m_Rc(i,j)-focal),2)*M_PI/(4*fnumber*m_Rc(i,j)*m_Rc(i,j));
            m_DirectLightG(i,j)=Elg*exp(-cg*m_Rc(i,j))*Mg/M_PI*pow(cos(m_Theta(i,j)),4)*Tl*pow((m_Rc(i,j)-focal),2)*M_PI/(4*fnumber*m_Rc(i,j)*m_Rc(i,j));
            m_DirectLightB(i,j)=Elb*exp(-cb*m_Rc(i,j))*Mb/M_PI*pow(cos(m_Theta(i,j)),4)*Tl*pow((m_Rc(i,j)-focal),2)*M_PI/(4*fnumber*m_Rc(i,j)*m_Rc(i,j));
        }
    }
}


void Equations::computeCamera(osg::ref_ptr<osg::Node> _node, const double _pixel_size, osg::Vec3d _eye, osg::Vec3d _target, osg::Vec3d _up, CameraModel _cameraModel) {

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
    traits->width = (int)width_pixel;
    traits->height = (int)height_pixel;
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

//  viewer.getCamera()->setViewMatrix(_extrinsicCamera);
//  viewer.getCamera()->setProjectionMatrix(_intrinsicCamera);
    viewer.getCamera()->setProjectionMatrixAsOrtho2D(-width_meter/2,width_meter/2,-height_meter/2,height_meter/2);
    viewer.getCameraManipulator()->setHomePosition(_eye, _target, _up);

    viewer.setSceneData( root.get() );
    viewer.realize();

    viewer.frame();

    float zmin = box.zMin();
    float zmax = box.zMax();

    float delta = zmax - zmin;
    int width = (int)width_pixel;
    int height = (int)height_pixel;

    zImage->readPixels(0,0,width,height, GL_DEPTH_COMPONENT, GL_FLOAT);
    float *buffer= new float[width];
    const float no_data = -9999.0f;
    double u0=_cameraModel.getU0();
    double v0=_cameraModel.getV0();

    m_Theta.resize(height, width);
    m_Rc.resize(height, width);

    for(int i=0; i<height; i++) {

        float *data = (float *) zImage->data();

        for (int j = 0; j < width; j++) {
            float val = data[(height - i - 1) * width + j];
            if (val == 1.0f) {
                buffer[j] = no_data;
                m_Theta(i,j) = M_PI/2;
            }
            else{
                buffer[j] = (1.0f - val) * delta + zmin;
                m_Theta(i,j) = asin(sqrt(pow((j-u0)*_pixel_size , 2) + pow((i-v0)*_pixel_size, 2)) / buffer[j]);
            }
            m_Rc(i,j)=sqrt(pow((m_3DCoordinates(i,j).x()-_cameraModel.getTx()),2)+
                           pow((m_3DCoordinates(i,j).y()-_cameraModel.getTy()),2)+
                           pow((m_3DCoordinates(i,j).z()-_cameraModel.getTz()),2));
        }
    }



    delete [] buffer;

    viewer.done();
    viewer.setSceneData(nullptr);

}

void Equations::computeLight(osg::ref_ptr<osg::Node> _node, const double _pixel_size, osg::Vec3d _eye, osg::Vec3d _target, osg::Vec3d _up, LightModel _lightModel) {
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
    traits->width = (int)width_pixel;
    traits->height = (int)height_pixel;
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

//  viewer.getCamera()->setViewMatrix(_extrinsicLight);
//  viewer.getCamera()->setProjectionMatrix(_intrinsicLight);
    viewer.getCamera()->setProjectionMatrixAsOrtho2D(-width_meter/2,width_meter/2,-height_meter/2,height_meter/2);
    viewer.getCameraManipulator()->setHomePosition(_eye, _target, _up);

    viewer.setSceneData( root.get() );
    viewer.realize();

    viewer.frame();

    float zmin = box.zMin();
    float zmax = box.zMax();

    float delta = zmax - zmin;
    int width = (int)width_pixel;
    int height = (int)height_pixel;

    zImage->readPixels(0,0,width,height, GL_DEPTH_COMPONENT, GL_FLOAT);
    float *buffer= new float[width];
    const float no_data = -9999.0f;

    double xlight = _lightModel.getTx();
    double ylight = _lightModel.getTy();
    double zlight = _lightModel.getTz();

    m_Gamma.resize(height, width);
    m_Rs.resize(height, width);

    for(int i=0; i<height; i++) {

        float *data = (float *) zImage->data();

        for (int j = 0; j < width; j++) {
            float val = data[(height - i - 1) * width + j];
            if (val == 1.0f) {
                buffer[j] = no_data;
                m_Gamma(i,j) = M_PI/2;
            }
            else{
                buffer[j] = (1.0f - val) * delta + zmin;
                double u1 = xlight-m_3DCoordinates(i,j).x();
                double u2 = ylight-m_3DCoordinates(i,j).y();
                double u3 = zlight-m_3DCoordinates(i,j).z();
                double v1=m_Normals(i,j).x();
                double v2=m_Normals(i,j).y();
                double v3=m_Normals(i,j).z();

                // Scalar product
                m_Gamma(i,j) = acos((u1*v1+u2*v2+u3*v3)/(sqrt(u1*u1+u2*u2+u3*u3)*sqrt(v1*v1+v2*v2+v3*v3)));
            }
            m_Rs(i,j)=sqrt(pow((m_3DCoordinates(i,j).x()-_lightModel.getTx()),2)+
                    pow((m_3DCoordinates(i,j).y()-_lightModel.getTy()),2)+
                    pow((m_3DCoordinates(i,j).z()-_lightModel.getTz()),2));
        }
    }

    delete [] buffer;

    viewer.done();
    viewer.setSceneData(nullptr);

}
void Equations::compute3DCoordinates(osg::ref_ptr<osg::Node> _node, const double _pixel_size,
                                     osg::Vec3d _eye, osg::Vec3d _target, osg::Vec3d _up, osg::Matrixd _inverse,
                                     CameraModel _cameraModel) {
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
    traits->width = (int)width_pixel;
    traits->height = (int)height_pixel;
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

//  viewer.getCamera()->setViewMatrix(_extrinsicCamera);
//  viewer.getCamera()->setProjectionMatrix(_intrinsicCamera);
    viewer.getCamera()->setProjectionMatrixAsOrtho2D(-width_meter/2,width_meter/2,-height_meter/2,height_meter/2);
    viewer.getCameraManipulator()->setHomePosition(_eye, _target, _up);

    viewer.setSceneData( root.get() );
    viewer.realize();

    viewer.frame();

    float zmin = box.zMin();
    float zmax = box.zMax();

    float delta = zmax - zmin;
    int width = (int)width_pixel;
    int height = (int)height_pixel;

    zImage->readPixels(0,0,width,height, GL_DEPTH_COMPONENT, GL_FLOAT);
    float *buffer= new float[width];
    const float no_data = -9999.0f;
    double u0=_cameraModel.getU0();
    double v0=_cameraModel.getV0();
    double focalx=_cameraModel.getFocalX();
    double focaly=_cameraModel.getFocalY();
    m_3DCoordinates.resize(height, width);

    for(int i=0; i<height; i++) {

        float *data = (float *) zImage->data();

        for (int j = 0; j < width; j++) {
            float val = data[(height - i - 1) * width + j];
            if (val == 1.0f) {
                buffer[j] = no_data;
                m_3DCoordinates(i,j).x() = no_data;
                m_3DCoordinates(i,j).y() = no_data;
                m_3DCoordinates(i,j).z() = no_data;
            }
            else {
                buffer[j] = (1.0f - val) * delta + zmin;
                m_3DCoordinates(i,j).x() = (j - u0) * buffer[j] / focalx;
                m_3DCoordinates(i,j).y() = (i - v0) * buffer[j] / focaly;
                m_3DCoordinates(i,j).z() = -buffer[j];
            }

            // We convert to the world space
            osg::Vec4 coordinates = osg::Vec4(m_3DCoordinates(i,j).x(), m_3DCoordinates(i,j).y(), m_3DCoordinates(i,j).z(), 1);
            coordinates = _inverse*coordinates;
            m_3DCoordinates(i,j).x() = coordinates.x();
            m_3DCoordinates(i,j).y() = coordinates.y();
            m_3DCoordinates(i,j).z() = coordinates.z();

        }


    }

    delete [] buffer;

    viewer.done();
    viewer.setSceneData(nullptr);

}

void Equations::computeNormals(osg::ref_ptr<osg::Node> _node, const double _pixel_size) {
    BoxVisitor boxVisitor;
    _node->accept(boxVisitor);

    osg::BoundingBox box = boxVisitor.getBoundingBox();
    double x_max = box.xMax();
    double x_min = box.xMin();
    double y_max = box.yMax();
    double y_min = box.yMin();
    double width_pixel = ceil((x_max-x_min)/_pixel_size);
    double height_pixel = ceil((y_max-y_min)/_pixel_size);
    int width = (int)width_pixel;
    int height = (int)height_pixel;

    m_Normals.resize(height, width);

    for(int i=0; i<height; i++) {
        for (int j = 0; j < width; j++) {
            double x1 = m_3DCoordinates(i,j).x();
            double y1 = m_3DCoordinates(i,j).y();
            double z1 = m_3DCoordinates(i,j).z();
            double x2, y2, z2;
            double x3, y3, z3;
            if (i==height-1) {
                if(j==width-1){
                    // On the left
                    x2 = m_3DCoordinates(i,j-1).x();
                    y2 = m_3DCoordinates(i,j-1).y();
                    z2 = m_3DCoordinates(i,j-1).z();

                }
                else{
                    // On the right
                    x2 = m_3DCoordinates(i,j+1).x();
                    y2 = m_3DCoordinates(i,j+1).y();
                    z2 = m_3DCoordinates(i,j+1).z();
                }
                // At the bottom
                x3 = m_3DCoordinates(i-1,j).x();
                y3 = m_3DCoordinates(i-1,j).y();
                z3 = m_3DCoordinates(i-1,j).z();

            }
            else{
                if(j==width-1){
                    // On the left
                    x2 = m_3DCoordinates(i,j-1).x();
                    y2 = m_3DCoordinates(i,j-1).y();
                    z2 = m_3DCoordinates(i,j-1).z();
                }
                else{
                    // On the right
                    x2 = m_3DCoordinates(i,j+1).x();
                    y2= m_3DCoordinates(i,j+1).y();
                    z2 = m_3DCoordinates(i,j+1).z();
                }
                // At the top
                x3 = m_3DCoordinates(i+1,j).x();
                y3 = m_3DCoordinates(i+1,j).y();
                z3 = m_3DCoordinates(i+1,j).z();
            }

            // Vectors

            double u1 = x2-x1;
            double u2 = y2-y1;
            double u3 = z2-z1;

            double v1 = x3-x1;
            double v2 = y3-y1;
            double v3 = z3-z1;

            // Cross product

            m_Normals(i,j).x()=u2*v3-u3*v2;
            m_Normals(i,j).y()=u3*v1-u1*v3;
            m_Normals(i,j).z()=u1*v2-u2*v1;
        }
    }
}

Matrix<Vector3d ,Dynamic,Dynamic> Equations::get3DCoordinates() {
    return m_3DCoordinates;
}

Matrix<Vector3d ,Dynamic,Dynamic> Equations::getNormals() {
    return m_Normals;
}

Matrix<double,Dynamic,Dynamic> Equations::getTheta() {
    return m_Theta;
}

Matrix<double,Dynamic,Dynamic> Equations::getGamma() {
    return m_Gamma;
}

Matrix<double,Dynamic,Dynamic> Equations::getRc() {
    return m_Rc;
}

Matrix<double,Dynamic,Dynamic> Equations::getRs() {
return m_Rs;
}

Equations::~Equations()
{
}
