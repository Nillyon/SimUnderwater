#include "depth.h"
#include "rgb.h"
#include "light_model.h"
#include "camera_model.h"
#include "water_model.h"
#include "equations.h"
#include "box_visitor.h"

#include <iostream>
#include <osg/Camera>
#include <osg/Node>
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>
#include <cmath>
// GDAL
#if defined(_WIN32) || defined(__APPLE__)
#include "gdal_priv.h"
//#include "cpl_conv.h"
//#include "ogr_spatialref.h"
#else
#include "gdal/gdal_priv.h"
//#include "gdal/cpl_conv.h"
//#include "gdal/ogr_spatialref.h"
#endif

int main() {

    // Filename to read
    std::string filename;
    double xeye, yeye, zeye, xtarget, ytarget, ztarget, xup, yup, zup, tx, ty, tz, rx, ry, rz;
    std::cout << "Enter filename : ";
    std::cin>>filename;

    // Translation & Rotation vectors
    std::cout << "Enter translation : \n";
    std::cout << "tx : ";
    std::cin >> tx;
    std::cout << "ty : ";
    std::cin >>ty;
    std::cout << "tz : ";
    std::cin >> tz;

    std::cout << "Enter rotation : \n";
    std::cout << "rx : ";
    std::cin >> rx;
    std::cout << "ry : ";
    std::cin >>ry;
    std::cout << "rz : ";
    std::cin >> rz;

    // Conversion from pinhole camera model to OpenSceneGraph camera model
    xeye = tx;
    yeye = ty;
    zeye = tz;

    xup = sin(rz*M_PI/180);
    yup = cos(rz*M_PI/180);
    zup = 0;

    xtarget = xeye - sin(ry*M_PI/180);
    ytarget = yeye + sin(rx*M_PI/180);
    ztarget = zeye - cos(rx*M_PI/180) - cos(ry*M_PI/180) + 1;

    osg::Matrixd Rx = osg::Matrixd::rotate(rx,1,0,0);
    osg::Matrixd Ry = osg::Matrixd::rotate(ry,0,1,0);
    osg::Matrixd Rz = osg::Matrixd::rotate(rz,0,0,1);
    osg::Matrixd R = Rx*Ry*Rz;
    osg::Matrixd T = osg::Matrixd::translate(tx,ty,tz);



    // Entrée des vecteurs caméra
//    std::cout << "Enter eye vector : \n";
//    std::cout << "x : ";
//    std::cin>>xeye;
//    std::cout << "y : ";
//    std::cin>>yeye;
//    std::cout << "z : ";
//    std::cin>>zeye;
//
//    std::cout << "Enter target vector : \n";
//    std::cout << "x : ";
//    std::cin>>xtarget;
//    std::cout << "y : ";
//    std::cin>>ytarget;
//    std::cout << "z : ";
//    std::cin>>ztarget;
//
//    std::cout << "Enter up vector : \n";
//    std::cout << "x : ";
//    std::cin>>xup;
//    std::cout << "y : ";
//    std::cin>>yup;
//    std::cout << "z : ";
//    std::cin>>zup;

    osg::Vec3d eye(xeye,
                   yeye,
                   zeye);
    osg::Vec3d target( xtarget,
                       ytarget,
                       ztarget);
    osg::Vec3d up(xup,yup,zup);

    osg::Node* node = osgDB::readNodeFile(filename);
    osg::ref_ptr<osg::Camera> camera = new osg::Camera;
    camera->setClearMask( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    camera->setRenderOrder( osg::Camera::POST_RENDER );
    //camera->setReferenceFrame( osg::Camera::ABSOLUTE_RF );

    camera->setViewMatrixAsLookAt( eye, target, up);
    osg::ref_ptr<osg::Group> root = new osg::Group;
    root->addChild( node );
    root->addChild( camera );

    // Viewer construction
    osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer;
    viewer->setCameraManipulator(new osgGA::TrackballManipulator());
    viewer->getCameraManipulator()->setHomePosition(eye,target,up); // Viewer position
    viewer->setSceneData(root.get());

    std::string depthName;
    std::cout<<"Enter a name for your depth map : ";
    std::cin>>depthName;

    std::string rgbName;
    std::cout<<"Enter a name for your rgb image : ";
    std::cin>>rgbName;

    double pixel_size;
    std::cout<<"Enter your pixel size : ";
    std::cin>>pixel_size;

    double alpha, beta, u0, v0;

    BoxVisitor boxVisitor;
    node->accept(boxVisitor);

    osg::BoundingBox box = boxVisitor.getBoundingBox();

    // Create the edge of our picture
    double x_max = box.xMax();
    double x_min = box.xMin();
    double y_max = box.yMax();
    double y_min = box.yMin();
    double width = ceil((x_max-x_min)/pixel_size);
    double height = ceil((y_max-y_min)/pixel_size);

    double far=500;
    double near=0.1;

    // Normalized device coordinates
    osg::Matrixd NDC(-2.0f/width, 0.0f, 0.0f, 1.0f,
                    0.0f, 2.0f/height, 0.0f, -1.0f,
                    0.0f, 0.0f, -2.0f/(far-near), -(far+near)/(far-near),
                    0.0f, 0.0f, 0.0f, 1.0f);

    // Conversion Matrix from pinhole camera model to openGL/osg camera model
    osg::Matrixd KGL(-alpha, 0.0f, -(width-u0), 0.0f,
                    0.0f, -beta, -(height-v0), 0.0f,
                    0.0f, 0.0f, -(near+far), near*far,
                    0.0f, 0.0f, 1.0f, 0.0f);

    osg::Matrixd intrinsic = NDC*KGL;
    osg::Matrixd extrinsic = R*T;

    // 3D to image Matrix
//    osg::Matrixd Kimage(width/2, 0.0f, 0.0f, width/2,
//                       0.0f, height/2, 0.0f, height/2,
//                       0.0f, 0.0f, 0.0f, 1.0f);



    GDALAllRegister();

    bool save_rgb_image = RGB::process(node,rgbName,0,0, pixel_size, eye, target, up);

    if (save_rgb_image)
        std::cout<<"Your rgb image has succesfully been generated\n";
    else
    {
        std::cout<<"Error";
    }

    bool save_depth_map = Depth::Capture(node,pixel_size, 0, 0,depthName, eye, target, up);

    if (save_depth_map)
        std::cout<<"Your depth map has succesfully been generated\n";
    else
    {
        std::cout<<"Error";
    }

    GDALDestroyDriverManager();

    return viewer->run();
}
