#include "depth.h"
#include "rgb.h"
#include "light_model.h"
#include "camera_model.h"
#include "water_model.h"
#include "simulation.h"
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

    double xeyecam, yeyecam, zeyecam, xtargetcam, ytargetcam, ztargetcam, xupcam, yupcam, zupcam;
    double xeyelight, yeyelight, zeyelight, xtargetlight, ytargetlight, ztargetlight, xuplight, yuplight, zuplight;

    std::cout << "Enter filename : ";
    std::cin>>filename;

    std::string depthName;
    std::cout<<"Enter a name for your depth map : ";
    std::cin>>depthName;

    std::string rgbName;
    std::cout<<"Enter a name for your rgb image : ";
    std::cin>>rgbName;

    std::string simulatedName;
    std::cout<<"Enter a name for your simulated image : ";
    std::cin>>simulatedName;

    double pixel_size;
    std::cout<<"Enter your pixel size : ";
    std::cin>>pixel_size;

    // Node initialisation

    osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(filename);

    BoxVisitor boxVisitor;
    node->accept(boxVisitor);

    osg::BoundingBox box = boxVisitor.getBoundingBox();

    // Create the edge of our picture

    double x_max = box.xMax();
    double x_min = box.xMin();
    double y_max = box.yMax();
    double y_min = box.yMin();
    double width_pixel = ceil((x_max-x_min)/pixel_size);
    double height_pixel = ceil((y_max-y_min)/pixel_size);
    double width_meter = pixel_size*width_pixel;
    double height_meter = pixel_size*height_pixel;

    int width = (int)width_pixel;
    int height = (int)height_pixel;

    // Camera model initialisation

    double txcam, tycam, tzcam, rcam00, rcam01, rcam02, rcam10, rcam11, rcam12, rcam20, rcam21, rcam22, rxcam, rycam, rzcam;
    double alpha, beta, u0, v0, focal, fnumber, lenstransmittance;

    std::cout<<"Enter the parameters for your camera : \n";
    std::cout << "Translation : \n";
    std::cin >> txcam >> tycam >> tzcam;
    std::cout << "Rotation : \n";
    std::cin >> rxcam >> rycam >> rzcam;
//    std::cin >> rcam00 >> rcam01 >> rcam02 >> rcam10 >> rcam11 >> rcam12 >> rcam20 >> rcam21 >> rcam22;
    std::cout<<"Focal : ";
    std::cin>>focal;
    std::cout<<"X and Y scales : ";
    std::cin>>alpha>>beta;
    std::cout<<"X and Y offsets : ";
    std::cin>>u0>>v0;
    std::cout<<"F-number : ";
    std::cin>>fnumber;
    std::cout<<"Lens transmittance : ";
    std::cin>>lenstransmittance;

    CameraModel cameraModel(txcam, tycam, tzcam, rcam00, rcam01, rcam02, rcam10, rcam11, rcam12, rcam20, rcam21,
                            rcam22, focal, alpha, beta, u0, v0, lenstransmittance, fnumber);

    //  Coordinates to use if using setMatrixAsLookAt(eye,target,up)

    xeyecam = txcam;
    yeyecam = tycam;
    zeyecam = box.zMin()+tzcam;

    xupcam = sin(rzcam*M_PI/180);
    yupcam = cos(rzcam*M_PI/180);
    zupcam = 0;

    xtargetcam = xeyecam - sin(rycam*M_PI/180);
    ytargetcam = yeyecam + sin(rxcam*M_PI/180);
    ztargetcam = zeyecam - cos(rxcam*M_PI/180) - cos(rycam*M_PI/180);


    // Water model initialisation

    double Mr, Mg, Mb, cr, cg, cb;

    std::cout<<"Enter the parameters for the water : \n";
    std::cout<<"Attenuation coefficients for RGB : ";
    std::cin>>cr>>cg>>cb;
    std::cout<<"Reflectances for RGB : ";
    std::cin>>Mr>>Mg>>Mb;

    WaterModel waterModel(cr, cg, cb, Mr, Mg, Mb);


    // Light model initialisation

    double txlight,tylight,tzlight,rlight00,rlight01,rlight02,rlight10,rlight11,rlight12,rlight20,rlight21,rlight22, rxlight, rylight, rzlight;
    double Lr,Lg,Lb;

    std::cout<<"Enter the parameters for the light : ";
    std::cout<<"Translation : \n";
    std::cin>>txlight>>tylight>>tzlight;
    std::cout<<"Rotation : ";
    std::cin>>rxlight>>rylight>>rzlight;
//    std::cin>>rlight00>>rlight01>>rlight02>>rlight10>>rlight11>>rlight12>>rlight20>>rlight21>>rlight22;
    std::cout<<"Light Powers for RGB : ";
    std::cin>>Lr>>Lg>>Lb;

    LightModel lightModel(txlight, tylight, tzlight, rlight00, rlight01, rlight02, rlight10, rlight11, rlight12,
                          rlight20, rlight21, rlight22, Lr, Lg, Lb);


    //  Coordinates to use if using setMatrixAsLookAt(eye,target,up)

    xeyelight = txlight;
    yeyelight = tylight;
    zeyelight = box.zMin()+tzlight;

    xuplight = sin(rzlight*M_PI/180);
    yuplight = cos(rzlight*M_PI/180);
    zuplight = 0;

    xtargetlight = xeyelight - sin(rylight*M_PI/180);
    ytargetlight = yeyelight + sin(rxlight*M_PI/180);
    ztargetlight = zeyelight - cos(rxlight*M_PI/180) - cos(rylight*M_PI/180);

    // Transformation matrix for camera and its inverse

    osg::Matrixd TRcam = osg::Matrixd(rcam00, rcam01, rcam02, txcam,
                                      rcam10, rcam11, rcam12, tycam,
                                      rcam20, rcam21, rcam22, tzcam,
                                      0, 0, 0, 1);

    osg::Matrixd inverse = osg::Matrixd(rcam00, rcam10, rcam20, -rcam00*txcam-rcam10*tycam-rcam20*tzcam,
                                        rcam01, rcam11, rcam21, -rcam01*txcam-rcam11*tycam-rcam21*tzcam,
                                        rcam02, rcam12, rcam22, -rcam02*txcam-rcam12*tycam-rcam22*tzcam,
                                        0, 0, 0, 1);


    // Transformation matrix for light

    osg::Matrixd TRlight = osg::Matrixd(rlight00, rlight01, rlight02, txlight,
                                        rlight10, rlight11, rlight12, tylight,
                                        rlight20, rlight21, rlight22, tzlight,
                                        0, 0, 0, 1);


    // Projection matrix

    double far=100;
    double near=1;

    double focalx = cameraModel.getFocalX();
    double focaly = cameraModel.getFocalY();

    osg::Matrixd KGL = osg::Matrixd(-focalx, 0, -(width-u0), 0,
                                    0, -focaly, -(height-v0), 0,
                                    0, 0, -(near+far), near*far,
                                    0, 0, 1, 0);

    osg::Matrixd NDC = osg::Matrixd(-2.0f/(float)width, 0, 0, 1,
                                    0, 2.0f/(float)height, 0, -1,
                                    0, 0, -2.0f/(far-near), -(far+near)/(far-near),
                                    0, 0, 0, 1);

    osg::Matrixd intrinsic = NDC*KGL;


    osg::Vec3d eyecam(xeyecam,
                   yeyecam,
                   zeyecam);
    osg::Vec3d targetcam( xtargetcam,
                       ytargetcam,
                       ztargetcam);
    osg::Vec3d upcam(xupcam,yupcam,zupcam);

    osg::Vec3d eyelight(xeyelight,
                      yeyelight,
                      zeyelight);
    osg::Vec3d targetlight( xtargetlight,
                          ytargetlight,
                          ztargetlight);
    osg::Vec3d uplight(xuplight,yuplight,zuplight);

//    osg::ref_ptr<osg::Camera> camera = new osg::Camera;
//    camera->setClearMask( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
//    camera->setRenderOrder( osg::Camera::POST_RENDER );
//    //camera->setReferenceFrame( osg::Camera::ABSOLUTE_RF );
//    camera->setViewMatrix(TRcam);
//    camera->setProjectionMatrix(intrinsic);
//    osg::ref_ptr<osg::Group> root = new osg::Group;
//    root->addChild( node );
//    root->addChild( camera );



    GDALAllRegister();

    // Generation of depth, rgb and simulated image

    bool save_depth_map = Depth::Capture(node,pixel_size, 0, 0,depthName, eyecam, targetcam, upcam);

    if (save_depth_map)
        std::cout<<"Your depth map has succesfully been generated\n";
    else
    {
        std::cout<<"Error";
    }

    bool save_rgb_image = RGB::process(node,rgbName,0,0, pixel_size, eyecam, targetcam, upcam);

    if (save_rgb_image)
        std::cout<<"Your rgb image has succesfully been generated\n";
    else
    {
        std::cout<<"Error";
    }

//    Equations equations;
//    equations.computeDirectLight(node, pixel_size, eyecam, targetcam, upcam, inverse,
//                                 eyelight, targetlight, uplight,lightModel,waterModel,cameraModel);
//
//    bool save_simulation = Simulation::simulate(node,simulatedName,0,0,pixel_size,eyecam, targetcam, upcam,equations);
//
//    if (save_simulation)
//        std::cout<<"Your simulated image has succesfully been generated\n";
//    else
//    {
//        std::cout<<"Error";
//    }

    GDALDestroyDriverManager();

    // Viewer construction

    osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer;
    viewer->setCameraManipulator(new osgGA::TrackballManipulator());
    viewer->getCamera()->setClearColor(osg::Vec4(0.0f,0.0f,0.0f,0.0f));
    viewer->getCamera()->setProjectionMatrixAsOrtho2D(-width_meter/2,width_meter/2,-height_meter/2,height_meter/2);
    viewer->getCameraManipulator()->setHomePosition(eyecam, targetcam, upcam);
    viewer->setSceneData(node);

    return viewer->run();
}
