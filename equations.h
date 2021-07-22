#ifndef SIMUNDERWATER_EQUATIONS_H
#define SIMUNDERWATER_EQUATIONS_H


#include "water_model.h"
#include "camera_model.h"
#include "light_model.h"
#include <iostream>
#include <osg/Camera>
#include <osg/Node>
#include <osgViewer/Viewer>
#include <Eigen/Dense>
using namespace Eigen;
#include <osgGA/TrackballManipulator>

class Equations {

public:
    Equations();
    ~Equations();


    Matrix<double,Dynamic,Dynamic> getDirectLightR();
    Matrix<double,Dynamic,Dynamic> getDirectLightG();
    Matrix<double,Dynamic,Dynamic> getDirectLightB();

    double maxDirectLight(Matrix<double,Dynamic,Dynamic>);
    double minDirectLight(Matrix<double,Dynamic,Dynamic>);

    void computeDirectLight(osg::ref_ptr<osg::Node> _node, const double _pixel_size,
                            osg::Matrixd _intrinsicCamera, osg::Matrixd _extrinsicCamera, osg::Matrixd _inverse,
                            osg::Matrixd _intrinsicLight, osg::Matrixd _extrinsicLight, LightModel _lightModel,
                            WaterModel _waterModel, CameraModel _cameraModel);
    void computeForwardScatter();
    void computeBackScatter();
    void compute3DCoordinates(osg::ref_ptr<osg::Node> _node, const double _pixel_size,
                              osg::Matrixd _intrinsicCamera, osg::Matrixd _extrinsicCamera, osg::Matrixd _inverse,
                              CameraModel _cameraModel);
    void computeNormals(osg::ref_ptr<osg::Node> _node, const double _pixel_size);
    void computeCamera(osg::ref_ptr<osg::Node> _node, const double _pixel_size,
                    osg::Matrixd _intrinsicCamera, osg::Matrixd _extrinsicCamera, CameraModel _cameraModel);  //function to measure every geometrical parameter relative to the camera
    void computeLight(osg::ref_ptr<osg::Node> _node, const double _pixel_size,
                      osg::Matrixd _intrinsicLight, osg::Matrixd _extrinsicLight, LightModel _lightModel);  //function to measure every geometrical parameter relative to the camera

    Matrix<Vector3d ,Dynamic,Dynamic> get3DCoordinates();
    Matrix<Vector3d ,Dynamic,Dynamic> getNormals();
    Matrix<double,Dynamic,Dynamic> getTheta();    //angle between the incoming ray and the camera's optical axis
    Matrix<double,Dynamic,Dynamic> getGamma();    //angle between the ray from the light source and the structure's normal at X point
    Matrix<double,Dynamic,Dynamic> getRs();   //distance between light source and 3D model
    Matrix<double,Dynamic,Dynamic> getRc();   //distance between camera and 3D model

//    osg::Vec3d getEyeVector(double _tx, double _ty, double _tz);
//    osg::Vec3d getTargetVector(double _rx, double _ry, osg::Vec3d _eye);
//    osg::Vec3d getUpVector(double _rz);

protected:
    Matrix<double,Dynamic,Dynamic> m_Theta;
    Matrix<double,Dynamic,Dynamic> m_Gamma;
    Matrix<double,Dynamic,Dynamic> m_Rs;
    Matrix<double,Dynamic,Dynamic> m_Rc;
    Matrix<Vector3d ,Dynamic,Dynamic> m_3DCoordinates;
    Matrix<Vector3d ,Dynamic,Dynamic> m_Normals;
    Matrix<double,Dynamic,Dynamic> m_DirectLightR;
    Matrix<double,Dynamic,Dynamic> m_DirectLightG;
    Matrix<double,Dynamic,Dynamic> m_DirectLightB;

};


#endif //SIMUNDERWATER_EQUATIONS_H
