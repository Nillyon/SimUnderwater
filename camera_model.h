#ifndef SIMUNDERWATER_FOCAL_H
#define SIMUNDERWATER_FOCAL_H

#include <osg/Camera>
#include <osg/Node>
#include <osgViewer/Viewer>
#include <osg/Geometry>
#include <osg/ShapeDrawable>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Renderer>

class CameraModel {
public:
    CameraModel(double _tx, double _ty, double _tz, double _r00, double _r01, double _r02, double _r10, double _r11,
                double _r12, double _r20, double _r21, double _r22, double _focal, double _alpha, double _beta,
                double _u0, double _v0, double _Tl, double _fn);
    ~CameraModel();

    double getTx();
    double getTy();
    double getTz();
    double getFocal();
    double getLensTransmittance();
    double getFnumber();
    double getFocalX();
    double getFocalY();
    double getU0();
    double getV0();

protected:
    double m_tx;
    double m_ty;
    double m_tz;
    double m_r00;
    double m_r01;
    double m_r02;
    double m_r10;
    double m_r11;
    double m_r12;
    double m_r20;
    double m_r21;
    double m_r22;

    double m_focal;
    double m_focalx;
    double m_focaly;
    double m_u0;    //X offset
    double m_v0;    //Y offset
    double m_Tl;    //lens transmittance
    double m_fn;    //camera's f-number

};


#endif //SIMUNDERWATER_FOCAL_H
