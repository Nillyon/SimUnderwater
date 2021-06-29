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
    CameraModel();
    ~CameraModel();

    double getTx();
    double getTy();
    double getTz();
    double getRx();
    double getRy();
    double getRz();
    double getFocal();
    double getLensTransmittance();
    double getFnumber();
    double getXimage(double _X, double _Z);
    double getYimage(double _Y, double _Z);

    void setTranslation(double _tx, double _ty, double _tz);
    void setRotation(double _rx, double _ry, double _rz);
    void setFocals(double _focal, double _alpha, double _beta);
    void setOffsets(double _u0, double _v0);
    void setLensTransmittance(double _Tl);
    void setFnumber(double _fn);


protected:
    double m_tx;
    double m_ty;
    double m_tz;
    double m_rx;
    double m_ry;
    double m_rz;

    double m_focal;
    double m_focalx;
    double m_focaly;
    double m_u0;    //X offset
    double m_v0;    //Y offset
    double m_Tl;    //lens transmittance
    double m_fn;    //camera's f-number

};


#endif //SIMUNDERWATER_FOCAL_H
