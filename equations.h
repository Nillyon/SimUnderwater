#ifndef SIMUNDERWATER_EQUATIONS_H
#define SIMUNDERWATER_EQUATIONS_H

#include "water_model.h"
#include "camera_model.h"
#include "light_model.h"
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


class Equations {
public:
    Equations();
    ~Equations();

    void setLightsNumber(int _lightsnumber);
    void setWaterModel(WaterModel _watermodel);
    void addLightModel(LightModel _lightmodel);
    void setCameraModel(CameraModel _cameramodel);
    bool simulate(osg::ref_ptr<osg::Node> _node, const double _pixel_size, double _refLatitude, double _refLongitude,
                 std::string fileName,  osg::Vec3d _eye, osg::Vec3d _target, osg::Vec3d _up);
    double getDirectLight();
    double getForwardScatter();
    double getBackScatter();
    std::vector<std::vector<double>> getTheta(osg::ref_ptr<osg::Node> _node, const double _pixel_size,
                    osg::Vec3d _eye, osg::Vec3d _target, osg::Vec3d _up);  //angle between the incoming ray and the camera's optical axis
    std::vector<std::vector<double>> getGamma(osg::ref_ptr<osg::Node> _node, const double _pixel_size,
                    osg::Vec3d _eye, osg::Vec3d _target, osg::Vec3d _up);  //angle between the ray from the light source and the structure's normal at X point
    std::vector<std::vector<double>> getRs(osg::ref_ptr<osg::Node> _node, const double _pixel_size,
                 osg::Vec3d _eye, osg::Vec3d _target, osg::Vec3d _up); //distance between light source and 3D model
    std::vector<std::vector<double>> getRc(osg::ref_ptr<osg::Node> _node, const double _pixel_size,
                 osg::Vec3d _eye, osg::Vec3d _target, osg::Vec3d _up); //distance between camera and 3D model
    osg::Vec3d getEyeVector(double _tx, double _ty, double _tz);
    osg::Vec3d getTargetVector(double _rx, double _ry, osg::Vec3d _eye);
    osg::Vec3d getUpVector(double _rz);

protected:
    int m_lightsnumber;
    WaterModel m_watermodel;
    std::vector<LightModel> m_lightmodels;
    CameraModel m_cameramodel;
    int m_N;    //number of backscatter slices



};


#endif //SIMUNDERWATER_EQUATIONS_H
