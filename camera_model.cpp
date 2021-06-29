#include "camera_model.h"

#include <osg/Camera>
#include <osg/Node>
#include <osgViewer/Viewer>
#include <osg/Geometry>
#include <osg/ShapeDrawable>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Renderer>


CameraModel::CameraModel()
{
}

double CameraModel::getTx() {
    return m_tx;
}

double CameraModel::getTy() {
    return m_ty;
}

double CameraModel::getTz(){
    return m_tz;
}

double CameraModel::getRx() {
    return m_rx;
}

double CameraModel::getRy() {
    return m_ry;
}

double CameraModel::getRz() {
    return m_rz;
}

double CameraModel::getFocal() {
    return m_focal;
}

double CameraModel::getLensTransmittance() {
    return m_Tl;
}

double CameraModel::getFnumber() {
    return m_fn;
}

double CameraModel::getXimage(double _X, double _Z)
{
    double x = _X*m_focalx/_Z+m_u0;
    return x;
}

double CameraModel::getYimage(double _Y, double _Z)
{
    double y = _Y*m_focaly/_Z+m_v0;
    return y;
}

void CameraModel::setTranslation(double _tx, double _ty, double _tz)
{
    m_tx=_tx;
    m_ty=_ty;
    m_tz=_tz;
}

void CameraModel::setRotation(double _rx, double _ry, double _rz)
{
    m_rx=_rx;
    m_ry=_ry;
    m_rz=_rz;
}

void CameraModel::setFocals(double _focal, double _alpha, double _beta) {   //_alpha is X pixel scale, _beta is Y pixel scale
    m_focal=_focal;
    m_focalx=_alpha*m_focal;
    m_focaly=_beta*m_focal;
}

void CameraModel::setOffsets(double _u0, double _v0) {
    m_u0=_u0;
    m_v0=_v0;
}

void CameraModel::setLensTransmittance(double _Tl) {
    m_Tl=_Tl;
}

void CameraModel::setFnumber(double _fn) {
    m_fn=_fn;
}

CameraModel::~CameraModel()
{
}
