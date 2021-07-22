#include "camera_model.h"


CameraModel::CameraModel(double _tx, double _ty, double _tz, double _r00, double _r01, double _r02, double _r10,
                         double _r11, double _r12, double _r20, double _r21, double _r22, double _focal, double _alpha,
                         double _beta, double _u0, double _v0, double _Tl, double _fn) {
    m_tx=_tx;
    m_ty=_ty;
    m_tz=_tz;
    m_r00 = _r00;
    m_r01 = _r01;
    m_r02 = _r02;
    m_r10 = _r10;
    m_r11 = _r11;
    m_r12 = _r12;
    m_r20 = _r20;
    m_r21 = _r21;
    m_r22 = _r22;
    m_focal=_focal;
    m_focalx=_alpha*m_focal;
    m_focaly=_beta*m_focal;
    m_u0=_u0;
    m_v0=_v0;
    m_Tl=_Tl;
    m_fn=_fn;
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


double CameraModel::getFocal() {
    return m_focal;
}

double CameraModel::getLensTransmittance() {
    return m_Tl;
}

double CameraModel::getFnumber() {
    return m_fn;
}

double CameraModel::getFocalX() {
    return m_focalx;
}

double CameraModel::getFocalY() {
    return m_focaly;
}

double CameraModel::getU0() {
    return m_u0;
}

double CameraModel::getV0() {
    return m_v0;
}

CameraModel::~CameraModel()
{
}
