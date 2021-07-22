#include "light_model.h"

LightModel::LightModel(double _tx, double _ty, double _tz, double _r00, double _r01, double _r02, double _r10,
                       double _r11, double _r12, double _r20, double _r21, double _r22, double _Lr, double _Lg,
                       double _Lb) {
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
    m_Lr=_Lr;
    m_Lg=_Lg;
    m_Lb=_Lb;

}

double LightModel::getTx() {
    return m_tx;
}

double LightModel::getTy() {
    return m_ty;
}

double LightModel::getTz(){
    return m_tz;
}

double LightModel::getLr() {
    return m_Lr;
}

double LightModel::getLg() {
    return m_Lg;
}

double LightModel::getLb() {
    return m_Lb;
}

LightModel::~LightModel()
{
}
