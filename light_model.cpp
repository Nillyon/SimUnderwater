#include "light_model.h"

LightModel::LightModel()
{
}

void LightModel::setLightPower(double _Lr, double _Lg, double _Lb) {
    m_Lr=_Lr;
    m_Lg=_Lg;
    m_Lb=_Lb;
}

void LightModel::setTranslation(double _tx, double _ty, double _tz) {
    m_tx=_tx;
    m_ty=_ty;
    m_tz=_tz;
}

void LightModel::setRotation(double _rx, double _ry, double _rz) {
    m_rx=_rx;
    m_ry=_ry;
    m_rz=_rz;
}

LightModel::~LightModel()
{
}
