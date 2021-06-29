#include "water_model.h"

WaterModel::WaterModel()
{
}

void WaterModel::setAttenuationCoefficient(double _cr, double _cg, double _cb) {
    m_cr=_cr;
    m_cg=_cg;
    m_cb=_cb;
}

void WaterModel::setReflectance(double _Mr, double _Mg, double _Mb) {
    m_Mr=_Mr;
    m_Mg=_Mg;
    m_Mb=_Mb;
}

WaterModel::~WaterModel()
{
}
