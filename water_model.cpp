#include "water_model.h"

WaterModel::WaterModel(double _cr, double _cg, double _cb, double _Mr, double _Mg, double _Mb) {
    m_cr=_cr;
    m_cg=_cg;
    m_cb=_cb;
    m_Mr=_Mr;
    m_Mg=_Mg;
    m_Mb=_Mb;
}

double WaterModel::getcr() {
    return m_cr;
}

double WaterModel::getcg() {
    return m_cg;
}

double WaterModel::getcb() {
    return m_cb;
}

double WaterModel::getMr() {
    return m_Mr;
}

double WaterModel::getMg() {
    return m_Mg;
}

double WaterModel::getMb() {
    return m_Mb;
}

WaterModel::~WaterModel()
{
}
