#ifndef SIMUNDERWATER_WATER_MODEL_H
#define SIMUNDERWATER_WATER_MODEL_H


class WaterModel {
public :
    WaterModel(double _cr, double _cg, double _cb, double _Mr, double _Mg, double _Mb);
    ~WaterModel();
    double getcr();
    double getcg();
    double getcb();
    double getMr();
    double getMg();
    double getMb();

protected :
    double m_cr;    //red stripe
    double m_cg;    //green stripe
    double m_cb;    //blue stripe
    double m_Mr;
    double m_Mg;
    double m_Mb;
};


#endif //SIMUNDERWATER_WATER_MODEL_H
