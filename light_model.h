#ifndef SIMUNDERWATER_LIGHT_MODEL_H
#define SIMUNDERWATER_LIGHT_MODEL_H


class LightModel {
public:
    LightModel(double _tx, double _ty, double _tz, double _r00, double _r01, double _r02, double _r10, double _r11,
               double _r12, double _r20, double _r21, double _r22, double _Lr, double _Lg, double _Lb);
    ~LightModel();
    double getTx();
    double getTy();
    double getTz();
    double getLr();
    double getLg();
    double getLb();

protected:
    double m_Lr;    //red stripe
    double m_Lg;    //green stripe
    double m_Lb;    //blue stripe
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

};


#endif //SIMUNDERWATER_LIGHT_MODEL_H
