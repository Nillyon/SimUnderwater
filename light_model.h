#ifndef SIMUNDERWATER_LIGHT_MODEL_H
#define SIMUNDERWATER_LIGHT_MODEL_H


class LightModel {
public:
    LightModel();
    ~LightModel();
    void setLightPower(double _Lr, double _Lg, double _Lb);
    void setTranslation(double _tx, double _ty, double _tz);
    void setRotation(double _rx, double _ry, double _rz);
protected:
    double m_Lr;    //red stripe
    double m_Lg;    //green stripe
    double m_Lb;    //blue stripe
    double m_tx;
    double m_ty;
    double m_tz;
    double m_rx;
    double m_ry;
    double m_rz;

};


#endif //SIMUNDERWATER_LIGHT_MODEL_H
