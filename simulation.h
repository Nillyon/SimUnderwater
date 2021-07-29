#ifndef SIMUNDERWATER_SIMULATION_H
#define SIMUNDERWATER_SIMULATION_H

#include "water_model.h"
#include "camera_model.h"
#include "light_model.h"
#include "equations.h"

#include <iostream>
#include <osg/Camera>
#include <osg/Node>
#include <osgViewer/Viewer>
#include <Eigen/Dense>
using namespace Eigen;
#include <osgGA/TrackballManipulator>

class Simulation {
public:
    Simulation(const std::string &_filename, double _refLatitude, double _refLongitude, osg::BoundingBox _box, double _pixel_size);
    virtual void operator () (osg::RenderInfo& renderInfo, Equations _equations) const;

    bool status() const { return m_status; }

    static bool simulate(osg::ref_ptr<osg::Node> _node, const std::string &_filename, double _refLatitude, double _refLongitude,
                         double _pixel_size, osg::Vec3d _eye, osg::Vec3d _target, osg::Vec3d _up , Equations _equations, bool _disableTexture = false);

    ~Simulation();

private:
    std::string m_filename;
    osg::ref_ptr<osg::Image> m_image;
    osg::BoundingBox m_box;
    double m_pixel_size;
    double m_refLatitude;
    double m_refLongitude;
    bool m_status;
    mutable OpenThreads::Mutex m_mutex;
    Equations m_equations;
};


#endif //SIMUNDERWATER_SIMULATION_H
