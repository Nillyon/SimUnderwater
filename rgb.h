#ifndef SIMUNDERWATER_RGB_H
#define SIMUNDERWATER_RGB_H

#include <osg/Camera>


struct RGB : public osg::Camera::DrawCallback
{
public:
    ///
    /// \brief RGB
    /// \param _filename
    /// \param _ref_lat_lon
    /// \param _box
    /// \param _pixel_size
    /// \param _parentWidget
    ///
    RGB(const std::string& _filename, double _refLatitude, double _refLongitude,osg::BoundingBox _box, double _pixel_size);

    virtual void operator () (osg::RenderInfo& renderInfo) const;

    bool status() const { return m_status; }

    static bool process(osg::ref_ptr<osg::Node> _node, const std::string& _filename, double _refLatitude, double _refLongitude, double _pixel_size, osg::Vec3d _eye, osg::Vec3d _target, osg::Vec3d _up, bool _disableTexture = false);

    ~RGB();

private:
    std::string m_filename;
    osg::ref_ptr<osg::Image> m_image;
    osg::BoundingBox m_box;
    double m_pixel_size;
    double m_refLatitude;
    double m_refLongitude;
    bool m_status;
    mutable OpenThreads::Mutex m_mutex;
};


#endif //SIMUNDERWATER_RGB_H
