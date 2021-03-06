#include "simulation.h"
#include "equations.h"

#include <osg/RenderInfo>
#include <osg/Texture2D>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>
#include <osg/MatrixTransform>
#include <osg/Material>
#include <osg/BlendFunc>

#include <osgUtil/IntersectionVisitor>
#include <osgUtil/Optimizer>

// test
//#include <osgDB/WriteFile>

#include "node_user_data.h"
#include "box_visitor.h"

#if defined(_WIN32) || defined(__APPLE__)
#include "gdal_priv.h"
#include "cpl_conv.h"
#include "ogr_spatialref.h"
#else
#include "gdal/gdal_priv.h"
#include "gdal/cpl_conv.h"
#include "gdal/ogr_spatialref.h"
#endif

#include <iostream>

Simulation::Simulation(const std::string &_filename, double _refLatitude, double _refLongitude, osg::BoundingBox _box,
                       double _pixel_size) :
        m_filename( _filename ),
        m_refLatitude(_refLatitude),
        m_refLongitude(_refLongitude),
        m_box( _box ),
        m_pixel_size( _pixel_size ),
        m_status(true)
        {
        }

void Simulation::operator ()(osg::RenderInfo &renderInfo, Equations _equations) const
{   osg::Camera* camera = renderInfo.getCurrentCamera();

    osg::GraphicsContext* gc = camera->getGraphicsContext();

    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(m_mutex);

    if (gc->getTraits() != nullptr)
    {
        //GLenum buffer = camera->getGraphicsContext()->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
        osg::State& state = *renderInfo.getState();
        //state.glReadBuffer(camera->getDrawBuffer());
        state.glReadBuffer(GL_FRONT);

        int width = gc->getTraits()->width;
        int height = gc->getTraits()->height;
        osg::ref_ptr<osg::Image> image = new osg::Image;
        image->allocateImage(width, height, 1, GL_RGBA, GL_UNSIGNED_BYTE);

        // get the image
        image->readPixels( 0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE);

        // test
        //osgDB::writeImageFile(*image, "snap.png");

        // Variable for the command line "gdal_translate"
        double lat_0  = m_refLatitude;
        double lon_0 = m_refLongitude;
        double x_min = m_box.xMin();
        double y_max = m_box.yMax();

        std::string tiff_name = m_filename+".tif";

        CPLPushErrorHandler(CPLQuietErrorHandler);
        GDALDataset *geotiff_dataset;
        GDALDriver *driver_geotiff;

        driver_geotiff = GetGDALDriverManager()->GetDriverByName("GTiff");
        geotiff_dataset = driver_geotiff->Create(tiff_name.c_str(),width,height,4,GDT_Byte,NULL);

        int size = height*width;
        unsigned char *buffer_R = new unsigned char[width];
        unsigned char *buffer_G = new unsigned char[width];
        unsigned char *buffer_B = new unsigned char[width];
        unsigned char *buffer_A = new unsigned char[width];

        float min_buffer_R;
        float min_buffer_G;
        float min_buffer_B;

        float max_buffer_R;
        float max_buffer_G;
        float max_buffer_B;

        Matrix<double,Dynamic,Dynamic> directLightR = _equations.getDirectLightR();
        Matrix<double,Dynamic,Dynamic> directLightG = _equations.getDirectLightG();
        Matrix<double,Dynamic,Dynamic> directLightB = _equations.getDirectLightB();

        double min_directLightR = directLightR.minCoeff();
        double max_directLightR = directLightR.maxCoeff();

        double min_directLightG = directLightG.minCoeff();
        double max_directLightG = directLightG.maxCoeff();

        double min_directLightB = directLightB.minCoeff();
        double max_directLightB = directLightB.maxCoeff();

        for(int i=0; i<height; i++)
        {
            min_buffer_R=image->data(size - width*i - 1)[0];
            min_buffer_G=image->data(size - width*i - 1)[1];
            min_buffer_B=image->data(size - width*i - 1)[2];

            max_buffer_R=image->data(size - width*i - 1)[0];
            max_buffer_G=image->data(size - width*i - 1)[1];
            max_buffer_B=image->data(size - width*i - 1)[2];

            for(int j=0; j<(width); j++)
            {
                buffer_R[width-j-1] = image->data(size - ((width*i)+j) - 1)[0];
                buffer_G[width-j-1] = image->data(size - ((width*i)+j) - 1)[1];
                buffer_B[width-j-1] = image->data(size - ((width*i)+j) - 1)[2];
                buffer_A[width-j-1] = image->data(size - ((width*i)+j) - 1)[3];

                if(buffer_R[width-j-1]<min_buffer_R)
                    min_buffer_R=buffer_R[width-j-1];
                if(buffer_G[width-j-1]<min_buffer_G)
                    min_buffer_G=buffer_G[width-j-1];
                if(buffer_B[width-j-1]<min_buffer_B)
                    min_buffer_B=buffer_B[width-j-1];

                if(buffer_R[width-j-1]>max_buffer_R)
                    max_buffer_R=buffer_R[width-j-1];
                if(buffer_G[width-j-1]>max_buffer_G)
                    max_buffer_G=buffer_G[width-j-1];
                if(buffer_B[width-j-1]>max_buffer_B)
                    max_buffer_B=buffer_B[width-j-1];


            }
            // CPLErr GDALRasterBand::RasterIO( GDALRWFlag eRWFlag, int nXOff, int nYOff, int nXSize, int nYSize, void * pData, int nBufXSize, int nBufYSize, GDALDataType eBufType, int nPixelSpace, int nLineSpace )
            CPLErr res;
            res = geotiff_dataset->GetRasterBand(1)->RasterIO(GF_Write,0,i,width,1,buffer_R,width,1,GDT_Byte,0,0);
            res = geotiff_dataset->GetRasterBand(2)->RasterIO(GF_Write,0,i,width,1,buffer_G,width,1,GDT_Byte,0,0);
            res = geotiff_dataset->GetRasterBand(3)->RasterIO(GF_Write,0,i,width,1,buffer_B,width,1,GDT_Byte,0,0);
            res = geotiff_dataset->GetRasterBand(4)->RasterIO(GF_Write,0,i,width,1,buffer_A,width,1,GDT_Byte,0,0);
        }

        for(int i=0; i<height; i++)
        {
            for(int j=0; j<(width); j++)
            {
                buffer_R[width-j-1] = (uint8_t)(max_buffer_R*(image->data(size - ((width*i)+j) - 1)[0]*directLightR(i,width-j-1)-min_buffer_R*min_directLightR)/(max_buffer_R*max_directLightR-min_buffer_R*min_directLightR));
                buffer_G[width-j-1] = (uint8_t)(max_buffer_G*(image->data(size - ((width*i)+j) - 1)[1]*directLightG(i,width-j-1)-min_buffer_G*min_directLightG)/(max_buffer_G*max_directLightG-min_buffer_G*min_directLightG));
                buffer_B[width-j-1] = (uint8_t)(max_buffer_B*(image->data(size - ((width*i)+j) - 1)[2]*directLightB(i,width-j-1)-min_buffer_B*min_directLightB)/(max_buffer_B*max_directLightB-min_buffer_B*min_directLightB));
                buffer_A[width-j-1] = image->data(size - ((width*i)+j) - 1)[3];
            }
            // CPLErr GDALRasterBand::RasterIO( GDALRWFlag eRWFlag, int nXOff, int nYOff, int nXSize, int nYSize, void * pData, int nBufXSize, int nBufYSize, GDALDataType eBufType, int nPixelSpace, int nLineSpace )
            CPLErr res;
            res = geotiff_dataset->GetRasterBand(1)->RasterIO(GF_Write,0,i,width,1,buffer_R,width,1,GDT_Byte,0,0);
            res = geotiff_dataset->GetRasterBand(2)->RasterIO(GF_Write,0,i,width,1,buffer_G,width,1,GDT_Byte,0,0);
            res = geotiff_dataset->GetRasterBand(3)->RasterIO(GF_Write,0,i,width,1,buffer_B,width,1,GDT_Byte,0,0);
            res = geotiff_dataset->GetRasterBand(4)->RasterIO(GF_Write,0,i,width,1,buffer_A,width,1,GDT_Byte,0,0);
        }


        // Setup output coordinate system.
        double geo_transform[6] = { x_min, m_pixel_size, 0, y_max, 0, -m_pixel_size };
        geotiff_dataset->SetGeoTransform(geo_transform);
        char *geo_reference = NULL;
        OGRSpatialReference o_SRS;
        o_SRS.SetTM(lat_0,lon_0,0.9996,0,0);
        o_SRS.SetWellKnownGeogCS( "WGS84" );
        o_SRS.exportToWkt( &geo_reference );


        GDALClose(geotiff_dataset);

        CPLFree( geo_reference );

        CPLPopErrorHandler();

        delete [] buffer_R;
        delete [] buffer_G;
        delete [] buffer_B;
        delete [] buffer_A;

        const_cast<Simulation*>(this)->m_status = true;
    }
    else
    {
        const_cast<Simulation*>(this)->m_status = false;
    }

}

bool Simulation::simulate(osg::ref_ptr<osg::Node> _node, const std::string &_filename, double _refLatitude, double _refLongitude, double _pixel_size, osg::Vec3d _eye, osg::Vec3d _target, osg::Vec3d _up, Equations _equations , bool _disableTexture)
{

    BoxVisitor boxVisitor;
    _node->accept(boxVisitor);

    osg::BoundingBox box = boxVisitor.getBoundingBox();

    // Create the edge of our picture
    // Set graphics contexts
    double x_max = box.xMax();
    double x_min = box.xMin();
    double y_max = box.yMax();
    double y_min = box.yMin();
    int width_pixel = ceil((x_max-x_min)/_pixel_size);
    int height_pixel = ceil((y_max-y_min)/_pixel_size);
    double width_meter = _pixel_size*width_pixel;
    double height_meter = _pixel_size*height_pixel;
    double cam_center_x = (x_max+x_min)/2;
    double cam_center_y = (y_max+y_min)/2;

    osg::ref_ptr<NodeUserData> data = (NodeUserData*)(_node->getUserData());
    bool isComposite = data != nullptr && data->composite;
    osg::ref_ptr<osg::Node> triangles;

    osg::StateSet *stateSet= _node->getOrCreateStateSet();
    if(isComposite)
    {
        // process generated triangles
        osg::ref_ptr<osg::MatrixTransform> matrix = dynamic_cast<osg::MatrixTransform*>(_node.get());
        osg::ref_ptr<osg::Node> root = matrix->getChild(0);
        osg::Geode *geode = root->asGeode();
        unsigned int num_drawables = geode->getNumDrawables();
        for( unsigned int i = 0; i < num_drawables; i++ )
        {
            // Use 'asGeometry' as its supposed to be faster than a dynamic_cast
            // every little saving counts
            osg::Geometry *current_geometry = geode->getDrawable(i)->asGeometry();

            // Only process if the drawable is geometry
            if ( current_geometry )
            {
                // get the list of different geometry mode which were created
                osg::Geometry::PrimitiveSetList primitive_list = current_geometry->getPrimitiveSetList();

                for(unsigned int j = 0; j < primitive_list.size(); j++)
                {
                    osg::PrimitiveSet *primitive_set = primitive_list[j];
                    if(primitive_set->getMode() == osg::PrimitiveSet::POINTS)
                        continue;

                    // we have triangles here
                    stateSet = current_geometry->getOrCreateStateSet();
                    triangles = current_geometry;
                }
            }
        }
    }

    // disable texturing
    if(_disableTexture)
    {
        unsigned int mode = osg::StateAttribute::OVERRIDE|osg::StateAttribute::OFF;
        for( unsigned int ii=0; ii < 4; ii++ )
        {
            stateSet->setTextureMode( ii, GL_TEXTURE_1D, mode );
            stateSet->setTextureMode( ii, GL_TEXTURE_2D, mode );
            stateSet->setTextureMode( ii, GL_TEXTURE_3D, mode );
            stateSet->setTextureMode( ii, GL_TEXTURE_RECTANGLE, mode );
            stateSet->setTextureMode( ii, GL_TEXTURE_CUBE_MAP, mode);
        }
    }

    // get BLEND mode and alpha value
    osg::StateAttribute::GLModeValue blend = stateSet->getMode(GL_BLEND);
    stateSet->setMode( GL_BLEND, osg::StateAttribute::OFF);
    osg::StateAttribute* attr = stateSet->getAttribute(osg::StateAttribute::MATERIAL);
    osg::Material* material = dynamic_cast<osg::Material*>(attr);
    double alpha = 1.0;
    if(material != nullptr)
    {
        osg::Vec4 amb = material->getAmbient(osg::Material::FRONT);
        alpha = amb.a();
        stateSet->removeAttribute(osg::StateAttribute::MATERIAL);
    }

    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->x = 0;
    traits->y = 0;
    traits->width = width_pixel;
    traits->height = height_pixel;
    traits->pbuffer = true;
    //    traits->red = 8;      // = default value
    //    traits->green = 8;    // = default value
    //    traits->blue = 8;     // = default value
    traits->alpha = 8;
    //    traits->alpha = 1;
    //traits->depth = 32;   // default value = 24
    traits->sharedContext = 0; // = default value
    traits->doubleBuffer = false;
    traits->readDISPLAY();
    if(traits->displayNum < 0)
        traits->displayNum  = 0;
    traits->screenNum = 0;
    //    traits->setUndefinedScreenDetailsToDefaultScreen();

    osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());

    osg::ref_ptr< osg::Group > root( new osg::Group );
    if(isComposite && triangles != nullptr)
    {
        root->addChild( triangles );
    }
    else
    {
        root->addChild( _node );
    }

    // setup MRT camera
    osg::ref_ptr<osg::Camera> mrt_camera = new osg::Camera;
    mrt_camera->setGraphicsContext(gc);
    mrt_camera->setClearMask( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    mrt_camera->setRenderTargetImplementation( osg::Camera::FRAME_BUFFER );
    mrt_camera->setRenderOrder( osg::Camera::PRE_RENDER );
    mrt_camera->setViewport( 0, 0, width_pixel, height_pixel );
    mrt_camera->setClearColor(osg::Vec4(0., 0., 0., 0.));

    // Create our Texture
    osg::Texture2D* tex = new osg::Texture2D;
    tex->setTextureSize( width_pixel, height_pixel );
    tex->setSourceType( GL_UNSIGNED_BYTE );
    tex->setSourceFormat( GL_RGBA );
    tex->setInternalFormat( GL_RGBA32F_ARB );
    tex->setInternalFormatMode(osg::Texture2D::USE_IMAGE_DATA_FORMAT);
    tex->setFilter( osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR );
    tex->setFilter( osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR );
    mrt_camera->attach( osg::Camera::COLOR_BUFFER, tex );

    // set RTT textures to quad
    osg::Geode* geode( new osg::Geode );
    geode->addDrawable( osg::createTexturedQuadGeometry(
            osg::Vec3(-1,-1,0), osg::Vec3(2.0,0.0,0.0), osg::Vec3(0.0,2.0,0.0)) );
    geode->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex);
    geode->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

    // configure postRenderCamera to draw fullscreen textured quad
    osg::Camera* post_render_camera( new osg::Camera );
    post_render_camera->setClearMask( 0 );
    post_render_camera->setRenderTargetImplementation( osg::Camera::FRAME_BUFFER, osg::Camera::FRAME_BUFFER );
    post_render_camera->setReferenceFrame( osg::Camera::ABSOLUTE_RF );
    post_render_camera->setComputeNearFarMode( osg::Camera::DO_NOT_COMPUTE_NEAR_FAR );
    post_render_camera->setRenderOrder( osg::Camera::POST_RENDER );
    post_render_camera->setDrawBuffer(GL_FRONT);
    post_render_camera->setReadBuffer(GL_FRONT);
    post_render_camera->setViewMatrix( osg::Matrixd::identity() );
    post_render_camera->setProjectionMatrix( osg::Matrixd::identity() );

    post_render_camera->addChild( geode );

    root->addChild(post_render_camera);

    // Create the viewer
    osgViewer::Viewer viewer;
    viewer.setThreadingModel( osgViewer::Viewer::SingleThreaded );
    viewer.setUpThreading();
    viewer.setRunFrameScheme( osgViewer::ViewerBase::ON_DEMAND );
    viewer.setCamera( mrt_camera.get() );

    // put our model in the center of our viewer
    viewer.setCameraManipulator(new osgGA::TrackballManipulator());
    double cam_center_z= (x_max-x_min)/2 + (y_max-y_min)/2 ;

//  viewer.getCamera()->setViewMatrix(_extrinsic);
//  viewer.getCamera()->setProjectionMatrix(_intrinsic);
    viewer.getCamera()->setProjectionMatrixAsOrtho2D(-width_meter/2,width_meter/2,-height_meter/2,height_meter/2);
    viewer.getCameraManipulator()->setHomePosition(_eye, _target, _up);

    viewer.setSceneData( root.get() );
    viewer.realize();

    // setup the callback
    osg::BoundingBox image_bounds;
    image_bounds.xMin() = cam_center_x-width_meter/2;
    image_bounds.xMax() = cam_center_x+width_meter/2;
    image_bounds.yMin() = cam_center_y-height_meter/2;
    image_bounds.yMax() = cam_center_y+height_meter/2;

    viewer.home();

    Simulation* final_draw_callback = new Simulation(_filename,_refLatitude, _refLongitude, image_bounds, _pixel_size);
    mrt_camera->setFinalDrawCallback(reinterpret_cast<osg::Camera::DrawCallback *>(final_draw_callback));

    viewer.frame();

    bool status = final_draw_callback->status();

    mrt_camera->removeFinalDrawCallback(reinterpret_cast<osg::Camera::DrawCallback *>(final_draw_callback));

    // causes SEGV
    //delete final_draw_callback;

    viewer.setSceneData(nullptr);

    if(_disableTexture)
    {
        unsigned int mode = osg::StateAttribute::INHERIT|osg::StateAttribute::ON;
        for( unsigned int ii=0; ii < 4; ii++ )
        {
            stateSet->setTextureMode( ii, GL_TEXTURE_1D, mode );
            stateSet->setTextureMode( ii, GL_TEXTURE_2D, mode );
            stateSet->setTextureMode( ii, GL_TEXTURE_3D, mode );
            stateSet->setTextureMode( ii, GL_TEXTURE_RECTANGLE, mode );
            stateSet->setTextureMode( ii, GL_TEXTURE_CUBE_MAP, mode);
        }
    }

    // restore
    stateSet->setMode( GL_BLEND, blend);
    // ??????
    stateSet->setMode( GL_DEPTH_TEST, osg::StateAttribute::ON );

    if(alpha != 1)
    {

        if(isComposite)
        {
            osg::Material* material = material = new osg::Material;
            material->setAlpha( osg::Material::FRONT_AND_BACK, alpha);
            osg::ref_ptr<osg::BlendFunc> bf = new osg::BlendFunc(osg::BlendFunc::ONE_MINUS_SRC_ALPHA,osg::BlendFunc::SRC_ALPHA );
            stateSet->setAttributeAndModes(bf);
            stateSet->setAttributeAndModes ( material, osg::StateAttribute::ON );
        }
        else
        {
            material = new osg::Material;
            material->setAlpha( osg::Material::FRONT, alpha);

            // Turn on blending
            osg::ref_ptr<osg::BlendFunc> bf = new osg::BlendFunc(osg::BlendFunc::ONE_MINUS_SRC_ALPHA,osg::BlendFunc::SRC_ALPHA );
            stateSet->setAttributeAndModes(bf);
            stateSet->setAttributeAndModes( material, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
        }
    }

    return status;
}

Simulation::~Simulation() {
}






