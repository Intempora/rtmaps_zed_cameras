#pragma once

//ZED Includes
#include <sl/Camera.hpp>

// Includes maps sdk library header
//#include "opencv2/opencv.hpp"
//#define __IPL_H__
#include <maps.hpp>

#include <memory>
#include <atomic>
#include <thread>
#include <mutex>

#include "node.hpp"

using namespace std;
using namespace sl;

#define ComponentClass   MAPSZedCameras
#define ComponentName    "zed_cameras"
#define ComponentVersion "1.0.0"

enum class PointCloudFormat {
	XYZ, XYZ_RGB
};

enum class DataFormat {
	FLOAT_VECTOR, BGRA_IMAGE
};

#pragma pack(push,1)
struct MapsCalibrationParameters
{
    float fx; //Focal length in pixels along x axis.
    float fy; //Focal length in pixels along y axis.
    float cx; //Optical center along x axis, defined in pixels (usually close to width / 2).
    float cy; //Optical center along y axis, defined in pixels (usually close to height / 2)
    double disto[12]; //Distortion factor : [k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4]
    float v_fov; //Vertical field of view, in degrees
    float h_fov; //Horizontal field of view, in degrees
    float d_fov; //Diagonal field of view, in degrees
    size_t width; //Size in pixels of the images given by the camera.
    size_t height;
    float focal_length_metric; //Real focal length in millimeters.
};
#pragma pack(pop)

// Declares a new MAPSComponent child class
class ComponentClass : public MAPSComponent
{
	// Use standard header definition macro
	MAPS_COMPONENT_HEADER_CODE_WITHOUT_CONSTRUCTOR(ComponentClass)

	ComponentClass(const char* componentName, MAPSComponentDefinition& md);
    ~ComponentClass();

protected:
	// Override RTMaps functions
	void Dynamic() override;
	void Set(MAPSProperty& p, bool value);
	void Set(MAPSProperty& p, MAPSInt64 value);

	int getFPS();
	sl::RESOLUTION getResolution();
	sl::UNIT getUnit();
	sl::DEPTH_MODE getDepthMode();
	sl::COORDINATE_SYSTEM getCoordinate();
	PointCloudFormat getPointCloudFormat(); 
    sl::OBJECT_DETECTION_MODEL getDetectionModel();
    void ApplyVideoSettings();

    void GetImageLeft();
    void GetImageLeftThreaded();
    void GetImageRight();
    void GetImageRightThreaded();
    void GetDepth();
    void GetDepthThreaded();
    void GetConfidence();
    void GetConfidenceThreaded();
    void GetDisparity();
    void GetDisparityThreaded();
    void GetPointcloud();
    void GetPointcloudThreaded();
    void GetObjects();
    void GetObjectsThreaded();
    void GetCalibrationParameters();

private :
	bool m_first_time;
	bool m_initialized;
	bool m_isRunning;
	sl::Camera*	m_zed;
 
	sl::UNIT						m_unit; 
	PointCloudFormat				m_pointcloud_format; 
    DataFormat                     m_depth_data_format;
    DataFormat                     m_confidence_data_format;

	int			m_width;
	int			m_height;

	bool		m_output_left;
	bool		m_output_right;
	bool		m_output_depth;
	bool		m_output_disparity;
	bool		m_output_confidence;
	bool		m_output_pointcloud;
	bool		m_output_object;
	bool		m_output_calib_param;
    bool        m_use_multithread;

	bool		m_flip;
    double	    m_depth_max_dist;
    double		m_depth_min_dist;
	int			m_confidence_threshold;
    int         m_objects_vector_size;
    float       m_pointcloud_divide;

	long long		m_timestamp_offset;

    std::thread	m_threadImageLeft;
    MAPSEvent m_eventAcquireImageLeft;

    std::thread	m_threadImageRight;
    MAPSEvent m_eventAcquireImageRight;

    std::thread	m_threadDepth;
    MAPSEvent m_eventAcquireDepth;

    std::thread	m_threadPointcloud;
    MAPSEvent m_eventAcquirePointcloud;

    std::thread	m_threadConfidence;
    MAPSEvent m_eventAcquireConfidence;

    std::thread	m_threadDisparity;
    MAPSEvent m_eventAcquireDisparity;

    std::thread	m_threadObjects;
    MAPSEvent m_eventAcquireObjects;

    std::atomic<MAPSTimestamp> m_ts;

    sl::ObjectDetectionRuntimeParameters m_detection_parameters_rt;
    sl::Objects m_objects;
    sl::InitParameters m_init_params;

    fkyaml::node m_configFile;
    bool m_configFileLoaded;

    std::atomic<int> m_nbThreadRunning;
    std::mutex m_mutex;
    MAPSEvent m_checkRunningThreads;
};
