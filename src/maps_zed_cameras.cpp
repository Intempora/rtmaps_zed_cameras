#include "maps_zed_cameras.h"	// Includes the header of this component

//#define TEST_MODE

#include <sstream>
#include <chrono>

#define OUTPUT_CALIBRATION_PARAM    "calibration_parameters"
#define OUTPUT_MDL_LEFT_IPL			"left_image_ipl"
#define OUTPUT_MDL_RIGHT_IPL		"right_image_ipl"
#define OUTPUT_DEPTH_IPL		    "depth_ipl"
#define OUTPUT_CONFIDENCE_IPL		"confidence_ipl"
#define OUTPUT_DEPTH_FLOAT		    "depth_float"
#define OUTPUT_CONFIDENCE_FLOAT		"confidence_float"

#define OUTPUT_LEFT			            "left_image"
#define OUTPUT_RIGHT		            "right_image"
#define OUTPUT_DEPTH		            "depth"
#define OUTPUT_DISPARITY	            "disparity"
#define OUTPUT_CONFIDENCE	            "confidence"
#define OUTPUT_POINTCLOUD	            "pointcloud"
#define OUTPUT_OBJECT	                "object_detection"
#define OUTPUT_OBJECT_BOUNDING_BOX	    "object_detection_bounding_box"
#define OUTPUT_OBJECT_BOUNDING_BOX_3D	"object_detection_bounding_box_3d"
#define OUTPUT_OBJECT_LABEL	            "object_detection_labels"

// Use the macros to declare the inputs
MAPS_BEGIN_INPUTS_DEFINITION(ComponentClass)
MAPS_END_INPUTS_DEFINITION

// Use the macros to declare the outputs
MAPS_BEGIN_OUTPUTS_DEFINITION(ComponentClass)
{
    OUTPUT_CALIBRATION_PARAM, 0, { MAPS::Structure,new MAPSString((const char*)"MapsCalibrationParameters"),new MAPSString((const char*)NULL) }, 16 * sizeof(MapsCalibrationParameters), 16, 1, 1
},
	MAPS_OUTPUT(OUTPUT_MDL_LEFT_IPL, MAPS::IplImage, NULL, NULL, 0)
	MAPS_OUTPUT(OUTPUT_MDL_RIGHT_IPL, MAPS::IplImage, NULL, NULL, 0)
	MAPS_OUTPUT(OUTPUT_DEPTH_IPL, MAPS::IplImage, NULL, NULL, 0)
	MAPS_OUTPUT(OUTPUT_CONFIDENCE_IPL, MAPS::IplImage, NULL, NULL, 0)
	MAPS_OUTPUT(OUTPUT_DEPTH_FLOAT, MAPS::Float32, NULL, NULL, 0)
	MAPS_OUTPUT(OUTPUT_CONFIDENCE_FLOAT, MAPS::Float32, NULL, NULL, 0)
	MAPS_OUTPUT(OUTPUT_DISPARITY, MAPS::Float32, NULL, NULL, 0)
	MAPS_OUTPUT(OUTPUT_POINTCLOUD, MAPS::Float32, NULL, NULL, 0)
	MAPS_OUTPUT(OUTPUT_OBJECT, MAPS::RealObject, NULL, NULL, 0)
    MAPS_OUTPUT(OUTPUT_OBJECT_BOUNDING_BOX,MAPS::DrawingObject, nullptr, nullptr, 0)
    MAPS_OUTPUT(OUTPUT_OBJECT_BOUNDING_BOX_3D,MAPS::RealObject, nullptr, nullptr, 0)
	MAPS_OUTPUT(OUTPUT_OBJECT_LABEL, MAPS::DrawingObject, nullptr, nullptr, 0)
MAPS_END_OUTPUTS_DEFINITION

#define PROPS_MULTITHREAD			"use_multithread"
#define PROPS_CONFIG_FILE			"configuration_file_path"
#define PROPS_RESOLUTION			"resolution"
#define PROPS_FPS					"fps"
#define PROPS_UNIT_MODE				"unit"
#define PROPS_DEPTH_MODE		    "depth_mode"
#define PROPS_COORDINATE			"coordinate_system"

#define PROPS_OUTPUT_LEFT			"output_left"
#define PROPS_OUTPUT_RIGHT			"output_right"
#define PROPS_OUTPUT_DEPTH			"output_depth"
#define PROPS_OUTPUT_DISPARITY		"output_disparity"
#define PROPS_OUTPUT_CONFIDENCE	    "output_confidence"
#define PROPS_OUTPUT_POINTCLOUD		"output_pointcloud"
#define PROPS_OUTPUT_OBJECT		    "output_object_detection"
#define PROPS_OUTPUT_CALIB_PARAM    "output_calibration_parameters"

#define PROPS_DEPTH_FORMAT		    "depth_data_format"
#define PROPS_DEPTH_MIN_DIST		"depth_min_distance"
#define PROPS_DEPTH_MAX_DIST		"depth_max_distance"
#define PROPS_FLIP					"flip_image"
#define PROPS_CONFIDENCE_THRESHOLD	"confidence_threshold"
#define PROPS_CONFIDENCE_FORMAT		"confidence_data_format"

#define PROPS_POINTCLOUD_FORMAT		"pointcloud_format"
#define PROPS_OBJECTS_VECTOR_SIZE   "object_vector_size"
#define PROPS_OBJECTS_DETECTION_MODEL   "object_detection_model"

// Use the macros to declare the properties
MAPS_BEGIN_PROPERTIES_DEFINITION(ComponentClass)
	MAPS_PROPERTY(PROPS_MULTITHREAD, false, false, false)
	MAPS_PROPERTY_SUBTYPE(PROPS_CONFIG_FILE, "", false, false, MAPS::PropertySubTypeFile | MAPS::PropertySubTypeMustExist)
	MAPS_PROPERTY_ENUM(PROPS_RESOLUTION, "960 x 600|1920 x 1080|1920 x 1200", 2, false, false)
	MAPS_PROPERTY_ENUM(PROPS_FPS, "15|30|60|120", 1, false, false)
	MAPS_PROPERTY_ENUM(PROPS_UNIT_MODE, "MILLIMETER|CENTIMETER|METER", 0, false, false)
	MAPS_PROPERTY_ENUM(PROPS_COORDINATE, "IMAGE|LEFT_HANDED_Y_UP|RIGHT_HANDED_Y_UP|RIGHT_HANDED_Z_UP|LEFT_HANDED_Z_UP|RIGHT_HANDED_Z_UP_X_FWD", 0, false, false)
	MAPS_PROPERTY(PROPS_FLIP, false, false, false)

	MAPS_PROPERTY(PROPS_OUTPUT_LEFT, true, false, false)
	MAPS_PROPERTY(PROPS_OUTPUT_RIGHT, true, false, false)
	MAPS_PROPERTY(PROPS_OUTPUT_DEPTH, true, false, false)
	MAPS_PROPERTY(PROPS_OUTPUT_DISPARITY, true, false, false)
	MAPS_PROPERTY(PROPS_OUTPUT_CONFIDENCE, true, false, false)
	MAPS_PROPERTY(PROPS_OUTPUT_POINTCLOUD, true, false, false)
	MAPS_PROPERTY(PROPS_OUTPUT_OBJECT, true, false, false)
	MAPS_PROPERTY(PROPS_OUTPUT_CALIB_PARAM, true, false, false)

	MAPS_PROPERTY_ENUM(PROPS_DEPTH_FORMAT, "FLOAT_VECTOR|BGRA_IMAGE", 0, false, false)
	MAPS_PROPERTY_ENUM(PROPS_DEPTH_MODE, "NONE|PERFORMANCE|QUALITY|ULTRA|NEURAL|NEURAL_PLUS", 1, false, false)
	MAPS_PROPERTY(PROPS_DEPTH_MIN_DIST, -1.0, false, false)
	MAPS_PROPERTY(PROPS_DEPTH_MAX_DIST, -1.0, false, false)

	MAPS_PROPERTY_ENUM(PROPS_CONFIDENCE_FORMAT, "FLOAT_VECTOR|BGRA_IMAGE", 0, false, false)
	MAPS_PROPERTY(PROPS_CONFIDENCE_THRESHOLD, 95, false, true)

	MAPS_PROPERTY_ENUM(PROPS_POINTCLOUD_FORMAT, "XYZ|XYZ_RGB", 0, false, false)

	MAPS_PROPERTY_ENUM(PROPS_OBJECTS_DETECTION_MODEL, "MULTI_CLASS_BOX_FAST|MULTI_CLASS_BOX_MEDIUM|MULTI_CLASS_BOX_ACCURATE", 0, false, false)
	MAPS_PROPERTY(PROPS_OBJECTS_VECTOR_SIZE, 10, false, true)
MAPS_END_PROPERTIES_DEFINITION

// Use the macros to declare the actions
MAPS_BEGIN_ACTIONS_DEFINITION(ComponentClass)
    //MAPS_ACTION("aName",ComponentClass::ActionName)
MAPS_END_ACTIONS_DEFINITION

// Use the macros to declare this component (ZedCamera) behaviour
MAPS_COMPONENT_DEFINITION(ComponentClass,ComponentName,ComponentVersion,128,
			  MAPS::Threaded,MAPS::Threaded,
			  0, // Nb of inputs. Leave -1 to use the number of declared input definitions
			  0, // Nb of outputs. Leave -1 to use the number of declared output definitions
			  15, // Nb of properties. Leave -1 to use the number of declared property definitions
			  -1) // Nb of actions. Leave -1 to use the number of declared action definitions


ComponentClass::ComponentClass(const char* componentName, MAPSComponentDefinition& md) :
	MAPSComponent(componentName, md)
{
    m_confidence_threshold = 95;
} 

ComponentClass::~ComponentClass()
{

}

int ComponentClass::getFPS() 
{
	int fps_enum_idx = GetEnumProperty(PROPS_FPS).GetSelected();

	switch (fps_enum_idx) 
    {
	case 0:
		return 15;
	case 1:
		return 30;
	case 2:
		return 60; 
	case 3:
		return 120; 
	}

	Error("invalid fps");
	throw std::runtime_error("invalid fps");
}

sl::RESOLUTION ComponentClass::getResolution() 
{
	int resolution_enum_idx = GetEnumProperty(PROPS_RESOLUTION).GetSelected();

	//  960 x 600 - 1920 x 1080 - 1920 x 1200 
	switch (resolution_enum_idx) 
    {
	case 0:
		return sl::RESOLUTION::SVGA;
	case 1:
		return sl::RESOLUTION::HD1080;
	case 2:
		return sl::RESOLUTION::HD1200;
	}

	Error("invalid resolution");
	throw std::runtime_error("invalid resolution");
}

sl::UNIT ComponentClass::getUnit() 
{
	int unit_enum_idx = GetEnumProperty(PROPS_UNIT_MODE).GetSelected();

	switch (unit_enum_idx) 
    {
	case 0:
        m_pointcloud_divide = 1000.0;
		return sl::UNIT::MILLIMETER;
    case 1:
        m_pointcloud_divide = 100.0;
		return sl::UNIT::CENTIMETER;
	case 2:
        m_pointcloud_divide = 1.0;
		return sl::UNIT::METER;
	}

	Error("invalid unit mode");
	throw std::runtime_error("invalid unit mode");
}

sl::COORDINATE_SYSTEM ComponentClass::getCoordinate() 
{
	int unit_enum_idx = GetEnumProperty(PROPS_COORDINATE).GetSelected();

	switch (unit_enum_idx) 
    {
	case 0:
		return sl::COORDINATE_SYSTEM::IMAGE;
	case 1:
		return sl::COORDINATE_SYSTEM::LEFT_HANDED_Y_UP;
	case 2:
		return sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
	case 3:
		return sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP;
    case 4:
		return sl::COORDINATE_SYSTEM::LEFT_HANDED_Z_UP;
    case 5:
		return sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
	}

	Error("invalid coordinate system");
	throw std::runtime_error("invalid coordinate system");
}

sl::DEPTH_MODE ComponentClass::getDepthMode() 
{
	int enum_idx = GetEnumProperty(PROPS_DEPTH_MODE).GetSelected();

	// "none|performance|quality|ultra"
	switch (enum_idx) 
    {
	case 0:
		return sl::DEPTH_MODE::NONE;
	case 1:
		return sl::DEPTH_MODE::PERFORMANCE;
	case 2:
		return sl::DEPTH_MODE::QUALITY;
	case 3:
		return sl::DEPTH_MODE::ULTRA;
    case 4:
		return sl::DEPTH_MODE::NEURAL;
    case 5:
		return sl::DEPTH_MODE::NEURAL_PLUS;
	}

	Error("invalid camera mode");
	throw std::runtime_error("invalid camera mode");
}

sl::OBJECT_DETECTION_MODEL ComponentClass::getDetectionModel() 
{
	int enum_idx = GetEnumProperty(PROPS_OBJECTS_DETECTION_MODEL).GetSelected();

	// "none|performance|quality|ultra"
	switch (enum_idx) 
    {
	case 0:
		return sl::OBJECT_DETECTION_MODEL::MULTI_CLASS_BOX_FAST;
	case 1:
		return sl::OBJECT_DETECTION_MODEL::MULTI_CLASS_BOX_MEDIUM;
	case 2:
		return sl::OBJECT_DETECTION_MODEL::MULTI_CLASS_BOX_ACCURATE;
	}

	Error("invalid camera mode");
	throw std::runtime_error("invalid camera mode");
}

PointCloudFormat ComponentClass::getPointCloudFormat() 
{
	int enum_idx = GetEnumProperty(PROPS_POINTCLOUD_FORMAT).GetSelected();

	// "XYZ|XYZ_RGB"
	switch (enum_idx) 
    {
	case 0:
		return PointCloudFormat::XYZ;
	case 1:
		return PointCloudFormat::XYZ_RGB;
	}

	Error("invalid pointcloud mode");
	throw std::runtime_error("invalid pointcloud mode");
}

void ComponentClass::Set(MAPSProperty& p, bool value) 
{
	/* update image flipping */
	if (&p == &Property(PROPS_FLIP)) 
    {
		m_flip = value;
	}

	MAPSComponent::Set(p, value);
}

void ComponentClass::Set(MAPSProperty& p, MAPSInt64 value) 
{
	/* update depth clamping value */
    if (&p == &Property(PROPS_CONFIDENCE_THRESHOLD)) 
    {
		if (value < 0) 
        {
			ReportInfo("Incorrect value for this property. It has to be between 0 and 100. Value is set to 0.");
			value = 0;
		}

		if (value > 100) 
        {
			ReportInfo("Incorrect value for this property. It has to be between 0 and 100. Value is set to 100.");
			value = 100;
		}

		m_confidence_threshold = (int)value; 
	}

	MAPSComponent::Set(p, value);
}

////////////////////////////////////////////////////
void ComponentClass::ApplyVideoSettings()
{  
    if(!m_configFileLoaded)
        return;

    ReportInfo("Applying video settings....");
    sl::VIDEO_SETTINGS setting;
    sl::ERROR_CODE err;
    std::string logStr;
    int camVal;
    int settingVal;
    int settingValMin;
    int settingValMax;

    if(!m_configFile.contains("video"))
    {
        ReportError("Can't apply video settings because config file does not contain 'video' category.");
        return;
    }

    if(m_configFile["video"].contains("exposure_time"))
    {
        setting = sl::VIDEO_SETTINGS::EXPOSURE;
        settingVal = m_configFile["video"]["exposure_time"].get_value<int>();
#ifndef TEST_MODE
        if(m_zed->setCameraSettings(setting, settingVal) == sl::ERROR_CODE::SUCCESS)
        {
            logStr = "Exposure setted to " + std::to_string(settingVal);
            ReportInfo(logStr.c_str());
        }
#endif
    }
    else
    {
        ReportError("Can't apply exposure time because config file does not contain 'exposure_time' value.");
    }

    if(m_configFile["video"].contains("auto_exposure_time_range_min") && m_configFile["video"].contains("auto_exposure_time_range_max"))
    {
        setting = sl::VIDEO_SETTINGS::AUTO_EXPOSURE_TIME_RANGE;
        settingValMin = m_configFile["video"]["auto_exposure_time_range_min"].get_value<int>();
        settingValMax = m_configFile["video"]["auto_exposure_time_range_max"].get_value<int>();
#ifndef TEST_MODE
        if(m_zed->setCameraSettings(setting, settingValMin, settingValMax) == sl::ERROR_CODE::SUCCESS)
        {
            logStr = "Auto exposure time range setted to " + std::to_string(settingValMin) + " - " + std::to_string(settingValMax);
            ReportInfo(logStr.c_str());
        }
#endif
    }
    else
    {
        ReportError("Can't apply exposure time range because config file does not contain 'auto_exposure_time_range_min' or 'auto_exposure_time_range_max' value.");
    }

    if(m_configFile["video"].contains("exposure_compensation"))
    {
        setting = sl::VIDEO_SETTINGS::EXPOSURE_COMPENSATION;
        settingVal = m_configFile["video"]["exposure_compensation"].get_value<int>();
#ifndef TEST_MODE
        if(m_zed->setCameraSettings(setting, settingVal) == sl::ERROR_CODE::SUCCESS)
        {
            logStr = "Exposure compensation setted to " + std::to_string(settingVal);
            ReportInfo(logStr.c_str());
        }
#endif
    }
    else
    {
        ReportError("Can't apply exposure compensation because config file does not contain 'exposure_compensation' value.");
    }

    if(m_configFile["video"].contains("analog_gain"))
    {
        setting = sl::VIDEO_SETTINGS::ANALOG_GAIN;
        settingVal = m_configFile["video"]["analog_gain"].get_value<int>();
#ifndef TEST_MODE
        if(m_zed->setCameraSettings(setting, settingVal) == sl::ERROR_CODE::SUCCESS)
        {
            logStr = "Analog gain setted to " + std::to_string(settingVal);
            ReportInfo(logStr.c_str());
        }
#endif
    }
    else
    {
        ReportError("Can't apply analog gain because config file does not contain 'analog_gain' value.");
    }

    if(m_configFile["video"].contains("auto_analog_gain_range_min") && m_configFile["video"].contains("auto_analog_gain_range_max"))
    {
        setting = sl::VIDEO_SETTINGS::AUTO_ANALOG_GAIN_RANGE;
        settingValMin = m_configFile["video"]["auto_analog_gain_range_min"].get_value<int>();
        settingValMax = m_configFile["video"]["auto_analog_gain_range_max"].get_value<int>();
#ifndef TEST_MODE
        if(m_zed->setCameraSettings(setting, settingValMin, settingValMax) == sl::ERROR_CODE::SUCCESS)
        {
            logStr = "Auto analog gain range setted to " + std::to_string(settingValMin) + " - " + std::to_string(settingValMax);
            ReportInfo(logStr.c_str());
        }
#endif
    }
    else
    {
        ReportError("Can't apply analog gain range because config file does not contain 'auto_analog_gain_range_min' or 'auto_analog_gain_range_max' value.");
    }

    if(m_configFile["video"].contains("digital_gain"))
    {
        setting = sl::VIDEO_SETTINGS::DIGITAL_GAIN;
        settingVal = m_configFile["video"]["digital_gain"].get_value<int>();
#ifndef TEST_MODE
        if(m_zed->setCameraSettings(setting, settingVal) == sl::ERROR_CODE::SUCCESS)
        {
            logStr = "Digital gain setted to " + std::to_string(settingVal);
            ReportInfo(logStr.c_str());
        }
#endif
    }
    else
    {
        ReportError("Can't apply digital gain because config file does not contain 'digital_gain' value.");
    }

    if(m_configFile["video"].contains("auto_digital_gain_range_min") && m_configFile["video"].contains("auto_digital_gain_range_max"))
    {
        setting = sl::VIDEO_SETTINGS::AUTO_DIGITAL_GAIN_RANGE;
        settingValMin = m_configFile["video"]["auto_digital_gain_range_min"].get_value<int>();
        settingValMax = m_configFile["video"]["auto_digital_gain_range_max"].get_value<int>();
#ifndef TEST_MODE
        if(m_zed->setCameraSettings(setting, settingValMin, settingValMax) == sl::ERROR_CODE::SUCCESS)
        {
            logStr = "Auto digital gain range setted to " + std::to_string(settingValMin) + " - " + std::to_string(settingValMax);
            ReportInfo(logStr.c_str());
        }
#endif
    }
    else
    {
        ReportError("Can't apply digital gain range because config file does not contain 'auto_digital_gain_range_min' or 'auto_digital_gain_range_max' value.");
    }

    if(m_configFile["video"].contains("denoising"))
    {
        setting = sl::VIDEO_SETTINGS::DENOISING;
        settingVal = m_configFile["video"]["denoising"].get_value<int>();
#ifndef TEST_MODE
        if(m_zed->setCameraSettings(setting, settingVal) == sl::ERROR_CODE::SUCCESS)
        {
            logStr = "Denoising setted to " + std::to_string(settingVal);
            ReportInfo(logStr.c_str());
        }
#endif
    }
    else
    {
        ReportError("Can't apply denoising because config file does not contain 'denoising' value.");
    }
}

////////////////////////////////////////////////////
void ComponentClass::Dynamic() 
{
    std::string configFilePath = GetStringProperty(PROPS_CONFIG_FILE).Beginning();
    m_configFileLoaded = false;

    if(!configFilePath.empty())
    {
        if(configFilePath.find(".yaml") == std::string::npos)
        {
            ReportError("Config file must be a .yaml file.");
        }
        else
        {
            std::ifstream configFile(configFilePath);

            if (!configFile.is_open())
            {
                ReportError("Can't open config file.");
            }
            else
            {
                m_configFile = fkyaml::node::deserialize(configFile);
                m_configFileLoaded = true;
                
                if(m_configFile.contains("general"))
                {
                    if(m_configFile["general"].contains("grab_resolution"))
                        DirectSetProperty(PROPS_RESOLUTION, m_configFile["general"]["grab_resolution"].get_value<int>());

                    if(m_configFile["general"].contains("grab_frame_rate"))
                        DirectSetProperty(PROPS_FPS, m_configFile["general"]["grab_frame_rate"].get_value<int>());

                    if(m_configFile["general"].contains("unit_mode"))
                        DirectSetProperty(PROPS_UNIT_MODE, m_configFile["general"]["unit_mode"].get_value<int>());

                    if(m_configFile["general"].contains("coordinate_system"))
                        DirectSetProperty(PROPS_COORDINATE, m_configFile["general"]["coordinate_system"].get_value<int>());
                }
                else
                {
                    ReportWarning("No 'general' section found in config file.");
                }
            }
        }
    }

    m_use_multithread = GetBoolProperty(PROPS_MULTITHREAD);
	m_output_left = GetBoolProperty(PROPS_OUTPUT_LEFT);
	m_output_right = GetBoolProperty(PROPS_OUTPUT_RIGHT);
	m_output_depth = GetBoolProperty(PROPS_OUTPUT_DEPTH);
	m_output_disparity = GetBoolProperty(PROPS_OUTPUT_DISPARITY);
	m_output_confidence = GetBoolProperty(PROPS_OUTPUT_CONFIDENCE);
	m_output_pointcloud = GetBoolProperty(PROPS_OUTPUT_POINTCLOUD);
	m_output_object = GetBoolProperty(PROPS_OUTPUT_OBJECT);
	m_output_calib_param = GetBoolProperty(PROPS_OUTPUT_CALIB_PARAM);

    if (m_output_calib_param)
    {
         NewOutput(OUTPUT_CALIBRATION_PARAM);
    }

    if (m_output_left)
    {
         NewOutput(OUTPUT_MDL_LEFT_IPL, OUTPUT_LEFT);
    }

    if (m_output_right)
    {
        NewOutput(OUTPUT_MDL_RIGHT_IPL, OUTPUT_RIGHT);
    }

	if (m_output_depth) 
    {
        m_depth_data_format = static_cast<DataFormat>(NewProperty(PROPS_DEPTH_FORMAT).IntegerValue());

        if(m_configFileLoaded)
        {
            if(m_configFile.contains("depth"))
            {
                if(m_configFile["depth"].contains("format"))
                {
                    DirectSetProperty(PROPS_DEPTH_FORMAT, m_configFile["depth"]["format"].get_value<int>());
                    m_depth_data_format = static_cast<DataFormat>(m_configFile["depth"]["format"].get_value<int>());
                }
            }
            else
            {
                ReportWarning("No 'depth' section found in config file.");
            }
        }

        if(m_depth_data_format == DataFormat::FLOAT_VECTOR)
            NewOutput(OUTPUT_DEPTH_FLOAT, OUTPUT_DEPTH);
        else
            NewOutput(OUTPUT_DEPTH_IPL, OUTPUT_DEPTH);
    }

	if (m_output_disparity) 
    {
        NewOutput(OUTPUT_DISPARITY);
    }

	if (m_output_confidence)
    {
        m_confidence_data_format = static_cast<DataFormat>(NewProperty(PROPS_CONFIDENCE_FORMAT).IntegerValue());

        if(m_configFileLoaded)
        {
            if(m_configFile.contains("confidence"))
            {
                if(m_configFile["confidence"].contains("format"))
                {
                    DirectSetProperty(PROPS_CONFIDENCE_FORMAT, m_configFile["confidence"]["format"].get_value<int>());
                    m_confidence_data_format = static_cast<DataFormat>(m_configFile["confidence"]["format"].get_value<int>());
                }
            }
            else
            {
                ReportWarning("No 'confidence' section found in config file.");
            }
        }

        if(m_confidence_data_format == DataFormat::FLOAT_VECTOR)
            NewOutput(OUTPUT_CONFIDENCE_FLOAT, OUTPUT_CONFIDENCE);
        else
            NewOutput(OUTPUT_CONFIDENCE_IPL, OUTPUT_CONFIDENCE);
    }

	if (m_output_pointcloud) 
    {
		NewOutput(OUTPUT_POINTCLOUD);
		NewProperty(PROPS_POINTCLOUD_FORMAT);

        if(m_configFileLoaded)
        {
            if(m_configFile.contains("pointcloud"))
            {
                if(m_configFile["pointcloud"].contains("format"))
                {
                    DirectSetProperty(PROPS_POINTCLOUD_FORMAT, m_configFile["pointcloud"]["format"].get_value<int>());
                }
            }
            else
            {
                ReportWarning("No 'pointcloud' section found in config file.");
            }
        }
	}

    if(m_output_object)
    {
        NewOutput(OUTPUT_OBJECT);
        NewOutput(OUTPUT_OBJECT_BOUNDING_BOX);
        NewOutput(OUTPUT_OBJECT_BOUNDING_BOX_3D);
        NewOutput(OUTPUT_OBJECT_LABEL);
		m_objects_vector_size = NewProperty(PROPS_OBJECTS_VECTOR_SIZE).IntegerValue();
		NewProperty(PROPS_OBJECTS_DETECTION_MODEL).IntegerValue();

        if(m_configFileLoaded)
        {
            if(m_configFile.contains("object_detection"))
            {
                if(m_configFile["object_detection"].contains("model"))
                {
                    DirectSetProperty(PROPS_OBJECTS_DETECTION_MODEL, m_configFile["object_detection"]["model"].get_value<int>());
                }
            }
            else
            {
                ReportWarning("No 'object_detection' section found in config file.");
            }
        }
    }

    const bool calc_depth = m_output_depth || m_output_pointcloud || m_output_confidence || m_output_object;

    if(calc_depth)
    {
        NewProperty(PROPS_DEPTH_MODE);
        m_depth_min_dist = NewProperty(PROPS_DEPTH_MIN_DIST).IntegerValue();
        m_depth_max_dist = NewProperty(PROPS_DEPTH_MAX_DIST).IntegerValue();
        m_confidence_threshold = NewProperty(PROPS_CONFIDENCE_THRESHOLD).IntegerValue();

        if(m_configFileLoaded)
        {
            if(m_configFile.contains("depth"))
            {
                if(m_configFile["depth"].contains("mode"))
                {
                    DirectSetProperty(PROPS_DEPTH_MODE, m_configFile["depth"]["mode"].get_value<int>());
                }

                if(m_configFile["depth"].contains("min_depth"))
                {
                    DirectSetProperty(PROPS_DEPTH_MIN_DIST, m_configFile["depth"]["min_depth"].get_value<double>());
                    m_depth_min_dist = m_configFile["depth"]["min_depth"].get_value<double>();
                }

                if(m_configFile["depth"].contains("max_depth"))
                {
                    DirectSetProperty(PROPS_DEPTH_MAX_DIST, m_configFile["depth"]["max_depth"].get_value<double>());
                    m_depth_max_dist = m_configFile["depth"]["max_depth"].get_value<double>();
                }
            }
            else
            {
                ReportWarning("No 'depth' section found in config file.");
            }

            if(m_configFile.contains("confidence"))
            {
                if(m_configFile["confidence"].contains("threshold"))
                {
                    DirectSetProperty(PROPS_CONFIDENCE_THRESHOLD, m_configFile["confidence"]["threshold"].get_value<int>());
                    m_confidence_threshold = m_configFile["confidence"]["threshold"].get_value<int>();
                }
            }
            else
            {
                ReportWarning("No 'confidence' section found in config file.");
            }
        }

        if(m_depth_max_dist < -1)
            m_depth_max_dist = -1;

        if(m_depth_min_dist < -1)
            m_depth_min_dist = -1;

        m_init_params.depth_mode = getDepthMode();
        m_init_params.depth_maximum_distance = m_depth_max_dist;
	    m_init_params.depth_minimum_distance = m_depth_min_dist;
    }

	sl::RESOLUTION res = getResolution();
	int fps_value = getFPS();
 
	switch (res) 
    {
	case sl::RESOLUTION::SVGA:
		if (fps_value != 15 && fps_value != 30 && fps_value != 60 && fps_value != 120) 
        {
			ReportWarning("Only 15,30,60 and 100 fps are valid for SVGA");
		}
		break;

	case sl::RESOLUTION::HD1080:
		if (fps_value != 15 && fps_value != 30 && fps_value != 60 ) 
        {
			ReportWarning("Only 15,30 and 60 fps are valid for HD1080");
		}
		break;

	case sl::RESOLUTION::HD1200:
		if (fps_value != 15 && fps_value != 30 && fps_value != 60 ) 
        {
			ReportWarning("Only 15,30 and 60 fps are valid for HD1200");
		}
		break; 
	} 
}

////////////////////////////////////////////////////
void ComponentClass::Birth()
{  
	m_first_time = true;

    m_flip = GetBoolProperty(PROPS_FLIP);

	m_init_params.coordinate_units = getUnit();
	m_init_params.sdk_verbose = false;
	m_init_params.coordinate_system = getCoordinate();
	m_init_params.camera_fps = getFPS();
	m_init_params.camera_resolution = getResolution();
	m_init_params.camera_image_flip = m_flip;

#ifndef TEST_MODE
    m_zed = new sl::Camera();
	sl::ERROR_CODE err = m_zed->open(m_init_params);
	if (err != sl::ERROR_CODE::SUCCESS) 
    {
        m_initialized = false;
		std::ostringstream oss;
		oss << "Can't initialize ZED camera: " << sl::toString(err);
		Error(oss.str().c_str());
	}
#endif
    m_initialized = true;

#ifndef TEST_MODE
	// timestamp handling
	m_timestamp_offset = long(m_zed->getTimestamp(sl::TIME_REFERENCE::IMAGE))/1000 - MAPS::CurrentTime();
	// Print camera information
	MAPSStreamedString msg;

	auto camera_info = m_zed->getCameraInformation();
	msg << "ZED Model                 : " << (int)camera_info.camera_model << "\n";
	msg << "ZED Serial Number         : " << camera_info.serial_number << "\n";
	msg << "ZED Camera Firmware       : " << camera_info.camera_configuration.firmware_version << "/" << camera_info.sensors_configuration.firmware_version << "\n";
	msg << "ZED Camera Resolution     : " << camera_info.camera_configuration.resolution.width << "x" << camera_info.camera_configuration.resolution.height << "\n";
	msg << "ZED Camera FPS            : " << m_zed->getInitParameters().camera_fps << "\n";
	msg << "ZED Depth Mode            : " << static_cast<int>(m_zed->getInitParameters().depth_mode) << "\n";
	
	ReportInfo(msg); 

	// get image size 
	m_width = camera_info.camera_configuration.resolution.width;
	m_height = camera_info.camera_configuration.resolution.height;
#else
    m_width = 1920;
    m_height = 1080;
#endif

	if (m_output_pointcloud) 
    {
		m_pointcloud_format = getPointCloudFormat();
	}

    if(m_output_object)
    {

        ObjectDetectionParameters detection_parameters;

        detection_parameters.detection_model = getDetectionModel();

        if(m_configFileLoaded)
        {
            if(m_configFile.contains("object_detection"))
            {
                if(m_configFile["object_detection"].contains("enable_tracking"))
                    detection_parameters.enable_tracking = m_configFile["object_detection"]["enable_tracking"].get_value<bool>();

                if(m_configFile["object_detection"].contains("enable_segmentation"))
                    detection_parameters.enable_segmentation = m_configFile["object_detection"]["enable_segmentation"].get_value<bool>();

                if(m_configFile["object_detection"].contains("allow_reduced_precision_inference"))
                    detection_parameters.allow_reduced_precision_inference = m_configFile["object_detection"]["allow_reduced_precision_inference"].get_value<bool>();

                if(m_configFile["object_detection"].contains("max_range"))
                    detection_parameters.max_range = m_configFile["object_detection"]["max_range"].get_value<float>();

                if(m_configFile["object_detection"].contains("prediction_timeout"))
                    detection_parameters.prediction_timeout_s = m_configFile["object_detection"]["prediction_timeout"].get_value<float>();

                if(m_configFile["object_detection"].contains("filtering_mode"))
                    detection_parameters.filtering_mode = (sl::OBJECT_FILTERING_MODE)m_configFile["object_detection"]["filtering_mode"].get_value<int>();

                if(m_configFile["object_detection"].contains("confidence_threshold"))
                    m_detection_parameters_rt.detection_confidence_threshold = m_configFile["object_detection"]["confidence_threshold"].get_value<float>();

                std::vector<sl::OBJECT_CLASS> mObjDetFilter;
                if (m_configFile["object_detection"]["mc_people"].get_value<bool>()) 
                {
                    mObjDetFilter.push_back(sl::OBJECT_CLASS::PERSON);
                }
                if (m_configFile["object_detection"]["mc_vehicle"].get_value<bool>()) 
                {
                    mObjDetFilter.push_back(sl::OBJECT_CLASS::VEHICLE);
                }
                if (m_configFile["object_detection"]["mc_bag"].get_value<bool>()) 
                {
                    mObjDetFilter.push_back(sl::OBJECT_CLASS::BAG);
                }
                if (m_configFile["object_detection"]["mc_animal"].get_value<bool>()) 
                {
                    mObjDetFilter.push_back(sl::OBJECT_CLASS::ANIMAL);
                }
                if (m_configFile["object_detection"]["mc_electronics"].get_value<bool>()) 
                {
                    mObjDetFilter.push_back(sl::OBJECT_CLASS::ELECTRONICS);
                }
                if (m_configFile["object_detection"]["mc_fruit_vegetable"].get_value<bool>()) 
                {
                    mObjDetFilter.push_back(sl::OBJECT_CLASS::FRUIT_VEGETABLE);
                }
                if (m_configFile["object_detection"]["mc_sport"].get_value<bool>()) 
                {
                    mObjDetFilter.push_back(sl::OBJECT_CLASS::SPORT);
                }
                m_detection_parameters_rt.object_class_filter = mObjDetFilter;
            }
            else
            {
                ReportWarning("No 'object_detection' section found in config file.");
            }
        }

        // If you want to have object tracking you need to enable positional tracking first
#ifndef TEST_MODE
        if (detection_parameters.enable_tracking)
            m_zed->enablePositionalTracking();    

        if (m_zed->enableObjectDetection(detection_parameters) != ERROR_CODE::SUCCESS) 
        {
            Error("Can't enable object detection.");
        }
#endif
    }

    if(m_use_multithread)
    {
        ////////start threads///////////
        m_isRunning = true;
        if(m_output_left)
        {
            m_eventAcquireImageLeft.Reset();
            m_threadImageLeft = std::thread(&ComponentClass::GetImageLeftThreaded, this);
        }

        if(m_output_right)
        {
            m_eventAcquireImageRight.Reset();
            m_threadImageRight = std::thread(&ComponentClass::GetImageRightThreaded, this);
        }

        if(m_output_depth)
        {
            m_eventAcquireDepth.Reset();
            m_threadDepth = std::thread(&ComponentClass::GetDepthThreaded, this);
        }

        if(m_output_pointcloud)
        {
            m_eventAcquirePointcloud.Reset();
            m_threadPointcloud = std::thread(&ComponentClass::GetPointcloudThreaded, this);
        }

        if(m_output_confidence)
        {
            m_eventAcquireConfidence.Reset();
            m_threadConfidence = std::thread(&ComponentClass::GetConfidenceThreaded, this);
        }

        if(m_output_disparity)
        {
            m_eventAcquireDisparity.Reset();
            m_threadDisparity = std::thread(&ComponentClass::GetDisparityThreaded, this);
        }

        if(m_output_object)
        {
            m_eventAcquireObjects.Reset();
            m_threadObjects = std::thread(&ComponentClass::GetObjectsThreaded, this);
        }
    }


    ////////alloc buffers///////////
    IplImage image_model_bgra = MAPS::IplImageModel(m_width, m_height, MAPS_CHANNELSEQ_BGRA);

    if (m_output_left) 
        Output(OUTPUT_LEFT).AllocOutputBufferIplImage(image_model_bgra);
    if (m_output_right) 
        Output(OUTPUT_RIGHT).AllocOutputBufferIplImage(image_model_bgra);


    if (m_output_depth)
    { 
        if(m_depth_data_format == DataFormat::FLOAT_VECTOR)
            Output(OUTPUT_DEPTH).AllocOutputBuffer(m_width * m_height);
        else
            Output(OUTPUT_DEPTH).AllocOutputBufferIplImage(image_model_bgra);
    }

    if (m_output_disparity) 
        Output(OUTPUT_DISPARITY).AllocOutputBuffer(m_width * m_height);

    if (m_output_confidence) 
    { 
        if(m_confidence_data_format == DataFormat::FLOAT_VECTOR)
            Output(OUTPUT_CONFIDENCE).AllocOutputBuffer(m_width * m_height);
        else
            Output(OUTPUT_CONFIDENCE).AllocOutputBufferIplImage(image_model_bgra);
    }

    if (m_output_pointcloud) 
    {
        switch (m_pointcloud_format) 
        {
        case PointCloudFormat::XYZ:
            Output(OUTPUT_POINTCLOUD).AllocOutputBuffer(m_width * m_height * 3);//3 pour XYZ
            break;
        case PointCloudFormat::XYZ_RGB:
            Output(OUTPUT_POINTCLOUD).AllocOutputBuffer(m_width * m_height * 3 * 3);//3 pour XYZ et 3 pour RGB
            break;
        default:
            Error("unexpected pointcloud format");
        }
    }

    if(m_output_object)
    {
        Output(OUTPUT_OBJECT).AllocOutputBuffer(m_objects_vector_size);
        Output(OUTPUT_OBJECT_BOUNDING_BOX).AllocOutputBuffer(m_objects_vector_size);
        Output(OUTPUT_OBJECT_BOUNDING_BOX_3D).AllocOutputBuffer(m_objects_vector_size);
        Output(OUTPUT_OBJECT_LABEL).AllocOutputBuffer(m_objects_vector_size);
    }

    ApplyVideoSettings();
}

void ComponentClass::Core() 
{ 
	const bool calc_depth = m_output_depth || m_output_pointcloud || m_output_confidence || m_output_object;

	sl::RuntimeParameters params;
	
	params.enable_depth = calc_depth;
	params.confidence_threshold = m_confidence_threshold;
    m_nbThreadRunning = 0;

#ifndef TEST_MODE
	if (m_zed->grab(params) == ERROR_CODE::SUCCESS)
	{
		m_ts = long(m_zed->getTimestamp(sl::TIME_REFERENCE::IMAGE))/1000 - MAPS::CurrentTime();
#else
    if(true)
    {
        std::this_thread::sleep_for(1000ms);
		m_ts = MAPS::CurrentTime();
#endif

        if(m_output_calib_param)
        {
            GetCalibrationParameters();
        }

        if(m_output_left)
        {
            if(m_use_multithread)
            {
                m_eventAcquireImageLeft.Set();
                std::unique_lock<std::mutex> lock(m_mutex);
                m_nbThreadRunning++;
            }
            else
            {
                GetImageLeft();
            }
        }

        if(m_output_right)
        {
            if(m_use_multithread)
            {
                m_eventAcquireImageRight.Set();
                std::unique_lock<std::mutex> lock(m_mutex);
                m_nbThreadRunning++;
            }
            else
            {
                GetImageRight();
            }
        }

        if(m_output_depth)
        {
            if(m_use_multithread)
            {
                m_eventAcquireDepth.Set();
                std::unique_lock<std::mutex> lock(m_mutex);
                m_nbThreadRunning++;
            }
            else
            {
                GetDepth();
            }
        }

        if(m_output_pointcloud)
        {
            if(m_use_multithread)
            {
                m_eventAcquirePointcloud.Set();
                std::unique_lock<std::mutex> lock(m_mutex);
                m_nbThreadRunning++;
            }
            else
            {
                GetPointcloud();
            }
        }

        if(m_output_confidence)
        {
            if(m_use_multithread)
            {
                m_eventAcquireConfidence.Set();
                std::unique_lock<std::mutex> lock(m_mutex);
                m_nbThreadRunning++;
            }
            else
            {
                GetConfidence();
            }
        }

        if(m_output_disparity)
        {
            if(m_use_multithread)
            {
                m_eventAcquireDisparity.Set();
                std::unique_lock<std::mutex> lock(m_mutex);
                m_nbThreadRunning++;
            }
            else
            {
                GetDisparity();
            }
        }

        if(m_output_object)
        {
            if(m_use_multithread)
            {
                m_eventAcquireObjects.Set();
                std::unique_lock<std::mutex> lock(m_mutex);
                m_nbThreadRunning++;
            }
            else
            {
                GetObjects();
            }
        }

        if(m_use_multithread)
        {
            while(!IsDying())
            {
                {
                    std::unique_lock<std::mutex> lock(m_mutex);
                    if(m_nbThreadRunning == 0)
                    {
                        break;
                    }
                }

                m_checkRunningThreads.Wait(1000);
                m_checkRunningThreads.Reset();
            }
        }
	}
    else
    {
        Error("Camera must be disconnected.");
    }
}

void ComponentClass::Death()
{
    if(!m_initialized)
        return;
    
    if(m_use_multithread)
    {
        m_isRunning = false;

        m_eventAcquireImageLeft.Set();
        m_eventAcquireImageRight.Set();
        m_eventAcquireDepth.Set();
        m_eventAcquirePointcloud.Set();
        m_eventAcquireConfidence.Set();
        m_eventAcquireDisparity.Set();
        m_eventAcquireObjects.Set();
        m_checkRunningThreads.Set();

        if(m_output_left)
        {
            m_threadImageLeft.join();
        }

        if(m_output_right)
        {
            m_threadImageRight.join();
        }

        if(m_output_depth)
        {
            m_threadDepth.join();
        }

        if(m_output_pointcloud)
        {
            m_threadPointcloud.join();
        }

        if(m_output_confidence)
        {
            m_threadConfidence.join();
        }

        if(m_output_disparity)
        {
            m_threadDisparity.join();
        }

        if(m_output_object)
        {
            m_threadObjects.join();
    #ifndef TEST_MODE
            m_zed->disablePositionalTracking(); 
            m_zed->disableObjectDetection(); 
    #endif
        }
    }
    else
    {
        if(m_output_object)
        {
    #ifndef TEST_MODE
            m_zed->disablePositionalTracking(); 
            m_zed->disableObjectDetection(); 
    #endif
        }
    }

    ReportInfo("Closing zed.");
#ifndef TEST_MODE
	m_zed->close();
    delete m_zed;
#endif
    m_initialized = false;
}

void ComponentClass::GetCalibrationParameters()
{
    MAPSTimestamp ts = m_ts;
    MAPSIOElt* out = StartWriting(Output(OUTPUT_CALIBRATION_PARAM));
    MapsCalibrationParameters param;
#ifndef TEST_MODE
    sl::CameraParameters calibration_params = m_zed->getCameraInformation().camera_configuration.calibration_parameters.left_cam;
    param.fx = calibration_params.fx;
    param.fy = calibration_params.fy;
    param.cx = calibration_params.cx;
    param.cy = calibration_params.cy;

    for(int i = 0; i < 12; ++i)
    {
        param.disto[i] = calibration_params.disto[i];
    }

    param.v_fov = calibration_params.v_fov;
    param.h_fov = calibration_params.h_fov;
    param.d_fov = calibration_params.d_fov;
    param.width = calibration_params.image_size.width;
    param.height = calibration_params.image_size.height;
    param.focal_length_metric = calibration_params.focal_length_metric;
#else
    param.fx = 0.1;
    param.fy = 0.2;
    param.cx = 0.3;
    param.cy = 0.4;

    for(int i = 0; i < 12; ++i)
    {
        param.disto[i] = i+1;
    }

    param.v_fov = 0.5;
    param.h_fov = 0.6;
    param.d_fov = 0.7;
    param.width = 1;
    param.height = 2;
    param.focal_length_metric = 0.8;
#endif

    memcpy(out->Data(), &param, sizeof(MapsCalibrationParameters));
    out->Timestamp() = ts;
    StopWriting(out);
}

void ComponentClass::GetImageLeftThreaded()
{
    while(m_isRunning)
    {
        m_eventAcquireImageLeft.Wait();
        m_eventAcquireImageLeft.Reset();

        if(IsDying())
            return;

        try
        {
            GetImageLeft();

	        std::unique_lock<std::mutex> lock(m_mutex);
            m_nbThreadRunning--;
            m_checkRunningThreads.Set();
        }
        catch (int ex)
        {
            if (ex != MAPS::ModuleDied) { // if not normal behaviour (StartWriting blocked by component death)
                MAPSStreamedString sx;
                sx << "Failed to write data: " << ex;
                ReportError(sx);
            }
        }
        catch (const std::exception& e)
        {
            ReportError(e.what());
        }
        catch (...)
        {
            ReportError("Unknown error");
        }
    }
}

void ComponentClass::GetImageLeft()
{
    MAPSTimestamp ts = m_ts;
    MAPSIOElt* out = StartWriting(Output(OUTPUT_LEFT));
    IplImage& out_img = out->IplImage();

#ifndef TEST_MODE
    sl::Mat data(m_width, m_height, sl::MAT_TYPE::U8_C4, (sl::uchar1*)out_img.imageData, out_img.widthStep, MEM::CPU);
    m_zed->retrieveImage(data, VIEW::LEFT);
#endif

    out->Timestamp() = ts;
    StopWriting(out);
}

void ComponentClass::GetImageRightThreaded()
{
    while(m_isRunning)
    {
        m_eventAcquireImageRight.Wait();
        m_eventAcquireImageRight.Reset();

        if(IsDying())
            return;

        try
        {
            GetImageRight();

            std::unique_lock<std::mutex> lock(m_mutex);
            m_nbThreadRunning--;
            m_checkRunningThreads.Set();
        }
        catch (int ex)
        {
            if (ex != MAPS::ModuleDied) { // if not normal behaviour (StartWriting blocked by component death)
                MAPSStreamedString sx;
                sx << "Failed to write data: " << ex;
                ReportError(sx);
            }
        }
        catch (const std::exception& e)
        {
            ReportError(e.what());
        }
        catch (...)
        {
            ReportError("Unknown error");
        }
    }
}

void ComponentClass::GetImageRight()
{

    MAPSTimestamp ts = m_ts;

    MAPSIOElt* out = StartWriting(Output(OUTPUT_RIGHT));
    IplImage& out_img = out->IplImage();

#ifndef TEST_MODE
    sl::Mat data(m_width, m_height, sl::MAT_TYPE::U8_C4, (sl::uchar1*)out_img.imageData, out_img.widthStep, MEM::CPU);
    m_zed->retrieveImage(data, VIEW::RIGHT);
#endif

    out->Timestamp() = ts;
    StopWriting(out);
}

void ComponentClass::GetDepthThreaded()
{
    while(m_isRunning)
    {
        m_eventAcquireDepth.Wait();
        m_eventAcquireDepth.Reset();

        if(IsDying())
            return;

        try
        {
            GetDepth();

            std::unique_lock<std::mutex> lock(m_mutex);
            m_nbThreadRunning--;
            m_checkRunningThreads.Set();
        }
        catch (int ex)
        {
            if (ex != MAPS::ModuleDied) { // if not normal behaviour (StartWriting blocked by component death)
                MAPSStreamedString sx;
                sx << "Failed to write data: " << ex;
                ReportError(sx);
            }
        }
        catch (const std::exception& e)
        {
            ReportError(e.what());
        }
        catch (...)
        {
            ReportError("Unknown error");
        }
    }
}

void ComponentClass::GetDepth()
{
    MAPSTimestamp ts = m_ts;

    MAPSIOElt* out = StartWriting(Output(OUTPUT_DEPTH));

#ifndef TEST_MODE
    if(m_depth_data_format == DataFormat::FLOAT_VECTOR)
    {
        sl::Mat data(m_width, m_height, sl::MAT_TYPE::F32_C1, (sl::uchar1*)out->Data(), m_width * 4, MEM::CPU);
        m_zed->retrieveMeasure(data, MEASURE::DEPTH);
    }
    else
    {
        IplImage& out_img = out->IplImage();
        sl::Mat data(m_width, m_height, sl::MAT_TYPE::U8_C4, (sl::uchar1*)out_img.imageData, out_img.widthStep, MEM::CPU);
        m_zed->retrieveImage(data, VIEW::DEPTH);
    }
#endif

    out->Timestamp() = ts;
    StopWriting(out);
}

void ComponentClass::GetConfidence()
{
    MAPSTimestamp ts = m_ts;

    MAPSIOElt* out = StartWriting(Output(OUTPUT_CONFIDENCE));

#ifndef TEST_MODE
    if(m_confidence_data_format == DataFormat::FLOAT_VECTOR)
    {
        sl::Mat data(m_width, m_height, sl::MAT_TYPE::F32_C1, (sl::uchar1*)out->Data(), m_width * 4, MEM::CPU);
        m_zed->retrieveMeasure(data, MEASURE::CONFIDENCE);
    }
    else
    {
        IplImage& out_img = out->IplImage();
        sl::Mat data(m_width, m_height, sl::MAT_TYPE::U8_C4, (sl::uchar1*)out_img.imageData, out_img.widthStep, MEM::CPU);
        m_zed->retrieveImage(data, VIEW::CONFIDENCE);
    }
#endif

    out->Timestamp() = ts;
    StopWriting(out);
}

void ComponentClass::GetConfidenceThreaded()
{
	while(m_isRunning)
    {
        m_eventAcquireConfidence.Wait();
        m_eventAcquireConfidence.Reset();

        if(IsDying())
            return;

        try
        {
            GetConfidence();

            std::unique_lock<std::mutex> lock(m_mutex);
            m_nbThreadRunning--;
            m_checkRunningThreads.Set();
        }
        catch (int ex)
        {
            if (ex != MAPS::ModuleDied) { // if not normal behaviour (StartWriting blocked by component death)
                MAPSStreamedString sx;
                sx << "Failed to write data: " << ex;
                ReportError(sx);
            }
        }
        catch (const std::exception& e)
        {
            ReportError(e.what());
        }
        catch (...)
        {
            ReportError("Unknown error");
        }
    }
}

void ComponentClass::GetDisparity()
{
     MAPSTimestamp ts = m_ts;

    MAPSIOElt* out = StartWriting(Output(OUTPUT_DISPARITY));
#ifndef TEST_MODE
    sl::Mat data(m_width, m_height, sl::MAT_TYPE::F32_C1, (sl::uchar1*)out->Data(), m_width * 4, MEM::CPU);
    m_zed->retrieveMeasure(data, MEASURE::DISPARITY);
#endif
    out->Timestamp() = ts;
    StopWriting(out);
}

void ComponentClass::GetDisparityThreaded()
{
	while(m_isRunning)
    {
        m_eventAcquireDisparity.Wait();
        m_eventAcquireDisparity.Reset();

        if(IsDying())
            return;

        try
        {
            GetDisparity();

            std::unique_lock<std::mutex> lock(m_mutex);
            m_nbThreadRunning--;
            m_checkRunningThreads.Set();
        }
        catch (int ex)
        {
            if (ex != MAPS::ModuleDied) { // if not normal behaviour (StartWriting blocked by component death)
                MAPSStreamedString sx;
                sx << "Failed to write data: " << ex;
                ReportError(sx);
            }
        }
        catch (const std::exception& e)
        {
            ReportError(e.what());
        }
        catch (...)
        {
            ReportError("Unknown error");
        }
    }
}

void ComponentClass::GetPointcloud()
{
    MAPSTimestamp ts = m_ts;

    MAPSIOElt* out = StartWriting(Output(OUTPUT_POINTCLOUD));
    MAPSFloat32* ptr = reinterpret_cast<MAPSFloat32*>(out->Data());
    sl::Mat data;

#ifndef TEST_MODE
    switch (m_pointcloud_format) 
    {
    case PointCloudFormat::XYZ:
    {
        m_zed->retrieveMeasure(data, MEASURE::XYZ);
        MAPSFloat32* ptrData = data.getPtr<MAPSFloat32>();

        for(int i = 0; i < m_width * m_height; ++i)
        {
            int index = i * 3;
            int indexData = i * 4;
            ptr[index] = ptrData[indexData] / m_pointcloud_divide;
            ptr[index + 1] = ptrData[indexData + 1] / m_pointcloud_divide;
            ptr[index + 2] = ptrData[indexData + 2] / m_pointcloud_divide;
        }
    }
        break;
    case PointCloudFormat::XYZ_RGB:
    {
        m_zed->retrieveMeasure(data, MEASURE::XYZRGBA);
        MAPSFloat32* ptrData = data.getPtr<MAPSFloat32>();

        for(int i = 0; i < m_width * m_height; ++i)
        {
            int index = i * 6;
            int indexData = i * 4;
            ptr[index] = ptrData[indexData] / m_pointcloud_divide;
            ptr[index + 1] = ptrData[indexData + 1] / m_pointcloud_divide;
            ptr[index + 2] = ptrData[indexData + 2] / m_pointcloud_divide;
            unsigned char* ptrColor = reinterpret_cast<unsigned char*>(&ptrData[indexData + 3]);
            ptr[index + 3] = (float)ptrColor[0] / 255.0;
            ptr[index + 4] = (float)ptrColor[1] / 255.0;
            ptr[index + 5] = (float)ptrColor[2] / 255.0;
        }
    }
        break;
    default:
        Error("unexpected pointcloud format");
    }
#endif

    out->Timestamp() = ts;
    StopWriting(out);
}

void ComponentClass::GetPointcloudThreaded()
{
	while(m_isRunning)
    {
        m_eventAcquirePointcloud.Wait();
        m_eventAcquirePointcloud.Reset();

        if(IsDying())
            return;

        try
        {
            GetPointcloud();

            std::unique_lock<std::mutex> lock(m_mutex);
            m_nbThreadRunning--;
            m_checkRunningThreads.Set();
        }
        catch (int ex)
        {
            if (ex != MAPS::ModuleDied) { // if not normal behaviour (StartWriting blocked by component death)
                MAPSStreamedString sx;
                sx << "Failed to write data: " << ex;
                ReportError(sx);
            }
        }
        catch (const std::exception& e)
        {
            ReportError(e.what());
        }
        catch (...)
        {
            ReportError("Unknown error");
        }
    }
}

void ComponentClass::GetObjects()
{
    MAPSTimestamp ts = m_ts;

    MAPSIOElt* out = StartWriting(Output(OUTPUT_OBJECT));
    MAPSIOElt* outBounding = StartWriting(Output(OUTPUT_OBJECT_BOUNDING_BOX));
    MAPSIOElt* outBounding3d = StartWriting(Output(OUTPUT_OBJECT_BOUNDING_BOX_3D));
    MAPSIOElt* outLabel = StartWriting(Output(OUTPUT_OBJECT_LABEL));
    MAPSRealObject* objOut = &out->RealObject();
    MAPSRealObject* bounding3d = &outBounding3d->RealObject();
    MAPSDrawingObject* bounding = &outBounding->DrawingObject();
    MAPSDrawingObject* label = &outLabel->DrawingObject();

    int nbObjectsOut = 0;
    int nbDrawingOut = 0;
    int nbBoundingBox3dOut = 0;

#ifndef TEST_MODE
    m_zed->retrieveObjects(m_objects, m_detection_parameters_rt);
#else
    m_objects.object_list.clear();
    sl::ObjectData data;
    data.id = 1;
    data.sublabel = OBJECT_SUBCLASS::CAR;
    data.dimensions[0] = 100;
    data.dimensions[1] = 101;
    data.dimensions[2] = 102;
    data.position[0] = 200;
    data.position[1] = 201;
    data.position[2] = 202;
    data.bounding_box_2d.push_back({1,2});
    data.bounding_box_2d.push_back({3,4});
    data.bounding_box_2d.push_back({5,6});
    data.bounding_box_2d.push_back({7,8});
    m_objects.object_list.push_back(data);
#endif
    
    if (!m_objects.object_list.empty()) 
    {
        int nbObjects = m_objects.object_list.size();

        if(nbObjects > m_objects_vector_size)
        {
            ReportWarning("Too much objects detected, truncating... (increase object_vector_size property)");
            nbObjects = m_objects_vector_size;
        }

        for(int i = 0; i < nbObjects; ++i)
        {
            sl::ObjectData obj = m_objects.object_list[i];

            objOut->id = obj.id;
            objOut->kind =  MAPSRealObject::Tree;
            MAPSInt32 color = MAPS_RGB(255,255,255);

            switch(obj.sublabel)
            {
            case OBJECT_SUBCLASS::BICYCLE:
                objOut->kind = MAPSRealObject::Vehicle;
                objOut->vehicle.kind = MAPSVehicle::Bike;
                color = MAPS_RGB(3,252,161);
                break;
            case OBJECT_SUBCLASS::MOTORBIKE:
                objOut->kind = MAPSRealObject::Vehicle;
                objOut->vehicle.kind = MAPSVehicle::Motorcycle;
                color = MAPS_RGB(3,252,211);
                break;
            case OBJECT_SUBCLASS::CAR:
                objOut->kind = MAPSRealObject::Vehicle;
                objOut->vehicle.kind = MAPSVehicle::Car;
                color = MAPS_RGB(252,44,3);
                break;
            case OBJECT_SUBCLASS::BUS:
                objOut->kind = MAPSRealObject::Vehicle;
                objOut->vehicle.kind = MAPSVehicle::Motorcycle;
                color = MAPS_RGB(65,252,3);
                break;
            case OBJECT_SUBCLASS::TRUCK:
                objOut->kind = MAPSRealObject::Vehicle;
                objOut->vehicle.kind = MAPSVehicle::Truck;
                color = MAPS_RGB(232,252,3);
                break;
            case OBJECT_SUBCLASS::PERSON:
            case OBJECT_SUBCLASS::PERSON_HEAD:
            case OBJECT_SUBCLASS::BIRD:
            case OBJECT_SUBCLASS::CAT:
            case OBJECT_SUBCLASS::DOG:
            case OBJECT_SUBCLASS::HORSE:
            case OBJECT_SUBCLASS::SHEEP:
            case OBJECT_SUBCLASS::COW:
                objOut->kind = MAPSRealObject::Custom;
                objOut->custom.length = obj.dimensions[2];
                objOut->custom.width = obj.dimensions[0];
                objOut->custom.height = obj.dimensions[1];
                color = MAPS_RGB(3,7,252);
                break;
            default:
                break;
            }
            
            if(objOut->kind != MAPSRealObject::Tree)
            {
                if (objOut->kind == MAPSRealObject::Vehicle)
                {
                    objOut->vehicle.length = obj.dimensions[2];
                    objOut->vehicle.width = obj.dimensions[0];
                    objOut->vehicle.height = obj.dimensions[1];
                    objOut->vehicle.speed = std::sqrt(std::pow(obj.velocity[0], 2) + std::pow(obj.velocity[1], 2) + std::pow(obj.velocity[3], 2));
                    objOut->vehicle.confidence = obj.confidence;
                }

                objOut->x = obj.position[0];
                objOut->y = obj.position[1];
                objOut->z = obj.position[2];
                objOut->color = color;

                objOut++;
                nbObjectsOut++;
            }

            std::string labelStr = "unknown";
            switch(obj.sublabel)
            {
            case OBJECT_SUBCLASS::BICYCLE:
                labelStr = "Bicycle";
                break;
            case OBJECT_SUBCLASS::MOTORBIKE:
                labelStr = "Motorbike";
                break;
            case OBJECT_SUBCLASS::CAR:
                labelStr = "Car";
                break;
            case OBJECT_SUBCLASS::BUS:
                labelStr = "Bus";
                break;
            case OBJECT_SUBCLASS::TRUCK:
                labelStr = "Truck";
                break;
            case OBJECT_SUBCLASS::PERSON:
                labelStr = "Person";
                break;
            case OBJECT_SUBCLASS::PERSON_HEAD:
                labelStr = "Person head";
                break;
            case OBJECT_SUBCLASS::BIRD:
                labelStr = "Bird";
                break;
            case OBJECT_SUBCLASS::CAT:
                labelStr = "Cat";
                break;
            case OBJECT_SUBCLASS::DOG:
                labelStr = "Dog";
                break;
            case OBJECT_SUBCLASS::HORSE:
                labelStr = "Horse";
                break;
            case OBJECT_SUBCLASS::SHEEP:
                labelStr = "Sheep";
                break;
            case OBJECT_SUBCLASS::COW:
                labelStr = "Cow";
                break;
            case OBJECT_SUBCLASS::CELLPHONE:
                labelStr = "Cellphone";
                break;
            case OBJECT_SUBCLASS::LAPTOP:
                labelStr = "Laptop";
                break;
            case OBJECT_SUBCLASS::BANANA:
                labelStr = "Banana";
                break;
            case OBJECT_SUBCLASS::APPLE:
                labelStr = "Apple";
                break;
            case OBJECT_SUBCLASS::ORANGE:
                labelStr = "Orange";
                break;
            case OBJECT_SUBCLASS::CARROT:
                labelStr = "Carrot";
                break;
            case OBJECT_SUBCLASS::SPORTSBALL:
                labelStr = "Sports ball";
                break;
            default:
                labelStr = "unknown";
                break;
            }

            bounding3d->kind = MAPSRealObject::Custom;
            bounding3d->x = obj.position[0];
            bounding3d->y = obj.position[1];
            bounding3d->z = obj.position[2];
            bounding3d->custom.width = obj.dimensions[0];
            bounding3d->custom.height = obj.dimensions[1];
            bounding3d->custom.length = obj.dimensions[2];
            nbBoundingBox3dOut++;
            bounding3d++;

            if(obj.bounding_box_2d.size() == 4)
            {
                MAPS::Memset(bounding, 0, sizeof(MAPSDrawingObject));
                bounding->kind = MAPSDrawingObject::Rectangle;
                bounding->id = obj.id;
                bounding->color = color;
                bounding->width = 2;
                bounding->rectangle.x1 = obj.bounding_box_2d[0][0];
                bounding->rectangle.y1 = obj.bounding_box_2d[0][1];
                bounding->rectangle.x2 = obj.bounding_box_2d[2][0];
                bounding->rectangle.y2 = obj.bounding_box_2d[2][1];

                MAPS::Memset(label, 0, sizeof(MAPSDrawingObject));
                label->kind = MAPSDrawingObject::Text;
                label->id = obj.id;
                label->color = color;
                label->width = 2;
                label->text.x = bounding->rectangle.x1 + 10;
                label->text.y = bounding->rectangle.y1 + 10;
                label->text.cheight = 10;
                label->text.cwidth = 10;
                MAPS::Strcpy(label->text.text, labelStr.c_str());

                bounding++;
                label++;
                nbDrawingOut++;
            }
        }
    }

    out->VectorSize() = nbObjectsOut;
    outBounding->VectorSize() = nbDrawingOut;
    outBounding3d->VectorSize() = nbBoundingBox3dOut;
    outLabel->VectorSize() = nbDrawingOut;
    out->Timestamp() = ts;
    outBounding->Timestamp() = ts;
    outBounding3d->Timestamp() = ts;
    outLabel->Timestamp() = ts;
    StopWriting(out);
    StopWriting(outBounding);
    StopWriting(outBounding3d);
    StopWriting(outLabel);
}

void ComponentClass::GetObjectsThreaded()
{
	while(m_isRunning)
    {
        m_eventAcquireObjects.Wait();
        m_eventAcquireObjects.Reset();

        if(IsDying())
            return;

        try
        {
            GetObjects();

            std::unique_lock<std::mutex> lock(m_mutex);
            m_nbThreadRunning--;
            m_checkRunningThreads.Set();
        }
        catch (int ex)
        {
            if (ex != MAPS::ModuleDied) { // if not normal behaviour (StartWriting blocked by component death)
                MAPSStreamedString sx;
                sx << "Failed to write data: " << ex;
                ReportError(sx);
            }
        }
        catch (const std::exception& e)
        {
            ReportError(e.what());
        }
        catch (...)
        {
            ReportError("Unknown error");
        }
    }
}