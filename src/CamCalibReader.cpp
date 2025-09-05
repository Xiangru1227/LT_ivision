#include "CamCalibReader.h"
#include <string>
#include <algorithm>

CalibHandle::CalibHandle()
	: cData(CalibData())
{

}

CalibHandle::~CalibHandle() {

}

int CalibHandle::readCalibFile(const char * filePath) {

	try {
		if (filePath == NULL)
		{
			return -1;
		}
		if (strcmp(filePath, "") == 0)
		{
			return -2;
		}
		std::ifstream calibFile(filePath);

		if (!calibFile.is_open()) {
			return -3;
		}

		jsonReader.clear();

		if ((calibFile.peek() == std::ifstream::traits_type::eof()))
		{
			calibFile.close();
			return -4;
		}
		calibFile >> jsonReader;

		return parseCalibFile();
	}
	catch (std::exception &e) {
		// need to handle if the json file is corrupted
		std::cout << "Exception :" << std::endl;
		return -1;
	}
}

int CalibHandle::parseCalibFile() {

	// test
	if (!jsonReader["Product_Info"].isMember("Serial_Number")
		|| !jsonReader["Cam_Calib"].isMember("FX")
		|| !jsonReader["Cam_Calib"].isMember("CX")
		|| !jsonReader["Cam_Calib"].isMember("FY")
		|| !jsonReader["Cam_Calib"].isMember("CY")
		|| !jsonReader["Dist_Coeff"].isMember("K1")
		|| !jsonReader["Dist_Coeff"].isMember("K2")
		|| !jsonReader["Dist_Coeff"].isMember("P1")
		|| !jsonReader["Dist_Coeff"].isMember("P2")
		|| !jsonReader["Dist_Coeff"].isMember("K3")
		|| !jsonReader["Parallax"].isMember("Distance")
		|| !jsonReader["Parallax"].isMember("X")
		|| !jsonReader["Parallax"].isMember("Y")
		|| !jsonReader["Parallax"].isMember("AutoCalib_X_offset")
		|| !jsonReader["Parallax"].isMember("AutoCalib_Y_offset")
		//|| !jsonReader["Parallax"].isMember("Size")
		)
	{
		return -5;
	}

	// validate parallax calibration table
	unsigned int distance_size = jsonReader["Parallax"]["Distance"].size();
	if (jsonReader["Parallax"]["X"].size() != distance_size
		|| jsonReader["Parallax"]["Y"].size() != distance_size
		/*|| jsonReader["Parallax"]["Size"].size() != distance_size*/) {
		return -6;
	}

	//set parallax calibration table
	cData.parallaxTable = ParallaxTableStruct();
	for (unsigned int i = 0; i < distance_size; ++i) {
		cData.parallaxTable.distance.push_back(jsonReader["Parallax"]["Distance"][i].asDouble());
		std::pair<float,float> off = std::make_pair(jsonReader["Parallax"]["X"][i].asDouble(),jsonReader["Parallax"]["Y"][i].asDouble());
		cData.parallaxTable.offset.push_back(off);
		//cData.parallaxTable.size.push_back(jsonReader["Parallax"]["Size"][i].asDouble());
	}

	//get serial number
	cData.serial_num = jsonReader["Product_Info"]["Serial_Number"].asUInt();

	//set camera calibration matrix values
	cData.camCalib.cam_fx = jsonReader["Cam_Calib"]["FX"].asDouble();
	cData.camCalib.cam_cx = jsonReader["Cam_Calib"]["CX"].asDouble();
	cData.camCalib.cam_fy = jsonReader["Cam_Calib"]["FY"].asDouble();
	cData.camCalib.cam_cy = jsonReader["Cam_Calib"]["CY"].asDouble();

	//set distortion coefficients
	cData.camCalib.dist_k1 = jsonReader["Dist_Coeff"]["K1"].asDouble();
	cData.camCalib.dist_k2 = jsonReader["Dist_Coeff"]["K2"].asDouble();
	cData.camCalib.dist_p1 = jsonReader["Dist_Coeff"]["P1"].asDouble();
	cData.camCalib.dist_p2 = jsonReader["Dist_Coeff"]["P2"].asDouble();
	cData.camCalib.dist_k3 = jsonReader["Dist_Coeff"]["K3"].asDouble();

	if (jsonReader["iProbe_info"].isMember("FC_in_FB") && jsonReader["iProbe_info"]["FC_in_FB"].isArray()) {
		for (int i = 0; i < 3; ++i) {
			if (jsonReader["iProbe_info"]["FC_in_FB"][i].isArray()) {
				for (int j = 0; j < 3; ++j) {
					cData.FC_in_FB.at<double>(i, j) = jsonReader["iProbe_info"]["FC_in_FB"][i][j].asDouble();
				}
			}
		}
	} else {
		cData.FC_in_FB = cv::Mat::eye(3, 3, CV_64F);
	}

	//Image Processing threshold setting from config file
	if (jsonReader["Product_Info"].isMember("Bright_Threshold")) {
		cData.bright_threshold = (uint8_t)jsonReader["Product_Info"]["Bright_Threshold"].asUInt();
	}
	else {
		cData.bright_threshold = 240;
	}

	if (jsonReader["Product_Info"].isMember("red_thresh_y")) {
		cData.red_thresh_y = (uint8_t)jsonReader["Product_Info"]["red_thresh_y"].asUInt();
	}
	else {
		cData.red_thresh_y = 70;
	}

	if (jsonReader["Product_Info"].isMember("red_thresh_cr")) {
		cData.red_thresh_cr = (uint8_t)jsonReader["Product_Info"]["red_thresh_cr"].asUInt();
	}
	else {
		cData.red_thresh_cr = 25;
	}

	if (jsonReader["Product_Info"].isMember("green_thresh_cr")) {
		cData.green_thresh_cr = (uint8_t)jsonReader["Product_Info"]["green_thresh_cr"].asUInt();
	}
	else {
		cData.green_thresh_cr = 22;
	}

	if (jsonReader["Product_Info"].isMember("green_thresh_cb")) {
		cData.green_thresh_cb = (uint8_t)jsonReader["Product_Info"]["green_thresh_cb"].asUInt();
	}
	else {
		cData.green_thresh_cb = 22;
	}


	//adding LED Flash settings to the config file
	if (jsonReader["Product_Info"].isMember("flash_duration")) {
		cData.flash_duration = jsonReader["Product_Info"]["flash_duration"].asFloat();
	}
	else {
		cData.flash_duration = 40.0f;
	}

	if (jsonReader["Product_Info"].isMember("flash_offset")) {
		cData.flash_offset = jsonReader["Product_Info"]["flash_offset"].asFloat();
	}
	else {
		cData.flash_offset = 1.0f;
	}

	if (jsonReader["Product_Info"].isMember("flash_brightness")) {
		cData.flash_brightness = jsonReader["Product_Info"]["flash_brightness"].asFloat();
	}
	else {
		cData.flash_brightness = 0.1f;
	}

	//backside detection control
	if (jsonReader["Product_Info"].isMember("back_side_detection")) {
		cData.back_side_detection = jsonReader["Product_Info"]["back_side_detection"].asBool();
	}
	else {
		cData.back_side_detection = true;
	}

	//spiral search threshold to stop
	if (jsonReader["Product_Info"].isMember("Spiral_threshold")) {
		cData.spiral_threshold = jsonReader["Product_Info"]["Spiral_threshold"].asFloat();
	}
	else {
		cData.spiral_threshold = 0.1f;
	}

	//spiral search timeout to stop
	if (jsonReader["Product_Info"].isMember("Spiral_timeout")) {
		cData.spiral_timeout = jsonReader["Product_Info"]["Spiral_timeout"].asFloat();
	}
	else {
		cData.spiral_timeout = 5;
	}

	//spiral freq
	if (jsonReader["Product_Info"].isMember("Spiral_freq")) {
		cData.spiral_freq = jsonReader["Product_Info"]["Spiral_freq"].asFloat();
	}
	else {
		cData.spiral_freq = 0.004f;
	}

	//Camera Auto Calib switch
	if (jsonReader["Product_Info"].isMember("cam_auto_calib")) {
		cData.cam_auto_calib = jsonReader["Product_Info"]["cam_auto_calib"].asFloat();
	}
	else {
		cData.cam_auto_calib = false;
	}

	//auto flash adjust factor
	if (jsonReader["Product_Info"].isMember("auto_flash_adjust_factor")) {
		cData.auto_flash_adjust_factor = jsonReader["Product_Info"]["auto_flash_adjust_factor"].asFloat();
	}
	else {
		cData.auto_flash_adjust_factor = 0.5f;
	}

	//target intensity for auto exposure
	if (jsonReader["Product_Info"].isMember("target_intensity_for_auto_exp")) {
		cData.target_intensity_for_auto_exp = jsonReader["Product_Info"]["target_intensity_for_auto_exp"].asFloat();
	}
	else {
		cData.target_intensity_for_auto_exp = 128.0f;
	}

	//auto exposure reset interval
	if (jsonReader["Product_Info"].isMember("auto_exp_reset_interval")) {
		cData.auto_exp_reset_interval = jsonReader["Product_Info"]["auto_exp_reset_interval"].asUInt();
	}
	else {
		cData.auto_exp_reset_interval = 200;
	}

	//target intensity threshold for auto exposure
	if (jsonReader["Product_Info"].isMember("target_intensity_threshold_high")) {
		cData.target_intensity_threshold_high = jsonReader["Product_Info"]["target_intensity_threshold_high"].asFloat();
	}
	else {
		cData.target_intensity_threshold_high = 60.0f;
	}

	//target intensity threshold for auto exposure
	if (jsonReader["Product_Info"].isMember("target_intensity_threshold_low")) {
		cData.target_intensity_threshold_low = jsonReader["Product_Info"]["target_intensity_threshold_low"].asFloat();
	}
	else {
		cData.target_intensity_threshold_low = 20.0f;
	}
	//Parallax auto calib offset setting from config file
	if (jsonReader["Parallax"].isMember("AutoCalib_X_offset")) {
		cData.parallaxTable.auto_calib_x_offset = jsonReader["Parallax"]["AutoCalib_X_offset"].asDouble();
	}
	else {
		cData.parallaxTable.auto_calib_x_offset = 0.0;
	}

	if (jsonReader["Parallax"].isMember("AutoCalib_Y_offset")) {
		cData.parallaxTable.auto_calib_y_offset = jsonReader["Parallax"]["AutoCalib_Y_offset"].asDouble();
	}
	else {
		cData.parallaxTable.auto_calib_y_offset = 0.0;
	}

	//adding camera interface control to the config file
	if (jsonReader["Camera_Interface"].isMember("res_x")) {
		cData.CamProp.res_x = jsonReader["Camera_Interface"]["res_x"].asFloat();
	}
	else {
		cData.CamProp.res_x = 3264;
	}

	if (jsonReader["Camera_Interface"].isMember("res_y")) {
		cData.CamProp.res_y = jsonReader["Camera_Interface"]["res_y"].asFloat();
	}
	else {
		cData.CamProp.res_y = 2464;
	}

	if (jsonReader["Camera_Interface"].isMember("video_res_x")) {
		cData.CamProp.video_res_x =jsonReader["Camera_Interface"]["video_res_x"].asUInt();
	}
	else {
		cData.CamProp.video_res_x = 960;
	}

	if (jsonReader["Camera_Interface"].isMember("video_res_y")) {
		cData.CamProp.video_res_y = jsonReader["Camera_Interface"]["video_res_y"].asUInt();
	}
	else {
		cData.CamProp.video_res_y = 720;
	}

	if (jsonReader["Camera_Interface"].isMember("video_roi_x")) {
		cData.CamProp.video_roi_x = jsonReader["Camera_Interface"]["video_roi_x"].asFloat();
	}
	else {
		cData.CamProp.video_roi_x = 0;
	}

	if (jsonReader["Camera_Interface"].isMember("video_roi_y")) {
		cData.CamProp.video_roi_y = jsonReader["Camera_Interface"]["video_roi_y"].asFloat();
	}
	else {
		cData.CamProp.video_roi_y = 0;
	}

	if (jsonReader["Camera_Interface"].isMember("video_roi_width")) {
		cData.CamProp.video_roi_width = jsonReader["Camera_Interface"]["video_roi_width"].asFloat();
	}
	else {
		cData.CamProp.video_roi_width = 1;
	}

	if (jsonReader["Camera_Interface"].isMember("video_roi_height")) {
		cData.CamProp.video_roi_height = jsonReader["Camera_Interface"]["video_roi_height"].asFloat();
	}
	else {
		cData.CamProp.video_roi_height = 1;
	}

	if (jsonReader["Camera_Interface"].isMember("fps")) {
		cData.CamProp.fps = jsonReader["Camera_Interface"]["fps"].asInt();
	}
	else {
		cData.CamProp.fps = 21;
	}

	if (jsonReader["Camera_Interface"].isMember("exposure")) {
		cData.CamProp.exposure = jsonReader["Camera_Interface"]["exposure"].asDouble();
	}
	else {
		cData.CamProp.exposure = 6.0;
	}

	if (jsonReader["Camera_Interface"].isMember("analog_gain")) {
		cData.CamProp.analog_gain = jsonReader["Camera_Interface"]["analog_gain"].asFloat();
	}
	else {
		cData.CamProp.analog_gain = 1.5f;
	}

	if (jsonReader["Camera_Interface"].isMember("digital_gain")) {
		cData.CamProp.digital_gain = jsonReader["Camera_Interface"]["digital_gain"].asFloat();
	}
	else {
		cData.CamProp.digital_gain = 1.0f;
	}

	if (jsonReader["Camera_Interface"].isMember("stream_exposure")) {
		cData.CamProp.stream_exposure = jsonReader["Camera_Interface"]["stream_exposure"].asDouble();
	}
	else {
		cData.CamProp.stream_exposure = 15.0;
	}

	if (jsonReader["Camera_Interface"].isMember("stream_analog_gain")) {
		cData.CamProp.stream_analog_gain = jsonReader["Camera_Interface"]["stream_analog_gain"].asFloat();
	}
	else {
		cData.CamProp.stream_analog_gain = 1.5f;
	}

	if (jsonReader["Camera_Interface"].isMember("stream_digital_gain")) {
		cData.CamProp.stream_digital_gain = jsonReader["Camera_Interface"]["stream_digital_gain"].asFloat();
	}
	else {
		cData.CamProp.stream_digital_gain = 1.0f;
	}


	if (jsonReader["Camera_Interface"].isMember("flash_on")) {
		cData.CamProp.flash_on = jsonReader["Camera_Interface"]["flash_on"].asBool();
	}
	else {
		cData.CamProp.flash_on = true;
	}

	return 0;
}

int CalibHandle::writeCalibFile(const char * filePath) {

	try {
		if (filePath == NULL)
		{
			return -1;
		}
		if (strcmp(filePath, "") == 0)
		{
			return -2;
		}
		std::ofstream wrFile(filePath);

		if (!wrFile.is_open()) {
			return -3;
		}
		jsonReader.clear();
		std::cout << "Writing the file now " << std::endl;
		updateCalibFields();
		wrFile << jsonReader;
		wrFile.close();

	}
	catch (...) {

	}
	return 0;

}
int CalibHandle::updateCalibFields() {

	//check parallax table size
	int table_size = cData.parallaxTable.distance.size();
	if (cData.parallaxTable.offset.size() != table_size/* || cData.parallaxTable.size.size() != table_size*/) {
		return -8;
	}
	//sorting the distance and offset to make sure the order is not messed up
	
	std::vector<CalibrationPoint> temp_parallax_table;
	//populate temporary table for sorting
	for (size_t i = 0; i < table_size; ++i) {
		CalibrationPoint c;
		c.distance = cData.parallaxTable.distance[i];
		c.az = cData.parallaxTable.offset[i].first;
		c.el = cData.parallaxTable.offset[i].second;
		temp_parallax_table.push_back(c);
	}

	std::sort(temp_parallax_table.begin(), temp_parallax_table.end(), [](const CalibrationPoint& lhs, const CalibrationPoint& rhs) {
		return lhs.distance < rhs.distance;
	});
	//update parallax table
	Json::Value l_vector_distance(Json::arrayValue);
	Json::Value l_vector_offset_x(Json::arrayValue);
	Json::Value l_vector_offset_y(Json::arrayValue);
	//Json::Value l_vector4(Json::arrayValue);
	for (int i = 0; i < table_size; ++i) {
		l_vector_distance.append(temp_parallax_table[i].distance);
		l_vector_offset_x.append(temp_parallax_table[i].az);
		l_vector_offset_y.append(temp_parallax_table[i].el);
		//l_vector4.append(Json::Value(cData.parallaxTable.size[i]));
	}
	jsonReader["Parallax"]["Distance"] = l_vector_distance;
	jsonReader["Parallax"]["X"] = l_vector_offset_x;
	jsonReader["Parallax"]["Y"] = l_vector_offset_y;
	//jsonReader["Parallax"]["Size"] = l_vector4;

	//update transform matrix
	Json::Value transMat(Json::arrayValue);
	for (int i = 0; i < 3; ++i) {
		Json::Value row(Json::arrayValue);
		for (int j = 0; j < 3; ++j) {
			row.append(cData.FC_in_FB.at<double>(i, j));
		}
		transMat.append(row);
	}
	jsonReader["iProbe_info"]["FC_in_FB"] = transMat;

	//update the AutoCalib x and y offset
	jsonReader["Parallax"]["AutoCalib_X_offset"] = cData.parallaxTable.auto_calib_x_offset;
	jsonReader["Parallax"]["AutoCalib_Y_offset"] = cData.parallaxTable.auto_calib_y_offset;
	
	//update serial number
	jsonReader["Product_Info"]["Serial_Number"] = cData.serial_num;

	//update cam calibration matrix values
	jsonReader["Cam_Calib"]["FX"] = cData.camCalib.cam_fx;
	jsonReader["Cam_Calib"]["CX"] = cData.camCalib.cam_cx;
	jsonReader["Cam_Calib"]["FY"] = cData.camCalib.cam_fy;
	jsonReader["Cam_Calib"]["CY"] = cData.camCalib.cam_cy;

	//update distortion coefficients
	jsonReader["Dist_Coeff"]["K1"] = cData.camCalib.dist_k1;
	jsonReader["Dist_Coeff"]["K2"] = cData.camCalib.dist_k2;
	jsonReader["Dist_Coeff"]["P1"] = cData.camCalib.dist_p1;
	jsonReader["Dist_Coeff"]["P2"] = cData.camCalib.dist_p2;
	jsonReader["Dist_Coeff"]["K3"] = cData.camCalib.dist_k3;

	//threshold adjustment
	jsonReader["Product_Info"]["Bright_Threshold"] = cData.bright_threshold;
	jsonReader["Product_Info"]["red_thresh_y"] = cData.red_thresh_y;
	jsonReader["Product_Info"]["red_thresh_cr"] = cData.red_thresh_cr;
	jsonReader["Product_Info"]["green_thresh_cr"] = cData.green_thresh_cr;
	jsonReader["Product_Info"]["green_thresh_cb"] = cData.green_thresh_cb;

	//flash settings
	jsonReader["Product_Info"]["flash_duration"] = cData.flash_duration;
	jsonReader["Product_Info"]["flash_brightness"] = cData.flash_brightness;
	jsonReader["Product_Info"]["flash_offset"] = cData.flash_offset;
	//backside detection control
	jsonReader["Product_Info"]["back_side_detection"] = cData.back_side_detection;
	//spiral Threshold value
	jsonReader["Product_Info"]["Spiral_threshold"] = cData.spiral_threshold;
	//set spiral_timeout
	jsonReader["Product_Info"]["Spiral_timeout"] = cData.spiral_timeout;
	//set spiral_freq
	jsonReader["Product_Info"]["Spiral_freq"] = cData.spiral_freq;
	//set auto calib config
	jsonReader["Product_Info"]["cam_auto_calib"] = cData.cam_auto_calib;
	//set auto flash adjust factor
	jsonReader["Product_Info"]["auto_flash_adjust_factor"] = cData.auto_flash_adjust_factor;
	//set auto target intensity for auto exposure
	jsonReader["Product_Info"]["target_intensity_for_auto_exp"] = cData.target_intensity_for_auto_exp;
	//auto exp reset interval
	jsonReader["Product_Info"]["auto_exp_reset_interval"] = cData.auto_exp_reset_interval;
	//target intensity threshold for auto exposure
	jsonReader["Product_Info"]["target_intensity_threshold_high"] = cData.target_intensity_threshold_high;
	jsonReader["Product_Info"]["target_intensity_threshold_low"] = cData.target_intensity_threshold_low;


	//updating camera interface setting using config file
	jsonReader["Camera_Interface"]["res_x"] = cData.CamProp.res_x;
	jsonReader["Camera_Interface"]["res_y"] = cData.CamProp.res_y;
	jsonReader["Camera_Interface"]["video_res_x"] = cData.CamProp.video_res_x;
	jsonReader["Camera_Interface"]["video_res_y"] = cData.CamProp.video_res_y;
	jsonReader["Camera_Interface"]["video_roi_x"] = cData.CamProp.video_roi_x;
	jsonReader["Camera_Interface"]["video_roi_y"] = cData.CamProp.video_roi_y;
	jsonReader["Camera_Interface"]["video_roi_width"] = cData.CamProp.video_roi_width;
	jsonReader["Camera_Interface"]["video_roi_height"] = cData.CamProp.video_roi_height;
	jsonReader["Camera_Interface"]["fps"] = cData.CamProp.fps;
	jsonReader["Camera_Interface"]["exposure"] = cData.CamProp.exposure;
	jsonReader["Camera_Interface"]["analog_gain"] = cData.CamProp.analog_gain;
	jsonReader["Camera_Interface"]["digital_gain"] = cData.CamProp.digital_gain;
	jsonReader["Camera_Interface"]["stream_exposure"] = cData.CamProp.stream_exposure;
	jsonReader["Camera_Interface"]["stream_analog_gain"] = cData.CamProp.stream_analog_gain;
	jsonReader["Camera_Interface"]["stream_digital_gain"] = cData.CamProp.stream_digital_gain;
	jsonReader["Camera_Interface"]["flash_on"] = cData.CamProp.flash_on;
	
	return 0;
}

int CalibHandle::getUpdatedCalibData(CalibData& val) {
	val = cData;
	return 0;
}

int CalibHandle::updateCalibData(const CalibData &val) {
	cData = val;
	return 0;
}
