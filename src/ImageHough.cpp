// -*- C++ -*-
/*!
 * @file  ImageHough.cpp
 * @brief ImageHoughLines
 * @date $Date$
 *
 * $Id$
 */

#include "ImageHough.h"

enum IMAGE_FORMAT {
  FMT_GRAY,
  FMT_RGB,
  FMT_JPEG,
  FMT_PNG,
};

bool convertCvMatToImg(const cv::Mat& srcImage, Img::CameraImage& dstImage, const IMAGE_FORMAT outFormat, const int compression_ratio=75) {
  int width = srcImage.cols;
  int height = srcImage.rows;
  int depth = srcImage.depth();
  int inChannels = srcImage.channels();
  int outChannels = outFormat == FMT_GRAY ? 1 : 3;

  cv::Mat procImage;
  if (outChannels > inChannels) {
    cv::cvtColor(srcImage, procImage, CV_GRAY2RGB);
  } else if (outChannels < inChannels) {
    cv::cvtColor(srcImage, procImage, CV_RGB2GRAY);
  } else {
    procImage = srcImage;
  }

  dstImage.image.width = width;
  dstImage.image.height = height;

  switch(outFormat) {
  case FMT_RGB:
    dstImage.image.format = Img::CF_RGB;
    dstImage.image.raw_data.length( width * height * outChannels );
    for(int i = 0;i < height;i++) {
      memcpy(&(dstImage.image.raw_data[i * width * outChannels]),
	     &(procImage.data[i * procImage.step]),
	     width * outChannels);
    }
    break;
  case FMT_JPEG:
    {
      dstImage.image.format = Img::CF_JPEG;
      //Jpeg encoding using OpenCV image compression function
      std::vector<int> compression_param = std::vector<int>(2); 
      compression_param[0] = CV_IMWRITE_JPEG_QUALITY;
      compression_param[1] = compression_ratio;
      //Encode raw image data to jpeg data
      std::vector<uchar> compressed_image;
      cv::imencode(".jpg", procImage, compressed_image, compression_param);
      //Copy encoded jpeg data to Outport Buffer
      dstImage.image.raw_data.length(compressed_image.size());
      memcpy(&dstImage.image.raw_data[0], &compressed_image[0], sizeof(unsigned char) * compressed_image.size());
    }
    break;
  case FMT_PNG:
    {
      dstImage.image.format = Img::CF_PNG;
      //Jpeg encoding using OpenCV image compression function
      std::vector<int> compression_param = std::vector<int>(2); 
      compression_param[0] = CV_IMWRITE_PNG_COMPRESSION;
      compression_param[1] = (int)((double)compression_ratio/10.0);
      if(compression_param[1] == 10)
	compression_param[1] = 9;
      //Encode raw image data to jpeg data
      std::vector<uchar> compressed_image;
      cv::imencode(".png", procImage, compressed_image, compression_param);
      //Copy encoded jpeg data to Outport Buffer
      dstImage.image.raw_data.length(compressed_image.size());
      memcpy(&dstImage.image.raw_data[0], &compressed_image[0], sizeof(unsigned char) * compressed_image.size());
    }
    break;
  case FMT_GRAY:
    {
      dstImage.image.format = Img::CF_GRAY;
      dstImage.image.raw_data.length( width * height * outChannels);
      for(int i(0); i< height; ++i) {
	memcpy(&(dstImage.image.raw_data[i * width * outChannels]),
	       &(procImage.data[i * procImage.step]),
	       width * outChannels);
      }
    }
    break;
  default:
    return false;
  }
  return  true;
}

bool convertImgToCvMat(const Img::CameraImage& srcImage, cv::Mat& dstImage) {
  int channels = 1;
  int width = srcImage.image.width;
  int height = srcImage.image.height;
  int format = srcImage.image.format;
  int data_length = srcImage.image.raw_data.length();
  int image_size = width * height * channels;
  
  switch(format) {
  case Img::CF_GRAY:
    channels = 1;
    break;
  case Img::CF_RGB:
  case Img::CF_PNG:
  case Img::CF_JPEG:
    channels = 3;
    break;
  default:
    channels = (srcImage.image.raw_data.length()/width/height);
  }
  
  if (channels == 3) {
    dstImage.create(height, width, CV_8UC3);
  } else {
    dstImage.create(height, width, CV_8UC1);
  }
  
  switch(format) {
  case Img::CF_RGB:
    {
      for(int i = 0; i < height;i++) {
	memcpy(&dstImage.data[i*dstImage.step], 
	       &srcImage.image.raw_data[i*width*channels],
	       sizeof(unsigned char)*width*channels);
      }
      if(channels == 3) {
	//cv::cvtColor(dstImage, dstImage, CV_RGB2BGR);
      }
    }
    break;
  case Img::CF_JPEG:
  case Img::CF_PNG:
    {
      std::vector<uchar> compressed_image(data_length);
      memcpy(&compressed_image[0], &srcImage.image.raw_data[0], sizeof(unsigned char) * data_length);
      
      //Decode received compressed image
      cv::Mat decoded_image;
      if(channels == 3) {
	decoded_image = cv::imdecode(cv::Mat(compressed_image), CV_LOAD_IMAGE_COLOR);
	//cv::cvtColor(decoded_image, dstImage, CV_RGB2BGR);
      }
      else {
	decoded_image = cv::imdecode(cv::Mat(compressed_image), CV_LOAD_IMAGE_GRAYSCALE);
	dstImage = decoded_image;
      }
    }
    break;
  default:
    return false;
  }
  return true;
}


// Module specification
// <rtc-template block="module_spec">
static const char* imagehough_spec[] =
  {
    "implementation_id", "ImageHough",
    "type_name",         "ImageHough",
    "description",       "ImageHoughLines",
    "version",           "1.0.0",
    "vendor",            "Sugar Sweet Robotics",
    "category",          "Experimental",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.mode", "standard",
    "conf.default.rho", "1.0",
    "conf.default.theta", "180",
    "conf.default.threshold", "100",
    "conf.default.srn", "0",
    "conf.default.stn", "0",
    // Widget
    "conf.__widget__.mode", "spin",
    "conf.__widget__.rho", "text",
    "conf.__widget__.theta", "text",
    "conf.__widget__.threshold", "text",
    "conf.__widget__.srn", "text",
    "conf.__widget__.stn", "text",
    // Constraints
    "conf.__constraints__.mode", "standard,probabilistic",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
ImageHough::ImageHough(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_inIn("in", m_in),
    m_outOut("out", m_out),
    m_linesOut("lines", m_lines)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
ImageHough::~ImageHough()
{
}



RTC::ReturnCode_t ImageHough::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("in", m_inIn);
  
  // Set OutPort buffer
  addOutPort("out", m_outOut);
  addOutPort("lines", m_linesOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("mode", m_mode, "standard");
  bindParameter("rho", m_rho, "1.0");
  bindParameter("theta", m_theta, "180");
  bindParameter("threshold", m_threshold, "100");
  bindParameter("srn", m_srn, "0");
  bindParameter("stn", m_stn, "0");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ImageHough::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ImageHough::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ImageHough::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t ImageHough::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t ImageHough::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t ImageHough::onExecute(RTC::UniqueId ec_id)
{
  if (m_inIn.isNew()) {
    m_inIn.read();
    if (convertImgToCvMat(m_in.data, m_srcImage)) {
      if (m_srcImage.channels() == 3) {
	cv::cvtColor(m_srcImage, m_grayImage, CV_RGB2GRAY);
	cv::Canny(m_grayImage, m_grayImage, 50, 200, 3);
      } else {
	cv::Canny(m_srcImage, m_grayImage, 50, 200, 3);	
      }
      if (m_mode == "standard") {
	cv::HoughLines(m_grayImage, m_Lines, m_rho, CV_PI/m_theta, m_threshold, m_srn, m_stn);
	
	m_lines.data.length(m_Lines.size()*4);
	for(size_t i = 0;i < m_Lines.size();i++) {
	  float rho = m_Lines[i][0];
	  float theta = m_Lines[i][1];
	  double a = cos(theta), b = sin(theta);
	  double x0 = a*rho, y0 = b*rho;
	  double x1 = cvRound(x0 + 1000*(-b));
	  double y1 = cvRound(y0 + 1000*(a));
	  double x2 = cvRound(x0 - 1000*(-b));
	  double y2 = cvRound(y0 - 1000*(a));
	  cv::Point pt1(x1, y1);
	  cv::Point pt2(x2, y2);
	  m_lines.data[i+0] = x1;
	  m_lines.data[i+1] = y1;
	  m_lines.data[i+2] = x2;
	  m_lines.data[i+3] = y2;
	  cv::line(m_srcImage, pt1, pt2, cv::Scalar(0,0,255), 3, 8);
	  m_linesOut.write();
	}
      }
      convertCvMatToImg(m_srcImage, m_out.data, FMT_RGB);
      setTimestamp(m_out);
      m_out.data.captured_time = m_out.tm;
      m_outOut.write();
      
    } else {
      RTC_DEBUG(("Failed To Decode Image."));
      return RTC::RTC_ERROR;
    }
  }
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ImageHough::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ImageHough::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ImageHough::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ImageHough::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ImageHough::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void ImageHoughInit(RTC::Manager* manager)
  {
    coil::Properties profile(imagehough_spec);
    manager->registerFactory(profile,
                             RTC::Create<ImageHough>,
                             RTC::Delete<ImageHough>);
  }
  
};


