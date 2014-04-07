// -*- C++ -*-
/*!
 * @file  ImageHough.h
 * @brief ImageHoughLines
 * @date  $Date$
 *
 * $Id$
 */

#ifndef IMAGEHOUGH_H
#define IMAGEHOUGH_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>
// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">
#include "ImgStub.h"
#include "ImgStub.h"

// </rtc-template>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>
#include <opencv2/opencv.hpp>

using namespace RTC;
using namespace Img;

/*!
 * @class ImageHough
 * @brief ImageHoughLines
 *
 */
class ImageHough
  : public RTC::DataFlowComponentBase
{
 public:
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  ImageHough(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~ImageHough();

  // <rtc-template block="public_attribute">
  
  // </rtc-template>

  // <rtc-template block="public_operation">
  
  // </rtc-template>

  /***
   *
   * The initialize action (on CREATED->ALIVE transition)
   * formaer rtc_init_entry() 
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onInitialize();

  /***
   *
   * The finalize action (on ALIVE->END transition)
   * formaer rtc_exiting_entry()
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onFinalize();

  /***
   *
   * The startup action when ExecutionContext startup
   * former rtc_starting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  /***
   *
   * The shutdown action when ExecutionContext stop
   * former rtc_stopping_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  /***
   *
   * The activated action (Active state entry action)
   * former rtc_active_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  /***
   *
   * The deactivated action (Active state exit action)
   * former rtc_active_exit()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  /***
   *
   * The execution action that is invoked periodically
   * former rtc_active_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  /***
   *
   * The aborting action when main logic error occurred.
   * former rtc_aborting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  /***
   *
   * The error action in ERROR state
   * former rtc_error_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  /***
   *
   * The reset action that is invoked resetting
   * This is same but different the former rtc_init_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  /***
   *
   * The state update action that is invoked after onExecute() action
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  /***
   *
   * The action that is invoked when execution context's rate is changed
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // <rtc-template block="protected_attribute">
  
  // </rtc-template>

  // <rtc-template block="protected_operation">
  
  // </rtc-template>

  // Configuration variable declaration
  // <rtc-template block="config_declare">
  /*!
   * 
   * - Name:  mode
   * - DefaultValue: standard
   */
  std::string m_mode;
  /*!
   * Distance resolution of the accumulator in pixels
   * - Name:  rho
   * - DefaultValue: 1.0
   */
  double m_rho;
  /*!
   * Angle resolution of the accumulator in radians
   * - Name:  theta
   * - DefaultValue: 180
   */
  double m_theta;
  /*!
   * The accumulator threshold parameter. Only those lines arereturned that get enough votes ( >
   * \texttt{threshold} )
   * - Name:  threshold
   * - DefaultValue: 100
   */
  double m_threshold;
  /*!
   * For the multi-scale Hough transform it is the divisor for
   * the distance resolution rho .The coarse accumulator distance
   * resolution will be rho and the accurate accumulator
   * resolution will be rho/srn . If both srn=0 and stn=0 then
   * the classical Hough transform is used, otherwise both these
   * parameters should be positive.
   * - Name:  srn
   * - DefaultValue: 0
   */
  double m_srn;
  /*!
   * For the multi-scale Hough transform it is the divisor for
   * the distance resolution theta
   * - Name:  stn
   * - DefaultValue: 0
   */
  double m_stn;

  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  Img::TimedCameraImage m_in;
  /*!
   */
  InPort<Img::TimedCameraImage> m_inIn;
  
  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  Img::TimedCameraImage m_out;
  /*!
   */
  OutPort<Img::TimedCameraImage> m_outOut;
  RTC::TimedDoubleSeq m_lines;
  /*!
   * Lines Point Data
   * format :  Line k is defined by two points : (xk0, yk0) to
   * (xk1, yk1)
   * (if two lines, )
   * [x00, y00, x01, y01, x10, y10, x11, y11]
   */
  OutPort<RTC::TimedDoubleSeq> m_linesOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  // <rtc-template block="private_attribute">
  
  // </rtc-template>

  // <rtc-template block="private_operation">
  
  // </rtc-template>

  cv::Mat m_srcImage;
  //CvMemStorage *m_pStorage;
  std::vector<cv::Vec2f> m_Lines;
  cv::Mat m_grayImage;
};


extern "C"
{
  DLL_EXPORT void ImageHoughInit(RTC::Manager* manager);
};

#endif // IMAGEHOUGH_H
