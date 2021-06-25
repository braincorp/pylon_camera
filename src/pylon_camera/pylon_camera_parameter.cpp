/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2016, Magazino GmbH. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Magazino GmbH nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include <pylon_camera/pylon_camera_parameter.h>
#include <sensor_msgs/image_encodings.h>

namespace pylon_camera
{

PylonCameraParameter::PylonCameraParameter() :
        camera_frame_("pylon_camera"),
        device_user_id_(""),
        frame_rate_(5.0),
        camera_info_url_(""),
        image_encoding_(""),
        balance_white_(""),
        pgi_(false),
        noise_reduction_(0.0),
        sharpness_enhancement_(1.0),
        balance_ratio_(false),
        balance_red_(1.0),
        balance_green_(1.0),
        balance_blue_(1.0),
        binning_x_(1),
        binning_y_(1),
        binning_x_given_(false),
        binning_y_given_(false),
        downsampling_factor_exp_search_(1),
        // ##########################
        //  image intensity settings
        // ##########################
        exposure_(10000.0),
        exposure_given_(false),
        gain_(0.5),
        gain_given_(false),
        gamma_(1.0),
        gamma_given_(false),
        brightness_(100),
        brightness_given_(false),
        brightness_continuous_(false),
        exposure_auto_(true),
        gain_auto_(true),
        // #########################
        exposure_search_timeout_(5.),
        auto_exp_upper_lim_(0.0),
        mtu_size_(3000),
        inter_pkg_delay_(1000),
        shutter_mode_(SM_DEFAULT),
        auto_flash_(false)
{}

PylonCameraParameter::~PylonCameraParameter()
{}

void PylonCameraParameter::readFromRosParameterServer(const ros::NodeHandle& nh)
{
    nh.param<std::string>("camera_frame", camera_frame_, "pylon_camera");

    nh.param<std::string>("device_user_id", device_user_id_, "");
    std::cout << "device_user_id is " << device_user_id_ << std::endl;

    if ( nh.hasParam("frame_rate") )
    {
        nh.getParam("frame_rate", frame_rate_);
        std::cout << "frame_rate is given and has value " << frame_rate_ << std::endl;
    }

    nh.param<std::string>("camera_info_url", camera_info_url_, "");
    if ( nh.hasParam("camera_info_url") )
    {
        nh.getParam("camera_info_url", camera_info_url_);
        std::cout << "camera_info_url is given and has value " << camera_info_url_ << std::endl;
    }

    binning_x_given_ = nh.hasParam("binning_x");
    if ( binning_x_given_ )
    {
        int binning_x;
        nh.getParam("binning_x", binning_x);
        std::cout << "binning x is given and has value " << binning_x << std::endl;
        if ( binning_x > 32 || binning_x < 0 )
        {
            ROS_WARN_STREAM("Desired horizontal binning_x factor not in valid "
                << "range! Binning x = " << binning_x << ". Will reset it to "
                << "default value (1)");
            binning_x_given_ = false;
        }
        else
        {
            binning_x_ = static_cast<size_t>(binning_x);
        }
    }
    binning_y_given_ = nh.hasParam("binning_y");
    if ( binning_y_given_ )
    {
        int binning_y;
        nh.getParam("binning_y", binning_y);
        std::cout << "binning y is given and has value " << binning_y << std::endl;
        if ( binning_y > 32 || binning_y < 0 )
        {
            ROS_WARN_STREAM("Desired vertical binning_y factor not in valid "
                << "range! Binning y = " << binning_y << ". Will reset it to "
                << "default value (1)");
            binning_y_given_ = false;
        }
        else
        {
            binning_y_ = static_cast<size_t>(binning_y);
        }
    }
    nh.param<int>("downsampling_factor_exposure_search",
                  downsampling_factor_exp_search_,
                  20);

    if ( nh.hasParam("image_encoding") )
    {
        std::string encoding;
        nh.getParam("image_encoding", encoding);
        std::cout << "image_encoding is given and has value " << image_encoding_ << std::endl;
        if ( !encoding.empty() &&
             !sensor_msgs::image_encodings::isMono(encoding) &&
             !sensor_msgs::image_encodings::isColor(encoding) &&
             !sensor_msgs::image_encodings::isBayer(encoding) &&
             encoding != sensor_msgs::image_encodings::YUV422 )
        {
            ROS_WARN_STREAM("Desired image encoding parameter: '" << encoding
                << "' is not part of the 'sensor_msgs/image_encodings.h' list!"
                << " Will not set encoding");
            encoding = std::string("");
        }
        image_encoding_ = encoding;
    }

    if ( nh.hasParam("balance_white") )
    {
        nh.getParam("balance_white", balance_white_);
        std::cout << "balance white is given and has value " << balance_white_ << std::endl;
    }

    if ( nh.hasParam("pgi") )
    {
        nh.getParam("pgi", pgi_);
        std::cout << "PGI is on" << std::endl;

        if ( nh.hasParam("noise_reduction") )
        {
            nh.getParam("noise_reduction", noise_reduction_);
            std::cout << "noise reduction set to " << noise_reduction_ << std::endl;
        }

        if ( nh.hasParam("sharpness_enhancement") )
        {
            nh.getParam("sharpness_enhancement", sharpness_enhancement_);
            std::cout << "sharpness enhancement set to " << sharpness_enhancement_ << std::endl;
        }
    }
    
    if ( nh.hasParam("balance_ratio") )
    {
        nh.getParam("balance_ratio", balance_ratio_);
        std::cout << "Setting white balance channels individually" << std::endl;
        
        if ( nh.hasParam("balance_red") )
        {
            nh.getParam("balance_red", balance_red_);
            if (balance_red_ < 0 || balance_red_ > 16)
            {
                ROS_WARN_STREAM("Desired red balance " << balance_red_
                    << "not in range! Will reset it to 1.0");
            }
            std::cout << "White balance red channel is " << balance_red_ << std::endl;
        }
        
        if ( nh.hasParam("balance_green") )
        {
            nh.getParam("balance_green", balance_green_);
            if (balance_green_ < 0 || balance_green_ > 16)
            {
                ROS_WARN_STREAM("Desired green balance " << balance_green_
                    << "not in range! Will reset it to 1.0");
            }
            std::cout << "White balance green channel is " << balance_green_ << std::endl;
        }
        
        if ( nh.hasParam("balance_blue") )
        {
            nh.getParam("balance_blue", balance_blue_);
            if (balance_blue_ < 0 || balance_blue_ > 16)
            {
                ROS_WARN_STREAM("Desired blue balance " << balance_blue_
                    << "not in range! Will reset it to 1.0");
            }
            std::cout << "White balance blue channel is " << balance_blue_ << std::endl;
        }
    }

    // ##########################
    //  image intensity settings
    // ##########################

    // > 0: Exposure time in microseconds
    exposure_given_ = nh.hasParam("exposure");
    if ( exposure_given_ )
    {
        nh.getParam("exposure", exposure_);
        std::cout << "exposure is given and has value " << exposure_ << std::endl;
    }

    gain_given_ = nh.hasParam("gain");
    if ( gain_given_ )
    {
        nh.getParam("gain", gain_);
        std::cout << "gain is given and has value " << gain_ << std::endl;
    }

    gamma_given_ = nh.hasParam("gamma");
    if ( gamma_given_ )
    {
        nh.getParam("gamma", gamma_);
        std::cout << "gamma is given and has value " << gamma_ << std::endl;
    }

    brightness_given_ = nh.hasParam("brightness");
    if ( brightness_given_ )
    {
        nh.getParam("brightness", brightness_);
        std::cout << "brightness is given and has value " << brightness_
            << std::endl;
        if ( gain_given_ && exposure_given_ )
        {
            ROS_WARN_STREAM("Gain ('gain') and Exposure Time ('exposure') "
                << "are given as startup ros-parameter and hence assumed to be "
                << "fix! The desired brightness (" << brightness_ << ") can't "
                << "be reached! Will ignore the brightness by only "
                << "setting gain and exposure . . .");
            brightness_given_ = false;
        }
        else
        {
            if ( nh.hasParam("brightness_continuous") )
            {
                nh.getParam("brightness_continuous", brightness_continuous_);
                std::cout << "brightness is continuous" << std::endl;
            }
            if ( nh.hasParam("exposure_auto") )
            {
                nh.getParam("exposure_auto", exposure_auto_);
                std::cout << "exposure is set to auto" << std::endl;
            }
            if ( nh.hasParam("gain_auto") )
            {
                nh.getParam("gain_auto", gain_auto_);
                std::cout << "gain is set to auto" << std::endl;
            }
        }
    }
    // ##########################

    nh.param<double>("exposure_search_timeout", exposure_search_timeout_, 5.);
    nh.param<double>("auto_exposure_upper_limit", auto_exp_upper_lim_, 10000000.);

    if ( nh.hasParam("gige/mtu_size") )
    {
        nh.getParam("gige/mtu_size", mtu_size_);
        std::cout << "gige/mtu_size is given and has value " << mtu_size_ << std::endl;
    }

    if ( nh.hasParam("gige/inter_pkg_delay") )
    {
        nh.getParam("gige/inter_pkg_delay", inter_pkg_delay_);
        std::cout << "gige/inter_pkg_delay is given and has value " << inter_pkg_delay_ << std::endl;
    }

    std::string shutter_param_string;
    nh.param<std::string>("shutter_mode", shutter_param_string, "");
    if ( shutter_param_string == "rolling" )
    {
        shutter_mode_ = SM_ROLLING;
    }
    else if ( shutter_param_string == "global" )
    {
        shutter_mode_ = SM_GLOBAL;
    }
    else if ( shutter_param_string == "global_reset" )
    {
        shutter_mode_ = SM_GLOBAL_RESET_RELEASE;
    }
    else
    {
        shutter_mode_ = SM_DEFAULT;
    }

    nh.param<bool>("auto_flash", auto_flash_, false);
    nh.param<bool>("auto_flash_line_2", auto_flash_line_2_, true);
    nh.param<bool>("auto_flash_line_3", auto_flash_line_3_, true);

    ROS_WARN("Autoflash: %i, line2: %i , line3: %i ", auto_flash_, auto_flash_line_2_, auto_flash_line_3_);
    validateParameterSet(nh);
    std::cout << "camera_frame: " << camera_frame_ << "\n"
              << "device_user_id: " << device_user_id_ << "\n"
              << "frame_rate: " << frame_rate_ << "\n"
              << "camera_info_url: " << camera_info_url_ << "\n"
              << "image_encoding: " << image_encoding_ << "\n"
              << "balance_white: " << balance_white_ << "\n"
              << "noise_reduction: " << noise_reduction_ << "\n"
              << "sharpness_enhancement: " << sharpness_enhancement_ << "\n"
              << "balance_ratio: " << balance_ratio_ << "\n"
              << "balance_red: " << balance_red_ << "\n"
              << "balance_green: " << balance_green_ << "\n"
              << "balance_blue: " << balance_blue_ << "\n"
              << "binning_x: " << binning_x_ << "\n"
              << "binning_y: " << binning_y_ << "\n"
              << "binning_x_given: " << binning_x_given_ << "\n"
              << "binning_y_given: " << binning_y_given_ << "\n"
              << "downsampling_factor_exp_search: " << downsampling_factor_exp_search_ << "\n"
              << "exposure: " << exposure_ << "\n"
              << "exposure_given: " << exposure_given_ << "\n"
              << "gain: " << gain_ << "\n"
              << "gain_given: " << gain_given_ << "\n"
              << "gamma: " << gamma_ << "\n"
              << "gamma_given: " << gamma_given_ << "\n"
              << "brightness: " << brightness_ << "\n"
              << "brightness_given: " << brightness_given_ << "\n"
              << "brightness_continuous: " << brightness_continuous_ << "\n"
              << "exposure_auto: " << exposure_auto_ << "\n"
              << "gain_auto: " << gain_auto_ << "\n"
              << "exposure_search_timeout: " << exposure_search_timeout_ << "\n"
              << "auto_exp_upper_lim: " << auto_exp_upper_lim_ << "\n"
              << "mtu_size: " << mtu_size_ << "\n"
              << "inter_pkg_delay: " << inter_pkg_delay_ << "\n"
              << "shutter_mode: " << shutter_mode_ << "\n"
              << "auto_flash: " << auto_flash_ << std::endl;
    return;
}

void PylonCameraParameter::adaptDeviceUserId(const ros::NodeHandle& nh, const std::string& device_user_id)
{
    device_user_id_ = device_user_id;
    nh.setParam("device_user_id", device_user_id_);
}

void PylonCameraParameter::validateParameterSet(const ros::NodeHandle& nh)
{
    if ( !device_user_id_.empty() )
    {
        ROS_INFO_STREAM("Trying to open the following camera: "
            << device_user_id_.c_str());
    }
    else
    {
        ROS_INFO_STREAM("No Device User ID set -> Will open the camera device "
                << "found first");
    }

    if ( frame_rate_ < 0 && frame_rate_ != -1 )
    {
        ROS_WARN_STREAM("Unexpected frame rate (" << frame_rate_ << "). Will "
                << "reset it to default value which is 5 Hz");
        frame_rate_ = 5.0;
        nh.setParam("frame_rate", frame_rate_);
    }

    if ( exposure_given_ && ( exposure_ <= 0.0 || exposure_ > 1e7 ) )
    {
        ROS_WARN_STREAM("Desired exposure measured in microseconds not in "
                << "valid range! Exposure time = " << exposure_ << ". Will "
                << "reset it to default value!");
        exposure_given_ = false;
    }

    if ( gain_given_ && ( gain_ < 0.0 || gain_ > 1.0 ) )
    {
        ROS_WARN_STREAM("Desired gain (in percent) not in allowed range! "
                << "Gain = " << gain_ << ". Will reset it to default value!");
        gain_given_ = false;
    }

    if ( brightness_given_ && ( brightness_ < 0.0 || brightness_ > 255 ) )
    {
        ROS_WARN_STREAM("Desired brightness not in allowed range [0 - 255]! "
               << "Brightness = " << brightness_ << ". Will reset it to "
               << "default value!");
        brightness_given_ = false;
    }

    if ( exposure_search_timeout_ < 5.)
    {
        ROS_WARN_STREAM("Low timeout for exposure search detected! Exposure "
            << "search may fail.");
    }
    return;
}

const std::string& PylonCameraParameter::deviceUserID() const
{
    return device_user_id_;
}

std::string PylonCameraParameter::shutterModeString() const
{
    if ( shutter_mode_ == SM_ROLLING )
    {
        return "rolling";
    }
    else if ( shutter_mode_ == SM_GLOBAL )
    {
        return "global";
    }
    else if ( shutter_mode_ == SM_GLOBAL_RESET_RELEASE )
    {
        return "global_reset";
    }
    else
    {
        return "default_shutter_mode";
    }
}

const std::string& PylonCameraParameter::imageEncoding() const
{
    return image_encoding_;
}

const std::string& PylonCameraParameter::balanceWhite() const
{
    return balance_white_;
}

const std::string& PylonCameraParameter::cameraFrame() const
{
    return camera_frame_;
}

const double& PylonCameraParameter::frameRate() const
{
    return frame_rate_;
}

void PylonCameraParameter::setFrameRate(const ros::NodeHandle& nh,
                                        const double& frame_rate)
{
    frame_rate_ = frame_rate;
    nh.setParam("frame_rate", frame_rate_);
}

const std::string& PylonCameraParameter::cameraInfoURL() const
{
    return camera_info_url_;
}

void PylonCameraParameter::setCameraInfoURL(const ros::NodeHandle& nh,
                                            const std::string& camera_info_url)
{
    camera_info_url_ = camera_info_url;
    nh.setParam("camera_info_url", camera_info_url_);
}

}  // namespace pylon_camera
