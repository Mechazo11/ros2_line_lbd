/*
 * line_detection interface
 * Originally by Shichao Yang,2018, Carnegie Mellon University
 * Email: shichaoy@andrew.cmu.edu
 * ROS2 modified by Azmyin Md. Kamal
 * Date: 01/17/24
 * Email: azmyin12@gmail.com
 */

// C++ inclues
#include <iostream>
#include <fstream>
#include <ctime>

// Includes from this package
#include "ros2_line_lbd/line_descriptor.hpp"

//* OpenCV includes
// #include "opencv2/core/utility.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

// TODO: One liner descriptor for this .h file
#include "ros2_line_lbd/line_lbd_allclass.h"

//* ROS2 includes
#include "rclcpp/rclcpp.hpp"

#define pass (void)0 // Python's equivalent of "pass" i.e. no operation

// I don't support using namespaces even for simpler code. There is high chance of causing confusion when the code scales up
// using namespace cv;
// using namespace std;

//* Class definition
class LineLBD : public rclcpp::Node{
  public:
    
    // Gobal variables and settings
    bool useLSDAlgorithm = true;
    int numOfOctave = 1;
    float octaveRatio = 2.0;  
    cv::Mat raw_img; // Variable to hold a raw image

    LineLBD():Node("line_lbd"){
      // Constructor
      RCLCPP_INFO(this->get_logger(), "LineLBD Constructor called");
      std::string homeDir = getenv("HOME");
      std::string packagePath = "ros2_test/src/ros2_line_lbd/"; //! HARDCODED
      std::string pathToTestImage = homeDir + "/" + packagePath + "data/cabinet.png";

      raw_img = cv::imread(pathToTestImage, 1); // Read image in color

      // Debug
      // RCLCPP_INFO(this->get_logger(), "pathToTestImage %s", pathToTestImage.c_str());
      // Initialization complete
    
    }

    ~LineLBD(){
      // Destructor
    }

    void doLineDetection(){
      //* Runs once
      // Declare an object to use the line_lbd_lib library within this package
      line_lbd_detect* lineLbdPtr = new line_lbd_detect(numOfOctave,octaveRatio); 
      lineLbdPtr->use_LSD = useLSDAlgorithm;
      lineLbdPtr->line_length_thres = 15;  // remove short edges
      
      
      //* Code mostly same as Dr. Yang`s code from his ROS1 package
      cv::Mat out_edges;
      std::vector< KeyLine> keylines_raw,keylines_out;
      lineLbdPtr->detect_raw_lines(this->raw_img,keylines_raw);
      lineLbdPtr->filter_lines(keylines_raw,keylines_out);  // remove short lines

      // show image
      if( this->raw_img.channels() == 1 ){
        cv::cvtColor(this->raw_img, this->raw_img, cv::COLOR_GRAY2BGR);
      }
        
      cv::Mat raw_img_cp;
      drawKeylines(this->raw_img, keylines_out, raw_img_cp, cv::Scalar( 0, 150, 0 ),2); //BGR, drawKeylines is part of cv::line_descriptor method
      cv::imshow( "Line detector", raw_img_cp );
      cv::waitKey();

    }

};



int main(int argc, char** argv )
{
    rclcpp::init(argc, argv); // Always the first line, initialize this node

    // A node declaration
    auto node = std::make_shared<LineLBD>();
    node->doLineDetection();
    rclcpp::spin(node); // Blocking node
    
    cv::destroyAllWindows(); // Clears all openCV windows
    rclcpp::shutdown();
    return 0;
  
}
