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
    
    // Gobal variables
    bool useLSDAlgorithm = true;

    LineLBD():Node("line_lbd"){
      // Constructor
      RCLCPP_INFO(this->get_logger(), "LineLBD Constructor called");
      std::string homeDir = getenv("HOME");
      std::string packagePath = "ros2_test/src/ros2_line_lbd/"; //! HARDCODED
      std::string pathToTestImage = homeDir + "/" + packagePath + "data/cabinet.png";


      // Debug
      RCLCPP_INFO(this->get_logger(), "pathToTestImage %s", pathToTestImage.c_str());
    
    }

    ~LineLBD(){
      // Destructor
      cv::destroyAllWindows(); // Clears all openCV windows
    }

    void runMethod(){
      pass;
    }

  private:

};



int main(int argc, char** argv )
{
    rclcpp::init(argc, argv); // Always the first line, initialize this node

    // A node declaration
    auto node = std::make_shared<LineLBD>();
    // Execution function

    // rclcpp::spin(node); // Blocking node
    rclcpp::shutdown();
    return 0;
  
}
