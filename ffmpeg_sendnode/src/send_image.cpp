#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "transdata/transdata.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

Transdata g_TRANSDATA;
std::mutex g_M_IMAGE_BUF;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  image_transport::ImageTransport it(n);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  ros::Rate loop_rate(10);

  int count = 0;
  sensor_msgs::ImagePtr msg;

  if (g_TRANSDATA.transdataInit() < 0)
  {
    cout << "init error !" << endl;
    return -1;
  }

  while (ros::ok())
  {
    // Receive image and display
    g_TRANSDATA.transdataRecdata();

    // You need to use a mutex lock because you need to operate image data
    g_M_IMAGE_BUF.lock();
    if (!g_TRANSDATA.image_test_.empty())
    {
      //            imshow("test",transdata.image_test);
      //            waitKey(10);
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", g_TRANSDATA.image_test_).toImageMsg();
      pub.publish(msg);
      cout << " send image " << count << endl;
      count++;
      g_TRANSDATA.image_test_.release();
    }
    g_M_IMAGE_BUF.unlock();

    ros::spinOnce();
  }
  return 0;
}
