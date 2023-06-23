#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "cv_bridge/cv_bridge.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>


class ImageSubscriber : public rclcpp::Node {
public:
  ImageSubscriber() : Node("image_subscriber"){
    clientSocket = socket(AF_INET, SOCK_DGRAM, 0);
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(8080);  // Replace PORT_NUMBER with the desired port
    serverAddr.sin_addr.s_addr = inet_addr("server_ip_address");  // Replace SERVER_IP_ADDRESS with the desired server IP address
    connect(clientSocket, (struct sockaddr *)&serverAddr, sizeof(serverAddr));

    // Subscribe to the /camera/color/image_raw topic
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/color/image_raw", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr image) {
        cv_bridge::CvImagePtr cvImagePtr;
        try{
          cvImagePtr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        }catch (cv_bridge::Exception& e){
          // Handle the exception if the conversion fails
          std::cout<<"cv_bridge exception: " << e.what() << '\n';
          return;
        }
        cv::Mat cvImage = cvImagePtr->image;
        sendImage(cvImage);
	std::cout<<"Image sended\n";
      });
  }

  void sendImage(const cv::Mat& image) {
        std::vector<uchar> buffer;
        cv::imencode(".jpg", image, buffer);

        // Send the image data
        send(clientSocket, buffer.data(), buffer.size(), 0);
  }

  ~ImageSubscriber() {
        // Close the socket
        close(clientSocket);
  }

private:
  int clientSocket;
  struct sockaddr_in serverAddr;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
