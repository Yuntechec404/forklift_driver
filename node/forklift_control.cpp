#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <forklift_driver/msg/meteorcar.hpp>

#ifdef _WIN32
#include <conio.h>
#else
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#endif

auto msg = "t : up (+z)\n"
          "b : down (-z)\n"
          "g : stop (z)\n"
          "q/z : increase/decrease max speeds by 10%\n";

// 定義保存終端設置的函數，返回終端設置
termios saveTerminalSettings() {
  termios settings;
  tcgetattr(STDIN_FILENO, &settings);
  return settings;
}

// 定義恢復終端設置的函數，使用傳入的設置進行恢復
void restoreTerminalSettings(termios old_settings) {
  #ifdef _WIN32
    return;
  #else
    tcsetattr(STDIN_FILENO, TCSANOW, &old_settings);
  #endif
}

// 定義獲取按鍵的函數
char getKey(termios settings) {
  #ifdef _WIN32
    // 在 Windows 上獲取按鍵
    return _getch();
  #else
    // 在 Linux 上獲取按鍵
    struct termios new_settings;
    tcgetattr(STDIN_FILENO, &new_settings);
    new_settings.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);

    char key;
    std::cin >> key;

    // 恢復原來的終端設置
    tcsetattr(STDIN_FILENO, TCSANOW, &settings);

    return key;
  #endif
}
termios settings;
class ForkLiftControl : public rclcpp::Node
{
  public:
    
    ForkLiftControl(): Node("teleop_forklift_keyboard")
    {
      publisher_ = this->create_publisher<forklift_driver::msg::Meteorcar>("/cmd_fork", 5);
    }

  private:
    void publish_message()
    {
      auto message = forklift_driver::msg::Meteorcar();
      char key = getKey(settings);
      if(key == 'g' || key == 'G'){
        message.fork_position = 0.0f;
        std::cout << "fork_position" << message.fork_position << std::endl;
      }
      else if(key == 't' || key == 'T'){
        message.fork_position += 10.0f;
        std::cout << "fork_position" << message.fork_position << std::endl;
      }
      else if (key == 'b' || key == 'B')
      {
        message.fork_position -= 10.0f;
        std::cout << "fork_position" << message.fork_position << std::endl;
      }

      if(key == 'q' || key == 'Q'){
        message.fork_velocity *= 1.1f;
      std::cout << "fork_velocity" << message.fork_velocity << std::endl;
      }
      else if(key == 'z' || key == 'Z'){
        message.fork_velocity *= 0.1f;
        std::cout << "fork_velocity" << message.fork_velocity << std::endl;
      }

      publisher_->publish(message);
    }
    // rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<forklift_driver::msg::Meteorcar>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  // 保存終端設置
  settings = saveTerminalSettings();
  std::cout << msg << std::endl;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ForkLiftControl>());

  // 恢復原來的終端設置
  restoreTerminalSettings(settings);
  rclcpp::shutdown();
  return 0;
}