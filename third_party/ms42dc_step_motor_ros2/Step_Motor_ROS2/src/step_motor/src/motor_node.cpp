#include "rclcpp/rclcpp.hpp"
#include "step_motor/msg/motor.hpp"
#include "serial/serial.h"

class Step_Motor:public rclcpp::Node
{
private:
  uint8_t dir;
  uint8_t sub_divide;
  uint8_t mode;
  std::string usart_port_name;
  uint32_t baudrate;
  rclcpp::Subscription<step_motor::msg::Motor>::SharedPtr sub;
  rclcpp::Publisher<step_motor::msg::Motor>::SharedPtr pub;
  uint16_t speed;
  uint32_t angle;
  uint32_t rcc_check;
public:
  serial::Serial Stm32_Serial;
  uint8_t send_data[11];
  uint8_t recv_data[9];

  uint8_t check_rcc(uint8_t *data,uint8_t num);
  void motor_control_callback(const step_motor::msg::Motor &msg);
  Step_Motor(const std::string &node_name);
};

Step_Motor::Step_Motor(const std::string &node_name):Node(node_name)
{
  this->declare_parameter<std::string>("usart_port_name", "/dev/motor_serial");
  this->get_parameter("usart_port_name", usart_port_name);

  this->declare_parameter("serial_baud_rate",115200);
  this->get_parameter("serial_baud_rate", baudrate);


  pub=this->create_publisher<step_motor::msg::Motor>("motor_state",10);
  sub=this->create_subscription<step_motor::msg::Motor>("motor_control",10,std::bind(&Step_Motor::motor_control_callback,this,std::placeholders::_1));
  try
  { 
    //Attempts to initialize and open the serial port //尝试初始化与开启串口
    Stm32_Serial.setPort(usart_port_name); //Select the serial port number to enable //选择要开启的串口号
    Stm32_Serial.setBaudrate(baudrate); //Set the baud rate //设置波特率
    serial::Timeout _time = serial::Timeout::simpleTimeout(2000); //Timeout //超时等待
    Stm32_Serial.setTimeout(_time);
    Stm32_Serial.open(); //Open the serial port //开启串口
  }
  catch (serial::IOException& e)
  {
    RCLCPP_ERROR(this->get_logger(),"wheeltec_robot can not open serial port,Please check the serial port cable! "); //If opening the serial port fails, an error message is printed //如果开启串口失败，打印错误信息
  }
  if(Stm32_Serial.isOpen())
  {
    RCLCPP_INFO(this->get_logger(),"wheeltec_robot serial port opened"); //Serial port opened successfully //串口开启成功提示
  }
}

void Step_Motor::motor_control_callback(const step_motor::msg::Motor &msg)
{
  RCLCPP_INFO(this->get_logger(),"%d \n",msg.dir); //Serial port opened successfully //串口开启成功提示
  if(!msg.state)
  {
    send_data[0]=0x7b;
    send_data[1]=msg.id;
    send_data[2]=msg.mode;
    mode=msg.mode;
    send_data[3]=msg.dir;
    dir=msg.dir;
    send_data[4]=msg.sub_divide;
    sub_divide=msg.sub_divide;
    send_data[5]=msg.angle>>8;
    send_data[6]=msg.angle;
    send_data[7]=msg.speed>>8;
    send_data[8]=msg.speed;
    send_data[9]=check_rcc(send_data,9);
    send_data[10]=0x7d;
  }
  else
  {
    send_data[0]=0x7b;
    send_data[1]=msg.id;
    send_data[2]=0;
    send_data[3]=0;
    send_data[4]=0;
    send_data[5]=0;
    send_data[6]=0;
    send_data[7]=0;
    send_data[8]=0;
    send_data[9]=check_rcc(send_data,9);
    send_data[10]=0x7d;
  }
  try
  {
    Stm32_Serial.write(send_data,sizeof (send_data)); //Sends data to the downloader via serial port //通过串口向下位机发送数据 
    if(msg.state)
    {
      rclcpp::sleep_for(std::chrono::milliseconds(10));
      Stm32_Serial.read(recv_data,9);
      if(check_rcc(recv_data,8)==recv_data[8])
      {
        step_motor::msg::Motor motor;
        speed=recv_data[2]<<8|recv_data[3];
        angle=recv_data[4]<<24|recv_data[5]<<16|recv_data[6]<<8|recv_data[7];
        motor.id=msg.id;
        motor.angle=angle;
        motor.state=recv_data[1];
        motor.speed=speed;
        motor.dir=dir;
        motor.mode=mode;
        motor.sub_divide=sub_divide;
        pub->publish(motor);
      }
    }
    // for(uint8_t i=0;i<sizeof(send_data);i++)
    // {
    //   printf("%x ",send_data[i]);
    // }


    // for(uint8_t i=0;i<sizeof(recv_data);i++)
    // {
    //   printf("%x ",recv_data[i]);
    // }
    // printf("\n");
  }
  catch (serial::IOException& e)   
  {
    RCLCPP_ERROR(this->get_logger(),("Unable to send data through serial port")); //If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
  }
}

uint8_t Step_Motor::check_rcc(uint8_t *data,uint8_t num)
{
  uint8_t check=0;
  for(uint8_t i=0;i<num;i++)
  {
    check^=data[i];
  }
  return check;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Step_Motor>("motor_node");
  rclcpp::spin(node);  
  rclcpp::shutdown();
}
