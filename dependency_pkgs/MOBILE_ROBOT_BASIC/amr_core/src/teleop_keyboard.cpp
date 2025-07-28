#include "teleop_keyboard/teleop_keyboard.h"

std::unordered_map<char, std::vector<float>> moveBindings = {
    {'i', {1, 0, 0, 0}}, {'o', {1, 0, 0, -1}}, {'j', {0, 0, 0, 1}}, {'l', {0, 0, 0, -1}},
    {'u', {1, 0, 0, 1}}, {',', {-1, 0, 0, 0}}, {'.', {-1, 0, 0, 1}}, {'m', {-1, 0, 0, -1}}};

std::unordered_map<char, std::vector<float>> speedBindings = {
    {'q', {1.1, 1.1}}, {'z', {0.9, 0.9}}, {'w', {1.1, 1}}, {'x', {0.9, 1}}, {'e', {1, 1.1}}, {'c', {1, 0.9}}};

const char *msg = R"(
===========================
 Teleop Keyboard Control   
===========================

Control your robot with the following keys:

   u    i    o
   j    k    l
   m    ,    .

Moving around:
---------------------------------------
   i : Move forward
   , : Move backward
   j : Turn left
   l : Turn right
   u : Move forward + Turn left
   o : Move forward + Turn right
   m : Move backward + Turn right
   . : Move backward + Turn left
   k : Stop (reset movement)

Speed control:
---------------------------------------
   q : Increase speed (linear & angular)
   z : Decrease speed (linear & angular)
   w : Increase linear speed only
   x : Decrease linear speed only
   e : Increase angular speed only
   c : Decrease angular speed only

Exit:
---------------------------------------
   Ctrl + C : Quit teleoperation

---------------------------------------
)";

TeleopKeyboard::TeleopKeyboard()
    : Node("teleop_keyboard"), key_(' '), x_(0), y_(0), z_(0), th_(0), state_(State::MANUAL)
{
    getParam();
    
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_name_, 10);

    timer1_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TeleopKeyboard::timer1_callback, this));
    timer2_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / control_hz_)),
                                      std::bind(&TeleopKeyboard::timer2_callback, this));
}

void TeleopKeyboard::getParam()
{
    this->declare_parameter<std::string>("topic_name", "manual_vel");
    this->declare_parameter<double>("speed", 0.3);
    this->declare_parameter<double>("turn", 0.5236);
    this->declare_parameter<double>("control_hz", 10.0);

    this->get_parameter("topic_name", topic_name_);
    this->get_parameter("speed", speed_);
    this->get_parameter("turn", turn_);
    this->get_parameter("control_hz", control_hz_);
}

int TeleopKeyboard::getch()
{
    struct termios oldt, newt;
    int ch = 0;
    fd_set set;
    struct timeval timeout;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000; 

    if (select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout) > 0)
    {
        ch = getchar();
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

void TeleopKeyboard::timer1_callback()
{
    char prev_key = key_;
    key_ = getch(); 
    if (moveBindings.count(key_))
    {
        state_ = State::MANUAL;
        x_ = moveBindings[key_][0];
        y_ = moveBindings[key_][1];
        z_ = moveBindings[key_][2];
        th_ = moveBindings[key_][3];
    }
    else if (speedBindings.count(key_))
    {
        speed_ = std::max(speed_ * speedBindings[key_][0], 0.01);
        turn_ = std::max(turn_ * speedBindings[key_][1], 0.01);
    }
    else if (key_ == 'k') 
    {
        x_ = y_ = z_ = th_ = 0;
    }
    else if (key_ == '\x03')  // Ctrl+C to exit
    {
        rclcpp::shutdown();
    }

    if (key_ != prev_key)
    {
        printf("\rCurrent: speed %.2f | turn %.2f | Press key...  \n", speed_, turn_);
    }
}

void TeleopKeyboard::timer2_callback()
{
    if (state_ == State::MANUAL)
    {
        twist_.linear.x = x_ * speed_;
        twist_.angular.z = th_ * turn_;
        pub_->publish(twist_);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopKeyboard>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
