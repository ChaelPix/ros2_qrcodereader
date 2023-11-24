#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>

#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <fcntl.h>
#include <unistd.h>

#include <cmath>
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"


class NavDuBled : public rclcpp::Node {
public:
 enum class ActualState {Init, GoToFirstWall, TurnT1Right, StopAtQr, MoveRToQr, WaitQr, 
 BackwardT1, TurningCube5Right, AligningWithCube, GoingC5, CloseC5, BackwardC5, LeavingC5,
 ExitT1Right, EnterT3, EnterT3Left, StopT3, OpenT3C5Claw, ExitT3, BackSquare,
 END};

    NavDuBled() : Node("robot_controller") {

        // Initialiser les publishers pour chaque roue
        wheel_pub_[0] = this->create_publisher<std_msgs::msg::Int16>("/drivetrain/cmd_mot0", 10);
        wheel_pub_[1] = this->create_publisher<std_msgs::msg::Int16>("/drivetrain/cmd_mot3", 10);
        wheel_pub_[2] = this->create_publisher<std_msgs::msg::Int16>("/drivetrain/cmd_mot2", 10);

        servo_pub = this->create_publisher<std_msgs::msg::Float32>("/drivetrain/servo0_17_angle", 10);

        //Subscriber
        ussSub = this->create_subscription<sensor_msgs::msg::Range>(
            "/drivetrain/uss_1", 10, std::bind(&NavDuBled::UpdateUSS1, this, std::placeholders::_1));

        ussSub1 = this->create_subscription<sensor_msgs::msg::Range>(
            "/drivetrain/uss_0", 10, std::bind(&NavDuBled::UpdateUSS, this, std::placeholders::_1));

        enc_sub[0] = this->create_subscription<std_msgs::msg::Int32>(
            "/drivetrain/encoder_0", 10, std::bind(&NavDuBled::updateEncoders1, this, std::placeholders::_1));
        enc_sub[1] = this->create_subscription<std_msgs::msg::Int32>(
            "/drivetrain/encoder_3", 10, std::bind(&NavDuBled::updateEncoders2, this, std::placeholders::_1));
        enc_sub[2] = this->create_subscription<std_msgs::msg::Int32>(
            "/drivetrain/encoder_2", 10, std::bind(&NavDuBled::updateEncoders3, this, std::placeholders::_1));

        gyroSub = this->create_subscription<sensor_msgs::msg::Imu>(
            "/drivetrain/xnav_data", 10, std::bind(&NavDuBled::updateGyron, this, std::placeholders::_1));

        // Créer un timer pour vérifier les événements de la souris régulièrement
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(25),
            std::bind(&NavDuBled::machineState, this));
    }

    ~NavDuBled() {
    }

    double quaternionToVector3(const geometry_msgs::msg::Quaternion& quaternion) {

            double z = quaternion.z;
            double w = quaternion.w;
            double yaw = atan2(2.0 * (w * z), 1.0 - 2.0 * (z * z));
            return yaw;
    }

    void UpdateUSS(const sensor_msgs::msg::Range range)
    {
        this->distanceFromWall = range.range;
    }

    void UpdateUSS1(const sensor_msgs::msg::Range range)
    {
        this->distanceFromWallBehind = range.range;
    }

    void updateEncoders1(const std_msgs::msg::Int32 enc)
    {
        encs[0] = enc.data;
    }

    void updateEncoders2(const std_msgs::msg::Int32 enc)
    {
         encs[1] = enc.data;
    }

    void updateEncoders3(const std_msgs::msg::Int32 enc)
    {
         encs[2] = enc.data;
    }
    
    void updateGyron(const sensor_msgs::msg::Imu imu)
    {
         gyroVector = quaternionToVector3(imu.orientation);
    }

    void sendToMotors(int mspeed)
    {
        std_msgs::msg::Int16 cmd;
        cmd.data = mspeed; 

        // Envoyer les commandes aux roues
        for (int i = 0; i < 3; ++i) {
            wheel_pub_[i]->publish(cmd);
        }
    }

#pragma region Gyro
    double gyroLeft = 0.91;
    double gyroRight = -2.18;
    double gyroUp = -0.6;
    double gyroUpLeft = 0.159;
    double gyroDown = 2.54;
    double gyroTolerence = 0.1;

    double gyroTarget;
    double gyroNormalize = 10;
    void SetupOrientationGoal(std::string dir)
    {
        if(dir == "left")
        {
            gyroTarget = gyroLeft;
        } else if(dir == "right")
        {
            gyroTarget = gyroRight;
        }else if(dir == "up")
        {
            gyroTarget = gyroUp;
        }else if(dir == "down")
        {
            gyroTarget = gyroDown;
        }
        else if(dir == "upLeft")
        {
            gyroTarget = gyroUpLeft;
        }
    }
    bool isOrientationOk()
    {
        double y = gyroVector + gyroNormalize;
        double t = gyroTarget + gyroNormalize;
        
        std::string log = "y : " + std::to_string(y) + " tgr : " + std::to_string(t)
        + " tol : " + std::to_string(t + gyroTolerence );
 RCLCPP_INFO(this->get_logger(), log.c_str());

        return (y < t + gyroTolerence 
        &&      y > t - gyroTolerence);
            
    }
   #pragma endregion 
   
    double w1Tick, w2Tick;
    bool updateTicks = false;

void GoForward()
{
    // if(updateTicks)
    // {
    //     w1Tick = encs[1];
    //     w2Tick = encs[0];
    //     updateTicks = false;
    // }

    // double diffTick1 = abs(w1Tick - encs[1]);
    // double diffTick2 = abs(w2Tick - encs[0]);
    // double l_speed = speed;
    // double r_speed = speed;

    // if(diffTick1 > diffTick2)
    // {
    //     l_speed = speed + 2;
    // } else if(diffTick1 < diffTick2)
    // {
    //     r_speed = speed + 2;
    // }

    std_msgs::msg::Int16 cmd;
    cmd.data = speed; 
    wheel_pub_[1]->publish(cmd);
    cmd.data = speed;
    wheel_pub_[0]->publish(cmd);
    cmd.data = 0;
    wheel_pub_[2]->publish(cmd);
}
   
void GoDiagUpRight()
{
            std_msgs::msg::Int16 cmd;
            cmd.data = -speed; 
            wheel_pub_[1]->publish(cmd);
            cmd.data = speed;
            wheel_pub_[2]->publish(cmd);
            cmd.data = 0;
            wheel_pub_[0]->publish(cmd);
}
   
void GoDiagDownLeft()
{
            std_msgs::msg::Int16 cmd;
            cmd.data = speed; 
            wheel_pub_[1]->publish(cmd);
            cmd.data = -speed;
            wheel_pub_[2]->publish(cmd);
            cmd.data = 0;
            wheel_pub_[0]->publish(cmd);
}
void GoBackward()
{
            std_msgs::msg::Int16 cmd;
            cmd.data = -speed; 
            wheel_pub_[1]->publish(cmd);
            cmd.data = speed;
            wheel_pub_[0]->publish(cmd);
            cmd.data = 0;
            wheel_pub_[2]->publish(cmd);
}
   

void OpenClaw()
{
    std_msgs::msg::Float32 cmd;
    cmd.data = maxAngle;
    servo_pub->publish(cmd);
}

void CloseClaw()
{
    std_msgs::msg::Float32 cmd;
    cmd.data = minAngle;
    servo_pub->publish(cmd);
}

bool isHolding = false;

    void machineState() {
  
        std::string log = "";

        switch(actualState)
        {
            /*-------------------*/
            case ActualState::Init:

            if(timer > 5000)
            {
                    timer = 0;
                    actualState = ActualState::GoToFirstWall;
                    updateTicks = true;
                    break;
            }

            sendToMotors(0);
            OpenClaw();
            timer += 25;
            log = "Start timer: " + std::to_string(timer) + "/5000";
            break;

            /*-------------------*/
            case ActualState::GoToFirstWall:

            if(distanceFromWallBehind < 0.28)
            {
                actualState = ActualState::TurnT1Right;
                SetupOrientationGoal("down");
                sendToMotors(0);
                break;
            }                  

            GoForward();
            log = "Distance from wall: " + std::to_string(distanceFromWallBehind); 
            break;

            /*-------------------*/
            case ActualState::TurnT1Right:

            if(isOrientationOk())
            {
                actualState = ActualState::StopAtQr;
                sendToMotors(0);
                break;
            }         

            sendToMotors(speed);
            log = "Y :"  + std::to_string(gyroVector) + " / " + std::to_string(gyroTarget);
            break;

            /*-------------------*/
            case ActualState::StopAtQr:

            if(distanceFromWallBehind < 0.3)
            {
                actualState = ActualState::MoveRToQr;
                sendToMotors(0);
                break;
            }                  

            GoForward();
            log = "Distance from wall: " + std::to_string(distanceFromWallBehind); 
            break;

             /*-------------------*/
            case ActualState::MoveRToQr:

            if(timer > 1250)
            {
                timer = 0;
                actualState = ActualState::WaitQr;
                break;
            }
            GoDiagUpRight();
            timer += 25;
            break;

            /*-------------------*/
            case ActualState::WaitQr:

            if(timer > 2000)
            {
                timer = 0;
                actualState = ActualState::AligningWithCube;
                log = "Qr Code : 31";
                break;
            }
            sendToMotors(0);
            timer += 25;
            break;

            /*-----------------*/
            case ActualState::AligningWithCube:

            if(timer > 3150)
            {
                timer = 0;
                actualState = ActualState::GoingC5;
                break;
            }
            GoDiagUpRight();
            timer += 25;
            break;

            /*-----------------*/
            case ActualState::GoingC5:

             if(distanceFromWallBehind < 0.11)
            {
                actualState = ActualState::CloseC5;
                sendToMotors(0);
                break;
            }                  

            GoForward();
            log = "Distance from wall: " + std::to_string(distanceFromWallBehind); 
            break;

            /*-----------------*/
            case ActualState::CloseC5:

            if(timer > 2000)
            {
                timer = 0;
                actualState = ActualState::BackwardC5;
                SetupOrientationGoal("upLeft");
                break;
            }
            isHolding = true;
            sendToMotors(0);
            timer += 25;
            break;

            /*-----------------*/
            case ActualState::BackwardC5:

            if(isOrientationOk())
            {
                actualState = ActualState::LeavingC5;
                
                break;
            }
            sendToMotors(speed);
            break;

             /*-----------------*/
            case ActualState::LeavingC5:

            if(distanceFromWall > .90)
            {
                actualState = ActualState::EnterT3Left;
                SetupOrientationGoal("left");
                break;
            }

            GoForward();
            break;
            
            /*-----------------*/
            case ActualState::EnterT3Left:

            if(isOrientationOk())
            {
                actualState = ActualState::EnterT3;
                break;
            }
            sendToMotors(-speed);
            break;

            /*-----------------*/
            case ActualState::EnterT3:

            if(distanceFromWallBehind < 0.15)
            {
                actualState = ActualState::StopT3;
                break;
            }
            GoForward();
            break;

            /*-----------------*/
            case ActualState::StopT3:

            if(timer > 2000)
            {
                timer = 0;
                actualState = ActualState::ExitT3;
                break;
            }
            timer += 25;
            isHolding = false;
            sendToMotors(0);
            break;

            /*-----------------*/
            case ActualState::ExitT3:

            if(distanceFromWall > 0.80)
            {
                actualState = ActualState::BackSquare;
                SetupOrientationGoal("right");
                sendToMotors(0);
                break;
            }
            GoBackward();
            break;

             /*-----------------*/
            case ActualState::BackSquare:

            if(isOrientationOk())
            {
                isHolding = true;
                actualState = ActualState::END;
                break;
            }
            sendToMotors(speed);
            break;
        }
    
        if(isHolding)
            CloseClaw();
        else
            OpenClaw();

       //log = "Y :"  + std::to_string(gyroVector);

        if(log != "")
        {
            log += " State Number : " + std::to_string(static_cast<int>(actualState));
            RCLCPP_INFO(this->get_logger(), log.c_str());
        }
    }

   

private:
    float distanceFromWall, distanceFromWallBehind;
    ActualState actualState = ActualState::Init;
    int timer = 0;
    int startEncs[3];
    int targetEncs[3];
    bool end = false;
    double gyroVector;


int timerCervo = 0;
    int maxTimer = 500;
    float minAngle = 80, maxAngle = 150;
    bool isCervoOpen = false;

    int tickFor90 = 1300;

    int speed = 20;
    int fd_;
    struct libevdev *dev_ = nullptr;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr wheel_pub_[3];
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr claw_pub[2];
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ussSub;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ussSub1;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr enc_sub[3];
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr gyroSub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr servo_pub;
    int encs[3];
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavDuBled>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

