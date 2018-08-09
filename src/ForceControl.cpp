#include <iostream>
#include <string>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include "falcon/core/FalconDevice.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconCLIBase.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/stamper/StamperUtils.h"
#include "falcon/kinematic/FalconKinematicStamper.h"
#include "falcon/core/FalconGeometry.h"
#include "falcon/gmtl/gmtl.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/grip/FalconGripFourButton.h"

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Dense>

#include <std_msgs/Bool.h>


using namespace libnifalcon;
using namespace std;
using namespace StamperKinematicImpl;
FalconDevice m_falconDevice;

ros::Publisher joy_pub, setpoints_pub, rc_overide_pub, joy_cam_pub;
ros::Subscriber setpoints_sub, odom_sub, mavros_state_sub;


std::array<bool,3> button;

float vehicle_current_heading;
geometry_msgs::Point des_pos, des_vel, des_acc;
int idx, idy, idz;
float scale_x, scale_y, scale_z, yaw_scale;

std::array<double, 3> Pos;
std::array<double, 3> newHome, prevHome;
int yaw_rate_int = 0;
float velocity_z_max, velocity_xy_max;
double prev_time, cur_time;
float KpGainX, KpGainY, KpGainZ;
float KdGainX, KdGainY, KdGainZ;
float KiGainX, KiGainY, KiGainZ;
float sat_error_x, sat_error_y, sat_error_z;
Eigen::Vector3f sum_errors;
float force_max;

Eigen::Vector3f current_pos, prev_pos, vel_filt;
float filt_b;

//std::array<double, 3> pos0
//pos[2] = [7.49 17.4]
// pos[0] = [-5.38 5.4]
// pos[1] = [-5.5 5.5]
Eigen::Matrix3f pos_lims;
Eigen::Vector3f offsets;
Eigen::Vector3f vel_limits;
Eigen::Vector3f rc_trims;
Eigen::Vector3f min_rc;
Eigen::Vector3f max_rc;
Eigen::Vector3f yaw_rc;

void setpointCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);



void loadParameters(const ros::NodeHandle& n);
void bound_errors(void);
void velocity_filter(std::array<double, 3> prev_pos, double dt);
void initiaizeValues(void);
	
void loadParameters(const ros::NodeHandle& n)
{
    initiaizeValues();

    n.getParam("velocity_xy_max", velocity_xy_max);
    n.getParam("velocity_z_max", velocity_z_max);
    n.getParam("KpGainX", KpGainX);
    n.getParam("KpGainY", KpGainY);
    n.getParam("KpGainZ", KpGainZ);

    n.getParam("KdGainX", KdGainX);
    n.getParam("KdGainY", KdGainY);
    n.getParam("KdGainZ", KdGainZ);

    n.getParam("KiGainX", KiGainX);
    n.getParam("KiGainY", KiGainY);
    n.getParam("KiGainZ", KiGainZ);

    n.getParam("SatErrorX", sat_error_x);
    n.getParam("SatErrorY", sat_error_y);
    n.getParam("SatErrorZ", sat_error_z); 
    n.getParam("VelFilterB", filt_b);
    n.getParam("ForceMax", force_max);
    n.getParam("YawScale", yaw_scale);
}

bool init_falcon(int NoFalcon) 
{  
    idx = 2;
    idy = 0;
    idz = 1;    
    
    vel_limits(0) = velocity_xy_max;
    vel_limits(1) = velocity_xy_max;
    vel_limits(2) = velocity_z_max;
    
    pos_lims.setIdentity();
    pos_lims(1,0) = -5.38/100;
    pos_lims(1,1) = 5.4/100;
    pos_lims(2,0) = -5.5/100;
    pos_lims(2,1) = 5.5/100;
    pos_lims(0,0) = 7.49/100;
    pos_lims(0,1) = 17.49/100;
    
    scale_x = (vel_limits(0) - (-vel_limits(0)))/(pos_lims(0,1) - pos_lims(0,0));
    scale_y = (vel_limits(1) - (-vel_limits(1)))/(pos_lims(1,1) - pos_lims(1,0));
    scale_z = (vel_limits(2) - (-vel_limits(2)))/(pos_lims(2,1) - pos_lims(2,0));

    scale_x = vel_limits(0)/(pos_lims(0,1) - pos_lims(0,0));
    scale_y = vel_limits(1)/(pos_lims(1,1) - pos_lims(1,0));
    scale_z = vel_limits(2)/(pos_lims(2,1) - pos_lims(2,0));
    
    
    cout << "Setting up LibUSB" << endl;
    m_falconDevice.setFalconFirmware<FalconFirmwareNovintSDK>(); //Set Firmware
    m_falconDevice.setFalconGrip<FalconGripFourButton>(); //Set Grip
    if(!m_falconDevice.open(NoFalcon)) //Open falcon @ index
    {
        cout << "Failed to find falcon" << endl;
        return false;
    }
    else
    {
        cout << "Falcon Found" << endl;
    }
    m_falconDevice.setFalconFirmware<FalconFirmwareNovintSDK>();

    bool skip_checksum = false;
    //See if we have firmware
    bool firmware_loaded = false;
    firmware_loaded = m_falconDevice.isFirmwareLoaded();
    if(!firmware_loaded)
    {
        cout << "Loading firmware" << endl;
        uint8_t* firmware_block;
        long firmware_size;
        {

            firmware_block = const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE);
            firmware_size = NOVINT_FALCON_NVENT_FIRMWARE_SIZE;


            for(int i = 0; i < 20; ++i)
            {
                if(!m_falconDevice.getFalconFirmware()->loadFirmware(skip_checksum, NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE)))

                {
                    cout << "Firmware loading try failed" <<endl;
                }
                else
                {
                    firmware_loaded = true;
                    break;
                }
            }
        }
    }
    else if(!firmware_loaded)
    {
        cout << "No firmware loaded to device, and no firmware specified to load (--nvent_firmware, --test_firmware, etc...). Cannot continue" << endl;
        return false;
    }
    if(!firmware_loaded || !m_falconDevice.isFirmwareLoaded())
    {
        cout << "No firmware loaded to device, cannot continue" << endl;
        return false;
    }
    cout << "Firmware loaded" << endl;

    m_falconDevice.getFalconFirmware()->setHomingMode(true); //Set homing mode (keep track of encoders !needed!)
    cout << "Homing Set" << endl;
    std::array<int, 3> forces;
    //m_falconDevice.getFalconFirmware()->setForces(forces);
    m_falconDevice.runIOLoop(); //read in data

    bool stop = false;
    bool homing = false;
    bool homing_reset = false;
    usleep(100000);
    int tryLoad = 0;
    while(!stop)
    {
        if(!m_falconDevice.runIOLoop()) continue;
        if(!m_falconDevice.getFalconFirmware()->isHomed())
        {
            if(!homing)
            {
                m_falconDevice.getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::RED_LED);
                cout << "Falcon not currently homed. Move control all the way out then push straight all the way in." << endl;

            }
            homing = true;
        }

        if(homing && m_falconDevice.getFalconFirmware()->isHomed())
        {
            m_falconDevice.getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::BLUE_LED);
            cout << "Falcon homed." << endl;
            homing_reset = true;
            stop = true;
            ROS_INFO("Home is %f %f %f", newHome[0]*100, newHome[1]*100, newHome[2]*100);
            offsets(0) = newHome[idx];
            offsets(1) = newHome[idy];
            offsets(2) = newHome[idx];
        }
        tryLoad++;
    }

    prev_time = ros::Time::now().toSec();

    m_falconDevice.runIOLoop();
    return true;
}


int main(int argc, char* argv[])
{
    ros::init(argc,argv, "FalconJoystick");

    ros::NodeHandle node("~");
    int falcon_int;
    bool debug;

    bool clutchPressed, coagPressed;
    node.param<int>("falcon_number", falcon_int, 0);
    node.param<bool>("falcon_debug", debug, false);
    node.param<bool>("falcon_clutch", clutchPressed, true);
    node.param<bool>("falcon_coag", coagPressed, true);

    //debug = true;


    joy_cam_pub = node.advertise<std_msgs::Bool>("joy_button", 10);


    loadParameters(node);

    if(init_falcon(falcon_int))
    {
        cout << "Falcon Initialised Starting ROS Node" << endl;

        m_falconDevice.setFalconKinematic<libnifalcon::FalconKinematicStamper>();

        //Start ROS Publisher
        ros::Publisher pub = node.advertise<sensor_msgs::Joy>("/falcon/joystick",10);
        ros::Rate loop_rate(2000);    

        while(node.ok())
        {
            sensor_msgs::Joy Joystick;
            std::array<double, 3> prevPos;

            Joystick.buttons.resize(1);
            Joystick.axes.resize(3);

            std::array<double,3> forces;
            Eigen::Vector3f errors;
            errors.setZero();
            
            int buttons;
            
            if(m_falconDevice.runIOLoop())
            {
                if(m_falconDevice.getFalconGrip()->getDigitalInputs() & libnifalcon::FalconGripFourButton::BUTTON_1)
                {
                    std::cout<<"button 1"<<std::endl;
                    button[0] = true;   
                    yaw_rate_int = 1;                 
                }
                else
                {
                    button[0] = false;
                }
                if(m_falconDevice.getFalconGrip()->getDigitalInputs() & libnifalcon::FalconGripFourButton::BUTTON_2)
                {
                    std::cout<<"button 2"<<std::endl;
                    button[1] = true;
                    // This should be take a picture
                }
                else
                {
                    button[1] = false;
                }
                if(m_falconDevice.getFalconGrip()->getDigitalInputs() & libnifalcon::FalconGripFourButton::BUTTON_3)
                {
                    std::cout<<"button 3"<<std::endl;
                    button[2] = true;
                }
                else
                {
                    button[2] = false;
                }
                if(m_falconDevice.getFalconGrip()->getDigitalInputs() & libnifalcon::FalconGripFourButton::BUTTON_4)
                {
                    std::cout<<"button 4"<<std::endl;
                    button[3] = true;   
                    yaw_rate_int = -1;
                }
                else
                {
                    button[3] = false;
                }
                Pos = m_falconDevice.getPosition(); 

                buttons = m_falconDevice.getFalconGrip()->getDigitalInputs();

                //Publish ROS values
                for (int i = 0; i<4; i++)
                    Joystick.buttons[i] = (int)button[i];

                Joystick.axes[0] = Pos[0];
                Joystick.axes[1] = Pos[1];
                Joystick.axes[2] = Pos[2];
                pub.publish(Joystick);

                cur_time = ros::Time::now().toSec();
                double dt = cur_time - prev_time;                
                
                errors(0) = Pos[0]- (pos_lims(2,1) + pos_lims(2,0))/2.0f + 0/100;
                errors(1) = Pos[1]- (pos_lims(1,1) + pos_lims(1,0))/2.0f -4/100;
                errors(2) = Pos[2] - (pos_lims(0,1) + pos_lims(0,0))/2.0f;

                sum_errors += errors;
                bound_errors();
                velocity_filter(prevPos, dt);

                forces[0] = - KpGainX*errors(0) - KdGainX*vel_filt(0) - KiGainX*sum_errors(0);
                forces[1] = - KpGainY*errors(1) - KdGainY*vel_filt(1) - KiGainY*sum_errors(1);
                forces[2] = - KpGainZ*errors(2) - KdGainZ*vel_filt(2) - KiGainZ*sum_errors(2);

                if (forces[0] > force_max)
                    forces[0] = force_max;
                if (forces[0] < -force_max)
                    forces[0] = -force_max;

                if (forces[1] > force_max)
                    forces[1] = force_max;
                if (forces[1] < -force_max)
                    forces[1] = -force_max;

                if (forces[2] > force_max)
                    forces[2] = force_max;
                if (forces[2] < -force_max)
                    forces[2] = -force_max;

                m_falconDevice.setForce(forces);
                if(debug)
                {
                    cout << "Position= " << Pos[0]*100 <<" " << Pos[1]*100 << " " << Pos[2]*100 <<  endl; 
                    cout << "newHome  = " << newHome[0]*100 <<" " << newHome[1]*100 << " " << newHome[2]*100 <<  endl;
                    cout << "Error   =" << (Pos[0] - newHome[0])*100 <<" " << (Pos[1]-newHome[1])*100 << " " << (Pos[2] -newHome[2])*100 <<  endl;
                    cout << "Force= " << forces[0] <<" " << forces[1] << " " << forces[2] <<  endl;
                }
                prevPos = Pos;
                prevHome = newHome;
            }
            ros::spinOnce();
            loop_rate.sleep();
            prev_time = cur_time;
        }
        m_falconDevice.close();
    }
    return 0;
}

void bound_errors(void)
{
    if (sum_errors(idx) > sat_error_x)
        sum_errors(idx) = sat_error_x;
    else if (sum_errors(idx) < -sat_error_x)
        sum_errors(idx) = -sat_error_x;
    if (sum_errors(idy) > sat_error_y)
        sum_errors(idy) = sat_error_y;
    else if (sum_errors(idy) < -sat_error_y)
        sum_errors(idy) = -sat_error_y;
    if (sum_errors(idz) > sat_error_z)
        sum_errors(idz) = sat_error_z;
    else if (sum_errors(idz) < -sat_error_z)
        sum_errors(idz) = -sat_error_z;
}

void velocity_filter(std::array<double, 3> prev_pos, double dt)
{
    Eigen::Vector3f vel_raw(Pos[0] - prev_pos[0], Pos[1] - prev_pos[1], Pos[2] - prev_pos[2]);


    vel_filt = (1 - filt_b)*vel_filt + filt_b*vel_raw/dt;
    if (vel_filt(0) != vel_filt(0) || vel_filt(1) != vel_filt(1) || vel_filt(2) != vel_filt(2) || std::isinf(vel_filt(0)) || std::isinf(vel_filt(1)) || std::isinf(vel_filt(2)))
        vel_filt.setZero();
}

void initiaizeValues(void)
{
    KpGainX = 200;
    KpGainY = 200;
    KpGainZ = 200;

    KdGainX = 500;
    KdGainY = 500;
    KdGainZ = 500;

    KiGainX = 200.0f;
    KiGainY = 200.0f;
    KiGainZ = 200.0f;

    sat_error_x = 10;
    sat_error_y = 10;
    sat_error_z = 10;

    sum_errors.setZero();

    vel_filt.setZero();
    filt_b = 1.0f;


    velocity_xy_max = 3.0f;
    velocity_z_max = 3.0f;

    force_max = 3.0f;
}
