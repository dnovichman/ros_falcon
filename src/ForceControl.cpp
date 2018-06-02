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

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/ParamGet.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <eigen3/Eigen/Dense>
#include <std_msgs/Bool.h>


using namespace libnifalcon;
using namespace std;
using namespace StamperKinematicImpl;
FalconDevice m_falconDevice;

ros::Publisher joy_pub, setpoints_pub, rc_overide_pub, joy_cam_pub;
ros::Subscriber setpoints_sub, odom_sub, mavros_state_sub;

nav_msgs::Odometry vehicle_odom;
tf::Matrix3x3 Rotation_R;

std::array<bool,3> button;

mavros_msgs::PositionTarget des_traj_mavros;
float vehicle_current_heading;
geometry_msgs::Point des_pos, des_vel, des_acc;
int idx, idy, idz;
float scale_x, scale_y, scale_z;

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


mavros_msgs::State mavros_state;
ros::ServiceClient param_listener;

mavros_msgs::ParamGet param_get_srv;
mavros_msgs::OverrideRCIn rc_in;

void setpointCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
void setpointMavrosCallback(const mavros_msgs::PositionTarget::ConstPtr& msg);
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);
void readRcParams(void);
mavros_msgs::OverrideRCIn convertToRc(Eigen::Vector3f curp);
void loadParameters(const ros::NodeHandle& n);
void bound_errors(void);
void velocity_filter(std::array<double, 3> prev_pos, double dt);
void initiaizeValues(void);
double lowPass_filter2(double  xk, double  xk1, double  a );
	
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
}

bool init_falcon(int NoFalcon) 
{  
    readRcParams();
    
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

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    param_get_srv.request.param_id = "RC1_MAX";
    param_listener.call(param_get_srv);
    int rc1max = param_get_srv.response.value.real;
    param_get_srv.request.param_id = "RC1_MIN";
    param_listener.call(param_get_srv);
    int rc1min = param_get_srv.response.value.real;

    param_get_srv.request.param_id = "RC1_TRIM";
    param_listener.call(param_get_srv);
    int rc1trim = param_get_srv.response.value.real;


	vehicle_odom.header = msg->header;
	vehicle_odom.pose = msg->pose;
    vehicle_odom.twist = msg->twist;

	double roll, pitch, yaw;
	tf::Matrix3x3 R_matrxix;
	tf::Quaternion q(vehicle_odom.pose.pose.orientation.x, vehicle_odom.pose.pose.orientation.y, vehicle_odom.pose.pose.orientation.z, vehicle_odom.pose.pose.orientation.w);
	R_matrxix.setRotation(q);
	R_matrxix.getEulerYPR(yaw, pitch, roll);
	Rotation_R.setRotation(q);
	vehicle_current_heading = yaw;
	tf::Vector3 vel_int(vehicle_odom.twist.twist.linear.x, vehicle_odom.twist.twist.linear.y, vehicle_odom.twist.twist.linear.z);
	tf::Vector3 vel_body;
	vel_body = Rotation_R.transpose()*vel_int;
	//pos2 is x pos0 is y pos1 is z

    // Please check the offsets
	newHome[idx] = (vel_body.getX() - (-vel_limits(0)))/(vel_limits(0) - (-vel_limits(0)))*(pos_lims(0,1) - pos_lims(0,0)) + pos_lims(0,0);
	newHome[idy] = (vel_body.getY() - (-vel_limits(1)))/(vel_limits(1) - (-vel_limits(1)))*(pos_lims(1,1) - pos_lims(1,0)) + pos_lims(1,0);
	newHome[idz] = (vel_body.getZ() - (-vel_limits(2)))/(vel_limits(2) - (-vel_limits(2)))*(pos_lims(2,1) - pos_lims(2,0)) + pos_lims(2,0);
}

void setpointMavrosCallback(const mavros_msgs::PositionTarget::ConstPtr& msg)
{
    tf::Vector3 final_vel_body, final_pose_body;
	tf::Vector3 vel_int(msg->velocity.x, msg->velocity.y, msg->velocity.z);
	tf::Vector3 pose_int(msg->position.x, msg->position.y, msg->position.z);

	final_pose_body = Rotation_R.transpose()*pose_int;
	final_vel_body = Rotation_R.transpose()*vel_int;	
	
	// this should actually be pos
	des_pos = msg->position;
	
    tf::Vector3 des_final_vel_b((Pos[idx]- (pos_lims(0,1) + pos_lims(0,0))/2.0f)*scale_x, Pos[idy]*scale_y, (Pos[idz])*scale_z);
	tf::Vector3 des_final_vel_i = Rotation_R*des_final_vel_b;
	
    des_traj_mavros.position.x = msg->position.x;
    des_traj_mavros.position.y = msg->position.y;
    des_traj_mavros.position.z = msg->position.z;

	des_traj_mavros.velocity.x = des_final_vel_i.getX();
	des_traj_mavros.velocity.y = des_final_vel_i.getY();
	des_traj_mavros.velocity.z = des_final_vel_i.getZ();
    float des_yawrate = yaw_rate_int*0.5f;

	des_traj_mavros.yaw = vehicle_current_heading + des_yawrate*10;

    des_traj_mavros.yaw_rate = msg->yaw_rate + des_yawrate;

    des_traj_mavros.acceleration_or_force.x = 0.0f;
    des_traj_mavros.acceleration_or_force.y = 0.0f;
    des_traj_mavros.acceleration_or_force.z = 0.0f;

    yaw_rate_int = 0.0f;
	
    // only publish if vehicle is in position control mode
    std::string posctrl_string = "POSCTL";
    if (posctrl_string.compare(mavros_state.mode) == 0)
    {
        
        Eigen::Vector3f curp(des_final_vel_b.getX(), des_final_vel_b.getY(), des_final_vel_b.getZ());
        rc_in = convertToRc(curp);
        rc_in.channels[4] = rc_in.channels[5] = rc_in.channels[6] = rc_in.channels[7] = 65535;
        //rc_overide_pub.publish(rc_in);
        bool cam_b;
        cam_b = button[1];
        joy_cam_pub.publish(cam_b);
        setpoints_pub.publish(des_traj_mavros);
        ROS_INFO("Pos cont veld %f %f %f %d",  des_final_vel_b.getX(),  des_final_vel_b.getY(),  des_final_vel_b.getZ(), yaw_rate_int);
    }
}

void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    mavros_state.header = msg->header;
    mavros_state.mode = msg->mode;
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

    debug = true;
    odom_sub = node.subscribe<nav_msgs::Odometry>("odom", 10, odomCallback);
    setpoints_sub = node.subscribe("setpoint_sub", 3, setpointMavrosCallback);
    setpoints_pub = node.advertise<mavros_msgs::PositionTarget>("setpoint_pub", 10);
    mavros_state_sub = node.subscribe("/mavros/state", 3, mavrosStateCallback);

    rc_overide_pub = node.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);
    joy_cam_pub = node.advertise<std_msgs::Bool>("joy_button", 10);

    param_listener = node.serviceClient<mavros_msgs::ParamGet>("/mavros/param/get");

    loadParameters(node);

    if(init_falcon(falcon_int))
    {
        cout << "Falcon Initialised Starting ROS Node" << endl;

        m_falconDevice.setFalconKinematic<libnifalcon::FalconKinematicStamper>();

        //Start ROS Publisher
        ros::Publisher pub = node.advertise<sensor_msgs::Joy>("/falcon/joystick",10);
        ros::Rate loop_rate(2000);
        // Eric's stuff
        std::array<double,3> errors, p1error, x_k, zk, zpk;
        errors[0] = errors[1] = errors[2] = 0;
        float KP = 21;
    float KI = 0.45;
    float KL = 0.83;
    float KD = 0.028;
    float Rat = 0.5;

float Ts = 0.001;

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
                
                errors(0) = Pos[0]- (pos_lims(2,1) + pos_lims(2,0))/2.0f;
                errors(1) = Pos[1]- (pos_lims(1,1) + pos_lims(1,0))/2.0f;
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

               /* // Eric's controller
                std::array<double,3> ppos;
                ppos[0] = Pos[0];
                ppos[1] = Pos[1];
                ppos[2] = (Pos[2]-0.12);
                static double vel_filter[3] = {0, 0, 0};
    static int i = 0;
// low pass filter the velocity for servoing the position

    for (i=0; i < 3; i++)
    {
        vel_filter[i] = lowPass_filter2(vel_filter[i], ppos[i], 0 ); // full pass when last parameter is 0;
}

                p1error[0] = errors[0];
    p1error[1] = errors[1];
    p1error[2] = errors[2];
    
    float scale = 0.55;   

    errors[0] = (vel_filter[0]/10-ppos[0]);//x in right direction
    errors[1] = (vel_filter[1]/10-ppos[1]);//y in front
    errors[2] = (vel_filter[2]/10-ppos[2]);
    double zpk[3];
    zpk[0] = (errors[0]-p1error[0])/Ts;
    zpk[1] = (errors[1]-p1error[1])/Ts;
    zpk[2] = (errors[2]-p1error[2])/Ts;

    zk[0] = Rat*zk[0]+(1-Rat)*zpk[0];
    zk[1] = Rat*zk[1]+(1-Rat)*zpk[1];
    zk[2] = Rat*zk[2]+(1-Rat)*zpk[2];

    x_k[0] = KL*tanh(x_k[0]+errors[0]);
    x_k[1] = KL*tanh(x_k[1]+errors[1]);
    x_k[2] = KL*tanh(x_k[2]+errors[2]);

    forces[0] = scale*1.5*KP*(errors[0]+1.0* KI*x_k[0]+KD*zk[0])*20;//pid force applied to falcon to servo the position
    forces[1] = scale*1.5*KP*(errors[1]+1.0 *KI*x_k[1]+1.0*KD*zk[1])*20+0.85;
forces[2] = scale*1.5*KP*(errors[2]+1.0*KI*x_k[2]+KD*zk[2])*20*1.5;

        */

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

    rc_in.channels[0] = rc_in.channels[1] = rc_in.channels[3] = 1500; 
    rc_in.channels[2] = 1000;
    rc_in.channels[5] = rc_in.channels[6] = rc_in.channels[7] = 65535;
    rc_in.channels[4] = 1500;
    rc_trims.setZero();

    Rotation_R.setIdentity();
    offsets.setZero();

    velocity_xy_max = 3.0f;
    velocity_z_max = 3.0f;

    force_max = 3.0f;
}

void readRcParams(void)
{
    // Get RC param values
    param_get_srv.request.param_id = "RC1_TRIM";
    param_listener.call(param_get_srv); 
    rc_trims(1) = (float)param_listener.call(param_get_srv); //Roll is y
    param_get_srv.request.param_id = "RC2_TRIM";
    param_listener.call(param_get_srv); 
    rc_trims(0) = (float)param_listener.call(param_get_srv); //Roll is x
    param_get_srv.request.param_id = "RC3_TRIM";
    param_listener.call(param_get_srv); 
    rc_trims(2) = (float)param_listener.call(param_get_srv); //Roll is z

    param_get_srv.request.param_id = "RC1_MIN";
    param_listener.call(param_get_srv); 
    min_rc(1) = (float)param_listener.call(param_get_srv); //Roll is y
    param_get_srv.request.param_id = "RC2_MIN";
    param_listener.call(param_get_srv); 
    min_rc(0) = (float)param_listener.call(param_get_srv); //Roll is x
    param_get_srv.request.param_id = "RC3_MIN";
    param_listener.call(param_get_srv); 
    min_rc(2) = (float)param_listener.call(param_get_srv); //Roll is z

    param_get_srv.request.param_id = "RC1_MAX";
    param_listener.call(param_get_srv); 
    max_rc(1) = (float)param_listener.call(param_get_srv); //Roll is y
    param_get_srv.request.param_id = "RC2_MAX";
    param_listener.call(param_get_srv); 
    max_rc(0) = (float)param_listener.call(param_get_srv); //Roll is x
    param_get_srv.request.param_id = "RC3_MAX";
    param_listener.call(param_get_srv); 
    max_rc(2) = (float)param_listener.call(param_get_srv); //Roll is z


    param_get_srv.request.param_id = "RC4_TRIM";
    param_listener.call(param_get_srv); 
    yaw_rc(1) = (float)param_listener.call(param_get_srv);
    param_get_srv.request.param_id = "RC4_MIN";
    param_listener.call(param_get_srv); 
    yaw_rc(0) = (float)param_listener.call(param_get_srv);
    param_get_srv.request.param_id = "RC4_MAX";
    param_listener.call(param_get_srv); 
    yaw_rc(2) = (float)param_listener.call(param_get_srv);  
}

mavros_msgs::OverrideRCIn convertToRc(Eigen::Vector3f curp)
{
    mavros_msgs::OverrideRCIn rc;
    rc.channels[0] = (curp(1) - (-vel_limits(1)))/(vel_limits(1) - (-vel_limits(1)))*(max_rc(0) - min_rc(0)) + min_rc(0);
    rc.channels[1] = (curp(0) - (-vel_limits(0)))/(vel_limits(0) - (-vel_limits(0)))*(max_rc(1) - min_rc(1)) + min_rc(1);
    rc.channels[2] = (curp(2) - (-vel_limits(2)))/(vel_limits(2) - (-vel_limits(2)))*(max_rc(2) - min_rc(2)) + min_rc(2);
    return rc;
}

double lowPass_filter2(double  xk, double  xk1, double  a )
{
    double  yk;

    yk = a * xk + (1 - a) * xk1;
    return(yk);
}