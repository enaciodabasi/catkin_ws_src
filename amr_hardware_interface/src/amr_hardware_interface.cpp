#include "../include/amr_hardware_interface.hpp"

void AmrEcController::cyclicTask()
{
    clock_gettime(m_DcHelper.clock, &m_DcHelper.wakeupTime);

    while(runEcThread)
    {
        setTaskWakeUpTime();
        sleep_task(m_DcHelper.clock, TIMER_ABSTIME, &m_DcHelper.wakeupTime, NULL);

        m_Master->setMasterTime(timespecToNanoSec(m_DcHelper.wakeupTime));
        m_Master->receive("amr_domain");

        m_Master->updateDomainStates();
        m_Master->updateMasterState();
        m_Master->updateSlaveStates();

        bool slavesEnabled = m_Master->enableSlaves();
        m_Master->write<int8_t>(
                    "amr_domain",
                    "EL7221_9014_0",
                    "op_mode",
                    0x09
        );
        m_Master->write<int8_t>(
            "amr_domain",
            "EL7221_9014_1",
            "op_mode",
            0x09
        );

        // Read from EC
        if(slavesEnabled)
        {

            auto leftMotorPosOpt = m_Master->read<int32_t>("amr_domain", "EL7221_9014_0", "current_position");
            auto rightMotorPosOpt = m_Master->read<int32_t>("amr_domain", "EL7221_9014_0", "current_position");

            if(leftMotorPosOpt != std::nullopt && rightMotorPosOpt != std::nullopt)
            {
                setData<int32_t>("EL7221_9014_0", "current_position", leftMotorPosOpt.value());
                setData<int32_t>("EL7221_9014_1", "current_position", rightMotorPosOpt.value());
            }

            auto leftMotorVelOpt = m_Master->read<int32_t>("amr_domain", "EL7221_9014_0", "current_velocity");
            auto rightMotorVelOpt = m_Master->read<int32_t>("amr_domain", "EL7221_9014_1", "current_velocity");
            if(leftMotorVelOpt != std::nullopt && rightMotorVelOpt != std::nullopt)
            {
                setData<int32_t>("EL7221_9014_0", "current_velocity", leftMotorVelOpt.value());
                setData<int32_t>("EL7221_9014_1", "current_velocity", rightMotorVelOpt.value());
            }
            
        }

        // Write to EC
        if(slavesEnabled)
        {
            auto leftTargetVelOpt = getData<int32_t>("EL7221_9014_0", "target_velocity");
            auto rightTargetVelOpt = getData<int32_t>("EL7221_9014_1", "target_velocity");

            if(leftTargetVelOpt != std::nullopt && rightTargetVelOpt != std::nullopt)
            {
                m_Master->write("amr_domain", "EL7221_9014_0", "target_velocity", leftTargetVelOpt.value());
                m_Master->write("amr_domain", "EL7221_9014_1", "target_velocity", rightTargetVelOpt.value());
            }
        }

        clock_gettime(m_DcHelper.clock, &m_DcHelper.currentTime);
        m_Master->syncMasterClock(timespecToNanoSec(m_DcHelper.currentTime), m_DcHelper);
        m_Master->send("amr_domain");

    }
}

namespace amr
{
    HardwareInterface::HardwareInterface(ros::NodeHandle& nh)
        : m_NodeHandle(nh)
    {
        
        this->loadParams();
        m_JointNames[0] = "lw_joint";
        m_JointNames[1] = "rw_joint";

        cm = std::make_shared<controller_manager::ControllerManager>(
            this,
            m_NodeHandle
        );

        bool ecSetupOK = m_EthercatController->setup();
        if(!ecSetupOK)
        {
            ros::shutdown();
        }

        m_NumJoints = m_JointNames.size();

        m_JointPositions.resize(m_NumJoints);
        m_JointVelocities.resize(m_NumJoints);
        m_JointEfforts.resize(m_NumJoints);
        m_VelocityCommands.resize(m_NumJoints);
        for(std::size_t i = 0; i < m_NumJoints; i++)
        {
            hardware_interface::JointStateHandle jointStateHandle(
                m_JointNames.at(i),
                &m_JointPositions.at(i),
                &m_JointVelocities.at(i),
                &m_JointEfforts.at(i)
            );
            m_JointStateInterface.registerHandle(jointStateHandle);
            hardware_interface::JointHandle jointHandle(
                jointStateHandle,
                &m_VelocityCommands.at(i)
            );
            m_VelJointInterface.registerHandle(jointHandle);
            
        }
        
            this->registerInterface(&m_JointStateInterface);

            this->registerInterface(&m_VelJointInterface);
        
    }

    HardwareInterface::~HardwareInterface()
    {

    }

    void HardwareInterface::loadParams()
    {
        if(m_NodeHandle.hasParam("/amr/hardware_interface/joints"))
        {
            m_NodeHandle.getParam("/amr/hardware_interface/joints", m_JointNames);
        }
        else
        {
            ROS_ERROR("No joint names find in the parameter server. Shutting down the hardware interface.");
            if(ros::ok())
                ros::shutdown();
        }
        if(m_NodeHandle.hasParam("/amr/harware_interface/loop_hz"))
        {
            m_NodeHandle.getParam("/amr/hardware_interface/loop_hz", m_LoopFrequency);
        }
        else
        {
            ROS_INFO("Loop freuency is not specified in the parameter server. Defaulting back to 50 Hz");
        }
        if(m_NodeHandle.hasParam("/amr/driver_info"))
        {   
            m_NodeHandle.getParam("/amr/driver_info/velocity_enc_resolution", m_DriverInfo.velocityEncoderResolution);
            m_NodeHandle.getParam("/amr/driver_info/wheel_side_gear", m_DriverInfo.wheelSideGear);
            m_NodeHandle.getParam("/amr/driver_info/motor_side_gear", m_DriverInfo.motorSideGear);
            m_NodeHandle.getParam("/amr/driver_info/motor_gear_heat", m_DriverInfo.motorGearHeat);
            m_NodeHandle.getParam("/amr/driver_info/wheel_diameter", m_DriverInfo.wheelDiameter);
            m_NodeHandle.getParam("/amr/driver_info/motor_max_rpm", m_DriverInfo.motorMaxRPM);
            
            m_PositionHelper.gearRatio = (m_DriverInfo.wheelSideGear / m_DriverInfo.motorSideGear) * m_DriverInfo.motorGearHeat;
            m_NodeHandle.getParam("/amr/hardware_interface/encoders/resolution", m_PositionHelper.encoderResolution);
        }
        else
        {
            ROS_INFO("Loop freuency is not specified in the parameter server. Defaulting back to 50 Hz");
        }
              
    }    
    
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "amr_hardware_interface_node");
    ros::NodeHandle nh;
    amr::HardwareInterface hw(nh);
    ros::AsyncSpinner spinner(0);
    auto prev_time =  ros::Time::now();
    double r = 0.0;
    nh.getParam("/amr/hardware_interface/loop_hz", r);
    spinner.start();
    ros::Rate rate(r);

    while(ros::ok())
    {

        // Read

        auto rawLeftPosOpt = hw.m_EthercatController->getData<int32_t>("EL7221_9014_0", "current_velocity");
        auto rawRightPosOpt = hw.m_EthercatController->getData<int32_t>("EL7221_9014_1", "current_velocity");
        
        // Write to values to joints
        if(rawLeftPosOpt != std::nullopt && rawRightPosOpt != std::nullopt)
        {
            double leftPositionRad = amr::utils::motorPositionToWheelPositionRad(rawLeftPosOpt.value(), hw.m_PositionHelper);
            double rightPositionRad = amr::utils::motorPositionToWheelPositionRad(rawRightPosOpt.value(), hw.m_PositionHelper);

            double leftVel = 0.0;
            double rightVel = 0.0;

            auto rawLeftVelOpt = hw.m_EthercatController->getData<int32_t>("EL7221_9014_0", "current_velocity");
            auto rawRightVelOpt = hw.m_EthercatController->getData<int32_t>("EL7221_9014_1", "current_velocity");

            if(rawLeftVelOpt != std::nullopt && rawRightVelOpt != std::nullopt)
            {
                leftVel = amr::utils::driverVelToLinear(rawLeftVelOpt.value(), hw.m_DriverInfo) / hw.m_WheelRadius;
                rightVel = amr::utils::driverVelToLinear(rawRightVelOpt.value(), hw.m_DriverInfo) / hw.m_WheelRadius;
            }

            hw.setJointInfo(
                leftPositionRad * -1.0,
                rightPositionRad,
                leftVel,
                rightVel
            );
            
        }

        // Update the Controller Manager instance

        const auto current = ros::Time::now();
        const auto period = ros::Duration(current - prev_time);

        hw.cm->update(current, period);

        const std::pair<double, double> velCmds = hw.getTargetVelocities();

        int32_t leftTargetVel = amr::utils::linearVelToDriverCmd(velCmds.first * -1.0, hw.m_DriverInfo);
        int32_t rightTargetVel = amr::utils::linearVelToDriverCmd(velCmds.second, hw.m_DriverInfo);

        hw.m_EthercatController->setData("EL7221_9014_0", "target_velocity", leftTargetVel);
        hw.m_EthercatController->setData("EL7221_9014_1", "target_velocity", rightTargetVel);
        
        // Write

        prev_time = current;
        rate.sleep();
    }

    ros::waitForShutdown();
    return 0;
}