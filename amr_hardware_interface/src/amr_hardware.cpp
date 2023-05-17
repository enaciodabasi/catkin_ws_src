/**
 * @file amr_hardware.cpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "../include/amr_hardware.hpp"
#include "../include/offset_el7221_9014.hpp"

namespace amr
{
    namespace hardware
    {
        using namespace ethercat_interface;

        double HardwareInterface::limit(double& v, double v0, double v1, double dt)
        {
            const double tempVel = v;

            limitAccel(v, v0, dt);
            limitVel(v);

            return tempVel != 0.0 ? v / tempVel : 1.0;
        }

        double HardwareInterface::limitVel(double& v)
        {
            const double tempVel = v;

            v = clamp(v, m_MinVelocity, m_MaxVelocity);

            return tempVel != 0.0 ? v / tempVel : 1.0;
        }

        double HardwareInterface::limitAccel(double& v, double v0, double dt)
        {
            const double tempVel = v;

            const double dvMin = m_MinAccel * dt;
            const double dvMax = m_MaxAccel * dt;

            const double dv = clamp(v - v0, dvMin, dvMax);

            v = v0 + dv;

            return tempVel != 0.0 ? v / tempVel : 1.0; 
        }

        HardwareInterface::HardwareInterface(ros::NodeHandle& nh)
            : m_NodeHandle{nh}
        {
            this->loadParams();
            m_JointNames[0] = "lw_joint";
            m_JointNames[1] = "rw_joint";
            cm = std::make_shared<controller_manager::ControllerManager>(this, m_NodeHandle);
            PERIOD_NS = period_nanosec(500);
            m_CycleTime = {0, PERIOD_NS};

            //const auto sysuser = getenv("USER");
            m_Logger = std::make_shared<logger::Logger>("/home/ubuntu/catkin_ws/src/amr_hardware_interface/logs/", logger::FILE);
            ROS_INFO("Creating master");
            m_Master = new master::Master(0, m_Logger);
            ROS_INFO("Created master");
            m_Domain = new domain::Domain("amr_domain", m_Logger);
            ROS_INFO("Created domain");
            m_Master->registerDomain(m_Domain);
            ROS_INFO("Registered domain.");

            m_Domain->registerSlave(
                new slave::Slave(
                    "EK1100_0",
                    "/home/ubuntu/catkin_ws/src/amr_hardware_interface/config/amr_config.yaml",
                    nullptr,
                    m_Logger,
                    false
                )
            );
            ROS_INFO("Created Slave EK1100");
            m_Domain->registerSlave(
                new slave::Slave(
                    "EL7221_9014_0",
                    "/home/ubuntu/catkin_ws/src/amr_hardware_interface/config/amr_config.yaml",
                    new EL7221_9014_Offset(),
                    m_Logger,
                    true
                )
            );
            ROS_INFO("Created Slave EL7221_9014_0");

            m_Domain->registerSlave(
                new slave::Slave(
                    "EL7221_9014_1",
                    "/home/ubuntu/catkin_ws/src/amr_hardware_interface/config/amr_config.yaml",
                    new EL7221_9014_Offset(),
                    m_Logger,
                    true
                )
            );
            ROS_INFO("Created Slave EL7221_9014_1");

            /* m_DriveStatusServer = m_NodeHandle.advertiseService(
                "change_drive_status",
                &HardwareInterface::callback_drive_status_change,
                this
            ); */

            m_Master->configureDomains();
            m_Master->setupDomains();

            if(!m_Master->activateMaster())
            {
                ROS_FATAL("Can't activate master.");
                
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
            /* m_ControllerManager = boost::make_shared<controller_manager::ControllerManager>(
                this,
                m_NodeHandle
            ); */

            clock_gettime(m_ClockToUse, &m_WakeupTime);

            /* ros::Duration updateFrequency = ros::Duration(0.002);
            m_Loop = m_NodeHandle.createTimer(
                updateFrequency,
                &HardwareInterface::update,
                this
            ); */

        }

        HardwareInterface::~HardwareInterface()
        {
            delete m_Domain;
            delete m_Master;
        }

        void HardwareInterface::write()
        {   
            std::unique_lock<std::mutex> lck(m_DataMutex);
            double targetVelLeft = m_TargetVelLeft;

            double targetVelRight = m_TargetVelRight;
            lck.unlock();

            //hw.m_TargetVelLeft = amr::utils::linearVelToDriverCmd(
        //        hw.m_VelocityCommands[0] / 10.0,
        //        hw.m_DriverInfo
        //) * -1 ;
//
        //hw.m_TargetVelRight =  amr::utils::linearVelToDriverCmd(
        //    hw.m_VelocityCommands[1] / 10.0,
        //    hw.m_DriverInfo
        //);

            int32_t targetEncoderVelLeft = amr::utils::linearVelToDriverCmd(
                targetVelLeft,
                m_DriverInfo 
            ) * -1;

            int32_t targetEncoderVelRight = amr::utils::linearVelToDriverCmd(
                targetVelRight,
                m_DriverInfo
            );

            m_Master->write<int32_t>(
                "amr_domain",
                "EL7221_9014_0",
                "target_velocity",
                targetEncoderVelLeft
            );
            //// sag
            m_Master->write<int32_t>(
                "amr_domain",
                "EL7221_9014_1",
                "target_velocity",
                targetEncoderVelRight
            );
            
        }

        void HardwareInterface::read()
        {
            
            std::unique_lock<std::mutex> lck(m_DataMutex);
            auto posLeft = m_Master->read<int32_t>("amr_domain", "EL7221_9014_0", "current_position");


            //double leftWheelPos = 0.0;
            auto posRight = m_Master->read<int32_t>("amr_domain", "EL7221_9014_1", "current_position");

            if(INITIAL_EC_READ)
            {
                POS_DIFF_LEFT = posLeft;
                POS_DIFF_RIGHT = posRight;
                INITIAL_EC_READ = false;
            }
            //double rightWheelPos = 0.0;
            
            auto velLeft = m_Master->read<int32_t>("amr_domain", "EL7221_9014_0", "current_velocity");
            auto velRight = m_Master->read<int32_t>("amr_domain", "EL7221_9014_1", "current_velocity");

            //m_JointVelocities[0] = utils::driverVelToLinear(leftWheelVel, m_DriverInfo) / m_WheelRadius;
            //            
            //m_JointVelocities[1] = utils::driverVelToLinear(rightWheelVel, m_DriverInfo) / m_WheelRadius;
            
            m_LeftWheelPos = utils::motorPositionToWheelPositionRad(posLeft, m_PositionHelper) - utils::motorPositionToWheelPositionRad(POS_DIFF_LEFT, m_PositionHelper);
            m_RightWheelPos = utils::motorPositionToWheelPositionRad(posRight, m_PositionHelper) - utils::motorPositionToWheelPositionRad(POS_DIFF_RIGHT, m_PositionHelper);
        
            m_CurrentVelLeft = utils::driverVelToLinear(velLeft, m_DriverInfo)/m_WheelRadius;
            m_CurrentVelRight = utils::driverVelToLinear(velRight, m_DriverInfo)/m_WheelRadius;

            lck.unlock();

        }

        /* bool HardwareInterface::callback_drive_status_change(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
        {

            return true;
        } */

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

        
        void HardwareInterface::update()
        {

            //ros::Time prev_time = ros::Time::now();
            int counter {500};
            clock_gettime(this->m_ClockToUse, &this->m_WakeupTime);
            while(ros::ok())
            {
                this->m_WakeupTime = addTimespec(this->m_WakeupTime, this->m_CycleTime);
                sleep_task(this->m_ClockToUse, TIMER_ABSTIME, &this->m_WakeupTime, NULL);

                debug::measureTime(this->m_Measurer, this->m_WakeupTime);
                //const ros::Time time = ros::Time::now();
                //const ros::Duration period = prev_time - time;
                //const ros::Duration period (0,2000000);

                this->m_Master->setMasterTime(timespecToNanoSec(this->m_WakeupTime));
                this->m_Master->receive("amr_domain");

                this->m_Master->updateDomainStates();
//              this->m_Master->updateSlaveStates();
                bool slavesEnabled = this->m_Master->enableSlaves();

                if (counter<1) {
                auto timingStats = this->m_Measurer.getTimingStats();

                    auto  status0 = this->m_Master->read<uint16_t>("amr_domain", "EL7221_9014_0", "status_word");
                    auto  status1 = this->m_Master->read<uint16_t>("amr_domain", "EL7221_9014_1", "status_word");
                    auto  cmd0 = this->m_Master->read<uint16_t>("amr_domain", "EL7221_9014_0", "ctrl_word");
                    auto  cmd1 = this->m_Master->read<uint16_t>("amr_domain", "EL7221_9014_1", "ctrl_word");

                    std::string str;
                            if (slavesEnabled) str += "Slaves Enabled \n";
                            else str += "Slaves Disabled \n";
                            str += "cmd 0: " + std::to_string(cmd0) + " status 0: " + std::to_string(status0) + "\n" ;
                            str += "cmd 1: " + std::to_string(cmd1) + " status 1: " + std::to_string(status1) + "\n" ;  
                            str += "Left Wheel Vel:" + std::to_string(this->m_VelocityCommands[0]) + " Right Wheel Vel: " + std::to_string(this->m_VelocityCommands[1]) + '\n';
                            str += "Left Wheel Pos: " + std::to_string(m_JointPositions[0]) + " Right Wheel Pos:" + std::to_string(m_JointPositions[1]) + "\n";
                    ROS_INFO(timingStats.c_str());
                    ROS_INFO(str.c_str());
                    counter=500;
                    this->m_Master->updateMasterState();
                }

                counter --;

    
                this->m_Master->write<int8_t>(
                    "amr_domain",
                    "EL7221_9014_0",
                    "op_mode",
                    0x09
                );
                this->m_Master->write<int8_t>(
                    "amr_domain",
                    "EL7221_9014_1",
                    "op_mode",
                    0x09
                );

                if(slavesEnabled)
                    this->read();


                //cm->update(time, period);

                if(slavesEnabled)
                    this->write();

                bool syncRefClock = false;
                if(this->ref_clock_counter)
                {
                    this->ref_clock_counter -= 1;
                }
                else
                {
                    this->ref_clock_counter = 1;
                    syncRefClock = true;
                }
                //prev_time = time;
                clock_gettime(this->m_ClockToUse, &this->m_Time);
                this->m_Master->syncMasterClock(timespecToNanoSec(this->m_Time), syncRefClock);
                this->m_Master->send("amr_domain");
                this->m_Measurer.updateEndTime();
            }
        }
    }

    
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "amr_hardware_interface_node");
    ros::NodeHandle nh;
    amr::hardware::HardwareInterface hw(nh);
    ros::AsyncSpinner spinner(0);

    std::thread controller_thread(&amr::hardware::HardwareInterface::update, &hw);
    ScheduledThread::setScheduling(controller_thread, SCHED_FIFO, 99);
    auto prev_time =  ros::Time::now();
    double r = 0.0;
    nh.getParam("/amr/hardware_interface/loop_hz", r);
    spinner.start();
    ros::Rate rate(r);
    while(ros::ok())
    {
        //ros::spinOnce();
        std::unique_lock<std::mutex> lck(hw.m_DataMutex);
        hw.m_JointPositions[0] = hw.m_LeftWheelPos * (-1.0);
        hw.m_JointPositions[1] = hw.m_RightWheelPos;
        //auto msg = "POSITIONS: "+ std::to_string(hw.m_JointPositions[0]) + std::to_string(hw.m_JointPositions[1]);
        //ROS_INFO(msg.c_str());

        hw.m_JointVelocities[0] = hw.m_CurrentVelLeft;
        hw.m_JointVelocities[1] = hw.m_CurrentVelRight;

        const auto curr = ros::Time::now();
        const auto period = ros::Duration(curr - prev_time);

        hw.cm->update(curr, period);

        hw.m_TargetVelLeft = hw.m_VelocityCommands[0] / 10.0;
        hw.m_TargetVelRight = hw.m_VelocityCommands[1] / 10.0;

        //hw.m_TargetVelLeft = amr::utils::linearVelToDriverCmd(
        //        hw.m_VelocityCommands[0] / 10.0,
        //        hw.m_DriverInfo
        //) * -1 ;
//
        //hw.m_TargetVelRight =  amr::utils::linearVelToDriverCmd(
        //    hw.m_VelocityCommands[1] / 10.0,
        //    hw.m_DriverInfo
        //);

        lck.unlock();
        
        prev_time = curr;

        rate.sleep();

    }
    
    if(controller_thread.joinable())
    {
        controller_thread.join();
    }
    //ros::AsyncSpinner spinner(1);
    //spinner.start();
    /* int counter {500};
    

    ros::Time prev_time = ros::Time::now();

    clock_gettime(hw.m_ClockToUse, &hw.m_WakeupTime);
    auto printTimer = std::chrono::high_resolution_clock::now();

    while(ros::ok())
    {   
        
        hw.m_WakeupTime = addTimespec(hw.m_WakeupTime, hw.m_CycleTime);
        sleep_task(hw.m_ClockToUse, TIMER_ABSTIME, &hw.m_WakeupTime, NULL);
        
        debug::measureTime(hw.m_Measurer, hw.m_WakeupTime);
        const ros::Time time = ros::Time::now();
        //const ros::Duration period = prev_time - time;
        const ros::Duration period (0,2000000);

        hw.m_Master->setMasterTime(timespecToNanoSec(hw.m_WakeupTime));
        hw.m_Master->receive("amr_domain");

        hw.m_Master->updateDomainStates();
//      hw.m_Master->updateSlaveStates();
        bool slavesEnabled = hw.m_Master->enableSlaves();

        if (counter<1) {
        auto timingStats = hw.m_Measurer.getTimingStats();

<<<<<<< HEAD
        auto  status0 = hw.m_Master->read<uint16_t>("amr_domain", "EL7221_9014_0", "status_word");
        auto  status1 = hw.m_Master->read<uint16_t>("amr_domain", "EL7221_9014_1", "status_word");
        auto  cmd0 = hw.m_Master->read<uint16_t>("amr_domain", "EL7221_9014_0", "ctrl_word");
        auto  cmd1 = hw.m_Master->read<uint16_t>("amr_domain", "EL7221_9014_1", "ctrl_word");
        
        std::string str;
                if (slavesEnabled) str += "Slaves Enabled \n";
                else str += "Slaves Disabled \n";
                str += "cmd 0: " + std::to_string(cmd0) + " status 0: " + std::to_string(status0) + "\n" ;
                str += "cmd 1: " + std::to_string(cmd1) + " status 1: " + std::to_string(status1) + "\n" ;  
                str += "Left Wheel Vel:" + std::to_string(hw.m_VelocityCommands[0]) + " Right Wheel Vel: " + std::to_string(hw.m_VelocityCommands[1]) + '\n';
        ROS_INFO(timingStats.c_str());
        ROS_INFO(str.c_str());
        counter=500;
        hw.m_Master->updateMasterState();
        }
        
        counter --;
=======
        bool shouldPrint = [&printTimer](){
            auto now = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                now - printTimer
            );
            if(duration.count() >= 1)
                return true;

            return false;
        }();

        if(shouldPrint)
            ROS_INFO(timingStats.c_str());

        printTimer = std::chrono::high_resolution_clock::now();
>>>>>>> 7b3a55e1693f825628f05097868521a33ebcdc02

  
        hw.m_Master->write<int8_t>(
            "amr_domain",
            "EL7221_9014_0",
            "op_mode",
            0x09
        );
        hw.m_Master->write<int8_t>(
            "amr_domain",
            "EL7221_9014_1",
            "op_mode",
            0x09
        );
        
        if(slavesEnabled)
            hw.read();
        
        
        cm.update(time, period);

        if(slavesEnabled)
            hw.write();

        bool syncRefClock = false;
        if(hw.ref_clock_counter)
        {
            hw.ref_clock_counter -= 1;
        }
        else
        {
            hw.ref_clock_counter = 1;
            syncRefClock = true;
        }
        prev_time = time;
        clock_gettime(hw.m_ClockToUse, &hw.m_Time);
        hw.m_Master->syncMasterClock(timespecToNanoSec(hw.m_Time), syncRefClock);
        hw.m_Master->send("amr_domain");
        hw.m_Measurer.updateEndTime();
        ros::spinOnce();
    }
 */
    ros::waitForShutdown();

    return 0;
}