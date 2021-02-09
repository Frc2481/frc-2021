#include "subsystems/ShooterSubsystem.h"
#include "Constants.h"
#include "RobotParameters.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "networktables/NetworkTableInstance.h"
#include "Utils/Interpolate.h"
    ShooterSubsystem::ShooterSubsystem() :
    m_speed(0.0),
    m_shooting(false),
    m_shooterOnTarget(false),
    m_shooterMotor(FalconIDs::kShooterMotorID , "shooterMotor"),//kShooterMotorID
    m_shooterFollower(FalconIDs::KShooterFollowerID, "shooterFollower"),//KShooterFollowerID
    m_shooterFollowerFollower(FalconIDs::kShooterFollowerFollowerID, "shooterFollowerFollower"),
    m_shooterAdjust(SolenoidPorts::kShooterSolenoidPort,SolenoidPorts::kShooterSolenoidReversePort){   
         m_shooterMotor.ConfigFactoryDefault();
         m_shooterFollower.ConfigFactoryDefault();
         m_shooterFollowerFollower.ConfigFactoryDefault();
         m_shooterMotor.Config_kP(0, RobotParameters::k_shooterP);
         m_shooterMotor.Config_kI(0,RobotParameters::k_shooterI);
         m_shooterMotor.Config_kD(0,RobotParameters::k_shooterD);
         m_shooterMotor.Config_kF(0,RobotParameters::k_shooterF);
         m_shooterMotor.Config_IntegralZone(0,25);
         m_shooterMotor.SetInverted(true);
         m_shooterFollower.SetInverted(true);
         m_shooterFollowerFollower.SetInverted(true);
         m_shooterFollower.Follow(m_shooterMotor.GetBase()); 
         m_shooterFollowerFollower.Follow(m_shooterMotor.GetBase()); 
         m_shooterAdjust.Set(frc::DoubleSolenoid::Value::kReverse);//TODO find out if correct
         m_closeShot = false;
         m_speedVect.push_back(ShooterConstants::kDefaultShooterShortSpeed);

	    m_speedVect.push_back(ShooterConstants::kDefaultShooterFarSpeed);
        m_distVect.push_back(13*12);
        m_distVect.push_back(32*12);
    }
    /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
   
   void ShooterSubsystem::Periodic(){       
       if (m_shooterMotor.GetVelocity()> m_speed){
           if(m_shooterMotor.GetVelocity() - m_speed< ShooterConstants::kUpperOnTargetTolerance){
               m_shooterOnTarget = true;
           }else{
               m_shooterOnTarget = false;
           }  
       }   
       else{
           if(m_speed - m_shooterMotor.GetVelocity() <  ShooterConstants::kLowerOnTargetTolerance){
               m_shooterOnTarget = true;
           }else{
               m_shooterOnTarget = false;
           } 

       } 
       distanceToTarget();
       frc::SmartDashboard::PutBoolean("shooter on target", m_shooterOnTarget);
       frc::SmartDashboard::PutNumber("Shoot Current Speed", m_shooterMotor.GetVelocity());
       frc::SmartDashboard::PutNumber("Shooter Setpoint", m_speed);
   }
   void ShooterSubsystem::setShooterSpeed(double speed){
       m_shooterMotor.Set(CommonModes::Velocity, speed);
       m_speed = speed;
   } 
   bool ShooterSubsystem::isShooterOn(){
       return m_shooting;
   }

   bool ShooterSubsystem::isShooterOnTarget(){
       return m_shooterOnTarget;
   }

   double ShooterSubsystem::getShooterSpeed(){
       return m_speed;
   }
   void ShooterSubsystem::stopShooter(){
       m_shooting = false;
       m_shooterMotor.Set(CommonModes::PercentOutput, 0.0);   
   }

   void ShooterSubsystem::startShooter(){
       m_shooting = true;
   }
  double ShooterSubsystem::distanceToTarget(){
    bool targetVisable = (bool)nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv",0);
    double theata = (nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty",0)-LimeLightConstants::kLimeLightAngle + LimeLightConstants::kLimeLightHardWarePanAngle)*MATH_CONSTANTS_PI/180;
    double height = LimeLightConstants::kTargetHeight - LimeLightConstants::kLimeLightHeight;
    if(!targetVisable){
        return -1;
    }if(theata == 0){
        return 0;
    }

    frc::SmartDashboard::PutNumber("robot distance", fabs(height/tan(theata)));
    return fabs(height/tan(theata));

  }
  
  void ShooterSubsystem::setCloseShot(bool close){ 
      if(close){
        //   printf("--------------extend shoot--------------\n");
        m_shooterAdjust.Set(frc::DoubleSolenoid::Value::kReverse);   
      }else{
        //   printf("--------------retract shoot--------------\n");
        m_shooterAdjust.Set(frc::DoubleSolenoid::Value::kForward);
      }   
      m_closeShot = close;
  }
  bool ShooterSubsystem::isCloseShot(){
    return m_closeShot;
  }

    void ShooterSubsystem::autoShootSpeed(){
        double dist = distanceToTarget();
        if(dist <= 0){
            m_shooterMotor.Set(CommonModes::Velocity, ShooterConstants::kDefaultShooterFarSpeed);
        }
        else{
            m_speed = interpolate::interp(m_distVect, m_speedVect, dist, false);
            if(dist < 25){
                setCloseShot(true);
            }else{
                setCloseShot(false);
            }
            // frc::SmartDashboard::PutNumber("auto speed", m_speed)
            m_shooterMotor.Set(CommonModes::Velocity, m_speed);
        }
        
    }
   void ShooterSubsystem::updatePID(double p, double i, double d, double f, double IZone){
       m_shooterMotor.Config_kP(0,p);
       m_shooterMotor.Config_kI(0,i);
       m_shooterMotor.Config_kD(0,d);
       m_shooterMotor.Config_kF(0,f);
       m_shooterMotor.Config_IntegralZone(0,IZone);
   }