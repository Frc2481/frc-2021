#pragma once
#include <frc2/command/SubsystemBase.h>
#include <frc/doubleSolenoid.h>
#include <ctre/Phoenix.h>
#include "components/TalonFXMotorController.h"
#include "components/SparkMaxMotorController.h"
#include <frc/Solenoid.h>
class ShooterSubsystem : public frc2::SubsystemBase {

    
public:
    ShooterSubsystem();
    /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
   void Periodic() override;
   void setShooterSpeed(double speed); 
   bool isShooterOn();
   bool isShooterOnTarget();
   double getShooterSpeed();
   void stopShooter();
   void startShooter();
   void updatePID(double p, double i, double d, double f, double IZone);
   double distanceToTarget();
   void setCloseShot(bool close);
   bool isCloseShot();
   void autoShootSpeed();
private: 
    double m_speed;
    bool m_shooting;
    bool m_shooterOnTarget;
    bool m_closeShot;
    std::vector<double> m_speedVect;
    std::vector<double> m_distVect;
    TalonFXMotorController m_shooterMotor;
    TalonFXMotorController m_shooterFollower; 
    TalonFXMotorController m_shooterFollowerFollower;   
    frc::DoubleSolenoid m_shooterAdjust;
}; 
