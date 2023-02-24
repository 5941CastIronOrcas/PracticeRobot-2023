// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot  
{
  XboxController Controller = new XboxController(0);
  VictorSP FrontRightDriveMotor = new VictorSP(2);
  VictorSP FrontLeftDriveMotor = new VictorSP(0);
  VictorSP BackRightDriveMotor = new VictorSP(3);
  VictorSP BackLeftDriveMotor = new VictorSP(1);
  
  double TimeSinceStartAtAutoStart;
  double turnMultiplier = 0.5;
  double crouchedSpeedMult = 0.25;
  double crouchedTurnMult = 0.5;
  boolean crouched = false;

  private final Vision m_vision = new Vision();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() 
  {
    FrontRightDriveMotor.setInverted(true);
    //BackRightDriveMotor.setInverted(true);
    FrontLeftDriveMotor.setInverted(true);
    BackLeftDriveMotor.setInverted(true);
    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic()
  {
    SmartDashboard.putNumber("FidId", Vision.obtainTargets().getFiducialId());
    if (Vision.getEstimatedGlobalPose().isPresent()) {
      SmartDashboard.putBoolean("Pose present", true);
      SmartDashboard.putNumber("Est X Pos", Vision.getEstimatedGlobalPose().get().getX());
      SmartDashboard.putNumber("Est Y Pos", Vision.getEstimatedGlobalPose().get().getY());
    }
    else {
      SmartDashboard.putBoolean("Pose present", false);
      SmartDashboard.putNumber("Est X Pos", -1);
      SmartDashboard.putNumber("Est Y Pos", -1);
    }
  }

  /** This function is run once each time the robot enters autonomous mode. */

  public void autonomousInit() 
  {
    TimeSinceStartAtAutoStart=Timer.getFPGATimestamp();

    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() 
  {
    if(Timer.getFPGATimestamp() - TimeSinceStartAtAutoStart < 2)
    {
      
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() 
  {
    if(Controller.getLeftStickButtonPressed())
    {
      crouched =!crouched;
    }

    double forwardBack = (crouched?crouchedSpeedMult:1) * exponential(deadZone(Controller.getLeftY(), 0.1));
    double rightLeft = (crouched?crouchedTurnMult:1) * turnMultiplier * exponential(deadZone(Controller.getRightX(), 0.1));
    FrontRightDriveMotor.set(forwardBack+rightLeft);
    BackRightDriveMotor.set(forwardBack+rightLeft);
    FrontLeftDriveMotor.set(forwardBack-rightLeft);
    BackLeftDriveMotor.set(forwardBack-rightLeft);
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  public double deadZone(double value, double margin){
    if(Math.abs(value) <= margin){
      return 0;
    }
    return value;
  }

  public double exponential(double value){
    return value*Math.abs(value);
  }
}
