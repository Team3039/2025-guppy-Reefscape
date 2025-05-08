
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

// import frc.robot.subsystems.Elevator.ElevatorState;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.PowerDistribution;
// import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;


  private final RobotContainer m_robotContainer;
    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final boolean kUseLimelight = true;

  UsbCamera usbCamera;

    


  CvSource outputStream;

  public Robot() {


    usbCamera = CameraServer.startAutomaticCapture();
    usbCamera.setResolution(640, 480);
    usbCamera.setFPS(30);

    m_robotContainer = new RobotContainer();
   

    Pathfinding.setPathfinder(new LocalADStar());
    
    outputStream = CameraServer.putVideo("Rectangle", 640, 480);
    outputStream = CameraServer.putVideo("limelight", 1280, 700);}

    
  @Override
  public void robotPeriodic() {



    NetworkTableInstance.getDefault().getTable("limelight").getEntry("limelight").getDoubleArray(new double[6]);
    CommandScheduler.getInstance().run();

    
    if (kUseLimelight) {
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

//This not working is a crime.

      LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);

      }

      
    }

    
    
    SmartDashboard.putString("update pose", m_robotContainer.drivetrain.getState().Pose.toString());
    
  }
  

  @Override
  public void disabledInit() {


  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {

     m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}