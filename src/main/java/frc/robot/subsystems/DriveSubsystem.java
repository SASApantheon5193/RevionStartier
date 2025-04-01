// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.utilities.LimelightHelpers;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft =
      new MAXSwerveModule(
          DriveConstants.kFrontLeftDrivingCanId,
          DriveConstants.kFrontLeftTurningCanId,
          DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight =
      new MAXSwerveModule(
          DriveConstants.kFrontRightDrivingCanId,
          DriveConstants.kFrontRightTurningCanId,
          DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft =
      new MAXSwerveModule(
          DriveConstants.kRearLeftDrivingCanId,
          DriveConstants.kRearLeftTurningCanId,
          DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight =
      new MAXSwerveModule(
          DriveConstants.kRearRightDrivingCanId,
          DriveConstants.kRearRightTurningCanId,
          DriveConstants.kBackRightChassisAngularOffset);

  private final Field2d m_field = new Field2d();

  // The gyro sensor
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  

  private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics,
          Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          },
          new Pose2d());

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    //m_gyro.reset(); // Ensure gyro starts at zero
    //SmartDashboard.putData("Field", m_field);
    
    // Set the Limelight to use the correct pipeline for AprilTags
    LimelightHelpers.setPipelineIndex("limelight", 0); // Adjust index as needed

    
}
  

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });

        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        if (poseEstimate.tagCount > 0) {
            m_poseEstimator.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
            SmartDashboard.putBoolean("Vision Data Received", true);
            SmartDashboard.putNumber("Vision X", poseEstimate.pose.getX());
            SmartDashboard.putNumber("Vision Y", poseEstimate.pose.getY());
            SmartDashboard.putNumber("Vision Rotation", poseEstimate.pose.getRotation().getDegrees());
      

                    // Update the vision pose marker on Field2d
        m_field.getObject("Vision Pose").setPose(poseEstimate.pose);
      } else {
          SmartDashboard.putBoolean("Vision Data Received", false);
          // Optionally clear the vision pose marker if no valid data is received
          // m_field.getObject("Vision Pose").setPose(new Pose2d());
      }

          // Update the Field2d with the robot's current (fused) pose
    m_field.setRobotPose(getPose());
        
        Pose2d estimatedPose = m_poseEstimator.getEstimatedPosition();
        SmartDashboard.putNumber("PathPlanner Pose X", estimatedPose.getX());
        SmartDashboard.putNumber("PathPlanner Pose Y", estimatedPose.getY());
        SmartDashboard.putNumber("PathPlanner Pose Rotation", estimatedPose.getRotation().getDegrees());
    
        // Update SmartDashboard with the field visualization
        updateSmartDashboard();

  }

public void updateSmartDashboard() {
    SmartDashboard.putData("Field", m_field);
}
   
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
}


  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose
    );
}

  
public void addVisionMeasurement() {
  LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

  if (poseEstimate.tagCount > 0 && poseEstimate.avgTagDist < 3.0) { // Ignore if tags are too far
      m_poseEstimator.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
  }
}

  
  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public Command setXCommand() {
    return this.run(
        () -> {
          m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
          m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        });
  }

public Command applyRequest(Runnable request) {

        return new RunCommand(request, this);

    }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public Command zeroHeadingCommand() {
    return this.runOnce(() -> m_gyro.reset());
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate(IMUAxis.kZ) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

// Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig configure;{
      try{
        configure = RobotConfig.fromGUISettings();
      } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
      }}

      public void updateRobotConfig() {
        try {
            // Reload configuration from the GUI
            RobotConfig newConfig = RobotConfig.fromGUISettings();
            // Update your configuration usage as needed. For instance, if you have a field:
            this.configure = newConfig;
            SmartDashboard.putString("RobotConfig Status", "Updated from GUI");
        } catch (Exception e) {
            e.printStackTrace();
            SmartDashboard.putString("RobotConfig Status", "Failed to update");
        }
    }

    public Command resetPoseWithVision() {
      return this.runOnce(() -> {
          LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
  
          if (poseEstimate.tagCount > 0 && poseEstimate.avgTagDist < 3.0) { // Check reliability
              m_poseEstimator.resetPosition(
                  Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)), 
                  new SwerveModulePosition[] {
                      m_frontLeft.getPosition(),
                      m_frontRight.getPosition(),
                      m_rearLeft.getPosition(),
                      m_rearRight.getPosition()
                  }, 
                  poseEstimate.pose
              );
          }
      });
  }
  

  {
    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            configure, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }



    public void driveRobotRelative(ChassisSpeeds speeds){
        drive(speeds, false);
    }

    public void drive(ChassisSpeeds speeds,boolean fieldRelative){
        if(fieldRelative)
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation());
        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        setModuleStates(swerveModuleStates);
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public SwerveModuleState[] getModuleStates(){
        return new SwerveModuleState[]{
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState()
        };
      }
      
    }