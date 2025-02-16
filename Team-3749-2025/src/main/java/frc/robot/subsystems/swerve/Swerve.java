// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;

import choreo.auto.AutoTrajectory;

import org.littletonrobotics.junction.Logger;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.buttons.JoystickIO;
import frc.robot.buttons.ButtonBoard.ScoringMode;
import frc.robot.commands.auto.AutoConstants;
import frc.robot.commands.auto.AutoUtils;
import frc.robot.subsystems.swerve.GyroIO.GyroData;
import frc.robot.subsystems.swerve.SwerveConstants.MotorConstants;
import frc.robot.subsystems.swerve.ToPosConstants.Setpoints.PPSetpoints;
import frc.robot.subsystems.swerve.sim.GyroSim;
import frc.robot.subsystems.swerve.sim.SwerveModuleSim;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.MotorData;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.SysIdTuner;
import frc.robot.utils.UtilityFunctions;
import frc.robot.subsystems.swerve.SwerveConstants.ControlConstants;
import frc.robot.subsystems.swerve.SwerveConstants.DrivetrainConstants;
import frc.robot.subsystems.swerve.real.*;
import frc.robot.subsystems.vision.VisionConstants;

/***
 * Subsystem class for swerve drive, used to manage four swerve
 * modules and set their states. Also includes a pose estimator,
 * gyro, and logging information
 * 
 * Rotation standard: everything is relative to blue alliance. 0 is
 * from blue alliance wall, CCP
 * 
 * @author Noah Simon
 * @author Neel Adem
 * @author Rohin Sood
 * @author Raadwan Masum
 * 
 * 
 */
public class Swerve extends SubsystemBase {

  private SwerveModule[] modules = new SwerveModule[4];

  private GyroIO gyro;
  private GyroData gyroData = new GyroData();

  // equivilant to a odometer, but also intakes vision
  private SwerveDrivePoseEstimator swerveDrivePoseEstimator;

  private PIDController xController = new PIDController(AutoConstants.kPDrive, 0, AutoConstants.kDDrive);
  private PIDController yController = new PIDController(AutoConstants.kPDrive, 0, AutoConstants.kDDrive);
  private PIDController turnController = new PIDController(AutoConstants.kPTurn, 0, AutoConstants.kDTurn);

  private boolean utilizeVision = true;
  private double velocity = 0;
  private double yaw;

  private LoggedTunableNumber kPDriving = new LoggedTunableNumber("/subsystems/swerve/kP Drive", AutoConstants.kPDrive);
  private LoggedTunableNumber kDDriving = new LoggedTunableNumber("/subsystems/swerve/kD Drive", AutoConstants.kDDrive);
  private LoggedTunableNumber kPTurn = new LoggedTunableNumber("/subsystems/swerve/kP Turn controller", AutoConstants.kPTurn);
  private LoggedTunableNumber kDTurn = new LoggedTunableNumber("/subsystems/swerve/kD Turn controller", AutoConstants.kDTurn);


  // Logging
  private ShuffleData<String> currentCommandLog = new ShuffleData<String>(this.getName(), "current command", "None");

  private ShuffleData<Double[]> odometryLog = new ShuffleData<Double[]>(
      this.getName(),
      "odometry",
      new Double[] { 0.0, 0.0, 0.0 });

  private ShuffleData<Double[]> realStatesLog = new ShuffleData<Double[]>(
      this.getName(),
      "real states",
      new Double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });

  private ShuffleData<Double[]> desiredStatesLog = new ShuffleData<Double[]>(
      this.getName(),
      "desired states",
      new Double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });

  private ShuffleData<Double> velocityLog = new ShuffleData<Double>(
      this.getName(),
      "velocity",
      0.0);
  private ShuffleData<Double> accelerationLog = new ShuffleData<Double>(
      this.getName(),
      "acceleration",
      0.0);

  private ShuffleData<Double> yawLog = new ShuffleData<Double>(
      this.getName(),
      "yaw",
      0.0);

  private ShuffleData<Double> pitchLog = new ShuffleData<Double>(
      this.getName(),
      "pitch",
      0.0);

  private ShuffleData<Double> rollLog = new ShuffleData<Double>(
      this.getName(),
      "roll",
      0.0);

  private ShuffleData<Double> rotationalVelocityLog = new ShuffleData<Double>(
      this.getName(),
      "rotational velocity",
      0.0);

  private ShuffleData<Boolean> gyroConnectedLog = new ShuffleData<Boolean>(
      this.getName(),
      "gyro connected",

      false);
  private ShuffleData<Boolean> gyroCalibratingLog = new ShuffleData<Boolean>(
      this.getName(),
      "gyro calibrating",
      false);

  private ShuffleData<Double> headingLog = new ShuffleData<Double>(
      this.getName(),
      "heading",
      0.0);

  private ShuffleData<Boolean> utilizeVisionLog = new ShuffleData<Boolean>(
      this.getName(),
      "utilize vision",
      true);

      private ShuffleData<Double[]> setpointPositionLog = new ShuffleData<Double[]>(
        this.getName(),
        "setpoint position",
        new Double[] { 0.0, 0.0, 0.0 });
  
    private ShuffleData<Double[]> setpointGoalStateLog = new ShuffleData<Double[]>(
        this.getName(),
        "setpoint end goal",
        new Double[] { 0.0, 0.0, 0.0 });

        private ShuffleData<Double[]> setpointVelocityLog = new ShuffleData<Double[]>(
          this.getName(),
          "setpoint velocity",
          new Double[] { 0.0, 0.0, 0.0 });
    
      private ShuffleData<Double[]> setpointAccelerationLog = new ShuffleData<Double[]>(
          this.getName(),
          "setpoint acceleration",
          new Double[] { 0.0, 0.0, 0.0 });
    
  private ShuffleData<Double> setpointRotationalVelocityLog = new ShuffleData<Double>(
      this.getName(),
      "setpoint rotational velocity",
      0.0);

  private ShuffleData<Double> setpointRotationalAccelerationLog = new ShuffleData<Double>(
      this.getName(),
      "setpoint rotational acceleration",
      0.0);

  private int currentPPSetpointIndex = 0; //what "index" do we currently want to go to for OTF
  private int currentPPApproachSetpointIndex = 0;

  private boolean isOTF = false; //are we OTF driving rn

  public int getPPSetpointIndex()
  {
    return currentPPApproachSetpointIndex;
  }

  public void setPPSetpointIndex(int index)
  {
    currentPPSetpointIndex = index;
  }

  public int getApproachSetpointIndex()
  {
    return currentPPApproachSetpointIndex;
  }

  public void setApproachSetpointIndex(int index)
  {
    currentPPApproachSetpointIndex = index;
  }

  public boolean getIsOTF()
  {
    return isOTF;
  }

  public void setIsOTF(boolean otf)
  {
    isOTF = otf;
  }

  public Swerve() {

    // if simulation
    if (Robot.isSimulation()) {
      gyro = new GyroSim();
      for (int i = 0; i < 4; i++) {
        modules[i] = new SwerveModule(i, new SwerveModuleSim());
      }
    }
    // if real
    else {
      // gyro = new NavX2Gyro();
      gyro = new PigeonGyro();
      for (int i = 0; i < 4; i++) {

        modules[i] = new SwerveModule(i, new SwerveModuleSpark(i));
      }
    }
    // pose estimator
    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
        DrivetrainConstants.driveKinematics,
        new Rotation2d(0),
        new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        },
        new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
        VecBuilder.fill(0.045, 0.045, 0.0004), // 6328's 2024 numbers with factors of 1.5x, 1.5x, 2x
        VecBuilder.fill(VisionConstants.StandardDeviations.PreMatch.xy,
            VisionConstants.StandardDeviations.PreMatch.xy,
            VisionConstants.StandardDeviations.PreMatch.thetaRads));

  
    turnController.enableContinuousInput(-Math.PI, Math.PI);

    // put us on the field with a default orientation
    resetGyro();
    setOdometry(new Pose2d(0,0, new Rotation2d(0)));
    logSetpoints(1.33, 0, 0, 5.53, 0, 0, 0, 0, 0);

  }

  

  public boolean getRotated() {
    return UtilityFunctions.withinMargin(0.01, modules[0].getModuleData().turnPositionRad,
        Rotation2d.fromDegrees(45).getRadians())
        && UtilityFunctions.withinMargin(0.01, modules[1].getModuleData().turnPositionRad,
            Rotation2d.fromDegrees(135).getRadians())
        && UtilityFunctions.withinMargin(0.01, modules[2].getModuleData().turnPositionRad,
            Rotation2d.fromDegrees(225).getRadians())
        && UtilityFunctions.withinMargin(0.01, modules[3].getModuleData().turnPositionRad,
            Rotation2d.fromDegrees(315).getRadians());

  }

  /**
   * Returns the coordinate and angular velocity of the robot
   * 
   * @return chassisSpeeds - ChasssisSpeeds object with a x, y, and rotaitonal
   *         velocity
   */
  public ChassisSpeeds getChassisSpeeds() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        DrivetrainConstants.driveKinematics.toChassisSpeeds(states),
        getRotation2d());
    return speeds;
  }

  /**
   * @return Returns direction the robot is facing as a Rotation2d object
   */
  public Rotation2d getRotation2d() {
    Rotation2d rotation = swerveDrivePoseEstimator
        .getEstimatedPosition()
        .getRotation();
    // return rotation;
    double heading = rotation.getDegrees();

    while (heading < 0) {
      heading += 360;
    }
    return new Rotation2d(heading / 180 * Math.PI);
  }

  /**
   * @return Returns coordinates and head the robot as a Pose2d object
   */
  public Pose2d getPose() {
    Pose2d estimatedPose = swerveDrivePoseEstimator.getEstimatedPosition();
    return new Pose2d(estimatedPose.getTranslation(), getRotation2d());
    // return new Pose2d(new Translation2d(2, 4.9), new Rotation2d(Math.PI/2));
  }

  /**
   * @return Returns max speed to be achieved by the robot based on telop or auto
   */
  public double getMaxDriveSpeed() {
    return DriverStation.isTeleopEnabled() ? ControlConstants.teleopMaxSpeedMetersPerSecond
        : ControlConstants.autoMaxSpeedMetersPerSecond;
  }

  /**
   * @return Returns max angular speed to be achieved by the robot based on telop
   *         or auto
   */
  public double getMaxAngularSpeed() {
    return DriverStation.isTeleopEnabled() ? ControlConstants.teleopMaxAngularSpeedRadPerSecond
        : ControlConstants.autoMaxAngularSpeedRadPerSecond;
  }

  public SwerveDrivePoseEstimator getPoseEstimator() {
    return swerveDrivePoseEstimator;
  }

  /**
   * Turns a set of coordinate and angular velocities into module states, then
   * sets all modules to those states
   * 
   * @param chassisSpeeds - ChasssisSpeeds object with a x, y, and rotaitonal
   *                      velocity
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {

    // Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DrivetrainConstants.driveKinematics.toSwerveModuleStates(
        chassisSpeeds);
    setModuleStates(moduleStates);

  }

  /**
   * takes a set of module states and sets individual modules to those states,
   * capping speeds to the maxmimum of the wheels
   * 
   * @param desiredStates - the individual module states coupled. FLD, FLT, FRD,
   *                      FRT, BLD, BLT, BRD, BRT
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates,
        getMaxDriveSpeed());
    
    modules[0].setDesiredState(desiredStates[0]);
    modules[1].setDesiredState(desiredStates[1]);
    modules[2].setDesiredState(desiredStates[2]);
    modules[3].setDesiredState(desiredStates[3]);

  }

  //this is only really relevant for testing purposes: as this is logged as endgoal position or smth like that
  //shows what the end position will be like in advantage scope
  public void showSetpointEndGoal() {
    setpointGoalStateLog.set(
        new Double[] { getPPSetpoint().setpoint.getX(), getPPSetpoint().setpoint.getY(),
            getPPSetpoint().setpoint.getRotation().getRadians() });
  }

  public void showApproachSetpointEndGoal() {
    setpointGoalStateLog.set(
        new Double[] { getPPSetpoint().approachPoint.getX(), getPPSetpoint().approachPoint.getY(),
            getPPSetpoint().approachPoint.getRotation().getRadians() });
  }

  /**
   * 
   * @param curPose the current pose of the robot in meters, used for measuring
   *                error
   * @param sample  the setpoint sample with position, velocity, acceleration, and
   *                forces
   * 
   * @see https://choreo.autos/choreolib/getting-started/#setting-up-the-drive-subsystem
   * 
   * @note verticle flipping relies on choreo detecting rotational symetry on the
   *       field
   */

   //called by OTF, given the position and velocity at the points generated in the dynamic path: 
   //calc the speeds and throw them into the speed
  public void followSample(Pose2d positions, Pose2d velocities) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        new ChassisSpeeds(
            xController.calculate(getPose().getX(), positions.getX()) + velocities.getX(),
            yController.calculate(getPose().getY(), positions.getY()) + velocities.getY(),
            turnController.calculate(getPose().getRotation().getRadians(), positions.getRotation().getRadians())
                + velocities.getRotation().getRadians()),
        getPose().getRotation());
        logSetpoints(positions, velocities);

    Robot.swerve.setChassisSpeeds(speeds);
  }

  public void followSample(SwerveSample sample, boolean isFlipped) {

    // ternaries are for x-axis flipping

    double xPos = sample.x;

    double xVel = sample.vx;
    double xAcc = sample.ax;
    double yPos = isFlipped ? AutoUtils.flipper.flipY(sample.y) : sample.y;

    double yVel = isFlipped ? -sample.vy : sample.vy;
    double yAcc = isFlipped ? -sample.ay : sample.ay;

    double heading = isFlipped ? new Rotation2d(Math.PI - sample.heading).rotateBy(new Rotation2d(Math.PI)).getRadians()
        : sample.heading;

    double omega = isFlipped ? -sample.omega : sample.omega;
    double alpha = isFlipped ? -sample.alpha : sample.alpha;

    Robot.swerve.logSetpoints(xPos, xVel, xAcc, yPos, yVel, yAcc, heading, omega, alpha);

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        new ChassisSpeeds(
            xController.calculate(getPose().getX(), xPos) + xVel,
            yController.calculate(getPose().getY(), yPos) + yVel,
            turnController.calculate(getPose().getRotation().getRadians(), heading) + omega),
        getPose().getRotation());

    Robot.swerve.setChassisSpeeds(speeds);
  }

  public void setBreakMode(boolean enable) {
    for (int i = 0; i < 4; i++) {
      modules[i].setBreakMode(enable);
    }
  }

  /**
   * Toggles whether or not vision updates odometry
   * 
   * @param utilize - whether or not to use vision updates on odometery, true is
   *                yes
   */
  public void setUtilizeVision(boolean utilize) {
    utilizeVision = utilize;
  }

  //this is only used for testing, pressing B to cycle through all of the stuff
  public void cyclePPSetpoint() {
    currentPPSetpointIndex++;
    if(JoystickIO.buttonBoard.getScoringMode()==ScoringMode.ALGAE && 
    currentPPSetpointIndex>=2&&currentPPSetpointIndex<=25)
    {
      setSetpointToClosestSideToSetpoint();
    }
    if(JoystickIO.buttonBoard.getScoringMode()==ScoringMode.L1 && 
    currentPPSetpointIndex>=2&&currentPPSetpointIndex<=24 && currentPPSetpointIndex%2==0)
    {
      currentPPSetpointIndex++;
    }
    if(JoystickIO.buttonBoard.getScoringMode()!=ScoringMode.L1 && 
    currentPPSetpointIndex>=3&&currentPPSetpointIndex<=25 && currentPPSetpointIndex%2!=0)
    {
      currentPPSetpointIndex++;
    }

    if (currentPPSetpointIndex >= ToPosConstants.Setpoints.PPSetpoints.values().length) {
      currentPPSetpointIndex = 0;
    }
  }

  public PPSetpoints getPPSetpoint() {
    return PPSetpoints.values()[currentPPSetpointIndex];
  }

  private void setSetpointToClosestSideToSetpoint() {
    Pose2d closestSide = getPPSetpoint().setpoint.nearest(ToPosConstants.Setpoints.reefSides);
    // Iterate through the reef branch mappings to set the correct setpoint
    for (Pose2d side : ToPosConstants.Setpoints.driveRelativeBranches.keySet()) {
        if (closestSide.equals(side)) {
              setPPSetpointIndex(ToPosConstants.Setpoints.driveRelativeBranches.get(side)[2]);
        }
    }
}


  //called when the button board is pressed with the (ppsetpoint)"index" the button is associated w to drive to

  public void startOnTheFly(int setpointIndex) {
    currentPPSetpointIndex = setpointIndex;

    if(JoystickIO.buttonBoard.getScoringMode()==ScoringMode.ALGAE && 
    currentPPSetpointIndex>=2&&currentPPSetpointIndex<=25)
    {
      setSetpointToClosestSideToSetpoint();
    }

    if(JoystickIO.buttonBoard.getScoringMode()==ScoringMode.L1 && 
    currentPPSetpointIndex>=2&&currentPPSetpointIndex<=24 && currentPPSetpointIndex%2==0)
    {
      currentPPSetpointIndex++;
    }
    //the ppsetpoints from 2 to 25 are the reef, and alternate between
    //L234 (even index) //L1 (odd index == L234 index+1)
    //if we're on l1, within the range, and on an equal index, add one to get to the L1 setpoint,
    //this becomes problematic only when switching between l1-l4 on testing, should be ok on the real bot

    if(JoystickIO.buttonBoard.getScoringMode()!=ScoringMode.L1 && 
    currentPPSetpointIndex>=3&&currentPPSetpointIndex<=25 && currentPPSetpointIndex%2!=0)
    {
      currentPPSetpointIndex++;
    }
    //js the same thing i said but L234 
    isOTF = true;
  }

  /**
   * Manually sets our odometry position
   * 
   * @param pose - Pose2d object of what to set our position to
   */
  public void setOdometry(Pose2d pose) {
    // System.out.println("Set Odometry: " + pose.getX() + ", " + pose.getY() + ", " + pose.getRotation().getDegrees());
    Rotation2d gyroHeading = Rotation2d.fromDegrees(gyroData.yawDeg);

    swerveDrivePoseEstimator.resetPosition(
        gyroHeading,
        new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        },
        pose);
  }

  public void setRotation() {
    modules[0].setTurnPosition(45 * (Math.PI / 180));
    modules[1].setTurnPosition(135 * Math.PI / 180);

    modules[2].setTurnPosition(225 * Math.PI / 180);
    modules[3].setTurnPosition(315 * Math.PI / 180);

  }

  /**
   * Updates our odometry position based on module encoders. Ran in Periodic
   */
  public void updateOdometry() {
    // convert to -pi to pi

    swerveDrivePoseEstimator.update(
        Rotation2d.fromDegrees(gyroData.yawDeg),
        new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        });
  }

  /**
   * Updates our odometry position based on vision. Called by vision subsystem
   */
  public void visionUpdateOdometry(Pose2d pose, double timestamp) {
    if (utilizeVision) {
      swerveDrivePoseEstimator.addVisionMeasurement(pose,
          timestamp);
    }
  }

  /**
   * Sets voltage to all swerve motors to 0
   */
  public void stopModules() {
    for (SwerveModule module : modules) {
      module.stop();
    }
  }

  /**
   * When called, makes the robot's current direction "forward"
   */
  public void resetGyro() {
    gyro.resetGyro();
    if (UtilityFunctions.isRedAlliance()) {
      swerveDrivePoseEstimator.resetPosition(new Rotation2d(), new SwerveModulePosition[] {
          modules[0].getPosition(),
          modules[1].getPosition(),
          modules[2].getPosition(),
          modules[3].getPosition()
      }, new Pose2d(swerveDrivePoseEstimator.getEstimatedPosition().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      swerveDrivePoseEstimator.resetPosition(new Rotation2d(), new SwerveModulePosition[] {
          modules[0].getPosition(),
          modules[1].getPosition(),
          modules[2].getPosition(),
          modules[3].getPosition()
      }, new Pose2d(swerveDrivePoseEstimator.getEstimatedPosition().getTranslation(), new Rotation2d()));
    }
  }

  public void logSetpoints(Pose2d position, Pose2d velocity) {
    // setpoint logging for automated driving
    Double[] positions = new Double[] { position.getX(), position.getY(), position.getRotation().getRadians() };
    setpointPositionLog.set(positions);

    Double[] velocities = new Double[] { velocity.getX(), velocity.getY(), velocity.getRotation().getRadians() };
    setpointVelocityLog.set(velocities);
    setpointAccelerationLog.set(new Double[] { 0.0, 0.0, 0.0 });

  }


  /**
   * logs all setpoints for the swerve subsystem in autonomous functions
   * 
   * @param the swerve sample of setpoints
   */
  public void logSetpoints(double posX, double velX, double accX, double posY, double velY, double accY, double heading,
      double omega, double alpha) {
    // setpoint logging for automated driving
    Double[] positions = new Double[] { posX, posY, heading };
    Logger.recordOutput("/subsystems/swerve/swerve position x", positions[0]);
    Logger.recordOutput("/subsystems/swerve/swerve position y", positions[1]);
    Logger.recordOutput("/subsystems/swerve/swerve position heading", positions[2]);

    Double[] velocities = new Double[] { velX, velY, omega };
    double velocity = 0;
    for (int i = 0; i < 2; i++) {
      velocity += Math.pow(velocities[i], 2);
    }
    velocity = Math.sqrt(velocity);
    // setpointVelocityLog.set(velocity);
    setpointRotationalVelocityLog.set(velocities[2]);
    Logger.recordOutput("/subsystems/swerve/setpoint velocity", velocity);
    Logger.recordOutput("/subsystems/swerve/velocity", velocities[2]);
    velocity = velocities[2];

    Double[] accelerations = new Double[] { accX, accY, alpha };
    double acceleration = 0;
    for (int i = 0; i < 2; i++) {
      acceleration += Math.pow(accelerations[i], 2);
    }
    acceleration = Math.sqrt(acceleration);
    // setpointAccelerationLog.set(acceleration);
    setpointRotationalAccelerationLog.set(accelerations[2]);
    Logger.recordOutput("/subsystems/swerve/setpoint acceleration", acceleration);
    Logger.recordOutput("/subsystems/swerve/setpoint rotational acceleration", accelerations[2]);

  }

  /**
   * log all serve data
   */
  private void logData() {
    // logging of our module states
    // Double[] realStates = {
    //     modules[0].getState().angle.getRadians(),
    //     modules[0].getState().speedMetersPerSecond,
    //     modules[1].getState().angle.getRadians(),
    //     modules[1].getState().speedMetersPerSecond,
    //     modules[2].getState().angle.getRadians(),
    //     modules[2].getState().speedMetersPerSecond,
    //     modules[3].getState().angle.getRadians(),
    //     modules[3].getState().speedMetersPerSecond
    // };

    // Double[] desiredStates = {
    //     modules[0].getDesiredState().angle.getRadians(),
    //     modules[0].getDesiredState().speedMetersPerSecond,
    //     modules[1].getDesiredState().angle.getRadians(),
    //     modules[1].getDesiredState().speedMetersPerSecond,
    //     modules[2].getDesiredState().angle.getRadians(),
    //     modules[2].getDesiredState().speedMetersPerSecond,
    //     modules[3].getDesiredState().angle.getRadians(),
    //     modules[3].getDesiredState().speedMetersPerSecond
    // };

    // Logger.recordOutput("/subsystems/swerve/real states", realStates);
    // Logger.recordOutput("/subsystems/swerve/desired states", desiredStates);
    
    double[] odometry = {
        getPose().getX(),
        getPose().getY(),
        getPose().getRotation().getRadians()};
    Logger.recordOutput("/subsystems/swerve/odometry", odometry);
    Logger.recordOutput("/subsystems/swerve/utilize vision", utilizeVision);

    // gyro logging
    Logger.recordOutput("/subsystems/swerve/rotational velocity", (gyroData.yawDeg - yaw) / 0.02);
    Logger.recordOutput("/subsystems/swerve/gyro yaw", gyroData.yawDeg);
    yaw = gyroData.yawDeg;
    Logger.recordOutput("/subsystems/swerve/gyro pitch", gyroData.pitchDeg);
    Logger.recordOutput("/subsystems/swerve/gyro roll", gyroData.rollDeg);
    Logger.recordOutput("/subsystems/swerve/is gyro connected", gyroData.isConnected);
    Logger.recordOutput("/subsystems/swerve/gyro rotation", getRotation2d().getDegrees());

    // velocity and acceleration logging
    double robotVelocity = Math.hypot(getChassisSpeeds().vxMetersPerSecond,
        getChassisSpeeds().vyMetersPerSecond);
    Logger.recordOutput("/subsystems/swerve/gyro rotation", (robotVelocity - velocity) / .02);
    Logger.recordOutput("/subsystems/swerve/gyro rotation", robotVelocity);
    String currentCommand = this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName();
    Logger.recordOutput("/subsystems/swerve/current command", currentCommand);

    AutoConstants.kPDrive = kPDriving.get();
    AutoConstants.kDDrive = kDDriving.get();
    AutoConstants.kPTurn = kPTurn.get();
    AutoConstants.kDTurn = kDTurn.get();
  }

  @Override
  public void periodic() {
    gyro.updateData(gyroData);
    updateOdometry();

    // periodic method for individual modules
    for (int i = 0; i < 4; i++) {
      modules[i].periodic();
    }

    logData();

  }

}