// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import java.util.Arrays;
import org.frcteam3539.Byte_Swerve_Lib.control.HolonomicMotionProfiledTrajectoryFollower;
import org.frcteam3539.Byte_Swerve_Lib.control.PidConstants;
import org.frcteam3539.Byte_Swerve_Lib.control.Trajectory;
import org.frcteam3539.Byte_Swerve_Lib.util.DrivetrainFeedforwardConstants;
import org.frcteam3539.Byte_Swerve_Lib.util.HolonomicFeedforward;

public class DriveSubsystem extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
	/** Creates a new DrivetrainSubsystem. */
	private SwerveRequest swerveRequest = new SwerveRequest.Idle();

	private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();


	 public double maxVelocity = 0.0;
	 public double maxRotationalVelocity = 0.0;

	public Pigeon2 pigeon = new Pigeon2(Constants.pigeonID, "rio");


	public DriveSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
		super(TalonFX::new, TalonFX::new, CANcoder::new, driveTrainConstants, modules);

		maxVelocity = modules[0].SpeedAt12Volts;

		Translation2d[] moduleLocations = new Translation2d[modules.length];

		for (int i = 0; i < modules.length; i++) {
			moduleLocations[i] = new Translation2d(modules[i].LocationX, modules[i].LocationY);
			this.getModule(i).getDriveMotor().getConfigurator()
				.apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(40)
                        .withSupplyCurrentLimitEnable(true)
							.withStatorCurrentLimit(modules[i].SlipCurrent).withStatorCurrentLimitEnable(true));
		}

		double dtRadius = new Translation2d().nearest(Arrays.asList(moduleLocations)).getDistance(new Translation2d());
		maxRotationalVelocity = (maxVelocity / dtRadius);

		configureAutoBuilder();
	}


	// public void seedFieldRelative(Trajectory trajectory) {
	// 	this.seedFieldRelative(trajectory.calculate(0).getPathState().getPose2d());
	// }

	public Pose2d getPose2d() {
		return this.getState().Pose;
	}

	public void applyRequest(SwerveRequest request) {
		this.swerveRequest = request;
	}

	public void log() {
	}
private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(Constants.TranslationkP, Constants.TranslationkI, Constants.TranslationkD),
                    // PID constants for rotation
                    new PIDConstants(Constants.RotationkP, Constants.RotationkI, Constants.RotationkD)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

	@Override
	public void periodic() {
		SwerveRequest request = new SwerveRequest.Idle();
			request = swerveRequest;

		this.setControl(request);
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("/DriveTrain/BatteryVoltage", RobotController.getBatteryVoltage());

	}
}
