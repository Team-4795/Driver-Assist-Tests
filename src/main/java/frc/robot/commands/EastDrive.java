// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EastDrive extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final Supplier<Double> m_xaxisSpeedSupplier;
  private final Supplier<Double> m_zaxisRotateSupplier;
  private final Supplier<Double> m_throttle;
  private final Supplier<Boolean> m_reverse;
  private long lastSpeedPress;
  private double targetDirection;
  private double forward = 1.0;

  public EastDrive(
      Drivetrain drivetrain,
      Supplier<Double> xaxisSpeedSupplier,
      Supplier<Double> zaxisRotateSupplier,
      Supplier<Double> throttle,
      Supplier<Boolean> reverse) {
    m_drivetrain = drivetrain;
    m_xaxisSpeedSupplier = xaxisSpeedSupplier;
    m_zaxisRotateSupplier = zaxisRotateSupplier;
    m_throttle = throttle;
    m_reverse = reverse;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastSpeedPress = 0;
    targetDirection = m_drivetrain.getGyroAngleZ();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = m_xaxisSpeedSupplier.get();
    double rotation = m_zaxisRotateSupplier.get();
    double throttle = m_throttle.get();
    boolean reverse = m_reverse.get();
    double angle = m_drivetrain.getGyroAngleZ();

    SmartDashboard.putNumber("Reverse", forward);

    throttle = (1.0 - throttle * 0.75);

    if(reverse) forward *= -1.0;

    speed *= forward;

    if(Math.abs(speed) > 0) lastSpeedPress = System.currentTimeMillis();
    if(Math.abs(rotation) > 0) targetDirection = angle;

    double adjust = 0;//targetDirection - angle;

    if(Math.abs(adjust) > 1.75) {
      adjust = Math.signum(adjust) * 0.06;
    } else {
      adjust = 0.0;
    }
    
    speed *= throttle;
    
    if(speed == 0) {
      double transitionRamp = Math.min(Math.max((System.currentTimeMillis() - lastSpeedPress) / 500.0, 0.5), 1.0);
      rotation *= Math.max(throttle, 0.4) * transitionRamp * 0.6;
      m_drivetrain.curvatureDrive(speed, rotation, true);
    } else {
      m_drivetrain.curvatureDrive(speed, rotation + adjust, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
