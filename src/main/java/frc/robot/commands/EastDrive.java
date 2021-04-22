// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;

public class EastDrive extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final Supplier<Double> m_xaxisSpeedSupplier;
  private final Supplier<Double> m_zaxisRotateSupplier;
  private final Supplier<Double> m_throttle;
  private long lastSpeedPress;
  private double targetDirection;

  public EastDrive(
      Drivetrain drivetrain,
      Supplier<Double> xaxisSpeedSupplier,
      Supplier<Double> zaxisRotateSupplier,
      Supplier<Double> throttle) {
    m_drivetrain = drivetrain;
    m_xaxisSpeedSupplier = xaxisSpeedSupplier;
    m_zaxisRotateSupplier = zaxisRotateSupplier;
    m_throttle = throttle;
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
    double angle = m_drivetrain.getGyroAngleZ();

    throttle = (1.0 - throttle * 0.75);

    if(Math.abs(speed) > 0) lastSpeedPress = System.currentTimeMillis();
    if(Math.abs(rotation) > 0) targetDirection = angle;

    double adjust = targetDirection - angle;

    if(adjust > 1) {
      adjust = Math.signum(adjust) * 0.1;
    } else {
      adjust = 0.0;
    }
    
    speed *= throttle;
    
    if(speed == 0) {
      double rampMultiplier = Math.min(Math.max((System.currentTimeMillis() - lastSpeedPress) / 500.0, 0.25), 1.0);
      rotation *= throttle * rampMultiplier;
      m_drivetrain.arcadeDrive(speed, rotation);
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
