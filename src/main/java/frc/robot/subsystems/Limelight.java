
package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Util.RectanglePoseArea;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
  CommandSwerveDrivetrain drivetrain;
  Alliance alliance;
  private String ll = "limelight";
  private Boolean enable = false;
  private Boolean trust = false;
  private int fieldError = 0;
  private int distanceError = 0;
  private Pose2d botpose;
//   private static final RectanglePoseArea field =
//         new RectanglePoseArea(new Translation2d(0.0, 0.0), new Translation2d(16.54, 8.02));

  /** Creates a new Limelight. */
  public Limelight(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    SmartDashboard.putNumber("Field Error", fieldError);
    SmartDashboard.putNumber("Limelight Error", distanceError);
  }

  @Override
  public void periodic() {
    


}
      


  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
  }

  public void useLimelight(boolean enable) {
    this.enable = enable;
  }

  public void trustLL(boolean trust) {
    this.trust = trust;
  }
}