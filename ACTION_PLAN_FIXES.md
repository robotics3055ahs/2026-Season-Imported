# Action Plan: Fixing Your Swerve Drive

## Quick Diagnosis

Your swerve drive isn't working because:

1. **No built-in PID on the motors** - You're using RoboRIO PID at 50Hz instead of TalonFX PID at 1kHz
2. **CANcoder is not configured** - The turn motor doesn't know to use it as feedback
3. **No MotionMagic** - Modules rotate erratically instead of smoothly
4. **CAN bus congestion** - Sending 16 frames per cycle instead of 2-3
5. **Broken odometry** - Getting stale single samples instead of synchronized multi-sample packets

---

## Immediate Actions (Priority Order)

### ⚠️ CRITICAL - Fix This First

#### 1. Configure Motor Controllers Properly

**File:** `SwerveModule.java`

Currently you have:
```java
m_driveMotor = new TalonFX(driveMotorChannel);
m_turningMotor = new TalonFX(turningMotorChannel);
```

You need to add Phoenix 6 configuration. **Best approach:** Use AdvantageKit's structure with a generated TunerConstants class from Phoenix Tuner X.

**Option A (Recommended):** Use AdvantageKit's approach
- Run Phoenix Tuner X on your robot
- Generate TunerConstants.java
- Use SwerveModuleConstants for all your configuration
- Let Phoenix handle the constants

**Option B (Quick Fix):** Manually configure for now
```java
public SwerveModule(...) {
    // ... existing code ...
    
    // Configure drive motor
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput.Inverted = /* your inversion */;
    driveConfig.CurrentLimits.StatorCurrentLimit = 60;  // Adjust as needed
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // Apply PID gains (replace with tuned values)
    driveConfig.Slot0.kP = 0.1;
    driveConfig.Slot0.kI = 0.001;
    driveConfig.Slot0.kD = 0.001;
    m_driveMotor.getConfigurator().apply(driveConfig);
    m_driveMotor.setPosition(0.0);

    // Configure turn motor
    TalonFXConfiguration turnConfig = new TalonFXConfiguration();
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.MotorOutput.Inverted = /* your inversion */;
    turnConfig.Feedback.FeedbackRemoteSensorID = turningEncoderChannels;
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turnConfig.Feedback.RotorToSensorRatio = /* your gear ratio */;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    // Apply PID gains
    turnConfig.Slot0.kP = 0.1;
    turnConfig.Slot0.kI = 0.001;
    turnConfig.Slot0.kD = 0.001;
    // MotionMagic for smooth turning
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0;  // rot/sec
    turnConfig.MotionMagic.MotionMagicAcceleration = 1000.0;   // rot/sec²
    turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12;
    turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    m_turningMotor.getConfigurator().apply(turnConfig);
}
```

#### 2. Fix CANcoder Configuration

**File:** `SwerveModule.java` constructor

Add CANcoder configuration:
```java
// Configure CANcoder
CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
cancoderConfig.MagnetSensor.MagnetOffset = 0.0;  // TODO: Calibrate per module!
cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
m_turningEncoderNew.getConfigurator().apply(cancoderConfig);
```

#### 3. Replace Control Requests

**File:** `SwerveModule.java`

Remove:
```java
private final ProfiledPIDController m_drivePIDController = ...;
private final ProfiledPIDController m_turningPIDController = ...;
private final SimpleMotorFeedforward m_driveFeedforward = ...;
```

Add:
```java
private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0.0);
private final PositionVoltage turnPositionRequest = new PositionVoltage(0.0);
```

Replace `setDesiredState()`:
```java
public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(m_turningEncoderNew.getPosition().getValue());
    
    // Optimize the reference state
    desiredState.optimize(encoderRotation);
    desiredState.cosineScale(encoderRotation);
    
    // Convert m/s to rot/sec for the drive motor
    double wheelRadius = ModuleConstants.kWheelDiameterMeters / 2.0;
    double driveVelocityRadPerSec = desiredState.speedMetersPerSecond / wheelRadius;
    double driveVelocityRotPerSec = driveVelocityRadPerSec / (2.0 * Math.PI);
    
    // Command motors using Phoenix 6 control requests
    m_driveMotor.setControl(driveVelocityRequest.withVelocity(driveVelocityRotPerSec));
    m_turningMotor.setControl(turnPositionRequest.withPosition(desiredState.angle.getRotations()));
}
```

---

### 🔴 CRITICAL - Implement Odometry Thread

**File:** `DriveSubsystem.java`

This is the **most important change** for odometry accuracy.

```java
public class DriveSubsystem extends SubsystemBase {
    // ... existing code ...
    
    @Override
    public void periodic() {
        // Call PhoenixOdometryThread to process queued odometry data
        PhoenixOdometryThread.getInstance().scheduleFrameUpdate(
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_rearLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearRight.getPosition()
            }
        );
        
        // Update odometry with vision (if enabled)
        if(m_vision != null){
            m_vision.updateCamera();
            var visionPoseEstimate = m_vision.getEstimatedGlobalPose();
            if(visionPoseEstimate != null){
                m_odometry.addVisionMeasurement(
                    visionPoseEstimate.estimatedPose.toPose2d(), 
                    visionPoseEstimate.timestampSeconds
                );
            }
        }
        
        // Update odometry with gyro
        m_odometry.update(       
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_rearLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearRight.getPosition()
            }
        );
        
        posePublisher.set(m_odometry.getEstimatedPosition());
    }
}
```

---

## Phase 2 Actions (After Phase 1 works)

### 📊 Optimize CAN Bus Usage

**File:** `SwerveModule.java` constructor

Add after configuring motors:
```java
// Set update frequencies to reduce CAN traffic
BaseStatusSignal.setUpdateFrequencyForAll(
    250.0,  // Odometry signals at 250Hz
    m_driveMotor.getPosition(),
    m_turningMotor.getPosition()
);

BaseStatusSignal.setUpdateFrequencyForAll(
    50.0,   // Monitoring signals at 50Hz
    m_driveMotor.getVelocity(),
    m_turningMotor.getVelocity(),
    m_driveMotor.getMotorVoltage(),
    m_turningMotor.getMotorVoltage(),
    m_driveMotor.getStatorCurrent(),
    m_turningMotor.getStatorCurrent(),
    m_turningEncoderNew.getAbsolutePosition()
);

// Optimize bus
ParentDevice.optimizeBusUtilizationForAll(m_driveMotor, m_turningMotor);
```

### 🔧 Update Constants

**File:** `Constants.java` - Module Constants section

```java
public static final class ModuleConstants {
    // Remove these old PID constants
    // public static final double kPModuleDriveController = 7.0;
    // public static final double kIModuleDriveController = 0.001;
    // public static final double kDModuleDriveController = 0.001;
    // public static final double ksModuleDriveController = 0.0;
    // public static final double kvModuleDriveController = 0.0;
    
    // Add new motor controller PID gains (tuned via Phoenix Tuner X)
    public static final double kPDriveController = 0.1;  // TODO: Tune
    public static final double kIDriveController = 0.0;
    public static final double kDDriveController = 0.0;
    
    public static final double kPTurnController = 0.1;   // TODO: Tune
    public static final double kITurnController = 0.0;
    public static final double kDTurnController = 0.0;
    
    // Wheel and gear ratio constants
    public static final double kWheelDiameterMeters = 0.102;
    public static final double kDriveMotorGearRatio = 6.75;  // (e.g., MK4i L1)
    public static final double kTurningMotorGearRatio = /* your ratio */;
    
    // MotionMagic for turning
    public static final double kMaxTurnVelocityRotPerSec = 100.0;
    public static final double kMaxTurnAccelerationRotPerSec2 = 1000.0;
}
```

---

## Phase 3 Actions (After Initial Testing)

### 🎯 Tune PID Gains

Use Phoenix Tuner X to characterize your drive:

1. Connect to robot with Phoenix Tuner X
2. Go to Devices → Select TalonFX
3. Device Config → Tuner → Characterize
4. Follow the tuning wizard
5. Extract gains and update your code

### 📐 Calibrate CANcoder Offsets

For each swerve module:
1. Point wheels straight forward
2. Use Phoenix Tuner X to read CANcoder value
3. Record as `MagnetOffset` in configuration
4. Repeat for all 4 modules

### ✅ Test Sequence

1. **Static Tests**
   - Robot disabled: Verify wheel positions are correct
   - Enable: Verify wheels can rotate
   - Command: Verify velocity mode works

2. **Movement Tests**
   - Forward/backward straight
   - Rotation in place
   - Diagonal movement
   - Combined translation + rotation

3. **Odometry Tests**
   - Drive in a square
   - Check if pose returns to origin
   - Verify heading is correct

---

## Troubleshooting Guide

### Symptom: Modules don't turn
**Cause:** CANcoder feedback not configured
**Fix:** Check that `turnConfig.Feedback.FeedbackSensorSource` is set to `RemoteCANcoder`

### Symptom: Robot goes uncontrolled
**Cause:** High CAN latency from bus congestion
**Fix:** Implement CAN optimization and reduce polling

### Symptom: Turning is jerky
**Cause:** No MotionMagic or poor PID tuning
**Fix:** Enable MotionMagic and tune PID gains

### Symptom: Odometry drifts badly
**Cause:** Single stale samples instead of multi-sample synchronized data
**Fix:** Implement PhoenixOdometryThread

### Symptom: Motor won't hold position when disabled
**Cause:** NeutralMode not set to Brake
**Fix:** Add `config.MotorOutput.NeutralMode = NeutralModeValue.Brake`

---

## Files to Modify (Priority Order)

1. **SwerveModule.java** - Motor configuration, control requests, feedback setup
2. **DriveSubsystem.java** - Odometry thread integration
3. **Constants.java** - Update PID gains and add new constants
4. **imports** - Add new Phoenix 6 classes

---

## Key Imports Needed

```java
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.BaseStatusSignal;
```

---

## Timeline

- **Immediate (Today):** Motor configuration + CANcoder setup
- **Soon (This week):** Implement odometry thread
- **Next (Next week):** Test and debug
- **Follow-up (Ongoing):** Tune PID gains and offsets

This is a significant rewrite, but your drive will work **dramatically** better after these changes. The main insight: **Let the motor controllers do their job** instead of trying to control everything from the RoboRIO.
