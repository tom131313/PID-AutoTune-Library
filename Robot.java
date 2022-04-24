package frc.robot;

// https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/tree/master/Java%20Talon%20FX%20(Falcon%20500)

// auto tune PID controller
// coding heavily depends on speed being -1 to +1 -- %VBus and Talon native velocity units encoder ticks/100 ms
// also built-in 1 ms Talon differentiator/integrator period and 1023 throttle units max voltage

// changes could include setting battery voltage compensation to say 11 or 12 volts. More reproducible but slower response the lower the voltage.

import java.util.function.Supplier;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PID_ATune.CONTROL_TYPE;
import frc.robot.PID_ATune.DIRECTION;

public class Robot extends TimedRobot {
  private static final int sampleTime = 5; // milliseconds loop sample time period
  private static final int TIMEOUT_MS = 30;
  // stubs for unit conversions
  Supplier<Double> getFlywheelSpeed;
  double KuUnitsConversion;
  double PuUnitsConversion;
  
  // private static TalonFX flywheelMotor;
  private static TalonSRX flywheelMotor;
  
  double RPM_per_VelocityUnit;
 
  private PID_ATune tuner;
  private long millisStart;
  // control (output) signal limits
  double controllerMin = 0., controllerMax = 1.; // FIXME
  // double controllerMin = -1., controllerMax = 1.;
  double output= 0.30; // center of the relay pulse %vBus for Talon FIXME
  double oStep = 0.15; // + and - step size for the output perturbation (relay)
  double setpoint; // below we'll find out the velocity we get for the output signal specified above [native units]
  // leave everything (almost) in native units so no conversion required
  boolean tuning = true;

  int pidIdx = 0; // Talon primary closed loop control
  int slotIdx = 0; // Talon index to select which set of k's to use
  double[] kP = new double[4];
  double[] kI = new double[4];
  double[] kD = new double[4];
  double[] kF = new double[4];

  StripChart myChart;

  RunningRegression aLine = new RunningRegression(); // construct regression object used for calculating kF

  double flipNearZero = Double.MIN_NORMAL; // trick for the SmartDashboard line plot

  Robot()
  {
    super((double)sampleTime/1000.); // set the robot loop time
    LiveWindow.disableAllTelemetry();
  }

  @Override
  public void robotInit()
  {
    configFlywheelMotor(); //FIXME: uncomment the right one and comment out all the others
  }

  @Override
  public void teleopInit() {

    kF[slotIdx] = 0.96*computeKf(); // reduce the full kF a little - close for a flywheel but gives the controller a little room to control
    
    stabilizeInitialSpeed();

    tuner = new PID_ATune( setpoint, output, oStep, DIRECTION.DIRECT, CONTROL_TYPE.PID_CONTROL, sampleTime );
    
    //   S E T U P   S T R I P C H A R T   T O   D I S P L A Y   A C T I O N S
    // center of left process variable graph; minimum value of right controller graph; maximum value of right controller graph
    myChart = new StripChart( setpoint,	output-oStep,	output+oStep );

    millisStart = System.currentTimeMillis(); // start time now will be relative 0;
  }

  @Override
  public void teleopPeriodic()
  {
    if(!tuning) return;

      double speed = getFlywheelSpeed.get(); // get the speed of the current output
      int loopStartTime = (int)(System.currentTimeMillis() - millisStart);
      int rc = tuner.Runtime( speed, loopStartTime );
      // 0 still looking for peaks
      // 1 stop - didn't find peaks; give up; or okay - peaks found and tuned
      // 2 waiting for time; no processing
      // 3 tuned but keep going
      //System.out.println("tuner ret " + rc);

      switch (rc) // 0 took a time step; 1 done tuning; 2 called faster than sample rate; 3 intermediate parameters available
      {
      case 1:  // tuning just completed; mark that event, print the peak, and move on
        tuning = false;
      case 3:  // time step okay, print the peak, and process time step
        System.out.format("  peaks %d-%d, Ku=%.5f, Pu=%.5f, Kp=%.5f, Ki=%.5f, Kd=%.5f %%VBus/velocity\n", 
          (int)(1000.*tuner.GetPeak_1()), (int)(1000.*tuner.GetPeak_2()), tuner.GetKu(), tuner.GetPu(), tuner.GetKp(), tuner.GetKi(), tuner.GetKd());

        // Ku ultimate gain of the controller - units of process input/process output
        // Pu ultimate period - units of tuning time (tuner uses milliseconds but returns seconds)
        // convert to whatever the motor controller units are
				kP[slotIdx] = tuner.GetKp() * KuUnitsConversion;
				kI[slotIdx] = tuner.GetKi() * KuUnitsConversion/PuUnitsConversion;
				kD[slotIdx] = tuner.GetKd() * KuUnitsConversion*PuUnitsConversion;
				System.out.println("[Talon] PID " + slotIdx + ", Kp = " + kP[slotIdx] + ", Ki = " + kI[slotIdx] + ", Kd = " + kD[slotIdx] + ", kF = " + kF[slotIdx] + "\n");
      case 0:  // time step okay, process time step
        break;
      case 2:  // too fast, skipping this step
        return;
      default:
        System.err.println("\n\nUnknown return from Runtime()\n\n");
      }

      output = tuner.getOutput(); // get the new output
      setFlywheelSpeed(output); // set a new speed using the new output

      // time in milliseconds;	process output variable velocity /\/\/\/\;	process input such as %VBus - the step function _-_-_-_-
      System.out.print("\n" + myChart.PrintStripChart( loopStartTime,	speed, output ) );
      SmartDashboard.putNumber("speed", speed);

      if(!tuning)
      {
        setFlywheelSpeed(0.); // stop
      }
  }      

  @Override
  public void disabledInit()
  {
    // not sure if clear needed in this version - was needed in some previous circumstances
    System.out.println("[Talon] get kI accum " + flywheelMotor.getIntegralAccumulator(pidIdx));
    System.out.println("[Talon] clear kI accum " + flywheelMotor.setIntegralAccumulator(0., pidIdx, TIMEOUT_MS));
  }

  @Override
  public void autonomousInit()
  {
    // set the Talon PID constants calculated during teleop
    flywheelMotor.selectProfileSlot(slotIdx, pidIdx);

    System.out.println("[Talon] set kF " + flywheelMotor.config_kF(0, kF[slotIdx], TIMEOUT_MS));
    System.out.println("[Talon] set kP " + flywheelMotor.config_kP(0, kP[slotIdx], TIMEOUT_MS));
    System.out.println("[Talon] set kI " + flywheelMotor.config_kI(0, kI[slotIdx], TIMEOUT_MS));
    System.out.println("[Talon] set kD " + flywheelMotor.config_kD(0, kD[slotIdx], TIMEOUT_MS));
  }
  
  @Override
  public void autonomousPeriodic()
  {
    // run the motor with PID control
    flywheelMotor.set(TalonSRXControlMode.Velocity, setpoint);
    var d = flywheelMotor.getClosedLoopError(pidIdx)*RPM_per_VelocityUnit;
    var s = getFlywheelSpeed.get()*RPM_per_VelocityUnit;
    System.out.println("setpoint RPM " + setpoint*RPM_per_VelocityUnit + ", actual RPM " + s + ", error RPM " + d);
    SmartDashboard.putNumber("RPM", s);
    SmartDashboard.putNumber("%VBus", flywheelMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("RPM error", d==0?(flipNearZero=-flipNearZero):d); // trick SmartDashboard plot into thinking value is changing if 0
  }

  // private void configFlywheelMotor() // TalonFX
  // {
  //     int flywheelMotorPort=13;
  //     flywheelMotor = new TalonFX(flywheelMotorPort);
  //     System.out.println("[Talon] set factory default " + flywheelMotor.configFactoryDefault(TIMEOUT_MS));
  //     flywheelMotor.setInverted(false);
  //     flywheelMotor.setNeutralMode(NeutralMode.Coast);
  //     System.out.println("[Talon] set vel period " + flywheelMotor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms, TIMEOUT_MS));
  //     System.out.println("[Talon] set vel window " + flywheelMotor.configVelocityMeasurementWindow(1, TIMEOUT_MS));
  //     FeedbackDevice sensor = FeedbackDevice.IntegratedSensor;
  //     System.out.println("[Talon] set feedback sensor "  + sensor.toString() + " " + flywheelMotor.configSelectedFeedbackSensor(sensor, 0, TIMEOUT_MS));
  //     flywheelMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, TIMEOUT_MS);
  //     System.out.println("[Talon] set status 13 " + flywheelMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, sampleTime, TIMEOUT_MS)); // PID error
  //     flywheelMotor.setSensorPhase(false);
  //     flywheelMotor.setSelectedSensorPosition(0);
  
  //     // Falcon with TalonFX  
  //     kUnitsPerRevolution = 2048; /* this is constant for Talon FX integrated sensor */
  //     double FLYWHEEL_GEAR_RATIO = 60.0 / 24.0;
  //     RPM_per_VelocityUnit = // multiplicative factor to convert motor controller units to engineering units; for Talon it's really RPM_per_TICK_PER_100MS
  //       1./kUnitsPerRevolution * // rev/tick
  //       10.      * // 100ms/sec
  //       60.      * // sec/min
  //       FLYWHEEL_GEAR_RATIO;

  //     getFlywheelSpeed = () -> flywheelMotor.getSelectedSensorVelocity(pidIdx);

  //     //////////////////////////////////////////////////////////////////////////////////
  //     //            START Conversion to Talon units
  //     //
  //     // Tuned with %VBus/velocity unit and 1 second time base
  //     // if controller integrator/differentiator time step not one unit of time of tuner then convert time units
  //     // (K constants and integrator/differentiator delta time must be consistent with tuner time)
  //     // Talon PID controller internally uses:
  //     //    throttle units/encoder edges per 100ms
  //     //    millisecond integrator/differentiator time steps
  //     //           1023 throttle units per %VBus / (2048 encoder pulses per motor rev
  //     //           * 60 / 24 motor revs per gear box output shaft rev /
  //     //           (60 seconds per minute * 10 100ms per seconds))
  //     //				double KuUnitsConversion = 1023. / (( 2048 * 60 / 24 ) / ( 60. * 10. ));
  //     // Tuned with %VBus/velocity units and 1 second time base (after conversations from milliseconds in autotune)
  //     // thus only need to account for the 1023 throttle units/%VBus and the 1000 ms/second.
  //     // Talon uses 1023 ticks/%VBus and 1 millisecond for integration and differentiation loop
  //     KuUnitsConversion = 1023.; // 1023 throttle units per %VBus (no change made to the native velocity units since that was used for tuning)
  //     PuUnitsConversion = 1000.; // 1000 milliseconds per second
  //     //
  //     //            END Conversion to Talon units
  //     /////////////////////////////////////////////////////////////////////////////////

  // }

  private void configFlywheelMotor() // TalonSRXNeveRest20
  {
      int flywheelMotorPort=0;
      flywheelMotor = new TalonSRX(flywheelMotorPort);
      System.out.println("[Talon] set factory default " + flywheelMotor.configFactoryDefault(TIMEOUT_MS));
      flywheelMotor.setInverted(true);
      flywheelMotor.setNeutralMode(NeutralMode.Brake); // return energy to battery and ease driving/steering
      // period 20ms or 25ms fastest a slow motor/encoder can run otherwise no counts per period
      // averaging window 1 or 2 okay for slow motor/encoder if not noisy which I have never seen
      System.out.println("[Talon] set vel period " + flywheelMotor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms, TIMEOUT_MS));
      System.out.println("[Talon] set vel window " + flywheelMotor.configVelocityMeasurementWindow(1, TIMEOUT_MS));
      FeedbackDevice sensor = FeedbackDevice.QuadEncoder;
      System.out.println("[Talon] set feedback sensor " + sensor.toString() + " " + flywheelMotor.configSelectedFeedbackSensor(sensor, 0, TIMEOUT_MS));
      System.out.println("[Talon] set nominal output reverse " + flywheelMotor.configNominalOutputReverse(0., TIMEOUT_MS));
      System.out.println("[Talon] set status 2 " + flywheelMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, sampleTime, TIMEOUT_MS));
      System.out.println("[Talon] set status 13 " + flywheelMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, sampleTime, TIMEOUT_MS)); // PID error
      // flywheelMotor.setSensorPhase(false);
      flywheelMotor.setSelectedSensorPosition(0);

      // NeveRest 20:1 motor/gearbox   0.2 %VBus is 60 rpm
      RPM_per_VelocityUnit = // multiplicative factor to convert motor controller units to engineering units; for Talon it's really RPM_per_TICK_PER_100MS
        1./7./4. * // 1 revolution of the motor shaft per 7 encoder pulses per 4 quadrature edges per pulse
                   // (1 pulse is encoder signal A leading and trailing edges and B leading and trailing edges)
        10.      * // 100ms/sec
        60.      * // sec/min
        1./20.;    // NeveRest internal reduction gears 1 output shaft revolution per 20 motor revolutions

      getFlywheelSpeed = () -> flywheelMotor.getSelectedSensorVelocity(pidIdx);

      //////////////////////////////////////////////////////////////////////////////////
      //            START Conversion to Talon native units
      //
      // Tuned with %VBus/velocity unit and 1 second time base
      // if controller integrator/differentiator time step not one unit of time of tuner then convert time units
      // (K constants and integrator/differentiator delta time must be consistent with tuner time)
      // Talon PID controller internally uses:
      //    throttle units/encoder edges per 100ms
      //    millisecond integrator/differentiator time steps
      //           1023 throttle units per %VBus / (4 edges per quadrature encoder pulse * 7 encoder pulses per motor rev
      //           * 20 motor revs per gear box output shaft rev /
      //           (60 seconds per minute * 10 100ms per seconds))
      //				double KuUnitsConversion = 1023. / (( 4. * 7. * 20 ) / ( 60. * 10. ));
      // Tuned with %VBus/velocity units and 1 second time base (after conversations from milliseconds in autotune)
      // thus only need to account for the 1023 throttle units/%VBus and the 1000 ms/second.
      // Talon uses 1023 ticks/%VBus and 1 millisecond for integration and differentiation loop
      KuUnitsConversion = 1023.; // 1023 throttle units per %VBus (no change made to the native velocity units since that was used for tuning)
      PuUnitsConversion = 1000.; // 1000 milliseconds per second
      //
      //            END Conversion to Talon native units
      //////////////////////////////////////////////////////////////////////////////////
 
      // get and display the motor parms
      TalonSRXConfiguration allConfigs = new TalonSRXConfiguration();
      flywheelMotor.getAllConfigs(allConfigs, TIMEOUT_MS);
      System.out.println("[Talon] flywheel motor configs\n" + allConfigs);
    }
    
  /**
   * 
   * @param speed %VBus -1 to +1
   */
  public void setFlywheelSpeed(double speed)
  {
      flywheelMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * compute one kF assuming it's the same over the selected range.
   * More sophistication is possible -LUT kF vs speed
   */
  public double computeKf()
  {
    double minPctVBus = 0.2; // minimum (probably inclusive) and maximum (probably not inclusive) %VBus to be considered for kF
    double maxPctVBus = 0.81; // if you want more precision on stepping, use int and multiply by the scale factor
    setFlywheelSpeed(minPctVBus); // start the motor at the minimum
    Timer.delay(1.); // wait to help stabilize at minimum speed starting from 0, we are hopeful, more wait below
    // get velocity many times at each of several %VBus and least squares fit them all
    for(double rampPctVBus = minPctVBus; rampPctVBus < maxPctVBus; rampPctVBus+=0.05)
    {
      setFlywheelSpeed(rampPctVBus);
      Timer.delay(3.); // wait to stabilize speed, we are hopeful
      for(int countSamplesAtPctVBus = 1; countSamplesAtPctVBus <= 500; countSamplesAtPctVBus++)
      {
        aLine.Push(rampPctVBus, getFlywheelSpeed.get());
        Timer.delay((double)sampleTime/1000.);
      }
      System.out.println("%VBus " + rampPctVBus + ", velocity (native units) " + getFlywheelSpeed.get() + ", velocity RPM " + getFlywheelSpeed.get()*RPM_per_VelocityUnit);
    }
    var kF = 1023./aLine.Slope(); // kF in native units related to %VBus/velocity units
    System.out.println("full kF = " + kF);
    return kF;
  }

  public void stabilizeInitialSpeed() 
  {
    // start tuning at the center velocity
    // Tuned with %VBus/velocity units of gearbox shaft output
    setFlywheelSpeed(output);

    try {Thread.sleep(5000);} // let motor speed stabilize
        catch (InterruptedException e) {e.printStackTrace();}

    //get the (average) speed (setpoint or input) at this output (power level)
    int numSamples = 20;
    setpoint = 0;
    for(int idx =1; idx <=numSamples; idx++)
    {
        var speed = getFlywheelSpeed.get();
        setpoint += speed;
        System.out.println("speed " + speed + " " + setpoint);
        try {Thread.sleep(100);} // let motor speed stabilize
          catch (InterruptedException e) {e.printStackTrace();}
    }

    setpoint /= (double)numSamples; // average of the speed
    
    System.out.println(" power level %VBus " + output + ", power level step +- " + oStep
      + ", Native Velocity " + setpoint + ", RPM " + setpoint*RPM_per_VelocityUnit);
  }
}
/*
page 179
Closed-Loop configs per slot (four slots available)
Name Description
kF Feed Fwd gain for Closed loop. See documentation for calculation details. If using velocity, motion
magic, or motion profile, use (1023 * duty-cycle / sensor-velocity-sensor-units-per-100ms)
kP Proportional gain for closed loop. This is multiplied by closed loop error in sensor units. Note the closed
loop output interprets a final value of 1023 as full output. So use a gain of ‘0.25’ to get full output if err
is 4096u (Mag Encoder 1 rotation)
kI Integral gain for closed loop. This is multiplied by closed loop error in sensor units every PID Loop.
Note the closed loop output interprets a final value of 1023 as full output. So use a gain of ‘0.00025’ to
get full output if err is 4096u (Mag Encoder 1 rotation) after 1000 loops
kD Derivative gain for closed loop. This is multiplied by derivative error (sensor units per PID loop). Note
the closed loop output interprets a final value of 1023 as full output. So use a gain of ‘250’ to get full
output if derr is 4096u per (Mag Encoder 1 rotation) per 1000 loops (typ 1 sec)
Loop
Period
Ms
Number of milliseconds per PID loop. Typically, this is 1ms.
Allowable
Error
If the closed loop error is within this threshold, the motor output will be neutral. Set to 0 to disable.
Value is in sensor units.
I Zone Integral Zone can be used to auto clear the integral accumulator if the sensor pos is too far from the
target. This prevent unstable oscillation if the kI is too large. Value is in sensor units.
Max
Integral
Accum
Cap on the integral accumulator in sensor units. Note accumulator is multiplied by kI AFTER this cap
takes effect.
Peak

OLD EXAMPLE

********** Robot program starting **********
[Talon] set factory default OK
[Talon] set vel period OK
[Talon] set vel window OK
[Talon] set feedback sensor FeedbackDevice.QuadEncoder OK
[Talon] set nominal output reverse OK
NT: server: client CONNECTED: 10.42.37.5 port 55455
[Talon] flywheel motor configs
.peakCurrentLimit = 1;
.peakCurrentDuration = 1;
.continuousCurrentLimit = 1;
.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
.primaryPID.selectedFeedbackCoefficient = 1.0;
.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
.auxiliaryPID.selectedFeedbackCoefficient = 1.0;
.forwardLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
.reverseLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
.forwardLimitSwitchDeviceID = 0;
.reverseLimitSwitchDeviceID = 0;
.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
.sum0Term = FeedbackDevice.QuadEncoder;
.sum1Term = FeedbackDevice.QuadEncoder;
.diff0Term = FeedbackDevice.QuadEncoder;
.diff1Term = FeedbackDevice.QuadEncoder;
.openloopRamp = 0.0;
.closedloopRamp = 0.0;
.peakOutputForward = 1.0;
.peakOutputReverse = -1.0;
.nominalOutputForward = 0.0;
.nominalOutputReverse = 0.0;
.neutralDeadband = 0.04007820136852395;
.voltageCompSaturation = 0.0;
.voltageMeasurementFilter = 32;
.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_25Ms;
.velocityMeasurementWindow = 1;
.forwardSoftLimitThreshold = 0.0;
.reverseSoftLimitThreshold = 0.0;
.forwardSoftLimitEnable = false;
.reverseSoftLimitEnable = false;
.slot0.kP = 0.0;
.slot0.kI = 0.0;
.slot0.kD = 0.0;
.slot0.kF = 0.0;
.slot0.integralZone = 0.0;
.slot0.allowableClosedloopError = 0.0;
.slot0.maxIntegralAccumulator = 0.0;
.slot0.closedLoopPeakOutput = 1.0;
.slot0.closedLoopPeriod = 1;
.slot1.kP = 0.0;
.slot1.kI = 0.0;
.slot1.kD = 0.0;
.slot1.kF = 0.0;
.slot1.integralZone = 0.0;
.slot1.allowableClosedloopError = 0.0;
.slot1.maxIntegralAccumulator = 0.0;
.slot1.closedLoopPeakOutput = 1.0;
.slot1.closedLoopPeriod = 1;
.slot2.kP = 0.0;
.slot2.kI = 0.0;
.slot2.kD = 0.0;
.slot2.kF = 0.0;
.slot2.integralZone = 0.0;
.slot2.allowableClosedloopError = 0.0;
.slot2.maxIntegralAccumulator = 0.0;
.slot2.closedLoopPeakOutput = 1.0;
.slot2.closedLoopPeriod = 1;
.slot3.kP = 0.0;
.slot3.kI = 0.0;
.slot3.kD = 0.0;
.slot3.kF = 0.0;
.slot3.integralZone = 0.0;
.slot3.allowableClosedloopError = 0.0;
.slot3.maxIntegralAccumulator = 0.0;
.slot3.closedLoopPeakOutput = 1.0;
.slot3.closedLoopPeriod = 1;
.auxPIDPolarity = false;
.filter0.remoteSensorDeviceID = 0;
.filter0.remoteSensorSource = RemoteSensorSource.Off;
.filter1.remoteSensorDeviceID = 0;
.filter1.remoteSensorSource = RemoteSensorSource.Off;
.motionCruiseVelocity = 0.0;
.motionAcceleration = 0.0;
.motionCurveStrength = 0;
.motionProfileTrajectoryPeriod = 0;
.feedbackNotContinuous = false;
.remoteSensorClosedLoopDisableNeutralOnLOS = false;
.clearPositionOnLimitF = false;
.clearPositionOnLimitR = false;
.clearPositionOnQuadIdx = false;
.limitSwitchDisableNeutralOnLOS = false;
.softLimitDisableNeutralOnLOS = false;
.pulseWidthPeriod_EdgesPerRot = 1;
.pulseWidthPeriod_FilterWindowSz = 1;
.trajectoryInterpolationEnable = true;
.customParam0 = 0;
.customParam1 = 0;
********** Robot program startup complete **********
[Talon] get kI accum 0.0
[Talon] clear kI accum OK
%VBus 0.2, velocity (native units) 56.0, velocity RPM 60.0
%VBus 0.25, velocity (native units) 68.0, velocity RPM 72.85714285714286
%VBus 0.3, velocity (native units) 84.0, velocity RPM 90.0
%VBus 0.35, velocity (native units) 100.0, velocity RPM 107.14285714285714
%VBus 0.39999999999999997, velocity (native units) 112.0, velocity RPM 120.0
%VBus 0.44999999999999996, velocity (native units) 132.0, velocity RPM 141.42857142857142
%VBus 0.49999999999999994, velocity (native units) 144.0, velocity RPM 154.28571428571428
%VBus 0.5499999999999999, velocity (native units) 160.0, velocity RPM 171.42857142857142
%VBus 0.6, velocity (native units) 176.0, velocity RPM 188.57142857142856
%VBus 0.65, velocity (native units) 196.0, velocity RPM 210.0
%VBus 0.7000000000000001, velocity (native units) 208.0, velocity RPM 222.85714285714286
%VBus 0.7500000000000001, velocity (native units) 224.0, velocity RPM 240.0
%VBus 0.8000000000000002, velocity (native units) 240.0, velocity RPM 257.1428571428571
kF = 3.3127955406632568
speed 84.0 84.0
speed 88.0 172.0
speed 88.0 260.0
speed 88.0 348.0
speed 88.0 436.0
speed 84.0 520.0
speed 84.0 604.0
speed 84.0 688.0
speed 84.0 772.0
speed 84.0 856.0
speed 84.0 940.0
speed 84.0 1024.0
speed 84.0 1108.0
speed 88.0 1196.0
speed 88.0 1284.0
speed 84.0 1368.0
speed 84.0 1452.0
speed 88.0 1540.0
speed 88.0 1628.0
speed 84.0 1712.0
power level %VBus 0.3, power level step +- 0.15, Native Velocity 85.6, RPM 91.71428571428571
[Time msecs] [PV chart min, value, max] ~~~~~~~~~ [Controller chart, min, value, max]
teleopPeriodic(): 0.020187s
SmartDashboard.updateValues(): 0.000013s
robotPeriodic(): 0.000226s
LiveWindow.updateValues(): 0.000068s
Shuffleboard.update(): 0.000007s
teleopInit(): 80.826877s
2 83.200 88.000 88.000 . . . . . + . . . . | ~ | . . . . + . . . . . 0.150 0.150 0.450Warning at edu.wpi.first.wpilibj.Tracer.lambda$printEpochs$0(Tracer.java:63): teleopPeriodic(): 0.020187s
SmartDashboard.updateValues(): 0.000013s
robotPeriodic(): 0.000226s
LiveWindow.updateValues(): 0.000068s
Shuffleboard.update(): 0.000007s
teleopInit(): 80.826877s
Loop time of 0.005s overrun
Warning at edu.wpi.first.wpilibj.IterativeRobotBase.printLoopOverrunMessage(IterativeRobotBase.java:359): Loop time of 0.005s overrun
65 44.000 44.000 127.200 | . . . . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
71 44.000 44.000 127.200 | . . . . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
78 44.000 52.000 127.200 . | . . . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
91 44.000 72.000 127.200 . . . .|. + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
122 44.000 124.000 127.200 . . . . . + . . . .|. ~ | . . . . + . . . . . 0.150 0.150 0.450
132 43.200 128.000 128.000 . . . . . + . . . . | ~ | . . . . + . . . . . 0.150 0.150 0.450
147 43.200 112.000 128.000 . . . . . + . . | . . ~ | . . . . + . . . . . 0.150 0.150 0.450
159 43.200 72.000 128.000 . . . .|. + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
169 43.200 56.000 128.000 . .|. . . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
185 43.200 68.000 128.000 . . . | . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
196 43.200 96.000 128.000 . . . . . + | . . . . ~ | . . . . + . . . . . 0.150 0.150 0.450
206 43.200 116.000 128.000 . . . . . + . . .|. . ~ | . . . . + . . . . . 0.150 0.150 0.450
218 43.200 116.000 128.000 . . . . . + . . .|. . ~ | . . . . + . . . . . 0.150 0.150 0.450
230 43.200 96.000 128.000 . . . . . + | . . . . ~ | . . . . + . . . . . 0.150 0.150 0.450
242 43.200 72.000 128.000 . . . .|. + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
257 43.200 56.000 128.000 . .|. . . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
267 43.200 64.000 128.000 . . .|. . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
278 43.200 104.000 128.000 . . . . . + . | . . . ~ | . . . . + . . . . . 0.150 0.150 0.450
293 43.200 120.000 128.000 . . . . . + . . . | . ~ | . . . . + . . . . . 0.150 0.150 0.450
310 43.200 96.000 128.000 . . . . . + | . . . . ~ | . . . . + . . . . . 0.150 0.150 0.450 peaks 293-206, Ku=0.00455, Pu=0.08700, Kp=0.00273, Ki=0.06272, Kd=0.00003 %VBus/velocity
[Talon] PID 0, Kp = 2.7911229734230094, Ki = 0.06416374651547149, Kd = 30.353462335975227, kF = 3.3127955406632568
322 43.200 68.000 128.000 . . . | . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
400 43.200 48.000 128.000 .|. . . . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
412 43.200 72.000 128.000 . . . .|. + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
421 43.200 100.000 128.000 . . . . . + .|. . . . ~ | . . . . + . . . . . 0.150 0.150 0.450
432 43.200 116.000 128.000 . . . . . + . . .|. . ~ | . . . . + . . . . . 0.150 0.150 0.450
439 43.200 116.000 128.000 . . . . . + . . .|. . ~ | . . . . + . . . . . 0.150 0.150 0.450 peaks 432-293, Ku=0.00455, Pu=0.13900, Kp=0.00273, Ki=0.03926, Kd=0.00005 %VBus/velocity
[Talon] PID 0, Kp = 2.7911229734230094, Ki = 0.040160042783064884, Kd = 48.4957616632248, kF = 3.3127955406632568
450 43.200 92.000 128.000 . . . . . +|. . . . . ~ | . . . . + . . . . . 0.150 0.150 0.450
509 43.200 44.000 128.000 | . . . . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
516 40.000 40.000 131.200 | . . . . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
529 40.000 52.000 131.200 . .|. . . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
534 40.000 52.000 131.200 . .|. . . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
551 40.000 108.000 131.200 . . . . . + . .|. . . ~ | . . . . + . . . . . 0.150 0.150 0.450
567 40.000 120.000 131.200 . . . . . + . . . | . ~ | . . . . + . . . . . 0.150 0.150 0.450
577 40.000 108.000 131.200 . . . . . + . .|. . . ~ | . . . . + . . . . . 0.150 0.150 0.450 peaks 567-432, Ku=0.00434, Pu=0.13500, Kp=0.00260, Ki=0.03858, Kd=0.00004 %VBus/velocity
[Talon] PID 0, Kp = 2.664253747358328, Ki = 0.03947042588679005, Kd = 44.95928198667179, kF = 3.3127955406632568
589 40.000 68.000 131.200 . . . | . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
626 40.000 44.000 131.200 .|. . . . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
644 40.000 64.000 131.200 . . .|. . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
649 40.000 76.000 131.200 . . . . | + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
655 40.000 92.000 131.200 . . . . . +|. . . . . ~ | . . . . + . . . . . 0.150 0.150 0.450
660 40.000 104.000 131.200 . . . . . + . | . . . ~ | . . . . + . . . . . 0.150 0.150 0.450
665 40.000 104.000 131.200 . . . . . + . | . . . ~ | . . . . + . . . . . 0.150 0.150 0.450
670 40.000 116.000 131.200 . . . . . + . . .|. . ~ | . . . . + . . . . . 0.150 0.150 0.450
676 40.000 120.000 131.200 . . . . . + . . . | . ~ | . . . . + . . . . . 0.150 0.150 0.450
681 40.000 116.000 131.200 . . . . . + . . .|. . ~ | . . . . + . . . . . 0.150 0.150 0.450 peaks 676-567, Ku=0.00434, Pu=0.10900, Kp=0.00260, Ki=0.04779, Kd=0.00004 %VBus/velocity
[Talon] PID 0, Kp = 2.664253747358328, Ki = 0.04888538985978584, Kd = 36.30045730775722, kF = 3.3127955406632568
689 40.000 96.000 131.200 . . . . . + | . . . . ~ | . . . . + . . . . . 0.150 0.150 0.450
706 40.000 60.000 131.200 . . | . . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
711 40.000 52.000 131.200 . .|. . . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
717 40.000 48.000 131.200 . | . . . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
723 40.000 48.000 131.200 . | . . . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
728 40.000 52.000 131.200 . .|. . . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
733 40.000 64.000 131.200 . . .|. . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
740 40.000 76.000 131.200 . . . . | + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
745 40.000 88.000 131.200 . . . . . | . . . . . ~ | . . . . + . . . . . 0.150 0.150 0.450
756 40.000 108.000 131.200 . . . . . + . .|. . . ~ | . . . . + . . . . . 0.150 0.150 0.450
772 40.000 116.000 131.200 . . . . . + . . .|. . ~ | . . . . + . . . . . 0.150 0.150 0.450
779 40.000 104.000 131.200 . . . . . + . | . . . ~ | . . . . + . . . . . 0.150 0.150 0.450 peaks 772-676, Ku=0.00434, Pu=0.09600, Kp=0.00260, Ki=0.05426, Kd=0.00003 %VBus/velocity
[Talon] PID 0, Kp = 2.664253747358328, Ki = 0.0555052864032985, Kd = 31.97104496829994, kF = 3.3127955406632568
784 40.000 84.000 131.200 . . . . . | . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
801 40.000 56.000 131.200 . .|. . . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
806 40.000 52.000 131.200 . .|. . . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
815 40.000 68.000 131.200 . . . | . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
820 40.000 84.000 131.200 . . . . . | . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
829 40.000 104.000 131.200 . . . . . + . | . . . ~ | . . . . + . . . . . 0.150 0.150 0.450
836 40.000 116.000 131.200 . . . . . + . . .|. . ~ | . . . . + . . . . . 0.150 0.150 0.450
844 40.000 120.000 131.200 . . . . . + . . . | . ~ | . . . . + . . . . . 0.150 0.150 0.450
849 40.000 120.000 131.200 . . . . . + . . . | . ~ | . . . . + . . . . . 0.150 0.150 0.450 peaks 844-772, Ku=0.00434, Pu=0.07200, Kp=0.00260, Ki=0.07234, Kd=0.00002 %VBus/velocity
[Talon] PID 0, Kp = 2.664253747358328, Ki = 0.07400704853773134, Kd = 23.97828372622495, kF = 3.3127955406632568
855 40.000 108.000 131.200 . . . . . + . .|. . . ~ | . . . . + . . . . . 0.150 0.150 0.450
DISABLE TELEOP

[Talon] get kI accum 0.0
[Talon] clear kI accum OK

ENABLE AUTONOMOUS

[Talon] set kF OK
[Talon] set kP OK
[Talon] set kI OK
[Talon] set kD OK
setpoint RPM 91.71428571428571, actual RPM 0.0, error RPM 0.0
setpoint RPM 91.71428571428571, actual RPM 102.85714285714286, error RPM 0.0
setpoint RPM 91.71428571428571, actual RPM 102.85714285714286, error RPM 0.0
setpoint RPM 91.71428571428571, actual RPM 102.85714285714286, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 102.85714285714286, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 98.57142857142857, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 98.57142857142857, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 98.57142857142857, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 98.57142857142857, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -11.785714285714285
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM 1.0714285714285714
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 94.28571428571428, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144
setpoint RPM 91.71428571428571, actual RPM 90.0, error RPM -3.2142857142857144

DISABLE AUTONOMOUS

[Talon] get kI accum 522.0
[Talon] clear kI accum OK


*/

/*
2.27.2 Setting Status Frame Periods
All Phoenix devices have a setStatusFramePeriod() routine/VI that allows for tweaking the frame periods of each
status group. The status group contents and default updates rates are listed below.
Status Groups
Motor Controllers
Status 1 (Default Period 10ms):
• Applied Motor Output
• Fault Information
• Limit Switch Information
Tip: Motor controllers that are followers can have slower update rates for this group without impacting performance.
Status 2 (Default Period 20ms):
• Selected Sensor Position (PID 0)
• Selected Sensor Velocity (PID 0)
• Brushed Supply Current Measurement
• Sticky Fault Information
Tip: Motor controllers that are followers can have slower update rates for this group without impacting performance.
Status 3 (Default Period >100ms):
• Quadrature Information
Status 4 (Default Period >100ms):
• Analog Input
• Supply Battery Voltage
• Controller Temperature
Status 8 (Default Period >100ms):
• Pulse Width Information
2.27. Common Device API 183

*/