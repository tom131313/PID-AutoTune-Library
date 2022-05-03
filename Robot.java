package frc.robot;

/**
 * auto tune PID controller
 * 
 * All dependencies on the motor, motor controller, and gearing are gathers in config Flywheel.
 * Make a new version of that method for each combination of motor, motor controller, and gearing.
 * 
 * Run the code in TeleOperated Enable until the motors stop - tuning has completed - then Disable
 * 
 * Change to Autonomous and Enable to run the PIDF values.  Disable to stop motor.
 * 
 * 
 * improvements could include setting battery voltage compensation to say 11 or 12 volts. More reproducible but slower response the lower the voltage.
 *
 * https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/tree/master/Java%20Talon%20FX%20(Falcon%20500)
*/

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PID_ATune.CONTROL_TYPE;
import frc.robot.PID_ATune.DIRECTION;

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

public class Robot extends TimedRobot {

  // private static TalonFX flywheelMotor; //FIXME:
  private static TalonSRX flywheelMotor; //FIXME:
  /*milliseconds*/ private static final int sampleTime = 5; // milliseconds loop sample time period
  /*milliseconds*/ private static final int TIMEOUT_MS = 30;
  
  double engineeringVelocityUnit;
 
  private PID_ATune tuner;
  /*milliseconds*/ private long tunerStartTime;
  double controlSignalStop; // stop motor
  double controlSignal; // center of the relay pulse
  double oStep; // + and - step size for the controlSignal perturbation (relay)
  double minPctVBus;
  double maxPctVBus;
  BiConsumer<Double, Double> printSpeed;
  Consumer<Double> setFlywheelControlSignal;
  Runnable printAndClearIntegrator;
  Runnable setPIDFvalues;
  double setpoint; // velocity we get for the control signal
  boolean tuning;
  Supplier<Double> getFlywheelSpeed;
  Supplier<Double> getMotorOutput;
  Function<Double, Double> setFlywheelVelocity;
  double KuUnitsConversion;
  double PuUnitsConversion;

  final int pidIdx = 0; // Talon primary closed loop control
  final int slotIdx = 0; // Talon index to select which set of k's to use
  double[] kP = new double[4];
  double[] kI = new double[4];
  double[] kD = new double[4];
  double[] kF = new double[4];
  double kFtuner;

  StripChart myChart;

  double flipNearZero = Double.MIN_NORMAL; // trick for the SmartDashboard line plot

  Robot()
  {
    super((double)sampleTime/1000.); // set the robot loop time
    LiveWindow.disableAllTelemetry(); // don't waste time on stuff we don't need
  }

  @Override
  public void robotInit()
  {
    configFlywheel(); //FIXME: uncomment the right one and comment out all the others
  }

  @Override
  public void teleopInit() {

    System.out.println("Start auto tuning PIDF");
    tuning = true;

    System.out.println("Start computing kF");
    kFtuner = computeKf(minPctVBus, maxPctVBus);
    
    System.out.println("Stabilize tuning center speed");
    setpoint = stabilizeInitialSpeed(controlSignal);
    System.out.println(" control signal " + controlSignal + ", control signal step +- " + oStep + ", average velocity " + setpoint);

    tuner = new PID_ATune( setpoint, controlSignal, oStep, DIRECTION.DIRECT, CONTROL_TYPE.PID_CONTROL, sampleTime );
    
    //   S E T U P   S T R I P C H A R T   T O   D I S P L A Y   A C T I O N S
    // center of left process variable graph; minimum value of right controller graph; maximum value of right controller graph
    myChart = new StripChart( setpoint,	controlSignal-oStep,	controlSignal+oStep );

    System.out.println("Activate relay stepping");
    tunerStartTime = System.currentTimeMillis(); // tuner time now will be relative 0;
  }

  @Override
  public void teleopPeriodic()
  {
    if(!tuning)
    {
      return;
    }

      double speed = getFlywheelSpeed.get(); // get the speed of the current control signal
      int loopStartTime = (int)(System.currentTimeMillis() - tunerStartTime);
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
        System.out.format("  peaks %d-%d, Ku=%.5f, Pu=%.5f, Kp=%.5f, Ki=%.5f, Kd=%.5f\n", 
          (int)(1000.*tuner.GetPeak_1()), (int)(1000.*tuner.GetPeak_2()), tuner.GetKu(), tuner.GetPu(), tuner.GetKp(), tuner.GetKi(), tuner.GetKd());

        // Ku ultimate gain of the controller - units of process input/process output
        // Pu ultimate period - units of tuning time (tuner uses milliseconds but returns seconds)
        // convert to whatever the motor controller units are
				kP[slotIdx] = tuner.GetKp() * KuUnitsConversion;
				kI[slotIdx] = tuner.GetKi() * KuUnitsConversion/PuUnitsConversion;
				kD[slotIdx] = tuner.GetKd() * KuUnitsConversion*PuUnitsConversion;
        kF[slotIdx] = kFtuner * KuUnitsConversion;  // use 100% of kF for a flywheel
				System.out.println("[PIDF] " + slotIdx + ", Kp = " + kP[slotIdx] + ", Ki = " + kI[slotIdx] + ", Kd = " + kD[slotIdx] + ", kF = " + kF[slotIdx] + "\n");
      case 0:  // time step okay, process time step
        break;
      case 2:  // too fast, skipping this step
        return;
      default:
        System.err.println("\n\nUnknown return from Runtime()\n\n");
      }

      controlSignal = tuner.getOutput(); // get the new control signal
      setFlywheelControlSignal.accept(controlSignal); // set a new speed using the new control signal

      // time in milliseconds;	process output variable velocity /\/\/\/\;	process input (control signal) - the step function _-_-_-_-
      System.out.print("\n" + myChart.PrintStripChart( loopStartTime,	speed, controlSignal ) );
      SmartDashboard.putNumber("speed", speed);

      if(!tuning)
      {
        setFlywheelControlSignal.accept(controlSignalStop); // stop
      }
  }      

  @Override
  public void disabledInit()
  {
    printAndClearIntegrator.run();
  }

  @Override
  public void autonomousInit()
  {
    setPIDFvalues.run(); // set the PIDF constants calculated during teleop
  }
  
  /**
   * Run the motor with the PIDF values calculated in teleOp
   */
  @Override
  public void autonomousPeriodic()
  {
    // run the motor with PID control and view results
    var d = setFlywheelVelocity.apply(setpoint); // sets velocity and returns current velocity error
    var s = getFlywheelSpeed.get(); // current velocity

    // display this stuff in RPM
    d *= engineeringVelocityUnit;
    s *= engineeringVelocityUnit;
    var sp = setpoint*engineeringVelocityUnit;

    System.out.println("Engineering: setpoint " + sp + ", actual " + s + ", error " + d);
    SmartDashboard.putNumber("Engineering speed ", s);
    SmartDashboard.putNumber("control signal", getMotorOutput.get());
    SmartDashboard.putNumber("Engineering speed error", d==0?(flipNearZero=-flipNearZero):d); // trick SmartDashboard plot into thinking value is changing if 0
  }

  // WARNING this commented out method is missing a bunch of needed stuff - mostly copy the one not commented out
  // private void configFlywheel() // TalonFX
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

  /**
   * Everything unique to the motor controller and gearing TalonSRX, NeveRest20 with planetary gear box and encoder
   * Leave almost everything in native units so little conversion required
   */
  private void configFlywheel()
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

      // Talon SRX NeveRest 20:1 motor/gearbox   0.2 %VBus is 60 rpm
      engineeringVelocityUnit = // multiplicative factor to convert motor controller units to engineering units
      // for this Talon on a flywheel it's really RPM_per_TICK_PER_100MS
      // for a drive train use linear velocity such as meters/second
        1./7./4. * // 1 revolution of the motor shaft per 7 encoder pulses per 4 quadrature edges per pulse
                   // (1 pulse is encoder signal A leading and trailing edges and B leading and trailing edges)
        10.      * // 100ms/sec
        60.      * // sec/min
        1./20.;    // NeveRest internal reduction gears 1 output shaft revolution per 20 motor revolutions

      setFlywheelControlSignal = (speed) -> flywheelMotor.set(ControlMode.PercentOutput, speed);

      getFlywheelSpeed = () -> flywheelMotor.getSelectedSensorVelocity(pidIdx);
      printSpeed = (controlSignal, speed) -> System.out.println("%VBus " + controlSignal + ", velocity (native units) " + speed + ", engineering velocity " + speed*engineeringVelocityUnit);
      getMotorOutput = () -> flywheelMotor.getMotorOutputPercent();

      printAndClearIntegrator = () ->
        {
          System.out.println("[Talon] get kI accum " + flywheelMotor.getIntegralAccumulator(pidIdx));
          System.out.println("[Talon] clear kI accum " + flywheelMotor.setIntegralAccumulator(0., pidIdx, TIMEOUT_MS));
        };

      setPIDFvalues = () ->
        {
          // set the Talon PID constants calculated during teleop
          flywheelMotor.selectProfileSlot(slotIdx, pidIdx);
      
          System.out.println("[Talon] set kF " + flywheelMotor.config_kF(0, kF[slotIdx], TIMEOUT_MS));
          System.out.println("[Talon] set kP " + flywheelMotor.config_kP(0, kP[slotIdx], TIMEOUT_MS));
          System.out.println("[Talon] set kI " + flywheelMotor.config_kI(0, kI[slotIdx], TIMEOUT_MS));
          System.out.println("[Talon] set kD " + flywheelMotor.config_kD(0, kD[slotIdx], TIMEOUT_MS));
        };

      setFlywheelVelocity = (setpoint) ->
      {
        flywheelMotor.set(TalonSRXControlMode.Velocity, setpoint);
        return flywheelMotor.getClosedLoopError(pidIdx);
      };  

      /*%VBus*/ controlSignalStop = 0.; // stop motor
      /*%VBus*/ controlSignal= 0.30; // center of the tuner relay pulse %vBus for Talon FIXME
      /*%VBus*/ oStep = 0.15; // + and - step size for the tuner controlSignal perturbation (relay)
      /*%VBus*/ minPctVBus = 0.2; // min control signal to consider for kF
      /*%VBus*/ maxPctVBus = 0.81; // max control signal to consider for kF

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
    * compute one kF assuming the same (average) over the selected range.
    * More sophistication is possible -LUT kF vs speed
    * @param minControlSignal minimum (probably inclusive) to be considered for kF follows units of setFlywheelControlSignal()
    * @param maxControlSignal maximum (probably not inclusive) to be considered for kF follows units of getFlywheelSpeed()
    * @return Kf follows units setFlywheelControlSignal()/getFlywheelSpeed()
    */
  // if you want more precision on stepping, use int and multiply by the scale factor);
  public double computeKf(double minControlSignal, double maxControlSignal)
  {
    RunningRegression speedFuncOfControlSignal = new RunningRegression(); // construct regression object used for calculating kF
    RunningRegression controlSignalFuncOfSpeed = new RunningRegression(); // construct regression object used for calculating kF
    double speed = 0;

    setFlywheelControlSignal.accept(minControlSignal); // start the motor at the minimum
    Timer.delay(1.); // wait to help stabilize at minimum speed, more waiting below
    // get velocity many times at each of several control signals and least squares fit them all
    for(double controlSignal = minControlSignal; controlSignal < maxControlSignal; controlSignal+=0.05)
    {
      setFlywheelControlSignal.accept(controlSignal);
      Timer.delay(3.); // wait to stabilize speed, we are hopeful
      for(int countSamplesAtPctVBus = 1; countSamplesAtPctVBus <= 500; countSamplesAtPctVBus++)
      {
        speed = getFlywheelSpeed.get();
        speedFuncOfControlSignal.Push(controlSignal, speed);
        controlSignalFuncOfSpeed.Push(speed, controlSignal);
        Timer.delay((double)sampleTime/1000.);
      }
      printSpeed.accept(controlSignal, speed);
    }

    System.out.println("control signal = f(speed)\n" + controlSignalFuncOfSpeed.toString());

    return 1./speedFuncOfControlSignal.Slope(); // kF inverse velocity units/control signal
  }

  /**
   * lingers at a setpoint to stabilize the motor at that speed
   * used to start tuning at a stable center velocity
   * 
   * Units are neutral
   * @param controlSignal follows units of setFlywheelControlSignal()
   * @return follows units of getFlywheelSpeed()
   */
  public double stabilizeInitialSpeed(double controlSignal) 
  {
    setFlywheelControlSignal.accept(controlSignal);

    try {Thread.sleep(5000);} // let motor speed stabilize
        catch (InterruptedException e) {e.printStackTrace();}

    //get the (average) speed at this control signal
    int numSamples = 20;
    double averageSpeed = 0;

    for(int idx =1; idx <=numSamples; idx++)
    {
        var speed = getFlywheelSpeed.get();
        averageSpeed += speed;
        System.out.println("speed " + speed + " " + averageSpeed);
        try {Thread.sleep(100);} // let motor speed stabilize
          catch (InterruptedException e) {e.printStackTrace();}
    }

    averageSpeed /= (double)numSamples; // average of the speed
    
    return averageSpeed;
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

EXAMPLE

NOTE THAT THE FIRST RUN IN AUTONOMOUS TO USER THE PIDF VALUES HAS A DISPLAY OF THE MOTOR VELOCITY RUNNING BUT IT ISN"T ACTUALLY RUNNING

********** Robot program starting **********
NT: server: client CONNECTED: 10.42.37.5 port 60672
[Talon] set factory default OK
[Talon] set vel period OK
[Talon] set vel window OK
[Talon] set feedback sensor FeedbackDevice.QuadEncoder OK
[Talon] set nominal output reverse OK
[Talon] set status 2 OK
[Talon] set status 13 OK
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
Loop time of 0.005s overrun
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
Warning at edu.wpi.first.wpilibj.IterativeRobotBase.printLoopOverrunMessage(IterativeRobotBase.java:359): Loop time of 0.005s overrun
Default disabledPeriodic() method... Override me!
Default robotPeriodic() method... Override me!
SmartDashboard.updateValues(): 0.039240s
disabledInit(): 0.038865s
robotPeriodic(): 0.000316s
LiveWindow.updateValues(): 0.000091s
Shuffleboard.update(): 0.008907s
disabledPeriodic(): 0.012961s
Warning at edu.wpi.first.wpilibj.Tracer.lambda$printEpochs$0(Tracer.java:63): SmartDashboard.updateValues(): 0.039240s
disabledInit(): 0.038865s
robotPeriodic(): 0.000316s
LiveWindow.updateValues(): 0.000091s
Shuffleboard.update(): 0.008907s
disabledPeriodic(): 0.012961s
[phoenix] Library initialization is complete.
[phoenix-diagnostics] Server 1.9.0 (Jan 4 2022,20:28:13) running on port: 1250
Loop time of 0.005s overrun
Warning at edu.wpi.first.wpilibj.IterativeRobotBase.printLoopOverrunMessage(IterativeRobotBase.java:359): Loop time of 0.005s overrun
SmartDashboard.updateValues(): 0.001042s
robotPeriodic(): 0.002397s
LiveWindow.updateValues(): 0.000532s
Shuffleboard.update(): 0.002631s
disabledPeriodic(): 0.001251s
Warning at edu.wpi.first.wpilibj.Tracer.lambda$printEpochs$0(Tracer.java:63): SmartDashboard.updateValues(): 0.001042s
robotPeriodic(): 0.002397s
LiveWindow.updateValues(): 0.000532s
Shuffleboard.update(): 0.002631s
disabledPeriodic(): 0.001251s
Loop time of 0.005s overrun
Warning at edu.wpi.first.wpilibj.IterativeRobotBase.printLoopOverrunMessage(IterativeRobotBase.java:359): Loop time of 0.005s overrun
SmartDashboard.updateValues(): 0.000009s
robotPeriodic(): 0.000008s
LiveWindow.updateValues(): 0.000005s
Shuffleboard.update(): 0.007566s
disabledPeriodic(): 0.000091s
Warning at edu.wpi.first.wpilibj.Tracer.lambda$printEpochs$0(Tracer.java:63): SmartDashboard.updateValues(): 0.000009s
robotPeriodic(): 0.000008s
LiveWindow.updateValues(): 0.000005s
Shuffleboard.update(): 0.007566s
disabledPeriodic(): 0.000091s
Start auto tuning PIDF
Start computing kF
Loop time of 0.005s overrun
Warning at edu.wpi.first.wpilibj.IterativeRobotBase.printLoopOverrunMessage(IterativeRobotBase.java:359): Loop time of 0.005s overrun
%VBus 0.2, velocity (native units) 52.0, engineering velocity 55.714285714285715
%VBus 0.25, velocity (native units) 68.0, engineering velocity 72.85714285714286
%VBus 0.3, velocity (native units) 84.0, engineering velocity 90.0
%VBus 0.35, velocity (native units) 100.0, engineering velocity 107.14285714285714
%VBus 0.39999999999999997, velocity (native units) 116.0, engineering velocity 124.28571428571428
%VBus 0.44999999999999996, velocity (native units) 132.0, engineering velocity 141.42857142857142
%VBus 0.49999999999999994, velocity (native units) 144.0, engineering velocity 154.28571428571428
%VBus 0.5499999999999999, velocity (native units) 160.0, engineering velocity 171.42857142857142
%VBus 0.6, velocity (native units) 176.0, engineering velocity 188.57142857142856
%VBus 0.65, velocity (native units) 192.0, engineering velocity 205.71428571428572
%VBus 0.7000000000000001, velocity (native units) 208.0, engineering velocity 222.85714285714286
%VBus 0.7500000000000001, velocity (native units) 220.0, engineering velocity 235.7142857142857
%VBus 0.8000000000000002, velocity (native units) 240.0, engineering velocity 257.1428571428571
control signal = f(speed)
NumDataValues: 6500
y = A + Bx ==> y = 0.0189338 +0.00330927x
Slope: 0.003309
Intercept: 0.018934
r^2 Correlation: 0.999415
Stabilize tuning center speed
speed 84.0 84.0
speed 84.0 168.0
speed 84.0 252.0
speed 84.0 336.0
speed 84.0 420.0
speed 84.0 504.0
speed 84.0 588.0
speed 84.0 672.0
speed 80.0 752.0
speed 84.0 836.0
speed 84.0 920.0
speed 84.0 1004.0
speed 84.0 1088.0
speed 84.0 1172.0
speed 84.0 1256.0
speed 84.0 1340.0
speed 84.0 1424.0
speed 84.0 1508.0
speed 84.0 1592.0
speed 84.0 1676.0
control signal 0.3, control signal step +- 0.15, average velocity 83.8
[Time msecs] [PV chart min, value, max] ~~~~~~~~~ [Controller chart, min, value, max]
Activate relay stepping
teleopPeriodic(): 0.023546s
SmartDashboard.updateValues(): 0.000013s
robotPeriodic(): 0.000301s
LiveWindow.updateValues(): 0.000006s
Shuffleboard.update(): 0.000006s
teleopInit(): 80.887824s
2 83.600 84.000 84.000 . . . . . + . . . . | ~ . . . . . + . . . . | 0.150 0.450 0.450Warning at edu.wpi.first.wpilibj.Tracer.lambda$printEpochs$0(Tracer.java:63): teleopPeriodic(): 0.023546s
SmartDashboard.updateValues(): 0.000013s
robotPeriodic(): 0.000301s
LiveWindow.updateValues(): 0.000006s
Shuffleboard.update(): 0.000006s
teleopInit(): 80.887824s
Loop time of 0.005s overrun
68 43.600 124.000 124.000 . . . . . + . . . . | ~ | . . . . + . . . . . 0.150 0.150 0.450Warning at edu.wpi.first.wpilibj.IterativeRobotBase.printLoopOverrunMessage(IterativeRobotBase.java:359): Loop time of 0.005s overrun
73 43.600 124.000 124.000 . . . . . + . . . . | ~ | . . . . + . . . . . 0.150 0.150 0.450
78 39.600 128.000 128.000 . . . . . + . . . . | ~ | . . . . + . . . . . 0.150 0.150 0.450
83 39.600 128.000 128.000 . . . . . + . . . . | ~ | . . . . + . . . . . 0.150 0.150 0.450
88 39.600 120.000 128.000 . . . . . + . . . | . ~ | . . . . + . . . . . 0.150 0.150 0.450
102 39.600 92.000 128.000 . . . . . + | . . . . ~ | . . . . + . . . . . 0.150 0.150 0.450
115 39.600 64.000 128.000 . . . | . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
129 39.600 52.000 128.000 . .|. . . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
149 39.600 84.000 128.000 . . . . . | . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
168 39.600 112.000 128.000 . . . . . + . . | . . ~ | . . . . + . . . . . 0.150 0.150 0.450
186 39.600 120.000 128.000 . . . . . + . . . | . ~ | . . . . + . . . . . 0.150 0.150 0.450
198 39.600 88.000 128.000 . . . . . +|. . . . . ~ | . . . . + . . . . . 0.150 0.150 0.450
220 39.600 60.000 128.000 . . .|. . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
240 39.600 60.000 128.000 . . .|. . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
260 39.600 108.000 128.000 . . . . . + . .|. . . ~ | . . . . + . . . . . 0.150 0.150 0.450
279 39.600 108.000 128.000 . . . . . + . .|. . . ~ | . . . . + . . . . . 0.150 0.150 0.450
298 39.600 68.000 128.000 . . . | . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450 peaks 260-186, Ku=0.00503, Pu=0.07400, Kp=0.00302, Ki=0.08150, Kd=0.00003
[PIDF] 0, Kp = 3.0849253916780643, Ki = 0.083376361937245, Kd = 28.53555987302209, kF = 3.3893510306440726
318 39.600 64.000 128.000 . . . | . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
384 35.600 132.000 132.000 . . . . . + . . . . | ~ | . . . . + . . . . . 0.150 0.150 0.450
389 35.600 128.000 132.000 . . . . . + . . . .|. ~ | . . . . + . . . . . 0.150 0.150 0.450
394 35.600 132.000 132.000 . . . . . + . . . . | ~ | . . . . + . . . . . 0.150 0.150 0.450
399 35.600 124.000 132.000 . . . . . + . . . .|. ~ | . . . . + . . . . . 0.150 0.150 0.450
404 35.600 124.000 132.000 . . . . . + . . . .|. ~ | . . . . + . . . . . 0.150 0.150 0.450
409 35.600 112.000 132.000 . . . . . + . . | . . ~ | . . . . + . . . . . 0.150 0.150 0.450
424 35.600 80.000 132.000 . . . . .|+ . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
445 35.600 56.000 132.000 . . | . . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
465 35.600 96.000 132.000 . . . . . + | . . . . ~ | . . . . + . . . . . 0.150 0.150 0.450
489 35.600 112.000 132.000 . . . . . + . . | . . ~ | . . . . + . . . . . 0.150 0.150 0.450
512 35.600 68.000 132.000 . . . .|. + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
525 35.600 52.000 132.000 . .|. . . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
558 35.600 104.000 132.000 . . . . . + . | . . . ~ | . . . . + . . . . . 0.150 0.150 0.450
584 35.600 100.000 132.000 . . . . . + .|. . . . ~ | . . . . + . . . . . 0.150 0.150 0.450
603 35.600 60.000 132.000 . . .|. . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
614 35.600 52.000 132.000 . .|. . . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
636 35.600 68.000 132.000 . . . .|. + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
655 35.600 112.000 132.000 . . . . . + . . | . . ~ | . . . . + . . . . . 0.150 0.150 0.450
675 35.600 120.000 132.000 . . . . . + . . . | . ~ | . . . . + . . . . . 0.150 0.150 0.450
691 35.600 88.000 132.000 . . . . . +|. . . . . ~ | . . . . + . . . . . 0.150 0.150 0.450 peaks 675-489, Ku=0.00477, Pu=0.18600, Kp=0.00286, Ki=0.03080, Kd=0.00007
[PIDF] 0, Kp = 2.9306791220941606, Ki = 0.03151267873219527, Kd = 68.13828958868923, kF = 3.3893510306440726
697 35.600 76.000 132.000 . . . . .|+ . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
722 35.600 48.000 132.000 . | . . . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
727 35.600 48.000 132.000 . | . . . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
732 35.600 48.000 132.000 . | . . . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
746 35.600 72.000 132.000 . . . . | + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
751 35.600 84.000 132.000 . . . . . | . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
766 35.600 112.000 132.000 . . . . . + . . | . . ~ | . . . . + . . . . . 0.150 0.150 0.450
771 35.600 116.000 132.000 . . . . . + . . .|. . ~ | . . . . + . . . . . 0.150 0.150 0.450
782 35.600 124.000 132.000 . . . . . + . . . .|. ~ | . . . . + . . . . . 0.150 0.150 0.450
787 35.600 120.000 132.000 . . . . . + . . . | . ~ | . . . . + . . . . . 0.150 0.150 0.450
792 35.600 116.000 132.000 . . . . . + . . .|. . ~ | . . . . + . . . . . 0.150 0.150 0.450 peaks 782-675, Ku=0.00455, Pu=0.10700, Kp=0.00273, Ki=0.05100, Kd=0.00004
[PIDF] 0, Kp = 2.7911229734230094, Ki = 0.05217052286771981, Kd = 37.33126976953276, kF = 3.3893510306440726
805 35.600 76.000 132.000 . . . . .|+ . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
824 35.600 56.000 132.000 . . | . . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
829 35.600 52.000 132.000 . .|. . . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
835 35.600 56.000 132.000 . . | . . + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
846 35.600 72.000 132.000 . . . . | + . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
852 35.600 84.000 132.000 . . . . . | . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450
880 35.600 120.000 132.000 . . . . . + . . . | . ~ | . . . . + . . . . . 0.150 0.150 0.450
888 35.600 128.000 132.000 . . . . . + . . . .|. ~ | . . . . + . . . . . 0.150 0.150 0.450
906 35.600 108.000 132.000 . . . . . + . .|. . . ~ | . . . . + . . . . . 0.150 0.150 0.450 peaks 888-782, Ku=0.00455, Pu=0.10600, Kp=0.00273, Ki=0.05148, Kd=0.00004
[PIDF] 0, Kp = 2.7911229734230094, Ki = 0.0526626976117549, Kd = 36.98237939785488, kF = 3.3893510306440726
Loop time of 0.005s overrun
teleopPeriodic(): 0.004915s
SmartDashboard.updateValues(): 0.000012s
robotPeriodic(): 0.000009s
LiveWindow.updateValues(): 0.000005s
Shuffleboard.update(): 0.000006s
915 35.600 80.000 132.000 . . . . .|+ . . . . . ~ . . . . . + . . . . | 0.150 0.450 0.450Warning at edu.wpi.first.wpilibj.IterativeRobotBase.printLoopOverrunMessage(IterativeRobotBase.java:359): Loop time of 0.005s overrun
Warning at edu.wpi.first.wpilibj.Tracer.lambda$printEpochs$0(Tracer.java:63): teleopPeriodic(): 0.004915s
SmartDashboard.updateValues(): 0.000012s
robotPeriodic(): 0.000009s
LiveWindow.updateValues(): 0.000005s
Shuffleboard.update(): 0.000006s
Loop time of 0.005s overrun
Warning at edu.wpi.first.wpilibj.IterativeRobotBase.printLoopOverrunMessage(IterativeRobotBase.java:359): Loop time of 0.005s overrun
teleopPeriodic(): 0.000066s
SmartDashboard.updateValues(): 0.007403s
robotPeriodic(): 0.000006s
LiveWindow.updateValues(): 0.000316s
Shuffleboard.update(): 0.000008s
   from: edu.wpi.first.wpilibj.Tracer.lambda$printEpochs$0(Tracer.java:63)

Warning at edu.wpi.first.wpilibj.Tracer.lambda$printEpochs$0(Tracer.java:63): teleopPeriodic(): 0.000066s
SmartDashboard.updateValues(): 0.007403s
robotPeriodic(): 0.000006s
LiveWindow.updateValues(): 0.000316s
Shuffleboard.update(): 0.000008s
[Talon] get kI accum 0.0
[Talon] clear kI accum OK
Loop time of 0.005s overrun
Warning at edu.wpi.first.wpilibj.IterativeRobotBase.printLoopOverrunMessage(IterativeRobotBase.java:359): Loop time of 0.005s overrun
SmartDashboard.updateValues(): 0.000009s
robotPeriodic(): 0.000007s
LiveWindow.updateValues(): 0.006767s
Shuffleboard.update(): 0.000027s
disabledPeriodic(): 0.000070s
Warning at edu.wpi.first.wpilibj.Tracer.lambda$printEpochs$0(Tracer.java:63): SmartDashboard.updateValues(): 0.000009s
robotPeriodic(): 0.000007s
LiveWindow.updateValues(): 0.006767s
Shuffleboard.update(): 0.000027s
disabledPeriodic(): 0.000070s
Loop time of 0.005s overrun
[Talon] set kF OK
Warning at edu.wpi.first.wpilibj.IterativeRobotBase.printLoopOverrunMessage(IterativeRobotBase.java:359): Loop time of 0.005s overrun
[Talon] set kP OK
[Talon] set kI OK
[Talon] set kD OK
Engineering: setpoint 89.78571428571428, actual 0.0, error 1.0714285714285714
autonomousInit(): 0.040275s
SmartDashboard.updateValues(): 0.000013s
robotPeriodic(): 0.000708s
LiveWindow.updateValues(): 0.000005s
Shuffleboard.update(): 0.000007s
autonomousPeriodic(): 0.058868s
Warning at edu.wpi.first.wpilibj.Tracer.lambda$printEpochs$0(Tracer.java:63): autonomousInit(): 0.040275s
SmartDashboard.updateValues(): 0.000013s
robotPeriodic(): 0.000708s
LiveWindow.updateValues(): 0.000005s
Shuffleboard.update(): 0.000007s
autonomousPeriodic(): 0.058868s

DISABLE

ENABLE AUTONOMOUS

Engineering: setpoint 89.78571428571428, actual 98.57142857142857, error -13.928571428571429
Engineering: setpoint 89.78571428571428, actual 98.57142857142857, error -9.642857142857142
Engineering: setpoint 89.78571428571428, actual 98.57142857142857, error -9.642857142857142
Engineering: setpoint 89.78571428571428, actual 98.57142857142857, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 98.57142857142857, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 98.57142857142857, error -9.642857142857142
Engineering: setpoint 89.78571428571428, actual 98.57142857142857, error -9.642857142857142
Engineering: setpoint 89.78571428571428, actual 94.28571428571428, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 94.28571428571428, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 94.28571428571428, error -9.642857142857142
Engineering: setpoint 89.78571428571428, actual 94.28571428571428, error -9.642857142857142
Engineering: setpoint 89.78571428571428, actual 94.28571428571428, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 94.28571428571428, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 94.28571428571428, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 94.28571428571428, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 94.28571428571428, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 94.28571428571428, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 94.28571428571428, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 94.28571428571428, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 94.28571428571428, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 94.28571428571428, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 94.28571428571428, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 94.28571428571428, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 94.28571428571428, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 94.28571428571428, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 94.28571428571428, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 94.28571428571428, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 94.28571428571428, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 90.0, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 90.0, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 90.0, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 94.28571428571428, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 94.28571428571428, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 90.0, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 90.0, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 94.28571428571428, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 94.28571428571428, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 90.0, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 90.0, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 90.0, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 90.0, error -5.357142857142857
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 90.0, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 90.0, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 90.0, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 90.0, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 90.0, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error -1.0714285714285714
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 85.71428571428571, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 90.0, error 3.2142857142857144
Engineering: setpoint 89.78571428571428, actual 90.0, error 3.2142857142857144
[Talon] get kI accum 330.0
[Talon] clear kI accum OK
Loop time of 0.005s overrun
Warning at edu.wpi.first.wpilibj.IterativeRobotBase.printLoopOverrunMessage(IterativeRobotBase.java:359): Loop time of 0.005s overrun
SmartDashboard.updateValues(): 0.000009s
robotPeriodic(): 0.000008s
LiveWindow.updateValues(): 0.006429s
Shuffleboard.update(): 0.000025s
disabledPeriodic(): 0.000079s
Warning at edu.wpi.first.wpilibj.Tracer.lambda$printEpochs$0(Tracer.java:63): SmartDashboard.updateValues(): 0.000009s
robotPeriodic(): 0.000008s
LiveWindow.updateValues(): 0.006429s
Shuffleboard.update(): 0.000025s
disabledPeriodic(): 0.000079s
DISABLE AUTONOMOUS

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