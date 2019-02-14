/**
 * Example demonstrating the Position closed-loop servo.
 * Tested with Logitech F350 USB Gamepad inserted into Driver Station]
 *
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 *
 * After deploying/debugging this to your RIO, first use the left Y-stick
 * to throttle the Talon manually.  This will confirm your hardware setup.
 * Be sure to confirm that when the Talon is driving forward (green) the
 * position sensor is moving in a positive direction.  If this is not the case
 * flip the boolean input to the reverseSensor() call below.
 *
 * Once you've ensured your feedback device is in-phase with the motor,
 * use the button shortcuts to servo to target position.
 *
 * Tweak the PID gains accordingly.
 */
#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "Constants.h"

using namespace frc;

class Robot: public TimedRobot {
public:
	TalonSRX * _talon_right = new TalonSRX(2);
	Joystick * _joy = new Joystick(0);
	std::string _sb;
	int _loops = 0;
	int _deadBand = .1;
	bool _lastButton1 = false;
	bool _lastButton2 = false;
	bool _lastButton3 = false;

	/** save the target position to servo to */
	double targetPositionRotations;

	void RobotInit() {
		/* use the low level API to set the quad encoder signal to start at 0 */
		_talon_right->SetSelectedSensorPosition(0, kPIDLoopIdx,
				kTimeoutMs);

		/* choose the sensor and sensor direction */
		_talon_right->ConfigSelectedFeedbackSensor(
				FeedbackDevice::QuadEncoder, kPIDLoopIdx,
				kTimeoutMs);
		_talon_right->SetSensorPhase(false);

		/* set speed controller direction */
		_talon_right->SetInverted(false);

		/* set the peak and nominal outputs, 12V means full */
		_talon_right->ConfigNominalOutputForward(0, kTimeoutMs);
		_talon_right->ConfigNominalOutputReverse(0, kTimeoutMs);
		_talon_right->ConfigPeakOutputForward(1, kTimeoutMs);
		_talon_right->ConfigPeakOutputReverse(-1, kTimeoutMs);

		/* set closed loop gains in slot0 */
		_talon_right->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		_talon_right->Config_kP(kPIDLoopIdx, 20.0, kTimeoutMs); 
		_talon_right->Config_kI(kPIDLoopIdx, 0.001, kTimeoutMs);
		_talon_right->Config_kD(kPIDLoopIdx, 200.0, kTimeoutMs);
		_talon_right->Config_IntegralZone(kPIDLoopIdx, 50, kTimeoutMs);

		/* set deadband */
		_talon_right->ConfigNeutralDeadband(_deadBand, kTimeoutMs);

		/* set closed loop ramping rate */
		_talon_right->ConfigClosedloopRamp(.01, kTimeoutMs);
	}

	/**
	 * This function is called periodically during operator control
	 */
	void TeleopPeriodic() {
		/* get gamepad axis */
		double leftYstick = _joy->GetRawAxis(1) * -1;
		
		double motorOutputRight = _talon_right->GetMotorOutputPercent();

		/* get gamepad buttons */
		bool button1 = _joy->GetRawButton(1);
		bool button2 = _joy->GetRawButton(2);
		bool button3 = _joy->GetRawButton(3);
		bool button4 = _joy->GetRawButton(4);

		/* prepare line to print */
		_sb.append("\tout right: ");
		_sb.append(std::to_string(motorOutputRight));
		_sb.append("\tpos right: ");
		_sb.append(std::to_string(_talon_right->GetSelectedSensorPosition(kPIDLoopIdx)));

		/* on button1 press enter closed-loop mode on target position */
		if (!_lastButton1 && button1) {
			/* Position mode - button just pressed */
		 	targetPositionRotations = 5.0 * 80 * 12; /* 5 Rotations - Cimcoder has a resolution of 80 - overall gear ratio to mechanism is 12:1 */
		 	_talon_right->Set(ControlMode::Position, targetPositionRotations); /* 5 rotations */
		}
		
		/* on button2 press enter closed-loop mode on target position */
		else if (!_lastButton2 && button2) {
			/* Position mode - button just pressed */
		 	targetPositionRotations = 10.0 * 80 * 12; /* 10 Rotations - Cimcoder has a resolution of 80 - overall gear ratio to mechanism is 12:1 */
		 	_talon_right->Set(ControlMode::Position, targetPositionRotations); /* 10 rotations */
		}

		/* on button3 press enter closed-loop mode on target position */
		else if (!_lastButton3 && button3) {
			/* Position mode - button just pressed */
		 	targetPositionRotations = 15.0 * 80 * 12; /* 15 Rotations in - Cimcoder has a resolution of 80 - overall gear ratio to mechanism is 12:1 */
		 	_talon_right->Set(ControlMode::Position, targetPositionRotations); /* 15 rotations */
		}

		/* Joystick control using position mode */
		else {
			/* check if joystick is moving */
			if (abs(leftYstick) > 0.01) {
				int currentPos = _talon_right->GetSelectedSensorPosition(kPIDLoopIdx);
				if ( targetPositionRotations - 50 > currentPos  || currentPos > targetPositionRotations + 50) {
					targetPositionRotations = leftYstick * 150 + currentPos;
					_talon_right->Set(ControlMode::Position, targetPositionRotations);
				}
				else {
					targetPositionRotations = leftYstick * 150 + targetPositionRotations;
					_talon_right->Set(ControlMode::Position, targetPositionRotations);
				}
			}
		}

		/* if Talon is in position closed-loop, print some more info */
		if (_talon_right->GetControlMode() == ControlMode::Position) {
			/* append more signals to print when in speed mode. */
			_sb.append("\terrNative: ");
			_sb.append(std::to_string(_talon_right->GetClosedLoopError(kPIDLoopIdx)));
			_sb.append("\ttrg: ");
			_sb.append(std::to_string(targetPositionRotations));
		}

		/* print every ten loops, printing too much too fast is generally bad for performance */
		if (++_loops >= 10) {
			_loops = 0;
			printf("%s\n", _sb.c_str());
		}
		_sb.clear();

		/* save button state for on press detect */
		_lastButton1 = button1;
		_lastButton2 = button2;
		_lastButton3 = button3;
	}
};

START_ROBOT_CLASS(Robot)