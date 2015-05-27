mBed code readme

Developed in online mBed IDE

Repo at: https://developer.mbed.org/teams/jetfishteam/code/fish_firmware_v_5/

Task class [task.h]
	Wraps mBed RTOS framework
	Contains RTOS Thread object
	Intended to be used as abstract base class for software modules
	Provides generic constructor, destructor, start(), and stop() methods
	Derived classes must define their own _run() method which is executed
	in seperate thread started by call to start()


Modules: (this is the current design concept, may not yet be fully reflected in implementation)
	{These are implemented as classes derived from Task.}
	Supervisor [supervisor.h]
		This task oversees the status of the fish, and is in charge of commnding the swim mode. It also interfaces with the buttons via the Buttons object for user io.
	RaspiComm [raspi_comm.h]
		This task handles serial communication with the Raspberry Pi.
		Uses the mBed Serial class.
		Implements simply sync-byte protocal, with byte escaping and fixed message definitions.
	SwimController [swim_controller.h]
		This task operates in one of several swim modes, and sets actuator commands accordingly.
		Maintains SwimCmdState object which tracks several levels of command setpoints.
		Implements functions that map higher-level setpoints to lower-level ones.
		Commands tail driver, SDU, and dive plane servos.
	TailDriver [tail_driver.h]
		This task takes tail actuator commands (freq, amp, etc) and calculates the corresonding
		command waveform to send the gear pump motor. 
	SDU [sdu.h]
		Driver for the static diving unit.
	IMDU [imdu.h]
		Driver for the inertial measurement and depth unit.
		Polls sensors periodically and makes data available to other modules.

