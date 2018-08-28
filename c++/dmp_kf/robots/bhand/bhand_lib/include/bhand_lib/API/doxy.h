///////////////////////////////////////////////////////////////////////////////
//                   (C) Barrett Technology Inc. 2009-2010                   //
///////////////////////////////////////////////////////////////////////////////

/*!
  \mainpage Barrett Hand API Documentation

  \section qs Programming Quickstart

  To learn to write software for the Barrett Hand, Barrett recommends the following course.
  - Run the GUI, examples, and demos in either Windows or Ubuntu
  - Follow the instructions in the README to compile the GUI, API, examples, and demos in either Windows or Ubuntu
  - Read this documentation and refer to it while programming the BarretHand

  Additional documentation that the user should find helpful:
  - The BH8-Series User Manual for the Firmware
  - The BarrettHand Control GUI Manual

  This documentation applies to both the 262 and 280 hand.

  \section intro Introduction

  The BarrettHand API consists of a C++ class that provides the user with a
  convenient program interface to the BarrettHand.  It is a typical C++
  library, providing a class BHand, from which an application may instantiate
  BHand objects.  This instantiated object will make Supervisory and RealTime
  control methods available for your particular hand that is connected to your
  PC.  Serial supports communication with multiple hands although with CAN only
  one of the newer BH8-280 hands may be controlled at a time due to limitations
  of the newer drivers.

  Users are encouraged to become familiar with the API in order, starting with
  - supervisory commands that block execution until finished (open/close,
     etc),
  - writing RealTime mode velocity, torque, and/or position controllers, and then
  - learn about asynchronously initializing the library and running
    non-blocking supervisory commands.  Both of these operations are necessary to
    initialize the API more elegantly.  Refer to the code behind the
	BarrettHand Control GUI to see how to initialize the library
	asynchronously.

  The API uses a multithreaded mechanism that allows Supervisory commands
  to run with non-blocking method calls.  Calls to non-blocking methods run
  asynchronous and always return immediately.  The default
  behavior of the API is to have commands execute synchronously or one after
  another.  This mode means that calls to commands will block until they are
  complete.  Blocking calls to commands are simpler to use so they are
  recommended but they will often take time away from the running thread.  This
  can cause the application to have higher response times.

  Supervisory commands are the general name for sequentially run hand
  communication methods.  Examples of these are methods such as the high-level
  open and close commands, which belong to the set of motor movement commands.  There are
  initialization commands, property Get/Set commands, and some miscillaneous
  commands.  One of the miscillaneous commands is able to execute commands
  from the GCL (grasper common language) that takes an input string and
  may fill a character buffer with a command result depending on the command executed.
  This special command is not known until Run-Time.  The command string is
  parsed and processed by a 262 hand or by the API for a 280 hand.

  Calling supervisory methods asynchronously is possible in the API.  This is a
  feature that makes programming the hand easier for some situations.  
  The intermediate representation for supervisory commands and the
  multithreaded nature of the API makes this possible.  Briefly, the API
  handles this as described for a few different commands.  All data required to
  execute commands is stored for each API command (e.g. step size for step
  open/close or positions for a GotoPosition command).  A request is made to a
  high priority communications thread to execute each command.  The low-level
  thread waits for such command requests and then communicates the command to
  the hand.  Results from running previous commands will remain available until
  after receiving a notification from a callback method or polling to
  find out if the command is finished.  Also, some commands such as init
  hand take a good deal of time and blocking the application until finished
  is undesirable.

  As expected, a few of the functions and parameters, including those for
  multithreading, are advanced and segregated in this documentation.  While
  they are documented for completeness, we recommend avoiding use of these
  commands or changing the parameter defaults.  Describing the basics of
  multithreaded processing is beyond the scope of this documentation.  We refer
  the reader to the extensive documentation provided on the Web with a keyword
  search like "Multithreaded Programming".

  The software architecture is designed with the intention that the user will
  be using the BHand object almost exclusively.  It is highly recommended to
  stick to using the public methods that the BHand instance offers.  The API
  is structured for BHand and future updates.  Keep this in mind if modifying
  the API.  Barrett reserves the right to change the structure internally
  whenever needed but aims to provide a high level of backwards compatibility
  as well.  With that in mind, we move on to describing the hand software
  architecture.

  \section softarch Software Architecture

  The BarrettHand API is able to communicate Supervisory commands to
  the hand or have RealTime control of the fingers and/or spread.  The
  following is a top-down view of the BHand class, which is useful to understand how
  the API works.  The BHandSupervisoryRealtime class is inheritted by BHand,
  incorporates the multithreaded mechanism that allows non-blocking supervisory
  commands in the API, and provides supervisory and RealTime control modes
  for the hand.  The next level down uses an abstract hardware communications
  interface provided by BHandDriver.  There are hardware drivers for both
  serial and CAN which communicate command requests and receive responses.  The
  hardware drivers are implementations that provide the same interface for
  running methods and intend to provide as much of the same functionality as
  possible.  Presently, the only hardware communication classes implementing
  the BHandDriver interface are BHandCANDriver and BHandSerialDriver.

  Properties contain values that determine the state of the hand or motor
  controllers contained in the hand.  Earlier documentation called BarrettHand
  properties parameters instead.  In the BH8-262 hand parameters are either "global"
  or "motor" based.  Parameters have been renamed and are now referred to as
  properties.  The 280 hand has only "motor" properties since there is no
  longer a single processor.  Refer to the user manual for information on the
  feature set of each of the Barrett hands.

  Information is stored on the different BarrettHand models such as
  the known properties.  Property values are stored in the hand and are
  described in the API by attributes.  Attributes contain data that is useful
  such as property names, a description, the range of possible
  values, etc.  It is important to set the correct hardware description
  before using a BarrettHand instance so that the min/max range attributes may
  be used to validate values for certain commands.  The hand has static
  methods available to be able to query how many hardware descriptions there
  are and obtain pointers to them.  The user has to set the hardware
  description for each hand instance.  The example programs that are installed
  as part of the BarrettHand SDK show how to do this.  Refer to the README file
  to compile and run these example programs.

  Many functions in the API return an integer status code.  If the BarrettHand
  command needs to return a value and a status code then the
  API function will require a pointer to the return value passed as a
  parameter.  Commands that do not pass status codes are identified in their
  descriptions.  A return value of zero indicates successful completion.
  Positive status codes are sent by the firmware; the API generates
  negative status codes.  See \ref statusanderrorcodessec for more information
  on status codes.  Note that, most supervisory methods and the main RTUpdate
  for the BH8-280 hand do not return useful error codes, however, it is
  recommended to get in the habbit of checking for success noted by zero and
  failure as non-zero.  Users are urged to check documentation for
  particalular methods and beware that asynchronous mode changes status codes.
  Supervisory methods may return BHERR_PENDING, a negative API error code,
  because commands have not been completed yet.

  \section funcorg Functional Organization

  The BarrettHand API is organized into three layers of processing:
  low, intermediate, and high-level:

  -# The low-level processing is responsible for actual communication between
     the BarrettHand and the host computer.  This level runs in a separate
     thread with the highest priority.  It is idle in the absence of
     communication requests (without consuming CPU time), and is only
     activated when requests need to be sent and responses received.
  -# The intermediate-level processing provides a uniform interface to the
     programmer for sending commands.  This level fills device independent
     datastructures describing entire commands.  Rather than executing
     the actual data exchange, this level places communication requests
     to the low-level, and optionally waits for their completion.  This level
     is used by all high-level functions to communicate with the hardware.
  -# The high-level processing contains the functions that the user program
     normally calls (unless you wish to access the lower-level mechanisms
     directly - see \ref serialcommsec).  These functions allow users control
     of the hand without having to manage the low-level communication
     protocols.

  The design goal behind this layered organization is the following:

  Low-level processing allows the hardware communication to have priority over
  all other tasks, such as refreshing windows, disk read/write, or other user
  program processing.  Delays are minimized and no characters are lost between
  the host computer and the BarrettHand.  At the same time we do not want the
  user program to run with high priority, because it will execute many other
  operations unrelated to hardware communication, running all of them with high
  priority.  This would significantly increase the response time of the
  operating system.  Note that some programs should be given the RealTime process
  priority to ensure that the low-level thread is scheduled to run with more
  deterministic behavior.  Options to increase process priority are found in
  Windows task manager.

  The intermediate-level processing provides both synchronous and asynchronous
  communication modes for user programs, allowing you to send commands to the
  BarrettHand and have the option to wait for a response or continue with the
  program.  The multithreaded mechanism allows the user to select between
  having either blocking or non-blocking API commands.  This is not possible
  with most APIs since they usually focus on one or the other exclusively.
  The amount of complexity with providing non-blocking initialization and for
  supervisory commands is justified to provide the programmer with a simpler
  means to develop applications without using multithreading themselves.

  The high-level processing uses the functionality of both the low-level and
  intermediate-level processing.  This approach allows completely different
  low-level hardware communication implementations, it reuses the
  intermediate-level processing with device independent supervisory command
  representation and multithreaded mechanism, and provides functionality
  through the same high-level interface.

  The remaining part of this section describes the three levels in bottom-up
  order. It closely follows the structure of the BHandSupervisoryRealtime class
  declaration.

  \subsection lowlevelsec Low-Level Processing

  Low-level processing performs the communication between the BarrettHand and
  the host computer.  This level is transparent, but is included so that you can
  better understand the API.  This process runs in a separate thread and is
  given the highest priority.  The following function performs low-level hand
  communication.

  void run()

  The BHandSupervisoryRealtime class implements the POCO \c Runnable interface
  that provides an entry point for a BarrettHand thread.  See \ref pocosec for
  more information on the POCO library.  This thread is started during the
  contruction of the BHandSupervisoryRealtime object and contains a single
  loop, which detects communication requests and executes them.  The
  synchronization between the low-level and the user program thread (containing
  the two higher levels) is accomplished using the POCO Events requestPending
  and requestComplete.  Briefly, the thread function is blocked until the user
  program signals requestPending, at which point the thread function sends a
  command to the hand and waits for a response.  The expected response is
  determined by the request type (one of BHREQ_REALTIME, BHREQ_SUPERVISE,
  BHREQ_EXIT, or BHREQ_CLEAR).  When request processing is finished, the thread
  function signals requestComplete and blocks its execution until the next
  request is detected.

  Immediately before the end of processing the request, the thread function
  calls an optional user-supplied callback function, which can perform a
  desired periodic activity.  Keep in mind that the callback function is
  executed with high priority and it should not be computationally intensive.
  To specify a callback function, set the pCallback pointer to its address.

  The CTB library supports timeouts and these are utilized by the low-level
  serial communication methods in the BHandSerialDriver class for sending and
  receiving bytes.  Timeout parameters may be set (in milliseconds) that
  determine how long serial port functions should wait before returning with a
  timeout error.  These parameters are set to default values during the
  initialization of the API but can be changed later by using the
  (intermediate-level) function ComSetTimeouts.

  The PCAN library is used for sending and receiving CAN messages that the
  BHandCANDriver relies on.  Messages that are sent and received are wrapped
  in Set/Get property methods with error codes.  These are the main means used
  to communicate with the Puck 2.  There is a timeout value used for getting
  Puck 2 property values.  For this timeout it spins in a loop, polls for new
  messages, and delays a little before checking for new messages.  The reason
  for this loop is to guarantee that Get property methods will always return.
  Status codes returned are usually zero even if a timeout occurs although
  more non-zero status codes may be returned in the future to identify
  problems encounted when running commands.

  Note that the timeout values affect the behavior of the low-level processing
  thread only.  The high-level user functions provide a separate timeout
  mechanism specifying how long the user program (in synchronous mode) should
  wait for the low-level thread before it resumes execution.

  \subsection intermidiatelevelsec Intermediate-Level Processing

  The intermediate-level processing allows you to control the host computer
  more closely.  This level of processing contains the common code used to
  implement the multithreaded mechanism.  It includes the mechanisms to be able
  to make command requests, detect when commands are completed, and wait for
  commands to complete, and hold return values in a datastructure determined
  by the command that is run.

  The BHandDriver contains the intermediate-level methods that are called with
  supervisory commands formed in the BHandSupervisoryRealtime module.  Each of
  these commands shares the common pattern of making a call to the important
  runSupervisoryCommand method to begin running the command and also check
  for pending communication before storing the desired command return values.
  An outside call from higher-level processing will be made from to begin
  running particular Supervisory commands, runSupervisoryCommand method will
  start running the command in the low-level thread, and optionally wait to
  store return values.  When the low-level thread desires hardware
  communication for a command a call to ExecuteSupervisory call will call the
  blocking version of the hardware communication method that must be
  implemented by the particular BarrettHand hardware driver.

  Programs are able to control the synchronous or asynchronous call behavior
  with the low-level thread through higher-level processing methods described
  in the next subsection. See below for more information on how high-level
  functions and variables that affect intermediate-level processing.

  \subsection highlevelsec High-Level Processing

  The remaining functions are the high-level functions called in the user
  program to control the hand.  The functions in this section have a common
  structure.  The command is prepared by calling a command constructor for the
  desired command.  A command request is placed to the low-level communications
  thread with a call to the method of the particular BHandDriver implementation.
  If a value from running a command is returned, the output from the hand is interpreted
  accordingly.  Results will be returned or return values passed by a pointer
  will be set in synchronous mode.  In asynchronous mode, the results from a
  get-based command will need to be queried using GetResult().  In this mode
  error codes or status codes may be obtained using the ComGetError method.

  High-level processing allows you to control the hand via a set of standard
  functions.  These functions perform the following:

  -# Initialize the the hand for serial or CAN communications.
  -# Initialize the BarrettHand (resets parameters and aligns encoders and motors).
  -# Issue standard open and close commands for individual or multiple motors.
  -# Move individual or multiple motors to different positions.
  -# Terminate control of individual or multiple motors.
  -# Set or Get motor properties.
  -# Load and save desired or default motor properties to the EEPROM.
  -# Get hand temperature.
  -# Send Grasper Common Language (GCL) command strings to the BarrettHand.
  -# Allow control of the hand in RealTime mode.

  The execution of all functions is affected by the three variables:
  syncMode, requestTimeout, and pCallback.  See \ref apivariablessec for more
  information on these variables.  The timeout settings for serial
  communication can affect high-level functions if motion commands take more
  time than the read timeout constant allows.  For instance, slow open/close
  commands or the long running Hand-Initialize command may cause a read timeout
  because it takes too long for an acknowledgement.

  It is possible to use the high-level commands synchronously or asynchronously
  by setting the variable syncMode.  During synchronous mode, the program will
  not regain control until the BarrettHand has finished executing the command
  or the timeout limit has been reached.  During asynchronous mode, all user
  functions will return immediately with result BHERR_PENDING.  You can check
  if the processing has been completed using
  BHandSupervisoryRealtime::ComIsPending(), or wait for the processing to
  complete using BHandSupervisoryRealtime::ComWaitForCompletion().

  \section apiforcan Overview of the CAN Driver

  The BarrettHand CAN driver communicates with the with the Puck2.  There is
  one Puck 2 that controls each motor and the API provides the functionality as
  close as possible as the 262 hand.  All Pucks in the hand are connected on a
  common CAN bus.  The API contains all CAN communication code in the
  driver so that it is not necessary for the user to send or receive CAN
  messages.  The rest of the section provides more information on what the CAN
  driver is doing.

  Puck control is property based and Puck 2 CAN frames are constructed and
  sent over the CAN bus to get or set properties.  Properties may be 8, 16, or
  32 bits.  Control over communication is by the PC and a get message must be
  sent in order to receive a property value.

  There are objects created for each Puck and a manager to keep track of
  whether they are woken up or not.  Each Puck is woken up after the CAN driver
  is opened and a reset command is sent to the hand.  Supervisory commands are
  prefixed with Hand that typically call BPuck2 methods Get/Set property
  methods.  For example, BHandCANDriver implements HandInit, HandClose,
  HandGotoPosition, etc.  Some commands may just set Puck properties and others
  may need to poll to find out when commands finish such as with movement
  commands.  At this level, there is only synchronous communication.

  \section usingapi Using the BarrettHand API

  The BarrettHand API is written in C++ and this section describes how to use
  that API.  Many applications will use just the main class named BHand.
  Define an object of that class, and use it for all hand control.  Make sure
  the object does not go out of scope (i.e. define it as either a global
  variable, a member of another class, or in the main function).

  The BarrettHand API uses third party libraries that let it be used in Windows
  and Linux.  See \ref dependenciessec for more information on including these
  to be able to use the BarrettHand API.  The rest of this section assumes that
  the headers and libraries for BarrettHand depenencies are set up correctly.
  It is probably easiest to get started by modifying one of the demos or
  examples since these have both the code and cross-platform compatible
  Makefiles for small BarrettHand programs.

  Here is an example of a simple program that initializes a hand on COM1, and
  closes all fingers (in practice you should check for status codes):

  \code
  #include <BHand.h>
  void main()
  {
    BHand bh;            // Declare BHand object, bh
    char grasp[] = "G";  // Defines grasp as "G"
    char all[] = "";     // Defines all as ""

    bh.InitSoftware(1);  // Initialize the Library, use COM 1
    bh.InitHand(all);    // Initialize the BarrettHand
    bh.Close(grasp);     // Close the Grasp
  }
  \endcode

  All example code in this documentation assume an object named bh of class
  BHand and a status code of type int named err were previously defined.  More
  example code can be found in the documentation for the supervisory commands
  contained in the BHandSupervisoryRealtime class.

  \subsection installation Installation

  See the included README file for installing the BarrettHand GUI, API,
  examples, and demos on your platform.  Also included are instructions on
  building these from the sources.

  \subsection supervisorymodesec Supervisory Mode

  Supervisory mode allows control of the hand at a high-level, allowing you to
  command individual or multiple motors to close, open, and move to specific
  positions.  This set of commands is used for most grasping situations.  For
  the 262 hand, this mode takes advantage of the supervisory capabilities of
  the Motorola 68HC11.  The Motorola 68HC11 controls the hand motion by
  supervising four HCTL-1100 motion-control microprocessors, one for each
  motor.  With the 280 hand, the Supervisory commands will be running on the
  Puck 2 motor controllers.  If "real-time" control of the motor position,
  velocity, or strain is needed, then use the RealTime control mode described
  below in the \ref realtimemode section.  The following table is a summary of
  the different supervisory commands divided into a few categories.

  <center>
  <b> Table of Supervisory Commands </b>
  <table border>
    <tr>
      <td><b> Category </b></td>
      <td><b> Commands </b></td>
    </tr>
    <tr>
	  <td> Init, Calibration, Reset, etc. </td>
      <td> InitHand, Reset </td>
    </tr>
    <tr>
	  <td> Motor Movement </td>
      <td> Close, Open, GoToDefault, GoToDifferentPositions, GoToHome, GoToPosition, StepClose, StepOpen, TorqueClose, TorqueOpen </td>
    </tr>
    <tr>
	  <td> Property Get/Set </td>
      <td> Get, Set, PGet, PSet, Default, Load, Save, Temperature </td>
    </tr>
    <tr>
	  <td> Misc. </td>
      <td> Command, Delay, StopMotor, Baud </td>
    </tr>

	
  </table>
  </center>


  In the default Supervisory mode, the Hand accepts a command from one of the
  functions in the API and will not return control of the Hand until the
  command is finished processing.  When the Hand completes its command, it may
  return a status code.  See \ref statusanderrorcodessec for more detailed
  information on status codes.  Look at the BHandSupervisoryRealtime class to
  view documentation for all supervisory mode commands.  The
  next table shows methods to call to obtain command results, which are
  necessary only when running commands asynchronously.  Use this table as a
  reference when searching for return values from non-blocking versions of
  these commands.  Note that there are the usual means of returning values if
  running commands synchronously by return value or through passing the address
  of a datatype that holds the return value.

  <center>
  <b> Supervisory Command Results </b>
  <table border>
    <tr>
      <td><b> Command </b></td>
      <td><b> Type </b></td>
      <td><b> Get Method(s) </b></td>
    </tr>
    <tr>
      <td> Reset </td>
      <td> BHandResultReset </td>
      <td> bool responseReceived() </td>
    </tr>
    <tr>
      <td> Get </td>
      <td> BHandResultGet </td>
      <td> int getResult(unsigned int i), unsigned int getNumResults() </td>
    </tr>
    <tr>
      <td> PGet </td>
      <td> BHandResultPGet </td>
      <td> int getResult() </td>
    </tr>
    <tr>
      <td> Temperature </td>
      <td> BHandResultTemperature </td>
      <td> int getTemperature() </td>
    </tr>
    <tr>
      <td> Command </td>
      <td> BHandResultCommand </td>
      <td> const char * getReceive() </td>
    </tr>
  </table>
  </center>

  \subsection realtimemode RealTime Mode

  RealTime mode allows users to write custom control laws that can be computed
  by a powerful host processor.  This mode allows the host computer to send
  low-level control data and receive feedback data from the Hand in real-time.

  RealTime mode is implemented in the following sequence:
  -# Prepare RealTime mode:
    -# Use RTSetFlags() to define the subset of information to be sent by the
	   host PC and by the Hand.  Custom communication-packet blocks are then
	   optimized for fastest possible bandwidth.  Note that all control and
	   feedback motor flags can be set with the Set command.  Some BH8-280
	   properties such as "LCP" and "LFPPS" can only be set this way.
    -# Call RTStart() to switch from Supervisory mode to RealTime mode.
	   (Note that the Hand always begins in Supervisory mode upon powering up
	   and after running a reset command.)
  -# Command RealTime mode:
    -# Write control data to a host buffer with RTSet methods.  These methods
	   are not thread safe so interleave them between any RTUpdate calls.
    -# Use RTUpdate() to start the exchange of data already specified by
	   RTSetFlags().  Also, there is a special RTUpdate aimed at getting
	   arbitrary properties from the 280 Puck 2 such as 12-bit strain, inner
	   joint positions, etc.
    -# Read feedback data from the host buffer with RTGet methods.
  -# Exit RealTime mode by using RTAbort().

  RealTime properties are either flags or coefficients.  RealTime flags
  determine what data will be sent and received continuously from the hand
  during RealTime control.  RealTime coefficients affect commands that are used
  during RealTime control but are set once and are not continuously transmitted.
  You can specify different blocks of information for each motor by setting
  RealTime flags for each motor.  Some properties are only available on the
  BH8-262 and some are only available on the new BH8-280 hand.  You can also
  set different values for RealTime variables for each motor.  The possible
  control data that can be sent is listed in the RealTime Control Property
  Tables.  See the BH8-Series User Manual for details on these properties.

  <center>
  <b> RealTime Motor Control Properties </b>
  <table border>
    <tr>
      <td><b> Parameter </b></td>
      <td><b> Name </b></td>
      <td><b> Type </b></td>
      <td><b> Function </b></td>
      <td><b> 262 Size (Bytes/type) </b></td>
      <td><b> 280 Size (Bytes/type,notes) </b></td>
    </tr>
    <tr>
      <td> LCV </td>
      <td> Loop Control Velocity </td>
      <td> Flag </td>
      <td> If True, RealTime control block will contain control velocity </td>
      <td> 1 signed byte </td>
      <td> 2 signed bytes </td>
    </tr>
    <tr>
      <td> LCVC </td>
      <td> Loop Control Velocity Coefficient</td>
      <td> Coefficient (1 to 255) </td>
      <td> LCV is multiplied by LCVC to determine control velocity </td>
      <td> N/A </td>
      <td> not needed </td>
    </tr>
    <tr>
      <td> LCPG </td>
      <td> Loop Control Proportional Gain </td>
      <td> Flag </td>
      <td> If True, RealTime control block will contain proportional gain </td>
      <td> 1 unsigned byte </td>
      <td> not needed </td>
    </tr>
    <tr>
      <td> LCP </td>
      <td> Loop Control Position </td>
      <td> Flag </td>
      <td> If True, RealTime control block will contain control position </td>
      <td> N/A </td>
      <td> 4 byte, signed </td>
    </tr>
    <tr>
      <td> LCT </td>
      <td> Loop Control Torque </td>
      <td> Flag </td>
      <td> If True, RealTime control block will contain control torque </td>
      <td> 2 signed bytes </td>
      <td> 2 signed bytes </td>
    </tr>
    <tr>
      <td> LFAIN </td>
      <td> Loop Feedback Analog Input </td>
      <td> Flag </td>
      <td> If True, RealTime feedback block will contain analog input value </td>
      <td> 1 unsigned byte </td>
      <td> unused </td>
    </tr>
    <tr>
      <td> LFBP </td>
      <td> Loop Feedback Breakaway Position </td>
      <td> Flag </td>
      <td> If True, RealTime feedback block will contain breakaway position </td>
      <td> 2 unsigned bytes </td>
      <td> unused </td>
    </tr>
    <tr>
      <td> LFV </td>
      <td> Loop Feedback Velocity </td>
      <td> Flag </td>
      <td> If True, RealTime feedback block will contain feedback velocity </td>
      <td> 1 signed byte </td>
      <td> 2 signed bytes </td>
    </tr>
    <tr>
      <td> LFVC </td>
      <td> Loop Feedback Velocity Coefficient </td>
      <td> Coefficient (1 to 255) </td>
      <td> Actual velocity is divided by LFVC to get LFV </td>
      <td> N/A </td>
      <td> unused </td>
    </tr>
    <tr>
      <td> LFS </td>
      <td> Loop Feedback Strain </td>
      <td> Flag </td>
      <td> If True, RealTime feedback block will contain strain information </td>
      <td> 1 unsigned byte </td>
      <td> 1 unsigned byte (12-bit shifted by 4) </td>
    </tr>
    <tr>
      <td> LFAP </td>
      <td> Loop Feedback Absolute Position </td>
      <td> Flag </td>
      <td> If True, RealTime feedback block will contain absolute position </td>
      <td> 2 unsigned bytes </td>
      <td> 2 unsigned bytes </td>
    </tr>
    <tr>
      <td> LFDP </td>
      <td> Loop Feedback Delta Position </td>
      <td> Flag </td>
      <td> If True, RealTime feedback block will contain delta position </td>
      <td> 1 signed byte </td>
      <td> N/A </td>
    </tr>
    <tr>
      <td> LFDPC </td>
      <td> Loop Feedback Delta Position Coefficient </td>
      <td> Coefficient </td>
      <td> The actual delta position is divided by this to get LFDP </td>
      <td> N/A </td>
      <td> N/A </td>
    </tr>
    <tr>
      <td> LFDPD </td>
      <td> Loop Feedback Delta Position Discard </td>
      <td> Flag </td>
      <td> If True, any delta position overflow is discarded </td>
      <td> N/A </td>
      <td> N/A </td>
    </tr>
    <tr>
      <td> LFPPS </td>
      <td> Loop Feedback Pressure Profile Sensors </td>
      <td> Flag </td>
      <td> If True, RealTime control block will contain pressure profile sensor data </td>
      <td> N/A </td>
      <td> 24 readings, 12-bit unsigned </td>
    </tr>
  </table>
  </center>

  \par

  <center>
  <b> RealTime Global Control Properties </b>
  <table border>
    <tr>
      <td><b> Parameter </b></td>
      <td><b> Name </b></td>
      <td><b> Type </b></td>
      <td><b> Function </b></td>
      <td><b> Size (Bytes/type) </b></td>
    </tr>
    <tr>
      <td> LFT </td>
      <td> Loop Feedback Temp. </td>
      <td> Flag </td>
      <td> If True, RealTime feedback block will contain temperature </td>
      <td> 1 unsigned byte </td>
    </tr>
  </table>
  </center>

  \subsection serialcommsec Serial Communication Functions

  The serial communication functions allow you to communicate with the hand
  without ever using the high-level Supervisory or RealTime commands.  These
  serial communication functions allow taking complete advantage of what
  already exists for BarrettHand API.  Using these methods directly would allow
  building a custom software interface to the BarrettHand, however they are
  intended to support only the BarrettHand API.

  The functions relevant here may be found in the Serial Communications section
  of the header file, BHandSerialDriver.h.  These methods have the "Com" prefix
  so they can be quickly identified as being part of the serial communications
  module.  The serial communications port needs to be initialized with the
  desired comport and baud rate.  Timeout parameters may be changed from the
  defaults.  Then you may write and read data from the serial port.

  The remaining methods such as Buffer and Response provide read-only access of
  the transmit and receive buffers.  These methods are not expected to be used
  but are public in case an application is trying to observe the contents of
  these buffers.  The remaining variables in the serial driver header file for
  the hand are for storing serial communication settings, buffering Supervisory
  commands, and keeping track of RealTime flags as well as the most recent
  control and feedback data.

  \subsection apivariablessec BarrettHand API Variables

  These are the public variables in the API:
   - pCallback (#BHCallback)
     - This variable, if different from NULL, is a pointer to a function that
       will be called right before the low-level thread signals the user
       program that processing the previous supervisory command has finished.
   - requestTimeout (unsigned int)
     - This variable specifies the timeout interval (in milliseconds) used in
       synchronous mode.
   - syncMode (#BHSyncMode)
     - This variable determines whether/how the user program waits for the
	   low-level thread to complete the request before it continues.

  \subsection statusanderrorcodessec Status and Error Codes

  For all BH8-262 functions that return an integer, the value is determined
  according to the following rule:

  -#  If an error in the use of the API itself is detected, such
      as inability to read from the COM port, a negative integer is
      returned according to the errors listed in the table below
  -#  Else, if the BarrettHand Firmware reports one or more status-code
      integers according to the Table for Status Codes in the User Manual, the
      sum of those integers is returned;
  -#  Else 0 (zero) is returned, indicating successful completion of the function.

  <center>
  <b> API Error Codes </b>
  <table border>
    <tr>
      <td><b> BHERR Status Code </b></td>
      <td><b> Name </b></td>
      <td><b> Description </b></td>
    </tr>
    <tr> <td> 0 </td> <td> N/A </td> <td> Successful completion of function </td> </tr>
    <tr> <td> -1 </td> <td> BHERR_BHANDEXISTS </td> <td> Attempt to initialize a second Bhand object on a com port already running a Bhand object. </td> </tr>
    <tr> <td> -2 </td> <td> BHERR_OPENCOMMPORT </td> <td> Error opening the specified com port </td> </tr>
    <tr> <td> -3 </td> <td> BHERR_GETCOMMSTATE </td> <td> Could not read the state of the com port </td> </tr>
    <tr> <td> -4 </td> <td> BHERR_SETCOMMSTATE </td> <td> Could not set the state of the com port </td> </tr>
    <tr> <td> -5 </td> <td> BHERR_SETCOMMTIMEOUT </td> <td> Could not set the com port timeout parameters </td> </tr>
    <tr> <td> -6 </td> <td> BHERR_THREADSTART </td> <td> Could not start the low-level thread </td> </tr>
    <tr> <td> -7 </td> <td> BHERR_THREADPRIORITY </td> <td> Error setting the thread priority </td> </tr>
    <tr> <td> -8 </td> <td> BHERR_WRITECOM </td> <td> Error writing to the com port (including timeout) </td> </tr>
    <tr> <td> -9 </td> <td> BHERR_READCOM </td> <td> Error reading from the com port (including timeout) </td> </tr>
    <tr> <td> -10 </td> <td> BHERR_BADRESPONSE </td> <td> Hand responded with an incorrect sequence of characters </td> </tr>
    <tr> <td> -11 </td> <td> BHERR_CLEARBUFFER </td> <td> Could not clear com port buffers </td> </tr>
    <tr> <td> -12 </td> <td> BHERR_TIMEOUT </td> <td> Request to low-level thread timed out </td> </tr>
    <tr> <td> -13 </td> <td> BHERR_NOTCOMPLETED </td> <td> Previous request not completed (in ASYNCNOW) </td> </tr>
    <tr> <td> -14 </td> <td> BHERR_PENDING </td> <td> Request is still being processed (normal in ASYNC mode) </td> </tr>
    <tr> <td> -15 </td> <td> BHERR_NOTINITIALIZED </td> <td> A BHand object is not initialized (with InitSoftware) </td> </tr>
    <tr> <td> -16 </td> <td> BHERR_BADPARAMETER </td> <td> The parameter code passed to Get is invalid </td> </tr>
    <tr> <td> -17 </td> <td> BHERR_LONGSTRING </td> <td> Send or receive string exceeds BH_MAXCHAR </td> </tr>
    <tr> <td> -18 </td> <td> BHERR_OUTOFRANGE </td> <td> The parameter is not within a valid range </td> </tr>
    <tr> <td> -19 </td> <td> BHERR_MOTORINACTIVE </td> <td> The motor is not activated </td> </tr>
    <tr> <td> -20 </td> <td> BHERR_PORTOUTOFRANGE </td> <td> The requested serial port is out of range </td> </tr>
    </tr>
  </table>
  </center>

  Error codes for the BH8-280 hand are in development and return zero on
  success.  Non-zero API return values are subject to change.

  \section dependenciessec Third Party Dependencies

  The BarrettHand API uses cross-platform compatible libraries that support
  multithreading and serial communication.  They help to simplify the
  BarrettHand library and ensure that the low-level API provides the same
  dependability across platforms.  However, it is necessary that the programmer
  include these dependencies to compile their own programs for the BarrettHand.

  Instructions for building the third party dependencies for your platform can
  be found in a README file that will be installed with the BarrettHand
  software. Programs written for the BarrettHand will need to include headers
  and libraries for the \ref pocosec and the \ref ctblibsec.  See the
  subsections below for details on including these in your programs.

  These third party libraries are used primarily for:
  - The main thread created in BHandSupervisoryRealtime
  - Providing serial communication methods

  The subsections below provide useful information for including the
  BarrettHand dependencies that is independent of an IDE.  Users will have
  to refer to other sources or the examples to see how headers and libraries
  are included for their development environment.  Barrett has included the
  source code for these libraries and instructions on how to compile it under
  Linux (with the GNU C++ Compiler) and Windows (with MinGW).  See the demos,
  examples, and Control GUI that are included with the BarrettHand software.
  Makefiles are provided that compile the demos and examples.  The Control GUI
  uses the CodeBlocks IDE.  These programs illustrate how to compile your own
  programs for the BarrettHand.

  \subsection pocosec POCO C++ Libraries

  The following is available from "About POCO Libraries" online:

  "The POCO C++ Libraries (POCO stands for POrtable COmponents) are open source C++ class libraries that simplify and accelerate the development of network-centric, portable applications in C++. The libraries integrate perfectly with the C++ Standard Library and fill many of the functional gaps left open by it. Their modular and efficient design and implementation makes the POCO C++ Libraries extremely well suited for embedded development, an area where the C++ programming language is becoming increasingly popular, due to its suitability for both low-level (device I/O, interrupt handlers, etc.) and high-level object-oriented development. Of course, the POCO C++ Libraries are also ready for enterprise-level challenges."

  Visit http://pocoproject.org/info/ for more information.

  The POCO headers and libraries are typically located in these directories under Windows and Linux:
  - Windows Header Files: C:\\poco-1.3.6p2\\Foundation\\include
  - Windows Libary Files: C:\\poco-1.3.6p2\\lib\\MinGW\\ia32
  - Linux Header Files: /usr/include or /usr/local/include
  - Linux Library Files: /usr/lib or /usr/local/lib

  The POCO C++ Libraries contains many components: Foundation, XML, Net, Util,
  etc.  The BarrettHand API only requires the Foundation component.  By default
  POCO will undefine some Windows functions.  If applications are not able to
  compile with functions that are used in Windows.h then you will need to add
  POCO_NO_UNWINDOWS to the project defines.

  \subsection ctblibsec CTB Library

  The following is available from the IFTOOLS website online:

  "CTB was developed to simplify and standardize the communication with other devices through different interfaces (RS232, GPIB). And this is platform independent!"

  Visit https://iftools.com/opensource/ctb.en.php for more information.

  The CTB library has been integrated into the BarrettHand API so there is no
  need to specify another dependency.  The following information in this
  subsection will be left here for the time being for reference.

  The CTB headers and libraries are typically located in these directories under Windows and Linux:
  - Windows Header Files: C:\\Program Files\\Barrett Technology\\BHand\\ThirdParty\\ctb-0.14\\include
  - Windows Libary Files: C:\\Program Files\\Barrett Technology\\BHand\\ThirdParty\\ctb-0.14\\lib
  - Linux Header Files: /usr/local/Barrett/BHand/thirdparty/ctb-0.14/include
  - Linux Library Files: /usr/local/Barrett/BHand/thirdparty/ctb-0.14/lib

  In Windows there is also the "ctb-0.14.dll" file that will automatically be
  installed in the Windows system directory.  You will need to include it when
  deploying a BarrettHand application.  This DLL is required with applications
  created with MinGW.  If using Visual Studio then this DLL is not required
  because the CTB library is built as a static library.  In the future, the CTB
  library may be removed from the ThirdParty directory but for now it is left
  to ensure backwards compatibility with some older versions of the software.
  However, new users are encouraged to drop the additional CTB include and
  library dependencies.

  \subsection pcanlibsec PCAN Library

  The 280 hand depends on the PCAN library from Peak-System Technik GmbH.
  An actual USB-to-CAN adapter from Peak Systems is required to connect the
  hand to the PC.

  The PCAN headers and libraries are typically located in these directories under Windows:
  - Windows Header File: C:\\Program Files\\Barrett Technology\\BHand\\ThirdParty\\Pcan
  - Windows Library File: C:\\Program Files\\Barrett Technology\\BHand\\ThirdParty\\lib

  In Windows there is also the "Pcan_usb.dll" file that will automatically be
  installed in the Windows system directory.  You will need to include it when
  deploying a BarrettHand application.

  \subsection otherlibsec Other Libraries

  There is also the Windows Multimedia library that applications need to link
  with.  This is the winmm.lib for Visual Studio and winmm.a for MinGW.  Both
  of these compilers should not need additional library paths because this is a
  standard library.  The API uses this Windows library for increasing the
  resolution of timers.

  \section backwardcom Backwards Compatibility

  The documentation for this API is for version 4.4.3.  It has only been tested
  with Firmware version 4.4.x so firmware greater than or equal to this
  version should be installed.  The BarrettHand API source files have changed
  significantly since version 4.3x although the syntax has been kept the same.
  Software built around the BarrettHand 4.3x C-Function Library should be able
  to migrate to using the new 4.4.x API without much difficulty.

  Version 4.4.x has left out support within BHand.h for version 1.0
  compatibility of the C style interface.  See Known_Issues.txt for the known
  limitations of the API.

 */
