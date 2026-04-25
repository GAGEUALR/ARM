# V1 Scope: Controller Mode

## V1 Goal
V1 is a manually controlled robotic arm. The arm should be controllable from a game controller through the Raspberry Pi, with the ESP32 handling servo control, limits, timing, and basic safety behavior.

## V1 Includes
- Manual controller mode
- Raspberry Pi to ESP32 UART communication
- ESP32 servo control
- Servo position/speed limits
- Smooth ramping or controlled movement
- Basic current sensing or current logging
- Basic fault reporting
- GUI/home screen showing system status
- Repeatable demo motion
- Clean GitHub documentation

## V1 Completion Criteria
V1 is complete when:
- Controller input maps predictably to servo movement
- Pi can start the control program reliably
- ESP32 connects over UART
- System moves arm to predefined startup position
- Servo limits prevent unsafe or impossible positions
- System can safely disable movement or return to home on command
- Basic status is visible through GUI
- Demo can be recorded without code changes
- Hardware has 3D printed casing and secure wiring

--- 

# V2 Scope: Chess Mode

## V2 Goal
V2 adds chess-playing functionality on top of the completed V1 controller-mode platform. The arm should be able to recognize the chessboard state, calculate a legal move, and move pieces on the board with camera/ToF-assisted positioning and manual recovery options.

V2 depends on V1 being stable.

## V2 Includes
- Chess mode GUI page
- Home/board-view position for camera alignment
- Camera integration for board and piece recognition
- ToF sensor integration for piece height/distance feedback
- Chessboard calibration process
- Board-state detection
- Stockfish integration for move calculation
- Conversion of detected board state into a chess position format
- Autonomous movement from source square to destination square
- Capture handling
- Basic move verification after the arm moves a piece
- Manual correction/recovery controls
- Error reporting through the GUI
- Demo showing at least one complete legal chess move

## V2 Does Not Include
- Full competitive chess engine development
- High-speed movement
- Guaranteed recovery from every possible board error
- Online multiplayer or networking features
- Advanced AI beyond board recognition and move execution

## V2 Completion Criteria
V2 is complete when:

- [ ] System can move to a predefined board-view position
- [ ] Camera can view the full chessboard from the home position
- [ ] Board calibration can map camera view to chessboard squares
- [ ] System can detect or manually confirm current board state
- [ ] Stockfish (chess engine) can generate a legal move from the current position
- [ ] System can translate a chess move into arm movement commands
- [ ] Arm can pick up a piece from a source square
- [ ] Arm can place the piece on the destination square
- [ ] Basic capture move can be handled
- [ ] ToF/camera feedback is used for positioning or verification
- [ ] GUI shows chess mode status, errors, and recovery options
- [ ] User can manually override or recover from failed detection/movement
- [ ] Engine difficulty and time control settings in a game configure screen
