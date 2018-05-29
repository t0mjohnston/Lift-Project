# Lift-Project
COMP2121 Lift emulator final project

# Table of Contents
   1. Description
   2. Requirements
   3. Advanced Features 

## 1. Description
In this project, you will develop a lift emulator system to be used on the AVR development board.
Generally, a lift (elevator) controller needs to control commands from many sources such as internal
floor select buttons, door control buttons and external call buttons on each floor. Your task in this
project is to emulate a simplified lift controller operating on the AVR microcontroller chip. You will
be demonstrating door operation as well as moving the lift between floors.
The lift you will be emulating has a single call button on each floor, and floor select, emergency,
open and close buttons inside the lift. Your lift emulating system should satisfy the following
requirements. Marks will be allocated amongst the points listed in the following section. More marks
will be given to more difficult features such as correctly servicing requests and correctly operating
the door

## 2. Requirements
1. Your lift can travel up and down amongst 10 floors numbered 0-9.
2. The keypad buttons 0-9 represent the floor select and call buttons. If one of these keys is pushed
it means someone wants to get on or off at that floor.
3. Multiple requests may be given and should be taken. i.e., somebody may push the button on
floor 8 while the lift is moving up to floor 7. The lift should service the requests as in a real lift
scenario. For example, when the lift is passing the 5th floor upwards to the 8th floor, the request
for floor 3 and 4 will be serviced after servicing floor 8.
4. The lift takes 2 second to travel between floors.
5. If the lift stops at a floor, it will open and close its door. The process for this is:
a. Open the door. This takes 1 second.
b. Leave the door open for three seconds.
c. Close the door. This takes 1 second.
6. Use the right push button as a Close button inside the lift. It should operate as follows:
a. If the Close button is pushed while the door is open, start closing the door without
waiting.
b. The Close button will NOT cancel the opening sequence (just the waiting sequence).
7. Use the ‘*’ button on the keypad for emergency. Pushing the emergency button should stop the
servicing of the lift (if the door is opening, it should close) and the lift should go to the ground
floor (i.e., floor 0), the lift should then open and close to get the people out, and the lift should
halt. The LCD should show the following message
“Emergency
Call 000”
The strobe LED should blink several times per second to denote the alarm for emergency. The
lift should resume normal operation only when the ‘*’ button is pressed again.
8. The LEDs should indicate the direction of the lift while moving, and the status of the door (i.e.,
open, close and moving) while the lift is stopped, in a format of your choice.
9. The current floor number should be displayed on the LCD in any way you please.
10. To indicate the door is opening or closing, the motor should spin at full speed.

## 3. Advanced Features 
Use the left push button as an Open button inside the lift. It should operate as follows:
1. If the Open button is pushed while the lift is stopped at a floor, it should open according to the
procedure in Point 5.
2. If the Open button is pushed while the lift is closing, the door should stop closing, re-open and
continue operating as in Point 5.
3. If the Open button is held down while the door is open, the door should remain open until the
button is released.
4. The Open button will NOT function while the lift is moving between floors.
