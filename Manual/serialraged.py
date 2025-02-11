import serial # FOR CONTROLLING FROM SERIAL PORT
import keyboard # FOR TAKING KEYS INPUTS FROM KEYBOARD
 
# SELECTING SERIAL PORT
arduino = serial.Serial("COM3", 9600, timeout=1) 

while True:

    # For Forward Movement
    if keyboard.is_pressed('w'):

        # Givng Command
        arduino.write(b's')

        # Infinite Loop when W is Pressed 
        while keyboard.is_pressed('w'):
            pass

        # When W is released send value of N
        arduino.write(b'q')
        
    # For Left Movement
    if keyboard.is_pressed('a'):

        arduino.write(b'a')
        while keyboard.is_pressed('a'):
            pass
        arduino.write(b'q')
        
    # For Right Movement    
    if keyboard.is_pressed('d'):

        arduino.write(b'd')
        while keyboard.is_pressed('d'):
            pass
        arduino.write(b'q')
         
    # For Reverse Movement    
    if keyboard.is_pressed('s'):

        arduino.write(b'w')
        while keyboard.is_pressed('s'):
            pass
        arduino.write(b'q')

    # For Camera Servo (To make it Face Downwards) 
    if keyboard.is_pressed('z'):

        arduino.write(b'z')
        while keyboard.is_pressed('z'):
            pass
        arduino.write(b'q')

    # For Camera Servo (To retract it to Orginal Position)  
    if keyboard.is_pressed('x'):

        arduino.write(b'x')
        while keyboard.is_pressed('x'):
            pass
        arduino.write(b'q')

    # For Bin Servo to Change its Position
    if keyboard.is_pressed('b'):

        arduino.write(b'b')
        while keyboard.is_pressed('b'):
            pass
        arduino.write(b'q')

    # For Claws Servo to Open    
    if keyboard.is_pressed('n'):

        arduino.write(b'n')
        while keyboard.is_pressed('n'):
            pass
        arduino.write(b'q')

    # For Claws Servo to Retract
    if keyboard.is_pressed('m'):
        arduino.write(b'm')
         
         
        while keyboard.is_pressed('m'):
            pass
        arduino.write(b'q')

    # For Lid to Open (Downward)
    if keyboard.is_pressed('l'):

        arduino.write(b'l')
        while keyboard.is_pressed('l'):
            pass
        arduino.write(b'q')
    # For Lid to Close (Upward)
    if keyboard.is_pressed('k'):

        arduino.write(b'k')
        while keyboard.is_pressed('k'):
            pass
        arduino.write(b'q')
