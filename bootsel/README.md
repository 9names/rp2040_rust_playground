# Bootsel demo

This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.  
Whenever the bootsel button is held down the LED will blink faster


Notes:
- This demo runs entirely from RAM, since it's tricky to read bootsel
when a program is running from flash. 
