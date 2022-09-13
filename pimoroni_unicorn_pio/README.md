# pimoroni_unicorn
Simple test of the pimoroni unicorn - bitbang each pixel of the matrix

Only allows each LED to be off or on, so there is only 8 possible colours: black white red green blue cyan magenta yellow

This program exists to show how data is clocked into the device.
We could quickly cycle colours to approximate shades of colour but it would be better to use the PIO program that Pimoroni wrote for this purpose.

LED scanning is based on the PIO program anyway:  
https://github.com/pimoroni/pimoroni-pico/blob/main/libraries/pico_unicorn/pico_unicorn.pio
