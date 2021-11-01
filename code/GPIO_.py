""""this code is used to test the GPIO pins connections.
    In order to check if the pin works change output pin
"""

import RPi.GPIO as GPIO
import time

# Pin Definitions
# link to table of reference, for pin numbers, using BCM mode:
# https://www.rs-online.com/designspark/rel-assets/dsauto/temp/uploaded/0523.png?w=1042

output_pin = 18  # CHANGE TO TEST


##Pins I've used##
# BOARD pin 12 = BCM pin 18 ->left person
# Board 7 = 4 BCM -> left car
# Board 13=27 BCM -> buzzer
# Board 19= 10 BCM -> right person
# Board 26= 7 BCM -> right car

def main():
    # Pin Setup:
    GPIO.setmode(GPIO.BCM)
    # set pin as an output pin with optional initial state of HIGH
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)

    print("Starting demo now! Press CTRL+C to exit")
    curr_value = GPIO.HIGH  # save current value

    try:
        while True:
            time.sleep(1)
            # Toggle the output every second
            print("Outputting {} to pin {}".format(curr_value, output_pin))
            GPIO.output(output_pin2, curr_value)
            curr_value ^= GPIO.HIGH
    finally:
        GPIO.cleanup()


if __name__ == '__main__':
    main()
