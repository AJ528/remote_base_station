

## system architecture

Overall idea is to not store specific codes on the base station (except for the sensor LED commands). Instead, the remote will send the command format, device number, subdevice number, and function number. The base station will use this information to create the proper series of pulses to send out the IR blasters.

The base station will have a queue to hold IR commands before they are executed.

How to handle repeats that are not "send the same command again"?

How to send commands from remote to base station? What format to use?


## error conditions

- if a command is sent to toggle sensor 0 led, and no sensor is plugged in, throw an error
- if a command is sent to toggle sensor 0 led, and sending the command doesn't toggle the led, throw an error
- try to make it so any error states (even fundamental errors?) end up with the red LED glowing


