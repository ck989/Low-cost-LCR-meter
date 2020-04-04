# Low-cost-LCR-meter
LCR meter is developed using TM4C123GH6PM microcontroller 

1 Overview

The goal of this project is design a system capable of measuring resistance, inductance (and ESR), and
capacitance. A design goal of this project is to limit the total cost of the daughterboard and components added to the TM4C123GXL evaluation board to 
$3 in 10k quantities.

Since connectors have significant associated cost, two shared connections will be used for the device
under test, regardless of whether a resistive, an inductive, or a capacitive device is measured, so any
commutation is included to allow any attached device to be measured.

The project is provided with a complete user interface through the virtual COM port on the evaluation board.

2 Hardware Description

Microcontroller:
An ARM M4F core (TM4C123GH6PMI microcontroller) is used.
Power LED:
A power LED isonnected through a current-limiting resister to indicate the daughterboard has
power.
Serial interface:
the UART0 tx/rx pair is routed to the ICDI that rovides a virtual COM port through a USB endpoint.
LCR measurement interface:
A circuit is provided is interfaced microcontroller and allow the user to test an L, C, or R
value. The output of this circuit can be connected to the analog comparator and analog-to-digital
converter inputs.

3.3V supply:
The circuit is powered completely from the 3.3V regulator output on the evaluation board.

Device under test (DUT) connection;
Two connectors, made of wire loop to save cost, are required to allow the DUT to be connected.

4 Software Description

A virtual COM power using a 115200 baud, 8N1 protocol with no hardware handshaking shall be provided
with support to the following commands.
Debug:
If “reset” is received, the hardware shall reset.

If “voltage” is received, the hardware shall return the voltage across DUT2-DUT1. The voltage is limited
to 0 to 3.3V.

LCR commands:
If “resistor” is received, return the resistance values from 10ohms to 100ohm.
If “capacitance” is received, return the capacitance value from 1nF to 100uF.
If “inductance” is received, return the inductance value from 1nH to 100mH.
If “esr” is received, return the ESR of the inductor under test.
