Udate on 4in1 module telemetry:
- A7105: telemetry can't work since the PA is always active, TXEN is forced to 3.3V. I've verified on my hubsan no more than 3 meters...

- NRF24L01: telemetry can't work since the PA is always active, TXEN is connected to the NRF VDD_PA pin, 
which I'm not sure it's how it should be done, but since there is an additional pull-up on the TXEN pin of the PA the signal is offset to 1.8V and 2.2V 
when PA should be switched on (a "1" is 1.2V or more). I've verified on my motorcycle no more than 2 meters...

- CYRF telemetry should work since the PA TXEN pin is well driven by the Cyrf XOUT but I'm getting small range with the RX I've got. 
So I will put for now the fault on the RX unless I find another RX to try out.

- CC2500 telemetry should work since the PA TXEN pin is well driven by the GDO0 pin of the CC2500. 
But I've not been able to get my RX F801 working reliably with this 4in1 module. I've tried to change the fine frequency tunning from -127 to +127 
and I'm getting time to time only a sporadic connection (by this I mean the servo move to the correct position but goes back right away to failsafe). 
I've tried my normal v2.3c module and confirm that it works fine with it.

Notes:
-Tthe CC2500 not working reliabily might come from a fault on my module. So I think someone else will have to give it a try and report.
- Above comments about TXEN and PA control have been verified using a scope so I'm sure of what I'm saying.

- Pascal


The RXEN pin is connected to VCC which is fine if you are driving correctly the TXEN pin since basically TXEN=0 -> RX and TXEN=1 ->TX.
The problem is the multiplexer they've added in between the RF chips and the PA/LNA.

The entry which is supposed to be connected to the A7105 is in fact connected to 3.3V.
The entry which should drive the pin TXEN to 1 or 0 from the NRF24L01 has a big pull up which completly shift the level of the control signal making it useless and force TXEN to 1.
The 2 above are the result of a bad design of the rf board.

For the CYRF and CC2500 TXEN is well driven to 1 or 0 so TX and RX should work.
But I haven't been able to use the CC2500 at all. It does not work in TX mode with the RX I have.
The Cyrf TX part works (no idea about range) but I have some doubt about the RX side.
The Cyrf and CC2500 tests just above might be due to my board being defective so I'll wait for someone to confirm.

- Pascal

Please wait for someone else to test the 4in1 RF module before buying it, this is all I can say.

Again here is a summary of my tests on the bench (ie no range test):
A7105
TX=OK, RX=broken at pcb level

NRF24L01
TX=OK, RX=broken at pcb level

CYRF
TX=OK, RX=should work but really small range. To be confirmed by someone else

CC2500
TX=not working, RX should work but can't test without TX part working... To be confirmed by someone else

- Pascal



I'm giving up on the 4in1 module since it's useless to me without the CC2500 working (even after trying to resolder it and all parts around it)... 

Here are the results of my findings:
Multiplexer CD4052 pinout:
Pin 11 <- CYRF6936.XOUT => selected for CYRF
Pin 12 <- 3v3 => selected for A7105
Pin 14 <- CC2500.GDO0 => selected for CC2500
Pin 15 <- NRF24L01.VDD_PA => selected for NRF

Pin 13 -> RFX2401C.TXEN through a 1K resitor and a 4K7 to VCC

The modification idea I had is to cut the trace going out of 4052.pin13 which is easy to do as there is a lot of space. 
Connect one side of a wire to the RFX2401C.TXEN 1K/4K7 resistor which is easy to do as well and then the other side to either 4052.pin11=CYRF or 4052.pin14=CC2500. 
And if possible a second wire from the RFX2401C.RXEN 1K/4K7 resistor to the CYRF6936.PACTL or CC2500.GDO2.
Then change the multi program with a #define 4in1_module the few calls to switch RX/TX/Standby states to use the CC2500 or CYRF6936 GPIOs instead which is trivial.

- Pascal




Pascal.
Why not cut trace on pin 12 and connect the pin to A7105 ...GIO1
You could do that but you would only fix the A7105 telemetry.

Quote:
Originally Posted by midelic  View Post
How about the pullup on nrf chip
You have no way to prevent it since the pullup issue is on the RFX2401C.TXEN 4K7 to VCC. Or you also need to remove the 4K7. But when the GPIOs are not initialized your TXEN pin could be floating and start flapping...

Personnaly, I would remove the 4052 all together and use one of the 4 chips (actually 3 since the NRF has no real pin to drive the PA/LNA...) to decide TX/RX/OFF for the others. This is simple and straight forward. This is what I was trying to explain earlier...

- Pascal
