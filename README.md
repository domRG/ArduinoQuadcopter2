
# ArduinoQuadcopter2
A revisit to my [*old project*](https://github.com/domRG/ArduinoQuadcopter) starting a fresh software build

Controls we're outdated.

Currently functions in a [Finite State Machine](https://www.drdobbs.com/cpp/state-machine-design-in-c/184401236) format
Essentially it uses one auxilliary switch to arm/disarm the motors, and the seconds aux switch to toggle angle/rate control (when armed) and to enter programming mode (when disarmed), with lock-states to prevent accidental arming.
For further details look in [the code](https://github.com/domRG/ArduinoQuadcopter2/blob/master/quad_t4_1.0/quad_t4_1.0.ino), specifically `int runRunState(){` currently at line 92.

The outdated control guide can be found at the bottom. This should generally be accurate, though functionality of the auxilliary switches has been change (to implement the state-machine above) so you should be exercised with caution!
(I recommend doing a run through with the propellers disconnected and the Serial port connected to see feedback of what's going on.)

## Caution

Always take care when <b>arming</b>.

---
### Big-Picture TODO
- [x] Angle control loop on top of current rate control loop
- [x] Require cycle of <span style="background-color:#ff0000;">{ARM}</span> if blocked on initial attempt
- [ ] Restructure code for improved encapsulation

---
#### Viewing trouble
View source-code of README.md using https://markdown-editor.github.io/ for colour coding


## ***Outdated*** Controls (for reference only)
#### Channel Map
| Channel | Assignment | Reference | Extra Details |
| :-: | :-: | :-: | :-: |
| 1 | Roll | <span style="background-color:#3cb371;">{R}</span> | HIGH = Stick RIGHT |
| 2 | Pitch | <span style="background-color:#006fff;">{P}</span> | HIGH = Stick UP |
| 3 | Throttle | <span style="background-color:#ffa500;">{T}</span> | HIGH = Stick UP
| 4 | Yaw | <span style="background-color:#ee82ee;">{Y}</span> | HIGH = Stick Right |
| 5 | Programming <br /> Mode Switch | <span style="background-color:#6a5acd;">{Psw}</span> | HIGH = ON |
| 6 | Arm Switch | <span style="background-color:#ff0000;">{ARM}</span> | HIGH = ON |

(CAUTION: Extra Details may be controller specific - I do not know!)

### Interaction (outdated - not necessarily correct, please examine code for more information)
#### To Fly:
1. <span style="background-color:#ffa500;">{T}</span> - LOW
2. <span style="background-color:#6a5acd;">{Psw}</span> - LOW
3. <span style="background-color:#ff0000;">{ARM}</span> - HIGH

Motors should now idle, quad is ready to fly.

#### To Tune PID:
1. <span style="background-color:#ff0000;">{ARM}</span> - LOW
2. <span style="background-color:#6a5acd;">{Psw}</span> - HIGH
3. <span style="background-color:#ee82ee;">{Y}</span> - CENTRED
4. Position <span style="background-color:#ffa500;">{T}</span> for parameter:
	<table>
		<tr>
			<th>Parameter</th>
			<th><span style="background-color:#ffa500;">{T}</span> Position</th>
		</th>
		<tr>
			<td> P </td>
			<td> HIGH </td>
		</tr>
		<tr>
			<td> I </td>
			<td> CENTRE </td>
		</tr>
		<tr>
			<td> D </td>
			<td> LOW </td>
		</tr>
	</table>
5. Use <span style="background-color:#3cb371;">{R}</span> and <span style="background-color:#006fff;">{P}</span> to adjust tuning (this is saved in memory - persistent across power cycles)
	<table>
		<tr>
			<td> <span style="background-color:#3cb371;">{R}</span> </td>
			<td> + / - </td>
			<td> Increase / Decrease </td>
			<td> <b><mark>Large<mark></b> Increments </td>
		</tr>
		<tr>
			<td> <span style="background-color:#006fff;">{P}</span> </td>
			<td> + / - </td>
			<td> Increase / Decrease </td>
			<td> <b><mark>Small<mark></b> Increments </td>
		</tr>
	</table>
6. When complete, <span style="background-color:#6a5acd;">{Psw}</span> LOW to exit Programming Mode

##### To reset PID tunings to 0

1. <span style="background-color:#ff0000;">{ARM}</span> - LOW
2. <span style="background-color:#6a5acd;">{Psw}</span> - HIGH
2. 
	1. <span style="background-color:#ee82ee;">{Y}</span> - HIGH
	2. <span style="background-color:#ffa500;">{T}</span> - LOW <br /> <span style="background-color:#3cb371;">{R}</span> - HIGH <br /> <span style="background-color:#006fff;">{P}</span> - LOW
	2. <span style="background-color:#3cb371;">{R}</span> & <span style="background-color:#006fff;">{P}</span> - CENTRE
	2. <span style="background-color:#ee82ee;">{Y}</span> - CENTRE

#### To Re-Calibrate the gyro:
1. Place quad down on a completely still surface (calibrates rate of rotation)
2. <b><mark>Do not touch the quad</mark></b>
2. <span style="background-color:#ff0000;">{ARM}</span> - LOW
2. <span style="background-color:#6a5acd;">{Psw}</span> - HIGH
2. 
	1. <span style="background-color:#ee82ee;">{Y}</span> - LOW
	2. <span style="background-color:#ffa500;">{T}</span> - HIGH <br /> <span style="background-color:#3cb371;">{R}</span> - LOW <br /> <span style="background-color:#006fff;">{P}</span> - HIGH
	2. <span style="background-color:#ffa500;">{T}</span> - LOW
	2. <span style="background-color:#3cb371;">{R}</span> & <span style="background-color:#006fff;">{P}</span> & <span style="background-color:#ee82ee;">{Y}</span> - CENTRE
2. <b><mark>Wait for 10 seconds<mark></b>
2. <b><mark>Quad may now be moved<mark></b>
2. <span style="background-color:#6a5acd;">{Psw}</span> - LOW, to exit
