# ArduinoQuadcopter2
A revisit to my [*old project*](https://github.com/domRG/ArduinoQuadcopter) starting a fresh software build

## Current Controls
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

### Interaction
#### To Fly:
1. <span style="background-color:#ffa500;">{T}</span>		LOW
2. <span style="background-color:#6a5acd;">{Psw}</span>	LOW
3. <span style="background-color:#ff0000;">{ARM}</span>	HIGH

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
		<td> <mark>Large</mark> Increments </td>
	</tr>
	<tr>
		<td> <span style="background-color:#006fff;">{P}</span> </td>
		<td> + / - </td>
		<td> Increase / Decrease </td>
		<td> <mark>Small</mark> Increments </td>
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
2. <mark>Do not touch the quad</mark>
2. <span style="background-color:#ff0000;">{ARM}</span> - LOW
2. <span style="background-color:#6a5acd;">{Psw}</span> - HIGH
2.
  1. <span style="background-color:#ee82ee;">{Y}</span>	LOW
  2. <span style="background-color:#ffa500;">{T}</span> - HIGH <br /> <span style="background-color:#3cb371;">{R}</span> - LOW <br /> <span style="background-color:#006fff;">{P}</span> - HIGH
  2. <span style="background-color:#ffa500;">{T}</span> - LOW
  2. <span style="background-color:#3cb371;">{R}</span> & <span style="background-color:#006fff;">{P}</span> & <span style="background-color:#ee82ee;">{Y}</span> - CENTRE
2. <mark>Wait for 10 seconds</mark>
2. <mark>Quad may now be moved</mark>
2. <span style="background-color:#6a5acd;">{Psw}</span> LOW to exit

## Caution

When <span style="background-color:#6a5acd;">{Psw}</span> and <span style="background-color:#ffa500;">{T}</span> are not LOW, they will block startup when <span style="background-color:#ff0000;">{ARM}</span> switch to HIGH
<b>BUT</b> <i>at the moment</i> will release to ARMED, so always double check <span style="background-color:#ff0000;">{ARM}</span> is LOW before exiting Programming Mode (<span style="background-color:#6a5acd;">{Psw}</span> - LOW)

---
### Big-Picture TODO
- [ ]Angle control loop on top of current rate control loop
- [ ]Require cycle of <span style="background-color:#ff0000;">{ARM}</span> if blocked on initial attempt
- [ ]Restructure code for improved encapsulation
