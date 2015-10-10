cx10_fnrf
=========

Cheerson CX10 rate mode firmware employing integrated RF controller and stock TX.

1. About

  The Cheerson CX10 is a quadcopter incorporating an ARM Cortex M0 MCU, Invensense MPU-6050 and Beken BK2423 or Panchip XN297 (Nordic NRF24 clones). In 2014, Felix Niessen released [alternative firmware](http://www.rcgroups.com/forums/showpost.php?p=30045580&postcount=1) for the original (red PCB) device which allows it to be flown in rate mode using a standard PPM receiver.

  This code builds upon Felix's work by enabling the original red, blue, and new red PCB versions to be flown using the onboard radio. The devices can be controlled using the original controller, a Walkera radio with [DeviationTX])( (http://www.deviationtx.com) firmware, or any standard radio using [Goebish' nrf24 JR module](https://github.com/goebish/nrf24_multipro).

2. Building

  Current binaries are provided in the /builds subdirectory, produced from both the Keil MDK-ARM toolchain, and the GNU ARM toolchain. The binaries may not be flight tested.

  A Keil uVision v5 project file is included to build/debug using MDK-ARM on Windows platforms (note that the code-size is such that the 32Kb-limited evaluation version may be used). A modification of the original Makefile is also included, which will build the firmware using a suitable GNU ARM toolchain under OS X, and presumably, Linux.

3. Programming

  To program the firmware will require that you connect to the SWD pads on the CX-10, before using an appropriate programming tool. Note that the device is level-1 protected by default, and these option bytes must be cleared before programming. Alternatively, the embedded UART boot-loader may be used to program the device. See the original firmware thread for more details.

4. Operation

  + Upon power-up the CX-10 will try to bind to a suitable TX, during this phase the LEDs will light alternately, flashing at a slow rate.
  + When bound to a transmitter, but waiting for valid data, the LEDs will flash rapidly.
  + Once valid data is received, the LEDs on one side of the device will illuminate, this indicates that the device is bound and disarmed
  + Use a 'forward-flip' command on the stock TX to arm (internally setting Aux 1 high). On a Mode 2 transmitter this involves pressing the right stick then pushing full forwards. This must be performed at zero-throttle. All lights will illuminate when armed.
  + Disarm using a 'backwards-flip' command on the stock TX. If multiple RF packets are lost, the device will disarm automatically.
  + Under low battery conditions, the LEDs will blink.

5. Credits

  + Felix Niessen's [original work](http://www.rcgroups.com/forums/showpost.php?p=30045580&postcount=1).
  + Bitcraze (for their [nrf24 driver](ttp://www.bitcraze.se)).
  + [Goebish](http://www.rcgroups.com/forums/member.php?u=302980), Victzh, and others associated with the DeviationTX project (for the reverse engineering of the red, green and blue protocols, and XN297/nRF24 interfacing).

6. Porting/future development

  + Support for the CX-10 green board requires knowledge of the pinout, protocol is known.
  + With little modification this code should also run on the Cheerson CX-11, which I believe also uses the Beken BK2423 RF chip.
