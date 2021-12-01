**td-deflect** is the primary PCB of td-crt, containing most of the pieces of a complete chassis.

The chassis is a multisync design with separate HV and deflection, allowing arbitrary scan rates, within the voltage and current limits of the ciruit.

RGB input
---------

There is a completely analog RGB path, based on a M51387P RGB amplifier IC. While effective, the chip has been out of production for a long time and therefore the **td-rgbinput** board exists as a future design that eliminates the IC.

Sync processing
---------------

The sync is separated by a LMH1980 sync separator IC. This is responsible for AC coupling the sync input, and separating it into separate H and V syncs (in the case of composite sync). It is also relied on for the back porch output, which is fed in to the clamping circuts, and the odd/even output, which is used for interlacing support.

Horizontal PLL
--------------

There is no analog horizontal PLL - instead, the horizontal sync is fed directly into the timers of the STM32F7. A software PLL is implemented using the internal timers. While this leads to quite robust sync processing, it also results in a minimum PLL jitter of a single clock period of 4.6ns. This is adequate for 15kHz content but not ideal. A future board may use the STM32G4 instead, which has an internal DLL that leads to minimum jitter on the order of 184ps.

Vertical ramp generator
-----------------------

The vertical ramp is generated in software. A software correction is applied for linearity, after which it is output via the stm32's DAC.

Vertical amplifier
------------------

The vertical amplifier is a classic discrete class AB amp, with current feedback (a voltage input corresponds to a current setpoint). The feedback network is fixed in analog components, which turns out not to be flexible enough for all yoke designs. In addition, it does not actually have any AB biasing, instead relying on feedback to avoid crossover distortion, which limits its performance. That said, it's quite effective as is. The amplifer is DC-coupled so that vertical position offsets are simply a DC bias on the input.

One leg of the amplifier is powered by 12V, and the other is powered by 48V in order to provide a faster retrace interval with high inductance yokes.

A planned future design replaces the vertical amplifier with a class D circuit with software feedback, to allow software tuning of the feedback network as well as increased power efficiency, though with the added complexity of dampled RLC output filter design.

Horizontal circuit
------------------

The horizontal circuit is a classic CRT horizontal class-C amplifier, which recovers the energy stored in the yoke for retrace.

The horizontal output transistor is a silicon carbide MOSFET for its very high voltage handling abilities. BJT-based HOTs are still manufactured as a backup option, however SiC MOSFETs are actually cheaper now and offer better performance. In addition, their intrinisic body diode renders an external freewheeling diode unnecessary.

A fixed film cap is used as the retrace timing capacitor.

There is a bank of 4 switchable S-capacitors allowing for 16 total possible S correction values.

There is no E-W correction or horizontal DC bias adjustment.

There is a current sense transformer provided for current feedback, but it is currently untested.

B+ power supply
---------------

The B+ power supply is a 100-200V boost converter to power the horizontal deflection circuits. It is based on a LT3758A asynchronous boost controller and is capable of up to 500mA of output current. It is currently thermally limited and could use a bit of work to increase efficiency and therefore boost the output power.

12V power supply
----------------

There is a 12V buck converter for 12V.

Negative 12V power supply
-----------------

There is a boost topology -12V converter. It currently does not work.


Heater power supply
-------------------

A linear regulator provides a heater power supply. Untested.

HV circuit
----------

Part of an extremely basic HV driver is present on the PCB, but is untested.

Software
--------

The on-board STM32 handles horizontal and vertical deflection, though the rest of the board runs without software intervention. The software is based on the [RTIC](https://rtic.rs) framework and written in Rust. It provides a very primitive serial interface to upload configuration and download runtime statistics. The software is also responsible for implementing all of the self protection logic.

There is a companion desktop GTK4 app with an extremely basic "remote" GUI.
