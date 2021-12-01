td-180v is a power supply designed to bolt on the back of a Nanao MS9 neck board and provide the cathode amplifier and heater voltages, independent of the B+ regulation of the main PCB. It also provides an optional linear regulator for the heater.

It is a simple asynchronous boost converter design based on the (relatively expensive) LT3758A. It is powered by the B+ rail input, and also requires a +12V rail for logic. As a result it can only regulate to voltages higher than B+.

The current design is too noisy for practical use.
