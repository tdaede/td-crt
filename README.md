# td-crt

This repo contains work-in-progress electronics to control a CRT monitor (aka a CRT chassis).

It is divided up into several separate projects, as follows:

* **[td-deflect](td-deflect/)** - Main PCB, containing microcontroller, horizontal and vertical deflection circuits, RGB amplifier, B+ power supply, 12V and -12V power supplies, and heater power supply. Functional with an enormous amount of rework.

* **[td-hv](td-hv/)** - Regulated HV/focus/screen power supply, based on K7000 clone flyback. Currently marginally functional, demos were done with the original monitor's HV.

* **[td-rgbinput](td-rgbinput/)** - Poorly named RGB amplifier that replaces the version on td-deflect, using more modern parts. Currently untested.

* **[td-180v](td-180v/)** - 180V (and heater) boost converter, designed to be installed on a neck board to provide an optional dedicated power supply for the cathode amplifiers.

* **[td-rgb](td-rgb/)** - Old RGB mod PCB, I should move it to a separate repo at some point.
