**td-hv** is a HV/EHT driver board. It uses a K7000 clone replacement flyback (053X0528-001).

The circuit is a classic current mode flyback driver circuit based on the UCC3801. The circuit integrates secondary-side feedback by (ab)using the resistive divider created by the focus and screen potentiometers of the flyback, inspired by the drive circuit on the MS-2931 and similar monitors.

The primary drive transistor is a SiC MOSFET which has a sufficiently low gate drive voltage to be driven directly by the 12V gate drive output of the UCC3801.

The circuit free-runs at 14kHz, but can by synchronized to a higher frequency with a sync input pin, which is critical for not having a jittery iamge.

The circuit is currently very unstable. The primary problem is that the enormous amount of leakage inductance and capacitance of the flyback causes very slow ringing of the primary current. If the duty cycle and frequency of the switching circuit turn on the MOSFET while the switching node is at a high voltage (e.g. more than 100V) the sudden amount of current produces a voltage spike that confuses the current mode sensing of the UCC3801. Series LR damping has proven only marginally effective.

There is also an X-Ray protection circuit which provides a redundant shutdown path by measuring the peak voltage off a tertiary winding of the flyback.
