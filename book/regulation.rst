Regulation
==========

There are a huge amount of ways to acheive power supply regulation in CRTs.

None
----
In this method, common on very old monitors, the B+ is directly derived by rectifying mains voltage, and this is used to power the main horizontal output circuit. All other voltages are derived from flyback windings. The circuit is carefully designed so that B+ or other voltage droops cancel out or are otherwise not too much of a problem.

Basic B+ regulation
-------------------
In this case, B+ is not simply rectified mains, but is regulated, e.g either with a separate flyback or buck regulator, before going onward to the horizontal / main flyback circuit.

Tertiary flyback winding regulation
-----------------------------------
In this case, the B+ voltage is not just constant, but rather it is regulated using a tertiary winding off of the main flyback. This means that the B+ voltage is more variable, but all supplies derived from the flyback (horizontal output, EHT, heater, etc) are more tightly regulated. This is the most common designed, used in e.g. the NEC Multisync 1401.

Width may be additionally regulated, e.g. using a magnetic amplifier.

Independent horizontal and EHT regulation
-----------------------------------------

In this case, the horizontal deflection and the flyback for EHT generation are completely separate. In this case, the B+ voltage is constant, and the EHT can be more tightly regulated in a couple of ways:

1. The duty cycle of the drive signal for the EHT flyback's transistor can be regulated. This is the method used in e.g. the Nanao MS-2931. Unfortunately, when locked to the horizontal scan rate, this prevents the usage of zero-voltage-switching (ZVS).

2. The B+ is additionally pre-regulated with a buck regulator or other switch ahead of the EHT flyback circuit. This allows the flyback to operate in ZVS mode, and the particular duty cycle of its transistor is not important (as long as it turns on while its parallel / intrinsic diode is conducting).

EHT feedback
~~~~~~~~~~~~

While a tertiary winding could be used for feedback, in the case of these more advanced designs, a more accurate measurement of EHT is desirable. A common method is a resistor divider built into the flyback attached to EHT, sometimes incorporating the focus and screen pots as part of the divider. This is used on the MS-2931. However, this has the drawback that this resistor divider must be very high impedance due to the low voltage, which makes it very sensitive to capacitive coupling from the flyback. Sufficiently filtering the signal to avoid crosstalk from flyback pulses leads to very low bandwidth. To remedy this, higher end monitors like the Sony FW900 incorporate both DC and AC feedback paths in the flyback. The DC path is a low-passed resistor divider, and the AC path is a very high voltage capacitor coupled to the EHT output. Both of these signals are mixed to produce a single feedback signal with both high and low frequency components.
