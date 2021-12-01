**td-rgbinput** is a replacement RGB analog path, providing the color controls between the RGB input and the final amplification stage on the neck board of the CRT.

Most consumer sets use an integrated RGB amplification IC, however none of these are still in production. Thus, this PCB was designed to instead use modern discrete amplifier components.

The primary challenge of the board is providing the global gain control across all 3 channels. This requires a voltage controlled amplifier, which is a difficult circuit to create when cross channel matching is desired.

The first stage of the input is AC coupling followed by a dc restore circuit, implemented as a clamp to ground. This is less accurate than a dc restore based on an op amp adder, but super simple to implement.

Next, the global gain control is applied via an AD8337 voltage controlled amplifier. The global gain voltage is divided by a per-channel gain adjustment before being fed into the AD8337's gain input.

Following this, a global and per-channel bias is applied via a resistor adder. The global bias voltage is carefully buffered to avoid compromising the high frequency response of the circut. In addition, a blanking input can pull the output voltage down to zero at this stage.

Finally, an amplfiication stage based on the ADA4861 provides a fixed 2x gain, and more importantly, higher current to the output. The outputs include a series 47 ohm resistor with the assumption that they will go directly to the base of transistors on the neck board.

The circuit also has a local 5V supply, -5V supply, and a blank OR-ing circuit for convenience. Termination can be enabled or disabled via a bank of DIP switches.
