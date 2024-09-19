# ToyTran: A toy circuit (or rather, network) transient simulator

## STILL UNDER DEVELOPMENT, but usable for simple cases

## Supported devices
Voltage: `Vname N+ N- value/pwl(t v t v ...)`

Current: `Iname N+ N- value/pwl(t v t v ...)`

Resistor: `Rname N+ N- value`

Capacitor: `Cname N+ N- value`

Inductor: `Lname N+ N- value`

VCVS: `Ename N+ N- NC+ NC- Value`

VCCS: `Gname N+ N- NC+ NC- Value`

CCVS: `Hname N+ N- NC+ NC- Value`

CCCS: `Fname N+ N- NC+ NC- Value`

## Supported commands and options
`.tran tstep tstop`: Specifies simulation time step and total simulation time.

`.debug 1`: Enable debug output to print MNA matrix and RHS vector, as well as solution to each time step.

`.option method=euler`: Specifies the method used to perform numerical integration. Valid methods are `euler` (backward Euler), `gear2` (Gear2 or BDF2) and `trap` (tapezoidal method).

`.plot V(NodeName) I(DeviceName)`: Generate a simple ASCII plot in terminal for easier debugging. Under developing.

`.measure tran variable_name trig V(node)/I(device)=trigger_value TD=xx targ V(node)/I(device)=target_value`: Measure the event time between trigger value happend and target value happend. 

## Compile and run
`git clone --recurse-submodules` and `make` should be sufficient. The executable is generated under current code directory and named "trans".

To run, just give the executable the spice deck you want to simulate. 

## Examples
`./trans circuit/rc.cir` gives the exponential curve of a capacitor being charged, as well as an example for `.measure` commands.

`./trans circuit/lc.cir` produces a oscillation curve of an LC circuit.

`./trans circuit/xtalk.cir` gives an example of the voltage curve of a capacitor with an aggressor toggling beside it.

`./trans circuit/network.cir` gives an example of simulation of a larger RC network.

## File format of tr0
https://github.com/l-chang/gwave/blob/b362dd6d98c255b35a96d9a69a80563b26c2612c/doc/hspice-output.txt

The output tr0 format still cannot be recognized by waveform viewer tools, not sure where the problem is.
