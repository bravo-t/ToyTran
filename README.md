# ToyTran: A toy circuit (or rather, network) transient simulator

## STILL UNDER DEVELOPMENT

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

`.option method=gear2`: Specifies the method used to perform numerical integration. Valid methods are `euler` (backward Euler) and `gear2` (Gear2 or BDF2). More to come.

`.plot V(NodeName) I(DeviceName)`: Generate a simple ASCII plot in terminal for easier debugging. Under developing.

## Compile and run
`git clone --recurse-submodules` and `make` should be sufficient. The executable is generated under current code directory and named "trans".

To run, just give the executable the spice deck you want to simulate. For example `./trans circuit/rc.cir`.

## File format of tr0
https://github.com/l-chang/gwave/blob/b362dd6d98c255b35a96d9a69a80563b26c2612c/doc/hspice-output.txt
