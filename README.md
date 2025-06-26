# ToyTran: A toy circuit (or rather, network) transient simulator

## NOTE: All delay calculation related parts have been moving to [ToyDelay](https://github.com/bravo-t/ToyDelay)

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

### Commands and options for transient simulation
`.tran [name] tstep tstop`: Specifies simulation time step and total simulation time. The `name` is useful when you would like to run the simulation on the same circuit with different options. `name` part is optional.

`.option [name] method=euler`: Specifies the method used to perform numerical integration. Valid methods are `euler` (backward Euler), `gear2` (Gear2 or BDF2) and `trap` (tapezoidal method).

### Commands and options for pole-zero analysis

`.pz [name] V(OUT) I(IN)`: Perform pole-zero analysis, and calculate pole-residual values for specified output node, and driver admittance at IN node. (The driver admittance part is still under development.)

`.option [name] pzorder=N` will be added, where the `N` means at most N pairs of poles and zeros will be calculated and used to approximate the output waveform.

### Global commands

`.debug [module] 1`: Enable debug output. This command now supports enable debug information for specified modules only, if `module` is omitted, debug information for all modules are enabled. Valid module names are `all` for enabling all modules, `root` for root solver, `sim` for transient simulation, `circuit` for circuit building, `pz` for pole-zero analysis.

`.plot tran [width=xx height=xx canvas=xxx] [name.]V(NodeName) [name.]I(DeviceName)`: Generate a simple ASCII plot in terminal for easier debugging. If `width` and `height` directives are not given, the tool will use current terminal size for plot width and height. Multiple simulation results can be plotted in a single chart by specifying a canvas name. Currently at most 4 plots can be drawn in one canvas. Now the command can plot data from different analysis data into one canvas, specified with `name.` prefix. (This command is not supported in PZ analysis.)

`.measure tran[.name] variable_name trig V(node)/I(device)=trigger_value TD=xx targ V(node)/I(device)=target_value`: Measure the event time between trigger value happend and target value happend. (This command is not supported in PZ analysis.)

## Compile and run
`git clone --recurse-submodules` and `make` should be sufficient. The executable is generated under current code directory and named "trans".

To run, just give the executable the spice deck you want to simulate. 

## Examples
`./trans circuit/rc.cir` gives the exponential curve of a capacitor being charged, as well as an example for `.measure` commands.

`./trans circuit/lc.cir` produces a oscillation curve of an LC circuit.

`./trans circuit/xtalk.cir` gives an example of the voltage curve of a capacitor with an aggressor toggling beside it, as well as the canvas-ed `.plot` command.

`./trans circuit/network.cir` gives an example of simulation of a larger RC network.

## File format of tr0
https://github.com/l-chang/gwave/blob/b362dd6d98c255b35a96d9a69a80563b26c2612c/doc/hspice-output.txt

The output tr0 format still cannot be recognized by waveform viewer tools, not sure where the problem is.

## Features under development

Note below features will not follow spice netlist grammar.

### Transfer function analysis

New instruction `.TF` will be implemented. Details TBD.
