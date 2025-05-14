# ToyTran: A toy circuit (or rather, network) transient simulator

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

### Commands for transient simulation
`.tran [name] tstep tstop`: Specifies simulation time step and total simulation time. The `name` is useful when you would like to run the simulation on the same circuit with different options. `name` part is optional.

`.option [name] method=euler`: Specifies the method used to perform numerical integration. Valid methods are `euler` (backward Euler), `gear2` (Gear2 or BDF2) and `trap` (tapezoidal method).

### Commands for pole-zero analysis

`.pz [name] V(OUT) I(IN)`: Perform pole-zero analysis, and calculate pole-residual values for specified output node, and driver admittance at IN node. (The driver admittance part is still under development.)

`.option [name] pzorder=N` will be added, where the `N` means at most N pairs of poles and zeros will be calculated and used to approximate the output waveform.

### Global commands

`.debug 1`: Enable debug output to print MNA matrix and RHS vector, as well as solution to each time step.

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

### Full stage delay calculation

The ultimate goal of this project is a full stage delay calculation engine, which calculates both cell arc delay (gate delay) and the network delay connected to the driver pin of the cell arc. To support this goal, new commands and options will be added, as drafted below:

`.lib lib_file`: Specifies the path of the library file.

`Xinst LibCellName pinA nodeA pinB nodeB ...`: Instantiates the standard cell. `LibCellName` shoule match the one in library file. `pinX` specifies the pin name of the gate cell, and `nodeX` specifies the node connected to `pinX`. 

`.option [name] driver={rampvoltage|current}`: Specifies the driver model of cell timing arcs. `rampvoltage` means a ramp voltage source, in series to a resistor connected to the voltage source, will be used to model the driver pin behavior. The details are described in "Performance computation for precharacterized CMOS gates with RC loads". `current` means a current source will be used to model the driver bahavior.

`.option [name] loader={fixed|varied}`: Specifies the behavior of the load capacitor of the loader pin. `fixed` means a fixed value will be used for the capacitor, whereas `varied` means the capacitor value will change, and the values come from receiver cap LUT.

`.option [name] net={tran|awe}`: Specifies how the RC network will be handled in delay calculation. `tran` means transient simulation will be used to calculate net delay, and `awe` means pole-zero analysis will be used.

`.delay`: Sets the analysis mode to full stage delay calculation. Internally the `X` devices, or standard cells, will be elaborated with basic devices, thus new devices and nodes will be created, based on the specified driver model and loader model. Specifically:

`driver=rampvoltage` creates new devices `inst/driverPin/Vd` as the ramp voltage source, `inst/driverPin/Rd` as the resistor connected to the ramp voltage source, and new node `inst/driverPin/VPOS` as the positive terminal of the ramp voltage source. The internal structure of the driver model is shown as below:

 ```
  +---------------------------------------------------------------+  
  |                                                               |  
  |                    Instance/driverPin/VPOS                    |  
  | loaderPin                 |                         driverPin |  
+---+                        v           +--------+             +---+
|   +---------+           +--------------|        +-------------+   |
+---+         |           |              +----+---+             +---+
  |            |           |                  ^                   |  
  |            |           |                  |                   |  
  |            |           |          Instance/driverPin/Rd       |  
  |         ---+---     +-----+                                   |  
  |    +-->             |  ^  |<--Instance/driverPin/Vd           |  
  |    |    ---+---     |  |  |                                   |  
  |    |       |        +--+--+                                   |  
  |    |       |           |                                      |  
  |    |       |           |                                      |  
  |    |       |           |                                      |  
  |    |    ---+---      --+--                                    |  
  |    |     -----        ---                                     |  
  |    |      ---          -                                      |  
  |  Instance/loaderPin/Cl                                        |  
  |                                         Instance              |  
  +---------------------------------------------------------------+  
```

`driver=current` creates new devices `inst/driverPin/Id` as the current source.

Loader models are the same on circuit structures, that create new capacitor `inst/loadPin/Cl`. The difference between `fixed` and `varied` are the values of the capacitor.
