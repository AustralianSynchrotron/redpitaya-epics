# redpitaya-epics

EPICS driver support for RedPitaya based on asynPortDriver. This module is to be run on the RP itself.

## Getting Started

It's really quite straight forward. Module consists of an EPICS support library and test application. All you need is:
* [RedPitaya](https://www.redpitaya.com)
* [EPICS Base](https://epics.anl.gov/download/base/index.php)
* [asynDriver](https://epics.anl.gov/modules/soft/asyn/) 

### RedPitaya
Code has been developed on and tested with [STEMLab 125-14](https://www.redpitaya.com/f130/STEMlab-board) (originally Red Pitaya v1.1) and RedPitaya library version 0.98-685-0759d71. RedPitaya library comes with the image you get on their website. To set it up, follow their [Quick start](http://redpitaya.readthedocs.io/en/latest/quickStart/quickStart.html) manual.

### EPICS Base
There is nothing too funky in the code so I believe any EPICS Base > 3.14.12.2 should be fine but it has only been tested with 3.15.5 and 3.16.1.

### asynDriver
Same story as with EPICS Base, but it has only been tested with asynDriver 4.26 and 4.31.

## Deployment
It takes quite some time to build EPICS Base and asynDriver on RedPitaya (and hour and 20 minutes for base) so I suggest you cross-compile the two. 

### Prerequisites
First you'll need build toolchain binaries for target architecture. RedPitaya is using Linaro's linux-arm-gnueabihf build toolchain so that's what you should get. I got mine from here: https://releases.linaro.org/components/toolchain/binaries/6.3-2017.02/arm-linux-gnueabihf/. Simply extract it to <arm\_compiler\_dir> and that's it.

### Cross-compiling EPICS Base
There are basically only 4 things you need to change to cross-compile EPICS base once you have your ARM compiler ready:
* <base\_top>/configure/CONFIG_SITE:
  * CROSS\_COMPILER\_TARGET\_ARCHS = linux-arm
  * CROSS\_COMPILER\_HOST\_ARCHS = <your\_host\_arch>
* <base\_top>/configure/os/CONFIG\_SITE.<your\_host\_arch>.linux-arm:
  * GNU\_TARGET = arm-linux-gnueabihf
  * GNU\_DIR = <arm\_compiler\_dir>
  
That should be it. Run make.

### Cross-compiling Other EPICS Components
For other components you want to cross-compile for RedPitaya you just have to make sure that they are being built against the EPICS base you have just built, or any other one that's cross compiled for RedPitaya, and set the target architecture:
* <component\_top>/configure/CONFIG\_SITE:
  * CROSS\_COMPILER\_TARGET\_ARCHS = linux-arm

### Building the Driver Itself
After you have EPICS Base and asynDriver ready (cross-compiled or built on the board) it's time to build this driver. It's designed to be built on the board itself, so begin by copying it to RedPitaya. Once this is done, simply modify the $(TOP)/configure/RELEASE file of this module to point to where your EPICS Base and asynDriver are and hit make.

## Running the Test App

### Stopping the Web Server
By default RedPitaya starts the nginx web server on startup, which can be used to access applications via web browser. Before you start the IOC, make sure this service is turned off. If you don't want it to automatically start on startup, disable it.
```
systemctl stop redpitaya_nginx
systemctl disable redpitaya_nginx
```

### Starting the IOC
After a (hopefully) successful build, you'll end up with two files in your $(TOP)/bin/linux-arm directory. You have to load the FPGA image prior to running the IOC. To do that simply execute the $(TOP)/bin/linux-arm/load\_fpga\_image.sh script as root.
When this is done you can start the IOC in the standard way by running $(TOP)/iocBoot/iocRedPitayaTest/st.cmd

## Usage
This paragraph assumes you're running the IOC from RedPitayaTestApp.

Almost all records also have corresponding <record\_name>\_STATUS and <record\_name>\_MONITOR read records. E.g. SR00RPA01:ACQ\_TRIGGER\_SRC\_CMD has SR00RPA01:ACQ\_TRIGGER\_SRC\_STATUS counterpart and SR00RPA01:ACQ\_TRIGGER\_DELAY\_SP has SR00RPA01:ACQ\_TRIGGER\_DELAY\_MONITOR.

### Data Acquisition

**Data acquisition start/stop**

| Record Name                               | Allowed Values  | Comment                                                                                              |
| ----------------------------------------- | --------------- | ---------------------------------------------------------------------------------------------------  |
| SR00RPA01:START\_CONT\_ACQ\_CMD   | 1               | Start a continuous acquisition. After a trigger, the device will rearm and wait for another trigger. |
| SR00RPA01:START\_SS\_ACQ\_CMD     | 1               | Start a single-shot acquisition. Record data after the fist trigger and then stop acquiring.         |
| SR00RPA01:STOP\_ACQ\_CMD     | 1               | Stops the acquisition.         |
| SR00RPA01:RESET\_ACQ\_CMD     | 1               | Resets all acquisition parameters (trigger source, trigger level, decimation, ...) and stops the acquisition. |

**Data acquisition trigger**

| Record Name                   | Allowed Values  | Comment                                                                                              |
| ------------------------------| --------------- | ---------------------------------------------------------------------------------------------------  |
| SR00RPA01:ACQ\_TRIGGER\_SRC\_CMD   | DISABLED  </br> NOW </br> CH1\_PE </br> CH1\_NE </br> CH2\_PE </br> CH2\_NE </br> EXT\_PE </br> EXT\_NE </br> AWG\_PE </br> AWG\_NE |-> Trigger is disabled </br> -> Trigger triggered now (immediately) <br/> -> Trigger set to Channel 1 threshold positive edge </br> -> Trigger set to Channel 1 threshold negative edge </br> -> Trigger set to Channel 2 threshold positive edge </br> -> Trigger set to Channel 2 threshold negative edge </br> -> Trigger set to external trigger positive edge (DIO0\_P pin) </br> -> Trigger set to external trigger negative edge (DIO0\_P pin) </br> -> Trigger set to arbitrary wave generator application positive edge <br/> ->  Trigger set to arbitrary wave generator application negative edge |
| SR00RPA01:ACQ\_TRIGGER\_DELAY\_SP     | 0 <= *nanos* <= 10000 | Start with the acquisition *nanos* nanoseconds after the triggger.  |
| SR00RPA01:ACQ\_TRIGGER\_LEVEL\_SP     | -20 <= *level* <= 20  | When trigger is set to one if the two channels, trigger when the volate crosses *level* volts. |
| SR00RPA01:ACQ\_TRIGGER\_HYST\_SP     | 0 <= *hyst* <= 1  | Trigger hysteresis in volts. |

**Data acquisitions configuration**

| Record Name                               | Allowed Values  | Comment                                                                                              |
| ----------------------------------------- | --------------- | ---------------------------------------------------------------------------------------------------  |
| SR00RPA01:ACQ\_DECIMATION\_CMD | 1, 8, 64, 1024, 8192, 65536 | Input data decimation values |
| SR00RPA01:ACQ\_SAMPLING\_RATE\_CMD | 125 MHz, 15.6 MHz, 1.9 MHz, 103.8 kHz, 15.2 kHz, 1.9 kHz | Rate at which we're sampling. |
| SR00RPA01:ACQ\_AVERAGING\_CMD | Off, On | Enable or disable averaging |

**Data acquisition per-channel configuration**

| Record Name                               | Allowed Values  | Comment                                                                                              |
| ----------------------------------------- | --------------- | ---------------------------------------------------------------------------------------------------  |
| SR00RPA01:IN<1,2>\_GAIN\_CMD | Low, High | Sets the acquire gain state for channel 1 or 2. The gain should be set to the same value as it is set on the RedPitaya hardware by the LV/HV gain jumpers. Low = LV = 1V; High = HV = 20V. |

**Data reading:**

| Record Name                               | Comment                                                                                              |
| ----------------------------------------- | ---------------------------------------------------------------------------------------------------  |
| SR00RPA01:IN<1,2>\_DATA\_MONITOR | Read the data acquired on channel 1 or 2. |


#### Examples

Use external trigger's positive edge to continuously acquire every 1024th data point:
```
# External positive edge trigger
caput SR00RPA01:ACQ_TRIGGER_SRC_CMD EXT_PE

# Every 1024th point
caput SR00RPA01:ACQ_DECIMATION_CMD 1024

# Start acquisition
caput SR00RPA01:START_CONT_ACQ_CMD 1

# Plot the data somehow by reading the waveform from 
camonitor SR00RPA01:IN1_DATA_MONITOR

# Stop acquisition
caput SR00RPA01:STOP_ACQ_CMD 1
 
```

Manually trigger and rearm after each trigger.:
```
# Reset acquisition config. This sets trigger source to DISABLED
caput SR00RPA01:RESET_ACQ_CMD 1

# Start continuous acquisition 
caput SR00RPA01:START_CONT_ACQ_CMD 1

# Manually fire off a soft internal trigger
caput SR00RPA01:ACQ_TRIGGER_SRC_CMD NOW

# Plot the data somehow by reading the waveform 
caget SR00RPA01:IN1_DATA_MONITOR

# Stop acquisition
caput SR00RPA01:STOP_ACQ_CMD 1

```
### Data Generation

**Output enable/disable**

| Record Name                               | Allowed Values  | Comment                                                                                              |
| ----------------------------------------- | --------------- | ---------------------------------------------------------------------------------------------------  |
| SR00RPA01:OUT<1,2>\_ENABLE\_CMD | Enable, Disable | Enable/disable output channel 1 or 2. |
| SR00RPA01:OUT\_RESET\_CMD | 1 | Reset data generation configuration. |

**Output trigger**

| Record Name                               | Allowed Values  | Comment                                                                                              |
| ----------------------------------------- | --------------- | ---------------------------------------------------------------------------------------------------  |
| SR00RPA01:OUT\_SS\_CHANNEL\_CMD | Channel 1, Channel 2, Both Channels | Which output channel should be triggered with a single shot trigger. |
| SR00RPA01:OUT\_SS\_TRIGGER\_CMD | 1 | Trigger single shot on selected output channels. |
| SR00RPA01:OUT<1,2>\_TRIGGER\_SRC\_CMD | Internal, EXT_PE, EXT_NE, Gated Burst | Select a trigger for output channel 1 or 2. |

**Signal generation**

| Record Name                               | Allowed Values  | Comment                                                                                              |
| ----------------------------------------- | --------------- | ---------------------------------------------------------------------------------------------------  |
| SR00RPA01:OUT<1,2>\_WAVEFORM\_TYPE\_CMD | Sine, Square, Triangle, Ramp Up, Ramp Down, DC, PWM, Arbitrary | Generated signal waveform type for output channel 1 or 2.
| SR00RPA01:OUT<1,2>\_AMP\_SP | 0 <= *amplitude* <= 1 | Generated signal amplitude in V.
| SR00RPA01:OUT<1,2>\_OFFSET\_SP | -1 <= *offset* <= 1 | Generated signal DC offset in V.
| SR00RPA01:OUT<1,2>\_FREQ\_SP | 0 <= *frequency* <= 62.5E6 | Signal frequency in Hz.
| SR00RPA01:OUT<1,2>\_PHASE\_SP | 0 <= *phase* <= 360 | Signal phase in degrees.
| SR00RPA01:OUT<1,2>\_DUTY\_CYCLE\_SP | 0 <= *duty cycle* <= 100 | If PWM waveform type is selected this is a duty cycle in percentage.
| SR00RPA01:OUT<1,2>\_GEN\_MODE\_CMD | Continuous, Burst | Waveform generation mode.
| SR00RPA01:OUT<1,2>\_BURST\_COUNT\_SP | -1 <= count <= 50000 | Number of generated waveforms in a burst. If -1 a continuous signal will be generated.
| SR00RPA01:OUT<1,2>\_BURST\_REPS\_SP | -1 <= repetitions <= 50000 | Number of generated bursts. If -1, infinite bursts will be generated.
| SR00RPA01:OUT<1,2>\_BURST\_PERIOD\_SP | 1 <= period <= 500000000 | Time/period of one burst in micro seconds. Period must be equal or greater then the time of one burst. If it is greater than the difference will be the delay between two consequential bursts.
| SR00RPA01:OUT<1,2>\_DATA\_SP | A waveform of [-1.1, 1.1] | Arbitrary waveform containing 16354 elements.


### Analog and Digital Pins

**Digital pins**

| Record Name                               | Allowed Values  | Comment                                                                                              |
| ----------------------------------------- | --------------- | ---------------------------------------------------------------------------------------------------  |
| SR00RPA01:DIGITAL\_<N,P><0...7>\_DIR\_CMD | Input, Output | Direction of the digital pin. |
| SR00RPA01:DIGITAL\_<N,P><0...7>\_STATE\_CMD | Low, High | State of the digital pin. |

**LEDs** 

| Record Name                               | Allowed Values  | Comment                                                                                              |
| ----------------------------------------- | --------------- | ---------------------------------------------------------------------------------------------------  |
| SR00RPA01:LED<0...7>\_STATE\_CMD | Off, On | State of the LED. |

**Analog pins**

| Record Name                               | Allowed Values  | Comment                                                                                              |
| ----------------------------------------- | --------------- | ---------------------------------------------------------------------------------------------------  |
| SR00RPA01:ANALOG\_OUT<0...3>\_VOLT\_SP  | 0 <= *output* <= 1.8 | Set the voltage on anaglog pin to *output* volts. |
| SR00RPA01:ANALOG\_IN<0...3>\_VOLT\_MONITOR  | 0 <= *input* <= 3.3  | Read the voltage on input analog pin. | 


