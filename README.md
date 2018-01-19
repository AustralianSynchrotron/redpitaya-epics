# redpitaya-epics

EPICS driver support for RedPitaya based on asynPortDriver. This module is to be run on the RP itself.

## Getting Started

It's really quite straight forward. Module consists of an EPICS support library and test application. All you need is:
* [RedPitaya](https://www.redpitaya.com)
* [EPICS Base](https://epics.anl.gov/download/base/index.php)
* [asynDriver](https://epics.anl.gov/modules/soft/asyn/) 

### RedPitaya
Code has been developed on and tested with [STEMLab 125-14](https://www.redpitaya.com/f130/STEMlab-board) (originally Red Pitaya v1.1) and RedPitaya library version 0.98-615-ace71ac. RedPitaya library comes with the image you get on their website. To set it up, follow their [Quick start](http://redpitaya.readthedocs.io/en/latest/quickStart/quickStart.html) manual.

### EPICS Base
There are nothing to funky in the code so I belive any EPICS Base > 3.14.12.2 should be fine but it has only been tested with 3.15.5 and 3.16.1.

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
After you have you EPICS Base and asynDriver ready (cross-compiled on built on the board) it's time to build this driver support. It's designed to be built on the board itself, so begin by copying it to RedPitaya. Once this is done, simply modify the $(TOP)/configure/RELEASE file of this module to point to where your EPICS Base and asynDriver are and hit make.

## Running the Test App
After a (hopefully) successful build, you'll end up with two files in your $(TOP)/bin/linux-arm directory. You have to load the FPGA image prior to running the IOC. To do that simply execute the $(TOP)/bin/linux-arm/load\_fpga\_image.sh script as root.
When this is done you can start the IOC in the standard way by running $(TOP)/iocBoot/iocRedPitayaTest/st.cmd

