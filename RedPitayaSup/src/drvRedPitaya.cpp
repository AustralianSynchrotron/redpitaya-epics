/* Description:
 * This is an ASYN port driver to be used with an on board IOC for
 * RedPitaya.
 *
 * Copyright (c) 2018  Australian Synchrotron
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Contact details:
 * andraz.pozar@synchrotron.org.au
 * 800 Blackburn Road, Clayton, Victoria 3168, Australia.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <algorithm>
#include <iterator>

#include <cantProceed.h>
#include <epicsExit.h>
#include <epicsExport.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <errlog.h>
#include <iocsh.h>
#include <math.h>
#include <unistd.h>

#include <asynParamType.h>

#include "rp.h"
#include "drvRedPitaya.h"

// Useful type neutral numerical macro fuctions.
//
#define MIN(a, b)           ((a) <= (b) ? (a) : (b))

// Calculates number of items in an array
//
#define ARRAY_LENGTH(xx)   ((int) (sizeof (xx) /sizeof (xx [0])))

#define REDPITAYA_DRIVER_VERSION   "2.1"

#define MAX_NUMBER_OF_ADDRESSES    8      // 0 to 8

#define FULL_SCALE_NORM            20.0

// Pin and led minimum and maximum values
//
#define MIN_CHANNEL_NUM    0
#define MAX_CHANNEL_NUM    1
#define MIN_DIG_PIN       0
#define MAX_DIG_PIN       7
#define MIN_ANALOG_PIN    0
#define MAX_ANALOG_PIN    3

#define N_PIN_OFFSET   16
#define P_PIN_OFFSET   8

static const uint32_t BUFFER_SIZE = 16354;

// Array used to read the buffer
//
float* ch1_buffer = (float *) malloc(BUFFER_SIZE * sizeof(float));
float* ch2_buffer = (float *) malloc(BUFFER_SIZE * sizeof(float));

float* ch1_output_monitor = (float *) malloc(BUFFER_SIZE * sizeof(float));
float* ch2_output_monitor = (float *) malloc(BUFFER_SIZE * sizeof(float));

// Array used to do callbacks
//
epicsFloat32* ch1_data = (epicsFloat32 *) malloc(BUFFER_SIZE * sizeof(epicsFloat32));
epicsFloat32* ch2_data = (epicsFloat32 *) malloc(BUFFER_SIZE * sizeof(epicsFloat32));


// Initialisation of decimation factors
//
const static int decimationFactors [6] = {1, 8, 64, 1024, 8192, 65536};

rp_acq_trig_src_t inTriggerSource;

struct QualifierDefinitions {
   asynParamType type;
   const char* name;
};

// Qualifier lookup table - used by RedPitaya_DrvUser_Create and
// RedPitaya_Qualifier_Image
//
// MUST be consistent with enum Qualifiers type out of RedPitayaDriver (in drvRedPitaya.h)
//
static const QualifierDefinitions qualifierList[] = {
      // General info
      //
      { asynParamOctet,        "DRVVER"      },  // DriverVersion

      // Digital pins
      //
      { asynParamInt32,        "NDPDIR"      },  // Set Digital Pin N to be an Input or Output
      { asynParamInt32,        "PDPDIR"      },  // Set Digital Pin P to be an Input or Output
      { asynParamInt32,        "NDPSTATE"    },  // Digital Pin N State
      { asynParamInt32,        "PDPSTATE"    },  // Digital Pin P State

      // LEDs and analogue pins
      //
      { asynParamInt32,        "LEDSTATE"    },  // LED State
      { asynParamFloat64,      "APOUTSTATE"  },  // Analog output pin voltage
      { asynParamFloat64,      "APINSTATE"   },  // Analog input pin voltage

      // Data acquisition on two fast input channels
      //
      { asynParamInt32,        "SACQSTART"   },  // Start acquisition of data after a single trigger
      { asynParamInt32,        "CACQSTART"   },  // Start continuous data acquisition
      { asynParamInt32,        "ACQSTOP"     },  // Stop data acquisition
      { asynParamInt32,        "ACQRST"      },  // Stop acquisition and set all parameters to default
      { asynParamInt32,        "ACQSTAT"     },  // Acquisition status
      { asynParamInt32,        "ACQDEC"      },  // Decimation factor
      { asynParamInt32,        "ACQSRAT"     },  // Sampling rate
      { asynParamInt32,        "ACQAVG"      },  // Averaging
      { asynParamInt32,        "ACQTRIGSRC"  },  // Acquisition trigger source
      { asynParamInt32,        "ACQTRIGSTATE"},  // Acquisition trigger state
      { asynParamInt32,        "ACQTRIGDLY"  },  // Acquisition trigger delay
      { asynParamFloat64,      "ACQTRIGHYST" },  // Acquisition trigger hysteresis
      { asynParamInt32,        "ACQGAIN"     },  // Acquisition source gain
      { asynParamFloat64,      "ACQTRIGLVL"  },  // Acquisition trigger level
      { asynParamInt32,        "ACQBUFF"     },  // Acquisition buffer size
      { asynParamFloat64,      "ACQRATE"     },  // Continuous acquisition rate
      { asynParamFloat32Array, "DATA"        },  // Data Read From the Channel

      // Waveform generation on two fast output channels
      //
      { asynParamInt32,        "GENRST"      },  // Stop generation and set all parameters to default
      { asynParamInt32,        "OUTENABLED"  },  // Output enabled
      { asynParamFloat64,      "SIGAMP"      },  // Output signal amplitude
      { asynParamFloat64,      "SIGOFFSET"   },  // Output signal DC offset
      { asynParamFloat64,      "SIGFREQ"     },  // Output signal frequency
      { asynParamFloat64,      "SIGPHASE"    },  // Output signal phase
      { asynParamInt32,        "WFTYPE"      },  // Signal signal waveform type
      { asynParamFloat64,      "DUTYCYCLE"   },  // PWM duty cycle ratio
      { asynParamInt32,        "GENMODE"     },  // Signal generation mode (CONTINUOUS, BURST, STREAM)
      { asynParamInt32,        "BURSTCNT"    },  // Number of generated waveforms in a burst
      { asynParamInt32,        "BURSTRPT"    },  // Number of burst repetitions
      { asynParamInt32,        "BURSTPRD"    },  // Burst period in uS
      { asynParamInt32,        "OUTTRGSRC"   },  // Output trigger source
      { asynParamInt32,        "OUTSSCHAN"   },  // Single shot trigger channel selection
      { asynParamInt32,        "OUTSSTRIG"   },  // Single shot trigger
      { asynParamFloat32Array, "RFDATA"      }   // Arbitrary output signal
   };

// Supported interrupts.
//
const static int interruptMask = asynFloat32ArrayMask;

// Any interrupt must also have an interface.
//
const static int interfaceMask = interruptMask | asynDrvUserMask | asynOctetMask
      | asynInt32Mask | asynFloat64Mask | asynFloat32ArrayMask;

const static int asynFlags = ASYN_MULTIDEVICE | ASYN_CANBLOCK;

// Static data.
//
static int verbosity = 4;               // High until set low
static int driver_initialised = false;
static bool singleShot;

//------------------------------------------------------------------------------
// Local functions
//------------------------------------------------------------------------------
//
static void devprintf(const int required_min_verbosity, const char* function,
      const int line_no, const char* format, ...) {
   if (verbosity >= required_min_verbosity) {
      char message[100];
      va_list arguments;
      va_start(arguments, format);
      vsnprintf(message, sizeof(message), format, arguments);
      va_end(arguments);
      errlogPrintf("RedPitayaDriver: %s:%d  %s", function, line_no, message);
   }
}

// Wrapper macros to devprintf.
//
#define ERROR(...)    devprintf (0, __FUNCTION__, __LINE__, __VA_ARGS__);
#define WARNING(...)  devprintf (1, __FUNCTION__, __LINE__, __VA_ARGS__);
#define INFO(...)     devprintf (2, __FUNCTION__, __LINE__, __VA_ARGS__);
#define DETAIL(...)   devprintf (3, __FUNCTION__, __LINE__, __VA_ARGS__);

//==============================================================================
//
//==============================================================================
//
const char* RedPitayaDriver::qualifierImage(const Qualifiers q) {
   static char result[24];

   if ((q >= 0) && (q < NUMBER_QUALIFIERS)) {
      return qualifierList[q].name;
   } else {
      sprintf(result, "unknown (%d)", q);
      return result;
   }
}

//------------------------------------------------------------------------------
// static
//
static void RedPitaya_Initialise(const int verbosityIn) {
   verbosity = verbosityIn;
   driver_initialised = true;
}

//------------------------------------------------------------------------------
//
void RedPitayaDriver::shutdown(void* arg) {
   RedPitayaDriver* self = (RedPitayaDriver*) arg;

   if (self && self->is_initialised) {
      INFO("RedPitayaDriver: shutting down: %s\n", self->full_name)
      // If we don't reset the FPGA continues on doing whatever
      // it has been doing
      //
      rp_Reset();
      rp_Release();
   }
}

//
void dataAcquisition(void *drvPvt) {
   RedPitayaDriver *self = (RedPitayaDriver *) drvPvt;

   self->dataAcquisition();
}

//------------------------------------------------------------------------------
//
RedPitayaDriver::RedPitayaDriver(const char* port_name) :
      asynPortDriver(port_name,              //
                     MAX_NUMBER_OF_ADDRESSES,//
                     NUMBER_QUALIFIERS,      //
                     interfaceMask,          //
                     interruptMask,          //
                     asynFlags,              //
                     1,                      // Auto-connect
                     0,                      // Default priority
                     0)                      // Default stack size

{

   this->is_initialised = false;   // flag object as not initialised.

   if (!driver_initialised) {
      ERROR("driver not initialised (call %s first)\n", "RedPitaya_Initialise");
      return;
   }

   // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

   // Verify port names are sensible
   //
   if ((port_name == NULL) || (strcmp(port_name, "") == 0)) {
      ERROR("null/empty port name\n", 0);
      return;
   }

   snprintf(this->full_name, sizeof(this->full_name), "%s", port_name);

   // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
   // Set up asyn parameters.
   //
   for (int j = 0; j < ARRAY_LENGTH(qualifierList); j++) {
      asynStatus status = createParam(qualifierList[j].name,
            qualifierList[j].type, &this->indexList[j]);
      if (status != asynSuccess) {
         ERROR("Parameter creation failed\n");
         return;
      }
   }

   int initStatus = rp_Init();
   if (initStatus != RP_OK) {
      ERROR("Driver initialization failed with error: %s\n",
            rp_GetError(initStatus));
      return;
   }

   INFO("RedPitaya library version: %s\n", rp_GetVersion());

   // Setting default acquisition sleep time
   //
   this->acquisitionSleep = 1.0;

   // By default allow single shot triggers on both channels
   //
   this->ssChannelSelected = 2;

   // Initialise output trigger sources to INTERNAL
   //
   this->out1TriggerSource = RP_GEN_TRIG_SRC_INTERNAL;
   this->out2TriggerSource = RP_GEN_TRIG_SRC_INTERNAL;

   // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
   // Register for epics exit callback.
   // Note: shutdown is a static function so we must pass this as a parameter.
   //
   epicsAtExit(RedPitayaDriver::shutdown, this);

   this->is_initialised = true;

   INFO("%s initialisation complete\n", this->full_name);
}

//------------------------------------------------------------------------------
//
RedPitayaDriver::~RedPitayaDriver() {
   // Clean up a bit
   //
   free (ch1_buffer);
   free (ch2_buffer);
   free (ch1_data);
   free (ch2_data);
   free (ch1_output_monitor);
   free (ch2_output_monitor);
}

//------------------------------------------------------------------------------
// Asyn callback functions
//------------------------------------------------------------------------------
//
void RedPitayaDriver::report(FILE * fp, int details) {
   if (details > 0) {
      fprintf(fp, "    driver info:\n");
      fprintf(fp, "        initialised:  %s\n",
            this->is_initialised ? "yes" : "no");
      fprintf(fp, "\n");
   }
}

//-----------------------------------------------------------------------------
// A function that runs in a separate thread and reads the data.
//-----------------------------------------------------------------------------
void RedPitayaDriver::dataAcquisition() {

   rp_acq_decimation_t decimation;
   double bufferTimeout;

   // Loop until the user requests the acquisition to stop
   //
   while (this->acquiring) {
      // Calculate the time it takes for the buffer to be filled with new data
      // and only then set a trigger.
      // At no decimation the buffer holds 0.000131 seconds of samples
      //
      rp_AcqGetDecimation(&decimation);
      bufferTimeout = decimationFactors[decimation] * 0.000131;

      int rpStatus = rp_AcqStart();
      if (rpStatus != RP_OK) {
         return;
      }

      // Give buffer time to flush old data
      //
      epicsThreadSleep(bufferTimeout);

      // Set the trigger that has been set from the EPICS database
      //
      rp_AcqSetTriggerSrc(inTriggerSource);

      // Get currently set trigger from the device. When the trigger goes to DISABLED we know that
      // recording of data has finished. In case the trigger changes while waiting, we exit the loop.
      //
      rp_acq_trig_src_t currentTriggerSource;
      rp_AcqGetTriggerSrc(&currentTriggerSource);
      while (currentTriggerSource != 0 && this->acquiring &&
            inTriggerSource == currentTriggerSource) {
         rp_AcqGetTriggerSrc(&currentTriggerSource);
      }

      // Break the loop if the acquisition has been cancelled while waiting
      // for the trigger
      //
      if (!this->acquiring) {
         break;
      }

      // Get pointer to the trigger in the buffer
      //
      uint32_t triggerPos;
      rp_AcqGetWritePointerAtTrig(&triggerPos);

      // Read the whole buffer for both channels
      //
      uint32_t buff_size = BUFFER_SIZE;
      rpStatus += rp_AcqGetOldestDataV(RP_CH_1, &buff_size, ch1_buffer);
      rpStatus += rp_AcqGetOldestDataV(RP_CH_2, &buff_size, ch2_buffer);

      if (rpStatus == RP_OK) {
         // Copy retrieved data to return array.
         //
         // We copy sample by sample beginning from the trigger position in the buffer, using modulo of
         // buffer length.
         //
         for (size_t i = 0; i < buff_size; i++) {
            ch1_data[(triggerPos+i)%buff_size] = (epicsFloat64) ch1_buffer[(triggerPos+i)%buff_size];
            ch2_data[(triggerPos+i)%buff_size] = (epicsFloat64) ch2_buffer[(triggerPos+i)%buff_size];
         }

         // Do the callbacks to EPICS database waveform records used to present the acquired data
         //
         doCallbacksFloat32Array(ch1_data, buff_size, SourceData, RP_CH_1);
         doCallbacksFloat32Array(ch2_data, buff_size, SourceData, RP_CH_2);
      }

      // If this is a single shot acquisition exit the loop otherwise sleep for a number of seconds
      // and repeat
      //
      if (singleShot) {
         // Set a flag to false so that a new acquisition thread is allowed to be started
         //
         this->acquiring = false;
         break;
      }

      epicsThreadSleep(acquisitionSleep);
   }
}

//------------------------------------------------------------------------------
//
asynStatus RedPitayaDriver::readOctet(asynUser* pasynUser, char* data,
      size_t maxchars, size_t* nbytesTransfered, int* eomReason) {
   const Qualifiers qualifier = static_cast<Qualifiers> (pasynUser->reason);
   asynStatus status;
   int addr;
   size_t length;

   *nbytesTransfered = 0;

   // Extract and validate address index.
   //
   pasynManager->getAddr(pasynUser, &addr);

   status = asynSuccess;        // hypothesize okay

   switch (qualifier) {

   case DriverVersion:
      strncpy(data, REDPITAYA_DRIVER_VERSION, maxchars);
      length = strlen(REDPITAYA_DRIVER_VERSION);
      *nbytesTransfered = MIN(length, maxchars);
      break;

   default:
      ERROR("%s Unexpected qualifier (%s)\n", this->full_name,
            qualifierImage(qualifier))
      ;
      status = asynError;
      break;
   }

   return status;
}

asynStatus RedPitayaDriver::checkAddrLimits(const Qualifiers qualifier,
      const int addr) {

   int minAddress, maxAddress;

   switch (qualifier) {
   case NDigPinDir:
   case NDigPinState:
   case PDigPinDir:
   case PDigPinState:
   case LedState:
      minAddress = MIN_DIG_PIN;
      maxAddress = MAX_DIG_PIN;
      break;
   case AnalogPinOut:
   case AnalogPinIn:
      minAddress = MIN_ANALOG_PIN;
      maxAddress = MAX_ANALOG_PIN;
      break;
   default:
      minAddress = MIN_CHANNEL_NUM;
      maxAddress = MAX_CHANNEL_NUM;
      break;
   }

   if (addr < minAddress || addr > maxAddress) {
      ERROR("%s address (%d) out of range\n", qualifierImage(qualifier), addr);
      return asynError;
   }

   return asynSuccess;
}

//------------------------------------------------------------------------------
//
asynStatus RedPitayaDriver::writeInt32(asynUser* pasynUser, epicsInt32 value) {
   const Qualifiers qualifier = static_cast<Qualifiers> (pasynUser->reason);
   asynStatus status;
   int addr;

   if (this->is_initialised != true) {
      return asynError;
   }

   // Extract and validate address index.
   //
   pasynManager->getAddr(pasynUser, &addr);

   status = checkAddrLimits(qualifier, addr);

   if (status != asynSuccess) {
      return status;
   }

   status = asynSuccess;       // hypothesise okay
   int rpStatus = 0;           // hypothesise okay

   switch (qualifier) {

   case PDigPinDir:
      rpStatus = rp_DpinSetDirection(rp_dpin_t(P_PIN_OFFSET + addr), rp_pinDirection_t(value));
      break;
   case NDigPinDir:
      rpStatus = rp_DpinSetDirection(rp_dpin_t(N_PIN_OFFSET + addr), rp_pinDirection_t(value));
      break;
   case PDigPinState:
      rpStatus = rp_DpinSetState(rp_dpin_t(N_PIN_OFFSET + addr), rp_pinState_t(value));
      break;
   case NDigPinState:
      rpStatus = rp_DpinSetState(rp_dpin_t(P_PIN_OFFSET + addr), rp_pinState_t(value));
      break;
   case LedState:
      rpStatus = rp_DpinSetState(rp_dpin_t(addr), rp_pinState_t(value));
      break;
   case ContAcquisitionStart:
   case SingleAcquisitionStart:
      if (value == 1 && !this->acquiring) {
         this->acquiring = true;
         singleShot = qualifier == SingleAcquisitionStart;
         status = (asynStatus)(
               epicsThreadCreate("rpReadData", epicsThreadPriorityMedium,
                     epicsThreadGetStackSize(epicsThreadStackMedium),
                     (EPICSTHREADFUNC) ::dataAcquisition, this) == NULL);
      }
      break;
   case AcquisitionStop:
      rpStatus = rp_AcqStop();
      this->acquiring = false;
      break;
   case AcquisitionReset:
      if (value == 1) {
         rpStatus = rp_AcqReset();
      }
      break;
   case Decimation:
      if (value >= 0 && value <= 5) {
         rpStatus = rp_AcqSetDecimation(rp_acq_decimation_t(value));
      } else {
         ERROR("%s Unexpected value for qualifier (%s)\n", this->full_name, qualifierImage(qualifier));
         status = asynError;
      }
      break;
   case SamplingRate:
      if (value >= 0 && value <= 5) {
         rpStatus = rp_AcqSetSamplingRate(rp_acq_sampling_rate_t(value));
      } else {
         ERROR("%s Unexpected value for qualifier (%s)\n", this->full_name, qualifierImage(qualifier));
         status = asynError;
      }
      break;
   case Averaging:
      if (value == 0 || value == 1) {
         rpStatus = rp_AcqSetAveraging(value == 1);
      } else {
         ERROR("%s Unexpected value for qualifier (%s)\n", this->full_name, qualifierImage(qualifier));
         status = asynError;
      }
      break;
      break;
   case TriggerSrc:
      if (value <= 9 && value >= 0) {
         // We save the value and then set the trigger source
         // when it's safe to do so
         //
         inTriggerSource = rp_acq_trig_src_t(value);
      } else {
         ERROR("%s Unexpected value for qualifier (%s)\n", this->full_name, qualifierImage(qualifier));
         status = asynError;
      }
      break;
   case TriggerDelay:
      rpStatus = rp_AcqSetTriggerDelayNs(int64_t(value));
      break;
   case AcquisitionGain:
      if (value == 0 || value == 1) {
         rpStatus = rp_AcqSetGain(rp_channel_t(addr), rp_pinState_t(value));
      } else {
         ERROR("%s Unexpected value for qualifier (%s)\n", this->full_name, qualifierImage(qualifier));
         status = asynError;
      }
      break;
   case GenerationReset:
      if (value == 1) {
         rpStatus = rp_GenReset();
      }
      break;
   case OutputEnabled:
      if (value == 1) {
         rpStatus = rp_GenOutEnable(rp_channel_t(addr));
      } else if (value == 0) {
         rpStatus = rp_GenOutDisable(rp_channel_t(addr));
      }
      break;
   case SignalWaveformType:
      if (value <= 7 && value >= 0) {
         rpStatus = rp_GenWaveform(rp_channel_t(addr), rp_waveform_t(value));
      } else {
         ERROR("%s Unexpected value for qualifier (%s)\n", this->full_name, qualifierImage(qualifier));
         status = asynError;
      }
      break;
   case SignalGenerationMode:
      if (value == 0 || value == 1) {
         rpStatus = rp_GenMode(rp_channel_t(addr), rp_gen_mode_t(value));
      } else {
         ERROR("%s Unexpected value for qualifier (%s)\n", this->full_name, qualifierImage(qualifier));
         status = asynError;
      }
      break;
   case SignalBurstCount:
      rpStatus = rp_GenBurstCount(rp_channel_t(addr), value);
      break;
   case SignalBurstRepetitions:
      rpStatus = rp_GenBurstRepetitions(rp_channel_t(addr), value);
      break;
   case SignalBurstPeriod:
      rpStatus = rp_GenBurstPeriod(rp_channel_t(addr), value);
      break;
   case OutputTriggerSrc:
      if (value <= 3 && value >= 0) {
         // Convert to 1 based indexing
         //
         rp_trig_src_t trigger = rp_trig_src_t(value + 1);
         rpStatus = rp_GenTriggerSource(rp_channel_t(addr), trigger);
         if (rpStatus == 0) {
            // 03/05/2018:
            // There's a bug in RedPitaya API where API doesn't return a valid
            // value for output trigger source that is why we're saving it in
            // the driver rather than reading it from the RP
            //
            rp_channel_t(addr) == RP_CH_1 ? this->out1TriggerSource = trigger : this->out2TriggerSource = trigger;
         }
      } else {
         ERROR("%s Unexpected value for qualifier (%s)\n", this->full_name, qualifierImage(qualifier));
         status = asynError;
      }
      break;
   case OutputSSChannel:
      if (value <= 2 && value >= 0) {
         // Convert to 1 based indexing
         //
         this->ssChannelSelected = value;
      } else {
         ERROR("%s Unexpected value for qualifier (%s)\n", this->full_name, qualifierImage(qualifier));
         status = asynError;
      }
      break;
   case OutputSSTrigger:
      if (value == 1) {
         rpStatus = rp_GenTrigger(this->ssChannelSelected);
      }
      break;
   default:
      ERROR("%s Unexpected qualifier (%s)\n", this->full_name, qualifierImage(qualifier))
      ;
      status = asynError;
      break;
   }

   if (rpStatus != 0) {
      ERROR("%s Command for qualifier (%s) failed with message: %s\n", this->full_name, qualifierImage(qualifier), rp_GetError(rpStatus));
      status = asynError;
   }

   return status;
}

//------------------------------------------------------------------------------
//
asynStatus RedPitayaDriver::readInt32(asynUser* pasynUser, epicsInt32* value) {
   const Qualifiers qualifier = static_cast<Qualifiers> (pasynUser->reason);
   asynStatus status;
   int addr;

   if (this->is_initialised != true) {
      return asynError;
   }

   // Extract and validate address index.
   //
   pasynManager->getAddr(pasynUser, &addr);

   status = checkAddrLimits(qualifier, addr);

   if (status != asynSuccess) {
      return status;
   }

   status = asynSuccess;       // hypothesise okay
   int rpStatus = 0;           // hypothesise okay

   switch (qualifier) {
   case PDigPinDir: {
      rp_pinDirection_t direction;
      rpStatus = rp_DpinGetDirection(static_cast<rp_dpin_t> (P_PIN_OFFSET + addr), &direction);
      if (rpStatus == RP_OK) {
         *value = direction;
      }
   }
      break;
   case NDigPinDir: {
      rp_pinDirection_t direction;
      rpStatus = rp_DpinGetDirection(static_cast<rp_dpin_t> (N_PIN_OFFSET + addr), &direction);
      if (rpStatus == RP_OK) {
         *value = direction;
      }
   }
      break;
   case PDigPinState: {
      rp_pinState_t state;
      rpStatus = rp_DpinGetState(static_cast<rp_dpin_t> (P_PIN_OFFSET + addr), &state);
      if (rpStatus == RP_OK) {
         *value = state;
      }
   }
      break;
   case NDigPinState: {
      rp_pinState_t state;
      rpStatus = rp_DpinGetState(static_cast<rp_dpin_t> (P_PIN_OFFSET + addr), &state);
      if (rpStatus == RP_OK) {
         *value = state;
      }
   }
      break;
   case LedState: {
      rp_pinState_t state;
      rpStatus = rp_DpinGetState(static_cast<rp_dpin_t> (addr), &state);
      if (rpStatus == RP_OK) {
         *value = state;
      }
   }
      break;
   case ContAcquisitionStart:
   case SingleAcquisitionStart:
   case AcquisitionStop:
   case AcquisitionReset:
   case GenerationReset:
   case OutputSSTrigger:
      // Nothing to read really
      //
      break;
   case AcquisitionStatus:
      *value = this->acquiring ? 1 : 0;
      break;
   case AcquisitionGain: {
      rp_pinState_t state;
      rpStatus = rp_AcqGetGain(rp_channel_t(addr), &state);
      if (rpStatus == RP_OK) {
         *value = state;
      }
   }
      break;
   case Decimation: {
      rp_acq_decimation_t decimation;
      rpStatus = rp_AcqGetDecimation(&decimation);
      if (rpStatus == RP_OK) {
         *value = decimation;
      }
   }
      break;
   case SamplingRate: {
      rp_acq_sampling_rate_t rate;
      rpStatus = rp_AcqGetSamplingRate(&rate);
      if (rpStatus == RP_OK) {
         *value = rate;
      }
   }
      break;
   case Averaging: {
      bool averaging;
      rpStatus = rp_AcqGetAveraging(&averaging);
      if (rpStatus == RP_OK) {
         *value = averaging;
      }
   }
      break;
   case TriggerSrc: {
      rp_acq_trig_src_t triggerSource;
      rpStatus = rp_AcqGetTriggerSrc(&triggerSource);
      if (rpStatus == RP_OK) {
         *value = triggerSource;
      }
   }
      break;
   case TriggerState: {
      rp_acq_trig_state_t triggerState;
      rpStatus = rp_AcqGetTriggerState(&triggerState);
      if (rpStatus == RP_OK) {
         *value = triggerState;
      }
   }
      break;
   case TriggerDelay: {
      int64_t triggerDelay;
      rpStatus = rp_AcqGetTriggerDelayNs(&triggerDelay);
      if (rpStatus == RP_OK) {
         *value = static_cast<epicsInt32> (triggerDelay);
      }
   }
      break;
   case BufferSize: {
      uint32_t size;
      rpStatus = rp_AcqGetBufSize(&size);
      if (rpStatus == RP_OK) {
         *value = static_cast<epicsInt32> (size);
      }
   }
      break;
   case OutputEnabled: {
      bool isEnabled = false;
      rpStatus = rp_GenOutIsEnabled(rp_channel_t(addr), &isEnabled);
      if (rpStatus == RP_OK) {
         *value = isEnabled;
      }
   }
      break;
   case SignalWaveformType: {
      rp_waveform_t signalWaveformType;
      rpStatus = rp_GenGetWaveform(rp_channel_t(addr), &signalWaveformType);
      if (rpStatus == RP_OK) {
         *value = signalWaveformType;
      }
   }
      break;
   case SignalGenerationMode: {
      rp_gen_mode_t signalGenerationMode;
      rpStatus = rp_GenGetMode(rp_channel_t(addr), &signalGenerationMode);
      if (rpStatus == RP_OK) {
         *value = signalGenerationMode;
      }
   }
      break;
   case SignalBurstCount: {
      int signalBurstCount;
      rpStatus = rp_GenGetBurstCount(rp_channel_t(addr), &signalBurstCount);
      if (rpStatus == RP_OK) {
         *value = signalBurstCount;
      }
   }
      break;
   case SignalBurstRepetitions: {
      int signalBurstRepetitions;
      rpStatus = rp_GenGetBurstRepetitions(rp_channel_t(addr), &signalBurstRepetitions);
      if (rpStatus == RP_OK) {
         *value = signalBurstRepetitions;
      }
   }
      break;
   case SignalBurstPeriod: {
      uint32_t signalBurstPeriod;
      rpStatus = rp_GenGetBurstPeriod(rp_channel_t(addr), &signalBurstPeriod);
      if (rpStatus == RP_OK) {
         *value = static_cast<epicsInt32> (signalBurstPeriod);
      }
   }
      break;
   case OutputTriggerSrc: {
      // 03/05/2018:
      // There's a bug in RedPitaya API where API doesn't return a valid
      // value for output trigger source that is why we're saving it in
      // the driver rather than reading it from the RP
      //
      int outputTriggerSrc = rp_channel_t(addr) == RP_CH_1 ? this->out1TriggerSource : this->out2TriggerSource;
      if (rpStatus == RP_OK) {
         // Convert to 0 based indexing
         //
         *value = outputTriggerSrc - 1;
      }
   }
      break;
   case OutputSSChannel:
      *value = this->ssChannelSelected;
      break;
   default:
      ERROR("%s Unexpected qualifier (%s)\n", this->full_name, qualifierImage(qualifier))
      ;
      status = asynError;
      break;
   }

   if (rpStatus != RP_OK) {
      ERROR("%s Request for qualifier (%s) failed with message: %s\n", this->full_name, qualifierImage(qualifier), rp_GetError(rpStatus));
      status = asynError;
   }

   return status;

}

//------------------------------------------------------------------------------
//
asynStatus RedPitayaDriver::writeFloat64(asynUser* pasynUser,
      epicsFloat64 value) {
   const Qualifiers qualifier = static_cast<Qualifiers> (pasynUser->reason);
   asynStatus status;
   int addr;

   if (this->is_initialised != true) {
      return asynError;
   }

   // Extract and validate address index.
   //
   pasynManager->getAddr(pasynUser, &addr);

   status = checkAddrLimits(qualifier, addr);

   if (status != asynSuccess) {
      return status;
   }

   status = asynSuccess;       // hypothesise okay
   int rpStatus = 0;           // hypothesise okay

   switch (qualifier) {
   case AnalogPinOut:
      rpStatus = rp_ApinSetValue(static_cast<rp_apin_t> (addr), value);
      break;
   case TriggerLevel:
      rpStatus = rp_AcqSetTriggerLevel(RP_CH_1, value);
      rpStatus += rp_AcqSetTriggerLevel(RP_CH_2, value);
      break;
   case AcquisitionRate:
      acquisitionSleep = value;
      break;
   case TriggerHysteresis:
      rpStatus = rp_AcqSetTriggerHyst(value);
      break;
   case SignalAmplitude:
      rpStatus = rp_GenAmp(rp_channel_t(addr), value);
      break;
   case SignalOffset:
      rpStatus = rp_GenOffset(rp_channel_t(addr), value);
      break;
   case SignalFrequency:
      rpStatus = rp_GenFreq(rp_channel_t(addr), value);
      break;
   case SignalPhase:
      rpStatus = rp_GenPhase(rp_channel_t(addr), value);
      break;
   case PWMDutyCycle:
      // Converting from percentage (user friendly) to parts (API friendly)
      //
      rpStatus = rp_GenDutyCycle(rp_channel_t(addr), value / 100.0);
      break;
   default:
      ERROR("%s Unexpected qualifier (%s)\n", this->full_name, qualifierImage(qualifier))
      ;
      status = asynError;
      break;
   }

   if (rpStatus != RP_OK) {
      ERROR("%s Command for qualifier (%s) failed with message: %s\n", this->full_name, qualifierImage(qualifier), rp_GetError(rpStatus));
      status = asynError;
   }

   return status;
}

//------------------------------------------------------------------------------
//
asynStatus RedPitayaDriver::readFloat64(asynUser* pasynUser,
      epicsFloat64* value) {
   const Qualifiers qualifier = static_cast<Qualifiers> (pasynUser->reason);

   asynStatus status;
   int addr;

   if (this->is_initialised != true) {
      return asynError;
   }

   // Extract and validate address index.
   //
   pasynManager->getAddr(pasynUser, &addr);

   status = checkAddrLimits(qualifier, addr);

   if (status != asynSuccess) {
      return status;
   }

   status = asynSuccess;       // hypothesise okay
   int rpStatus = 0;           // hypothesise okay

   switch (qualifier) {
   case AnalogPinOut: {
      float voltage;
      rpStatus = rp_ApinGetValue(static_cast<rp_apin_t> (addr), &voltage);
      if (rpStatus == RP_OK) {
         *value =  voltage;
      }
   }
      break;
   case AnalogPinIn: {
      float voltage;
      rpStatus = rp_AIpinGetValue(static_cast<rp_apin_t> (addr), &voltage);
      if (rpStatus == RP_OK) {
         *value = voltage;
      }
   }
      break;
   case TriggerLevel: {
      float level;
      rpStatus = rp_AcqGetTriggerLevel(&level);
      if (rpStatus == RP_OK) {
         *value = level;
      }
   }
      break;
   case AcquisitionRate:
      break;
   case TriggerHysteresis: {
      float hysteresis;
      rpStatus = rp_AcqGetTriggerHyst(&hysteresis);
      if (rpStatus == RP_OK) {
         *value = hysteresis;
      }
   }
      break;
   case SignalAmplitude: {
      float signalAmplitude;
      rpStatus = rp_GenGetAmp(rp_channel_t(addr), &signalAmplitude);
      // 03/05/2018:
      // There is a bug in RedPitaya API where the amplitude value returned from the FPGA
      // is divided by the full scale norm which is 20. In order to nullify this operation,
      // we multiply the returned value by 20.
      //
      signalAmplitude *= FULL_SCALE_NORM;

      if (rpStatus == RP_OK) {
         *value = signalAmplitude;
      }
   }
      break;
   case SignalOffset: {
      float signalOffset;
      rpStatus = rp_GenGetOffset(rp_channel_t(addr), &signalOffset);
      // 03/05/2018:
      // There is a bug in RedPitaya API where the offset value returned from the FPGA
      // is divided by the full scale norm which is 20. In order to nullify this operation,
      // we multiply the returned value by 20.
      //
      signalOffset *= FULL_SCALE_NORM;
      if (rpStatus == RP_OK) {
         *value = signalOffset;
      }
   }
      break;
   case SignalFrequency: {
      float signalFrequency;
      rpStatus = rp_GenGetOffset(rp_channel_t(addr), &signalFrequency);
      if (rpStatus == RP_OK) {
         *value = signalFrequency;
      }
   }
      break;
   case SignalPhase: {
      float signalPhase;
      rpStatus = rp_GenGetPhase(rp_channel_t(addr), &signalPhase);
      if (rpStatus == RP_OK) {
         *value = signalPhase;
      }
   }
      break;
   case PWMDutyCycle: {
      float dutyCycle;
      rpStatus = rp_GenGetDutyCycle(rp_channel_t(addr), &dutyCycle);
      if (rpStatus == RP_OK) {
         // Converting from parts (API friendly) to percentage (user friendly)
         //
         *value = dutyCycle * 100.0;
      }
   }
      break;
   default:
      ERROR("%s Unexpected qualifier (%s)\n", this->full_name, qualifierImage(qualifier))
      ;
      status = asynError;
      break;
   }

   if (rpStatus != RP_OK) {
      ERROR("%s Request for qualifier (%s) failed with message: %s\n", this->full_name, qualifierImage(qualifier), rp_GetError(rpStatus));
      status = asynError;
   }

   return status;
}

//------------------------------------------------------------------------------
//
asynStatus RedPitayaDriver::writeFloat32Array(asynUser *pasynUser, epicsFloat32 *value, size_t nElements) {
   const Qualifiers qualifier = static_cast<Qualifiers> (pasynUser->reason);

   asynStatus status;
   int addr;

   if (this->is_initialised != true) {
      return asynError;
   }

   // Extract and validate address index.
   //
   pasynManager->getAddr(pasynUser, &addr);

   status = checkAddrLimits(qualifier, addr);

   if (status != asynSuccess) {
      return status;
   }

   status = asynSuccess;       // hypothesise okay
   int rpStatus = 0;           // hypothesise okay

   switch (qualifier) {
   case SignalWaveform:
      rp_GenArbWaveform(rp_channel_t(addr), value, nElements);
      break;
   default:
      ERROR("%s Unexpected qualifier (%s)\n", this->full_name, qualifierImage(qualifier))
      ;
      status = asynError;
      break;
   }

   if (rpStatus != RP_OK) {
      ERROR("%s Command for qualifier (%s) failed with message: %s\n", this->full_name, qualifierImage(qualifier), rp_GetError(rpStatus));
      status = asynError;
   }

   return status;
}

//------------------------------------------------------------------------------
//
asynStatus RedPitayaDriver::readFloat32Array(asynUser *pasynUser, epicsFloat32 *value, size_t nElements, size_t *nIn) {
   const Qualifiers qualifier = static_cast<Qualifiers> (pasynUser->reason);

   asynStatus status;
   int addr;

   if (this->is_initialised != true) {
      return asynError;
   }

   // Extract and validate address index.
   //
   pasynManager->getAddr(pasynUser, &addr);

   status = checkAddrLimits(qualifier, addr);

   if (status != asynSuccess) {
      return status;
   }

   status = asynSuccess;       // hypothesise okay
   int rpStatus = 0;           // hypothesise okay

   switch (qualifier) {
   case SignalWaveform: {
      uint32_t numOfElements;
      float* waveform = (rp_channel_t(addr) == RP_CH_1 ? ch1_output_monitor : ch2_output_monitor);
      rpStatus = rp_GenGetArbWaveform(rp_channel_t(addr), waveform, &numOfElements);
      if (rpStatus == RP_OK) {
         std::copy(waveform, waveform + numOfElements, value);
         *nIn = numOfElements * sizeof(float);
      }
   }
       break;
   default:
      ERROR("%s Unexpected qualifier (%s)\n", this->full_name, qualifierImage(qualifier))
      ;
      status = asynError;
      break;
   }

   if (rpStatus != RP_OK) {
      ERROR("%s Request for qualifier (%s) failed with message: %s\n", this->full_name, qualifierImage(qualifier), rp_GetError(rpStatus));
      status = asynError;
   }

   return status;
}

//------------------------------------------------------------------------------
// IOC shell command registration
//------------------------------------------------------------------------------
//

// Define argument kinds
//
static const iocshArg verbosity_arg = { "Verbosity (0 .. 4)", iocshArgInt };
static const iocshArg port_name_arg = { "ASYN port name", iocshArgString };

//------------------------------------------------------------------------------
//
static const iocshArg * const RedPitaya_Initialise_Args[1] = { &verbosity_arg, };

static const iocshFuncDef RedPitaya_Initialise_Func_Def = { "RedPitaya_Initialise", 1, RedPitaya_Initialise_Args };

static void Call_RedPitaya_Initialise(const iocshArgBuf* args) {
   RedPitaya_Initialise(args[0].ival);
}

//------------------------------------------------------------------------------
//
static const iocshArg * const RedPitaya_Configure_Args[1] = { &port_name_arg, };

static const iocshFuncDef RedPitaya_Configure_Func_Def = { "RedPitaya_Configure", 1, RedPitaya_Configure_Args };

static void Call_RedPitaya_Configure(const iocshArgBuf* args) {
   new RedPitayaDriver(args[0].sval);
}

//------------------------------------------------------------------------------
//
static void RedPitaya_Startup(void) {
   INFO("RedPitaya Startup (driver version %s)\n", REDPITAYA_DRIVER_VERSION);
   iocshRegister(&RedPitaya_Initialise_Func_Def, Call_RedPitaya_Initialise);
   iocshRegister(&RedPitaya_Configure_Func_Def, Call_RedPitaya_Configure);
}

epicsExportRegistrar (RedPitaya_Startup);

// end
