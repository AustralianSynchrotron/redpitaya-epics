/* $File: //ASP/tec/daq/RedPitaya/trunk/RedPitayaSup/src/drvRedPitaya.cpp $
 * $Revision: #2 $
 * $DateTime: 2017/06/05 10:06:58 $
 * Last checked in by: $Author: pozara $
 *
 * Description:
 * This is a driver support to be used with an on board IOC for
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

#include <rp.h>
#include "drvRedPitaya.h"

// Useful type neutral numerical macro fuctions.
//
#define MIN(a, b)           ((a) <= (b) ? (a) : (b))

// Calculates number of items in an array
//
#define ARRAY_LENGTH(xx)   ((int) (sizeof (xx) /sizeof (xx [0])))

#define REDPITAYA_DRIVER_VERSION   "1.0"

#define MAX_NUMBER_OF_ADDRESSES    8      // 0 to 8

#define MIN_BUFFER_TIME_SPAN       0.000131

// Pin and led minimum and maximum values
//
#define MIN_SOURCE_NUM    0
#define MAX_SOURCE_NUM    1
#define MIN_DIG_PIN       0
#define MAX_DIG_PIN       7
#define MIN_ANALOG_PIN    0
#define MAX_ANALOG_PIN    3

#define N_PIN_OFFSET   16
#define P_PIN_OFFSET   8

static const uint32_t BUFFER_SIZE = 16354;

// Array used to read the buffer
//
float* ch1_buffer = (float *) malloc (BUFFER_SIZE * sizeof (float));
float* ch2_buffer = (float *) malloc (BUFFER_SIZE * sizeof (float));

// Array used to do callbacks
//
epicsFloat64* ch1_data = (epicsFloat64 *) malloc (BUFFER_SIZE * sizeof (epicsFloat64));
epicsFloat64* ch2_data = (epicsFloat64 *) malloc (BUFFER_SIZE * sizeof (epicsFloat64));


// Initialization of decimation factors
//
const static int decimationFactors [6] = {1, 8, 64, 1024, 8192, 65536};

// Tigger source set in the EPICS database
//
rp_acq_trig_src_t setTriggerSource;

struct QualifierDefinitions {
   asynParamType type;
   const char* name;
};

// Qualifier lookup table - used by RedPitaya_DrvUser_Create and
// RedPitaya_Qualifier_Image
//
// MUST be consistent with enum Qualifiers type out of RedPitayaDriver (in drvRedPitaya.h)
//
static const QualifierDefinitions qualifierList [] = {
      { asynParamOctet,        "DRVVER"      },  // DriverVersion
      { asynParamFloat64Array, "DATA"        },  // Data Read From the Channel
      { asynParamInt32,        "NDPDIR"      },  // Set Digital Pin N to be an Input or Output
      { asynParamInt32,        "PDPDIR"      },  // Set Digital Pin P to be an Input or Output
      { asynParamInt32,        "NDPSTATE"    },  // Digital Pin N State
      { asynParamInt32,        "PDPSTATE"    },  // Digital Pin P State
      { asynParamInt32,        "LEDSTATE"    },  // LED State
      { asynParamFloat64,      "APOUTSTATE"  },  // Analogue output pin voltage
      { asynParamFloat64,      "APINSTATE"   },  // Analogue input pin voltage
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
   };

// Supported interrupts.
//
const static int interruptMask = asynFloat64ArrayMask;

// Any interrupt must also have an interface.
//
const static int interfaceMask = interruptMask | asynDrvUserMask | asynOctetMask
      | asynInt32Mask | asynFloat64Mask;

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
static void devprintf (const int required_min_verbosity, const char* function,
      const int line_no, const char* format, ...) {
   if (verbosity >= required_min_verbosity) {
      char message [100];
      va_list arguments;
      va_start (arguments, format);
      vsnprintf (message, sizeof (message), format, arguments);
      va_end (arguments);
      errlogPrintf ("RedPitayaDriver: %s:%d  %s", function, line_no, message);
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
const char* RedPitayaDriver::qualifierImage (const Qualifiers q) {
   static char result [24];

   if ((q >= 0) && (q < NUMBER_QUALIFIERS)) {
      return qualifierList [q].name;
   } else {
      sprintf (result, "unknown (%d)", q);
      return result;
   }
}

//------------------------------------------------------------------------------
// static
//
static void RedPitaya_Initialise (const int verbosityIn) {
   verbosity = verbosityIn;
   driver_initialised = true;
}

//------------------------------------------------------------------------------
//
void RedPitayaDriver::shutdown (void* arg) {
   RedPitayaDriver* self = (RedPitayaDriver*) arg;

   if (self && self->is_initialised) {
      INFO ("RedPitayaDriver: shutting down: %s\n", self->full_name)
   }
}

//
void dataAcquisition (void *drvPvt) {
   RedPitayaDriver *self = (RedPitayaDriver *) drvPvt;

   self->dataAcquisition();
}

//------------------------------------------------------------------------------
//
RedPitayaDriver::RedPitayaDriver (const char* port_name) :
      asynPortDriver (port_name,              //
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
      ERROR ("driver not initialised (call %s first)\n", "RedPitaya_Initialise");
      return;
   }

   // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

   // Verify port names are sensible
   //
   if ((port_name == NULL) || (strcmp (port_name, "") == 0)) {
      ERROR ("null/empty port name\n", 0);
      return;
   }

   snprintf (this->full_name, sizeof (this->full_name), "%s", port_name);

   // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
   // Set up asyn parameters.
   //
   for (int j = 0; j < ARRAY_LENGTH (qualifierList); j++) {
      asynStatus status = createParam (qualifierList [j].name,
            qualifierList [j].type, &this->indexList [j]);
      if (status != asynSuccess) {
         ERROR ("Parameter creation failed\n");
         return;
      }
   }

   int initStatus = rp_Init ();
   if (initStatus != RP_OK) {
      ERROR ("Driver initialisation failed with error: %s\n",
            rp_GetError (initStatus));
      return;
   }

   INFO ("RedPitaya library version: %s\n", rp_GetVersion ());

   // Setting default acquisition sleep time
   //
   this->acquisitionSleep = 1.0;

   // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
   // Register for epics exit callback.
   // Note: shutdown is a static function so we must pass this as a parameter.
   //
   epicsAtExit (RedPitayaDriver::shutdown, this);

   this->is_initialised = true;

   INFO ("%s initialisation complete\n", this->full_name);
}

//------------------------------------------------------------------------------
//
RedPitayaDriver::~RedPitayaDriver () {
   // Clean up a bit
   //
   rp_Release ();
   free (ch1_buffer);
   free (ch2_buffer);
   free (ch1_data);
   free (ch2_buffer);
}

//------------------------------------------------------------------------------
// Asyn callback functions
//------------------------------------------------------------------------------
//
void RedPitayaDriver::report (FILE * fp, int details) {
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
void RedPitayaDriver::dataAcquisition () {

   rp_acq_decimation_t decimation;
   double bufferTimeout;

   // Loop until the user requests the acquisition to stop
   //
   while (acquiring) {
      // Calculate the time it takes for the buffer to be filled with new data
      // and only then set a trigger.
      // At no decimation the buffer holds 0.000131 seconds of samples
      //
      rp_AcqGetDecimation (&decimation);
      bufferTimeout = decimationFactors [decimation] * 0.000131;

      int rpStatus = rp_AcqStart ();
      if (rpStatus != RP_OK) {
         return;
      }

      // Give buffer time to flush old data
      //
      epicsThreadSleep (bufferTimeout);

      // Set the trigger that has been set from the EPICS database
      //
      rp_AcqSetTriggerSrc (setTriggerSource);

      // Get currently set trigger from the device. This should be the same
      // as what we set, but when the trigger goes to DISABLED we know that
      // recording of data has finished.
      //
      rp_acq_trig_src_t currentTriggerSource;
      rp_AcqGetTriggerSrc (&currentTriggerSource);
      while (currentTriggerSource != 0 && acquiring) {
         rp_AcqGetTriggerSrc (&currentTriggerSource);
      }

      // Break the loop if the acquisition has been cancelled while waiting
      // for the trigger
      //
      if (!acquiring) {
         break;
      }

      // Get pointer to the trigger in the buffer
      //
      uint32_t triggerPos;
      rp_AcqGetWritePointerAtTrig (&triggerPos);

      // Read the whole buffer for both channels
      //
      uint32_t buff_size = BUFFER_SIZE;
      rpStatus += rp_AcqGetOldestDataV (RP_CH_1, &buff_size, ch1_buffer);
      rpStatus += rp_AcqGetOldestDataV (RP_CH_2, &buff_size, ch2_buffer);

      if (rpStatus == RP_OK) {
         // Copy retrieved data to return array.
         //
         // We copy sample by sample beginning from the trigger position in the buffer, using modulo of
         // buffer length.
         //
         for (size_t i = 0; i < buff_size; i++) {
            ch1_data [(triggerPos+i) % buff_size] = (epicsFloat64) ch1_buffer [(triggerPos+i) % buff_size];
            ch2_data [(triggerPos+i) % buff_size] = (epicsFloat64) ch2_buffer [(triggerPos+i) % buff_size];
         }

         // Do the callbacks to EPICS database waveform records used to present the acquired data
         //
         doCallbacksFloat64Array (ch1_data, buff_size, SourceData, RP_CH_1);
         doCallbacksFloat64Array (ch2_data, buff_size, SourceData, RP_CH_2);
      }

      // If this is a single shot acquisition exit the loop otherwise sleep for a number of seconds
      // and repeat
      //
      if (singleShot) {
         // Set a flag to false so that a new acquisition thread is allowed to be started
         //
         acquiring = false;
         break;
      }

      epicsThreadSleep (acquisitionSleep);
   }
}

//------------------------------------------------------------------------------
//
asynStatus RedPitayaDriver::readOctet (asynUser* pasynUser, char* data,
      size_t maxchars, size_t* nbytesTransfered, int* eomReason) {
   const Qualifiers qualifier = (Qualifiers) pasynUser->reason;
   asynStatus status;
   int addr;
   size_t length;

   *nbytesTransfered = 0;

   // Extract and validate address index.
   //
   pasynManager->getAddr (pasynUser, &addr);

   status = asynSuccess;        // hypothesise okay

   switch (qualifier) {

   case DriverVersion:
      strncpy (data, REDPITAYA_DRIVER_VERSION, maxchars);
      length = strlen (REDPITAYA_DRIVER_VERSION);
      *nbytesTransfered = MIN (length, maxchars);
      break;

   default:
      ERROR ("%s Unexpected qualifier (%s)\n", this->full_name,
            qualifierImage (qualifier))
      ;
      status = asynError;
      break;
   }

   return status;
}

asynStatus RedPitayaDriver::checkAddrLimits (const Qualifiers qualifier,
      const int addr) {

   int minAddress, maxAddress;

   switch (qualifier) {
   case SourceData:
   case AcquisitionGain:
      minAddress = MIN_SOURCE_NUM;
      maxAddress = MAX_SOURCE_NUM;
      break;
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
      minAddress = 0;
      maxAddress = 0;
   }

   if (addr < minAddress || addr > maxAddress) {
      ERROR("%s address (%d) out of range\n", qualifierImage (qualifier), addr);
      return asynError;
   }

   return asynSuccess;
}

//------------------------------------------------------------------------------
//
asynStatus RedPitayaDriver::writeInt32 (asynUser* pasynUser, epicsInt32 value) {
   const Qualifiers qualifier = (Qualifiers) pasynUser->reason;
   asynStatus status;
   int addr;

   if (this->is_initialised != true) {
      return asynError;
   }

   // Extract and validate address index.
   //
   pasynManager->getAddr (pasynUser, &addr);

   status = checkAddrLimits (qualifier, addr);

   if (status != asynSuccess) {
      return status;
   }

   status = asynSuccess;       // hypothesise okay
   int rpStatus = RP_OK;       // hypothesise okay

   switch (qualifier) {

   case PDigPinDir:
      rpStatus = rp_DpinSetDirection ((rp_dpin_t) (P_PIN_OFFSET + addr),
            (rp_pinDirection_t) value);
      break;
   case NDigPinDir:
      rpStatus = rp_DpinSetDirection ((rp_dpin_t) (N_PIN_OFFSET + addr),
            (rp_pinDirection_t) value);
      break;
   case PDigPinState:
      rpStatus = rp_DpinSetState ((rp_dpin_t) (N_PIN_OFFSET + addr),
            (rp_pinState_t) value);
      break;
   case NDigPinState:
      rpStatus = rp_DpinSetState ((rp_dpin_t) (P_PIN_OFFSET + addr),
            (rp_pinState_t) value);
      break;
   case LedState:
      rpStatus = rp_DpinSetState ((rp_dpin_t) addr, (rp_pinState_t) value);
      break;
   case ContAcquisitionStart:
   case SingleAcquisitionStart:
      if (value == 1 && !acquiring) {
         acquiring = true;
         singleShot = qualifier == SingleAcquisitionStart;
         status = (asynStatus)(
               epicsThreadCreate ("rpReadData", epicsThreadPriorityMedium,
                     epicsThreadGetStackSize (epicsThreadStackMedium),
                     (EPICSTHREADFUNC)::dataAcquisition, this) == NULL);
      }
      break;
   case AcquisitionStop:
      rpStatus = rp_AcqStop ();
      acquiring = false;
      break;
   case AcquisitionReset:
      if (value == 1) {
         rpStatus = rp_AcqReset ();
      }
      break;
   case Decimation:
      if (value >= 0 && value <= 5) {
         rpStatus = rp_AcqSetDecimation ((rp_acq_decimation_t) value);
      } else {
         ERROR("%s Unexpected value for qualifier (%s)\n", this->full_name,
               qualifierImage (qualifier));
         status = asynError;
      }
      break;
   case SamplingRate:
      if (value >= 0 && value <= 5) {
         rpStatus = rp_AcqSetSamplingRate ((rp_acq_sampling_rate_t) value);
      } else {
         ERROR ("%s Unexpected value for qualifier (%s)\n", this->full_name,
               qualifierImage (qualifier));
         status = asynError;
      }
      break;
   case Averaging:
      if (value == 0 || value == 1) {
         rpStatus = rp_AcqSetAveraging (value == 1);
      } else {
         ERROR ("%s Unexpected value for qualifier (%s)\n", this->full_name,
               qualifierImage (qualifier));
         status = asynError;
      }
      break;
      break;
   case TriggerSrc:
      if (value <= 9 && value >= 0) {
         // We save the value in the global variable and then set the trigger source
         // when it's safe to do so
         //
         setTriggerSource = rp_acq_trig_src_t (value);
      } else {
         ERROR ("%s Unexpected value for qualifier (%s)\n", this->full_name,
               qualifierImage (qualifier));
         status = asynError;
      }
      break;
   case TriggerDelay:
      rpStatus = rp_AcqSetTriggerDelayNs (int64_t (value));
      break;
   case AcquisitionGain:
      if (value == 0 || value == 1) {
         rpStatus = rp_AcqSetGain ((rp_channel_t) addr, rp_pinState_t (value));
      } else {
         ERROR ("%s Unexpected value for qualifier (%s)\n", this->full_name,
               qualifierImage (qualifier));
         status = asynError;
      }
      break;
   default:
      ERROR ("%s Unexpected qualifier (%s)\n", this->full_name,
            qualifierImage (qualifier))
      ;
      status = asynError;
      break;
   }

   if (rpStatus != 0) {
      ERROR ("%s Command for qualifier (%s) failed with message: %s\n",
            this->full_name, qualifierImage (qualifier), rp_GetError (rpStatus));
      status = asynError;
   }

   return status;
}

//------------------------------------------------------------------------------
//
asynStatus RedPitayaDriver::readInt32 (asynUser* pasynUser, epicsInt32* value) {
   const Qualifiers qualifier = (Qualifiers) pasynUser->reason;
   asynStatus status;
   int addr;

   if (this->is_initialised != true) {
      return asynError;
   }

   // Extract and validate address index.
   //
   pasynManager->getAddr (pasynUser, &addr);

   status = checkAddrLimits (qualifier, addr);

   if (status != asynSuccess) {
      return status;
   }

   status = asynSuccess;       // hypothesise okay
   int rpStatus = RP_OK;       // hypothesise okay

   switch (qualifier) {
   case PDigPinDir: {
      rp_pinDirection_t direction;
      rpStatus = rp_DpinGetDirection ((rp_dpin_t) (P_PIN_OFFSET + addr),
            &direction);
      if (rpStatus == RP_OK) {
         *value = (epicsInt32) direction;
      }
   }
      break;
   case NDigPinDir: {
      rp_pinDirection_t direction;
      rpStatus = rp_DpinGetDirection ((rp_dpin_t) (N_PIN_OFFSET + addr),
            &direction);
      if (rpStatus == RP_OK) {
         *value = (epicsInt32) direction;
      }
   }
      break;
   case PDigPinState: {
      rp_pinState_t state;
      rpStatus = rp_DpinGetState ((rp_dpin_t) (P_PIN_OFFSET + addr), &state);
      if (rpStatus == RP_OK) {
         *value = (epicsInt32) state;
      }
   }
      break;
   case NDigPinState: {
      rp_pinState_t state;
      rpStatus = rp_DpinGetState ((rp_dpin_t) (P_PIN_OFFSET + addr), &state);
      if (rpStatus == RP_OK) {
         *value = (epicsInt32) state;
      }
   }
      break;
   case LedState: {
      rp_pinState_t state;
      rpStatus = rp_DpinGetState ((rp_dpin_t) addr, &state);
      if (rpStatus == RP_OK) {
         *value = (epicsInt32) state;
      }
   }
      break;
   case ContAcquisitionStart:
   case SingleAcquisitionStart:
   case AcquisitionStop:
   case AcquisitionReset:
      // Nothing to read really
      //
      break;
   case AcquisitionStatus:
      *value = (epicsInt32) acquiring ? 1 : 0;
      break;
   case AcquisitionGain: {
      rp_pinState_t state;
      rpStatus = rp_AcqGetGain ((rp_channel_t) addr, &state);
      if (rpStatus == RP_OK) {
         *value = (epicsInt32) state;
      }
   }
      break;
   case Decimation: {
      rp_acq_decimation_t decimation;
      rpStatus = rp_AcqGetDecimation (&decimation);
      if (rpStatus == RP_OK) {
         *value = (epicsInt32) decimation;
      }
   }
      break;
   case SamplingRate: {
      rp_acq_sampling_rate_t rate;
      rpStatus = rp_AcqGetSamplingRate (&rate);
      if (rpStatus == RP_OK) {
         *value = (epicsInt32) rate;
      }
   }
      break;
   case Averaging: {
      bool averaging;
      rpStatus = rp_AcqGetAveraging (&averaging);
      if (rpStatus == RP_OK) {
         *value = (epicsInt32) averaging;
      }
   }
      break;
   case TriggerSrc: {
      rp_acq_trig_src_t triggerSource;
      rpStatus = rp_AcqGetTriggerSrc (&triggerSource);
      if (rpStatus == RP_OK) {
         *value = (epicsInt32) triggerSource;
      }
   }
      break;
   case TriggerState: {
      rp_acq_trig_state_t triggerState;
      rpStatus = rp_AcqGetTriggerState (&triggerState);
      if (rpStatus == RP_OK) {
         *value = (epicsInt32) triggerState;
      }
   }
      break;
   case TriggerDelay: {
      int64_t triggerDelay;
      rpStatus = rp_AcqGetTriggerDelayNs (&triggerDelay);
      if (rpStatus == RP_OK) {
         *value = (epicsInt32) triggerDelay;
      }
   }
      break;
   case BufferSize: {
      uint32_t size;
      rpStatus = rp_AcqGetBufSize (&size);
      if (rpStatus == RP_OK) {
         *value = (epicsInt32) size;
      }
   }
      break;
   default:
      ERROR ("%s Unexpected qualifier (%s)\n", this->full_name,
            qualifierImage (qualifier));
      status = asynError;
      break;
   }

   if (rpStatus != RP_OK) {
      ERROR ("%s Request for qualifier (%s) failed with message: %s\n",
            this->full_name, qualifierImage (qualifier), rp_GetError (rpStatus));
      status = asynError;
   }

   return status;

}

//------------------------------------------------------------------------------
//
asynStatus RedPitayaDriver::writeFloat64 (asynUser* pasynUser,
      epicsFloat64 value) {
   const Qualifiers qualifier = (Qualifiers) pasynUser->reason;
   asynStatus status;
   int addr;

   if (this->is_initialised != true) {
      return asynError;
   }

   // Extract and validate address index.
   //
   pasynManager->getAddr (pasynUser, &addr);

   status = checkAddrLimits (qualifier, addr);

   if (status != asynSuccess) {
      return status;
   }

   status = asynSuccess;       // hypothesise okay
   int rpStatus = RP_OK;       // hypothesise okay

   switch (qualifier) {
   case AnalogPinOut:
      rpStatus = rp_ApinSetValue ((rp_apin_t) addr, value);
      break;
   case TriggerLevel:
      rpStatus = rp_AcqSetTriggerLevel (RP_CH_1, value);
      rpStatus += rp_AcqSetTriggerLevel (RP_CH_2, value);
      break;
   case AcquisitionRate:
      acquisitionSleep = value;
      break;
   case TriggerHysteresis:
      rpStatus = rp_AcqSetTriggerHyst ((float) value);
      break;
   default:
      ERROR ("%s Unexpected qualifier (%s)\n", this->full_name,
            qualifierImage (qualifier))
      ;
      status = asynError;
      break;
   }

   if (rpStatus != RP_OK) {
      ERROR ("%s Command for qualifier (%s) failed with message: %s\n",
            this->full_name, qualifierImage (qualifier), rp_GetError (rpStatus));
      status = asynError;
   }

   return status;
}

//------------------------------------------------------------------------------
//
asynStatus RedPitayaDriver::readFloat64 (asynUser* pasynUser,
      epicsFloat64* value) {
   const Qualifiers qualifier = (Qualifiers) pasynUser->reason;

   asynStatus status;
   int addr;

   if (this->is_initialised != true) {
      return asynError;
   }

   // Extract and validate address index.
   //
   pasynManager->getAddr (pasynUser, &addr);

   status = checkAddrLimits (qualifier, addr);

   if (status != asynSuccess) {
      return status;
   }

   status = asynSuccess;       // hypothesise okay
   int rpStatus = RP_OK;       // hypothesise okay

   switch (qualifier) {
   case AnalogPinOut: {
      float voltage;
      rpStatus = rp_ApinGetValue ((rp_apin_t) addr, &voltage);
      if (rpStatus == RP_OK) {
         *value = (epicsFloat64) voltage;
      }
   }
      break;
   case AnalogPinIn: {
      float voltage;
      rpStatus = rp_AIpinGetValue ((rp_apin_t) addr, &voltage);
      if (rpStatus == RP_OK) {
         *value = (epicsFloat64) voltage;
      }
   }
      break;
   case TriggerLevel: {
      float level;
      rpStatus = rp_AcqGetTriggerLevel (&level);
      if (rpStatus == RP_OK) {
         *value = (epicsFloat64) level;
      }
   }
      break;
   case AcquisitionRate:
      break;
   case TriggerHysteresis: {
      float hysteresis;
      rpStatus = rp_AcqGetTriggerHyst (&hysteresis);
      if (rpStatus == RP_OK) {
         *value = (epicsFloat64) hysteresis;
      }
   }
      break;
   default:
      ERROR ("%s Unexpected qualifier (%s)\n", this->full_name,
            qualifierImage (qualifier))
      ;
      status = asynError;
      break;
   }

   if (rpStatus != RP_OK) {
      ERROR("%s Request for qualifier (%s) failed with message: %s\n",
            this->full_name, qualifierImage (qualifier), rp_GetError (rpStatus));
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

static const iocshFuncDef RedPitaya_Initialise_Func_Def = {
      "RedPitaya_Initialise", 1, RedPitaya_Initialise_Args };

static void Call_RedPitaya_Initialise (const iocshArgBuf* args) {
   RedPitaya_Initialise(args[0].ival);
}

//------------------------------------------------------------------------------
//
static const iocshArg * const RedPitaya_Configure_Args [1] = { &port_name_arg, };

static const iocshFuncDef RedPitaya_Configure_Func_Def = {
      "RedPitaya_Configure", 1, RedPitaya_Configure_Args };

static void Call_RedPitaya_Configure (const iocshArgBuf* args) {
   new RedPitayaDriver(args[0].sval);
}

//------------------------------------------------------------------------------
//
static void RedPitaya_Startup (void) {
   INFO("RedPitaya Startup (driver version %s)\n", REDPITAYA_DRIVER_VERSION);

   iocshRegister (&RedPitaya_Initialise_Func_Def, Call_RedPitaya_Initialise);

   iocshRegister (&RedPitaya_Configure_Func_Def, Call_RedPitaya_Configure);

}

epicsExportRegistrar (RedPitaya_Startup);

// end
