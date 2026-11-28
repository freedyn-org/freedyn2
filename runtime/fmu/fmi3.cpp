// FMI 3.0 for Co-Simulation: the functions of a freedyn2 FMU (fdgen.fmu packs it). One shared
// library serves every model: the model file is resources/model.fdm of the FMU. Variables:
// Float64 inputs and outputs (value references as in slave.hpp) and the independent variable
// time (the value reference after the outputs). Model exchange, scheduled execution, event mode,
// FMU states and derivatives are not provided: those functions return fmi3Error (written from
// the types of fmi3FunctionTypes.h, so that every function of the standard is there).

#include <exception>
#include <memory>
#include <string>

#include "fmi3Functions.h"
#include "slave.hpp"

namespace {

struct Instance {
  std::string name;
  fmi3InstanceEnvironment env = nullptr;
  fmi3LogMessageCallback logger = nullptr;
  bool debug = false;
  std::unique_ptr<fd::fmu::Slave> slave;
};

void log(const Instance* in, fmi3Status status, const char* category, const std::string& msg) {
  if (!in || !in->logger) return;
  if (status == fmi3OK && !in->debug) return;
  in->logger(in->env, status, category, (in->name + ": " + msg).c_str());
}

template <class F>
fmi3Status guarded(fmi3Instance i, const char* what, F&& f) {
  auto* in = static_cast<Instance*>(i);
  if (!in || !in->slave) return fmi3Error;
  try {
    return f(*in);
  } catch (const std::exception& e) {
    log(in, fmi3Error, "logStatusError", std::string(what) + ": " + e.what());
    return fmi3Error;
  }
}

fmi3Status unsupported(fmi3Instance i, const char* what) {
  log(static_cast<Instance*>(i), fmi3Error, "logStatusError", std::string(what) + " is not supported");
  return fmi3Error;
}

}  // namespace

extern "C" {

const char* fmi3GetVersion(void) { return fmi3Version; }

fmi3Status fmi3SetDebugLogging(fmi3Instance instance, fmi3Boolean loggingOn, size_t, const fmi3String[]) {
  auto* in = static_cast<Instance*>(instance);
  if (!in) return fmi3Error;
  in->debug = loggingOn;
  return fmi3OK;
}

fmi3Instance fmi3InstantiateModelExchange(fmi3String, fmi3String, fmi3String, fmi3Boolean, fmi3Boolean,
                                          fmi3InstanceEnvironment, fmi3LogMessageCallback) {
  return nullptr;  // co-simulation only
}

fmi3Instance fmi3InstantiateScheduledExecution(fmi3String, fmi3String, fmi3String, fmi3Boolean, fmi3Boolean,
                                               fmi3InstanceEnvironment, fmi3LogMessageCallback, fmi3ClockUpdateCallback,
                                               fmi3LockPreemptionCallback, fmi3UnlockPreemptionCallback) {
  return nullptr;
}

fmi3Instance fmi3InstantiateCoSimulation(fmi3String instanceName, fmi3String, fmi3String resourcePath, fmi3Boolean,
                                         fmi3Boolean loggingOn, fmi3Boolean eventModeUsed, fmi3Boolean,
                                         const fmi3ValueReference[], size_t, fmi3InstanceEnvironment instanceEnvironment,
                                         fmi3LogMessageCallback logMessage, fmi3IntermediateUpdateCallback) {
  auto in = std::make_unique<Instance>();
  in->name = instanceName ? instanceName : "freedyn2";
  in->env = instanceEnvironment;
  in->logger = logMessage;
  in->debug = loggingOn;
  if (eventModeUsed) {
    log(in.get(), fmi3Error, "logStatusError", "event mode is not supported (hasEventMode = false)");
    return nullptr;
  }
  if (!resourcePath) {
    log(in.get(), fmi3Error, "logStatusError", "no resource path");
    return nullptr;
  }
  try {  // resourcePath: a native directory path with a trailing separator (FMI 3.0)
    in->slave = std::make_unique<fd::fmu::Slave>(fd::fmu::native_dir(resourcePath) + "/model.fdm");
  } catch (const std::exception& e) {
    log(in.get(), fmi3Error, "logStatusError", std::string("cannot load the model: ") + e.what());
    return nullptr;
  }
  return in.release();
}

void fmi3FreeInstance(fmi3Instance instance) { delete static_cast<Instance*>(instance); }

fmi3Status fmi3EnterInitializationMode(fmi3Instance instance, fmi3Boolean, fmi3Float64, fmi3Float64 startTime,
                                       fmi3Boolean, fmi3Float64) {
  return guarded(instance, "fmi3EnterInitializationMode", [&](Instance& in) {
    in.slave->setup(startTime);
    return fmi3OK;
  });
}

fmi3Status fmi3ExitInitializationMode(fmi3Instance instance) {
  return guarded(instance, "fmi3ExitInitializationMode", [](Instance& in) {
    in.slave->initialize();
    return fmi3OK;
  });
}

fmi3Status fmi3EnterStepMode(fmi3Instance instance) {
  return guarded(instance, "fmi3EnterStepMode", [](Instance&) { return fmi3OK; });
}

fmi3Status fmi3Terminate(fmi3Instance instance) {
  return guarded(instance, "fmi3Terminate", [](Instance&) { return fmi3OK; });
}

fmi3Status fmi3GetFloat64(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences,
                          fmi3Float64 values[], size_t nValues) {
  return guarded(instance, "fmi3GetFloat64", [&](Instance& in) {
    if (nValues < nValueReferences) return fmi3Error;  // scalars only
    for (size_t i = 0; i < nValueReferences; ++i) {
      if (valueReferences[i] == static_cast<fmi3ValueReference>(in.slave->n_variables())) {
        values[i] = in.slave->time();  // the independent variable
      } else if (!in.slave->get(valueReferences[i], values[i])) {
        log(&in, fmi3Error, "logStatusError", "fmi3GetFloat64: no variable " + std::to_string(valueReferences[i]));
        return fmi3Error;
      }
    }
    return fmi3OK;
  });
}

fmi3Status fmi3SetFloat64(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences,
                          const fmi3Float64 values[], size_t nValues) {
  return guarded(instance, "fmi3SetFloat64", [&](Instance& in) {
    if (nValues < nValueReferences) return fmi3Error;
    for (size_t i = 0; i < nValueReferences; ++i)
      if (!in.slave->set(valueReferences[i], values[i])) {
        log(&in, fmi3Error, "logStatusError", "fmi3SetFloat64: " + std::to_string(valueReferences[i]) + " is not an input");
        return fmi3Error;
      }
    return fmi3OK;
  });
}

fmi3Status fmi3DoStep(fmi3Instance instance, fmi3Float64 currentCommunicationPoint, fmi3Float64 communicationStepSize,
                      fmi3Boolean, fmi3Boolean* eventHandlingNeeded, fmi3Boolean* terminateSimulation,
                      fmi3Boolean* earlyReturn, fmi3Float64* lastSuccessfulTime) {
  if (eventHandlingNeeded) *eventHandlingNeeded = fmi3False;
  if (terminateSimulation) *terminateSimulation = fmi3False;
  if (earlyReturn) *earlyReturn = fmi3False;
  const fmi3Status s = guarded(instance, "fmi3DoStep", [&](Instance& in) {
    in.slave->do_step(currentCommunicationPoint, communicationStepSize);
    return fmi3OK;
  });
  auto* in = static_cast<Instance*>(instance);
  if (lastSuccessfulTime && in && in->slave) *lastSuccessfulTime = in->slave->time();
  return s;
}

// --- not provided (from the types of fmi3FunctionTypes.h) ---------------------------------------

fmi3Status fmi3EnterEventMode(fmi3Instance instance) { return unsupported(instance, "fmi3EnterEventMode"); }

fmi3Status fmi3Reset(fmi3Instance instance) { return unsupported(instance, "fmi3Reset"); }

fmi3Status fmi3GetFloat32(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, fmi3Float32 values[], size_t nValues) {
  return nValueReferences ? unsupported(instance, "fmi3GetFloat32") : fmi3OK;
}

fmi3Status fmi3GetInt8(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, fmi3Int8 values[], size_t nValues) {
  return nValueReferences ? unsupported(instance, "fmi3GetInt8") : fmi3OK;
}

fmi3Status fmi3GetUInt8(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, fmi3UInt8 values[], size_t nValues) {
  return nValueReferences ? unsupported(instance, "fmi3GetUInt8") : fmi3OK;
}

fmi3Status fmi3GetInt16(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, fmi3Int16 values[], size_t nValues) {
  return nValueReferences ? unsupported(instance, "fmi3GetInt16") : fmi3OK;
}

fmi3Status fmi3GetUInt16(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, fmi3UInt16 values[], size_t nValues) {
  return nValueReferences ? unsupported(instance, "fmi3GetUInt16") : fmi3OK;
}

fmi3Status fmi3GetInt32(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, fmi3Int32 values[], size_t nValues) {
  return nValueReferences ? unsupported(instance, "fmi3GetInt32") : fmi3OK;
}

fmi3Status fmi3GetUInt32(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, fmi3UInt32 values[], size_t nValues) {
  return nValueReferences ? unsupported(instance, "fmi3GetUInt32") : fmi3OK;
}

fmi3Status fmi3GetInt64(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, fmi3Int64 values[], size_t nValues) {
  return nValueReferences ? unsupported(instance, "fmi3GetInt64") : fmi3OK;
}

fmi3Status fmi3GetUInt64(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, fmi3UInt64 values[], size_t nValues) {
  return nValueReferences ? unsupported(instance, "fmi3GetUInt64") : fmi3OK;
}

fmi3Status fmi3GetBoolean(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, fmi3Boolean values[], size_t nValues) {
  return nValueReferences ? unsupported(instance, "fmi3GetBoolean") : fmi3OK;
}

fmi3Status fmi3GetString(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, fmi3String values[], size_t nValues) {
  return nValueReferences ? unsupported(instance, "fmi3GetString") : fmi3OK;
}

fmi3Status fmi3GetBinary(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, size_t valueSizes[], fmi3Binary values[], size_t nValues) {
  return nValueReferences ? unsupported(instance, "fmi3GetBinary") : fmi3OK;
}

fmi3Status fmi3GetClock(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, fmi3Clock values[]) {
  return nValueReferences ? unsupported(instance, "fmi3GetClock") : fmi3OK;
}

fmi3Status fmi3SetFloat32(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, const fmi3Float32 values[], size_t nValues) {
  return nValueReferences ? unsupported(instance, "fmi3SetFloat32") : fmi3OK;
}

fmi3Status fmi3SetInt8(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, const fmi3Int8 values[], size_t nValues) {
  return nValueReferences ? unsupported(instance, "fmi3SetInt8") : fmi3OK;
}

fmi3Status fmi3SetUInt8(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, const fmi3UInt8 values[], size_t nValues) {
  return nValueReferences ? unsupported(instance, "fmi3SetUInt8") : fmi3OK;
}

fmi3Status fmi3SetInt16(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, const fmi3Int16 values[], size_t nValues) {
  return nValueReferences ? unsupported(instance, "fmi3SetInt16") : fmi3OK;
}

fmi3Status fmi3SetUInt16(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, const fmi3UInt16 values[], size_t nValues) {
  return nValueReferences ? unsupported(instance, "fmi3SetUInt16") : fmi3OK;
}

fmi3Status fmi3SetInt32(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, const fmi3Int32 values[], size_t nValues) {
  return nValueReferences ? unsupported(instance, "fmi3SetInt32") : fmi3OK;
}

fmi3Status fmi3SetUInt32(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, const fmi3UInt32 values[], size_t nValues) {
  return nValueReferences ? unsupported(instance, "fmi3SetUInt32") : fmi3OK;
}

fmi3Status fmi3SetInt64(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, const fmi3Int64 values[], size_t nValues) {
  return nValueReferences ? unsupported(instance, "fmi3SetInt64") : fmi3OK;
}

fmi3Status fmi3SetUInt64(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, const fmi3UInt64 values[], size_t nValues) {
  return nValueReferences ? unsupported(instance, "fmi3SetUInt64") : fmi3OK;
}

fmi3Status fmi3SetBoolean(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, const fmi3Boolean values[], size_t nValues) {
  return nValueReferences ? unsupported(instance, "fmi3SetBoolean") : fmi3OK;
}

fmi3Status fmi3SetString(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, const fmi3String values[], size_t nValues) {
  return nValueReferences ? unsupported(instance, "fmi3SetString") : fmi3OK;
}

fmi3Status fmi3SetBinary(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, const size_t valueSizes[], const fmi3Binary values[], size_t nValues) {
  return nValueReferences ? unsupported(instance, "fmi3SetBinary") : fmi3OK;
}

fmi3Status fmi3SetClock(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, const fmi3Clock values[]) {
  return nValueReferences ? unsupported(instance, "fmi3SetClock") : fmi3OK;
}

fmi3Status fmi3GetNumberOfVariableDependencies(fmi3Instance instance, fmi3ValueReference valueReference, size_t* nDependencies) { return unsupported(instance, "fmi3GetNumberOfVariableDependencies"); }

fmi3Status fmi3GetVariableDependencies(fmi3Instance instance, fmi3ValueReference dependent, size_t elementIndicesOfDependent[], fmi3ValueReference independents[], size_t elementIndicesOfIndependents[], fmi3DependencyKind dependencyKinds[], size_t nDependencies) { return unsupported(instance, "fmi3GetVariableDependencies"); }

fmi3Status fmi3GetFMUState(fmi3Instance instance, fmi3FMUState* FMUState) { return unsupported(instance, "fmi3GetFMUState"); }

fmi3Status fmi3SetFMUState(fmi3Instance instance, fmi3FMUState FMUState) { return unsupported(instance, "fmi3SetFMUState"); }

fmi3Status fmi3FreeFMUState(fmi3Instance instance, fmi3FMUState* FMUState) { return unsupported(instance, "fmi3FreeFMUState"); }

fmi3Status fmi3SerializedFMUStateSize(fmi3Instance instance, fmi3FMUState FMUState, size_t* size) { return unsupported(instance, "fmi3SerializedFMUStateSize"); }

fmi3Status fmi3SerializeFMUState(fmi3Instance instance, fmi3FMUState FMUState, fmi3Byte serializedState[], size_t size) { return unsupported(instance, "fmi3SerializeFMUState"); }

fmi3Status fmi3DeserializeFMUState(fmi3Instance instance, const fmi3Byte serializedState[], size_t size, fmi3FMUState* FMUState) { return unsupported(instance, "fmi3DeserializeFMUState"); }

fmi3Status fmi3GetDirectionalDerivative(fmi3Instance instance, const fmi3ValueReference unknowns[], size_t nUnknowns, const fmi3ValueReference knowns[], size_t nKnowns, const fmi3Float64 seed[], size_t nSeed, fmi3Float64 sensitivity[], size_t nSensitivity) { return unsupported(instance, "fmi3GetDirectionalDerivative"); }

fmi3Status fmi3GetAdjointDerivative(fmi3Instance instance, const fmi3ValueReference unknowns[], size_t nUnknowns, const fmi3ValueReference knowns[], size_t nKnowns, const fmi3Float64 seed[], size_t nSeed, fmi3Float64 sensitivity[], size_t nSensitivity) { return unsupported(instance, "fmi3GetAdjointDerivative"); }

fmi3Status fmi3EnterConfigurationMode(fmi3Instance instance) { return unsupported(instance, "fmi3EnterConfigurationMode"); }

fmi3Status fmi3ExitConfigurationMode(fmi3Instance instance) { return unsupported(instance, "fmi3ExitConfigurationMode"); }

fmi3Status fmi3GetIntervalDecimal(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, fmi3Float64 intervals[], fmi3IntervalQualifier qualifiers[]) { return unsupported(instance, "fmi3GetIntervalDecimal"); }

fmi3Status fmi3GetIntervalFraction(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, fmi3UInt64 counters[], fmi3UInt64 resolutions[], fmi3IntervalQualifier qualifiers[]) { return unsupported(instance, "fmi3GetIntervalFraction"); }

fmi3Status fmi3GetShiftDecimal(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, fmi3Float64 shifts[]) { return unsupported(instance, "fmi3GetShiftDecimal"); }

fmi3Status fmi3GetShiftFraction(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, fmi3UInt64 counters[], fmi3UInt64 resolutions[]) { return unsupported(instance, "fmi3GetShiftFraction"); }

fmi3Status fmi3SetIntervalDecimal(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, const fmi3Float64 intervals[]) { return unsupported(instance, "fmi3SetIntervalDecimal"); }

fmi3Status fmi3SetIntervalFraction(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, const fmi3UInt64 counters[], const fmi3UInt64 resolutions[]) { return unsupported(instance, "fmi3SetIntervalFraction"); }

fmi3Status fmi3SetShiftDecimal(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, const fmi3Float64 shifts[]) { return unsupported(instance, "fmi3SetShiftDecimal"); }

fmi3Status fmi3SetShiftFraction(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, const fmi3UInt64 counters[], const fmi3UInt64 resolutions[]) { return unsupported(instance, "fmi3SetShiftFraction"); }

fmi3Status fmi3EvaluateDiscreteStates(fmi3Instance instance) { return unsupported(instance, "fmi3EvaluateDiscreteStates"); }

fmi3Status fmi3UpdateDiscreteStates(fmi3Instance instance, fmi3Boolean* discreteStatesNeedUpdate, fmi3Boolean* terminateSimulation, fmi3Boolean* nominalsOfContinuousStatesChanged, fmi3Boolean* valuesOfContinuousStatesChanged, fmi3Boolean* nextEventTimeDefined, fmi3Float64* nextEventTime) { return unsupported(instance, "fmi3UpdateDiscreteStates"); }

fmi3Status fmi3EnterContinuousTimeMode(fmi3Instance instance) { return unsupported(instance, "fmi3EnterContinuousTimeMode"); }

fmi3Status fmi3CompletedIntegratorStep(fmi3Instance instance, fmi3Boolean noSetFMUStatePriorToCurrentPoint, fmi3Boolean* enterEventMode, fmi3Boolean* terminateSimulation) { return unsupported(instance, "fmi3CompletedIntegratorStep"); }

fmi3Status fmi3SetTime(fmi3Instance instance, fmi3Float64 time) { return unsupported(instance, "fmi3SetTime"); }

fmi3Status fmi3SetContinuousStates(fmi3Instance instance, const fmi3Float64 continuousStates[], size_t nContinuousStates) { return unsupported(instance, "fmi3SetContinuousStates"); }

fmi3Status fmi3GetContinuousStateDerivatives(fmi3Instance instance, fmi3Float64 derivatives[], size_t nContinuousStates) { return unsupported(instance, "fmi3GetContinuousStateDerivatives"); }

fmi3Status fmi3GetEventIndicators(fmi3Instance instance, fmi3Float64 eventIndicators[], size_t nEventIndicators) { return unsupported(instance, "fmi3GetEventIndicators"); }

fmi3Status fmi3GetContinuousStates(fmi3Instance instance, fmi3Float64 continuousStates[], size_t nContinuousStates) { return unsupported(instance, "fmi3GetContinuousStates"); }

fmi3Status fmi3GetNominalsOfContinuousStates(fmi3Instance instance, fmi3Float64 nominals[], size_t nContinuousStates) { return unsupported(instance, "fmi3GetNominalsOfContinuousStates"); }

fmi3Status fmi3GetNumberOfEventIndicators(fmi3Instance instance, size_t* nEventIndicators) { return unsupported(instance, "fmi3GetNumberOfEventIndicators"); }

fmi3Status fmi3GetNumberOfContinuousStates(fmi3Instance instance, size_t* nContinuousStates) { return unsupported(instance, "fmi3GetNumberOfContinuousStates"); }

fmi3Status fmi3GetOutputDerivatives(fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences, const fmi3Int32 orders[], fmi3Float64 values[], size_t nValues) { return unsupported(instance, "fmi3GetOutputDerivatives"); }

fmi3Status fmi3ActivateModelPartition(fmi3Instance instance, fmi3ValueReference clockReference, fmi3Float64 activationTime) { return unsupported(instance, "fmi3ActivateModelPartition"); }

}  // extern "C"
