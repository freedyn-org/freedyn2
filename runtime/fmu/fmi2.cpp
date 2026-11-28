// FMI 2.0 for Co-Simulation: the functions of a freedyn2 FMU (fdgen.fmu packs it). One shared
// library serves every model: the model file is resources/model.fdm of the FMU.

#include <cstdarg>
#include <cstdio>
#include <exception>
#include <memory>
#include <string>

#include "fmi2Functions.h"
#include "slave.hpp"

namespace {

struct Instance {
  std::string name;
  fmi2CallbackFunctions cb{};
  bool debug = false;
  std::unique_ptr<fd::fmu::Slave> slave;
};

void log(const Instance* in, fmi2Status status, const char* category, const std::string& msg) {
  if (!in || !in->cb.logger) return;
  if (status == fmi2OK && !in->debug) return;
  in->cb.logger(in->cb.componentEnvironment, in->name.c_str(), status, category, "%s", msg.c_str());
}

// Calls into the slave: an exception is an error of the FMU, logged.
template <class F>
fmi2Status guarded(fmi2Component c, const char* what, F&& f) {
  auto* in = static_cast<Instance*>(c);
  if (!in || !in->slave) return fmi2Error;
  try {
    return f(*in);
  } catch (const std::exception& e) {
    log(in, fmi2Error, "logStatusError", std::string(what) + ": " + e.what());
    return fmi2Error;
  }
}

fmi2Status unsupported(fmi2Component c, const char* what) {
  log(static_cast<Instance*>(c), fmi2Error, "logStatusError", std::string(what) + " is not supported");
  return fmi2Error;
}

}  // namespace

extern "C" {

const char* fmi2GetTypesPlatform(void) { return fmi2TypesPlatform; }
const char* fmi2GetVersion(void) { return fmi2Version; }

fmi2Status fmi2SetDebugLogging(fmi2Component c, fmi2Boolean loggingOn, size_t, const fmi2String[]) {
  auto* in = static_cast<Instance*>(c);
  if (!in) return fmi2Error;
  in->debug = loggingOn;
  return fmi2OK;
}

fmi2Component fmi2Instantiate(fmi2String instanceName, fmi2Type fmuType, fmi2String, fmi2String resourceLocation,
                              const fmi2CallbackFunctions* functions, fmi2Boolean, fmi2Boolean loggingOn) {
  auto in = std::make_unique<Instance>();
  in->name = instanceName ? instanceName : "freedyn2";
  if (functions) in->cb = *functions;
  in->debug = loggingOn;
  if (fmuType != fmi2CoSimulation) {
    log(in.get(), fmi2Error, "logStatusError", "a freedyn2 FMU is for co-simulation only");
    return nullptr;
  }
  if (!resourceLocation) {
    log(in.get(), fmi2Error, "logStatusError", "no resource location");
    return nullptr;
  }
  try {
    in->slave = std::make_unique<fd::fmu::Slave>(fd::fmu::path_from_uri(resourceLocation) + "/model.fdm");
  } catch (const std::exception& e) {
    log(in.get(), fmi2Error, "logStatusError", std::string("cannot load the model: ") + e.what());
    return nullptr;
  }
  return in.release();
}

void fmi2FreeInstance(fmi2Component c) { delete static_cast<Instance*>(c); }

fmi2Status fmi2SetupExperiment(fmi2Component c, fmi2Boolean, fmi2Real, fmi2Real startTime, fmi2Boolean, fmi2Real) {
  return guarded(c, "fmi2SetupExperiment", [&](Instance& in) {
    in.slave->setup(startTime);
    return fmi2OK;
  });
}

fmi2Status fmi2EnterInitializationMode(fmi2Component c) {
  return guarded(c, "fmi2EnterInitializationMode", [](Instance&) { return fmi2OK; });
}

fmi2Status fmi2ExitInitializationMode(fmi2Component c) {
  return guarded(c, "fmi2ExitInitializationMode", [](Instance& in) {
    in.slave->initialize();
    return fmi2OK;
  });
}

fmi2Status fmi2Terminate(fmi2Component c) { return guarded(c, "fmi2Terminate", [](Instance&) { return fmi2OK; }); }

fmi2Status fmi2Reset(fmi2Component c) { return unsupported(c, "fmi2Reset"); }

fmi2Status fmi2GetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Real value[]) {
  return guarded(c, "fmi2GetReal", [&](Instance& in) {
    for (size_t i = 0; i < nvr; ++i)
      if (!in.slave->get(vr[i], value[i])) {
        log(&in, fmi2Error, "logStatusError", "fmi2GetReal: no variable " + std::to_string(vr[i]));
        return fmi2Error;
      }
    return fmi2OK;
  });
}

fmi2Status fmi2SetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[]) {
  return guarded(c, "fmi2SetReal", [&](Instance& in) {
    for (size_t i = 0; i < nvr; ++i)
      if (!in.slave->set(vr[i], value[i])) {
        log(&in, fmi2Error, "logStatusError", "fmi2SetReal: " + std::to_string(vr[i]) + " is not an input");
        return fmi2Error;
      }
    return fmi2OK;
  });
}

// no integer, boolean or string variables
fmi2Status fmi2GetInteger(fmi2Component c, const fmi2ValueReference[], size_t nvr, fmi2Integer[]) {
  return nvr ? unsupported(c, "fmi2GetInteger") : fmi2OK;
}
fmi2Status fmi2GetBoolean(fmi2Component c, const fmi2ValueReference[], size_t nvr, fmi2Boolean[]) {
  return nvr ? unsupported(c, "fmi2GetBoolean") : fmi2OK;
}
fmi2Status fmi2GetString(fmi2Component c, const fmi2ValueReference[], size_t nvr, fmi2String[]) {
  return nvr ? unsupported(c, "fmi2GetString") : fmi2OK;
}
fmi2Status fmi2SetInteger(fmi2Component c, const fmi2ValueReference[], size_t nvr, const fmi2Integer[]) {
  return nvr ? unsupported(c, "fmi2SetInteger") : fmi2OK;
}
fmi2Status fmi2SetBoolean(fmi2Component c, const fmi2ValueReference[], size_t nvr, const fmi2Boolean[]) {
  return nvr ? unsupported(c, "fmi2SetBoolean") : fmi2OK;
}
fmi2Status fmi2SetString(fmi2Component c, const fmi2ValueReference[], size_t nvr, const fmi2String[]) {
  return nvr ? unsupported(c, "fmi2SetString") : fmi2OK;
}

fmi2Status fmi2GetFMUstate(fmi2Component c, fmi2FMUstate*) { return unsupported(c, "fmi2GetFMUstate"); }
fmi2Status fmi2SetFMUstate(fmi2Component c, fmi2FMUstate) { return unsupported(c, "fmi2SetFMUstate"); }
fmi2Status fmi2FreeFMUstate(fmi2Component c, fmi2FMUstate*) { return unsupported(c, "fmi2FreeFMUstate"); }
fmi2Status fmi2SerializedFMUstateSize(fmi2Component c, fmi2FMUstate, size_t*) {
  return unsupported(c, "fmi2SerializedFMUstateSize");
}
fmi2Status fmi2SerializeFMUstate(fmi2Component c, fmi2FMUstate, fmi2Byte[], size_t) {
  return unsupported(c, "fmi2SerializeFMUstate");
}
fmi2Status fmi2DeSerializeFMUstate(fmi2Component c, const fmi2Byte[], size_t, fmi2FMUstate*) {
  return unsupported(c, "fmi2DeSerializeFMUstate");
}
fmi2Status fmi2GetDirectionalDerivative(fmi2Component c, const fmi2ValueReference[], size_t, const fmi2ValueReference[],
                                        size_t, const fmi2Real[], fmi2Real[]) {
  return unsupported(c, "fmi2GetDirectionalDerivative");
}

fmi2Status fmi2SetRealInputDerivatives(fmi2Component c, const fmi2ValueReference[], size_t, const fmi2Integer[],
                                       const fmi2Real[]) {
  return unsupported(c, "fmi2SetRealInputDerivatives");
}
fmi2Status fmi2GetRealOutputDerivatives(fmi2Component c, const fmi2ValueReference[], size_t, const fmi2Integer[],
                                        fmi2Real[]) {
  return unsupported(c, "fmi2GetRealOutputDerivatives");
}

fmi2Status fmi2DoStep(fmi2Component c, fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize,
                      fmi2Boolean) {
  return guarded(c, "fmi2DoStep", [&](Instance& in) {
    in.slave->do_step(currentCommunicationPoint, communicationStepSize);
    return fmi2OK;
  });
}

fmi2Status fmi2CancelStep(fmi2Component c) { return unsupported(c, "fmi2CancelStep"); }

fmi2Status fmi2GetStatus(fmi2Component c, const fmi2StatusKind, fmi2Status*) { return unsupported(c, "fmi2GetStatus"); }
fmi2Status fmi2GetRealStatus(fmi2Component c, const fmi2StatusKind s, fmi2Real* value) {
  return guarded(c, "fmi2GetRealStatus", [&](Instance& in) {
    if (s != fmi2LastSuccessfulTime) return fmi2Discard;
    *value = in.slave->time();
    return fmi2OK;
  });
}
fmi2Status fmi2GetIntegerStatus(fmi2Component c, const fmi2StatusKind, fmi2Integer*) {
  return unsupported(c, "fmi2GetIntegerStatus");
}
fmi2Status fmi2GetBooleanStatus(fmi2Component c, const fmi2StatusKind, fmi2Boolean*) {
  return unsupported(c, "fmi2GetBooleanStatus");
}
fmi2Status fmi2GetStringStatus(fmi2Component c, const fmi2StatusKind, fmi2String*) {
  return unsupported(c, "fmi2GetStringStatus");
}

}  // extern "C"
