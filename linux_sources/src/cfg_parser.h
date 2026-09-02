// Chirp configuration (.cfg) file handling.
// C++ port of readCfgFile.m, defineCLICommands.m, parseCLICommands2Struct.m
// and calculateChirpParams.m.
#ifndef CFG_PARSER_H
#define CFG_PARSER_H

#include <map>
#include <string>
#include <vector>

// Parsed CLI configuration. params maps a command name (e.g. "profileCfg")
// to a matrix of values: one row per occurrence of the command in the cfg
// file, one column per numeric parameter, in file order. This mirrors the
// MATLAB struct P where repeated commands append to parameter vectors.
struct CliConfig {
    std::vector<std::string> lines; // raw cfg lines, used when sending to device
    std::map<std::string, std::vector<std::vector<double>>> params;

    bool has(const std::string& cmd) const { return params.count(cmd) != 0; }
    size_t rows(const std::string& cmd) const;
    // Value of parameter column `col` of occurrence `row`; 0.0 if missing.
    double get(const std::string& cmd, size_t row, size_t col) const;
};

// Derived chirp/profile parameters (calculateChirpParams.m), one per profile.
struct ChirpParams {
    int    profileId = 0;
    double startFreq_GHz = 0;
    int    numADCSamples = 0;
    double adcSamplingRate_ksps = 0;
    double adcSamplingTime_usec = 0;
    double interChirpTime_usec = 0;
    double ttlBandwidth_MHz = 0;
    double validBandwidth_MHz = 0;
    double rangeMax_m = 0;
    double rangeResolution_m = 0;
    double velMax_mps = 0;
    double velResolution_mps = 0;
    double frameTime_msec = 0;
    double activeFrameTime_msec = 0;
    double dutyCycle_percent = 0;
    int    numRangeBins = 0;
    int    numDopplerBins = 0;
    double radarCubeSz_KB = 0;
    int    numTXChannel = 0;
    int    numRXChannel = 0;
};

// Read a cfg file into lines. Returns false if the file cannot be opened.
bool readCfgFile(const std::string& path, std::vector<std::string>& lines);

// Parse CLI command lines into a CliConfig, validating each command against
// the supported command set for mmWave SDK 03.03 + the TM demo commands.
CliConfig parseCliCommands(const std::vector<std::string>& lines);

// Compute derived chirp parameters for every profileCfg in the config.
std::vector<ChirpParams> calculateChirpParams(const CliConfig& P);

// Print the derived parameters to stdout (replaces the setup GUI summary).
void printChirpParams(const std::vector<ChirpParams>& params);

#endif // CFG_PARSER_H
