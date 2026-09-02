#include "cfg_parser.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace {

// Supported CLI commands and their accepted parameter counts, as defined in
// defineCLICommands.m for SDK version 03.03 plus the TM demo commands.
// Some gtrack commands accept two counts because newer TM demo cfg files
// (like the ones in chirp_configs/) use extended forms of them.
const std::map<std::string, std::vector<int>>& supportedCommands()
{
    static const std::map<std::string, std::vector<int>> cmds = {
        // Standard mmWave SDK 03.03 commands
        {"dfeDataOutputMode", {1}},
        {"channelCfg", {3}},
        {"adcCfg", {2}},
        {"adcbufCfg", {5}},
        {"profileCfg", {14}},
        {"chirpCfg", {8}},
        {"lowPower", {2}},
        {"frameCfg", {7}},
        {"advFrameCfg", {5}},
        {"subFrameCfg", {10}},
        {"guiMonitor", {7}},
        {"cfarCfg", {9}},
        {"multiObjBeamForming", {3}},
        {"calibDcRangeSig", {5}},
        {"clutterRemoval", {2}},
        {"aoaFovCfg", {5}},
        {"cfarFovCfg", {4}},
        {"compRangeBiasAndRxChanPhase", {25}},
        {"measureRangeBiasAndRxChanPhase", {3}},
        {"extendedMaxVelocity", {2}},
        {"CQRxSatMonitor", {5}},
        {"CQSigImgMonitor", {3}},
        {"analogMonitor", {2}},
        {"lvdsStreamCfg", {4}},
        {"bpmCfg", {4}},
        // Traffic Monitoring (demoType 'TM') commands
        {"trackingCfg", {8, 7}},
        {"staticBoundaryBox", {6}},
        {"boundaryBox", {6}},
        {"gatingParam", {5}},
        {"stateParam", {5, 6}},
        {"allocationParam", {6}},
        {"maxAcceleration", {3}},
        {"sensorPosition", {3}},
        {"presenceBoundaryBox", {6}},
    };
    return cmds;
}

std::vector<std::string> splitWhitespace(const std::string& s)
{
    std::vector<std::string> tokens;
    std::istringstream iss(s);
    std::string tok;
    while (iss >> tok)
        tokens.push_back(tok);
    return tokens;
}

int popcount32(unsigned v)
{
    int n = 0;
    for (; v; v >>= 1)
        n += static_cast<int>(v & 1u);
    return n;
}

// Parameter column indices used by calculateChirpParams (file order).
enum ProfileCfgCol { PC_PROFILE_ID = 0, PC_START_FREQ = 1, PC_IDLE_TIME = 2,
                     PC_ADC_START_TIME = 3, PC_RAMP_END_TIME = 4,
                     PC_FREQ_SLOPE = 7, PC_NUM_ADC_SAMPLES = 9,
                     PC_DIG_OUT_RATE = 10 };
enum FrameCfgCol { FC_CHIRP_START = 0, FC_CHIRP_END = 1, FC_NUM_LOOPS = 2,
                   FC_FRAME_PERIOD = 4 };
enum SubFrameCfgCol { SFC_SUBFRAME_NUM = 0, SFC_CHIRP_START_IDX = 2,
                      SFC_NUM_CHIRPS = 3, SFC_NUM_LOOPS = 4,
                      SFC_SUBFRAME_PERIOD = 9 };
enum ChirpCfgCol { CC_PROFILE_ID = 2 };

} // namespace

size_t CliConfig::rows(const std::string& cmd) const
{
    auto it = params.find(cmd);
    return it == params.end() ? 0 : it->second.size();
}

double CliConfig::get(const std::string& cmd, size_t row, size_t col) const
{
    auto it = params.find(cmd);
    if (it == params.end() || row >= it->second.size() || col >= it->second[row].size())
        return 0.0;
    return it->second[row][col];
}

bool readCfgFile(const std::string& path, std::vector<std::string>& lines)
{
    std::ifstream f(path);
    if (!f.is_open()) {
        std::fprintf(stderr, "File %s not found!\n", path.c_str());
        return false;
    }
    std::printf("Opening configuration file\n");
    std::string line;
    while (std::getline(f, line)) {
        // strip trailing CR from files edited on Windows
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        lines.push_back(line);
    }
    return true;
}

CliConfig parseCliCommands(const std::vector<std::string>& lines)
{
    std::printf("Parsing configuration file...\n");
    CliConfig cfg;
    cfg.lines = lines;
    const auto& supported = supportedCommands();

    for (const auto& line : lines) {
        std::vector<std::string> tokens = splitWhitespace(line);
        if (tokens.empty())
            continue;
        const std::string& command = tokens[0];
        if (command[0] == '%') // comment line
            continue;

        auto it = supported.find(command);
        if (it == supported.end()) {
            std::printf("Skip parsing for: %s command\n", command.c_str());
            continue;
        }
        const int numParams = static_cast<int>(tokens.size()) - 1;
        if (std::find(it->second.begin(), it->second.end(), numParams)
            == it->second.end()) {
            std::printf("Error: number of parameters is incorrect for %s command\n",
                        command.c_str());
            continue;
        }
        std::vector<double> values;
        values.reserve(tokens.size() - 1);
        for (size_t i = 1; i < tokens.size(); i++)
            values.push_back(std::stod(tokens[i]));
        cfg.params[command].push_back(std::move(values));
    }
    return cfg;
}

std::vector<ChirpParams> calculateChirpParams(const CliConfig& P)
{
    constexpr double c_speed_of_light = 3e8;
    constexpr double sec2usec = 1e6;
    constexpr double usec2msec = 1e-3;
    constexpr double KHz2Hz = 1e3;
    constexpr double MHz2Hz = 1e6;
    constexpr double GHz2Hz = 1e9;

    std::vector<ChirpParams> out;

    const int numTX = popcount32(static_cast<unsigned>(P.get("channelCfg", 0, 1)));
    const int numRX = popcount32(static_cast<unsigned>(P.get("channelCfg", 0, 0)));
    const bool advSfMode = (P.get("dfeDataOutputMode", 0, 0) == 3);

    const size_t numProfiles = P.rows("profileCfg");
    for (size_t i = 0; i < numProfiles; i++) {
        ChirpParams cp;
        cp.numTXChannel = numTX;
        cp.numRXChannel = numRX;
        cp.profileId = static_cast<int>(P.get("profileCfg", i, PC_PROFILE_ID));
        cp.startFreq_GHz = P.get("profileCfg", i, PC_START_FREQ);

        const double cliFreqScaleFactor = (cp.startFreq_GHz >= 76) ? 3.6 : 2.7; // 77GHz : 60GHz
        double freqSlope = P.get("profileCfg", i, PC_FREQ_SLOPE);
        // quantize slope the way the device CLI does (fix() truncates toward 0)
        freqSlope = std::trunc(freqSlope * static_cast<double>(1LL << 26) / cliFreqScaleFactor)
                    * (cliFreqScaleFactor / static_cast<double>(1LL << 26));

        const double idleTime = P.get("profileCfg", i, PC_IDLE_TIME);
        const double rampEndTime = P.get("profileCfg", i, PC_RAMP_END_TIME);
        cp.numADCSamples = static_cast<int>(P.get("profileCfg", i, PC_NUM_ADC_SAMPLES));
        cp.adcSamplingRate_ksps = P.get("profileCfg", i, PC_DIG_OUT_RATE);
        cp.adcSamplingTime_usec =
            (cp.numADCSamples / (cp.adcSamplingRate_ksps * KHz2Hz)) * sec2usec;
        cp.interChirpTime_usec = idleTime + (rampEndTime - cp.adcSamplingTime_usec);

        double numChirps = 0, numLoops = 0, framePeriod = 0;
        if (!advSfMode) {
            // normal frame mode: assume one profile per frame
            numChirps = P.get("frameCfg", i, FC_CHIRP_END)
                        - P.get("frameCfg", i, FC_CHIRP_START) + 1;
            numLoops = P.get("frameCfg", i, FC_NUM_LOOPS);
            framePeriod = P.get("frameCfg", i, FC_FRAME_PERIOD);
        } else {
            // advanced subframe mode: find the subframe using this profile
            int firstChirpIndex = -1;
            for (size_t k = 0; k < P.rows("chirpCfg"); k++) {
                if (P.get("chirpCfg", k, CC_PROFILE_ID) == cp.profileId) {
                    firstChirpIndex = static_cast<int>(k);
                    break;
                }
            }
            for (size_t k = 0; k < P.rows("subFrameCfg"); k++) {
                if (P.get("subFrameCfg", k, SFC_CHIRP_START_IDX) == firstChirpIndex) {
                    numChirps = P.get("subFrameCfg", k, SFC_NUM_CHIRPS);
                    numLoops = P.get("subFrameCfg", k, SFC_NUM_LOOPS);
                    framePeriod = P.get("subFrameCfg", k, SFC_SUBFRAME_PERIOD);
                    break;
                }
            }
        }

        cp.activeFrameTime_msec =
            numChirps * (idleTime + rampEndTime) * numLoops * usec2msec;
        cp.frameTime_msec = framePeriod;
        cp.dutyCycle_percent = (framePeriod > 0)
            ? (cp.activeFrameTime_msec / cp.frameTime_msec) * 100 : 0;

        cp.ttlBandwidth_MHz = rampEndTime * freqSlope;
        cp.validBandwidth_MHz = cp.adcSamplingTime_usec * freqSlope;
        cp.rangeResolution_m = c_speed_of_light / (2 * cp.validBandwidth_MHz * MHz2Hz);

        // assuming complex 1x output mode (only mode supported in SDK OOB)
        const double IFmax = 0.8 * cp.adcSamplingRate_ksps;
        cp.rangeMax_m = (IFmax * KHz2Hz) * c_speed_of_light
                        / (2 * (freqSlope * MHz2Hz * sec2usec));

        const double wavelength = c_speed_of_light / (cp.startFreq_GHz * GHz2Hz);
        cp.velResolution_mps = wavelength
            / (2 * numLoops * numTX * (idleTime + rampEndTime) / sec2usec);
        cp.velMax_mps = wavelength / (4 * numTX * (idleTime + rampEndTime) / sec2usec);

        cp.numDopplerBins = std::max(1 << static_cast<int>(std::ceil(std::log2(numLoops))), 8);
        cp.numRangeBins = 1 << static_cast<int>(std::ceil(std::log2(cp.numADCSamples)));

        const double bytesPerSample = 4;
        cp.radarCubeSz_KB = bytesPerSample * cp.numADCSamples * numChirps
                            * numLoops * numRX / 1024;
        out.push_back(cp);
    }
    return out;
}

void printChirpParams(const std::vector<ChirpParams>& params)
{
    for (const auto& p : params) {
        std::printf("Profile %d derived parameters:\n", p.profileId);
        std::printf("  Start frequency:      %.2f GHz\n", p.startFreq_GHz);
        std::printf("  TX / RX channels:     %d / %d\n", p.numTXChannel, p.numRXChannel);
        std::printf("  Range resolution:     %.4f m\n", p.rangeResolution_m);
        std::printf("  Max range:            %.2f m\n", p.rangeMax_m);
        std::printf("  Velocity resolution:  %.4f m/s\n", p.velResolution_mps);
        std::printf("  Max velocity:         %.2f m/s\n", p.velMax_mps);
        std::printf("  Frame period:         %.1f ms (duty cycle %.1f%%)\n",
                    p.frameTime_msec, p.dutyCycle_percent);
        std::printf("  Range / Doppler bins: %d / %d\n", p.numRangeBins, p.numDopplerBins);
        std::printf("  Radar cube size:      %.1f KB\n", p.radarCubeSz_KB);
    }
}
