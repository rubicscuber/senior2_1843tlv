// UART byte stream / .dat file parsing into TM frames.
// C++ port of parseBytes_TM.m, getTLV.m, getGtrackFrameHeader.m,
// getGtrackPtCloud.m, getGtrackTargetList.m, getGtrackPtType.m and
// readDATFile2Buffer.m.
#ifndef FRAME_PARSER_H
#define FRAME_PARSER_H

#include <cstdint>
#include <string>
#include <vector>

#include "tm_types.h"

// Frames are only counted once the *next* magic word is seen, so the byte
// buffer always retains the (possibly incomplete) trailing frame.
enum class ReadMode {
    FIFO, // parse only the oldest complete frame (real-time mode)
    ALL   // parse every complete frame (playback mode)
};

constexpr size_t BYTES_BUFFER_MAX_SIZE = 1u << 16;

// Scan `buffer` for complete frames delimited by the 8-byte magic word.
// Parsed frames are removed from the buffer; the trailing partial frame is
// kept for the next call. numFramesAvailable reports how many complete
// frames were present before parsing.
std::vector<Frame> parseBytesTM(std::vector<uint8_t>& buffer, ReadMode mode,
                                int& numFramesAvailable);

// Load a recorded UART stream into a byte buffer. Detects the format
// automatically: ASCII hex ("02 01 04 03 ...", as written by the MATLAB and
// C++ recorders) or raw binary. Returns false if the file cannot be read.
bool readDatFileToBuffer(const std::string& path, std::vector<uint8_t>& buffer);

#endif // FRAME_PARSER_H
