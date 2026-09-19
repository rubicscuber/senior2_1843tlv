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

// A frame is complete once the *next* magic word is seen, or, for the last
// frame in the buffer, once the packetLength bytes announced by its header
// have all arrived; an incomplete trailing frame is retained in the buffer.
enum class ReadMode {
    FIFO, // parse only the oldest complete frame (real-time mode)
    ALL   // parse every complete frame (playback mode)
};

constexpr size_t BYTES_BUFFER_MAX_SIZE = 1u << 16;

// Scan `buffer` for complete frames delimited by the 8-byte magic word.
// Parsed frames are removed from the buffer; the trailing partial frame is
// kept for the next call. numFramesAvailable reports how many complete
// frames were present before parsing.
//
// The buffer can never grow or stall without bound: if it reaches
// BYTES_BUFFER_MAX_SIZE without containing a complete frame, everything
// before the last frame start is discarded (or everything, if there is no
// frame start or the partial frame is itself oversized).
std::vector<Frame> parseBytesTM(std::vector<uint8_t>& buffer, ReadMode mode,
                                int& numFramesAvailable);

// Suppress the parser's diagnostic messages (bad frame lengths, discarded
// bytes). They go to stderr; a full-screen display can turn them off and
// show the counts instead.
void setFrameParserQuiet(bool quiet);

// Load a recorded UART stream into a byte buffer. Detects the format
// automatically: ASCII hex ("02 01 04 03 ...", as written by the MATLAB and
// C++ recorders) or raw binary. Returns false if the file cannot be read.
bool readDatFileToBuffer(const std::string& path, std::vector<uint8_t>& buffer);

#endif // FRAME_PARSER_H
