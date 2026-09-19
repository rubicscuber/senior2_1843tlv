#include "frame_parser.h"

#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <fstream>

namespace {

// UART frame packet start marker (parseBytes_TM.m)
const uint8_t MAGIC_WORD[8] = {2, 1, 4, 3, 6, 5, 8, 7};

// TLV message types (parseBytes_TM.m)
constexpr uint32_t UART_MSG_TRACKERPROC_DETECTED_POINTS = 1000;
constexpr uint32_t UART_MSG_TRACKERPROC_TARGET_LIST = 1010;
constexpr uint32_t UART_MSG_TRACKERPROC_TARGET_INDEX = 1011;

// frame header: magic word + 8 uint32 fields (getGtrackFrameHeader.m)
constexpr size_t HEADER_LEN = 8 + 8 * 4;
constexpr size_t PACKET_LENGTH_OFFSET = 12;

bool g_quiet = false;

void diag(const char* fmt, ...)
{
    if (g_quiet)
        return;
    va_list args;
    va_start(args, fmt);
    std::vfprintf(stderr, fmt, args);
    va_end(args);
}

uint32_t readU32(const uint8_t* p)
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8)
         | (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

float readF32(const uint8_t* p)
{
    float f;
    std::memcpy(&f, p, sizeof(f));
    return f;
}

std::vector<size_t> findMagicWords(const std::vector<uint8_t>& buf)
{
    std::vector<size_t> idx;
    if (buf.size() < sizeof(MAGIC_WORD))
        return idx;
    for (size_t i = 0; i + sizeof(MAGIC_WORD) <= buf.size(); i++) {
        if (std::memcmp(buf.data() + i, MAGIC_WORD, sizeof(MAGIC_WORD)) == 0)
            idx.push_back(i);
    }
    return idx;
}

// Parse the 40-byte frame header (getGtrackFrameHeader.m). Returns the
// number of header bytes consumed, or 0 if the packet is too short.
size_t parseHeader(const uint8_t* pkt, size_t pktLen, FrameHeader& hdr, bool& valid)
{
    valid = false;
    if (pktLen < HEADER_LEN) {
        diag("Issue with frame. Skipping. Cannot parse header. Missing bytes.\n");
        return 0;
    }
    std::memcpy(hdr.magicWord, pkt, 8);
    hdr.version        = readU32(pkt + 8);
    hdr.packetLength   = readU32(pkt + PACKET_LENGTH_OFFSET);
    hdr.platform       = readU32(pkt + 16);
    hdr.frameNumber    = readU32(pkt + 20);
    hdr.timestamp      = readU32(pkt + 24);
    hdr.numDetectedObj = readU32(pkt + 28);
    hdr.numTLVs        = readU32(pkt + 32);
    hdr.subFrameNumber = readU32(pkt + 36);

    valid = (hdr.packetLength == pktLen);
    if (!valid) {
        diag("Issue with Frame %u. Skipping. Expected packet length: %u; "
             "Actual length: %zu.\n",
             hdr.frameNumber, hdr.packetLength, pktLen);
    }
    return HEADER_LEN;
}

// TLV 1000: point cloud in spherical coordinates -> cartesian (getGtrackPtCloud.m)
PointCloud parsePtCloud(const uint8_t* payload, size_t len)
{
    constexpr size_t POINT_LEN = 4 * 4; // range, azimuth, elevation, doppler
    PointCloud pc;
    pc.numDetectedObj = static_cast<uint32_t>(len / POINT_LEN);
    for (uint32_t n = 0; n < pc.numDetectedObj; n++) {
        const uint8_t* p = payload + n * POINT_LEN;
        const float range = readF32(p);
        const float azimuth = readF32(p + 4);
        const float elev = readF32(p + 8);
        const float doppler = readF32(p + 12);
        const float rGround = range * std::cos(elev);
        pc.z.push_back(range * std::sin(elev));
        pc.y.push_back(rGround * std::cos(azimuth));
        pc.x.push_back(rGround * std::sin(azimuth));
        pc.doppler.push_back(doppler);
    }
    return pc;
}

// TLV 1010: tracked target list (getGtrackTargetList.m)
TargetList parseTargetList(const uint8_t* payload, size_t len)
{
    constexpr size_t TARGET_LEN = 28 * 4; // 112 bytes per target on the wire
    TargetList t;
    const size_t numTargets = len / TARGET_LEN;
    for (size_t n = 0; n < numTargets; n++) {
        const uint8_t* p = payload + n * TARGET_LEN;
        t.tid.push_back(readU32(p));
        t.posX.push_back(readF32(p + 4));
        t.posY.push_back(readF32(p + 8));
        t.posZ.push_back(readF32(p + 12));
        t.velX.push_back(readF32(p + 16));
        t.velY.push_back(readF32(p + 20));
        t.velZ.push_back(readF32(p + 24));
        t.accX.push_back(readF32(p + 28));
        t.accY.push_back(readF32(p + 32));
        t.accZ.push_back(readF32(p + 36));
        // remaining 72 bytes: error covariance + gating gain (unused)
    }
    return t;
}

Frame parseFramePacket(const uint8_t* pkt, size_t pktLen)
{
    Frame frame;
    size_t idx = parseHeader(pkt, pktLen, frame.header, frame.valid);
    if (!frame.valid)
        return frame;

    for (uint32_t i = 0; i < frame.header.numTLVs; i++) {
        // TLV header: uint32 type, uint32 payload length (getTLV.m)
        if (idx + 8 > pktLen)
            break;
        const uint32_t type = readU32(pkt + idx);
        const uint32_t length = readU32(pkt + idx + 4);
        idx += 8;
        if (idx + length > pktLen)
            break;
        const uint8_t* payload = pkt + idx;

        switch (type) {
        case UART_MSG_TRACKERPROC_DETECTED_POINTS:
            frame.detObj = parsePtCloud(payload, length);
            frame.havePointCloud = true;
            break;
        case UART_MSG_TRACKERPROC_TARGET_LIST:
            frame.targets = parseTargetList(payload, length);
            frame.haveTargetList = true;
            break;
        case UART_MSG_TRACKERPROC_TARGET_INDEX:
            frame.pointType.assign(payload, payload + length);
            break;
        default:
            break; // unknown TLV: skip payload
        }
        idx += length;
    }
    return frame;
}

} // namespace

std::vector<Frame> parseBytesTM(std::vector<uint8_t>& buffer, ReadMode mode,
                                int& numFramesAvailable)
{
    std::vector<Frame> frames;
    const std::vector<size_t> magicIdx = findMagicWords(buffer);

    // Frame n spans [magicIdx[n], ends[n]). A frame ends where the next magic
    // word starts; the trailing frame, which has no next magic word yet, is
    // complete as soon as the packetLength bytes announced by its own header
    // have arrived, so live mode does not wait one frame period for the next
    // frame before it can react. A trailing frame whose header claims more
    // bytes than are present (or an implausible length) stays incomplete
    // until the next magic word resolves it, as before.
    std::vector<size_t> ends;
    if (!magicIdx.empty()) {
        ends.assign(magicIdx.begin() + 1, magicIdx.end());
        const size_t start = magicIdx.back();
        const size_t avail = buffer.size() - start;
        if (avail >= HEADER_LEN) {
            const uint32_t packetLength = readU32(buffer.data() + start + PACKET_LENGTH_OFFSET);
            if (packetLength >= HEADER_LEN && packetLength <= avail)
                ends.push_back(start + packetLength);
        }
    }
    numFramesAvailable = static_cast<int>(ends.size());
    if (numFramesAvailable == 0) {
        // No complete frame. If the buffer has nevertheless reached its limit
        // (a stream without frame starts, or a corrupt oversized frame), drop
        // the dead bytes so the caller can keep reading instead of stalling
        // or growing without bound.
        if (buffer.size() >= BYTES_BUFFER_MAX_SIZE) {
            const size_t keepFrom = magicIdx.empty() ? buffer.size() : magicIdx.front();
            if (keepFrom == 0 || buffer.size() - keepFrom >= BYTES_BUFFER_MAX_SIZE) {
                diag("Discarding %zu buffered bytes: no complete frame found.\n",
                     buffer.size());
                buffer.clear();
            } else {
                diag("Discarding %zu bytes before the next frame start.\n", keepFrom);
                buffer.erase(buffer.begin(), buffer.begin() + keepFrom);
            }
        }
        return frames;
    }

    const size_t numToParse = (mode == ReadMode::FIFO) ? 1 : ends.size();
    for (size_t n = 0; n < numToParse; n++) {
        const size_t start = magicIdx[n];
        frames.push_back(parseFramePacket(buffer.data() + start, ends[n] - start));
    }

    // keep unconsumed bytes (later frames and the still-incomplete trailing one)
    buffer.erase(buffer.begin(), buffer.begin() + ends[numToParse - 1]);
    if (buffer.size() >= BYTES_BUFFER_MAX_SIZE) {
        diag("Discarding %zu buffered bytes: oversized partial frame.\n", buffer.size());
        buffer.clear();
    }
    return frames;
}

void setFrameParserQuiet(bool quiet)
{
    g_quiet = quiet;
}

bool readDatFileToBuffer(const std::string& path, std::vector<uint8_t>& buffer)
{
    std::ifstream f(path, std::ios::binary);
    if (!f.is_open())
        return false;
    std::vector<char> raw((std::istreambuf_iterator<char>(f)),
                          std::istreambuf_iterator<char>());
    if (raw.empty())
        return false;

    // Detect ASCII hex format: only hex digits and whitespace in the first chunk
    bool isHexText = true;
    const size_t probeLen = std::min<size_t>(raw.size(), 256);
    for (size_t i = 0; i < probeLen; i++) {
        const unsigned char c = static_cast<unsigned char>(raw[i]);
        if (!std::isxdigit(c) && !std::isspace(c)) {
            isHexText = false;
            break;
        }
    }

    buffer.clear();
    if (isHexText) {
        unsigned value;
        int nibbles = 0;
        value = 0;
        for (char ch : raw) {
            const unsigned char c = static_cast<unsigned char>(ch);
            if (std::isxdigit(c)) {
                value = value * 16
                      + static_cast<unsigned>(std::isdigit(c) ? c - '0'
                                                              : std::tolower(c) - 'a' + 10);
                if (++nibbles == 2) {
                    buffer.push_back(static_cast<uint8_t>(value));
                    nibbles = 0;
                    value = 0;
                }
            } else if (nibbles) { // lone nibble before separator
                buffer.push_back(static_cast<uint8_t>(value));
                nibbles = 0;
                value = 0;
            }
        }
        if (nibbles)
            buffer.push_back(static_cast<uint8_t>(value));
    } else {
        buffer.assign(raw.begin(), raw.end());
    }
    return !buffer.empty();
}
