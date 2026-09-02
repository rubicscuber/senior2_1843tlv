#include "frame_parser.h"

#include <cctype>
#include <cmath>
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
    constexpr size_t HEADER_LEN = 8 + 8 * 4;
    valid = false;
    if (pktLen < HEADER_LEN) {
        std::printf("Issue with frame. Skipping. Cannot parse header. Missing bytes.\n");
        return 0;
    }
    std::memcpy(hdr.magicWord, pkt, 8);
    hdr.version        = readU32(pkt + 8);
    hdr.packetLength   = readU32(pkt + 12);
    hdr.platform       = readU32(pkt + 16);
    hdr.frameNumber    = readU32(pkt + 20);
    hdr.timestamp      = readU32(pkt + 24);
    hdr.numDetectedObj = readU32(pkt + 28);
    hdr.numTLVs        = readU32(pkt + 32);
    hdr.subFrameNumber = readU32(pkt + 36);

    valid = (hdr.packetLength == pktLen);
    if (!valid) {
        std::printf("Issue with Frame %u. Skipping. Expected packet length: %u; "
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
    // a frame is complete only when the next frame's magic word has arrived
    numFramesAvailable = static_cast<int>(magicIdx.size()) - 1;
    if (numFramesAvailable <= 0) {
        numFramesAvailable = 0;
        return frames;
    }

    const size_t numToParse =
        (mode == ReadMode::FIFO) ? 1 : static_cast<size_t>(numFramesAvailable);
    for (size_t n = 0; n < numToParse; n++) {
        const size_t start = magicIdx[n];
        const size_t end = magicIdx[n + 1];
        frames.push_back(parseFramePacket(buffer.data() + start, end - start));
    }

    // keep unconsumed bytes (the trailing, still-incomplete frame)
    buffer.erase(buffer.begin(), buffer.begin() + magicIdx[numToParse]);
    if (buffer.size() > BYTES_BUFFER_MAX_SIZE)
        buffer.clear();
    return frames;
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
