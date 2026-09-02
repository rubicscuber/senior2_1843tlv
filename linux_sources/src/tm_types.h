// Common data types for the Traffic Monitoring terminal visualizer.
// C++ port of the data structures used by the MATLAB TM visualizer
// (tm_visualizer.m and its helper functions).
#ifndef TM_TYPES_H
#define TM_TYPES_H

#include <cstdint>
#include <string>
#include <vector>

// Frame header preceding every UART packet (see getGtrackFrameHeader.m).
// All fields are little-endian uint32 on the wire; magicWord is 8 bytes.
struct FrameHeader {
    uint8_t  magicWord[8] = {0};
    uint32_t version = 0;
    uint32_t packetLength = 0;
    uint32_t platform = 0;
    uint32_t frameNumber = 0;
    uint32_t timestamp = 0;
    uint32_t numDetectedObj = 0;
    uint32_t numTLVs = 0;
    uint32_t subFrameNumber = 0;
};

// Detected point cloud, TLV type 1000 (see getGtrackPtCloud.m).
// Wire format per point: float range (m), azimuth (rad), elevation (rad),
// doppler (m/s). Stored here already converted to cartesian coordinates.
struct PointCloud {
    uint32_t numDetectedObj = 0;
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;
    std::vector<float> doppler;

    bool empty() const { return numDetectedObj == 0; }
};

// Tracked target list, TLV type 1010 (see getGtrackTargetList.m).
// Wire format per target: 112 bytes; uint32 tid followed by 9 floats
// (posX posY posZ velX velY velZ accX accY accZ), remainder is the error
// covariance / gating info which the visualizer does not use.
struct TargetList {
    std::vector<uint32_t> tid;
    std::vector<float> posX, posY, posZ;
    std::vector<float> velX, velY, velZ;
    std::vector<float> accX, accY, accZ;

    bool empty() const { return tid.empty(); }
    size_t size() const { return tid.size(); }
};

// One fully parsed frame (see parseBytes_TM.m).
struct Frame {
    FrameHeader header;
    bool valid = false;
    bool havePointCloud = false;
    bool haveTargetList = false;
    PointCloud detObj;
    TargetList targets;
    std::vector<uint8_t> pointType; // TLV 1011: track index per point
};

// Sensor mounting offsets (GUI "offset" inputs in setup_tm).
struct Offset {
    double height = 0.0; // sensor height above ground, m
    double az = 0.0;     // azimuth tilt, degrees
    double el = 0.0;     // elevation tilt, degrees
};

// Optional traffic-lane definition (GUI "lanes" inputs in setup_tm /
// initLanes.m). Lanes are numLanes adjacent rectangles starting at (x, y),
// each w wide (X direction) and h deep (Y direction).
struct Lanes {
    bool enable = false;
    int numLanes = 0;
    double x = 0.0;
    double y = 0.0;
    double w = 0.0;
    double h = 0.0;
};

#endif // TM_TYPES_H
