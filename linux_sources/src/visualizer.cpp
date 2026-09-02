#include "visualizer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <sstream>

#include <sys/ioctl.h>
#include <unistd.h>

namespace {

constexpr double DEG2RAD = M_PI / 180.0;

// color codes used in the color grid
enum : uint8_t { COL_NONE = 0, COL_DIM = 1, COL_LANE = 2, COL_POINT = 3, COL_TARGET = 4 };

const char* ansiFor(uint8_t col)
{
    switch (col) {
    case COL_DIM:    return "\033[90m";       // gray: FOV guide lines
    case COL_LANE:   return "\033[31m";       // red: lanes (as in initLanes call)
    case COL_POINT:  return "\033[34m";       // blue: point cloud (stylePtCloud)
    case COL_TARGET: return "\033[1;33m";     // bold yellow: tracked objects
    default:         return "";
    }
}

void terminalSize(int& rows, int& cols)
{
    winsize ws{};
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0 && ws.ws_row > 0 && ws.ws_col > 0) {
        rows = ws.ws_row;
        cols = ws.ws_col;
    } else {
        rows = 24;
        cols = 80;
    }
}

// Apply the mounting-offset transform of translatePoints.m to one point.
void translatePoint(double& x, double& y, double& z, const Offset& offset)
{
    const double ca = std::cos(offset.az * DEG2RAD), sa = std::sin(offset.az * DEG2RAD);
    const double ce = std::cos(offset.el * DEG2RAD), se = std::sin(offset.el * DEG2RAD);
    // rotMat_az * rotMat_el * [x;y;z]
    const double ex = x;
    const double ey = ce * y - se * z;
    const double ez = se * y + ce * z;
    x = ca * ex - sa * ey;
    y = sa * ex + ca * ey;
    z = ez + offset.height;
}

const char* viewName(View v)
{
    switch (v) {
    case View::XY: return "X-Y";
    case View::YZ: return "Y-Z";
    case View::XZ: return "X-Z";
    }
    return "?";
}

} // namespace

TerminalViz::TerminalViz(double maxRange, double azFOV, double elFOV,
                         const Offset& offset, const Lanes& lanes)
    : lanes_(lanes)
{
    // FOV guide line endpoints as drawn by drawFOVLines.m
    const double sAz = std::sin(azFOV / 2 * DEG2RAD) * maxRange;
    const double cAz = std::cos(azFOV / 2 * DEG2RAD) * maxRange;
    const double sEl = std::sin(elFOV / 2 * DEG2RAD) * maxRange;
    const double cEl = std::cos(elFOV / 2 * DEG2RAD) * maxRange;

    // segments as {x0,y0,z0, x1,y1,z1}, all starting at the sensor origin
    std::vector<std::array<double, 6>> segs = {
        {0, 0, 0, 0, maxRange, 0},   // boresight
        {0, 0, 0, -sAz, cAz, 0},     // azimuth FOV edges
        {0, 0, 0, sAz, cAz, 0},
        {0, 0, 0, 0, cEl, -sEl},     // elevation FOV edges
        {0, 0, 0, 0, cEl, sEl},
    };

    double azX[2] = {0, 0}; // extent of azimuth-line X data after transform
    double maxY = 0, maxZ = 0;
    for (size_t s = 0; s < segs.size(); s++) {
        for (int e = 0; e < 2; e++) {
            double x = segs[s][e * 3], y = segs[s][e * 3 + 1], z = segs[s][e * 3 + 2];
            translatePoint(x, y, z, offset);
            segs[s][e * 3] = x;
            segs[s][e * 3 + 1] = y;
            segs[s][e * 3 + 2] = z;
            if (s == 1 || s == 2) {
                azX[0] = std::min(azX[0], x);
                azX[1] = std::max(azX[1], x);
            }
            maxY = std::max(maxY, y);
            maxZ = std::max(maxZ, z);
        }
    }
    fovSegments_ = std::move(segs);

    // axis limits as set in tm_visualizer.m after drawing the FOV lines
    xlim_[0] = azX[0];
    xlim_[1] = azX[1];
    ylim_[0] = 0;
    ylim_[1] = std::max(maxY, 1.0);
    zlim_[0] = 0;
    zlim_[1] = std::max({maxZ, offset.height, 1.0});
}

void TerminalViz::plotPoint(std::vector<std::string>& grid,
                            std::vector<std::vector<uint8_t>>& color,
                            double x, double y, double z, char c, uint8_t col) const
{
    double h = 0, v = 0;
    const double *hlim = nullptr, *vlim = nullptr;
    switch (view_) {
    case View::XY: h = x; v = y; hlim = xlim_; vlim = ylim_; break;
    case View::YZ: h = y; v = z; hlim = ylim_; vlim = zlim_; break;
    case View::XZ: h = x; v = z; hlim = xlim_; vlim = zlim_; break;
    }
    if (h < hlim[0] || h > hlim[1] || v < vlim[0] || v > vlim[1])
        return;
    const int cIdx = static_cast<int>((h - hlim[0]) / (hlim[1] - hlim[0]) * (plotCols_ - 1) + 0.5);
    const int rIdx = static_cast<int>((vlim[1] - v) / (vlim[1] - vlim[0]) * (plotRows_ - 1) + 0.5);
    if (rIdx < 0 || rIdx >= plotRows_ || cIdx < 0 || cIdx >= plotCols_)
        return;
    // higher color codes (points, targets) win over guide lines
    if (col >= color[rIdx][cIdx]) {
        grid[rIdx][cIdx] = c;
        color[rIdx][cIdx] = col;
    }
}

void TerminalViz::plotSegment(std::vector<std::string>& grid,
                              std::vector<std::vector<uint8_t>>& color,
                              const double a[3], const double b[3],
                              char c, uint8_t col) const
{
    const int steps = 2 * std::max(plotCols_, plotRows_);
    for (int i = 0; i <= steps; i++) {
        const double t = static_cast<double>(i) / steps;
        plotPoint(grid, color,
                  a[0] + (b[0] - a[0]) * t,
                  a[1] + (b[1] - a[1]) * t,
                  a[2] + (b[2] - a[2]) * t, c, col);
    }
}

void TerminalViz::render(const RenderData& d)
{
    int termRows, termCols;
    terminalSize(termRows, termCols);
    // reserve rows: title, stats, target list, key help + plot border (2)
    plotCols_ = std::max(20, std::min(termCols - 2, 158));
    plotRows_ = std::max(8, termRows - 7);

    std::vector<std::string> grid(plotRows_, std::string(plotCols_, ' '));
    std::vector<std::vector<uint8_t>> color(plotRows_,
                                            std::vector<uint8_t>(plotCols_, COL_NONE));

    // FOV guide lines (approximate guidelines only, as in the GUI)
    for (const auto& s : fovSegments_) {
        const double a[3] = {s[0], s[1], s[2]};
        const double b[3] = {s[3], s[4], s[5]};
        plotSegment(grid, color, a, b, '.', COL_DIM);
    }

    // lanes (X-Y view only, like the patch objects from initLanes.m)
    if (lanes_.enable && view_ == View::XY) {
        for (int i = 0; i <= lanes_.numLanes; i++) {
            const double xv = lanes_.x + lanes_.w * i;
            const double a[3] = {xv, lanes_.y, 0};
            const double b[3] = {xv, lanes_.y + lanes_.h, 0};
            plotSegment(grid, color, a, b, '|', COL_LANE);
        }
        const double xEnd = lanes_.x + lanes_.w * lanes_.numLanes;
        const double a0[3] = {lanes_.x, lanes_.y, 0};
        const double b0[3] = {xEnd, lanes_.y, 0};
        const double a1[3] = {lanes_.x, lanes_.y + lanes_.h, 0};
        const double b1[3] = {xEnd, lanes_.y + lanes_.h, 0};
        plotSegment(grid, color, a0, b0, '-', COL_LANE);
        plotSegment(grid, color, a1, b1, '-', COL_LANE);
    }

    // point cloud
    for (size_t i = 0; i < d.ptX.size(); i++)
        plotPoint(grid, color, d.ptX[i], d.ptY[i], d.ptZ[i], '*', COL_POINT);

    // tracked objects, labelled with the last digit of their track id
    for (size_t i = 0; i < d.tgtX.size(); i++) {
        const char label = static_cast<char>('0' + d.tgtId[i] % 10);
        plotPoint(grid, color, d.tgtX[i], d.tgtY[i], d.tgtZ[i], label, COL_TARGET);
    }

    // ---- compose the screen ----
    std::ostringstream out;
    if (firstRender_) {
        out << "\033[2J\033[?25l"; // clear screen, hide cursor
        firstRender_ = false;
    }
    out << "\033[H"; // cursor home

    out << "\033[1mTraffic Monitoring Visualizer (terminal)\033[0m"
        << "  view: " << viewName(view_);
    if (d.playback) {
        out << "  frame " << d.frameIndex << "/" << d.totalFrames
            << (d.paused ? "  \033[7m PAUSED \033[0m" : "  playing");
    }
    out << "\033[K\n";

    out << "Frame: " << d.frameNumber
        << " | Num Frames in Buffer: " << d.numFramesAvailable
        << " | Pt Cloud: ";
    if (d.numPoints >= 0)
        out << d.numPoints;
    out << " | Num Tracked Obj: ";
    if (d.numTargets >= 0)
        out << d.numTargets;
    if (!d.laneCounts.empty()) {
        out << " | Lanes:";
        for (size_t i = 0; i < d.laneCounts.size(); i++)
            out << " [" << (i + 1) << "] " << d.laneCounts[i];
    }
    out << "\033[K\n";

    out << '+' << std::string(plotCols_, '-') << "+\033[K\n";
    for (int r = 0; r < plotRows_; r++) {
        out << '|';
        uint8_t cur = COL_NONE;
        for (int c = 0; c < plotCols_; c++) {
            if (color[r][c] != cur) {
                out << "\033[0m" << ansiFor(color[r][c]);
                cur = color[r][c];
            }
            out << grid[r][c];
        }
        if (cur != COL_NONE)
            out << "\033[0m";
        out << "|\033[K\n";
    }
    out << '+' << std::string(plotCols_, '-') << "+\033[K\n";

    // target list line (replaces the GUI text labels next to each marker)
    out << "Targets:";
    for (size_t i = 0; i < d.tgtX.size() && i < 6; i++) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), " %u:(%.1f, %.1f, %.1f)m",
                      d.tgtId[i], d.tgtX[i], d.tgtY[i], d.tgtZ[i]);
        out << buf;
    }
    if (d.tgtX.size() > 6)
        out << " ...";
    out << "\033[K\n";

    out << "\033[90mkeys: 1=X-Y 2=Y-Z 3=X-Z";
    if (d.playback)
        out << "  space=play/pause  n=next  b=back";
    out << "  q=quit\033[0m\033[K";

    const std::string s = out.str();
    (void)!::write(STDOUT_FILENO, s.data(), s.size());
}

void TerminalViz::printStatsLine(const RenderData& d)
{
    std::printf("Frame: %u | Num Frames in Buffer: %d | Pt Cloud: %d | Num Tracked Obj: %d",
                d.frameNumber, d.numFramesAvailable,
                d.numPoints >= 0 ? d.numPoints : 0,
                d.numTargets >= 0 ? d.numTargets : 0);
    for (size_t i = 0; i < d.laneCounts.size(); i++)
        std::printf(" | Lane %zu: %d", i + 1, d.laneCounts[i]);
    for (size_t i = 0; i < d.tgtX.size(); i++)
        std::printf(" | tid %u (%.2f, %.2f, %.2f)",
                    d.tgtId[i], d.tgtX[i], d.tgtY[i], d.tgtZ[i]);
    std::printf("\n");
    std::fflush(stdout);
}
