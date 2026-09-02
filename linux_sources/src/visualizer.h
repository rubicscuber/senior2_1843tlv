// ANSI terminal renderer.
// Replaces the MATLAB figure created in tm_visualizer.m (init3DPlot_TM.m,
// drawFOVLines.m, initLanes.m, the stats annotation and the view popup menu)
// with an ASCII scatter plot drawn in the terminal.
#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "tm_types.h"

// Projection selected for the plot (the GUI's view popup: X-Y / Y-Z / X-Z;
// the "3D" option has no terminal equivalent).
enum class View { XY, YZ, XZ };

// Everything the renderer needs for one screen refresh. Coordinates must
// already be transformed to world coordinates (rotation + height offset).
struct RenderData {
    uint32_t frameNumber = 0;
    int numFramesAvailable = 0;
    int numPoints = -1;  // -1: no point-cloud TLV in this frame
    int numTargets = -1; // -1: no target-list TLV in this frame
    std::vector<double> ptX, ptY, ptZ;
    std::vector<double> tgtX, tgtY, tgtZ;
    std::vector<uint32_t> tgtId;
    std::vector<int> laneCounts;

    bool playback = false; // playback mode: show frame position and pause state
    bool paused = false;
    int frameIndex = 0;  // 1-based, playback only
    int totalFrames = 0; // playback only
};

class TerminalViz {
public:
    // Axis limits are derived from the sensor FOV guide lines exactly as in
    // tm_visualizer.m (drawFOVLines.m endpoints transformed by the mounting
    // offset). offset.az must already carry the CCW sign flip applied by main.
    TerminalViz(double maxRange, double azFOV, double elFOV,
                const Offset& offset, const Lanes& lanes);

    void setView(View v) { view_ = v; }
    View view() const { return view_; }

    // Redraw the whole screen for one frame.
    void render(const RenderData& d);

    // Print one plain-text stats line instead of a plot (--no-plot mode).
    static void printStatsLine(const RenderData& d);

private:
    void plotPoint(std::vector<std::string>& grid,
                   std::vector<std::vector<uint8_t>>& color,
                   double x, double y, double z, char c, uint8_t col) const;
    void plotSegment(std::vector<std::string>& grid,
                     std::vector<std::vector<uint8_t>>& color,
                     const double a[3], const double b[3], char c, uint8_t col) const;

    View view_ = View::XY;
    Lanes lanes_;
    double xlim_[2] = {0, 0};
    double ylim_[2] = {0, 0};
    double zlim_[2] = {0, 0};
    // FOV guide line segments in world coordinates, {x0,y0,z0,x1,y1,z1}
    std::vector<std::array<double, 6>> fovSegments_;
    int plotRows_ = 0;
    int plotCols_ = 0;
    bool firstRender_ = true;
};

#endif // VISUALIZER_H
