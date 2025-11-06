#include <openpose/hand/renderHand.hpp>
#include <openpose/hand/handParameters.hpp>
#include <openpose/utilities/fastMath.hpp>
#include <openpose/utilities/keypoint.hpp>
#include <set>

namespace op
{
    // Helper: determine if a hand pair is palm-to-knuckle (wrist to MCP) or fingertip-to-knuckle
    // Returns: true if palm-to-knuckle (green), false if fingertip-to-knuckle (white)
    inline bool isPalmToKnucklePair(const unsigned int partA, const unsigned int partB)
    {
        // Palm to knuckle connections: wrist (0) to MCP joints (1, 5, 9, 13, 17)
        static const std::set<unsigned int> mcpJoints{1, 5, 9, 13, 17};
        return (partA == 0 && mcpJoints.count(partB)) || (partB == 0 && mcpJoints.count(partA));
    }

    void renderHandKeypointsCpu(Array<float>& frameArray, const std::array<Array<float>, 2>& handKeypoints,
                                const float renderThreshold)
    {
        try
        {
            // Parameters
            const auto thicknessCircleRatio = 1.f/50.f;
            const auto thicknessLineRatioWRTCircle = 0.75f;
            const auto& pairs = HAND_PAIRS_RENDER;
            const auto& scales = HAND_SCALES_RENDER;

            // Build per-pair color array
            std::vector<float> colorsLinesPerPair;
            colorsLinesPerPair.reserve((pairs.size() / 2) * 3);
            for (auto pairIdx = 0u; pairIdx < pairs.size(); pairIdx += 2)
            {
                float r, g, b;
                if (isPalmToKnucklePair(pairs[pairIdx], pairs[pairIdx+1]))
                {
                    // Green for palm to knuckle
                    r = 0.f; g = 255.f; b = 0.f;
                }
                else
                {
                    // White for fingertip to knuckle (all finger segments)
                    r = 255.f; g = 255.f; b = 255.f;
                }
                colorsLinesPerPair.push_back(b); // BGR order
                colorsLinesPerPair.push_back(g);
                colorsLinesPerPair.push_back(r);
            }

            // Point color: orange RGB(239,106,17)
            const std::vector<float> colorsPoints{239.f, 106.f, 17.f};

            // Custom hand styling:
            // - Points: opaque orange circles
            // - Lines: dotted (palm-to-knuckle=green, fingertip-to-knuckle=white)
            const float circleScale = 1.0f; // normal size
            const bool forceFilledCircles = true; // opaque filled
            const bool dashedLines = true; // dotted
            const int dotLenPx = 3; // small dots
            const int gapLenPx = 3; // small gaps
            const int thinLinePx = 2; // slightly thicker than face

            // Render both hands
            if (!frameArray.empty())
                renderKeypointsCpuCustomPerPair(
                    frameArray, handKeypoints[0], pairs,
                    colorsPoints, colorsLinesPerPair,
                    thicknessCircleRatio, thicknessLineRatioWRTCircle, scales, renderThreshold,
                    circleScale, forceFilledCircles,
                    dashedLines, dotLenPx, gapLenPx, thinLinePx);
            if (!frameArray.empty())
                renderKeypointsCpuCustomPerPair(
                    frameArray, handKeypoints[1], pairs,
                    colorsPoints, colorsLinesPerPair,
                    thicknessCircleRatio, thicknessLineRatioWRTCircle, scales, renderThreshold,
                    circleScale, forceFilledCircles,
                    dashedLines, dotLenPx, gapLenPx, thinLinePx);
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }
}
