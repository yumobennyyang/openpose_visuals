#include <openpose/face/renderFace.hpp>
#include <openpose/face/faceParameters.hpp>
#include <openpose/utilities/fastMath.hpp>
#include <openpose/utilities/keypoint.hpp>

namespace op
{
    void renderFaceKeypointsCpu(Array<float>& frameArray, const Array<float>& faceKeypoints,
                                const float renderThreshold)
    {
        try
        {
            if (!frameArray.empty())
            {
                // Parameters
                const auto thicknessCircleRatio = 1.f/75.f;
                const auto thicknessLineRatioWRTCircle = 0.334f;
                const auto& pairs = FACE_PAIRS_RENDER;
                const auto& scales = FACE_SCALES_RENDER;

                // Custom face styling:
                // - Points: opaque orange RGB(239,106,17)
                // - Smaller circles
                // - Lines: thin white dashed
                const std::vector<float> facePointColors {239.f, 106.f, 17.f};
                const std::vector<float> faceLineColors  {255.f, 255.f, 255.f};
                const float circleScale = 0.75f; // slightly smaller
                const bool forceFilledCircles = true; // opaque filled
                const bool dashedLines = true;
                const int dashLenPx = 6;
                const int gapLenPx = 6;
                const int thinLinePx = 1; // thin

                renderKeypointsCpuCustom(
                    frameArray, faceKeypoints, pairs,
                    facePointColors, faceLineColors,
                    thicknessCircleRatio, thicknessLineRatioWRTCircle, scales, renderThreshold,
                    circleScale, forceFilledCircles,
                    dashedLines, dashLenPx, gapLenPx, thinLinePx);
            }
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }
}
