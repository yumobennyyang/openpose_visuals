#include <openpose/pose/renderPose.hpp>
#include <openpose/pose/poseParameters.hpp>
#include <openpose/utilities/fastMath.hpp>
#include <openpose/utilities/keypoint.hpp>
#include <set>

namespace op
{
    // Helper: determine if a body part is left, right, or center
    // Returns: -1=left, 0=center, 1=right
    inline int getBodyPartSide(const unsigned int partIndex, const PoseModel poseModel)
    {
        // BODY_25 mapping
        if (poseModel == PoseModel::BODY_25 || poseModel == PoseModel::BODY_25D || poseModel == PoseModel::BODY_25E)
        {
            // Left parts: 5(LShoulder), 6(LElbow), 7(LWrist), 12(LHip), 13(LKnee), 14(LAnkle), 19(LBigToe), 20(LSmallToe), 21(LHeel), 16(LEye), 18(LEar)
            // Right parts: 2(RShoulder), 3(RElbow), 4(RWrist), 9(RHip), 10(RKnee), 11(RAnkle), 22(RBigToe), 23(RSmallToe), 24(RHeel), 15(REye), 17(REar)
            // Center: 0(Nose), 1(Neck), 8(MidHip)
            static const std::set<unsigned int> leftParts{5,6,7,12,13,14,19,20,21,16,18};
            static const std::set<unsigned int> rightParts{2,3,4,9,10,11,22,23,24,15,17};
            if (leftParts.count(partIndex)) return -1;
            if (rightParts.count(partIndex)) return 1;
            return 0; // center
        }
        // COCO mapping
        else if (poseModel == PoseModel::COCO_18)
        {
            // Left: 5(LShoulder), 6(LElbow), 7(LWrist), 11(LHip), 12(LKnee), 13(LAnkle), 15(LEye), 17(LEar)
            // Right: 2(RShoulder), 3(RElbow), 4(RWrist), 8(RHip), 9(RKnee), 10(RAnkle), 14(REye), 16(REar)
            // Center: 0(Nose), 1(Neck)
            static const std::set<unsigned int> leftParts{5,6,7,11,12,13,15,17};
            static const std::set<unsigned int> rightParts{2,3,4,8,9,10,14,16};
            if (leftParts.count(partIndex)) return -1;
            if (rightParts.count(partIndex)) return 1;
            return 0;
        }
        // BODY_19 mapping
        else if (poseModel == PoseModel::BODY_19 || poseModel == PoseModel::BODY_19E || poseModel == PoseModel::BODY_19N || poseModel == PoseModel::BODY_19_X2)
        {
            // Similar to BODY_25 but without feet
            static const std::set<unsigned int> leftParts{5,6,7,12,13,14,16,18};
            static const std::set<unsigned int> rightParts{2,3,4,9,10,11,15,17};
            if (leftParts.count(partIndex)) return -1;
            if (rightParts.count(partIndex)) return 1;
            return 0;
        }
        // BODY_23 mapping
        else if (poseModel == PoseModel::BODY_23)
        {
            // Left: 4(LShoulder), 5(LElbow), 6(LWrist), 10(LHip), 11(LKnee), 12(LAnkle), 17(LBigToe), 18(LSmallToe), 19(LHeel), 14(LEye), 16(LEar)
            // Right: 1(RShoulder), 2(RElbow), 3(RWrist), 7(RHip), 8(RKnee), 9(RAnkle), 20(RBigToe), 21(RSmallToe), 22(RHeel), 13(REye), 15(REar)
            // Center: 0(Nose)
            static const std::set<unsigned int> leftParts{4,5,6,10,11,12,17,18,19,14,16};
            static const std::set<unsigned int> rightParts{1,2,3,7,8,9,20,21,22,13,15};
            if (leftParts.count(partIndex)) return -1;
            if (rightParts.count(partIndex)) return 1;
            return 0;
        }
        // Default: treat as center
        return 0;
    }

    // Helper: determine line color based on connected body parts
    // Returns RGB color: blue for left, red for right, white for center/mixed
    inline void getLineColorForPair(const unsigned int partA, const unsigned int partB, const PoseModel poseModel, float& r, float& g, float& b)
    {
        const int sideA = getBodyPartSide(partA, poseModel);
        const int sideB = getBodyPartSide(partB, poseModel);
        
        // If both parts are on the same side, use that side's color
        if (sideA == sideB)
        {
            if (sideA == -1) // Left - blue
            {
                r = 0.f; g = 0.f; b = 255.f;
            }
            else if (sideA == 1) // Right - red
            {
                r = 255.f; g = 0.f; b = 0.f;
            }
            else // Center - white
            {
                r = 255.f; g = 255.f; b = 255.f;
            }
        }
        else
        {
            // Mixed sides - white
            r = 255.f; g = 255.f; b = 255.f;
        }
    }

    void renderPoseKeypointsCpu(Array<float>& frameArray, const Array<float>& poseKeypoints, const PoseModel poseModel,
                                const float renderThreshold, const bool blendOriginalFrame)
    {
        try
        {
            if (!frameArray.empty())
            {
                // Background
                if (!blendOriginalFrame)
                    frameArray.getCvMat().setTo(0.f); // [0-255]

                // Parameters
                const auto thicknessCircleRatio = 1.f/75.f;
                const auto thicknessLineRatioWRTCircle = 0.75f;
                const auto& pairs = getPoseBodyPartPairsRender(poseModel);
                const auto& poseScales = getPoseScales(poseModel);

                // Build per-pair color array: left=blue, right=red, center/mixed=white
                std::vector<float> colorsLinesPerPair;
                colorsLinesPerPair.reserve((pairs.size() / 2) * 3);
                for (auto pairIdx = 0u; pairIdx < pairs.size(); pairIdx += 2)
                {
                    float r, g, b;
                    getLineColorForPair(pairs[pairIdx], pairs[pairIdx+1], poseModel, r, g, b);
                    colorsLinesPerPair.push_back(b); // BGR order
                    colorsLinesPerPair.push_back(g);
                    colorsLinesPerPair.push_back(r);
                }

                // Point color: orange RGB(239,106,17)
                const std::vector<float> colorsPoints{239.f, 106.f, 17.f};

                // Custom pose styling:
                // - Points: opaque orange circles
                // - Lines: dotted (left=blue, right=red, center=white)
                const float circleScale = 0.85f; // normal size
                const bool forceFilledCircles = true; // opaque filled
                const bool dashedLines = true; // dotted
                const int dotLenPx = 6; // small dots
                const int gapLenPx = 6; // small gaps
                const int thinLinePx = 1; // slightly thicker than face

                renderKeypointsCpuCustomPerPair(
                    frameArray, poseKeypoints, pairs,
                    colorsPoints, colorsLinesPerPair,
                    thicknessCircleRatio, thicknessLineRatioWRTCircle, poseScales, renderThreshold,
                    circleScale, forceFilledCircles,
                    dashedLines, dotLenPx, gapLenPx, thinLinePx);
            }
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }
}
