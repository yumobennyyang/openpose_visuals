#ifndef OPENPOSE_UTILITIES_KEYPOINT_HPP
#define OPENPOSE_UTILITIES_KEYPOINT_HPP

#include <openpose/core/common.hpp>

namespace op
{
    template <typename T>
    T getDistance(const Array<T>& keypoints, const int person, const int elementA, const int elementB);

    template <typename T>
    void averageKeypoints(Array<T>& keypointsA, const Array<T>& keypointsB, const int personA);

    template <typename T>
    void scaleKeypoints(Array<T>& keypoints, const T scale);

    template <typename T>
    void scaleKeypoints2d(Array<T>& keypoints, const T scaleX, const T scaleY);

    template <typename T>
    void scaleKeypoints2d(Array<T>& keypoints, const T scaleX, const T scaleY, const T offsetX, const T offsetY);

    template <typename T>
    void renderKeypointsCpu(
        Array<T>& frameArray, const Array<T>& keypoints, const std::vector<unsigned int>& pairs,
        const std::vector<T> colors, const T thicknessCircleRatio, const T thicknessLineRatioWRTCircle,
        const std::vector<T>& poseScales, const T threshold);

    // Customizable variant allowing distinct colors for points vs. lines and dashed/filled options
    template <typename T>
    void renderKeypointsCpuCustom(
        Array<T>& frameArray, const Array<T>& keypoints, const std::vector<unsigned int>& pairs,
        const std::vector<T> colorsPoints, const std::vector<T> colorsLines,
        const T thicknessCircleRatio, const T thicknessLineRatioWRTCircle,
        const std::vector<T>& poseScales, const T threshold,
        const T circleScale, const bool forceFilledCircles,
        const bool dashedLines, const int dashLenPx, const int gapLenPx, const int thinLinePx);

    // Variant with per-pair line colors (colorsLinesPerPair has 3*numPairs elements: RGB for each pair)
    template <typename T>
    void renderKeypointsCpuCustomPerPair(
        Array<T>& frameArray, const Array<T>& keypoints, const std::vector<unsigned int>& pairs,
        const std::vector<T> colorsPoints, const std::vector<T> colorsLinesPerPair,
        const T thicknessCircleRatio, const T thicknessLineRatioWRTCircle,
        const std::vector<T>& poseScales, const T threshold,
        const T circleScale, const bool forceFilledCircles,
        const bool dashedLines, const int dashLenPx, const int gapLenPx, const int thinLinePx);

    template <typename T>
    Rectangle<T> getKeypointsRectangle(
        const Array<T>& keypoints, const int person, const T threshold, const int firstIndex = 0,
        const int lastIndex = -1);

    template <typename T>
    T getAverageScore(const Array<T>& keypoints, const int person);

    template <typename T>
    T getKeypointsArea(const Array<T>& keypoints, const int person, const T threshold);

    template <typename T>
    int getBiggestPerson(const Array<T>& keypoints, const T threshold);

    template <typename T>
    int getNonZeroKeypoints(const Array<T>& keypoints, const int person, const T threshold);

    template <typename T>
    T getDistanceAverage(const Array<T>& keypoints, const int personA, const int personB, const T threshold);

    template <typename T>
    T getDistanceAverage(
        const Array<T>& keypointsA, const int personA, const Array<T>& keypointsB, const int personB,
        const T threshold);

    /**
     * Creates and Array<T> with a specific person.
     * @param keypoints Array<T> with the original data array to slice.
     * @param person indicates the index of the array to extract.
     * @param noCopy indicates whether to perform a copy. Copy will never go to undefined behavior, however, if
     * noCopy == true, then:
     *     1. It is faster, as no data copy is involved, but...
     *     2. If the Array keypoints goes out of scope, then the resulting Array will provoke an undefined behavior.
     *     3. If the returned Array is modified, the information in the Array keypoints will also be.
     * @return Array<T> with the same dimension than keypoints expect the first dimension being 1. E.g., if keypoints
     * is {p,k,m}, the resulting Array<T> is {1,k,m}.
     */
    template <typename T>
    Array<T> getKeypointsPerson(const Array<T>& keypoints, const int person, const bool noCopy = false);

    template <typename T>
    float getKeypointsRoi(const Array<T>& keypoints, const int personA, const int personB, const T threshold);

    template <typename T>
    float getKeypointsRoi(
        const Array<T>& keypointsA, const int personA, const Array<T>& keypointsB, const int personB,
        const T threshold);

    template <typename T>
    float getKeypointsRoi(
        const Rectangle<T>& rectangleA, const Rectangle<T>& rectangleB);
}

#endif // OPENPOSE_UTILITIES_KEYPOINT_HPP
