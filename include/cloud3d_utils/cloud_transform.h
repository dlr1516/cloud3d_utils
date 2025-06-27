#ifndef CLOUD3D_UTILS_CLOUD_TRANSFORM_H_
#define CLOUD3D_UTILS_CLOUD_TRANSFORM_H_

#include <chrono>
#include <random>

#include <cloud3d_utils/types.h>
#include <rofl/common/macros.h>

namespace cloud3d_utils {

//-------------------------------------------------------------------
// CLASS CLOUD RANDOM TRANSFORM
//-------------------------------------------------------------------

class CloudRandomTransformer {
   public:
    static CloudRandomTransformer& getInstance();

    std::default_random_engine& getRandomEngine();

    Transform3 generateRandomTransform(const std::array<bool, 6>& transformAxes,
                                       float translMax);

    void applyNoiseCloud(const PointCloudType::ConstPtr& cloudIn,
                         float noiseSigmaPerc,
                         PointCloudType::Ptr& cloudOut);

    void applyOcclusionCloud(const PointCloudType::ConstPtr& cloudIn,
                             float perc,
                             PointCloudType::Ptr& cloudOut);

    void addRandomPointsCloud(const PointCloudType::ConstPtr& cloudIn,
                              float perc,
                              PointCloudType::Ptr& cloudOut);

   private:
    std::default_random_engine randGen_;

    CloudRandomTransformer();

    virtual ~CloudRandomTransformer();
};

//-------------------------------------------------------------------
// FUNCTIONS
//-------------------------------------------------------------------

/**
 * @brief Initializes the seed used for random number generator.
 *
 */
void initRandSeed();

/**
 * @brief Generate a random rotation and translation.
 *
 * @param translMax the scale of translation values
 * @return Transform3
 */
Transform3 generateRandomTransform(float translMax);

Transform3 generateRandomTransform(const std::array<bool, 6>& transformAxes,
                                   float translMax);

/**
 * @brief Changes the coordinates of the input point cloud according to the
 * given transform. It is a wrapper of an equivalent PCL function.
 *
 * @param cloudIn the input point cloud
 * @param transform the transformation
 * @param cloudOut the transformed point cloud
 */
void transformCloud(const PointCloudType::ConstPtr& cloudIn,
                    const Transform3& transform,
                    PointCloudType::Ptr& cloudOut);

/**
 * @brief Returns a new point cloud obtained by adding normal random noise to
 * the point coordinates of input point cloud. The standard deviation of noise
 * is a percentage of the size of the input point cloud.
 *
 * @param cloudIn the input point cloud
 * @param noiseSigmaPerc the coeffient for computing the noise standard
 * deviation
 * @param cloudOut the transformed point cloud
 */
void applyNoiseCloud(const PointCloudType::ConstPtr& cloudIn,
                     float noiseSigmaPerc,
                     PointCloudType::Ptr& cloudOut);

/**
 * @brief Returns a new point cloud obtained by removing points in a random
 * occluding region
 *
 * @param cloudIn the input point cloud
 * @param perc percentage size of occlusion region w.r.t. the cloud size
 * @param cloudOut the transformed point cloud
 */
void applyOcclusionCloud(const PointCloudType::ConstPtr& cloudIn,
                         float perc,
                         PointCloudType::Ptr& cloudOut);

/**
 * @brief Returns a new point cloud obtained by adding outlier random points
 *
 * @param cloudIn the input point cloud
 * @param perc
 * @param cloudOut the transformed point cloud
 */
void addRandomPointsCloud(const PointCloudType::ConstPtr& cloudIn,
                          float perc,
                          PointCloudType::Ptr& cloudOut);

/**
 * @brief Returns a resampled point cloud.
 *
 * @param cloudIn the input point cloud
 * @param num approximate expected number of points
 * @param cloudOut the resampled point cloud
 */
void sampleCloud(const PointCloudType::ConstPtr& cloudIn,
                 int num,
                 PointCloudType::Ptr& cloudOut);

}  // namespace cloud3d_utils

#endif
