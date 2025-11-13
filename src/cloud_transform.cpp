#include <cloud3d_utils/cloud_transform.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/impl/uniform_sampling.hpp>

namespace cloud3d_utils {

CloudRandomTransformer& CloudRandomTransformer::getInstance() {
    static CloudRandomTransformer crt;
    return crt;
}

std::default_random_engine& CloudRandomTransformer::getRandomEngine() {
    return randGen_;
}

Transform3 CloudRandomTransformer::generateRandomTransform(
    const std::array<bool, 6>& transformAxes,
    float translMax) {
    std::uniform_real_distribution<float> randTransl(-translMax, translMax);
    std::uniform_real_distribution<float> randAngle(0.0, 2 * M_PI);
    Eigen::Vector3f transl, angles;
    float colat, lon, angleRot;
    Transform3 ret;

    // Random parameters of transformation
    transl = Eigen::Vector3f::Zero();
    angles = Eigen::Vector3f::Zero();

    // for (int i = 0; i < 6; ++i) {
    //     if (transformAxes[0]) {
    //         transl(0) = randTransl(randGen_);
    //     } else if (transformAxes[1]) {
    //         transl(1) = randTransl(randGen_);
    //     } else if (transformAxes[2]) {
    //         transl(2) = randTransl(randGen_);
    //     } else if (transformAxes[3]) {
    //         angles(0) = randAngle(randGen_);
    //     } else if (transformAxes[4]) {
    //         angles(1) = randAngle(randGen_);
    //     } else if (transformAxes[5]) {
    //         angles(2) = randAngle(randGen_);
    //     }
    // }
    if (transformAxes[0]) {
        transl(0) = randTransl(randGen_);
    }
    if (transformAxes[1]) {
        transl(1) = randTransl(randGen_);
    }
    if (transformAxes[2]) {
        transl(2) = randTransl(randGen_);
    }
    if (transformAxes[3]) {
        angles(0) = randAngle(randGen_);
    }
    if (transformAxes[4]) {
        angles(1) = randAngle(randGen_);
    }
    if (transformAxes[5]) {
        angles(2) = randAngle(randGen_);
    }

    // Computes the transformation matrix
    ret = Eigen::AngleAxis(angles(2), Eigen::Vector3f::UnitZ()) *
          Eigen::AngleAxis(angles(1), Eigen::Vector3f::UnitY()) *
          Eigen::AngleAxis(angles(0), Eigen::Vector3f::UnitX());
    ret.pretranslate(transl);
    return ret;
}

void CloudRandomTransformer::applyNoiseCloud(
    const PointCloudType::ConstPtr& cloudIn,
    float noiseSigmaPerc,
    PointCloudType::Ptr& cloudOut) {
    PointType pointMin, pointMax;
    float noiseSigma;

    *cloudOut = *cloudIn;

    pcl::getMinMax3D(*cloudIn, pointMin, pointMax);
    float sx = pointMax.x - pointMin.x;
    float sy = pointMax.y - pointMin.y;
    float sz = pointMax.z - pointMin.z;
    noiseSigma = noiseSigmaPerc * std::max(std::max(sx, sy), sz);

    // Random number generator for normal distribution
    // std::random_device randDev;
    // std::mt19937 randGen(randDev);
    // std::default_random_engine randGen;
    std::normal_distribution<> randNoise(0.0, noiseSigma);

    for (auto& p : cloudOut->points) {
        p.x += randNoise(randGen_);
        p.y += randNoise(randGen_);
        p.z += randNoise(randGen_);
    }
}

void CloudRandomTransformer::applyOcclusionCloud(
    const PointCloudType::ConstPtr& cloudIn,
    float occlPerc,
    PointCloudType::Ptr& cloudOut) {
    PointType pointMin, pointMax;
    pcl::Indices indices;
    std::vector<float> distancesSqr;
    // std::default_random_engine randGen;

    // Computes a circle centered on a randomly selected point of the cloud
    // and radius equal to the geometric mean size of the bounding box
    //  avg size = (sx * sy * sz)^{1/3} = exp(log(sx*sy*sz) / 3)
    pcl::getMinMax3D(*cloudIn, pointMin, pointMax);
    float sx = pointMax.x - pointMin.x;
    float sy = pointMax.y - pointMin.y;
    float sz = pointMax.z - pointMin.z;
    float radiusMean = occlPerc * exp((log(sx) + log(sy) + log(sz)) / 3.0);
    // float radiusSigma = 0.05 * radiusMean;
    std::uniform_int_distribution<> randIdx(0, cloudIn->size() - 1);
    PointType center = cloudIn->points[randIdx(randGen_)];

    // Finds the points inside the circulare neighborhood
    pcl::search::KdTree<PointType> searcher;
    searcher.setInputCloud(cloudIn);
    pcl::PointIndices::Ptr occluded(new pcl::PointIndices());
    searcher.radiusSearch(center, radiusMean, occluded->indices, distancesSqr);

    // Selects only the points not in the circular neighborhood
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(cloudIn);
    extract.setIndices(occluded);
    extract.setNegative(true);
    extract.filter(*cloudOut);
}

void CloudRandomTransformer::addRandomPointsCloud(
    const PointCloudType::ConstPtr& cloudIn,
    float perc,
    PointCloudType::Ptr& cloudOut) {
    PointType pointMin, pointMax, pointRand;
    pcl::getMinMax3D(*cloudIn, pointMin, pointMax);

    // Occlusion is proportional to the geometric mean of the size
    // of the point cloud
    std::uniform_real_distribution<float> coordX(pointMin.x, pointMax.x);
    std::uniform_real_distribution<float> coordY(pointMin.y, pointMax.y);
    std::uniform_real_distribution<float> coordZ(pointMin.z, pointMax.z);
    int num = (int)round(perc * cloudIn->size());
    // std::default_random_engine generator;
    *cloudOut = *cloudIn;
    for (int i = 0; i < num; ++i) {
        pointRand.x = coordX(randGen_);
        pointRand.y = coordY(randGen_);
        pointRand.z = coordZ(randGen_);
        cloudOut->push_back(pointRand);
    }
}

CloudRandomTransformer::CloudRandomTransformer() : randGen_() {
    std::chrono::time_point t_cur_point = std::chrono::system_clock::now();
    auto s = std::chrono::duration_cast<std::chrono::milliseconds>(
                 t_cur_point.time_since_epoch())
                 .count();
    randGen_.seed(s);

    ROFL_VAR1(s);
}

CloudRandomTransformer::~CloudRandomTransformer() {}

//-------------------------------------------------------------------
// FUNCTIONS
//-------------------------------------------------------------------

std::default_random_engine randGen;

void initRandSeed() {
    // In order to find the closest time instant with hour
    auto t_cur_point = std::chrono::system_clock::now();
    auto t_cur_timet = std::chrono::system_clock::to_time_t(t_cur_point);
    auto t_cur_local = localtime(&t_cur_timet);
    auto t_ref_local = t_cur_local;
    t_ref_local->tm_min = 0;
    t_ref_local->tm_sec = 0;
    auto t_ref_timet = mktime(t_ref_local);
    auto t_ref_point = std::chrono::system_clock::from_time_t(t_ref_timet);

    randGen.seed(std::chrono::duration_cast<std::chrono::microseconds>(
                     t_ref_point - t_ref_point)
                     .count());

    // std::chrono::high_resolution_clock::time_point t =
    //     std::chrono::high_resolution_clock::now();
    // unsigned int seedVal =
    // std::chrono::high_resolution_clock::since(t).count();
    // std::chrono::time_point_cast<std::chrono::microseconds>(t).randGen.seed(
    //     seedVal);
}

Transform3 generateRandomTransform(const std::array<bool, 6>& transformAxes,
                                   float translMax) {
    std::uniform_real_distribution<float> randTransl(-translMax, translMax);
    std::uniform_real_distribution<float> randAngle(0.0, 2 * M_PI);
    Eigen::Vector3f transl, angles;
    float colat, lon, angleRot;
    Transform3 ret;

    // Random parameters of transformation
    transl = Eigen::Vector3f::Zero();
    angles = Eigen::Vector3f::Zero();

    for (int i = 0; i < 6; ++i) {
        if (transformAxes[0]) {
            transl(0) = randTransl(randGen);
        } else if (transformAxes[1]) {
            transl(1) = randTransl(randGen);
        } else if (transformAxes[2]) {
            transl(2) = randTransl(randGen);
        } else if (transformAxes[3]) {
            angles(0) = randAngle(randGen);
        } else if (transformAxes[4]) {
            angles(1) = randAngle(randGen);
        } else if (transformAxes[5]) {
            angles(2) = randAngle(randGen);
        }
    }

    // Computes the transformation matrix
    ret = Eigen::AngleAxis(angles(2), Eigen::Vector3f::UnitZ()) *
          Eigen::AngleAxis(angles(1), Eigen::Vector3f::UnitY()) *
          Eigen::AngleAxis(angles(0), Eigen::Vector3f::UnitX());
    ret.pretranslate(transl);
    return ret;
}

Transform3 generateRandomTransform(float translMax) {
    std::uniform_real_distribution<float> randTransl(-translMax, translMax);
    std::uniform_real_distribution<float> randAngle(0.0, 2 * M_PI);
    Eigen::Vector3f transl, axisRot;
    float colat, lon, angleRot;
    Transform3 ret;

    // Random parameters of transformation
    transl << randTransl(randGen), randTransl(randGen), randTransl(randGen);
    colat = 0.5f * randAngle(randGen);
    lon = randAngle(randGen);
    angleRot = randAngle(randGen);
    axisRot << sin(colat) * cos(lon), sin(colat) * sin(lon), cos(colat);

    // Computes the transformation matrix
    ret = Transform3::Identity();
    ret.prerotate(Eigen::AngleAxis(angleRot, axisRot));
    ret.pretranslate(transl);
    return ret;
}

void transformCloud(const PointCloudType::ConstPtr& cloudIn,
                    const Transform3& transform,
                    PointCloudType::Ptr& cloudOut) {
    pcl::transformPointCloud(*cloudIn, *cloudOut, transform, true);
}

void applyNoiseCloud(const PointCloudType::ConstPtr& cloudIn,
                     float noiseSigmaPerc,
                     PointCloudType::Ptr& cloudOut) {
    PointType pointMin, pointMax;
    float noiseSigma;

    *cloudOut = *cloudIn;

    pcl::getMinMax3D(*cloudIn, pointMin, pointMax);
    float sx = pointMax.x - pointMin.x;
    float sy = pointMax.y - pointMin.y;
    float sz = pointMax.z - pointMin.z;
    noiseSigma = noiseSigmaPerc * std::max(std::max(sx, sy), sz);

    // Random number generator for normal distribution
    // std::random_device randDev;
    // std::mt19937 randGen(randDev);
    // std::default_random_engine randGen;
    std::normal_distribution<> randNoise(0.0, noiseSigma);

    for (auto& p : cloudOut->points) {
        p.x += randNoise(randGen);
        p.y += randNoise(randGen);
        p.z += randNoise(randGen);
    }
}

void applyOcclusionCloud(const PointCloudType::ConstPtr& cloudIn,
                         float occlPerc,
                         PointCloudType::Ptr& cloudOut) {
    PointType pointMin, pointMax;
    pcl::Indices indices;
    std::vector<float> distancesSqr;
    // std::default_random_engine randGen;

    // Computes a circle centered on a randomly selected point of the cloud
    // and radius equal to the geometric mean size of the bounding box
    //  avg size = (sx * sy * sz)^{1/3} = exp(log(sx*sy*sz) / 3)
    pcl::getMinMax3D(*cloudIn, pointMin, pointMax);
    float sx = pointMax.x - pointMin.x;
    float sy = pointMax.y - pointMin.y;
    float sz = pointMax.z - pointMin.z;
    float radiusMean = occlPerc * exp((log(sx) + log(sy) + log(sz)) / 3.0);
    // float radiusSigma = 0.05 * radiusMean;
    std::uniform_int_distribution<> randIdx(0, cloudIn->size() - 1);
    PointType center = cloudIn->points[randIdx(randGen)];

    // Finds the points inside the circulare neighborhood
    pcl::search::KdTree<PointType> searcher;
    searcher.setInputCloud(cloudIn);
    pcl::PointIndices::Ptr occluded(new pcl::PointIndices());
    searcher.radiusSearch(center, radiusMean, occluded->indices, distancesSqr);

    // Selects only the points not in the circular neighborhood
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(cloudIn);
    extract.setIndices(occluded);
    extract.setNegative(true);
    extract.filter(*cloudOut);
}

void addRandomPointsCloud(const PointCloudType::ConstPtr& cloudIn,
                          float perc,
                          PointCloudType::Ptr& cloudOut) {
    PointType pointMin, pointMax, pointRand;
    pcl::getMinMax3D(*cloudIn, pointMin, pointMax);

    // Occlusion is proportional to the geometric mean of the size
    // of the point cloud
    std::uniform_real_distribution<float> coordX(pointMin.x, pointMax.x);
    std::uniform_real_distribution<float> coordY(pointMin.y, pointMax.y);
    std::uniform_real_distribution<float> coordZ(pointMin.z, pointMax.z);
    int num = (int)round(perc * cloudIn->size());
    // std::default_random_engine generator;
    *cloudOut = *cloudIn;
    for (int i = 0; i < num; ++i) {
        pointRand.x = coordX(randGen);
        pointRand.y = coordY(randGen);
        pointRand.z = coordZ(randGen);
        cloudOut->push_back(pointRand);
    }
}

void rescaleCloud(const PointCloudType::ConstPtr& cloudIn,
                  double scale,
                  PointCloudType::Ptr& cloudOut) {
    float sx, sy, sz, smax;

    PointType pointMin, pointMax;
    pcl::getMinMax3D(*cloudIn, pointMin, pointMax);
    sx = pointMax.x - pointMin.x;
    smax = sx;
    sy = pointMax.y - pointMin.y;
    if (sy > smax)
        smax = sy;
    sz = pointMax.z - pointMin.z;
    if (sz > smax)
        smax = sz;

    float scaleFactor = (float)(scale / smax);
    pcl::transformPointCloud(*cloudIn, *cloudOut, Eigen::Vector3f::Zero(),
                             Eigen::Quaternionf::Identity(), scaleFactor);
}

void sampleCloud(const PointCloudType::ConstPtr& cloudIn,
                 int num,
                 PointCloudType::Ptr& cloudOut) {
    float sx, sy, sz, radius;

    // Computes the surface of the bounding box containing the input cloud.
    // The radius sample is computed as follows:
    //   bounding_box_surface = 2 * (sx * sy + sy * sz + sz * sx)
    //   search_box_surface = bounding_box_surface / num
    // We assume that a search_box_surface is inscribed into the search circle
    // where the search circles are partially overlapping.
    // The search_box_surface has side about sqrt(2) * radius. Thus,
    //   radius = sqrt(search_circle_surface / M_PI)
    // Final radius is
    PointType pointMin, pointMax;
    pcl::getMinMax3D(*cloudIn, pointMin, pointMax);
    sx = pointMax.x - pointMin.x;
    sy = pointMax.y - pointMin.y;
    sz = pointMax.z - pointMin.z;
    radius = sqrt(2 * (sx * sy + sy * sz + sz * sx) / (1.4142 * num));

    pcl::UniformSampling<cloud3d_utils::PointType> sampler;
    sampler.setInputCloud(cloudIn);
    sampler.setRadiusSearch(radius);
    sampler.filter(*cloudOut);
}

}  // namespace cloud3d_utils
