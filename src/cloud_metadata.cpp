#include <rofl/common/param_map.h>
#include <cloud3d_utils/cloud_metadata.h>
#include <rofl/common/io.h>
#include <eigen3/Eigen/Dense>

namespace cloud3d_utils {

CloudMetadata::CloudMetadata()
    : label(""),
      transform(),
      noiseSigma(0.0f),
      occlusionPerc(0.0f),
      randPointPerc(0.0f),
      translationMax(0.0f),
      formats(),
      transformAxes() {
    transformAxes.fill(false);
    formats.reserve(3);
}

CloudMetadata::~CloudMetadata() {}

void CloudMetadata::read(std::istream& is) {
    rofl::ParamMap params;
    float tx, ty, tz, qx, qy, qz, qw;

    params.read(is);
    params.getParam("label", label, std::string("undefined"));
    params.getParam("translation_x", tx, float(0.0f));
    params.getParam("translation_y", ty, float(0.0f));
    params.getParam("translation_z", tz, float(0.0f));
    params.getParam("quaternion_x", qx, float(0.0f));
    params.getParam("quaternion_y", qy, float(0.0f));
    params.getParam("quaternion_z", qz, float(0.0f));
    params.getParam("quaternion_w", qw, float(0.0f));
    params.getParam("noise_sigma", noiseSigma, float(0.0f));
    params.getParam("occlusion_perc", occlusionPerc, float(0.0f));
    params.getParam("rand_point_perc", randPointPerc, float(0.0f));

    transform = Transform3::Identity();
    transform.prerotate(Eigen::Quaternionf(qw, qx, qy, qz));
    transform.pretranslate(Eigen::Vector3f(tx, ty, tz));
    ARS_PRINT("transform: \n" << transform.matrix());
}

void CloudMetadata::read(const std::string& filename) {
    std::ifstream file(filename);
    if (!file) {
        ARS_ERROR("Cannot read file \"" << filename << "\"");
        return;
    }
    read(file);
    file.close();
}

void CloudMetadata::write(std::ostream& os) const {
    rofl::ParamMap params;

    Eigen::Vector3f t = transform.translation();
    Eigen::Matrix3f rot = transform.rotation();
    Eigen::Quaternionf q(rot);

    ARS_PRINT("cloud transformation\n"
              << transform.matrix() << "\ntranslation " << t.transpose()
              << "\nrotation: quaternion w " << q.w() << " x " << q.x() << " y "
              << q.y() << " z " << q.z() << "\nrotation matrix\n"
              << q.toRotationMatrix());

    params.setParam("label", label);
    params.setParam("translation_x", t(0));
    params.setParam("translation_y", t(1));
    params.setParam("translation_z", t(2));
    params.setParam("quaternion_x", q.x());
    params.setParam("quaternion_y", q.y());
    params.setParam("quaternion_z", q.z());
    params.setParam("quaternion_w", q.w());
    params.setParam("noise_sigma", noiseSigma);
    params.setParam("occlusion_perc", occlusionPerc);
    params.setParam("rand_point_perc", randPointPerc);
    params.write(os);
}

void CloudMetadata::write(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file) {
        ARS_ERROR("Cannot write file \"" << filename << "\"");
        return;
    }
    write(file);
    file.close();
}

std::string CloudMetadata::getName() const {
    std::stringstream ss;

    Eigen::Vector3f t = transform.translation();
    Eigen::Matrix3f rot = transform.rotation();
    Eigen::Quaternionf q(rot);

    ss << label;
    ss << "_noise" << std::setw(3) << std::setfill('0')
       << (int)floor(100 * noiseSigma);
    ss << "_occl" << std::setw(3) << std::setfill('0')
       << (int)floor(100 * occlusionPerc);
    ss << "_rand" << std::setw(3) << std::setfill('0')
       << (int)floor(100 * randPointPerc);
    ss << "_";
    return rofl::generateStampedString(ss.str(), "");
}

std::string CloudMetadata::getDirSuffix() const {
    std::stringstream ss;

    Eigen::Vector3f t = transform.translation();
    Eigen::Matrix3f rot = transform.rotation();
    Eigen::Quaternionf q(rot);

    ss << label;
    ss << "_noise" << std::setw(3) << std::setfill('0')
       << (int)floor(100 * noiseSigma);
    ss << "_occl" << std::setw(3) << std::setfill('0')
       << (int)floor(100 * occlusionPerc);
    ss << "_rand" << std::setw(3) << std::setfill('0')
       << (int)floor(100 * randPointPerc);
    return ss.str();
}

}  // namespace ars3d_test
