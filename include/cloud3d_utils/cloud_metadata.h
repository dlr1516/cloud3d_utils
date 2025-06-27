#ifndef CLOUD3D_UTILS_CLOUD_METADATA_H_
#define CLOUD3D_UTILS_CLOUD_METADATA_H_

#include <array>
#include <string>
#include <vector>

#include <ars/definitions.h>

#include <cloud3d_utils/types.h>

namespace cloud3d_utils {

struct CloudMetadata {
   public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::string label;
    cloud3d_utils::Transform3 transform;
    float noiseSigma;
    float occlusionPerc;
    float randPointPerc;
    float translationMax;
    std::vector<std::string> formats;
    std::array<bool, 6> transformAxes;

    CloudMetadata();

    virtual ~CloudMetadata();

    void read(std::istream& is);

    void read(const std::string& filename);

    void write(std::ostream& is) const;

    void write(const std::string& filename) const;

    std::string getName() const;

    std::string getDirSuffix() const;

    //    private:
    //     static std::array<char, 62> constexpr encodeTable{
    //         'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M',
    //         'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z',
    //         'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm',
    //         'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z',
    //         '0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};
};

}  // namespace cloud3d_utils

#endif
