
#include <cloud3d_utils/cloud_io.h>
#include <rofl/common/macros.h>

#define TINYPLY_IMPLEMENTATION
#include "thirdparty/tinyply.h"

#include <filesystem>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <pcl/io/pcd_io.h>

namespace cloud3d_utils {

int readCloud(const std::string& filename, PointCloudType& cloud) {
    std::string fileExt = std::filesystem::path(filename).extension();
    if (fileExt == ".ply") {
        return readPly(filename, cloud);
    } else if (fileExt == ".csv") {
        return readCsv(filename, cloud);
    } else if (fileExt == ".pcd") {
        return readPcd(filename, cloud);
    } else {
        ROFL_ERR("Cannot open file \"" << filename << "\": \"" << fileExt
                                       << "\" is not a supported extension");
        return -1;
    }
    return -1;
}

int writeCloud(const std::string& filename, PointCloudType& cloud) {
    std::string fileExt = std::filesystem::path(filename).extension();
    if (fileExt == ".ply") {
        return writePly(filename, cloud);
    } else if (fileExt == ".csv") {
        return writeCsv(filename, cloud);
    } else if (fileExt == ".pcd") {
        return writePcd(filename, cloud);
    } else {
        ROFL_ERR("Cannot open file \"" << filename << "\": \"" << fileExt
                                       << "\" is not a supported extension");
        return -1;
    }
    return -1;
}

int readPly(const std::string& filename, PointCloudType& cloud) {
    std::ifstream file(filename);
    if (!file) {
        ROFL_ERR("cannot open file \"" << filename << "\"");
        return -1;
    }

    tinyply::PlyFile filePly;
    filePly.parse_header(file);

    std::shared_ptr<tinyply::PlyData> vertices;
    try {
        vertices =
            filePly.request_properties_from_element("vertex", {"x", "y", "z"});
    } catch (const std::exception& e) {
        ROFL_ERR("tinyply exception: " << e.what());
        return -1;
    }

    filePly.read(file);

    if (vertices) {
        std::cout << "\tRead " << vertices->count << " total vertices "
                  << std::endl;
        if (vertices->t == tinyply::Type::FLOAT32) {
            std::vector<float> verts_floats(3 * vertices->count);
            const size_t numVerticesBytes = vertices->buffer.size_bytes();
            std::memcpy(verts_floats.data(), vertices->buffer.get(),
                        numVerticesBytes);
            for (int i = 0; i + 2 < verts_floats.size(); i += 3) {
                PointType p;
                p.x = verts_floats[i];
                p.y = verts_floats[i + 1];
                p.z = verts_floats[i + 2];
                cloud.push_back(p);
            }
        }
        if (vertices->t == tinyply::Type::FLOAT64) {
            std::vector<double> verts_doubles(3 * vertices->count);
            const size_t numVerticesBytes = vertices->buffer.size_bytes();
            std::memcpy(verts_doubles.data(), vertices->buffer.get(),
                        numVerticesBytes);
            for (int i = 0; i + 1 < verts_doubles.size(); i += 3) {
                PointType p;
                p.x = verts_doubles[i];
                p.y = verts_doubles[i + 1];
                p.z = verts_doubles[i + 2];
                cloud.push_back(p);
            }
        }
    }

    file.close();
    return cloud.size();
}

int writePly(const std::string& filename,
             const PointCloudType& cloud,
             bool isBinary) {
    std::ofstream file(filename);
    if (!file) {
        ROFL_ERR("cannot open file \"" << filename << "\"");
        return -1;
    }

    tinyply::PlyFile ply_file;
    std::vector<float> temp_vertices;
    for (auto& p : cloud.points) {
        temp_vertices.push_back(p.x);
        temp_vertices.push_back(p.y);
        temp_vertices.push_back(p.z);
    }
    ply_file.add_properties_to_element(
        "vertex", {"x", "y", "z"}, tinyply::Type::FLOAT32,
        (temp_vertices.size() / 3),
        reinterpret_cast<uint8_t*>(temp_vertices.data()),
        tinyply::Type::INVALID, 0);
    ply_file.write(file, isBinary);

    file.close();
    return 0;
}

int readPcd(const std::string& filename, PointCloudType& cloud) {
    if (pcl::io::loadPCDFile<PointType>(filename, cloud) == -1) {
        ROFL_ERR("Cannot open PCD file \"" << filename << "\"");
        return -1;
    }
    return cloud.size();
}

int writePcd(const std::string& filename,
             const PointCloudType& cloud,
             bool isBinary) {
    if (isBinary) {
        if (pcl::io::savePCDFileBinary<PointType>(filename, cloud) == -1) {
            ROFL_ERR("Cannot save to Binary PCD file \"" << filename << "\"");
            return -1;
        }
    } else {
        if (pcl::io::savePCDFileASCII<PointType>(filename, cloud) == -1) {
            ROFL_ERR("Cannot save to ASCII PCD file \"" << filename << "\"");
            return -1;
        }
    }

    return cloud.size();
}

int readCsv(const std::string& filename, PointCloudType& cloud) {
    PointType p;
    std::string line;
    std::vector<std::string> tokens;
    int j, k;

    std::ifstream file(filename);
    if (!file) {
        ROFL_ERR("Cannot open file \"" << filename << "\"");
        return -1;
    }
    // while (file >> p(0) >> p(1) >> p(2)) {
    //   points.push_back(p);
    // }
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        // Extract each coordinate value from column
        // for (j = 0; j < p.size() && std::getline(ss, coord, ','); ++j) {
        //   ARS_VAR3(j, line, coord);
        //   p(j) = boost::lexical_cast<double>(coord);
        // }
        // if (j == 3) {
        //   points.push_back(p);
        // } else {
        //   ROFL_ERR("invalid line \"" << line << "\": cannot be cast to a
        //   point");
        // }
        // ARS_VAR1(line);
        boost::split(tokens, line, boost::is_any_of(" \t,;"));

        j = 0;
        k = 0;
        while (k < 3 && j < tokens.size()) {
            // ARS_VAR3(j, tokens[j], k);
            if (tokens[j] != "") {
                // p(k) = boost::lexical_cast<double>(tokens[j]);
                p.data[k] = boost::lexical_cast<float>(tokens[j]);
                k++;
            }
            j++;
        }
        if (k == 3) {
            // ARS_VAR1(p.transpose());
            cloud.push_back(p);
        } else {
            ROFL_ERR("invalid line \"" << line << "\": found " << tokens.size()
                                       << " line cannot be cast to a point");
        }
    }
    file.close();
    return cloud.size();
}

int writeCsv(const std::string& filename, const PointCloudType& cloud) {
    std::ofstream file(filename);
    if (!file) {
        ROFL_ERR("cannot open file \"" << filename << "\"");
        return -1;
    }

    for (int i = 0; i < cloud.size(); ++i) {
        file << cloud.points[i].x << "," << cloud.points[i].y << ","
             << cloud.points[i].z << "\n";
    }

    file.close();
    return 0;
}

void convertCloudToPoints(const PointCloudType& cloud,
                          cloud3d_utils::VectorVector3& points) {
    points.resize(cloud.size());
    for (int i = 0; i < cloud.size(); ++i) {
        points[i](0) = cloud.at(i).x;
        points[i](1) = cloud.at(i).y;
        points[i](2) = cloud.at(i).z;
    }
}

void convertCloudToPointsDouble(const PointCloudType& cloud,
                                std::vector<Eigen::Vector3d>& points) {
    points.resize(cloud.size());
    for (int i = 0; i < cloud.size(); ++i) {
        points[i](0) = cloud.at(i).x;
        points[i](1) = cloud.at(i).y;
        points[i](2) = cloud.at(i).z;
    }
}

void convertPointsToCloud(const cloud3d_utils::VectorVector3& points,
                          PointCloudTypePtr cloud) {
    cloud->clear();
    cloud->resize(points.size());
    for (int i = 0; i < points.size(); ++i) {
        cloud->at(i).x = points[i](0);
        cloud->at(i).y = points[i](1);
        cloud->at(i).z = points[i](2);
    }
}

void dividePclCloudByScalar(PointCloudTypePtr cloud, int scalarVal) {
    for (int i = 0; i < cloud->size(); ++i) {
        cloud->at(i).x /= scalarVal;
        cloud->at(i).y /= scalarVal;
        cloud->at(i).z /= scalarVal;
    }
}

}  // namespace cloud3d_utils
