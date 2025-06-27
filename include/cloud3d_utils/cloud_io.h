#ifndef CLOUD3D_UTILS_CLOUD_IO_H_
#define CLOUD3D_UTILS_CLOUD_IO_H_

// #include <ars/definitions.h>
#include <fstream>
#include <iostream>
#include <string>

#include <cloud3d_utils/types.h>

namespace cloud3d_utils {

// using PointType = pcl::PointXYZ;
// using PointCloudType = pcl::PointCloud<PointType>;

/**
 * @brief Reads the cloud from the given filename according to the extension
 * of the filename.
 *
 * @param filename the name of the file to be read
 * @param cloud the read cloud
 * @return int return the size of the cloud if the read suceeded, otherwise
 *   a negative value
 */
int readCloud(const std::string& filename, PointCloudType& cloud);

/**
 * @brief Writes the cloud to a file whose format is specified by the filename
 * extension.
 * The extension must be one of the supprted one.
 *
 * @param filename the name of the file where the cloud is writen
 * @param cloud the cloud to be written on file
 * @return int return 0 in case of positive outcome
 */
int writeCloud(const std::string& filename, PointCloudType& cloud);

/**
 * @brief Reads the cloud from .ply file (if the file contains a mesh it used
 * the vertices).
 *
 * @param filename the name of the file to be read
 * @param cloud the read cloud
 * @return int return the size of the cloud if the read suceeded, otherwise
 *   a negative value
 */
int readPly(const std::string& filename, PointCloudType& cloud);

/**
 * @brief Writes the cloud to a .ply file (only vertices).
 *
 * @param filename the name of the file where the cloud is writen
 * @param cloud the cloud to be written on file
 * @param isBinary if true it save the .ply in binary format
 * @return int return 0 in case of positive outcome
 */
int writePly(const std::string& filename,
             const PointCloudType& cloud,
             bool isBinary = false);

/**
 * @brief Reads the cloud from .pcd file (if the file contains a mesh it used
 * the vertices). It is a wrapper for PCL functions.
 *
 * @param filename the name of the file to be read
 * @param cloud the read cloud
 * @return int return the size of the cloud if the read suceeded, otherwise
 *   a negative value
 */
int readPcd(const std::string& filename, PointCloudType& cloud);

/**
 * @brief Writes the cloud to a .pcd file.
 *
 * @param filename the name of the file where the cloud is writen
 * @param cloud the cloud to be written on file
 * @param isBinary if true it save the .ply in binary format
 * @return int return 0 in case of positive outcome
 */
int writePcd(const std::string& filename,
             const PointCloudType& cloud,
             bool isBinary = false);

/**
 * @brief Reads the cloud from a .csv file orgainized in rows with the three
 * coordinates of a point as:
 *
 *   x,y,z
 *
 * @param filename the name of the file where the cloud is writen
 * @param cloud the cloud to be written on file
 * @param isBinary if true it save the .ply in binary format
 * @return int return the size of the cloud if the read suceeded, otherwise
 *   a negative value
 */
int readCsv(const std::string& filename, PointCloudType& cloud);

/**
 * @brief Writes the cloud in a .csv file. It consists of rows with the three
 * coordinates of a point as:
 *
 *   x,y,z
 *
 * @param filename the name of the file where the cloud is writen
 * @param cloud the cloud to be written on file
 * @param isBinary if true it save the .ply in binary format
 * @return int return 0 in case of positive outcome
 */
int writeCsv(const std::string& filename, const PointCloudType& cloud);

/**
 * @brief Converts the input point cloud into a vector of points, which is
 * the type used by ARS.
 *
 * @param cloud
 * @param points
 */
void convertCloudToPoints(const PointCloudType& cloud,
                          cloud3d_utils::VectorVector3& points);

/**
 * @brief Converts the input point cloud into a vector of points with double
 * coordinates, which is the type used by ARS.
 *
 * @param cloud
 * @param points
 */
void convertCloudToPointsDouble(const PointCloudType& cloud,
                                std::vector<Eigen::Vector3d>& points);

/**
 * @brief Converts the input vector of points into a PCL point cloud.
 *
 * @param cloud
 * @param points
 */
void convertPointsToCloud(const cloud3d_utils::VectorVector3& points,
                          PointCloudTypePtr cloud);

/**
 * @brief Divide every coordinate of every point inside @param cloud
 * by @param scalarVal (default: scalarVal = 1000)
 */
void dividePclCloudByScalar(PointCloudTypePtr cloud, int scalarVal = 1000);

}  // namespace cloud3d_utils

#endif
