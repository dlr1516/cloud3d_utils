// #include <ars/definitions.h>
#include <rofl/common/io.h>
#include <rofl/common/param_map.h>

#include <pcl/common/common.h>
#include <pcl/filters/uniform_sampling.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <cloud3d_utils/cloud_io.h>
#include <cloud3d_utils/cloud_metadata.h>
#include <cloud3d_utils/cloud_transform.h>
#include <cloud3d_utils/types.h>

void createAndSave(cloud3d_utils::PointCloudType::Ptr& cloudOut,
                   cloud3d_utils::CloudMetadata& cloudData,
                   std::filesystem::path pathCloud);

int main(int argc, char** argv) {
    rofl::ParamMap params;
    std::string filenameIn, dirIn, filenameOut, dirOut, filenameCfg;
    std::vector<std::string> cloudFilenames;
    cloud3d_utils::PointCloudType::Ptr cloudIn(
        new cloud3d_utils::PointCloudType);
    cloud3d_utils::PointCloudType::Ptr cloudSampled(
        new cloud3d_utils::PointCloudType);
    cloud3d_utils::PointCloudType::Ptr cloudOut(
        new cloud3d_utils::PointCloudType);
    cloud3d_utils::CloudMetadata cloudData;
    float radius, scale, translMax;
    int num, cloudOutNum;
    bool outPly, outPcd, outCsv;
    std::string formats, transformAxes;
    std::filesystem::path pathCloudOut;

    params.read(argc, argv);
    params.getParam<std::string>("cfg", filenameCfg, "");
    std::cout << "config filename: " << filenameCfg << std::endl;
    if (filenameCfg != "") {
        params.read(filenameCfg);
    }

    params.read(argc, argv);
    params.getParam<std::string>("in", filenameIn, "");
    params.getParam<std::string>("dirIn", dirIn, "");
    params.getParam<std::string>("out", filenameOut, "");
    params.getParam<std::string>("dirOut", dirOut, "");
    params.getParam<float>("radius", radius, 0.001f);
    params.getParam<float>("scale", scale, 1.0f);
    params.getParam<int>("num", num, 1000);
    params.getParam<int>("cloudOutNum", cloudOutNum, 2);
    params.getParam<float>("transfMax", cloudData.translationMax, 1.0f);
    params.getParam<float>("noiseSigma", cloudData.noiseSigma, 0.0f);
    params.getParam<float>("occlPerc", cloudData.occlusionPerc, 0.0f);
    params.getParam<float>("randPerc", cloudData.randPointPerc, 0.0f);
    params.getParam<std::string>("axes", transformAxes,
                                 std::string("rx,ry,rz,tx,ty,tz"));
    params.getParam<std::string>("formats", formats,
                                 std::string(".ply,.pcd,.csv"));
    params.getParam<bool>("outPly", outPly, true);
    params.getParam<bool>("outCsv", outCsv, true);
    params.getParam<bool>("outPcd", outPcd, true);

    params.adaptTildeInPaths();
    params.getParam<std::string>("in", filenameIn, "");
    params.getParam<std::string>("dirIn", dirIn, "");
    params.getParam<std::string>("out", filenameOut, "");
    params.getParam<std::string>("dirOut", dirOut, "");

    std::cout << "\nParameter values:\n";
    params.write(std::cout);
    std::cout << std::endl;

    cloud3d_utils::initRandSeed();

    // Splits the formats into a list of formats
    boost::char_separator<char> sep(",;:");
    // std::stringstream ssformats(formats);
    // std::string strTmp;
    // while (getline(ssformats, strTmp,
    //                ',')) {  // use comma as delim for cutting string
    //     std::cout << "format \"" << strTmp << "\"" << std::endl;
    //     cloudData.formats.push_back(strTmp);
    // }
    boost::tokenizer<boost::char_separator<char> > tokens(formats, sep);
    std::vector<std::string> formatsTmp;
    for (auto sit = tokens.begin(); sit != tokens.end(); ++sit) {
        std::string tmpS = *sit;
        std::cout << "cloud file format \"" << tmpS << "\"" << std::endl;
        formatsTmp.push_back(tmpS);
        // cloudData.formats.push_back(tmpS);
    }
    cloudData.formats = formatsTmp;
    //  Splits the formats into a list of formats
    boost::tokenizer<boost::char_separator<char> > tokensAxes(transformAxes,
                                                              sep);
    int axisId = -1;
    for (auto sit = tokensAxes.begin(); sit != tokensAxes.end(); ++sit) {
        if (*sit == "tx") {
            axisId = 0;
            cloudData.transformAxes[0] = true;
            std::cout << "transform tx " << cloudData.transformAxes[0]
                      << std::endl;
        } else if (*sit == "ty") {
            axisId = 1;
            cloudData.transformAxes[1] = true;
            std::cout << "transform ty " << cloudData.transformAxes[1]
                      << std::endl;
        } else if (*sit == "tz") {
            axisId = 2;
            cloudData.transformAxes[2] = true;
            std::cout << "transform tz " << cloudData.transformAxes[2]
                      << std::endl;
        } else if (*sit == "rx") {
            axisId = 3;
            cloudData.transformAxes[3] = true;
            std::cout << "transform rx " << cloudData.transformAxes[3]
                      << std::endl;
        } else if (*sit == "ry") {
            axisId = 4;
            cloudData.transformAxes[4] = true;
            std::cout << "transform ry " << cloudData.transformAxes[4]
                      << std::endl;
        } else if (*sit == "rz") {
            axisId = 5;
            cloudData.transformAxes[5] = true;
            std::cout << "transform rz " << cloudData.transformAxes[5]
                      << std::endl;
        }
        std::cout << "token \"" << *sit << "\" transformation axis[" << axisId
                  << "]: \"" << cloudData.transformAxes[axisId] << "\""
                  << std::endl;
    }

    // Creates output directory for dataset
    std::filesystem::path pathDirIn = std::filesystem::path(dirIn);
    if (!std::filesystem::is_directory(pathDirIn)) {
        std::cout << "NOT a directory " << pathDirIn << ": removing filename"
                  << std::endl;
        pathDirIn = pathDirIn.remove_filename();
    }
    // pathDirIn = std::filesystem::path(dirIn)
    //                 .lexically_normal()
    //                 .remove_filename()
    //                 .parent_path();
    ARS_VAR4(dirIn, pathDirIn, pathDirIn.parent_path(), pathDirIn.stem());
    std::filesystem::path pathDirOut;
    if (dirOut == "") {
        pathDirOut = pathDirIn.parent_path();
    } else {
        pathDirOut = dirOut;
    }
    std::cout << "base output directory " << pathDirOut << std::endl;
    std::stringstream ss;
    ss << pathDirIn.stem().string() << "_sample" << std::setw(7)
       << std::setfill('0') << num << cloudData.getDirSuffix();
    std::cout << "appendix to output directory " << ss.str() << std::endl;
    pathDirOut = pathDirOut / std::filesystem::path(ss.str());
    // std::filesystem::path pathOut = pathIn.;
    std::cout << "Create output directory " << pathDirOut << std::endl;
    std::filesystem::create_directory(pathDirOut);

    rofl::getDirectoryFiles(dirIn, cloudFilenames);
    for (int i = 0; i < cloudFilenames.size(); ++i) {
        int ret = cloud3d_utils::readCloud(cloudFilenames[i], *cloudIn);
        if (ret >= 0) {
            cloud3d_utils::rescaleCloud(cloudIn, scale, cloudSampled);
            cloudIn->swap(*cloudSampled);
            cloud3d_utils::sampleCloud(cloudIn, num, cloudSampled);
            std::cout << "processing \"" << cloudFilenames[i]
                      << "\": downsampled from " << cloudIn->size() << " to "
                      << cloudOut->size() << " points" << std::endl;
            // std::swap(cloudIn, cloudOut);

            // Creates the first point cloud
            //*cloudIn = *cloudOut;

            for (int j = 0; j < cloudOutNum; ++j) {
                ss.str("");
                ss << "_" << std::setw(2) << std::setfill('0') << j;
                cloudData.label =
                    std::filesystem::path(cloudFilenames[i]).stem().string() +
                    ss.str();
                std::cout << "generating cloud \"" << cloudData.label << "\""
                          << std::endl;
                *cloudOut = *cloudSampled;
                createAndSave(cloudOut, cloudData, pathDirOut);
            }
        } else {
            ARS_ERROR("Unupported cloud format for file \"" << cloudFilenames[i]
                                                            << "\"")
        }
    }

    return 0;
}

void createAndSave(cloud3d_utils::PointCloudType::Ptr& cloudOut,
                   cloud3d_utils::CloudMetadata& cloudData,
                   std::filesystem::path pathCloud) {
    std::filesystem::path pathFile;
    cloud3d_utils::PointType pointMin, pointMax;
    cloud3d_utils::PointCloudType::Ptr cloudTmp(
        new cloud3d_utils::PointCloudType);

    // Computes the maximum size
    pcl::getMinMax3D(*cloudOut, pointMin, pointMax);
    float sx = pointMax.x - pointMin.x;
    float sy = pointMax.y - pointMin.y;
    float sz = pointMax.z - pointMin.z;
    float size = std::max(sx, std::max(sy, sz));

    // Creates random transformation of cloud coordinates
    // cloudData.transform =
    //     cloud3d_utils::generateRandomTransform(cloudData.translationMax *
    //     size);
    // cloudData.transform = cloud3d_utils::generateRandomTransform(
    //     cloudData.transformAxes, cloudData.translationMax * size);
    cloudData.transform =
        cloud3d_utils::CloudRandomTransformer::getInstance()
            .generateRandomTransform(cloudData.transformAxes,
                                     cloudData.translationMax * size);
    cloud3d_utils::transformCloud(cloudOut, cloudData.transform, cloudTmp);
    cloudOut->swap(*cloudTmp);

    ARS_VAR1(cloudData.transform.matrix());
    // Applies distortion transformations if enabled
    if (cloudData.noiseSigma > 0.0f) {
        // cloud3d_utils::applyNoiseCloud(cloudOut, cloudData.noiseSigma,
        // cloudOut);
        cloud3d_utils::CloudRandomTransformer::getInstance().applyNoiseCloud(
            cloudOut, cloudData.noiseSigma, cloudTmp);
        cloudOut->swap(*cloudTmp);
    }
    if (cloudData.occlusionPerc > 0.0f) {
        // cloud3d_utils::applyOcclusionCloud(cloudOut, cloudData.occlusionPerc,
        //                                 cloudOut);
        cloud3d_utils::CloudRandomTransformer::getInstance()
            .applyOcclusionCloud(cloudOut, cloudData.occlusionPerc, cloudTmp);
        cloudOut->swap(*cloudTmp);
    }
    if (cloudData.randPointPerc > 0.0f) {
        // cloud3d_utils::addRandomPointsCloud(cloudOut,
        // cloudData.occlusionPerc,
        //                                  cloudOut);
        cloud3d_utils::CloudRandomTransformer::getInstance()
            .addRandomPointsCloud(cloudOut, cloudData.occlusionPerc, cloudTmp);
        cloudOut->swap(*cloudTmp);
    }
    std::cout << "Saving output clouds in " << cloudData.formats.size()
              << " formats" << std::endl;
    // Saves the cloud in the required file formats
    std::string filenameOut = cloudData.getName();
    for (auto& format : cloudData.formats) {
        std::cout << "format \"" << format << "\"" << std::endl;
        pathFile = pathCloud / (filenameOut + format);
        std::cout << "output to " << pathFile << std::endl;
        cloud3d_utils::writeCloud(pathFile.string(), *cloudOut);
        // std::cout << "saving file \"" << filenameOut << "\"" << std::endl;
    }
    pathFile = pathCloud / (filenameOut + ".info");
    cloudData.write(pathFile.string());
    std::cout << "saving output metadata of point cloud on file " << pathFile
              << std::endl;
    std::cout << "*** CLOUD DATA written" << std::endl;
    cloudData.write(std::cout);
    std::cout << "*** CLOUD DATA read from " << pathFile << std::endl;
    cloud3d_utils::CloudMetadata cloudRead;
    cloudRead.read(pathFile.string());
    cloudRead.write(std::cout);
}
