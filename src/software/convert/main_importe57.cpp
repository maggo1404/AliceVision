// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <boost/program_options.hpp>

#include <E57SimpleData.h>
#include <E57SimpleReader.h>
#include <aliceVision/camera/camera.hpp>



// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

// convert from a SfMData format to another
int aliceVision_main(int argc, char **argv)
{
    // command-line parameters
    std::vector<std::string> e57filenames;
    std::string outputSfMDataFilename;
    int columnJump = 3;
    int rowJump = 3;

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
    ("input,i", po::value<std::vector<std::string>>(&e57filenames)->multitoken()->required(),
      "Path to e57 fomes.")
    ("output,o", po::value<std::string>(&outputSfMDataFilename)->required(),
        "Path to the output sfm file.");
    
    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("columnJump", po::value<int>(&columnJump)->default_value(columnJump), "Use one column every columnJump")
        ("rowJump", po::value<int>(&rowJump)->default_value(rowJump), "Use one row every rowJump");

    CmdLine cmdline("AliceVision importe57");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    sfmData::SfMData sfmData;
    auto & landmarks = sfmData.getLandmarks();

    //Create a camera intrinsics object with dummy parameters
    auto cam = camera::createPinhole(aliceVision::camera::EINTRINSIC::EQUIDISTANT_CAMERA, 1, 1, 1, 1, 0, 0);
    sfmData.getIntrinsics().emplace(0, cam);


    //Multiple files may be needed to create the environment
    size_t totalMeshCount = 0;
    for (const std::string & e57filename : e57filenames)
    {
        e57::Reader reader(e57filename, e57::ReaderOptions());
        if (!reader.IsOpen())
        {
            ALICEVISION_LOG_ERROR("Input not found");
            return EXIT_FAILURE;
        }

        e57::E57Root root;
        if (!reader.GetE57Root(root))
        {
            ALICEVISION_LOG_ERROR("E57 : error reading root");
            return EXIT_FAILURE;
        }

        size_t meshCount = reader.GetData3DCount();
        ALICEVISION_LOG_INFO("The input contains " << meshCount << " meshes");

        // There may be multiple ppoint cloud per file
        // Each point cloud is in its own local frame (Centered on sensor)
        for (int indexMesh = 0; indexMesh < meshCount; indexMesh++)
        {
            size_t idMesh = totalMeshCount + indexMesh;

            e57::Data3D scanHeader;
            if (!reader.ReadData3D(indexMesh, scanHeader))
            {
                ALICEVISION_LOG_ERROR("Error reading mesh #" << indexMesh);
                continue;
            }


            Eigen::Quaternion<double> q(scanHeader.pose.rotation.w, 
                                        scanHeader.pose.rotation.x, 
                                        scanHeader.pose.rotation.y, 
                                        scanHeader.pose.rotation.z);

            Eigen::Matrix3d R = q.normalized().toRotationMatrix();
            Eigen::Vector3d t;
            t(0) = scanHeader.pose.translation.x;
            t(1) = scanHeader.pose.translation.y;
            t(2) = scanHeader.pose.translation.z;

            //Pose is a world_T_sensor transform
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3, 3>(0, 0) = R.transpose();
            T.block<3, 1>(0, 3) = -R.transpose() * t;

            geometry::Pose3 pose(T);
            sfmData.getPoses().emplace(idMesh, pose);

            sfmData::View::sptr view = std::make_shared<sfmData::View>("nopath", idMesh, 0, idMesh, 1, 1);
            sfmData.getViews().emplace(idMesh, view);

            int64_t maxRows = 0;
            int64_t maxColumns = 0;
            int64_t countPoints = 0;
            int64_t countGroups = 0;
            int64_t maxGroupSize = 0;
            bool isColumnIndex;

            if (!reader.GetData3DSizes(0, maxRows, maxColumns, countPoints, countGroups, maxGroupSize, isColumnIndex))
            {
                ALICEVISION_LOG_ERROR("Error reading content of mesh #" << indexMesh);
                continue;
            }

            ALICEVISION_LOG_INFO("Current mesh has " << countPoints << " points");

            e57::Data3DPointsFloat data3DPoints(scanHeader);
            e57::CompressedVectorReader datareader = reader.SetUpData3DPointsData(indexMesh, countPoints, data3DPoints);
        

            unsigned readCount = 0;
            while ((readCount = datareader.read()) > 0)
            {
                if (data3DPoints.sphericalRange != nullptr)
                {
                    ALICEVISION_LOG_ERROR("Data contains spherical coordinates, this is not currently supported");
                    continue;
                }

                if (data3DPoints.cartesianX == nullptr)
                {
                    ALICEVISION_LOG_ERROR("Data contains no cartesian coordinates");
                    continue;
                }

                if (data3DPoints.columnIndex == nullptr)
                {
                    ALICEVISION_LOG_ERROR("Data contains no 2d column coordinates");
                    continue;
                }

                if (data3DPoints.rowIndex == nullptr)
                {
                    ALICEVISION_LOG_ERROR("Data contains no 2d row coordinates");
                    continue;
                }

                sfmData::Observation obs(Vec2(0.0, 0.0), landmarks.size(), 1.0);
                
                int count = 0;
                Eigen::Vector3d sum = Eigen::Vector3d::Zero();
                for (int pos = 0; pos < readCount; pos++)
                {
                    if (data3DPoints.columnIndex[pos] % columnJump != 0)
                    {
                        continue;
                    }

                    if (data3DPoints.rowIndex[pos] % rowJump != 0)
                    {
                        continue;
                    }

                    if (data3DPoints.cartesianInvalidState[pos])
                    {
                        continue;
                    }

                    Eigen::Vector3d pt;
                    pt(0) = data3DPoints.cartesianX[pos];
                    pt(1) = data3DPoints.cartesianY[pos];
                    pt(2) = data3DPoints.cartesianZ[pos];

                    Eigen::Vector3d tpt = R * pt + t;

                    sfmData::Landmark landmark(tpt, feature::EImageDescriberType::SIFT);
                    landmark.getObservations().emplace(idMesh, obs);
                    landmarks.emplace(landmarks.size(), landmark);
                }
            }
        }   

        reader.Close();
        totalMeshCount += meshCount;
    }


    sfmDataIO::save(sfmData, outputSfMDataFilename, sfmDataIO::ESfMData::ALL);

    return EXIT_SUCCESS;
}
