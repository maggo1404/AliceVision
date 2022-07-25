// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

using circle_info = std::pair<cv::Point2f, float>;

void model_explore(Ort::Session& session);

std::vector<std::string> get_images_paths(std::string path_images);
cv::Size resolution_verify(std::vector<std::string> files);

cv::Mat predict(Ort::Session& session, const std::string image_path, const cv::Size image_size);
cv::Mat compute_mask(Ort::Session& session, std::vector<std::string> files, const cv::Size image_size);

std::vector<std::pair<cv::Point2f, float>> compute_circles(const cv::Mat mask);

void export_json(std::string output_path, std::vector<circle_info> circles);
