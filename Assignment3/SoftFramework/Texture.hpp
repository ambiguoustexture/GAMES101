//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        auto  u_img  = u * width,
              v_img  = (1 - v) * height;
        float u_floo = std::floor(u_img),
              u_ceil = std::min((float)width,  std::ceil(u_img)),
              v_floo = std::floor(v_img),
              v_ceil = std::min((float)height, std::ceil(v_img)),
              s      = (u_img - u_floo) / (u_ceil - u_floo),
              t      = (v_img - v_ceil) / (v_floo - v_ceil);
        auto u00     = image_data.at<cv::Vec3b>(v_floo, u_floo),
             u01     = image_data.at<cv::Vec3b>(v_floo, u_ceil),
             u10     = image_data.at<cv::Vec3b>(v_ceil, u_floo),
             u11     = image_data.at<cv::Vec3b>(v_ceil, u_ceil),
             u0      = u00 + s * (u10 - u00),
             u1      = u01 + s * (u11 - u01),
             color   = u0  + t * (u1  - u0);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
};
#endif //RASTERIZER_TEXTURE_H
