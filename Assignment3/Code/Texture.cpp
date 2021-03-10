#include "Texture.hpp"

Eigen::Vector3f Texture::getColorBilinear(float u, float v)
{
    float u_img = u * width;
    float v_img = (1 - v) * height; //different coords

    // the pixel are close to the edge of image, take color without interpolated
    if (u_img < 0.5 | u_img > (width - 0.5) | v_img < 0.5 | v_img > (height - 0.5))
    {
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    //get nearst 4 point

    //get the nearst integer point
    int tmp_u = round(u_img);
    int tmp_v = round(v_img);

    //u_uv is the rael texture integer index's color, not the sample center

    auto u11 = image_data.at<cv::Vec3b>(tmp_v, tmp_u);
    auto u01 = image_data.at<cv::Vec3b>(tmp_v, tmp_u-1);
    auto u00 = image_data.at<cv::Vec3b>(tmp_v-1, tmp_u-1);
    auto u10 = image_data.at<cv::Vec3b>(tmp_v-1, tmp_u);

    auto convert_to_color = [](auto c) {return Eigen::Vector3f(c[0], c[1], c[2]);};

    auto c11 = convert_to_color(u11);
    auto c01 = convert_to_color(u01);
    auto c00 = convert_to_color(u00);
    auto c10 = convert_to_color(u10);

    //get s and t
    // c01 | c11
    // c00 | c10
    float s = u_img - (tmp_u - 1  + 0.5f);
    float t = v_img - (tmp_v - 1 + 0.5f);

    assert(s < 1);
    assert(t < 1);

    auto c1 = lerp_color(s, c01, c11);
    auto c2 = lerp_color(s, c00, c10);
    auto final_c = lerp_color(t, c2, c1);

    return final_c;
}