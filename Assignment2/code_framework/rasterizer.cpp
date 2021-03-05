// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    //Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f p(x, y, 0);//the pixel center
    Vector3f from, to, t_edge, p_edge;
    
    Vector3f last_product_res, curr_product_res;
    last_product_res = (_v[1] - _v[0]).cross(p - _v[0]);
    for (int i = 1; i <= 2; i++)
    {
        from = _v[i];
        to = _v[(i+1)%3];
        t_edge = to - from;
        p_edge = p - from;
        curr_product_res = t_edge.cross(p_edge);

        if (curr_product_res.dot(last_product_res) < 0)
            return false;
        
        last_product_res = curr_product_res;
    }
    return true;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

static float get_z_interpolated(float x, float y, const Triangle& t)
{
        auto v = t.toVector4();
        auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
        float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
        z_interpolated *= w_reciprocal;

        return z_interpolated;
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();// the vectri
    
    // Find out the bounding box of current triangle.
    float topf = -1, bottomf = INFINITY, leftf = INFINITY, rightf = -1;//box
    float x, y;
    for (auto vex : v)
    {
        x = vex[0], y = vex[1];
        topf = topf < y ? y : topf;
        bottomf = bottomf > y ? y : bottomf;
        leftf = leftf > x ? x : leftf;
        rightf = rightf < x ? x : rightf;
    }
    int top = (int)floor(topf), bottom = (int)ceil(bottomf), 
        left = (int)ceil(leftf), right = (int)floor(rightf);
    
    // iterate bounding box
    Vector2f sample;
    int sample_num = super_sample_size * super_sample_size;
    float sample_z_interpolated;
    for (size_t y = bottom; y <= top; y++)//y axis
    {
        for (size_t x = left; x <= right; x++)//x axis
        {
            bool hasUpdateZ = false;
            //Anti-aliasing: nxn super sampling
            //check each sample in which triangle
            for (int s_idx = 0; s_idx < sample_num; s_idx++)
            {
                sample = get_coordinate_by_sample_idx(x, y, s_idx);
                sample_z_interpolated = get_z_interpolated(sample.x(), sample.y(), t);
                int sample_idx = get_sample_idx(x, y, s_idx);
                if (insideTriangle(sample.x(), sample.y(), t.v) 
                    && sample_depth_buf[sample_idx] > sample_z_interpolated)
                {
                    sample_depth_buf[sample_idx] = sample_z_interpolated;
                    sample_fram_buf[sample_idx] = t.getColor();
                    hasUpdateZ = true;
                }
            }

            //Opt:: this pixel's sample not update any color, not need to re-mix color
            if (!hasUpdateZ)
                continue;

            //mix the all sample color in one pixel
            // int z_buffer_idx = get_index(x, y);
            Vector3f pixel_color(0, 0, 0);

            int mix_sample_idx = get_sample_idx(x, y, 0);
            for (int i = 0; i < super_sample_size*super_sample_size; i++)
            {
                pixel_color += sample_fram_buf[mix_sample_idx + i];
            }
            pixel_color = pixel_color / (super_sample_size*super_sample_size);
            set_pixel(Vector3f(x, y, 0), pixel_color);
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }

    if ((buff & rst::Buffers::SampleDepth) == rst::Buffers::SampleDepth)
    {
        std::fill(sample_depth_buf.begin(), sample_depth_buf.end(), std::numeric_limits<float>::infinity());
    }

    if ((buff & rst::Buffers::SampleColor) == rst::Buffers::SampleColor)
    {
        std::fill(sample_fram_buf.begin(), sample_fram_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    sample_depth_buf.resize(w * h * super_sample_size*super_sample_size);
    sample_fram_buf.resize(w * h * super_sample_size*super_sample_size);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

int rst::rasterizer::get_sample_idx(int x, int y, int s)
{
    //FIXME:: vaild this function
    return ((height-1-y)*width + x)*super_sample_size*super_sample_size + s;
}

Eigen::Vector2f rst::rasterizer::get_coordinate_by_sample_idx(float x, float y, int s)
{
    // idx increase from left to right, from bottom to top
    // Example: 3x3 sample
    // 6 | 7 | 8
    // 3 | 4 | 5
    // 0 | 1 | 2
    float step = pixel_width / super_sample_size;
    if (s == 0)
    {
        return Vector2f(x+step/2, y+step/2);
    }

    return Vector2f( (x+step/2+step*(s%super_sample_size)), 
        (y+step/2+step*(s/super_sample_size))
        );
}


// clang-format on