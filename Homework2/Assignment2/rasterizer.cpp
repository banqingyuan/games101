// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <map>


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

// static Eigen::Matrix4f matrix_product(const Eigen::Vector4f a, const Eigen::Vector4f b) {
//     Eigen::Matrix4f res = a * b;
//     return res;
// }

static bool insideTriangle(int x, int y, const std::array<Eigen::Matrix<float, 4, 1, 0>, 3> _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Eigen::Vector4f point1, point2, point3, _point;
    Eigen::Vector4f array1, array2, array3, _array1, _array2, _array3;
    _point << x, y, 1, 1;
    point1 << _v[0][0], _v[0][1], _v[0][2], _v[0][3];
    point2 << _v[1][0], _v[1][1], _v[1][2], _v[1][3];
    point3 << _v[2][0], _v[2][1], _v[2][2], _v[2][3];

    array1 = point2 - point1;
    _array1 = _point - point1;

    array2 = point3 - point2;
    _array2 = _point - point2;

    array3 = point1 - point3;
    _array3 = _point - point3;

    Eigen::Vector4f res1, res2, res3;
    res1 = array1.cross3(_array1);
    res2 = array2.cross3(_array2);
    res3 = array3.cross3(_array3);

    if (res1[2] * res2[2] < 0 || res1[2] * res3[2] < 0) {
        return false;
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

std::map<std::pair<int, int>, float> z_buffer;
typedef std::pair<int, int> Point;

std::map<std::pair<int, int>, Eigen::Vector3f> color_map;

Eigen::Vector3f super_sampling(int x, int y, Eigen::Vector3f current_color, int cnt = 2) {
    //return current_color;

    Eigen::Vector3f color = current_color;
    for (int i = 0; i < cnt; i++) {
        for (int j = 0; j < cnt; j++) {
            if (i == 0 && j == 0) {
                continue;
            }
            Point p (x-i, y-j);
            Eigen::Vector3f temp_color;
            if (color_map.find(p) == color_map.end()) {
                temp_color << 0.0, 0.0, 0.0;
            } else {
                temp_color = color_map[p];
            }
            color += temp_color;
            
            // 不加下面这一段，则超采样只取左下，右上线条不会超采样，仍然有锯齿
            // 拿右上像素颜色，因为第一次右上都是背景色【黑】所以图像最暗，第二帧会把前一阵颜色和当前像素颜色平均一次，画面会亮一点，每一帧都会更亮一点。
            Point p2 (x+i, y+j);
            Eigen::Vector3f temp_color2;
            if (color_map.find(p2) == color_map.end()) {
                temp_color2 << 0.0, 0.0, 0.0;
            } else {
                temp_color2 = color_map[p2];
            }
            color += temp_color2;
        }
    }
    int all_cnt = cnt * cnt;

    // 注意这里是7次，不是9次，有点hard code
    auto new_color = color/7;
    return new_color;

}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    auto v = t.toVector4();

    float l_bound, r_bound, t_bound, b_bound; 
    l_bound = DBL_MAX;
    r_bound = -DBL_MAX;
    t_bound = -DBL_MAX;
    b_bound = DBL_MAX;
    for (auto it = v.begin(); it != v.end(); it ++) {
        Eigen::Vector4f point;
        point = *it;
        float x;
        x = point[0];
        float y;
        y = point[1];
        if (x > r_bound) {
            r_bound = ceil(x);
        }
        if (x < l_bound) {
            l_bound = floor(x);
        }
        if (y > t_bound) {
            t_bound = ceil(y);
        }
        if (y < b_bound) {
            b_bound = floor(y);
        }
    }

    for (double i = l_bound; i <= r_bound; i++) {
        for (double j = b_bound; j <= t_bound; j++) {
            if (insideTriangle(i, j, v)) {
                //If so, use the following code to get the interpolated z value.
                auto[alpha, beta, gamma] = computeBarycentric2D(i, j, t.v);
                Eigen::Vector3f color;
                color = alpha * t.color[0] * 255 + beta * t.color[1] * 255 + gamma * t.color[2] * 255;
                Eigen::Vector3f point;
                point << i, j, 1;
                // std::cout << color;
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                Point p (i, j);
                if (z_buffer.find(p) == z_buffer.end() || z_buffer[p] >= z_interpolated) {
                    auto new_color = super_sampling(i, j, color);
                    set_pixel(point, new_color);
                    //std::cout << new_color[0] << " " << new_color[1] << " " << new_color[2] << std::endl;
                    color_map[p] = new_color;
                    z_buffer[p] = z_interpolated;
                }
            }
        }
    }
    
    

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
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
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
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

// clang-format on