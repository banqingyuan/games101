#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <iostream>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle_x, float rotation_angle_y, float rotation_angle_z)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    Eigen::Matrix4f rotation_x;
    Eigen::Matrix4f rotation_y;
    Eigen::Matrix4f rotation_z;
    rotation_angle_x = rotation_angle_x * 3.1415926 / 180;
    rotation_angle_y = rotation_angle_y * 3.1415926 / 180;
    rotation_angle_z = rotation_angle_z * 3.1415926 / 180;
    double cos_angle_z = std::cos(rotation_angle_z);
    double sin_angle_z = std::sin(rotation_angle_z);
    rotation_z << cos_angle_z, - sin_angle_z, 0, 0, 
                sin_angle_z, cos_angle_z, 0, 0, 
                0, 0, 1, 0,
                0, 0, 0, 1;
    model = rotation_z * model;

    double cos_angle_y = std::cos(rotation_angle_y);
    double sin_angle_y = std::sin(rotation_angle_y);
    rotation_y << cos_angle_y, 0, sin_angle_y, 0,
                    0, 1, 0, 0, 
                    - sin_angle_y, 0, cos_angle_y, 0,
                    0, 0, 0, 1;
    model = rotation_y * model;


    double cos_angle_x = std::cos(rotation_angle_x);
    double sin_angle_x = std::sin(rotation_angle_x);
    rotation_x << 1, 0, 0, 0,
                  0, cos_angle_x, - sin_angle_x, 0, 
                  0, sin_angle_x, cos_angle_x, 0, 
                  0, 0, 0, 1;
    model = rotation_x * model;


    return model;
}

Eigen::Matrix4f getRotation(Eigen::Vector3f n, double alpha) {
    Eigen::Matrix4f matrix;
    Eigen::Matrix3f _matrix;
    double cos_alp = std::cos(alpha);
    double sin_alp = std::sin(alpha);

    Eigen::Matrix3f unix_matrix;
    unix_matrix << 1, 0, 0,
                   0, 1, 0,
                   0, 0, 1;

    double n_x = n(0);
    double n_y = n(1);
    double n_z = n(2);

    Eigen::Matrix3f trans;
    trans << 0, -n_z, n_y, n_z, 0, - n_x, -n_y, n_x, 0; 
    trans = trans * sin_alp;

    _matrix = cos_alp * unix_matrix + (1 - cos_alp) * n * n.transpose() + sin_alp * trans;

    int i = 0;
    for(auto row : _matrix.rowwise()) {
        matrix(i, 0) = row(0);
        matrix(i, 1) = row(1);
        matrix(i, 2) = row(2);
        matrix(i, 3) = 0;
        i++;
    }
    matrix(3, 0) = 0;
    matrix(3, 1) = 0;
    matrix(3, 2) = 0;
    matrix(3, 3) = 1;
    std::cout << matrix;
    return matrix;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    //Eigen::Matrix4f persp2ortho;

    // persp2ortho << zNear, 0, 0, 0, 0, zNear, 0, 0, 0, 0, zNear + zFar, - (zNear * zFar), 0, 0, 1, 0;

    Eigen::Matrix4f oneStep;
    float tan_fov = std::tanf(eye_fov/2);
    oneStep << 1/tan_fov, 0, 0, 0,
                0, 1/tan_fov, 0, 0,
                0, 0, (zNear + zFar)/(zNear - zFar), - (2 * zNear * zFar)/(zNear - zFar),
                0, 0, 1, 0;
    projection = oneStep * projection;
    // projection = persp2ortho * projection;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle_x = 0;
    float angle_y = 0;
    float angle_z = 0;
    
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle_z = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, 0}, {0, 2, 0}, {-2, 0, 0}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}}; 

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle_x, angle_y, angle_z));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        Eigen::Vector3f n;
        n << 1, 0, 0;
        r.set_model(get_model_matrix(angle_x, angle_y, angle_z));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
        std::cout << '{' << angle_x << ' ' << angle_y << ' ' << angle_z << "}\n";
        switch (key)
        {
        case 'q':
            angle_x += 10;
            break;
        case 'e':
            angle_x -= 10;
            break;
        case 'a':
            angle_y += 10;
            break;
        case 'd':
            angle_y -= 10;
            break;
        case 'z':
            angle_z += 10;
            break;
        case 'c':
            angle_z -= 10;
            break;
        default:
            break;
        }
    }

    return 0;
}
