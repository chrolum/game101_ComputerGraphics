#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

int Rotate(Eigen::Matrix3f &t, double deg/*rad*/);
int translation_2d(Eigen::Matrix3f &m, int x, int y);

int main(){

    // Basic Example of cpp
    std::cout << "Example of cpp \n";
    float a = 1.0, b = 2.0;
    std::cout << a << std::endl;
    std::cout << a/b << std::endl;
    std::cout << std::sqrt(b) << std::endl;
    std::cout << std::acos(-1) << std::endl;
    std::cout << std::sin(30.0/180.0*acos(-1)) << std::endl;

    // Example of vector
    std::cout << "Example of vector \n";
    // vector definition
    Eigen::Vector3f v(1.0f,2.0f,3.0f);
    Eigen::Vector3f w(1.0f,0.0f,0.0f);
    // vector output
    std::cout << "Example of output \n";
    std::cout << v << std::endl;
    // vector add
    std::cout << "Example of add \n";
    std::cout << v + w << std::endl;
    // vector scalar multiply
    std::cout << "Example of scalar multiply \n";
    std::cout << v * 3.0f << std::endl;
    std::cout << 2.0f * v << std::endl;

    // Example of matrix
    std::cout << "Example of matrix \n";
    // matrix definition
    Eigen::Matrix3f i,j;
    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // matrix output
    std::cout << "Example of output \n";
    std::cout << i << std::endl;
    std::cout << "Example of output matrix add i + j\n";
    // matrix add i + j
    std::cout << i + j << std::endl;
    // matrix scalar multiply i * 2.0
    std::cout << "Example of output matrix i * 2.0\n";
    std::cout << i * 2.0 << std::endl;
    std::cout << "Example of output matrix add i * j\n";
    // matrix multiply i * j
    std::cout << i * j << std::endl;
    std::cout << "Example of output matrix add i * v\n";
    // matrix multiply vector i * v
    std::cout << i * v << std::endl;

    // homogeneous coordinates point(2,1)
    Eigen::Matrix3f t = Eigen::Matrix3f::Identity(3, 3);
    //Rotate 45 degree in rad
    double deg = 90.0/180.0*acos(-1);
    Rotate(t, deg);
    //translation
    Eigen::Vector3f p(1.0f, 1.0f, 1.0f);
    // translation_2d(t, 1, 2);
    p = t * p;

    std::cout << p << std::endl;

    return 0;
}

int Rotate(Eigen::Matrix3f &t, double deg/*rad*/)
{
    Eigen::Matrix3f R; //homogeneous form
    R << std::cos(deg), -std::sin(deg), 0, std::sin(deg), std::cos(deg), 0, 0, 0, 1;
    t = R * t;
    return 0;
}

int translation_2d(Eigen::Matrix3f &m, int x, int y)
{
    Eigen::Matrix3f t_xy;
    t_xy << 0, 0, x, 0, 0, y, 0, 0, 1;
    m = m + t_xy;
    return 0;
}