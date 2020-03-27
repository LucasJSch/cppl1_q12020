#include <cmath>
#include <cstdint>
#include "isometry.h"

namespace ekumen {
namespace math {

const double& Vector3::operator [] (const int index) const {
    if(index > 2) {
        throw "Invalid index number";
    }
    switch(index) {
        case 0:
            return x_;
        case 1:
            return y_;
        case 2:
            return z_;
    }
}

double& Vector3::operator [] (const int index) {
    if(index > 2) {
        throw "Invalid index number";
    }
    switch(index) {
        case 0:
            return x_;
        case 1:
            return y_;
        case 2:
            return z_;
    }
}

Vector3 Vector3::operator + (const Vector3& v) const {
    return Vector3(x_ + v.x_, y_ + v.y_, z_ + v.z_);
}

Vector3 Vector3::operator - (const Vector3& v) const {
    return Vector3(x_ - v.x_, y_ - v.y_, z_ - v.z_);
}

Vector3 Vector3::operator * (const Vector3& v) const {
    return Vector3(x_ * v.x_, y_ * v.y_, z_ * v.z_);
}

Vector3 Vector3::operator * (const double& d) const {
    return Vector3(x_ * d, y_ * d, z_ * d);
}

Vector3 Vector3::operator / (const Vector3&v) const {
    return Vector3(x_ / v.x_, y_ / v.y_, z_ / v.z_);
}

Vector3 Vector3::operator / (const double& d) const {
    return Vector3(x_ / d, y_ / d, z_ / d);
}

Vector3& Vector3::operator = (const Vector3& v) {
    x_ = v.x();
    y_ = v.y();
    z_ = v.z();
    return *this;
}

Vector3& ekumen::math::Vector3::operator += (const Vector3& v) { 
    x_ += v.x_;
    y_ += v.y_;
    z_ += v.z_;
    return *this;
}

Vector3& ekumen::math::Vector3::operator -= (const Vector3& v) {
    x_ -= v.x_;
    y_ -= v.y_;
    z_ -= v.z_;
    return *this; 
}

Vector3& ekumen::math::Vector3::operator *= (const Vector3& v) {
    x_ *= v.x_;
    y_ *= v.y_;
    z_ *= v.z_;
    return *this; 
}

Vector3& ekumen::math::Vector3::operator /= (const Vector3& v) {
    x_ /= v.x_;
    y_ /= v.y_;
    z_ /= v.z_;
    return *this; 
}

bool ekumen::math::Vector3::operator == (const Vector3& v) const {
    return (x_ == v.x_) && (y_ == v.y_) && (z_ == v.z_);
}

bool ekumen::math::Vector3::operator != (const Vector3& v) const {
    return !(*this == v);
}

bool ekumen::math::Vector3::operator == (const std::initializer_list<double>& list) const {
    if(list.size() != 3) {
        throw "Invalid initializer list size";
    }

    return (*this) == Vector3(list);
}

bool ekumen::math::Vector3::operator != (const std::initializer_list<double>& list) const {
    return !(*this == list);
}

// Computations
double Vector3::dot(const Vector3& v) const {
    return (x_ * v.x_) + (y_ * v.y_) + (z_ * v.z_);
}

double Vector3::norm() const {
    return sqrt( (x_ * x_) + (y_ * y_) + (z_* z_) );
}

Vector3 Vector3::cross(const Vector3& v) const {
    return Vector3(
    y_ * v.z_ - z_ * v.y_, 
    z_ * v.x_ - x_ * v.z_,
    x_ * v.y_ - y_ * v.x_);
}

// Class constants
const Vector3 Vector3::kUnitX = Vector3(1,0,0);
const Vector3 Vector3::kUnitY = Vector3(0,1,0);
const Vector3 Vector3::kUnitZ = Vector3(0,0,1);
const Vector3 Vector3::kZero = Vector3(0,0,0);

// Matrix3 class methods

// Constructors
Matrix3::Matrix3(const std::initializer_list<double>& list) {
    if(list.size() != 9) {
        throw "Invalid initializer list size";
    }
    std::initializer_list<double>::iterator it = list.begin();

    r1_[0] = *it++;
    r1_[1] = *it++;
    r1_[2] = *it++;

    r2_[0] = *it++;
    r2_[1] = *it++;
    r2_[2] = *it++;

    r3_[0] = *it++;
    r3_[1] = *it++;
    r3_[2] = *it++;
}

Matrix3::Matrix3(const std::initializer_list<double>& l1,
                 const std::initializer_list<double>& l2,
                 const std::initializer_list<double>& l3) {
    r1_ = Vector3(l1);
    r2_ = Vector3(l2);
    r3_ = Vector3(l3);
}

// Operators
Vector3& Matrix3::operator [] (const uint32_t index) {
    switch(index) {
        case 0:
            return r1_;
        case 1:
            return r2_;
        case 2:
            return r3_;
        default:
            throw "Error. Invalid row index for Matrix3";
    }
}

const Vector3& Matrix3::operator [] (const uint32_t index) const {
    switch(index) {
        case 0:
            return r1_;
        case 1:
            return r2_;
        case 2:
            return r3_;
        default:
            throw "Error. Invalid row index for Matrix3";
    }
}

Matrix3 Matrix3::operator + (const Matrix3& m) const {
    Matrix3 lhs(*this);
    lhs += m;
    return lhs;
}

Matrix3 Matrix3::operator - (const Matrix3& m) const {
    Matrix3 lhs(*this);
    lhs -= m;
    return lhs;
}

Matrix3 Matrix3::operator * (const Matrix3& m) const {
    Matrix3 lhs(*this);
    lhs *= m;
    return lhs;
}

Matrix3 Matrix3::operator * (const double d) const {
    return Matrix3(r1_ * d, r2_ * d, r3_ * d);
}

Matrix3 Matrix3::operator / (const Matrix3& m) const {
    return Matrix3(r1_ / m.r1_, r2_ / m.r2_, r3_ / m.r3_);
}

Matrix3 Matrix3::operator / (const double d) const {
    return Matrix3(r1_ / d, r2_ / d, r3_ / d);
}

Matrix3& Matrix3::operator = (const Matrix3& m) {
    r1_ = m.r1_;
    r2_ = m.r2_;
    r3_ = m.r3_;
    return *this;
}

Matrix3& Matrix3::operator += (const Matrix3& m) {
    r1_ += m.r1_;
    r2_ += m.r2_;
    r3_ += m.r3_;
    return *this;
}

Matrix3& Matrix3::operator -= (const Matrix3& m) {
    r1_ -= m.r1_;
    r2_ -= m.r2_;
    r3_ -= m.r3_;
    return *this;
}

Matrix3& Matrix3::operator *= (const Matrix3& m) {
    r1_ *= m.r1_;
    r2_ *= m.r2_;
    r3_ *= m.r3_;
    return *this;
}

Matrix3& Matrix3::operator /= (const Matrix3& m) {
    r1_ /= m.r1_;
    r2_ /= m.r2_;
    r3_ /= m.r3_;
    return *this;
}

bool Matrix3::operator == (const Matrix3& m) const {
    if ((r1_ == m.r1_) && (r2_ == m.r2_) && (r3_ == m.r3_)) {
        return true;
    }
    return false;
}

bool Matrix3::operator != (const Matrix3& m) const {
    return !(*this == m);
}

Vector3 Matrix3::operator * (const Vector3& v) const {
    return Vector3(r1_.dot(v), r2_.dot(v), r3_.dot(v));
}

Matrix3& Matrix3::operator = (Matrix3&& m) {
    r1_ = std::move(m.r1_);
    r2_ = std::move(m.r2_);
    r3_ = std::move(m.r3_);

    return *this;
}

Matrix3 Matrix3::operator ^ (const Matrix3& m) const { 
    Matrix3 result;
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            result[i][j] = (*this)[i].dot(m[j]);
        }
    }
    return result;
}

// Getters
Vector3 Matrix3::row(uint32_t index) const {
    return (*this)[index];
}

Vector3 Matrix3::col(uint32_t index) const {
    if(index > 2) {
        throw "Error. Invalid column index for Matrix3";
    }
    return Vector3(r1_[index], r2_[index], r3_[index]);
}

// Computations
double Matrix3::det() const {
    double subdet1 = r2_[1] * r3_[2] - r2_[2] * r3_[1];
    double subdet2 = r2_[0] * r3_[2] - r2_[2] * r3_[0];
    double subdet3 = r2_[0] * r3_[1] - r2_[1] * r3_[0];

    return r1_[0] * subdet1 - r1_[1] * subdet2 + r1_[2] * subdet3;
}

Matrix3 Matrix3::inverse() const {
    Matrix3 result;

    result[0][0] = r2_[1] * r3_[2] - r2_[2] * r3_[1];
    result[0][1] = r1_[2] * r3_[2] - r1_[1] * r3_[2];
    result[0][2] = r1_[1] * r2_[2] - r2_[1] * r1_[2];

    result[1][0] = r2_[2] * r3_[0] - r2_[0] * r3_[2];
    result[1][1] = r1_[0] * r3_[2] - r1_[2] * r3_[0];
    result[1][2] = r1_[2] * r2_[0] - r1_[0] * r2_[2];

    result[2][0] = r2_[0] * r3_[1] - r2_[1] * r3_[0];
    result[2][1] = r1_[1] * r3_[0] - r1_[0] * r3_[1];
    result[2][2] = r1_[0] * r2_[1] - r1_[1] * r2_[0];

    return result/((*this).det());
}

// Class constants
const Matrix3 Matrix3::kIdentity = 
    Matrix3(Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1));

const Matrix3 Matrix3::kZero = 
    Matrix3(Vector3(0, 0, 0), Vector3(0, 0, 0), Vector3(0, 0, 0));

const Matrix3 Matrix3::kOnes = 
    Matrix3(Vector3(1, 1, 1), Vector3(1, 1, 1), Vector3(1, 1, 1));

// Isometry class methods

Isometry Isometry::FromTranslation(const Vector3& v) {
    return Isometry(v);
}

// If the axis vector is not a unit vector, the function will normalize it.
Isometry Isometry::RotateAround(const Vector3& axis, const double rotation_angle) {
    if (axis == Vector3()) {
        throw "Invalid axis vector.";
    }

    const double cangle{std::cos(rotation_angle)};
    const double sangle{std::sin(rotation_angle)};

    // Normalized axis.
    Vector3 norm_axis = axis/axis.norm();
    // Axis with squared values.
    Vector3 sq_axis = norm_axis * norm_axis;

    Matrix3 rot_matrix = {cangle + sq_axis.x() * (1 - cangle),
                          norm_axis.x() * norm_axis.y() * (1 - cangle) - norm_axis.z() * sangle,
                          norm_axis.x() * norm_axis.z() * (1 - cangle) + norm_axis.y() * sangle,

                          norm_axis.y() * norm_axis.x() * (1 - cangle) + norm_axis.z() * sangle,
                          cangle + sq_axis.y() * (1 - cangle),
                          norm_axis.y() * norm_axis.z() * (1 - cangle) - norm_axis.x() * sangle,

                          norm_axis.z() * norm_axis.x() * (1 - cangle) - norm_axis.y() * sangle,
                          norm_axis.z() * norm_axis.y() * (1 - cangle) + norm_axis.x() * sangle,
                          cangle + sq_axis.z() * (1 - cangle)};

    return Isometry(Vector3(), rot_matrix);
}

Isometry Isometry::FromEulerAngles(const double roll, 
                                   const double pitch,
                                   const double yaw) {
    return Isometry(Isometry::RotateAround(Vector3::kUnitX, roll) *
                    Isometry::RotateAround(Vector3::kUnitY, pitch) *
                    Isometry::RotateAround(Vector3::kUnitZ, yaw));
}

// Operators
Isometry& Isometry::operator = (const Matrix3& m) {
    translation_vector_ = Vector3();
    rotation_matrix_ = m;
    return *this;
}

Vector3 Isometry::operator * (const Vector3& v) const {
    return (rotation_matrix_ * v) + translation_vector_;
}

Isometry Isometry::operator * (const Isometry& t) const {
    return Isometry((rotation() * t.translation()) + translation(),
                    rotation() ^ t.rotation());
}

bool Isometry::operator == (const Isometry& t) const {
    return (rotation() == t.rotation()) && 
           (translation() == t.translation());
}

// Misc

Vector3 Isometry::transform(const Vector3& v) const {
    return (*this) * v;
}

Isometry Isometry::compose(const Isometry& t) const {
    return (*this) * t;
}

// Getters
Isometry Isometry::inverse() const {
    return Isometry{(rotation_matrix_.inverse() * translation_vector_) * -1,
                    rotation_matrix_.inverse()};
}

const Vector3& Isometry::translation() const {
    return translation_vector_;
}

const Matrix3& Isometry::rotation() const {
    return rotation_matrix_; 
}

}  // namespace math
}  // namespace ekumen
