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

Isometry Isometry::RotateAround(const Vector3& v, const double rotation_angle) {

    const double cangle{std::cos(rotation_angle)};
    const double sangle{std::sin(rotation_angle)};

    Matrix3 rot_matrix;

    if (v == Vector3::kUnitX) {
        rot_matrix = Matrix3(Vector3::kUnitX,
                             Vector3(0, cangle, -sangle),
                             Vector3(0, sangle, cangle));
    }
    else if (v == Vector3::kUnitY) {
        rot_matrix = Matrix3(Vector3(cangle, 0, sangle),
                             Vector3::kUnitY,
                             Vector3(-sangle, 0, cangle));
    }
    else if (v == Vector3::kUnitZ) {
        rot_matrix = Matrix3(Vector3(cangle, -sangle, 0),
                             Vector3(sangle, cangle, 0),
                             Vector3::kUnitZ);
    }
    else {
        throw "Invalid rotation axis";
    }

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
Matrix3& Isometry::operator = (const Matrix3& m) {
    translation_vector_ = Vector3();
    rotation_matrix_ = m;
    return rotation_matrix_;
}

Vector3 Isometry::operator * (const Vector3& v) const {
    return Vector3(rotation_matrix_ * (v + translation_vector_));
}

Isometry Isometry::operator * (const Isometry& t) const {
    return Isometry(translation() + t.translation(),
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
    Vector3 trans(get_x_translation() + t.get_x_translation(),
                  get_y_translation() + t.get_y_translation(),
                  get_z_translation() + t.get_z_translation());
    
    return Isometry(trans, rotation_matrix_ ^ t.rotation_matrix_);
}

// Getters
Isometry Isometry::inverse() const {
    return Isometry(this->translation() * -1);
}

Vector3 Isometry::translation() const {
    return Vector3(get_x_translation(), get_y_translation(), get_z_translation());
}

Matrix3 Isometry::rotation() const {
    return rotation_matrix_; 
}

const double Isometry::get_x_translation() const {
    return translation_vector_[0];
}

const double Isometry::get_y_translation() const {
    return translation_vector_[1];
}

const double Isometry::get_z_translation() const {
    return translation_vector_[2];
}

}  // namespace math
}  // namespace ekumen
