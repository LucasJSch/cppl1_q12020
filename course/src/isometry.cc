#include <cmath>
#include <cstdint>
#include "isometry.h"

namespace ekumen {
namespace math {

const double& Vector4::operator [] (const uint8_t index) const {
    if(index > 3) {
        throw "Invalid index number";
    }
    switch(index) {
        case 0:
            return x_;
        case 1:
            return y_;
        case 2:
            return z_;
        case 3:
            return w_;
    }
}

double& Vector4::operator [] (const uint8_t index) {
    if(index > 3) {
        throw "Invalid index number";
    }
    switch(index) {
        case 0:
            return x_;
        case 1:
            return y_;
        case 2:
            return z_;
        case 3:
            return w_;
    }
}

Vector4 Vector4::operator + (const Vector4& v) const {
    return Vector4(x_ + v.x_, y_ + v.y_, z_ + v.z_, w_ + v.w_);
}

Vector4 Vector4::operator - (const Vector4& v) const {
    return Vector4(x_ - v.x_, y_ - v.y_, z_ - v.z_, w_ - v.w_);
}

Vector4 Vector4::operator * (const Vector4& v) const {
    return Vector4(x_ * v.x_, y_ * v.y_, z_ * v.z_, w_ * v.w_);
}

Vector4 Vector4::operator * (const double& d) const {
    return Vector4(x_ * d, y_ * d, z_ * d, w_ * d);
}

Vector4 Vector4::operator / (const Vector4&v) const {
    return Vector4(x_ / v.x_, y_ / v.y_, z_ / v.z_, w_ / v.w_);
}

Vector4& Vector4::operator = (const Vector4& v) {
    x_ = v.x();
    y_ = v.y();
    z_ = v.z();
    w_ = v.w();
    return *this;
}

Vector4& ekumen::math::Vector4::operator += (const Vector4& v) { 
    x_ += v.x_;
    y_ += v.y_;
    z_ += v.z_;
    w_ += v.w_;
    return *this;
}

Vector4& ekumen::math::Vector4::operator -= (const Vector4& v) {
    x_ -= v.x_;
    y_ -= v.y_;
    z_ -= v.z_;
    w_ -= v.w_;
    return *this; 
}

Vector4& ekumen::math::Vector4::operator *= (const Vector4& v) {
    x_ *= v.x_;
    y_ *= v.y_;
    z_ *= v.z_;
    w_ *= v.w_;
    return *this; 
}

Vector4& ekumen::math::Vector4::operator /= (const Vector4& v) {
    x_ /= v.x_;
    y_ /= v.y_;
    z_ /= v.z_;
    w_ /= v.w_;
    return *this; 
}

bool ekumen::math::Vector4::operator == (const Vector4& v) const {
    return (x_ == v.x_) && (y_ == v.y_) && (z_ == v.z_) && (w_ == v.w_);
}

bool ekumen::math::Vector4::operator != (const Vector4& v) const {
    return !(*this == v);
}

bool ekumen::math::Vector4::operator == (const std::initializer_list<double>& list) const {
    if(list.size() != 4) {
        throw "Invalid initializer list size";
    }

    return (*this) == Vector4(list);
}

bool ekumen::math::Vector4::operator != (const std::initializer_list<double>& list) const {
    return !(*this == list);
}

// Computations
double Vector4::dot(const Vector4& v) const {
    return (x_ * v.x_) + (y_ * v.y_) + (z_ * v.z_) + (w_ * v.w_);
}

double Vector4::norm() const {
    return sqrt( (x_ * x_) + (y_ * y_) + (z_* z_) + (w_* w_));
}

// Class constants
const Vector4 Vector4::kUnitX = Vector4(1, 0, 0, 0);
const Vector4 Vector4::kUnitY = Vector4(0, 1, 0, 0);
const Vector4 Vector4::kUnitZ = Vector4(0, 0, 1, 0);
const Vector4 Vector4::kUnitW = Vector4(0, 0, 0, 1);
const Vector4 Vector4::kZero = Vector4(0, 0, 0, 0);

// Constructors
Matrix4::Matrix4(const std::initializer_list<double>& list) {
    if(list.size() != 16) {
        throw "Invalid initializer list size";
    }
    std::initializer_list<double>::iterator it = list.begin();

    r1_[0] = *it++;
    r1_[1] = *it++;
    r1_[2] = *it++;
    r1_[3] = *it++;

    r2_[0] = *it++;
    r2_[1] = *it++;
    r2_[2] = *it++;
    r2_[3] = *it++;

    r3_[0] = *it++;
    r3_[1] = *it++;
    r3_[2] = *it++;
    r3_[3] = *it++;

    r4_[0] = *it++;
    r4_[1] = *it++;
    r4_[2] = *it++;
    r4_[3] = *it;
}

Matrix4::Matrix4(const std::initializer_list<double>& l1,
                 const std::initializer_list<double>& l2,
                 const std::initializer_list<double>& l3,
                 const std::initializer_list<double>& l4) {
    r1_ = Vector4(l1);
    r2_ = Vector4(l2);
    r3_ = Vector4(l3);
    r4_ = Vector4(l4);
}

// Operators
Vector4& Matrix4::operator [] (const uint8_t index) {
    switch(index) {
        case 0:
            return r1_;
        case 1:
            return r2_;
        case 2:
            return r3_;
        case 3:
            return r4_;
        default:
            throw "Error. Invalid row index for Matrix4";
    }
}

const Vector4& Matrix4::operator [] (const uint8_t index) const {
    switch(index) {
        case 0:
            return r1_;
        case 1:
            return r2_;
        case 2:
            return r3_;
        case 3:
            return r4_;
        default:
            throw "Error. Invalid row index for Matrix4";
    }
}

Matrix4 Matrix4::operator + (const Matrix4& m) const {
    return Matrix4(r1_ + m.r1_, r2_ + m.r2_, r3_ + m.r3_, r4_ + m.r4_);
}

Matrix4 Matrix4::operator - (const Matrix4& m) const {
    return Matrix4(r1_ - m.r1_, r2_ - m.r2_, r3_ - m.r3_, r4_ - m.r4_);
}

Matrix4 Matrix4::operator * (const Matrix4& m) const {
    return Matrix4(r1_ * m.r1_, r2_ * m.r2_, r3_ * m.r3_, r4_ * m.r4_);
}

Matrix4 Matrix4::operator * (const double d) const {
    return Matrix4(r1_ * d, r2_ * d, r3_ * d, r4_ * d);
}

Matrix4 Matrix4::operator / (const Matrix4& m) const {
    return Matrix4(r1_ / m.r1_, r2_ / m.r2_, r3_ / m.r3_, r4_ / m.r4_);
}

Matrix4& Matrix4::operator = (const Matrix4& m) {
    r1_ = m.r1_;
    r2_ = m.r2_;
    r3_ = m.r3_;
    r4_ = m.r4_;
    return *this;
}

Matrix4& Matrix4::operator += (const Matrix4& m) {
    r1_ += m.r1_;
    r2_ += m.r2_;
    r3_ += m.r3_;
    r4_ += m.r4_;
    return *this;
}

Matrix4& Matrix4::operator -= (const Matrix4& m) {
    r1_ -= m.r1_;
    r2_ -= m.r2_;
    r3_ -= m.r3_;
    r4_ -= m.r4_;
    return *this;
}

Matrix4& Matrix4::operator *= (const Matrix4& m) {
    r1_ *= m.r1_;
    r2_ *= m.r2_;
    r3_ *= m.r3_;
    r4_ *= m.r4_;
    return *this;
}

Matrix4& Matrix4::operator /= (const Matrix4& m) {
    r1_ /= m.r1_;
    r2_ /= m.r2_;
    r3_ /= m.r3_;
    r4_ /= m.r4_;
    return *this;
}

bool Matrix4::operator == (const Matrix4& m) {
    if ((r1_ == m.r1_) && (r2_ == m.r2_) && (r3_ == m.r3_) && (r3_ == m.r3_)) {
        return true;
    }
    return false;
}

bool Matrix4::operator != (const Matrix4& m) {
    return !(*this == m);
}

Vector4 Matrix4::operator * (const Vector4& v) {
    return Vector4(r1_.dot(v), r2_.dot(v), r3_.dot(v), r4_.dot(v));
}

Matrix4& Matrix4::operator = (Matrix4&& m) {
    r1_ = std::move(m.r1_);
    r2_ = std::move(m.r2_);
    r3_ = std::move(m.r3_);
    r4_ = std::move(m.r4_);

    return *this; // What about the original matrix? (m) It shouldn't have its rows available, if I'm not mistaken
}

// Getters
Vector4 Matrix4::row(uint8_t index) const {
    return Vector4((*this)[index]);
}

Vector4 Matrix4::col(uint8_t index) const {
    if(index > 3) {
        throw "Error. Invalid column index for Matrix4";
    }
    return Vector4(r1_[index], r2_[index], r3_[index], r4_[index]);
}

// Computations
double Matrix4::det() const {
    //scheinkerman Implement
}

// Class constants
const Matrix4 Matrix4::kIdentity = 
    Matrix4(Vector4(1, 0, 0, 0), Vector4(0, 1, 0, 0),
            Vector4(0, 0, 1, 0), Vector4(0, 0, 0, 1));

const Matrix4 Matrix4::kZero = Matrix4();

const Matrix4 Matrix4::kOnes = 
    Matrix4(Vector4(1, 1, 1 , 1), Vector4(1, 1, 1, 1),
            Vector4(1, 1, 1, 1), Vector4(1, 1, 1, 1));
}  // namespace math
}  // namespace ekumen
