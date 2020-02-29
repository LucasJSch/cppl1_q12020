#include <cmath>
#include <cstdint>
#include "isometry.h"

namespace ekumen {
namespace math {

const double& Vector4::operator [] (const uint8_t index) const {
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
        case 3:
            return w_;
    }
}

double& Vector4::operator [] (const uint8_t index) {
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
const Vector4 Vector4::kUnitW = Vector4(0, 0, 1, 0);
const Vector4 Vector4::kZero = Vector4(0, 0, 0, 0);

}  // namespace math
}  // namespace ekumen
