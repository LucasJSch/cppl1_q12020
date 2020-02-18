#include <cmath>
#include <cstdint>
#include "isometry.h"

namespace ekumen {
namespace math {

// Vector3 class methods

double& Vector3::operator [] (const int index) const {
    if(index > Vector3::MAX_INDEX) {
        //throw error
    }
    return *(data_[index]);
}

double& Vector3::operator [] (const int index) {
    if(index > Vector3::MAX_INDEX) {
        //throw error
    }
    return *(data_[index]);
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

Vector3 Vector3::operator / (const Vector3&v) const {
    return Vector3(x_ / v.x_, y_ / v.y_, z_ / v.z_);
}

Vector3 Vector3::operator = (const Vector3& v) {
    x_ = v.x();
    y_ = v.y();
    z_ = v.z();
    return Vector3(v); 
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
    // introduce tolerance
    return (x_ != v.x_) || (y_ != v.y_) || (z_ != v.z_);
}

bool ekumen::math::Vector3::operator == (const std::initializer_list<double>& list) const {
    //check list correct size (3)
    //check for loops that avoids SIGSEGV
    std::initializer_list<double>::iterator it;
    int i;
    for(it = list.begin(), i = 0; it != list.end();) {
         if(*it++ != (*this)[i++]) {
            return false;
        }
    }
    return true;
}

bool ekumen::math::Vector3::operator != (const std::initializer_list<double>& list) const {
    //check list correct size (3)
    //check for loops that avoids SIGSEGV
    std::initializer_list<double>::iterator it;
    int i;
    for(it = list.begin(), i = 0; it != list.end();) {
         if(*it++ != (*this)[i++]) {
            return true;
        }
    }
    return false;
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
    //check list size
    //implement
}


// Operators
const Vector3 Matrix3::operator [] (const int index) const {
    // check correct index value
    return Vector3(this->row(index));
}

//check for returning matrix reference instead of new matrix

Matrix3 Matrix3::operator + (const Matrix3& m) const {
    return Matrix3(r1_ + m.r1_, r2_ + m.r2_, r3_ + m.r3_);
}

Matrix3 Matrix3::operator - (const Matrix3& m) const {
    return Matrix3(r1_ - m.r1_, r2_ - m.r2_, r3_ - m.r3_);
}

Matrix3 Matrix3::operator * (const Matrix3& m) const {
    return Matrix3(r1_ * m.r1_, r2_ * m.r2_, r3_ * m.r3_);
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

bool Matrix3::operator == (const Matrix3& m) {
    if ((r1_ == m.r1_) && (r2_ == m.r2_) && (r3_ == m.r3_)) {
        return true;
    }
    return false;
}

// Getters
Vector3 Matrix3::row(int index) const {
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

Vector3 Matrix3::col(int index) const {
    if(index > 2) {
        throw "Error. Invalid column index for Matrix3";
    }
    return Vector3(r1_[index], r2_[index], r3_[index]);
}

// Computations
//double Matrix3::det() const;

// Class constants
const Matrix3 Matrix3::kIdentity = 
    Matrix3(Vector3(1,0,0), Vector3(0,1,0), Vector3(0,0,1)); //check line style guide

const Matrix3 Matrix3::kZero = Matrix3();

const Matrix3 Matrix3::kOnes = 
    Matrix3(Vector3(1,1,1), Vector3(1,1,1), Vector3(1,1,1));  //check line style guide


}  // namespace math
}  // namespace ekumen
