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
Vector3& Matrix3::operator [] (const uint8_t index) {
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

const Vector3& Matrix3::operator [] (const uint8_t index) const {
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
    return Matrix3(r1_ + m.r1_, r2_ + m.r2_, r3_ + m.r3_);
}

Matrix3 Matrix3::operator - (const Matrix3& m) const {
    return Matrix3(r1_ - m.r1_, r2_ - m.r2_, r3_ - m.r3_);
}

Matrix3 Matrix3::operator * (const Matrix3& m) const {
    return Matrix3(r1_ * m.r1_, r2_ * m.r2_, r3_ * m.r3_);
}

Vector3 Matrix3::operator * (const Vector3& v) const {
    return Vector3(r1_.dot(v), r2_.dot(v), r3_.dot(v));
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

bool Matrix3::operator != (const Matrix3& m) {
    return !(*this == m);
}

Matrix3& Matrix3::operator = (Matrix3&& m) {
    r1_ = std::move(m.r1_);
    r2_ = std::move(m.r2_);
    r3_ = std::move(m.r3_);

    return *this; // What about the original matrix? (m) It shouldn't have its rows available, if I'm not mistaken
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
Vector3 Matrix3::row(uint8_t index) const {
    return Vector3((*this)[index]);
}

Vector3 Matrix3::col(uint8_t index) const {
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
    Matrix3(Vector3(1,0,0), Vector3(0,1,0), Vector3(0,0,1));

const Matrix3 Matrix3::kZero = Matrix3();

const Matrix3 Matrix3::kOnes = 
    Matrix3(Vector3(1,1,1), Vector3(1,1,1), Vector3(1,1,1));




// Vector4


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





// Matrix 4

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

Vector4 Matrix4::operator * (const Vector4& v) const {
    return Vector4(r1_.dot(v), r2_.dot(v), r3_.dot(v), r4_.dot(v));
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

Matrix4 Matrix4::operator ^ (const Matrix4& m) const { 
    Matrix4 result;
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            result[i][j] = (*this)[i].dot(m[j]);
        }
    }
    return result;
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



// Isometry

Isometry Isometry::FromTranslation(const Vector3& v) {
    return Isometry(v);
}

Vector3 Isometry::RotateAround(const Vector3& v, const double rotation_angle) {

    const double cangle{std::cos(rotation_angle)};
    const double sangle{std::sin(rotation_angle)};

    Vector4 extended_vector(v); //scheinkerman add method Vector4 from Vector3

    Matrix4 x_rot_matrix(Vector4::kUnitX,
                        Vector4(0, cangle, -sangle, 0),
                        Vector4(0, sangle, cangle, 0), 
                        Vector4::kUnitW);
    Matrix4 y_rot_matrix(Vector4(cangle, 0, sangle, 0),
                        Vector4::kUnitY,
                        Vector4(-sangle, 0, cangle, 0), 
                        Vector4::kUnitW);
    Matrix4 z_rot_matrix(Vector4(cangle, -sangle, 0, 0),
                        Vector4(sangle, cangle, 0, 0),
                        Vector4::kUnitZ, 
                        Vector4::kUnitW);

    return ToVector3((x_rot_matrix ^ y_rot_matrix ^ z_rot_matrix) * extended_vector); //scheinkerman add method Vector3 from Vector4
}

Isometry Isometry::FromEulerAngles(const double, const double, const double) {
    //scheinkerman Implement
}


// Operators
Matrix3& Isometry::operator = (const Matrix3& m) {
    //scheinkerman Incorrect. Fix this.
    _rotation_matrix = ToMatrix4(m);
    _translation_matrix = ToMatrix4(m);
}

Vector3 Isometry::operator * (const Vector3& v) const {
    Vector4 extended_vector(v);
    return ToVector3((_translation_matrix ^ _rotation_matrix) * extended_vector);
}


// Misc

Vector3 Isometry::transform(const Vector3& v) const {
    return (*this) * v;
}

Isometry Isometry::compose(const Isometry& t) const {
    Vector3 trans(get_x_translation() + t.get_x_translation(),
                  get_y_translation() + t.get_y_translation(),
                  get_z_translation() + t.get_z_translation());
    
    return Isometry(trans, _rotation_matrix ^ t._rotation_matrix);
}


// Getters
Matrix3 Isometry::inverse() const {
    //scheinkerman Implement
}

Vector3 Isometry::traslation() const {
    return Vector3(get_x_translation(), get_x_translation(), get_x_translation());
}

Matrix3 Isometry::rotation() const {
    return ToMatrix3(_rotation_matrix); 
    //scheinkerman Implement Matrix3 from Matrix4
}

const double Isometry::get_x_translation() const {
    return _translation_matrix[0][3];
}

const double Isometry::get_y_translation() const {
    return _translation_matrix[1][3];
}

const double Isometry::get_z_translation() const {
    return _translation_matrix[2][3];
}

}  // namespace math
}  // namespace ekumen
