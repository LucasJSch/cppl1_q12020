#ifndef ISOMETRY_H
#define ISOMETRY_H

#include <cstdint>
#include <iostream>
#include <ostream>

namespace ekumen {
namespace math {

class Vector3{
    public:
        // Constructors
        Vector3(double x = 0, double y = 0, double z = 0) : x_{x}, y_{y}, z_{z} {};
        Vector3(const Vector3& v) : Vector3(v.x_, v.y_, v.z_) {};
        Vector3(const std::initializer_list<double>& l) {
            if(l.size() != 3) {
                throw "Invalid initializer list size";
            }
            std::initializer_list<double>::iterator it = l.begin();
            x_ = *it++;
            y_ = *it++;
            z_ = *it;
        }

        // Operators
        double& operator [] (const int);
        const double& operator [] (const int) const;
        Vector3 operator + (const Vector3&) const;
        Vector3 operator - (const Vector3&) const;
        Vector3 operator * (const Vector3&) const;
        Vector3 operator * (const double&) const;
        Vector3 operator / (const Vector3&) const;
        Vector3& operator = (const Vector3&);
        Vector3& operator += (const Vector3&);
        Vector3& operator -= (const Vector3&);
        Vector3& operator *= (const Vector3&);
        Vector3& operator /= (const Vector3&);
        bool operator == (const Vector3&) const;
        bool operator != (const Vector3&) const;
        bool operator == (const std::initializer_list<double>& list) const;
        bool operator != (const std::initializer_list<double>& list) const;

        friend Vector3 operator * (const double d, const Vector3& v) {
            return Vector3(v.x_ * d, v.y_ * d, v.z_ * d);
        }

        friend std::ostream& operator << (std::ostream& os, const Vector3& v) {
            os << std::string("(x: ") << v.x() << ", y: " << v.y() << ", z: " << v.z() << ")";
            return os;
        }

        // Getters
        double x() const {return x_;};
        double y() const {return y_;};
        double z() const {return z_;};

        // Setters
        double& x() {return x_;};
        double& y() {return y_;};
        double& z() {return z_;};

        // Computations
        double dot(const Vector3&) const;
        double norm() const;
        Vector3 cross(const Vector3&) const;

        // Class constants
        static const Vector3 kUnitX;
        static const Vector3 kUnitY;
        static const Vector3 kUnitZ;
        static const Vector3 kZero;

    private:
        // Attributes
        double x_;
        double y_;
        double z_;
};  // Vector3

class Matrix3{
    public:
        // Constructors
        Matrix3(const Vector3& r1 = Vector3(), 
                const Vector3& r2 = Vector3(), const Vector3& r3 = Vector3())
                 : r1_{r1}, r2_{r2}, r3_{r3} {};
        Matrix3(const Matrix3& m) : Matrix3(m.row(0), m.row(1), m.row(2)) {};
        Matrix3(const std::initializer_list<double>& list);
        Matrix3(const std::initializer_list<double>&,
                const std::initializer_list<double>&,
                const std::initializer_list<double>&);
        Matrix3(Matrix3&& m) : Matrix3(m.row(0), m.row(1), m.row(2)){}

        // Operators
        Vector3& operator [] (const uint8_t);
        const Vector3& operator [] (const uint8_t) const;
        Matrix3 operator + (const Matrix3&) const;
        Matrix3 operator - (const Matrix3&) const;
        Matrix3 operator * (const Matrix3&) const;
        Matrix3 operator * (const double) const;
        Matrix3 operator / (const Matrix3&) const;
        Matrix3& operator = (const Matrix3&);
        Matrix3& operator += (const Matrix3&);
        Matrix3& operator -= (const Matrix3&);
        Matrix3& operator *= (const Matrix3&);
        Matrix3& operator /= (const Matrix3&);
        bool operator == (const Matrix3&);
        bool operator != (const Matrix3&);
        Vector3 operator * (const Vector3&) const;
        Matrix3& operator = (Matrix3&&);
        // Matrix product
        Matrix3 operator ^ (const Matrix3&) const;

        friend Matrix3 operator * (const double d, const Matrix3& m) {
                return Matrix3(m.r1_ * d, m.r2_ * d, m.r3_ * d);
        }

        friend std::ostream& operator << (std::ostream& os, const Matrix3& m) {
            os << std::string("[");
            for(int i = 0; i < 3; i++) {
                os << m.row(i);
            }
            return os;
        }

        // Getters
        Vector3 row(uint8_t index) const;
        Vector3 col(uint8_t index) const;

        // Computations
        double det() const;

        // Class constants
        static const Matrix3 kIdentity;
        static const Matrix3 kZero;
        static const Matrix3 kOnes;

    private:
        // Ordered by rows.
        Vector3 r1_, r2_, r3_;

};  // Matrix3

class Vector4{
    public:
        // Constructors
        Vector4(double x = 0, double y = 0, double z = 0, double w = 0) : x_{x}, y_{y}, z_{z}, w_{w} {}
        Vector4(const Vector4& v) : Vector4(v.x_, v.y_, v.z_, v.w_) {}
        Vector4(const std::initializer_list<double>& l) {
            if(l.size() != 4) {
                throw "Invalid initializer list size";
            }
            std::initializer_list<double>::iterator it = l.begin();
            x_ = *it++;
            y_ = *it++;
            z_ = *it++;
            w_ = *it;
        }
        Vector4(const Vector3& v, double d = 0) : x_{v.x()}, y_{v.y()}, z_{v.z()}, w_{d} {}

        // Operators
        double& operator [] (const uint8_t);
        const double& operator [] (const uint8_t) const;
        Vector4 operator + (const Vector4&) const;
        Vector4 operator - (const Vector4&) const;
        Vector4 operator * (const Vector4&) const;
        Vector4 operator * (const double&) const;
        Vector4 operator / (const Vector4&) const;
        Vector4& operator = (const Vector4&);
        Vector4& operator += (const Vector4&);
        Vector4& operator -= (const Vector4&);
        Vector4& operator *= (const Vector4&);
        Vector4& operator /= (const Vector4&);
        bool operator == (const Vector4&) const;
        bool operator != (const Vector4&) const;
        bool operator == (const std::initializer_list<double>& list) const;
        bool operator != (const std::initializer_list<double>& list) const;

        friend Vector4 operator * (const double d, const Vector4& v) {
            return Vector4(v.x_ * d, v.y_ * d, v.z_ * d, v.w_ * d);
        }

        friend std::ostream& operator << (std::ostream& os, const Vector4& v) {
            os << std::string("(x: ") << v.x() << ", y: " << v.y() << ", z: " << v.z() << ", w: " << v.w() << ")";
            return os;
        }

        // Getters
        const double& x() const {return x_;};
        const double& y() const {return y_;};
        const double& z() const {return z_;};
        const double& w() const {return w_;};

        // Setters
        double& x() {return x_;};
        double& y() {return y_;};
        double& z() {return z_;};
        double& w() {return w_;};

        // Computations
        double dot(const Vector4&) const;
        double norm() const;

        // Class constants
        static const Vector4 kUnitX;
        static const Vector4 kUnitY;
        static const Vector4 kUnitZ;
        static const Vector4 kUnitW;
        static const Vector4 kZero;

    private:
        // Attributes
        double x_;
        double y_;
        double z_;
        double w_;
};

class Matrix4{
    public:
        // Constructors
        Matrix4(const Vector4& r1 = Vector4(), 
                const Vector4& r2 = Vector4(), 
                const Vector4& r3 = Vector4(),
                const Vector4& r4 = Vector4())
                 : r1_{r1}, r2_{r2}, r3_{r3}, r4_{r4} {}
        Matrix4(const Matrix4& m) : Matrix4(m.row(0), m.row(1), m.row(2), m.row(3)) {}
        Matrix4(const std::initializer_list<double>& list);
        Matrix4(const std::initializer_list<double>&,
                const std::initializer_list<double>&,
                const std::initializer_list<double>&,
                const std::initializer_list<double>&);
        Matrix4(Matrix4&& m) : Matrix4(m.row(0), m.row(1), m.row(2), m.row(3)){}
        Matrix4(const Matrix3& m) : r1_{m.row(0)}, r2_{m.row(1)}, r3_{m.row(2)}, r4_{0, 0, 0, 1} {}
        Matrix4 operator ^ (const Matrix4&) const;

        // Operators
        Vector4& operator [] (const uint8_t);
        const Vector4& operator [] (const uint8_t) const;
        Matrix4 operator + (const Matrix4&) const;
        Matrix4 operator - (const Matrix4&) const;
        Matrix4 operator * (const Matrix4&) const;
        Vector4 operator * (const ekumen::math::Vector4&) const;
        Matrix4 operator * (const double) const;
        Matrix4 operator / (const Matrix4&) const;
        Matrix4& operator = (const Matrix4&);
        Matrix4& operator += (const Matrix4&);
        Matrix4& operator -= (const Matrix4&);
        Matrix4& operator *= (const Matrix4&);
        Matrix4& operator /= (const Matrix4&);
        bool operator == (const Matrix4&) const;
        bool operator != (const Matrix4&) const;
        Vector4 operator * (const Vector4&);
        Matrix4& operator = (Matrix4&&);

        friend Matrix4 operator * (const double d, const Matrix4& m) {
                return Matrix4(m.r1_ * d, m.r2_ * d, m.r3_ * d);
        }

        friend std::ostream& operator << (std::ostream& os, const Matrix4& m) {
            os << std::string("[");
            for(int i = 0; i < 4; i++) {
                os << m.row(i);
            }
            return os;
        }

        // Getters
        Vector4 row(uint8_t index) const;
        Vector4 col(uint8_t index) const;

        // Computations
        double det() const;

        // Class constants
        static const Matrix4 kIdentity;
        static const Matrix4 kZero;
        static const Matrix4 kOnes;

    private:
        // Ordered by rows.
        Vector4 r1_, r2_, r3_, r4_;

};

class Isometry {

    public:
        // Constructors
        Isometry(const Vector3& translation_vector = Vector3(), 
                 const Matrix3& rotation_matrix = Matrix3::kIdentity) :
                translation_matrix_{{1, 0, 0, translation_vector.x()},
                                    {0, 1, 0, translation_vector.y()},
                                    {0, 0, 1, translation_vector.z()},
                                    Vector4::kUnitW},
                rotation_matrix_{ToMatrix4(rotation_matrix)} {}

        Isometry(const Vector4& translation_vector = Vector4(),
                 const Matrix4& rotation_matrix = Matrix4::kIdentity) :
                translation_matrix_{{1, 0, 0, translation_vector.x()},
                                    {0, 1, 0, translation_vector.y()},
                                    {0, 0, 1, translation_vector.z()},
                                    Vector4::kUnitW},
                rotation_matrix_{rotation_matrix} {}
        Isometry(const Isometry& t) : 
            translation_matrix_{t.translation_matrix_},
            rotation_matrix_{t.rotation_matrix_} {}
        Isometry(const Matrix4& trans_matrix, const Matrix4& rot_matrix) : 
            translation_matrix_{trans_matrix},
            rotation_matrix_{rot_matrix} {}

        // Static operations
        static Isometry FromTranslation(const Vector3&);
        static Isometry RotateAround(const Vector3&, const double);
        static Isometry FromEulerAngles(const double, const double, const double);

        // Operators
        Matrix3& operator = (const Matrix3&);
        Vector3 operator * (const Vector3&) const;
        Isometry operator * (const Isometry&) const;
        bool operator == (const Isometry&) const;

        // Misc
        Vector3 transform(const Vector3&) const;
        Isometry compose(const Isometry&) const;

        // Getters
        Vector3 translation() const;
        Matrix3 rotation() const;
        Isometry inverse() const;
        const double get_x_translation() const;
        const double get_y_translation() const;
        const double get_z_translation() const;

        friend std::ostream& operator << (std::ostream& os, const Isometry& isometry) {
            os << std::string("[");
            os << "T: " << isometry.translation() << ", R:" << isometry.rotation();
            return os;
        }

    private:
        static Matrix4 ToMatrix4(const Matrix3&);
        static Vector4 ToVector4(const Vector3&);
        static Matrix3 ToMatrix3(const Matrix4&);
        static Vector3 ToVector3(const Vector4&);
        // Attributes
        Matrix4 translation_matrix_;
        Matrix4 rotation_matrix_;
        Vector4 translation_vector_;

};  // Isometry

}  // namespace math
}  // namespace ekumen

#endif  // ISOMETRY_H
