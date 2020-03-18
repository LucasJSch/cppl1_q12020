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
        Vector3& operator [] (const uint32_t);
        const Vector3& operator [] (const uint32_t) const;
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
        bool operator == (const Matrix3&) const;
        bool operator != (const Matrix3&) const;
        Vector3 operator * (const Vector3&) const;
        Matrix3& operator = (Matrix3&&);
        // Matrix product
        Matrix3 operator ^ (const Matrix3&) const;

        friend Matrix3 operator * (const double d, const Matrix3& m) {
                return Matrix3(m.r1_ * d, m.r2_ * d, m.r3_ * d);
        }

        friend std::ostream& operator << (std::ostream& os, const Matrix3& m) {
            auto print_vector3 = [&](const Vector3& v) -> std::ostream& {
                    os << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
                    return os;
            };

            os << std::string("[");
            print_vector3(m.row(0)) << ", ";
            print_vector3(m.row(1)) << ", ";
            print_vector3(m.row(2)) << "]";
            return os;
        }

        // Getters
        Vector3 row(uint32_t index) const;
        Vector3 col(uint32_t index) const;

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

class Isometry {

    public:
        // Constructors
        Isometry(const Vector3& translation_vector = Vector3(), 
                 const Matrix3& rotation_matrix = Matrix3::kIdentity) :
                translation_vector_{translation_vector},
                rotation_matrix_{rotation_matrix} {}
        Isometry(const Isometry& t) : 
            translation_vector_{t.translation_vector_},
            rotation_matrix_{t.rotation_matrix_} {}

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
            os << "]";
            return os;
        }

    private:
        // Attributes
        Vector3 translation_vector_;
        Matrix3 rotation_matrix_;

};  // Isometry

}  // namespace math
}  // namespace ekumen

#endif  // ISOMETRY_H
