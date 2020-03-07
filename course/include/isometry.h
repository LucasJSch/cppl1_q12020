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
        Vector3 operator * (const Vector3&);
        Matrix3& operator = (Matrix3&&);

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

class Isometry : Matrix4 {

    public:
        // Constructors
        Isometry(const Vector3& translation_vector = Vector3(), 
                 const Matrix3& rotation_matrix = Matrix3()) :
                _translation_matrix{{1, 0, 0, translation_vector.x()},
                                    {0, 1, 0, translation_vector.y()},
                                    {0, 0, 1, translation_vector.z()},
                                    Vector4::kUnitW},
                _rotation_matrix{rotation_matrix} {} //scheinkerman Implement Matrix4 from Matrix3

        Isometry(const Vector4& translation_vector = Vector4(),
                 const Matrix4& rotation_matrix = Matrix4()) :
                _translation_matrix{translation_vector},
                _rotation_matrix{rotation_matrix} {}
                 

        // Static operations
        static Isometry FromTranslation(const Vector3&) const;
        static Vector3 RotateAround(const Vector3&, const double) const;
        //static const Isometry FromEulerAngles(const Vector3&); ?
        static Isometry FromEulerAngles(const double, const double, const double) const ;

        // Operators
        Matrix3& operator = (const Matrix3&) const;
        Vector3 operator * (const Vector3&) const;

        // Misc
        Vector3 transform(const Vector3&) const;
        Isometry compose(const Isometry&) const;

        // Getters
        Vector3 traslation() const;
        Matrix3 rotation() const;
        Matrix3 inverse() const;
        const double get_x_translation() const;
        const double get_y_translation() const;
        const double get_z_translation() const;

        friend std::ostream& operator << (std::ostream& os, const Isometry& isometry) {
            os << std::string("[");
            os << "T: " << traslation() << ", R:" << rotation();
            return os;
        }

    private:
        // Attributes
        Matrix4 _translation_matrix;
        Matrix4 _rotation_matrix;

};  // Isometry

}  // namespace math
}  // namespace ekumen

#endif  // ISOMETRY_H
