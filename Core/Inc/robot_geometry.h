#ifndef ROBOT_GEOMETRY_H
#define ROBOT_GEOMETRY_H

#include <math.h>

// Cac hang so toan hoc
#define PI 3.14159265359f
#define DEG_TO_RAD(x) ((x) * PI / 180.0f)   // Doi do sang radian
#define RAD_TO_DEG(x) ((x) * 180.0f / PI)   // Doi radian sang do
#define SQUARE(x) ((x) * (x))               // Binh phuong
#define MIN(a, b) ((a) < (b) ? (a) : (b))   // Gia tri nho nhat
#define MAX(a, b) ((a) > (b) ? (a) : (b))   // Gia tri lon nhat
#define SQRT3 1.73205080757f               // Can bac hai cua 3

// Cau truc hinh hoc cua robot (don vi cm)
typedef struct {
    float lp;       // Ban kinh mat phang tren
    float l1;       // Do dai canh tren
    float l2;       // Do dai canh duoi
    float lb;       // Ban kinh mat phang duoi

    float maxh;     // Chieu cao toi da
    float minh;     // Chieu cao toi thieu
    float h;        // Chieu cao hien tai
    float maxtheta; // Goc nghieng toi da (rad)
} RobotGeometry;

// Cau truc vector 3D
typedef struct {
    float x;
    float y;
    float z;
} Vector3D;

// Trang thai cua robot
typedef struct {
    RobotGeometry geom;   // Hinh hoc cua robot
    Vector3D E, D, F;     // Vi tri diem tren mat phang tren (previously A[0], A[1], A[2])
    Vector3D B_base, A_base, C_base; // Vi tri diem co dinh tren de (previously B[0], B[1], B[2])
    Vector3D Q, P, R;     // Vi tri diem trung gian (previously C[0], C[1], C[2])
    float theta[3];        // Goc cua servo (radian)
} RobotState;

// Nguyen mau ham tinh toan
float ComputeMaxH(const RobotGeometry *geom);
float ComputeMinH(const RobotGeometry *geom);

#endif /* ROBOT_GEOMETRY_H */
