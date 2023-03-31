#ifndef COMMON_LIB_H
#define COMMON_LIB_H
#define PCL_NO_PRECOMPILE // !! BEFORE ANY PCL INCLUDE!!

#include <so3_math.h>
#include <Eigen/Eigen>
#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <fast_lio/Pose6D.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <string.h>
#include "../src/custom_point.h"

using namespace std;
using namespace Eigen;

#define USE_IKFOM

#define PI_M (3.14159265358)
#define G_m_s2 (9.81)         // Gravaty const in GuangDong/China
#define DIM_STATE (18)        // Dimension of states (Let Dim(SO(3)) = 3)
#define DIM_PROC_N (12)       // Dimension of process noise (Let Dim(SO(3)) = 3)
#define CUBE_LEN  (6.0)
#define LIDAR_SP_LEN    (2)
#define INIT_COV   (1)
#define NUM_MATCH_POINTS    (5)
#define MAX_MEAS_DIM        (10000)

#define VEC_FROM_ARRAY(v)        v[0],v[1],v[2]
#define MAT_FROM_ARRAY(v)        v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8]
#define CONSTRAIN(v,min,max)     ((v>min)?((v<max)?v:max):min)
#define ARRAY_FROM_EIGEN(mat)    mat.data(), mat.data() + mat.rows() * mat.cols()
#define STD_VEC_FROM_EIGEN(mat)  vector<decltype(mat)::Scalar> (mat.data(), mat.data() + mat.rows() * mat.cols())
#define DEBUG_FILE_DIR(name)     (string(string(ROOT_DIR) + "Log/"+ name))

typedef fast_lio::Pose6D Pose6D;
typedef pcl::PointXYZRGBINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZRGBI;
typedef vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector;
typedef Vector3d V3D;
typedef Matrix3d M3D;
typedef Vector3f V3F;
typedef Matrix3f M3F;

#define MD(a,b)  Matrix<double, (a), (b)>
#define VD(a)    Matrix<double, (a), 1>
#define MF(a,b)  Matrix<float, (a), (b)>
#define VF(a)    Matrix<float, (a), 1>

M3D Eye3d(M3D::Identity());
M3F Eye3f(M3F::Identity());
V3D Zero3d(0, 0, 0);
V3F Zero3f(0, 0, 0);

struct MeasureGroup     // Lidar data and imu dates for the curent process
{
    MeasureGroup()
    {
        lidar_beg_time = 0.0;
        this->lidar.reset(new PointCloudXYZRGBI());
    };
    double lidar_beg_time;
    double lidar_end_time;
    PointCloudXYZRGBI::Ptr lidar;
    deque<sensor_msgs::ImageConstPtr> rgb_cam_1;
    deque<sensor_msgs::ImageConstPtr> rgb_cam_2;

    deque<sensor_msgs::Imu::ConstPtr> imu;
};

struct StatesGroup
{
    StatesGroup() {
		this->rot_end = M3D::Identity();
		this->pos_end = Zero3d;
        this->vel_end = Zero3d;
        this->bias_g  = Zero3d;
        this->bias_a  = Zero3d;
        this->gravity = Zero3d;
        this->cov     = MD(DIM_STATE,DIM_STATE)::Identity() * INIT_COV;
        this->cov.block<9,9>(9,9) = MD(9,9)::Identity() * 0.00001;
	};

    StatesGroup(const StatesGroup& b) {
		this->rot_end = b.rot_end;
		this->pos_end = b.pos_end;
        this->vel_end = b.vel_end;
        this->bias_g  = b.bias_g;
        this->bias_a  = b.bias_a;
        this->gravity = b.gravity;
        this->cov     = b.cov;
	};

    StatesGroup& operator=(const StatesGroup& b)
	{
        this->rot_end = b.rot_end;
		this->pos_end = b.pos_end;
        this->vel_end = b.vel_end;
        this->bias_g  = b.bias_g;
        this->bias_a  = b.bias_a;
        this->gravity = b.gravity;
        this->cov     = b.cov;
        return *this;
	};

    StatesGroup operator+(const Matrix<double, DIM_STATE, 1> &state_add)
	{
        StatesGroup a;
		a.rot_end = this->rot_end * Exp(state_add(0,0), state_add(1,0), state_add(2,0));
		a.pos_end = this->pos_end + state_add.block<3,1>(3,0);
        a.vel_end = this->vel_end + state_add.block<3,1>(6,0);
        a.bias_g  = this->bias_g  + state_add.block<3,1>(9,0);
        a.bias_a  = this->bias_a  + state_add.block<3,1>(12,0);
        a.gravity = this->gravity + state_add.block<3,1>(15,0);
        a.cov     = this->cov;
		return a;
	};

    StatesGroup& operator+=(const Matrix<double, DIM_STATE, 1> &state_add)
	{
        this->rot_end = this->rot_end * Exp(state_add(0,0), state_add(1,0), state_add(2,0));
		this->pos_end += state_add.block<3,1>(3,0);
        this->vel_end += state_add.block<3,1>(6,0);
        this->bias_g  += state_add.block<3,1>(9,0);
        this->bias_a  += state_add.block<3,1>(12,0);
        this->gravity += state_add.block<3,1>(15,0);
		return *this;
	};

    Matrix<double, DIM_STATE, 1> operator-(const StatesGroup& b)
	{
        Matrix<double, DIM_STATE, 1> a;
        M3D rotd(b.rot_end.transpose() * this->rot_end);
        a.block<3,1>(0,0)  = Log(rotd);
        a.block<3,1>(3,0)  = this->pos_end - b.pos_end;
        a.block<3,1>(6,0)  = this->vel_end - b.vel_end;
        a.block<3,1>(9,0)  = this->bias_g  - b.bias_g;
        a.block<3,1>(12,0) = this->bias_a  - b.bias_a;
        a.block<3,1>(15,0) = this->gravity - b.gravity;
		return a;
	};

    void resetpose()
    {
        this->rot_end = M3D::Identity();
		this->pos_end = Zero3d;
        this->vel_end = Zero3d;
    }

	M3D rot_end;      // the estimated attitude (rotation matrix) at the end lidar point
    V3D pos_end;      // the estimated position at the end lidar point (world frame)
    V3D vel_end;      // the estimated velocity at the end lidar point (world frame)
    V3D bias_g;       // gyroscope bias
    V3D bias_a;       // accelerator bias
    V3D gravity;      // the estimated gravity acceleration
    Matrix<double, DIM_STATE, DIM_STATE>  cov;     // states covariance
};

template<typename T>
T rad2deg(T radians)
{
  return radians * 180.0 / PI_M;
}

template<typename T>
T deg2rad(T degrees)
{
  return degrees * PI_M / 180.0;
}

template<typename T>
auto set_pose6d(const double t, const Matrix<T, 3, 1> &a, const Matrix<T, 3, 1> &g, \
                const Matrix<T, 3, 1> &v, const Matrix<T, 3, 1> &p, const Matrix<T, 3, 3> &R)
{
    Pose6D rot_kp;
    rot_kp.offset_time = t;
    for (int i = 0; i < 3; i++)
    {
        rot_kp.acc[i] = a(i);
        rot_kp.gyr[i] = g(i);
        rot_kp.vel[i] = v(i);
        rot_kp.pos[i] = p(i);
        for (int j = 0; j < 3; j++)  rot_kp.rot[i*3+j] = R(i,j);
    }
    return move(rot_kp);
}

/* comment
plane equation: Ax + By + Cz + D = 0
convert to: A/D*x + B/D*y + C/D*z = -1
solve: A0*x0 = b0
where A0_i = [x_i, y_i, z_i], x0 = [A/D, B/D, C/D]^T, b0 = [-1, ..., -1]^T
normvec:  normalized x0
*/
template<typename T>
bool esti_normvector(Matrix<T, 3, 1> &normvec, const PointVector &point, const T &threshold, const int &point_num)
{
    MatrixXf A(point_num, 3);
    MatrixXf b(point_num, 1);
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < point_num; j++)
    {
        A(j,0) = point[j].x;
        A(j,1) = point[j].y;
        A(j,2) = point[j].z;
    }
    normvec = A.colPivHouseholderQr().solve(b);
    
    for (int j = 0; j < point_num; j++)
    {
        if (fabs(normvec(0) * point[j].x + normvec(1) * point[j].y + normvec(2) * point[j].z + 1.0f) > threshold)
        {
            return false;
        }
    }

    normvec.normalize();
    return true;
}

float calc_dist(PointType p1, PointType p2){
    float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    return d;
}

template<typename T>
bool esti_plane(Matrix<T, 4, 1> &pca_result, const PointVector &point, const T &threshold)
{
    Matrix<T, NUM_MATCH_POINTS, 3> A;
    Matrix<T, NUM_MATCH_POINTS, 1> b;
    A.setZero();
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        A(j,0) = point[j].x;
        A(j,1) = point[j].y;
        A(j,2) = point[j].z;
    }

    Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
        {
            return false;
        }
    }
    return true;
}


//  COPY PRINTF FILE FROM R3live
const std::string _tools_color_printf_version = "V1.2";
const std::string _tools_color_printf_info = "[Enh]: Add delete lines, ANSI_SCREEN_FLUSH";
using std::cout;
using std::endl;
// clang-format off
#ifdef EMPTY_ANSI_COLORS
    #define ANSI_COLOR_RED ""
    #define ANSI_COLOR_RED_BOLD ""
    #define ANSI_COLOR_GREEN ""
    #define ANSI_COLOR_GREEN_BOLD ""
    #define ANSI_COLOR_YELLOW ""
    #define ANSI_COLOR_YELLOW_BOLD ""
    #define ANSI_COLOR_BLUE ""
    #define ANSI_COLOR_BLUE_BOLD ""
    #define ANSI_COLOR_MAGENTA ""
    #define ANSI_COLOR_MAGENTA_BOLD ""
#else
    #define ANSI_COLOR_RED "\x1b[0;31m"
    #define ANSI_COLOR_RED_BOLD "\x1b[1;31m"
    #define ANSI_COLOR_RED_BG "\x1b[0;41m"

    #define ANSI_COLOR_GREEN "\x1b[0;32m"
    #define ANSI_COLOR_GREEN_BOLD "\x1b[1;32m"
    #define ANSI_COLOR_GREEN_BG "\x1b[0;42m"

    #define ANSI_COLOR_YELLOW "\x1b[0;33m"
    #define ANSI_COLOR_YELLOW_BOLD "\x1b[1;33m"
    #define ANSI_COLOR_YELLOW_BG "\x1b[0;43m"

    #define ANSI_COLOR_BLUE "\x1b[0;34m"
    #define ANSI_COLOR_BLUE_BOLD "\x1b[1;34m"
    #define ANSI_COLOR_BLUE_BG "\x1b[0;44m"

    #define ANSI_COLOR_MAGENTA "\x1b[0;35m"
    #define ANSI_COLOR_MAGENTA_BOLD "\x1b[1;35m"
    #define ANSI_COLOR_MAGENTA_BG "\x1b[0;45m"

    #define ANSI_COLOR_CYAN "\x1b[0;36m"
    #define ANSI_COLOR_CYAN_BOLD "\x1b[1;36m"
    #define ANSI_COLOR_CYAN_BG "\x1b[0;46m"

    #define ANSI_COLOR_WHITE "\x1b[0;37m"
    #define ANSI_COLOR_WHITE_BOLD "\x1b[1;37m"
    #define ANSI_COLOR_WHITE_BG "\x1b[0;47m"

    #define ANSI_COLOR_BLACK "\x1b[0;30m"
    #define ANSI_COLOR_BLACK_BOLD "\x1b[1;30m"
    #define ANSI_COLOR_BLACK_BG "\x1b[0;40m"

    #define ANSI_COLOR_RESET "\x1b[0m"

    #define ANSI_DELETE_LAST_LINE "\033[A\33[2K\r"
    #define ANSI_DELETE_CURRENT_LINE "\33[2K\r"
    #define ANSI_SCREEN_FLUSH std::fflush(stdout);

    #define SET_PRINT_COLOR( a ) cout << a ;

#endif
// clang-format on

struct _Scope_color
{
    _Scope_color( const char * color )
    {
        cout << color;
    }

    ~_Scope_color()
    {
        cout << ANSI_COLOR_RESET;
    }
};

#define scope_color(a) _Scope_color _scope(a);



// COPY END
// For the consideration of avoiding alignment error while running, we prefer using datatype of Eigen with no alignment.
// In addition, for higher realtime performance (don't expect too much), you can modify the option with alignment, but you might carefully be aware of the crash of the program.
#define EIGEN_DATA_TYPE_DEFAULT_OPTION Eigen::DontAlign
// #define EIGEN_DATA_TYPE_DEFAULT_OPTION Eigen::AutoAlign

template < int M, int N, int option = (EIGEN_DATA_TYPE_DEFAULT_OPTION|Eigen::RowMajor) >
using eigen_mat_d = Eigen::Matrix< double, M, N, option >;

template < int M, int N, int option = (EIGEN_DATA_TYPE_DEFAULT_OPTION|Eigen::RowMajor) >
using eigen_mat_d = Eigen::Matrix< double, M, N, option >;

template < int M, int N, int option = (EIGEN_DATA_TYPE_DEFAULT_OPTION|Eigen::RowMajor) >
using eigen_mat_f = Eigen::Matrix< float, M, N, option >;

template < typename T, int M, int N, int option = (EIGEN_DATA_TYPE_DEFAULT_OPTION|Eigen::RowMajor) >
using eigen_mat_t = Eigen::Matrix< T, M, N, option >;

template < int M, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
using eigen_vec_d = Eigen::Matrix< double, M, 1, option >;

template < int M, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
using eigen_vec_f = Eigen::Matrix< float, M, 1, option >;

template < typename T, int M, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
using eigen_vec_t = Eigen::Matrix< T, M, 1, option >;

template < typename T, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
using eigen_q_t = Eigen::Quaternion< T, option >;

template < typename T >
using eigen_angleaxis_t = Eigen::AngleAxis< T >;

template < typename T, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
using eigen_pose_t = Eigen::Transform< T, 3, Eigen::Isometry, option >;

template < int M, int N >
using eigen_mat = eigen_mat_d< M, N >;

template < int M >
using eigen_vec = eigen_vec_d< M >;

typedef eigen_vec< 2 >                                                                 vec_2;
typedef eigen_vec< 3 >                                                                 vec_3;
typedef eigen_vec< 4 >                                                                 vec_4;
typedef eigen_vec< 6 >                                                                 vec_6;
typedef eigen_vec< 7 >                                                                 vec_7;
typedef eigen_vec< 12 >                                                                vec_12;
typedef eigen_mat< 3, 3 >                                                              mat_3_3;
typedef eigen_mat< 4, 4 >                                                              mat_4_4;
typedef eigen_mat< 6, 6 >                                                              mat_6_6;
typedef eigen_mat< 12, 12 >                                                            mat_12;
typedef eigen_mat< 6, 12 >                                                             mat_6_12;
typedef eigen_mat< 12, 6 >                                                             mat_12_6;
typedef eigen_angleaxis_t< double >                                                    eigen_angleaxis;
typedef Eigen::Quaternion< double, EIGEN_DATA_TYPE_DEFAULT_OPTION >                    eigen_q;
typedef Eigen::Transform< double, 3, Eigen::Isometry, EIGEN_DATA_TYPE_DEFAULT_OPTION > eigen_pose;
typedef std::vector< eigen_q >                                                         eigen_q_vec;

// namespace Common_tools
// {

template < typename T >
inline T angle_refine( const T &rad )
{
    // Refine angle to [-pi, pi]
    T rad_afr_refined = ( rad - ( floor( rad / T( 2 * M_PI ) ) * T( 2 * M_PI ) ) );
    if ( rad_afr_refined > T( M_PI ) )
    {
        rad_afr_refined -= T( 2 * M_PI );
    }
    return rad_afr_refined;
}

/*****
    Some operator based tools for Eigen::Vector<T>
    Example:
        a. Eigen::Vector<T> from array:                Eigen::Vector<T> << data_rhs
        b. Eigen::Vector<T> to array:                  Eigen::Vector<T> >> data_rhs
*****/
template < typename T, int M, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
inline const void operator<<( eigen_vec_t< T, M, option > &eigen_vec_lhs, const T *data_rhs )
{
    for ( size_t i = 0; i < M; i++ )
    {
        eigen_vec_lhs( i ) = data_rhs[ i ];
    }
}

template < typename T, int M, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
inline const void operator>>( const eigen_vec_t< T, M, option > &eigen_vec_lhs, T *data_rhs )
{
    for ( size_t i = 0; i < M; i++ )
    {
        data_rhs[ i ] = eigen_vec_lhs( i );
    }
}

template < typename T, int M, typename TT = T, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
inline const void operator<<( eigen_vec_t< T, M, option > &eigen_vec_lhs, const std::pair< std::vector< TT > *, int > &std_vector_start )
{
    // Loading data from a std::vector, from the starting point
    // Example: eigen_vec_lhs << std::make_pair(&std::vector, starting_point)
    for ( size_t i = 0; i < M; i++ )
    {
        eigen_vec_lhs( i ) = T( ( *std_vector_start.first )[ std_vector_start.second + i ] );
    }
}

template < typename T, int M, typename TT = T, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
inline const void operator<<( const eigen_vec_t< T, M, option > &eigen_vec_lhs, std::pair< std::vector< TT > *, int > &std_vector_start )
{
    for ( size_t i = 0; i < M; i++ )
    {
        ( *std_vector_start.first )[ std_vector_start.second + i ] = TT( eigen_vec_lhs( i ) );
    }
}

/*****
    Some operator based tools for Eigen::Quaternion, before using these tools, make sure you are using the uniform quaternion,
    otherwise some of the unwanted results will be happend.
    Example:
        a. Quaternion from array:                   Eigen::Quaternion << data_rhs
        b. Quaternion to array:                     Eigen::Quaternion >> data_rhs
        c. Rotation angle multiply(*=) a scalar:    Eigen::Quaternion *= scalar
        d. Rotation angle multiply(*=) a scalar:    Eigen::Quaternion * scalar
*****/
template < typename T, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
inline const void operator<<( eigen_q_t< T, option > &eigen_q_lhs, const T *data_rhs )
{
    eigen_q_lhs.w() = data_rhs[ 0 ];
    eigen_q_lhs.x() = data_rhs[ 1 ];
    eigen_q_lhs.y() = data_rhs[ 2 ];
    eigen_q_lhs.z() = data_rhs[ 3 ];
}

template < typename T, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
inline const void operator>>( const eigen_q_t< T, option > &eigen_q_lhs, T *data_rhs )
{
    data_rhs[ 0 ] = eigen_q_lhs.w();
    data_rhs[ 1 ] = eigen_q_lhs.x();
    data_rhs[ 2 ] = eigen_q_lhs.y();
    data_rhs[ 3 ] = eigen_q_lhs.z();
}

template < typename T, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
inline const eigen_q_t< T, option > &operator*=( eigen_q_t< T, option > &eigen_q_lhs, const T &s )
{
    Eigen::AngleAxis< T > angle_axis( eigen_q_lhs );
    angle_axis *= s;
    eigen_q_lhs = eigen_q_t< T, option >( angle_axis );
    return eigen_q_lhs;
}

template < typename T, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
inline const eigen_q_t< T, option > &operator*( const eigen_q_t< T, option > &eigen_q_lhs, const T &s )
{
    Eigen::AngleAxis< T > angle_axis( eigen_q_lhs );
    angle_axis *= s;
    return eigen_q_t< T, option >( angle_axis );
}

/*****
    Conversion between eigen angle_axis and data array
    Example:  
        a. AngleAxis from array: Eigen::AngleAxis << data_rhs
        b. AngleAxis to array:   Eigen::AngleAxis >> data_rhs
        c. Rotation angle multiply(*=) a scalar:    Eigen::AngleAxis *= scalar
        d. Rotation angle multiply(*=) a scalar:    Eigen::AngleAxis * scalar
*****/
template < typename T >
inline const void operator<<( Eigen::AngleAxis< T > &eigen_axisangle_lhs, const T *data_rhs )
{
    T vec_norm = sqrt( data_rhs[ 0 ] * data_rhs[ 0 ] + data_rhs[ 1 ] * data_rhs[ 1 ] + data_rhs[ 2 ] * data_rhs[ 2 ] );
    if ( vec_norm != T( 0.0 ) )
    {
        eigen_axisangle_lhs.angle() = vec_norm;
        eigen_axisangle_lhs.axis() << data_rhs[ 0 ] / vec_norm, data_rhs[ 1 ] / vec_norm, data_rhs[ 2 ] / vec_norm;
    }
    else
    {
        eigen_axisangle_lhs.angle() = vec_norm;
        eigen_axisangle_lhs.axis() << vec_norm * data_rhs[ 0 ], vec_norm * data_rhs[ 1 ], vec_norm * data_rhs[ 2 ]; // For the consideration of derivation
    }
}

template < typename T >
inline const void operator>>( const Eigen::AngleAxis< T > &eigen_axisangle_lhs, T *data_rhs )
{
    T vec_norm = eigen_axisangle_lhs.angle();
    data_rhs[ 0 ] = eigen_axisangle_lhs.axis()( 0 ) * vec_norm;
    data_rhs[ 1 ] = eigen_axisangle_lhs.axis()( 1 ) * vec_norm;
    data_rhs[ 2 ] = eigen_axisangle_lhs.axis()( 2 ) * vec_norm;
}

template < typename T >
inline const Eigen::AngleAxis< T > operator*=( Eigen::AngleAxis< T > &eigen_axisangle_lhs, const T &s )
{
    eigen_axisangle_lhs.angle() *= s;
    return eigen_axisangle_lhs;
}

template < typename T >
inline const Eigen::AngleAxis< T > operator*( const Eigen::AngleAxis< T > &eigen_axisangle_lhs, const T &s )
{
    Eigen::AngleAxis< T > angle_axis( eigen_axisangle_lhs );
    angle_axis.angle() *= s;
    return angle_axis;
}


#endif