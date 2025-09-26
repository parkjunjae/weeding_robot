#include <cmath>   // std::isfinite, std::atan2
#include <tuple>   // std::tuple 리턴

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>

namespace py = pybind11;
using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

struct Result {
  bool ok = false;
  double dx = 0, dy = 0, dyaw = 0;
  double fitness = 1e9;
};

class LidarOdomNDT {
public:
  LidarOdomNDT() : voxel_(0.3), ndt_res_(1.5), max_iter_(30), step_size_(0.1),
                   use_icp_fallback_(true), prev_set_(false) {
    T_odom_.setIdentity();
  }
    // ★ 반드시 정의가 이 파일 안에 존재해야 함
  void set_params(double voxel, double ndt_res, int max_iter, double step_size,
                  bool use_icp_fallback = true) {
    voxel_ = voxel;
    ndt_res_ = ndt_res;
    max_iter_ = max_iter;
    step_size_ = step_size;
    use_icp_fallback_ = use_icp_fallback;
  }

  void reset() {
    prev_.reset(new CloudT);
    prev_set_ = false;
    T_odom_.setIdentity();
  }

  Result push_frame(py::array_t<float, py::array::c_style | py::array::forcecast> xyz,
                    double /*ts*/) {
    auto buf = xyz.request();
    if (buf.ndim != 2 || buf.shape[1] != 3) {
      throw std::runtime_error("points must be (N,3) float32");
    }

    CloudT::Ptr curr(new CloudT);
    curr->resize(buf.shape[0]);
    const float* data = static_cast<float*>(buf.ptr);
    for (ssize_t i = 0; i < buf.shape[0]; ++i) {
      (*curr)[i].x = data[3*i+0];
      (*curr)[i].y = data[3*i+1];
      (*curr)[i].z = data[3*i+2];
    }
    if (voxel_ > 1e-6) {
      pcl::VoxelGrid<PointT> vg;
      vg.setLeafSize(voxel_, voxel_, voxel_);
      vg.setInputCloud(curr);
      CloudT::Ptr ds(new CloudT);
      vg.filter(*ds);
      curr = ds;
    }

    Result out;
    if (!prev_set_) {
        prev_ = curr;
        prev_set_ = true;
        return out; // ok=false (기준 저장)
    }

// 초기 추정(필요 시 IMU 기반으로 개선 가능)
Eigen::Matrix4f T_guess = Eigen::Matrix4f::Identity();

// --- NDT: source=prev_, target=curr  → 바로 prev->curr 산출
pcl::NormalDistributionsTransform<PointT, PointT> ndt;
ndt.setTransformationEpsilon(1e-3);
ndt.setStepSize(step_size_);
ndt.setResolution(ndt_res_);
ndt.setMaximumIterations(max_iter_);
ndt.setInputSource(prev_);
ndt.setInputTarget(curr);

CloudT::Ptr aligned(new CloudT);           // 필요하면 사용
ndt.align(*aligned, T_guess);

bool   converged = ndt.hasConverged();
double score     = ndt.getFitnessScore();
Eigen::Matrix4f T_motion = ndt.getFinalTransformation(); // ★ prev->curr (그대로 사용)

// --- ICP fallback도 동일하게 prev_->curr
if ((!converged || !std::isfinite(score) || score > 2.0) && use_icp_fallback_) {
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaximumIterations(50);
  icp.setMaxCorrespondenceDistance(2.0);
  icp.setInputSource(prev_);
  icp.setInputTarget(curr);
  CloudT::Ptr aligned2(new CloudT);
  icp.align(*aligned2, T_guess);
  if (icp.hasConverged()) {
    T_motion = icp.getFinalTransformation(); // ★ prev->curr
    score    = icp.getFitnessScore();
    converged = true;
  } else {
    converged = false;
  }
}

// --- 최종 적용(한 번만)
if (converged && std::isfinite(score)) {
  out.ok      = true;
  out.dx      = T_motion(0,3);
  out.dy      = T_motion(1,3);
  out.dyaw    = std::atan2(T_motion(1,0), T_motion(0,0));
  out.fitness = score;

  T_odom_ = T_odom_ * T_motion;
  prev_   = curr;
}

    return out;
  }

  std::tuple<double,double,double> odom_pose() const {
    double x = T_odom_(0,3);
    double y = T_odom_(1,3);
    double yaw = std::atan2(T_odom_(1,0), T_odom_(0,0));
    return {x,y,yaw};
  }

private:
  double voxel_;
  double ndt_res_;
  int    max_iter_;
  double step_size_;
  bool   use_icp_fallback_;

  CloudT::Ptr prev_;
  bool prev_set_;
  Eigen::Matrix4f T_odom_;
};

PYBIND11_MODULE(lidar_odom_ndt, m) {
  py::class_<LidarOdomNDT>(m, "LidarOdomNDT")
    .def(py::init<>())
    .def("set_params", &LidarOdomNDT::set_params,
         py::arg("voxel")=0.3, py::arg("ndt_res")=1.5,
         py::arg("max_iter")=30, py::arg("step_size")=0.1,
         py::arg("use_icp_fallback")=true)
    .def("reset", &LidarOdomNDT::reset)
    .def("push_frame", &LidarOdomNDT::push_frame,
         py::arg("points_xyz"), py::arg("ts"))
    .def("odom_pose", &LidarOdomNDT::odom_pose);

  py::class_<Result>(m, "Result")
    .def_readonly("ok", &Result::ok)
    .def_readonly("dx", &Result::dx)
    .def_readonly("dy", &Result::dy)
    .def_readonly("dyaw", &Result::dyaw)
    .def_readonly("fitness", &Result::fitness);
}
