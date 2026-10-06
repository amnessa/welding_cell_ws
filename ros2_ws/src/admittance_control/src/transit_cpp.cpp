/**
 * @file transit_cpp.cpp
 * @brief The marking cell's collision model in C++ and OMPL's official
 *        AnytimePathShortening (ROS's libompl 1.7) for the arm's free moves.
 *
 * notes/aps_transit_plan.md. Python module `admittance_control._transit_cpp`:
 *
 *   m = _transit_cpp.Model(collision_model.export_spec())
 *   m.frames(q)        -> 7 4x4 frames: shoulder, lift, elbow, wrist_1, wrist_2, wrist_3, tool0
 *   m.min_distance(q)  -> (distance, name_a, name_b)
 *   m.is_valid(q)      -> joint limits AND min distance >= clearance
 *   m.is_valid_many(Q) -> bool array (n,)
 *   _transit_cpp.plan(m, q_from, q_to, planner="aps", budget_s=1.0, ...) -> dict
 *
 * The collision model is a FAITHFUL PORT of admittance_control/collision.py: the same
 * primitives, the same distance algorithms (closed-form segment-segment, golden-section
 * segment-box to tol 1e-4, separating-axis box-box as 0 / +inf), the same pairs in the
 * same order, so `is_valid` agrees with the Python model (test/test_transit_cpp.py proves
 * it on random and near-contact configurations). The Python model stays the single
 * source of truth: CollisionModel.export_spec() hands over everything used here.
 *
 * Planning happens in SCALED joint coordinates y_j = w_j * q_j, so OMPL's own Euclidean
 * path length is the weighted joint travel (no cost callback). The validity checker is
 * const and pure, so APS's planner threads call it concurrently; the GIL is released
 * while OMPL plans.
 */
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/AnytimePathShortening.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/Console.h>

namespace py = pybind11;
namespace ob = ompl::base;
namespace og = ompl::geometric;

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Mat4 = Eigen::Matrix4d;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kInf = std::numeric_limits<double>::infinity();

// ---------------------------------------------------------------- transforms ----------
Mat4 rotx(double a)
{
  Mat4 T = Mat4::Identity();
  const double c = std::cos(a), s = std::sin(a);
  T(1, 1) = c; T(1, 2) = -s; T(2, 1) = s; T(2, 2) = c;
  return T;
}
Mat4 roty(double a)
{
  Mat4 T = Mat4::Identity();
  const double c = std::cos(a), s = std::sin(a);
  T(0, 0) = c; T(0, 2) = s; T(2, 0) = -s; T(2, 2) = c;
  return T;
}
Mat4 rotz(double a)
{
  Mat4 T = Mat4::Identity();
  const double c = std::cos(a), s = std::sin(a);
  T(0, 0) = c; T(0, 1) = -s; T(1, 0) = s; T(1, 1) = c;
  return T;
}
Mat4 trans(double x, double y, double z)
{
  Mat4 T = Mat4::Identity();
  T(0, 3) = x; T(1, 3) = y; T(2, 3) = z;
  return T;
}
// URDF rpy: extrinsic X-Y-Z = Rz(y) Ry(p) Rx(r)  (kinematics._rpy)
Mat4 rpy(double r, double p, double y) { return rotz(y) * roty(p) * rotx(r); }

// ---------------------------------------------------------------- primitives ----------
struct Prim
{
  std::string name;
  bool is_box = false;
  Vec3 p0 = Vec3::Zero(), p1 = Vec3::Zero();   // capsule
  double radius = 0.0;
  Vec3 centre = Vec3::Zero(), half = Vec3::Zero();  // box
  Mat3 R = Mat3::Identity();
};

Prim capsule(const std::string &name, const Vec3 &p0, const Vec3 &p1, double r)
{
  Prim c;
  c.name = name; c.is_box = false; c.p0 = p0; c.p1 = p1; c.radius = r;
  return c;
}

// collision._seg_seg_distance (Ericson 5.1.9), line by line
double seg_seg_distance(const Vec3 &p0, const Vec3 &p1, const Vec3 &q0, const Vec3 &q1)
{
  const Vec3 d1 = p1 - p0, d2 = q1 - q0, r = p0 - q0;
  const double a = d1.dot(d1), e = d2.dot(d2), f = d2.dot(r);
  const double eps = 1e-12;
  double s, t;
  if (a <= eps && e <= eps) return (p0 - q0).norm();
  if (a <= eps) {
    s = 0.0; t = std::clamp(f / e, 0.0, 1.0);
  } else {
    const double c = d1.dot(r);
    if (e <= eps) {
      t = 0.0; s = std::clamp(-c / a, 0.0, 1.0);
    } else {
      const double b = d1.dot(d2);
      const double denom = a * e - b * b;
      s = denom > eps ? std::clamp((b * f - c * e) / denom, 0.0, 1.0) : 0.0;
      t = (b * s + f) / e;
      if (t < 0.0) {
        t = 0.0; s = std::clamp(-c / a, 0.0, 1.0);
      } else if (t > 1.0) {
        t = 1.0; s = std::clamp((b - c) / a, 0.0, 1.0);
      }
    }
  }
  return ((p0 + d1 * s) - (q0 + d2 * t)).norm();
}

// collision._point_box_distance
double point_box_distance(const Vec3 &p, const Prim &bx)
{
  const Vec3 local = bx.R.transpose() * (p - bx.centre);
  const Vec3 d = (local.cwiseAbs() - bx.half).cwiseMax(0.0);
  return d.norm();
}

// collision._seg_box_distance: golden-section search, the same steps
double seg_box_distance(const Vec3 &p0, const Vec3 &p1, const Prim &bx, double tol = 1e-4)
{
  auto f = [&](double t) { return point_box_distance(p0 + (p1 - p0) * t, bx); };
  double lo = 0.0, hi = 1.0;
  const double g = (std::sqrt(5.0) - 1.0) / 2.0;
  double c = hi - g * (hi - lo), d = lo + g * (hi - lo);
  double fc = f(c), fd = f(d);
  while (hi - lo > tol) {
    if (fc < fd) {
      hi = d; d = c; fd = fc;
      c = hi - g * (hi - lo); fc = f(c);
    } else {
      lo = c; c = d; fc = fd;
      d = lo + g * (hi - lo); fd = f(d);
    }
  }
  return std::min({fc, fd, f(0.0), f(1.0)});
}

// collision._boxes_intersect (separating axes), the same tests in the same order
bool boxes_intersect(const Prim &a, const Prim &b, double inflate = 0.0)
{
  const Mat3 &Ra = a.R, &Rb = b.R;
  const Vec3 ha = a.half, hb = b.half.array() + inflate;
  const Vec3 t = Rb.transpose() * (a.centre - b.centre);
  const Mat3 R = Rb.transpose() * Ra;
  const Mat3 absR = R.cwiseAbs().array() + 1e-9;
  for (int i = 0; i < 3; ++i) {
    const double ra = ha.dot(absR.row(i).transpose()), rb = hb(i);
    if (std::abs(t(i)) > ra + rb) return false;
  }
  for (int j = 0; j < 3; ++j) {
    const double ra = ha(j), rb = hb.dot(absR.col(j));
    if (std::abs(t.dot(R.col(j))) > ra + rb) return false;
  }
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      Vec3 axis = Vec3::Unit(i).cross(R.col(j));
      const double n = axis.norm();
      if (n < 1e-9) continue;
      axis /= n;
      const double ra = ha.dot((R.transpose() * axis).cwiseAbs());
      const double rb = hb.dot(axis.cwiseAbs());
      if (std::abs(t.dot(axis)) > ra + rb) return false;
    }
  }
  return true;
}

// collision.primitive_distance
double primitive_distance(const Prim &a, const Prim &b)
{
  if (!a.is_box && !b.is_box)
    return std::max(0.0, seg_seg_distance(a.p0, a.p1, b.p0, b.p1) - a.radius - b.radius);
  if (!a.is_box && b.is_box)
    return std::max(0.0, seg_box_distance(a.p0, a.p1, b) - a.radius);
  if (a.is_box && !b.is_box) return primitive_distance(b, a);
  return boxes_intersect(a, b) ? 0.0 : kInf;
}

Vec3 bx_corner(const Prim &b, int sx, int sy, int sz)
{
  return b.R * Vec3(sx * b.half(0), sy * b.half(1), sz * b.half(2)) + b.centre;
}

// collision.lowest_point_z
double lowest_point_z(const Prim &p)
{
  if (!p.is_box) return std::min(p.p0(2), p.p1(2)) - p.radius;
  double z = kInf;
  for (int sx = -1; sx <= 1; sx += 2)
    for (int sy = -1; sy <= 1; sy += 2)
      for (int sz = -1; sz <= 1; sz += 2) {
        const Vec3 corner = bx_corner(p, sx, sy, sz);
        z = std::min(z, corner(2));
      }
  return z;
}

Prim transform(const Prim &p, const Mat4 &T)          // tool_model.transform_primitive
{
  Prim o = p;
  const Mat3 R = T.block<3, 3>(0, 0);
  const Vec3 t = T.block<3, 1>(0, 3);
  if (!p.is_box) {
    o.p0 = R * p.p0 + t; o.p1 = R * p.p1 + t;
  } else {
    o.centre = R * p.centre + t; o.R = R * p.R;
  }
  return o;
}

// ---------------------------------------------------------------- spec parsing --------
Vec3 vec3(const py::handle &h)
{
  auto v = h.cast<std::vector<double>>();
  if (v.size() != 3) throw std::invalid_argument("expected 3 numbers");
  return Vec3(v[0], v[1], v[2]);
}
Mat3 mat3(const py::handle &h)
{
  auto rows = h.cast<std::vector<std::vector<double>>>();
  if (rows.size() != 3) throw std::invalid_argument("expected a 3x3 matrix");
  Mat3 R;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) R(i, j) = rows.at(i).at(j);
  return R;
}
Prim prim_from(const py::dict &d)
{
  Prim p;
  p.name = d["name"].cast<std::string>();
  const auto type = d["type"].cast<std::string>();
  if (type == "capsule") {
    p.is_box = false;
    p.p0 = vec3(d["p0"]); p.p1 = vec3(d["p1"]); p.radius = d["radius"].cast<double>();
  } else if (type == "box") {
    p.is_box = true;
    p.centre = vec3(d["centre"]); p.half = vec3(d["half"]);
    p.R = d.contains("R") && !d["R"].is_none() ? mat3(d["R"]) : Mat3::Identity();
  } else {
    throw std::invalid_argument("unknown primitive type " + type);
  }
  return p;
}
}  // namespace

// ---------------------------------------------------------------- the model -----------
class Model
{
public:
  explicit Model(const py::dict &spec)
  {
    auto kin = spec["kin"].cast<std::vector<std::vector<double>>>();
    if (kin.size() != 6) throw std::invalid_argument("kin: 6 joints expected");
    for (int i = 0; i < 6; ++i) {
      if (kin[i].size() != 6) throw std::invalid_argument("kin: (x, y, z, roll, pitch, yaw)");
      for (int k = 0; k < 6; ++k) kin_[i][k] = kin[i][k];
    }
    auto radii = spec["radii"].cast<py::dict>();
    r_base_ = radii["base"].cast<double>();
    r_shoulder_ = radii["shoulder"].cast<double>();
    r_upper_ = radii["upper_arm"].cast<double>();
    r_fore_ = radii["forearm"].cast<double>();
    r_wrist_ = radii["wrist"].cast<double>();
    shoulder_offset_ = spec["shoulder_offset"].cast<double>();
    elbow_offset_ = spec["elbow_offset"].cast<double>();
    for (auto h : spec["tool"].cast<py::list>()) tool_.push_back(prim_from(h.cast<py::dict>()));
    for (auto h : spec["scene"].cast<py::list>()) scene_.push_back(prim_from(h.cast<py::dict>()));
    has_table_ = !spec["table_z"].is_none();
    table_z_ = has_table_ ? spec["table_z"].cast<double>() : 0.0;
    for (auto h : spec["table_exempt"].cast<py::list>()) table_exempt_.push_back(h.cast<std::string>());
    for (auto h : spec["self_pairs"].cast<py::list>()) self_pairs_.push_back(h.cast<std::string>());
    clearance_ = spec["clearance"].cast<double>();
    auto lim = spec["limits"].cast<std::vector<std::vector<double>>>();
    if (lim.size() != 6) throw std::invalid_argument("limits: 6 joints expected");
    for (int i = 0; i < 6; ++i) { lo_[i] = lim[i].at(0); hi_[i] = lim[i].at(1); }
  }

  // kinematics.ur5e_link_frames_params
  std::array<Mat4, 7> frames(const double *q) const
  {
    std::array<Mat4, 7> F;
    Mat4 T = rotz(kPi);
    for (int i = 0; i < 6; ++i) {
      const auto &k = kin_[i];
      T = T * trans(k[0], k[1], k[2]) * rpy(k[3], k[4], k[5]) * rotz(q[i]);
      F[i] = T;
    }
    F[6] = T * rpy(0, -kPi / 2, -kPi / 2) * rpy(kPi / 2, 0, kPi / 2);
    return F;
  }

  // collision.ur5e_capsules + the tool at tool0
  void bodies(const double *q, std::vector<Prim> &arm, std::vector<Prim> &tool) const
  {
    const auto F = frames(q);
    auto o = [&](int i) -> Vec3 { return F[i].block<3, 1>(0, 3); };
    const Vec3 z_lift = F[1].block<3, 1>(0, 2), z_elbow = F[2].block<3, 1>(0, 2);
    const Vec3 up0 = o(1) + shoulder_offset_ * z_lift, up1 = o(2) + shoulder_offset_ * z_lift;
    const Vec3 fa0 = o(2) + elbow_offset_ * z_elbow, fa1 = o(3) + elbow_offset_ * z_elbow;
    arm.clear();
    arm.push_back(capsule("base", Vec3(0, 0, 0), Vec3(0, 0, 0.1625), r_base_));
    arm.push_back(capsule("shoulder", o(0), o(1) + shoulder_offset_ * z_lift, r_shoulder_));
    arm.push_back(capsule("upper_arm", up0, up1, r_upper_));
    arm.push_back(capsule("forearm", fa0, fa1, r_fore_));
    arm.push_back(capsule("wrist_1", o(3), o(4), r_wrist_));
    arm.push_back(capsule("wrist_2", o(4), o(5), r_wrist_));
    tool.clear();
    for (const auto &p : tool_) tool.push_back(transform(p, F[6]));
  }

  // CollisionModel.min_distance: the pairs of pair_distances, the first minimum wins
  double min_distance(const double *q, std::string *name_a = nullptr, std::string *name_b = nullptr) const
  {
    std::vector<Prim> arm, tool;
    bodies(q, arm, tool);
    double best = kInf;
    const Prim *ba = nullptr;
    std::string bb;
    // Python's min(pairs, key=distance): the FIRST pair with the smallest distance
    auto consider = [&](double d, const Prim &a, const std::string &b) {
      if (ba == nullptr || d < best) { best = d; ba = &a; bb = b; }
    };
    auto visit = [&](const Prim &a) {
      for (const auto &s : scene_) consider(primitive_distance(a, s), a, s.name);
      if (has_table_ &&
          std::find(table_exempt_.begin(), table_exempt_.end(), a.name) == table_exempt_.end())
        consider(std::max(0.0, lowest_point_z(a) - table_z_), a, "table");
    };
    for (const auto &a : arm) visit(a);
    for (const auto &t : tool) visit(t);
    for (const auto &t : tool)
      for (const auto &n : self_pairs_)
        for (const auto &a : arm)
          if (a.name == n) consider(primitive_distance(t, a), t, n);
    if (ba == nullptr) return kInf;
    if (name_a) *name_a = ba->name;
    if (name_b) *name_b = bb;
    return best;
  }

  bool in_limits(const double *q) const
  {
    for (int i = 0; i < 6; ++i)
      if (q[i] < lo_[i] || q[i] > hi_[i]) return false;
    return true;
  }

  bool is_valid(const double *q) const { return in_limits(q) && !(min_distance(q) < clearance_); }

  double clearance() const { return clearance_; }
  double lo(int i) const { return lo_[i]; }
  double hi(int i) const { return hi_[i]; }

private:
  std::array<std::array<double, 6>, 6> kin_{};
  double r_base_ = 0, r_shoulder_ = 0, r_upper_ = 0, r_fore_ = 0, r_wrist_ = 0;
  double shoulder_offset_ = 0, elbow_offset_ = 0;
  std::vector<Prim> tool_, scene_;
  bool has_table_ = false;
  double table_z_ = 0;
  std::vector<std::string> table_exempt_, self_pairs_;
  double clearance_ = 0.01;
  double lo_[6] = {}, hi_[6] = {};
};

namespace
{
std::array<double, 6> as_q(const py::array_t<double, py::array::c_style | py::array::forcecast> &a)
{
  if (a.size() != 6) throw std::invalid_argument("q must have 6 joints");
  std::array<double, 6> q;
  for (int i = 0; i < 6; ++i) q[i] = a.data()[i];
  return q;
}

// ---------------------------------------------------------------- OMPL ----------------
class Checker : public ob::StateValidityChecker
{
public:
  Checker(const ob::SpaceInformationPtr &si, const Model &m, const std::array<double, 6> &w,
          std::atomic<long> *calls)
    : ob::StateValidityChecker(si), m_(m), w_(w), calls_(calls) {}
  bool isValid(const ob::State *s) const override
  {
    const auto *v = s->as<ob::RealVectorStateSpace::StateType>();
    double q[6];
    for (int j = 0; j < 6; ++j) q[j] = v->values[j] / w_[j];
    calls_->fetch_add(1, std::memory_order_relaxed);
    return m_.is_valid(q);
  }

private:
  const Model &m_;
  std::array<double, 6> w_;
  std::atomic<long> *calls_;
};

py::dict plan(const Model &model, const py::array_t<double, py::array::c_style | py::array::forcecast> &q_from,
              const py::array_t<double, py::array::c_style | py::array::forcecast> &q_to,
              const std::string &planner_name, double budget_s, std::vector<double> weights,
              double resolution, double range, int num_planners, int max_paths)
{
  const auto q0 = as_q(q_from), q1 = as_q(q_to);
  if (weights.size() != 6) throw std::invalid_argument("weights: 6 numbers");
  std::array<double, 6> w;
  for (int j = 0; j < 6; ++j) w[j] = weights[j];
  const double w_min = *std::min_element(w.begin(), w.end());

  ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
  auto space = std::make_shared<ob::RealVectorStateSpace>(6);
  ob::RealVectorBounds bounds(6);
  for (int j = 0; j < 6; ++j) {
    bounds.setLow(j, model.lo(j) * w[j]);
    bounds.setHigh(j, model.hi(j) * w[j]);
  }
  space->setBounds(bounds);
  auto si = std::make_shared<ob::SpaceInformation>(space);
  std::atomic<long> calls{0};
  si->setStateValidityChecker(std::make_shared<Checker>(si, model, w, &calls));
  // a scaled step of resolution * min(w) moves no joint more than `resolution` rad
  si->setStateValidityCheckingResolution(resolution * w_min / space->getMaximumExtent());
  si->setup();

  ob::ScopedState<ob::RealVectorStateSpace> start(space), goal(space);
  for (int j = 0; j < 6; ++j) { start[j] = q0[j] * w[j]; goal[j] = q1[j] * w[j]; }
  auto pdef = std::make_shared<ob::ProblemDefinition>(si);
  pdef->setStartAndGoalStates(start, goal);
  auto obj = std::make_shared<ob::PathLengthOptimizationObjective>(si);
  obj->setCostThreshold(ob::Cost(0.0));     // never "good enough": use the whole budget
  pdef->setOptimizationObjective(obj);

  ob::PlannerPtr planner;
  if (planner_name == "aps") {
    auto aps = std::make_shared<og::AnytimePathShortening>(si);
    for (int i = 0; i < std::max(1, num_planners); ++i) {
      auto rrtc = std::make_shared<og::RRTConnect>(si);
      if (range > 0) rrtc->setRange(range);
      ob::PlannerPtr p = rrtc;
      aps->addPlanner(p);
    }
    aps->setHybridize(true);
    aps->setShortcut(true);
    aps->setMaxHybridizationPath(static_cast<unsigned int>(std::max(2, max_paths)));
    planner = aps;
  } else if (planner_name == "rrt_connect") {
    auto rrtc = std::make_shared<og::RRTConnect>(si);
    if (range > 0) rrtc->setRange(range);
    planner = rrtc;
  } else {
    throw std::invalid_argument("planner: 'aps' or 'rrt_connect'");
  }
  planner->setProblemDefinition(pdef);
  planner->setup();

  const auto t0 = std::chrono::steady_clock::now();
  ob::PlannerStatus status;
  {
    py::gil_scoped_release release;            // APS threads run in parallel
    status = planner->solve(ob::timedPlannerTerminationCondition(budget_s));
  }
  const double elapsed =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

  py::dict out;
  const bool exact = status == ob::PlannerStatus::EXACT_SOLUTION;
  out["solved"] = exact;
  out["status"] = status.asString();
  out["time_s"] = elapsed;
  out["validity_calls"] = calls.load();
  out["solutions"] = static_cast<int>(pdef->getSolutionCount());
  std::vector<std::vector<double>> path;
  double cost = kInf;
  if (exact && pdef->hasSolution()) {
    auto *pg = pdef->getSolutionPath()->as<og::PathGeometric>();
    cost = pg->length();
    for (std::size_t k = 0; k < pg->getStateCount(); ++k) {
      const auto *v = pg->getState(k)->as<ob::RealVectorStateSpace::StateType>();
      std::vector<double> q(6);
      for (int j = 0; j < 6; ++j) q[j] = v->values[j] / w[j];
      path.push_back(q);
    }
  }
  out["cost"] = cost;                          // weighted joint travel (scaled length)
  out["path"] = path;
  return out;
}
}  // namespace

PYBIND11_MODULE(_transit_cpp, m)
{
  m.doc() = "The marking cell's collision model in C++ and OMPL's AnytimePathShortening "
            "(notes/aps_transit_plan.md).";
  py::class_<Model>(m, "Model")
    .def(py::init<const py::dict &>(), py::arg("spec"))
    .def("frames", [](const Model &self, const py::array_t<double, py::array::c_style | py::array::forcecast> &q) {
      const auto qq = as_q(q);
      const auto F = self.frames(qq.data());
      std::vector<Mat4> out(F.begin(), F.end());
      return out;
    })
    .def("min_distance", [](const Model &self, const py::array_t<double, py::array::c_style | py::array::forcecast> &q) {
      const auto qq = as_q(q);
      std::string a, b;
      const double d = self.min_distance(qq.data(), &a, &b);
      return py::make_tuple(d, a, b);
    })
    .def("is_valid", [](const Model &self, const py::array_t<double, py::array::c_style | py::array::forcecast> &q) {
      const auto qq = as_q(q);
      return self.is_valid(qq.data());
    })
    .def("is_valid_many", [](const Model &self, const py::array_t<double, py::array::c_style | py::array::forcecast> &Q) {
      if (Q.ndim() != 2 || Q.shape(1) != 6) throw std::invalid_argument("Q must be (n, 6)");
      const auto n = Q.shape(0);
      std::vector<std::array<double, 6>> rows(static_cast<std::size_t>(n));
      auto r = Q.unchecked<2>();
      for (py::ssize_t i = 0; i < n; ++i)
        for (int j = 0; j < 6; ++j) rows[static_cast<std::size_t>(i)][j] = r(i, j);
      std::vector<char> ok(static_cast<std::size_t>(n), 0);
      {
        py::gil_scoped_release release;
        for (std::size_t i = 0; i < rows.size(); ++i) ok[i] = self.is_valid(rows[i].data()) ? 1 : 0;
      }
      // a numpy bool array copied from the 0/1 bytes (`py::array_t<bool>(n)` filled through
      // mutable_unchecked returned every element as the LAST row's verdict here)
      return py::array(py::dtype("bool"), {static_cast<py::ssize_t>(n)},
                       {static_cast<py::ssize_t>(sizeof(char))}, ok.data());
    })
    .def_property_readonly("clearance", &Model::clearance);
  m.def("plan", &plan, py::arg("model"), py::arg("q_from"), py::arg("q_to"),
        py::arg("planner") = "aps", py::arg("budget_s") = 1.0,
        py::arg("weights") = std::vector<double>{2.0, 2.0, 1.5, 1.0, 1.0, 0.5},
        py::arg("resolution") = 0.01, py::arg("range") = 0.0, py::arg("num_planners") = 4,
        py::arg("max_paths") = 8,
        "Plan a collision-free joint path q_from -> q_to with OMPL (planner 'aps' = "
        "AnytimePathShortening over num_planners RRT-Connect threads, hybridized and "
        "shortcut until budget_s; 'rrt_connect' = the first RRT-Connect solution). "
        "Returns {'solved', 'status', 'path' (list of q), 'cost' (weighted joint travel), "
        "'time_s', 'validity_calls', 'solutions'}.");
}
