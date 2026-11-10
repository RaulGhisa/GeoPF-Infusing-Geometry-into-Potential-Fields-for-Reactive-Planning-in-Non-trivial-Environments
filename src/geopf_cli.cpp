// geopf — command-line front end for the GeoPF reactive planner.
//
// Reads a scene (start, goal, obstacles, gains) from YAML/JSON files, stdin,
// inline text and/or per-obstacle flags, runs the same point-mass
// potential-field integration as MockupAgent.cpp and prints the result
// (summary + trajectory) as one line of JSON on stdout. See README.md
// ("Command-line tool") for the full reference.
//
// Exit codes: 0 = goal reached, 1 = planning failed (max iterations,
// collision or numerical error), 2 = invalid usage / configuration.

#include "../include/attr_force.h"
#include "../include/geometry_force.h"

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef GEOPF_VERSION
#define GEOPF_VERSION "0.0.0"
#endif

// Global distance statistics updated by the force_*.cpp functions.
double min_dist = 1000;
double ave_dist = 0;
double ave_num = 0;

namespace {

using Eigen::Vector3d;

struct UsageError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

// ---------------------------------------------------------------- scene ---

struct Sphere { Vector3d pos; double rad; };
struct Line { Vector3d start, end; };
struct Plane { Vector3d p[4]; };
struct Cubic { Vector3d p[8]; };
struct Cylinder { Vector3d start, end; double rad; };

struct Scene {
  Vector3d start, goal;
  bool has_boundary = false;
  Vector3d boundary_pos = Vector3d::Zero();
  double boundary_rad = 0;
  std::vector<Sphere> spheres;
  std::vector<Line> lines;
  std::vector<Plane> planes;
  std::vector<Cubic> cubics;
  std::vector<Cylinder> cylinders;

  std::size_t num_obstacles() const {
    return spheres.size() + lines.size() + planes.size() + cubics.size() +
           cylinders.size();
  }
};

struct Params {
  double k_attr = 1.0, k_boundary = 0.1, k_circ = 0.1, k_line = 0.1,
         k_plane = 0.1, k_cubic = 0.1, k_cylinder = 0.1;
  double dt = 0.01, v_max = 0.04, mass = 1.0, goal_tolerance = 0.01;
  double sphere_rad = 0.01;  // radius for spheres given as a bare [x, y, z]
  double d_influence = 0.0;  // 0 = default (0.05 or 0.1, times length_scale)
  double length_scale = 1.0;  // scene units per metre (1000 for millimetres)
  long max_iter = 100000;
  std::set<std::string> given;  // parameters set explicitly by the user

  // Defaults of length parameters are in metres; convert them to scene units.
  void apply_length_scale() {
    if (!(length_scale > 0)) throw UsageError("length_scale must be positive");
    if (!given.count("v_max")) v_max *= length_scale;
    if (!given.count("goal_tolerance")) goal_tolerance *= length_scale;
    if (!given.count("sphere_rad")) sphere_rad *= length_scale;
  }
};

// ------------------------------------------------------------- parsing ---

Vector3d vec3_at(const std::vector<double> &v, std::size_t i) {
  return Vector3d(v[i], v[i + 1], v[i + 2]);
}

double parse_double_string(const std::string &s, const std::string &what) {
  try {
    std::size_t used = 0;
    double d = std::stod(s, &used);
    if (used != s.size()) throw 0;
    return d;
  } catch (...) {
    throw UsageError(what + ": expected a number but got '" + s + "'");
  }
}

Vector3d as_vec3(const YAML::Node &n, const std::string &what) {
  if (!n || !n.IsSequence() || n.size() != 3)
    throw UsageError(what + ": expected a list of 3 numbers [x, y, z]");
  return Vector3d(n[0].as<double>(), n[1].as<double>(), n[2].as<double>());
}

// Accepts `0.1` as well as the legacy `[0.1]` form.
double as_scalar(const YAML::Node &n, const std::string &what) {
  if (n && n.IsScalar()) return n.as<double>();
  if (n && n.IsSequence() && n.size() == 1) return n[0].as<double>();
  throw UsageError(what + ": expected a number");
}

// Calls fn(entry, label) for every entry of an obstacle section. Sections can
// be a list (`- {...}`) or a map (`circle1: {...}`, the original format).
template <typename F>
void for_each_entry(const YAML::Node &section, const std::string &name, F fn) {
  if (!section || section.IsNull()) return;
  if (section.IsSequence()) {
    for (std::size_t i = 0; i < section.size(); ++i)
      fn(section[i], name + "[" + std::to_string(i) + "]");
  } else if (section.IsMap()) {
    for (const auto &kv : section)
      fn(kv.second, name + "." + kv.first.as<std::string>());
  } else {
    throw UsageError("'" + name + "' must be a list or a map");
  }
}

// N corner points given either as `point1..pointN` keys or a `points` list.
template <std::size_t N>
void read_points(const YAML::Node &e, const std::string &label, Vector3d (&out)[N]) {
  if (e["points"]) {
    const YAML::Node pts = e["points"];
    if (!pts.IsSequence() || pts.size() != N)
      throw UsageError(label + ".points: expected " + std::to_string(N) + " points");
    for (std::size_t i = 0; i < N; ++i)
      out[i] = as_vec3(pts[i], label + ".points[" + std::to_string(i) + "]");
  } else {
    for (std::size_t i = 0; i < N; ++i) {
      const std::string key = "point" + std::to_string(i + 1);
      out[i] = as_vec3(e[key], label + "." + key);
    }
  }
}

YAML::Node first_of(const YAML::Node &e, std::initializer_list<const char *> keys) {
  for (const char *k : keys)
    if (e[k]) return e[k];
  return YAML::Node(YAML::NodeType::Undefined);
}

// A stack of config files; later files override earlier ones per top-level key.
class Config {
 public:
  // A file path, or "-" for standard input.
  void add_file(const std::string &path) {
    if (path == "-") {
      if (read_stdin_) throw UsageError("standard input ('-') can only be read once");
      read_stdin_ = true;
      std::ostringstream ss;
      ss << std::cin.rdbuf();
      add_text(ss.str(), "<stdin>");
      return;
    }
    try {
      add_node(YAML::LoadFile(path), path);
    } catch (const YAML::Exception &ex) {
      throw UsageError(path + ": " + ex.what());
    }
  }
  // Inline YAML or JSON text.
  void add_text(const std::string &text, const std::string &label) {
    try {
      add_node(YAML::Load(text), label);
    } catch (const YAML::Exception &ex) {
      throw UsageError(label + ": " + ex.what());
    }
  }
  YAML::Node get(const std::string &key) const {
    for (auto it = nodes_.rbegin(); it != nodes_.rend(); ++it)
      if ((*it)[key]) return (*it)[key];
    return YAML::Node(YAML::NodeType::Undefined);
  }
  YAML::Node get_any(std::initializer_list<const char *> keys) const {
    for (const char *k : keys)
      if (YAML::Node n = get(k)) return n;
    return YAML::Node(YAML::NodeType::Undefined);
  }

 private:
  void add_node(const YAML::Node &n, const std::string &label) {
    if (!n.IsNull() && !n.IsMap())
      throw UsageError(label + ": top level must be a mapping");
    nodes_.push_back(n);
  }
  std::vector<YAML::Node> nodes_;
  bool read_stdin_ = false;
};

void read_params(const Config &cfg, Params &p) {
  const std::pair<const char *, double *> keys[] = {
      {"k_attr", &p.k_attr},       {"k_boundary", &p.k_boundary},
      {"k_circ", &p.k_circ},       {"k_line", &p.k_line},
      {"k_plane", &p.k_plane},     {"k_cubic", &p.k_cubic},
      {"k_cylinder", &p.k_cylinder}, {"dt", &p.dt},
      {"v_max", &p.v_max},         {"mass", &p.mass},
      {"goal_tolerance", &p.goal_tolerance}, {"sphere_rad", &p.sphere_rad},
      {"d_influence", &p.d_influence}, {"length_scale", &p.length_scale},
  };
  for (const auto &kv : keys)
    if (YAML::Node n = cfg.get(kv.first)) {
      *kv.second = as_scalar(n, kv.first);
      p.given.insert(kv.first);
    }
  if (YAML::Node n = cfg.get_any({"max_iter", "limit_iter"}))
    p.max_iter = static_cast<long>(as_scalar(n, "max_iter"));
}

void read_scene(const Config &cfg, const Params &p, Scene &s, bool have_start,
                bool have_goal) {
  if (!have_start) {
    YAML::Node n = cfg.get("start");
    if (!n) throw UsageError("no start position (use --start or a 'start' key)");
    s.start = as_vec3(n, "start");
  }
  if (!have_goal) {
    YAML::Node n = cfg.get("goal");
    if (!n) throw UsageError("no goal position (use --goal or a 'goal' key)");
    s.goal = as_vec3(n, "goal");
  }

  // Accept the correct spelling and the original typo "worksapce_boundary".
  if (YAML::Node b = cfg.get_any({"workspace_boundary", "worksapce_boundary"})) {
    if (!b.IsNull()) {
      s.has_boundary = true;
      s.boundary_pos = as_vec3(b["pos"], "workspace_boundary.pos");
      s.boundary_rad = as_scalar(b["rad"], "workspace_boundary.rad");
    }
  }

  for_each_entry(cfg.get_any({"sphere", "circle"}), "sphere",
                 [&](const YAML::Node &e, const std::string &l) {
                   if (e.IsSequence())
                     s.spheres.push_back({as_vec3(e, l), p.sphere_rad});
                   else
                     s.spheres.push_back(
                         {as_vec3(e["pos"], l + ".pos"),
                          e["rad"] ? as_scalar(e["rad"], l + ".rad") : p.sphere_rad});
                 });
  for_each_entry(cfg.get("line"), "line",
                 [&](const YAML::Node &e, const std::string &l) {
                   s.lines.push_back({as_vec3(e["start"], l + ".start"),
                                      as_vec3(e["end"], l + ".end")});
                 });
  for_each_entry(cfg.get("plane"), "plane",
                 [&](const YAML::Node &e, const std::string &l) {
                   Plane pl;
                   read_points(e, l, pl.p);
                   s.planes.push_back(pl);
                 });
  for_each_entry(cfg.get_any({"cubic", "cuboid"}), "cubic",
                 [&](const YAML::Node &e, const std::string &l) {
                   Cubic c;
                   read_points(e, l, c.p);
                   s.cubics.push_back(c);
                 });
  for_each_entry(cfg.get("cylinder"), "cylinder",
                 [&](const YAML::Node &e, const std::string &l) {
                   s.cylinders.push_back(
                       {as_vec3(first_of(e, {"start", "obs_start"}), l + ".start"),
                        as_vec3(first_of(e, {"end", "obs_end"}), l + ".end"),
                        as_scalar(first_of(e, {"rad", "obs_rad"}), l + ".rad")});
                 });
}

// ------------------------------------------------------------ planning ---

struct Result {
  std::string status;  // reached_goal | max_iterations | collision | numerical_error
  std::vector<Vector3d> trajectory;
  long iterations = 0;
  double path_length = 0;
  double runtime_us = 0;
};

// Obstacle shapes prepared once per run (corner sorting, box frame).
struct Prepared {
  std::vector<Quad> planes;
  std::vector<Box> cubics;
  explicit Prepared(const Scene &s) {
    for (const auto &o : s.planes) planes.push_back(make_quad(o.p[0], o.p[1], o.p[2], o.p[3]));
    for (const auto &o : s.cubics) cubics.push_back(make_box(o.p));
  }
};

// Smallest obstacle distance along the whole step a -> b. The samples alone
// miss obstacles passed between two steps (walls, lines, thin boxes or a
// large v_max); 0 means the step went through a surface.
double step_min_dist(const Scene &s, const Prepared &g, const Vector3d &a, const Vector3d &b) {
  double d = std::numeric_limits<double>::infinity();
  for (const auto &o : s.spheres) d = std::min(d, line_dist_obs(a, b, o.pos).norm() - o.rad);
  for (const auto &o : s.lines) d = std::min(d, segment_segment_distance(a, b, o.start, o.end));
  for (const auto &q : g.planes)
    if (segment_crosses_quad(q, a, b)) d = std::min(d, 0.0);
  for (const auto &bx : g.cubics)
    if (segment_hits_box(bx, a, b)) d = std::min(d, std::min(0.0, box_signed_distance(bx, b)));
  for (const auto &o : s.cylinders)
    if (segment_hits_cylinder(o.start, o.end, o.rad, a, b)) d = std::min(d, 0.0);
  return d;
}

// Same integration scheme as MockupAgent.cpp.
Result plan(const Scene &s, const Params &p) {
  Result r;
  min_dist = 1000;
  ave_dist = 0;
  ave_num = 0;
  const Prepared geo(s);

  // The paper's gains and distances are in metres. With S scene units per
  // metre, lengths scale by S, the attraction (a force) by S and the
  // repulsive gains (force x distance) by S^2, so a scene planned in
  // millimetres with length_scale 1000 follows the same path as in metres.
  const double S = p.length_scale;
  const double d_inf = p.d_influence > 0.0 ? p.d_influence : 0.1 * S;
  const double d_circ = p.d_influence > 0.0 ? p.d_influence : 0.05 * S;
  const double slow_dist = 0.1 * S;
  const double k_attr = p.k_attr * S;
  const double S2 = S * S;

  Vector3d robot_pos = s.start;
  Vector3d robot_vel = Vector3d::Zero();
  Vector3d acc = Vector3d::Zero();
  double e = 1111 * S;  // as in the original: no slow-down on the first step

  const auto t0 = std::chrono::high_resolution_clock::now();
  if ((s.goal - s.start).norm() <= p.goal_tolerance) {
    r.status = "reached_goal";
  }
  while (r.status.empty()) {
    Vector3d force = attr_force(s.goal, robot_pos, k_attr);
    if (s.has_boundary)
      force += boundary_force(s.boundary_pos, s.boundary_rad, robot_pos, robot_vel,
                              s.goal, p.k_boundary * S2, d_circ);
    for (const auto &o : s.spheres)
      force += sphere_force(o.pos, o.rad, robot_pos, robot_vel, s.goal, p.k_circ * S2, d_circ);
    for (const auto &o : s.lines)
      force += line_force(o.start, o.end, robot_pos, robot_vel, s.goal, p.k_line * S2, d_circ);
    for (const auto &q : geo.planes)
      force += quad_force(q, robot_pos, robot_vel, s.goal, p.k_plane * S2, d_inf,
                          SteerEdge::NearestRobot).head<3>();
    for (const auto &b : geo.cubics)
      force += box_force(b, robot_pos, robot_vel, s.goal, p.k_cubic * S2, d_inf);
    for (const auto &o : s.cylinders)
      force += cylinder_force(o.start, o.end, o.rad, robot_pos, robot_vel, s.goal,
                              p.k_cylinder * S2, d_inf);

    acc = force / p.mass;
    robot_vel = robot_vel + acc * p.dt;
    if (robot_vel.norm() > p.v_max) robot_vel = robot_vel * p.v_max / robot_vel.norm();
    if (e < slow_dist) robot_vel = robot_vel * (e / slow_dist);

    const Vector3d last = robot_pos;
    robot_pos = robot_pos + 0.5 * acc * p.dt * p.dt + robot_vel * p.dt;
    r.path_length += (robot_pos - last).norm();
    e = (s.goal - robot_pos).norm();

    r.iterations++;
    r.trajectory.push_back(robot_pos);

    if (robot_pos.allFinite())
      min_dist = std::min(min_dist, step_min_dist(s, geo, last, robot_pos));

    if (!robot_pos.allFinite()) r.status = "numerical_error";
    else if (min_dist < 1.e-8) r.status = "collision";
    else if (e <= p.goal_tolerance) r.status = "reached_goal";
    else if (r.iterations >= p.max_iter) r.status = "max_iterations";
  }
  const auto t1 = std::chrono::high_resolution_clock::now();
  r.runtime_us =
      std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
  return r;
}

// -------------------------------------------------------------- output ---

void write_rows_csv(const std::string &path, const std::vector<Eigen::VectorXd> &rows) {
  std::ofstream f(path);
  if (!f) throw std::runtime_error("cannot write '" + path + "'");
  f << std::setprecision(12);
  for (const auto &row : rows) {
    for (Eigen::Index j = 0; j < row.size(); ++j) f << (j ? "," : "") << row(j);
    f << "\n";
  }
}

template <typename T, typename F>
void dump_section(const std::string &path, const std::vector<T> &items, F to_row) {
  std::vector<Eigen::VectorXd> rows;
  for (const auto &it : items) rows.push_back(to_row(it));
  write_rows_csv(path, rows);
}

// Same file names as MockupAgent.cpp so the matlabPlot/ scripts can read them.
void dump_scene(const std::string &dir, const Scene &s, const Result &r) {
  auto v = [](const Vector3d &x) { return Eigen::VectorXd(x); };
  auto sc = [](double d) { Eigen::VectorXd x(1); x << d; return x; };
  dump_section(dir + "/trajectory.csv", r.trajectory, v);
  std::vector<Eigen::VectorXd> wb;
  if (s.has_boundary) {
    Eigen::VectorXd b(4);
    b << s.boundary_pos, s.boundary_rad;
    for (int i = 0; i < 4; ++i) wb.push_back(sc(b(i)));  // 4x1 column, as original
  }
  write_rows_csv(dir + "/workspaceboundary.csv", wb);
  dump_section(dir + "/sphere.csv", s.spheres, [&](const Sphere &o) { return v(o.pos); });
  dump_section(dir + "/sphererad.csv", s.spheres, [&](const Sphere &o) { return sc(o.rad); });
  dump_section(dir + "/linestart.csv", s.lines, [&](const Line &o) { return v(o.start); });
  dump_section(dir + "/lineend.csv", s.lines, [&](const Line &o) { return v(o.end); });
  for (int k = 0; k < 4; ++k)
    dump_section(dir + "/planepoint" + std::to_string(k + 1) + ".csv", s.planes,
                 [&](const Plane &o) { return v(o.p[k]); });
  for (int k = 0; k < 8; ++k)
    dump_section(dir + "/cubicpoint" + std::to_string(k + 1) + ".csv", s.cubics,
                 [&](const Cubic &o) { return v(o.p[k]); });
  dump_section(dir + "/cylinderstart.csv", s.cylinders, [&](const Cylinder &o) { return v(o.start); });
  dump_section(dir + "/cylinderend.csv", s.cylinders, [&](const Cylinder &o) { return v(o.end); });
  dump_section(dir + "/cylinderrad.csv", s.cylinders, [&](const Cylinder &o) { return sc(o.rad); });
}

std::string json_num(double d) {
  if (!std::isfinite(d)) return "null";
  std::ostringstream o;
  o << std::setprecision(12) << d;
  return o.str();
}

std::string json_vec(const Vector3d &x) {
  return "[" + json_num(x(0)) + "," + json_num(x(1)) + "," + json_num(x(2)) + "]";
}

std::string json_str(const std::string &s) {
  std::string out = "\"";
  for (char c : s) {
    if (c == '"' || c == '\\') out += '\\';
    if (static_cast<unsigned char>(c) < 0x20) {
      char buf[8];
      std::snprintf(buf, sizeof buf, "\\u%04x", c);
      out += buf;
    } else {
      out += c;
    }
  }
  return out + "\"";
}

void print_summary(std::ostream &os, const Scene &s, const Result &r,
                   const std::string &out_path, bool include_traj) {
  const bool measured = ave_num > 0;
  os << "{"
     << "\"status\":" << json_str(r.status)
     << ",\"success\":" << (r.status == "reached_goal" ? "true" : "false")
     << ",\"iterations\":" << r.iterations
     << ",\"path_length\":" << json_num(r.path_length)
     << ",\"min_obstacle_distance\":" << (measured ? json_num(min_dist) : "null")
     << ",\"mean_obstacle_distance\":" << (measured ? json_num(ave_dist / ave_num) : "null")
     << ",\"runtime_us\":" << json_num(r.runtime_us)
     << ",\"num_obstacles\":" << s.num_obstacles()
     << ",\"start\":" << json_vec(s.start)
     << ",\"goal\":" << json_vec(s.goal)
     << ",\"final_position\":"
     << json_vec(r.trajectory.empty() ? s.start : r.trajectory.back())
     << ",\"trajectory_file\":" << (out_path.empty() ? "null" : json_str(out_path));
  if (include_traj) {
    os << ",\"trajectory\":[";
    for (std::size_t i = 0; i < r.trajectory.size(); ++i)
      os << (i ? "," : "") << json_vec(r.trajectory[i]);
    os << "]";
  }
  os << "}\n";
}

// ----------------------------------------------------------------- cli ---

const char *kUsage = R"(usage: geopf --start X,Y,Z --goal X,Y,Z [obstacle flags] [options]
   or: geopf -c SCENE.yaml [options]

Plans a path for a point from start to goal with GeoPF potential fields and
prints the result to stdout.

scene input (flags — comma- or space-separated numbers):
      --start X,Y,Z        start position (or --start X Y Z)
      --goal X,Y,Z         goal position  (or --goal X Y Z)
      --boundary X,Y,Z,R   workspace boundary sphere (centre, radius)
      --sphere X,Y,Z[,R]   add a sphere (if R omitted, uses --sphere-rad)
                                                              (repeatable)
      --line X1,Y1,Z1,X2,Y2,Z2
                           add a line segment                 (repeatable)
      --plane P1,P2,P3,P4  add a rectangle: 12 numbers, 4 corners in any
                           order                              (repeatable)
      --cubic P1,...,P8    add a box: 24 numbers, 8 corners in any order
                                                              (repeatable)
      --cylinder X1,Y1,Z1,X2,Y2,Z2,R
                           add a cylinder (axis start, end, radius)
                                                              (repeatable)
scene input (config files / inline):
  -c, --config FILE        YAML/JSON file, or '-' to read from stdin
                           (repeatable; obstacle flags are added on top)
      --scene TEXT         inline YAML/JSON text              (repeatable)

parameters:
      --k-attr V           gain overrides (also --k-boundary, --k-circ,
                           --k-line, --k-plane, --k-cubic, --k-cylinder)
      --sphere-rad R       default sphere radius         [0.01]
      --dt V               integration step              [0.01]
      --v-max V            speed limit                   [0.04]
      --mass V             point mass                    [1.0]
      --goal-tolerance V   stop distance to goal         [0.01]
      --max-iter N         iteration limit               [100000]
      --d-influence D      distance at which obstacles start to push
                           [0.05 spheres/lines/boundary, 0.1 others]
      --length-scale S     scene units per metre, e.g. 1000 for mm  [1]
                           multiplies the defaults of the lengths above
                           (including --sphere-rad) and scales the gains
                           so the same scene in other units plans the same

output:
      --format FMT         stdout format:
                             json  one JSON object incl. "trajectory" (default)
                             csv   trajectory rows "x,y,z"; JSON summary goes
                                   to stderr instead
      --no-trajectory      leave "trajectory" out of the JSON
  -o, --out FILE           also write the trajectory CSV to FILE
      --dump-scene DIR     write trajectory + obstacle CSVs for matlabPlot/
  -h, --help               show this help
      --version            show version

exit status: 0 goal reached, 1 planning failed, 2 bad usage/config
)";

void write_traj_csv(std::ostream &os, const std::vector<Vector3d> &traj) {
  os << std::setprecision(12);
  for (const auto &p : traj) os << p(0) << "," << p(1) << "," << p(2) << "\n";
}

}  // namespace

int main(int argc, char **argv) {
  // The force functions may print diagnostics on std::cout; keep stdout clean
  // for our output by routing std::cout to stderr while planning.
  std::ostream real_stdout(std::cout.rdbuf());
  std::cout.rdbuf(std::cerr.rdbuf());

  try {
    Config cfg;
    Params params;
    Scene scene;
    Scene extra;  // obstacles given as flags
    std::vector<Vector3d> extra_spheres_default_rad;
    std::string out_path, dump_dir, format = "json";
    bool include_traj = true, have_start = false, have_goal = false,
         have_cfg = false, have_boundary_flag = false;
    std::map<std::string, double> overrides;
    long max_iter_override = -1;

    const std::map<std::string, std::string> numeric_flags = {
        {"--k-attr", "k_attr"},       {"--k-boundary", "k_boundary"},
        {"--k-circ", "k_circ"},       {"--k-line", "k_line"},
        {"--k-plane", "k_plane"},     {"--k-cubic", "k_cubic"},
        {"--k-cylinder", "k_cylinder"}, {"--dt", "dt"},
        {"--v-max", "v_max"},         {"--mass", "mass"},
        {"--goal-tolerance", "goal_tolerance"},
        {"--sphere-rad", "sphere_rad"},
        {"--d-influence", "d_influence"},
        {"--length-scale", "length_scale"},
    };

    for (int i = 1; i < argc; ++i) {
      std::string a = argv[i], val;
      bool has_inline = false;
      if (a.rfind("--", 0) == 0 && a.find('=') != std::string::npos) {
        val = a.substr(a.find('=') + 1);
        a = a.substr(0, a.find('='));
        has_inline = true;
      }
      auto next = [&]() -> std::string {
        if (has_inline) return val;
        if (i + 1 >= argc) throw UsageError(a + " needs a value");
        return argv[++i];
      };
      // Reads between min_n and max_n numbers from one comma-separated token
      // or several space-separated tokens.
      auto next_nums = [&](std::size_t min_n, std::size_t max_n,
                           const std::string &form) -> std::vector<double> {
        const std::string count_str =
            min_n == max_n ? std::to_string(min_n)
                           : (std::to_string(min_n) + " or " + std::to_string(max_n));
        const std::string msg = a + ": expected " + count_str + " numbers (" + form + ")";
        std::vector<double> v;
        auto push_token = [&](const std::string &tok) {
          std::stringstream ss(tok);
          std::string item;
          while (std::getline(ss, item, ',')) {
            if (item.find_first_not_of(" \t") == std::string::npos) continue;
            try {
              std::size_t used = 0;
              v.push_back(std::stod(item, &used));
              if (item.find_first_not_of(" \t", used) != std::string::npos) throw 0;
            } catch (...) {
              throw UsageError(msg + ", got '" + tok + "'");
            }
          }
        };
        push_token(next());
        while (v.size() < min_n && !has_inline && i + 1 < argc) {
          push_token(argv[++i]);
        }
        // If min_n < max_n (e.g. --sphere X Y Z [R]), also consume optional extra numbers
        // if the next argv token starts with a digit, '.', '+' or '-' followed by a digit/dot.
        while (v.size() < max_n && !has_inline && i + 1 < argc) {
          const std::string peek = argv[i + 1];
          bool looks_numeric = false;
          if (!peek.empty()) {
            char c0 = peek[0];
            if ((c0 >= '0' && c0 <= '9') || c0 == '.') looks_numeric = true;
            else if ((c0 == '-' || c0 == '+') && peek.size() > 1 &&
                     ((peek[1] >= '0' && peek[1] <= '9') || peek[1] == '.'))
              looks_numeric = true;
          }
          if (!looks_numeric) break;
          push_token(argv[++i]);
        }
        if (v.size() != min_n && v.size() != max_n)
          throw UsageError(msg + ", got " + std::to_string(v.size()));
        return v;
      };

      if (a == "-h" || a == "--help") { real_stdout << kUsage; return 0; }
      else if (a == "--version") { real_stdout << "geopf " << GEOPF_VERSION << "\n"; return 0; }
      else if (a == "-c" || a == "--config") { cfg.add_file(next()); have_cfg = true; }
      else if (a == "--scene") { cfg.add_text(next(), "--scene"); have_cfg = true; }
      else if (a == "--start") {
        scene.start = vec3_at(next_nums(3, 3, "x,y,z"), 0);
        have_start = true;
      } else if (a == "--goal") {
        scene.goal = vec3_at(next_nums(3, 3, "x,y,z"), 0);
        have_goal = true;
      } else if (a == "--boundary") {
        const auto v = next_nums(4, 4, "x,y,z,r");
        extra.boundary_pos = vec3_at(v, 0);
        extra.boundary_rad = v[3];
        have_boundary_flag = true;
      } else if (a == "--sphere") {
        const auto v = next_nums(3, 4, "x,y,z[,r]");
        if (v.size() == 4)
          extra.spheres.push_back({vec3_at(v, 0), v[3]});
        else
          extra_spheres_default_rad.push_back(vec3_at(v, 0));
      } else if (a == "--line") {
        const auto v = next_nums(6, 6, "start x,y,z, end x,y,z");
        extra.lines.push_back({vec3_at(v, 0), vec3_at(v, 3)});
      } else if (a == "--plane") {
        const auto v = next_nums(12, 12, "4 corners x,y,z");
        Plane pl;
        for (int k = 0; k < 4; ++k) pl.p[k] = vec3_at(v, 3 * k);
        extra.planes.push_back(pl);
      } else if (a == "--cubic") {
        const auto v = next_nums(24, 24, "8 corners x,y,z");
        Cubic c;
        for (int k = 0; k < 8; ++k) c.p[k] = vec3_at(v, 3 * k);
        extra.cubics.push_back(c);
      } else if (a == "--cylinder") {
        const auto v = next_nums(7, 7, "start x,y,z, end x,y,z, r");
        extra.cylinders.push_back({vec3_at(v, 0), vec3_at(v, 3), v[6]});
      }
      else if (a == "-o" || a == "--out") out_path = next();
      else if (a == "--dump-scene") dump_dir = next();
      else if (a == "--format") {
        format = next();
        if (format != "json" && format != "csv")
          throw UsageError("--format must be 'json' or 'csv'");
      }
      else if (a == "--no-trajectory") include_traj = false;
      else if (a == "--include-trajectory") include_traj = true;  // default; kept for compatibility
      else if (a == "--max-iter") {
        max_iter_override = static_cast<long>(parse_double_string(next(), a));
      } else if (numeric_flags.count(a)) {
        overrides[numeric_flags.at(a)] = parse_double_string(next(), a);
      } else {
        throw UsageError("unknown option '" + a + "' (see --help)");
      }
    }
    if (!have_cfg && !(have_start && have_goal))
      throw UsageError("nothing to plan: give --start/--goal, --scene or -c (see --help)");

    read_params(cfg, params);
    for (const auto &kv : overrides) {
      double *slot = nullptr;
      const std::string &k = kv.first;
      if (k == "k_attr") slot = &params.k_attr;
      else if (k == "k_boundary") slot = &params.k_boundary;
      else if (k == "k_circ") slot = &params.k_circ;
      else if (k == "k_line") slot = &params.k_line;
      else if (k == "k_plane") slot = &params.k_plane;
      else if (k == "k_cubic") slot = &params.k_cubic;
      else if (k == "k_cylinder") slot = &params.k_cylinder;
      else if (k == "dt") slot = &params.dt;
      else if (k == "v_max") slot = &params.v_max;
      else if (k == "mass") slot = &params.mass;
      else if (k == "goal_tolerance") slot = &params.goal_tolerance;
      else if (k == "sphere_rad") slot = &params.sphere_rad;
      else if (k == "d_influence") slot = &params.d_influence;
      else if (k == "length_scale") slot = &params.length_scale;
      *slot = kv.second;
      params.given.insert(k);
    }
    if (max_iter_override >= 0) params.max_iter = max_iter_override;
    params.apply_length_scale();
    if (params.dt <= 0 || params.mass <= 0 || params.v_max <= 0 || params.max_iter < 1)
      throw UsageError("dt, mass, v_max and max_iter must be positive");

    read_scene(cfg, params, scene, have_start, have_goal);
    if (have_boundary_flag) {
      scene.has_boundary = true;
      scene.boundary_pos = extra.boundary_pos;
      scene.boundary_rad = extra.boundary_rad;
    }
    auto append = [](auto &dst, const auto &src) { dst.insert(dst.end(), src.begin(), src.end()); };
    append(scene.spheres, extra.spheres);
    for (const auto &pos : extra_spheres_default_rad)
      scene.spheres.push_back({pos, params.sphere_rad});
    append(scene.lines, extra.lines);
    append(scene.planes, extra.planes);
    append(scene.cubics, extra.cubics);
    append(scene.cylinders, extra.cylinders);

    const Result result = plan(scene, params);

    if (!out_path.empty()) {
      std::ofstream f(out_path);
      if (!f) throw std::runtime_error("cannot write '" + out_path + "'");
      write_traj_csv(f, result.trajectory);
    }
    if (!dump_dir.empty()) dump_scene(dump_dir, scene, result);

    if (format == "csv") {
      write_traj_csv(real_stdout, result.trajectory);
      print_summary(std::cerr, scene, result, out_path, false);
    } else {
      print_summary(real_stdout, scene, result, out_path, include_traj);
    }
    real_stdout.flush();
    return result.status == "reached_goal" ? 0 : 1;
  } catch (const UsageError &ex) {
    std::cerr << "geopf: error: " << ex.what() << "\n";
    return 2;
  } catch (const YAML::Exception &ex) {
    std::cerr << "geopf: config error: " << ex.what() << "\n";
    return 2;
  } catch (const std::exception &ex) {
    std::cerr << "geopf: error: " << ex.what() << "\n";
    return 2;
  }
}
