// geopf — command-line front end for the GeoPF reactive planner.
//
// Reads a scene (start, goal, obstacles, gains) from one or more YAML/JSON
// files, runs the same point-mass potential-field integration as
// MockupAgent.cpp, writes the trajectory as CSV and prints a JSON summary on
// stdout. See README.md ("Command-line tool") for the full reference.
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
  long max_iter = 100000;
};

// ------------------------------------------------------------- parsing ---

Vector3d parse_vec3_string(const std::string &s, const std::string &what) {
  std::stringstream ss(s);
  std::string item;
  std::vector<double> v;
  while (std::getline(ss, item, ',')) {
    try {
      std::size_t used = 0;
      v.push_back(std::stod(item, &used));
      if (item.find_first_not_of(" \t", used) != std::string::npos) throw 0;
    } catch (...) {
      throw UsageError(what + ": expected x,y,z but got '" + s + "'");
    }
  }
  if (v.size() != 3) throw UsageError(what + ": expected x,y,z but got '" + s + "'");
  return Vector3d(v[0], v[1], v[2]);
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
  void add_file(const std::string &path) {
    try {
      YAML::Node n = YAML::LoadFile(path);
      if (!n.IsNull() && !n.IsMap())
        throw UsageError(path + ": top level must be a mapping");
      nodes_.push_back(n);
    } catch (const YAML::Exception &ex) {
      throw UsageError(path + ": " + ex.what());
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
  std::vector<YAML::Node> nodes_;
};

void read_params(const Config &cfg, Params &p) {
  const std::pair<const char *, double *> keys[] = {
      {"k_attr", &p.k_attr},       {"k_boundary", &p.k_boundary},
      {"k_circ", &p.k_circ},       {"k_line", &p.k_line},
      {"k_plane", &p.k_plane},     {"k_cubic", &p.k_cubic},
      {"k_cylinder", &p.k_cylinder}, {"dt", &p.dt},
      {"v_max", &p.v_max},         {"mass", &p.mass},
      {"goal_tolerance", &p.goal_tolerance}, {"sphere_rad", &p.sphere_rad},
  };
  for (const auto &kv : keys)
    if (YAML::Node n = cfg.get(kv.first)) *kv.second = as_scalar(n, kv.first);
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

// Same integration scheme as MockupAgent.cpp.
Result plan(const Scene &s, const Params &p) {
  Result r;
  min_dist = 1000;
  ave_dist = 0;
  ave_num = 0;

  Vector3d robot_pos = s.start;
  Vector3d robot_vel = Vector3d::Zero();
  Vector3d acc = Vector3d::Zero();
  double e = 1111;  // as in the original: no slow-down on the first step

  const auto t0 = std::chrono::high_resolution_clock::now();
  if ((s.goal - s.start).norm() <= p.goal_tolerance) {
    r.status = "reached_goal";
  }
  while (r.status.empty()) {
    Vector3d force = attr_force(s.goal, robot_pos, p.k_attr);
    if (s.has_boundary)
      force += boundary_force(s.boundary_pos, s.boundary_rad, robot_pos, robot_vel,
                              s.goal, p.k_boundary);
    for (const auto &o : s.spheres)
      force += sphere_force(o.pos, o.rad, robot_pos, robot_vel, s.goal, p.k_circ);
    for (const auto &o : s.lines)
      force += line_force(o.start, o.end, robot_pos, robot_vel, s.goal, p.k_line);
    for (const auto &o : s.planes)
      force += plane_force(o.p[0], o.p[1], o.p[2], o.p[3], robot_pos, robot_vel,
                           s.goal, p.k_plane);
    for (const auto &o : s.cubics)
      force += cubic_force(o.p[0], o.p[1], o.p[2], o.p[3], o.p[4], o.p[5], o.p[6],
                           o.p[7], robot_pos, robot_vel, s.goal, p.k_cubic);
    for (const auto &o : s.cylinders)
      force += cylinder_force(o.start, o.end, o.rad, robot_pos, robot_vel, s.goal,
                              p.k_cylinder);

    acc = force / p.mass;
    robot_vel = robot_vel + acc * p.dt;
    if (robot_vel.norm() > p.v_max) robot_vel = robot_vel * p.v_max / robot_vel.norm();
    if (e < 0.1) robot_vel = robot_vel * (10 * e);

    const Vector3d last = robot_pos;
    robot_pos = robot_pos + 0.5 * acc * p.dt * p.dt + robot_vel * p.dt;
    r.path_length += (robot_pos - last).norm();
    e = (s.goal - robot_pos).norm();

    r.iterations++;
    r.trajectory.push_back(robot_pos);

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

const char *kUsage = R"(usage: geopf -c SCENE.yaml [-c MORE.yaml ...] [options]

Plans a path for a point from start to goal with GeoPF potential fields.
Config files are YAML (or JSON); later files override earlier ones per key.

input:
  -c, --config FILE        scene/params file (repeatable)
      --start X,Y,Z        override start position
      --goal X,Y,Z         override goal position
      --k-attr V           override a gain (also --k-boundary, --k-circ,
                           --k-line, --k-plane, --k-cubic, --k-cylinder)
      --dt V               integration step              [0.01]
      --v-max V            speed limit                   [0.04]
      --mass V             point mass                    [1.0]
      --goal-tolerance V   stop distance to goal         [0.01]
      --max-iter N         iteration limit               [100000]

output:
  -o, --out FILE           write trajectory CSV (one "x,y,z" row per step)
      --include-trajectory put the trajectory in the JSON summary as well
      --dump-scene DIR     write trajectory + obstacle CSVs for matlabPlot/
  -h, --help               show this help
      --version            show version

A one-line JSON summary is always printed on stdout.
exit status: 0 goal reached, 1 planning failed, 2 bad usage/config
)";

}  // namespace

int main(int argc, char **argv) {
  // The force functions may print diagnostics on std::cout; keep stdout clean
  // for the JSON summary by routing std::cout to stderr while planning.
  std::ostream json_out(std::cout.rdbuf());
  std::cout.rdbuf(std::cerr.rdbuf());

  try {
    Config cfg;
    Params params;
    Scene scene;
    std::string out_path, dump_dir;
    bool include_traj = false, have_start = false, have_goal = false, have_cfg = false;
    std::map<std::string, double> overrides;
    long max_iter_override = -1;

    const std::map<std::string, std::string> numeric_flags = {
        {"--k-attr", "k_attr"},       {"--k-boundary", "k_boundary"},
        {"--k-circ", "k_circ"},       {"--k-line", "k_line"},
        {"--k-plane", "k_plane"},     {"--k-cubic", "k_cubic"},
        {"--k-cylinder", "k_cylinder"}, {"--dt", "dt"},
        {"--v-max", "v_max"},         {"--mass", "mass"},
        {"--goal-tolerance", "goal_tolerance"},
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

      if (a == "-h" || a == "--help") { json_out << kUsage; return 0; }
      else if (a == "--version") { json_out << "geopf " << GEOPF_VERSION << "\n"; return 0; }
      else if (a == "-c" || a == "--config") { cfg.add_file(next()); have_cfg = true; }
      else if (a == "--start") { scene.start = parse_vec3_string(next(), a); have_start = true; }
      else if (a == "--goal") { scene.goal = parse_vec3_string(next(), a); have_goal = true; }
      else if (a == "-o" || a == "--out") out_path = next();
      else if (a == "--dump-scene") dump_dir = next();
      else if (a == "--include-trajectory") include_traj = true;
      else if (a == "--max-iter") {
        max_iter_override = static_cast<long>(parse_double_string(next(), a));
      } else if (numeric_flags.count(a)) {
        overrides[numeric_flags.at(a)] = parse_double_string(next(), a);
      } else {
        throw UsageError("unknown option '" + a + "' (see --help)");
      }
    }
    if (!have_cfg && !(have_start && have_goal))
      throw UsageError("nothing to plan: give -c SCENE.yaml (see --help)");

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
      *slot = kv.second;
    }
    if (max_iter_override >= 0) params.max_iter = max_iter_override;
    if (params.dt <= 0 || params.mass <= 0 || params.v_max <= 0 || params.max_iter < 1)
      throw UsageError("dt, mass, v_max and max_iter must be positive");

    read_scene(cfg, params, scene, have_start, have_goal);

    const Result result = plan(scene, params);

    if (!out_path.empty())
      dump_section(out_path, result.trajectory,
                   [](const Vector3d &x) { return Eigen::VectorXd(x); });
    if (!dump_dir.empty()) dump_scene(dump_dir, scene, result);

    print_summary(json_out, scene, result, out_path, include_traj);
    json_out.flush();
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
