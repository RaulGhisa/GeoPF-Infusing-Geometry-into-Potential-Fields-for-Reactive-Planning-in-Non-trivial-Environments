#ifndef CIRCFORCE_H
#define CIRCFORCE_H
#include <iostream>
#include <Eigen/Dense>
#include <memory>
#include "../include/IndexMinMax.h"



using namespace Eigen;

extern double min_dist;
extern double ave_dist;
extern double ave_num;

// ------------------------------------------------------------ helpers ---

// Unit direction of the sideways push away from an obstacle: the obstacle
// normal n (pointing from the obstacle to the robot) with its component along
// the goal direction g removed. When n is (almost) parallel to g, the robot's
// current sideways velocity is continued, or failing that a fixed
// perpendicular is used.
Vector3d deflection_dir(const Vector3d &n, const Vector3d &g, const Vector3d &robot_vel);

// Shortest distance between segments [a0, a1] and [b0, b1].
double segment_segment_distance(const Vector3d &a0, const Vector3d &a1,
                                const Vector3d &b0, const Vector3d &b1);

// A rectangle with its corners sorted around the edge (any input order is
// accepted, e.g. Z order) and the unit normal n, oriented so that
// p[0] -> p[1] -> p[2] runs counter-clockwise around n.
struct Quad {
  Vector3d p[4];
  Vector3d n;
};
Quad make_quad(const Vector3d &c1, const Vector3d &c2, const Vector3d &c3, const Vector3d &c4);

// Vector from the closest point of the rectangle to the robot.
Vector3d quad_dist_obs(const Quad &q, const Vector3d &robot_pos);

// True if the segment a -> b passes through the rectangle (the endpoints are
// strictly on opposite sides of its plane and the crossing point is inside).
bool segment_crosses_quad(const Quad &q, const Vector3d &a, const Vector3d &b);

// Which edge the steering correction (paper Eq. 12) heads for when the
// straight path to the goal goes through the rectangle.
enum class SteerEdge { NearestRobot, NearestGoal };

// Plane force (first 3 entries) and whether the path to the goal is blocked
// by the rectangle (4th entry, 1 or 0).
Vector4d quad_force(const Quad &q, const Vector3d &robot_pos, const Vector3d &robot_vel,
                    const Vector3d &goal, double k_plane, double d_inf, SteerEdge edge);

// A rectangular box. make_box accepts the 8 corners in the GeoPF order
// (corner1-4 one face around the edge, corner5 above corner1, ...) or in any
// other order (e.g. x/y/z bit order) and stores them in the GeoPF order.
struct Box {
  Vector3d corner[8];
  Vector3d center;
  Vector3d axis[3];  // unit edge directions
  double half[3];    // half edge lengths
  Quad face[6];
};
Box make_box(const Vector3d (&corners)[8]);

// Signed distance to the box: negative inside. Exact inside; outside it is
// only used for its sign.
double box_signed_distance(const Box &b, const Vector3d &p);

// True if any point of the segment a -> b lies inside the box (or on it).
bool segment_hits_box(const Box &b, const Vector3d &a, const Vector3d &b_end);

Vector3d box_force(const Box &b, const Vector3d &robot_pos, const Vector3d &robot_vel,
                   const Vector3d &goal, double k_cubic, double d_inf);

// True if any point of the segment a -> b lies inside the solid cylinder.
bool segment_hits_cylinder(const Vector3d &obs_start, const Vector3d &obs_end, double obs_rad,
                           const Vector3d &a, const Vector3d &b);

// ------------------------------------------------------------- forces ---
// These keep the original signatures (corners given as separate points) for
// the research programs; the CLI uses the Quad / Box versions above so the
// geometry is prepared once per run instead of every step.

Vector3d sphere_force(const Vector3d &obs_pos, const double &obs_rad, const Vector3d &robot_pos,
                const Vector3d &robot_vel, const Vector3d &goal, const double &k_circle,
                double d_inf = 0.05);

Vector3d boundary_force(const Vector3d &obs_pos, const double &obs_rad, const Vector3d &robot_pos,
                const Vector3d &robot_vel, const Vector3d &goal, const double &k_circle,
                double d_inf = 0.05);

Vector3d line_force(const Vector3d &obs_start, const Vector3d &obs_end, const Vector3d &robot_pos,
                const Vector3d &robot_vel, const Vector3d &goal, const double &k_line,
                double d_inf = 0.05);

Vector3d line_dist_obs(const Vector3d &obs_start, const Vector3d &obs_end, const Vector3d &robot_pos);

Vector3d plane_force(const Vector3d &obs1, const Vector3d &obs2, const Vector3d &obs3, const Vector3d &obs4, const Vector3d &robot_pos, const Vector3d &robot_vel,
                    const Vector3d &goal, const double &k_plane,
                    double d_inf = 0.1);

Vector4d plane_force_closest_goal(const Vector3d &obs1, const Vector3d &obs2, const Vector3d &obs3, const Vector3d &obs4, const Vector3d &robot_pos, const Vector3d &robot_vel,
                    const Vector3d &goal, const double &k_plane,
                    double d_inf = 0.1);


Vector3d plane_dist_obs(const Vector3d &obs1, const Vector3d &obs2, const Vector3d &obs3, const Vector3d &obs4, const Vector3d &robot_pos);

int sign(double x);

Vector3d cubic_force(const Vector3d &obs1, const Vector3d &obs2, const Vector3d &obs3, const Vector3d &obs4,
                    const Vector3d &obs5, const Vector3d &obs6, const Vector3d &obs7, const Vector3d &obs8,
                    const Vector3d &robot_pos, const Vector3d &robot_vel,
                    const Vector3d &goal, const double &k_cubic,
                    double d_inf = 0.1);

Vector3d cylinder_force(const Vector3d &obs_start, const Vector3d &obs_end, const double &obs_rad,
                        const Vector3d &robot_pos, const Vector3d &robot_vel,
                        const Vector3d &goal, const double &k_cylinder,
                        double d_inf = 0.1);

#endif
