// Utility functions for accessing a mesh, stored as a vector of
// ConvexPolygons.
#ifndef WALNUT_MESH_H__
#define WALNUT_MESH_H__

#include <vector>
#include <unordered_map>
#include <utility>

#include "walnut/big_int.h"
#include "walnut/convex_polygon.h"
#include "walnut/homo_point3.h"
#include "walnut/vector3.h"

namespace walnut {

template<typename Polygon>
HomoPoint3 GetTopPoint(const std::vector<Polygon>& mesh) {
  HomoPoint3 top(0, 0, 0, 0);

  for (const Polygon& polygon : mesh) {
    for (size_t i = 0; i < polygon.vertex_count(); ++i) {
      const HomoPoint3& point = polygon.vertex(i);
      if (top.w().IsZero() || HomoPoint3::TopnessLt(top, point)) {
        top = point;
      }
    }
  }
  return top;
}

// Gets the centroid of the solid defined by `mesh` as its border.
//
// `Polygon` must inherit from `ConvexPolygon`.
template<typename Polygon>
HomoPoint3 GetCentroid(const std::vector<Polygon>& mesh) {
  struct HashArray {
    size_t operator()(const std::array<BigInt, 3>& a) const {
      size_t b_hash = a[1].GetHash();
      size_t c_hash = a[2].GetHash();
      return a[0].GetHash() +
        ((b_hash << 1) | (b_hash >> (sizeof(size_t) * 8 - 1))) +
        ((c_hash << 2) | (c_hash >> (sizeof(size_t) * 8 - 2)));
    }
  };

  // Indexed by a sorted array of the w components of the triangle vertices.
  // Points to an index of `denom_values`.
  using DenomMap = std::unordered_map<std::array<BigInt, 3>, size_t, HashArray>;
  DenomMap denom_map;
  // Indexed by the triangle number. Points to an index of `denom_values`.
  std::vector<size_t> denom_indices;

  struct DenomValue {
    DenomValue(BigInt&& contribution, BigInt&& numerator_multiple) :
        contribution(std::move(contribution)),
        numerator_multiple(std::move(numerator_multiple)) {
    }

    // The contribution of this denominator to the overall least common
    // multiple of all the denominators.
    //
    // Specicifically it is the value of this denominator divided by its
    // greatest common divisor with `up_to_lcm`.
    BigInt contribution;
    // lcm / contribution
    BigInt numerator_multiple;
    // (lcm / contribution)^3
    BigInt numerator_multiple_cubed;
  };
  std::vector<DenomValue> denom_values;

  std::array<BigInt, 3> tri_denom;
  std::array<BigInt, 3> canonicalized_tri_denom;
  BigInt up_to_lcm(1);
  for (const Polygon& polygon : mesh) {
    if (polygon.vertex_count() < 3) {
      continue;
    }
    // `point_A` is common with all triangles in `polygon`.
    const HomoPoint3& point_a = polygon.vertex(0);
    tri_denom[0] = point_a.w();
    for (size_t i = 2; i < polygon.vertex_count(); ++i) {
      const HomoPoint3& point_b = polygon.vertex(i - 1);
      const HomoPoint3& point_c = polygon.vertex(i);
      tri_denom[1] = point_b.w();
      tri_denom[2] = point_c.w();
      std::pair<typename DenomMap::iterator, bool> existing =
        denom_map.emplace(tri_denom, -1);
      if (existing.second) {
        // The element was freshly inserted (did not exist yet). Check if the
        // element exists in a canonicalized form.
        size_t sorted[3] = {0, 1, 2};
        std::sort(&sorted[0], &sorted[2],
                  [&tri_denom](size_t a, size_t b) {
                    return tri_denom[a] < tri_denom[b];
                  });
        for (size_t j = 0; j < 3; ++j) {
          canonicalized_tri_denom[j] = tri_denom[sorted[j]];
        }
        std::pair<typename DenomMap::iterator, bool> canonicalized_existing =
          denom_map.emplace(canonicalized_tri_denom, -1);
        if (canonicalized_existing.second ||
            canonicalized_existing.first->second == size_t(-1)) {
          // Even the canonicalized version isn't in the map yet.
          canonicalized_existing.first->second = denom_values.size();
          // Note that inserting the canonicalized entry invalidated
          // `existing.first`. So look it up again.
          existing.first = denom_map.find(tri_denom);

          BigInt denom_product = tri_denom[0] * tri_denom[1] * tri_denom[2];
          BigInt gcd = up_to_lcm.GetGreatestCommonDivisor(denom_product);
          denom_values.emplace_back(/*contribution=*/denom_product / gcd, /*numerator_multiple=*/up_to_lcm / gcd);
          up_to_lcm *= denom_values.back().contribution;
        }
        existing.first->second = canonicalized_existing.first->second;
      }
      denom_indices.push_back(existing.first->second);
    }
  }
  BigInt after_lcm(1);
  for (auto it = denom_values.rbegin(); it != denom_values.rend(); ++it) {
    it->numerator_multiple *= after_lcm;
    it->numerator_multiple_cubed = it->numerator_multiple *
                                   it->numerator_multiple *
                                   it->numerator_multiple;
    after_lcm *= it->contribution;
  }
  assert(up_to_lcm == after_lcm);

  // volume = volume_numerator / after_lcm / 6
  BigInt volume_numerator(0);
  // centroid = 1 / 48 / volume * centroid_numerator / after_lcm^3
  Vector3 centroid_numerator(0, 0, 0);

  std::vector<size_t>::iterator denom_index_it = denom_indices.begin();
  for (const Polygon& polygon : mesh) {
    if (polygon.vertex_count() < 3) {
      continue;
    }
    // `point_A` is common with all triangles in `polygon`.
    const HomoPoint3& point_a = polygon.vertex(0);
    tri_denom[0] = point_a.w();
    for (size_t i = 2; i < polygon.vertex_count(); ++i, ++denom_index_it) {
      const HomoPoint3& point_b = polygon.vertex(i - 1);
      const HomoPoint3& point_c = polygon.vertex(i);

      const Vector3& point_a_v = point_a.vector_from_origin();
      const Vector3& point_b_v = point_b.vector_from_origin();
      const Vector3& point_c_v = point_c.vector_from_origin();

      volume_numerator += point_a_v.Dot(point_b_v.Cross(point_c_v)) *
        denom_values[*denom_index_it].numerator_multiple;

      centroid_numerator += (
          point_c.w() * point_a_v.Cross(point_b_v) +
          point_a.w() * point_b_v.Cross(point_c_v) +
          point_b.w() * point_c_v.Cross(point_a_v)
        ).Hadamard(
          (point_c.w() * (point_b.w() * point_a_v +
                          point_a.w() * point_b_v)).HadamardSquared() +
          (point_a.w() * (point_c.w() * point_b_v +
                          point_b.w() * point_c_v)).HadamardSquared() +
          (point_b.w() * (point_a.w() * point_c_v +
                          point_c.w() * point_a_v)).HadamardSquared()) *
        denom_values[*denom_index_it].numerator_multiple_cubed;
    }
  }

  return HomoPoint3(centroid_numerator,
                    BigInt(8) * volume_numerator * after_lcm * after_lcm);
}

// Returns the point out of the mesh that is farthest in the direction v is
// pointing.
template<typename Polygon>
HomoPoint3 GetFarthest(const std::vector<Polygon>& mesh, const Vector3& v) {
  HomoPoint3 farthest(0, 0, 0, 0);
  BigInt best_num(0);

  for (const Polygon& polygon : mesh) {
    for (size_t i = 0; i < polygon.vertex_count(); ++i) {
      const HomoPoint3& point = polygon.vertex(i);
      BigInt point_dist = point.vector_from_origin().Dot(v);
      if (farthest.w().IsZero() ||
          rational::IsLessThan(best_num, farthest.w(), point_dist, point.w())) {
        farthest = point;
        best_num = std::move(point_dist);
      }
    }
  }
  return farthest;
}

}  // walnut

#endif // WALNUT_MESH_H__
