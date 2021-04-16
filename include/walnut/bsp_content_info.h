#ifndef WALNUT_BSP_CONTENT_INFO_H__
#define WALNUT_BSP_CONTENT_INFO_H__

#include <cstdint>

namespace walnut {

struct BSPContentInfo {
  // Set to a positive value if this BSPNode or a descendant BSPNode has
  // polygons belonging to this content id, either in the interior of the cell
  // border or on the
  // cell border. Else it is set to zero if there are no polygons for this
  // content id.
  //
  // The implementer may use this as a count of the number of polygons of the
  // given content id, or it may be used as a simple bool.
  size_t has_polygons = 0;
  // PWN for this content at the M-value of the BSPNode.
  int64_t pwn = 0;
};

}  // walnut

#endif // WALNUT_BSP_CONTENT_INFO_H__
