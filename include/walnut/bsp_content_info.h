#ifndef WALNUT_BSP_CONTENT_INFO_H__
#define WALNUT_BSP_CONTENT_INFO_H__

#include <cstdint>

namespace walnut {

struct BSPContentInfo {
  // Set to true if this BSPNode or a descendant BSPNode has polygons belonging
  // to this content id, either in the interior of the cell border or on the
  // cell border.
  bool has_polygons = false;
  // PWN for this content at the M-value of the BSPNode.
  int64_t pwn = 0;
};

}  // walnut

#endif // WALNUT_BSP_CONTENT_INFO_H__
