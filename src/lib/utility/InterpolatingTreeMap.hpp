//
// Created by alexweiss on 8/13/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_UTILITY_INTERPOLATINGTREEMAP_HPP_
#define INC_7405M_CODE_SRC_LIB_UTILITY_INTERPOLATINGTREEMAP_HPP_

#include <map>
#include "InterpolatingDouble.hpp"

namespace util {
  template <typename K, typename V>
  class InterpolatingTreeMap {
   private:
    int max_;
    std::map<K, V> map_;
   public:
    InterpolatingTreeMap(int max_size) {
      max_ = max_size;
    }

   V put(K key, V value) {
      if (max_ > 0 && max_ <= map_.size()) {
        // "Prune" the tree if it is oversize
        map_.erase(map_.begin());
      }
      map_.insert({key, value});
      return value;
    }

    V getInterpolated(K key) {
      if(map_.count(key)) {
        V gotval = map_.at(key);
        return gotval;
      }
      /** Get surrounding keys for interpolation */
      K topBound =  map_.upper_bound(key)->first;
      V topElem = map_.upper_bound(key)->second;
      K bottomBound = map_.lower_bound(key)->first;
      V bottomElem = map_.lower_bound(key)->second;
      /**
       * If attempting interpolation at ends of tree, return the nearest data point
       */
      if (map_.upper_bound(key) == map_.end() && map_.lower_bound(key) == map_.end()) {
        return 0.0;
      } else if (map_.upper_bound(key) == map_.end()) {
        return bottomElem;
      } else if (map_.lower_bound(key) == map_.end()) {
        return topElem;
      }

      /** Get surrounding values for interpolation */
      return bottomElem.interpolate(topBound, bottomBound.inverseInterpolate(topBound, key));
    }
  };

  template class InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>;
}

#endif //INC_7405M_CODE_SRC_LIB_UTILITY_INTERPOLATINGTREEMAP_HPP_
