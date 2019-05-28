#ifndef CODE_V1_IPOSE2D_HPP
#define CODE_V1_IPOSE2D_HPP

#include "IRotation2d.hpp"
#include "ITranslation2d.hpp"

namespace lib::geometry {
  template<typename T> ICurvature :  IRotation2d<S>, ITranslation2d<S> {
    public:
      virtual Pose2d getPose() = 0;
      virtual S transformBy(Pose2d transform) = 0;
      virtual S mirror() = 0;
  };
}


#endif //CODE_V1_IPOSE2D_HPP
