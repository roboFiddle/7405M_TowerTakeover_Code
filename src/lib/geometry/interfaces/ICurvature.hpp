#ifndef CODE_V1_ICURVATURE_HPP
#define CODE_V1_ICURVATURE_HPP

namespace geometry {
   template<typename T> class ICurvature : State<T> {
     public:
       virtual double curvature() = 0;
       virtual double dcurvature() = 0;
    };
}

#endif //CODE_V1_ICURVATURE_HPP
