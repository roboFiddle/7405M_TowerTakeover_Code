#ifndef CODE_V1_ICURVATURE_HPP
#define CODE_V1_ICURVATURE_HPP

namespace lib::geometry {
   template<typename T> ICurvature : State<T> {
     public:
       virtual double getCurvature() = 0;
       virtual double getDCurvatureDs() = 0;
    };
}

#endif //CODE_V1_ICURVATURE_HPP
