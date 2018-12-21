//
// Created by robot on 18-9-4.
//

#ifndef IMI_NAV_COMMON_H
#define IMI_NAV_COMMON_H

#include <math.h>

const double PI = 3.1415926f;

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

struct SPose {
    double x;
    double y;
    double w;

    SPose() : x(0.0f), y(0.0f), w(0.0f) {}

    SPose(double px,double py,double pw):x(px),y(py),w(pw){}

    SPose(const SPose &other) : x(other.x), y(other.y), w(other.w) {}

    const SPose &operator=(const SPose &other) {
        if (this == &other) {
            return *this;
        }
        x = other.x;
        y = other.y;
        w = other.w;
        return *this;
    }
};

class CUtil {
public:
    static double getYaw(const double &x1, const double &y1, const double &x2, const double &y2) {
        double rota = acos((x1 * x2 + y1 * y2) / (sqrt((x1 * x1 + y1 * y1)) * sqrt((x2 * x2 + y2 * y2))));
        if (y2 < y1) {
            rota = -rota;
        }
        return rota;
    }

    static double getDistance(const double &x1, const double &y1, const double &x2, const double &y2) {
        return sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2));
    }

    static double normalizeYaw(const double &yaw) {
        if (yaw < -PI) {
            return normalizeYaw(yaw + 2 * PI);
        } else if (yaw > PI) {
            return normalizeYaw(yaw - 2 * PI);
        } else {
            return yaw;
        }
    }

    static int getMin(const int &i1, const int &i2){
      if(i1>i2)
        return i2;
      else
        return i1;
    }

    static int getMax(const int &i1, const int &i2){
      if(i1>i2)
        return i1;
      else
        return i2;
    }

    static bool getMinMax(const int &i1,const int &i2,const int &i3,const int &i4,int &min,int &max){
      min=getMin(getMin(i1,i2),getMin(i3,i4));
      max=getMax(getMax(i1,i2),getMax(i3,i4));
      if(min!=max)
        return true;
      return false;
    }

    static float getCross(const SPose &p1,const SPose &p2,const SPose &p)
    {
      return (p2.x - p1.x) * (p.y - p1.y) -(p.x - p1.x) * (p2.y - p1.y);
    }

    static bool isPointInRect(const SPose &p1,const SPose &p2,const SPose &p3,const SPose &p4,const SPose &p){
      float a=getCross(p1,p2,p)*getCross(p3,p4,p);
      float b=getCross(p2,p3,p)*getCross(p4,p1,p);
      if(fabs(a)<0.0001||a>0.0001){
        if(fabs(b)<0.0001||b>0.0001){
          return true;
        }
      }
      return false;
    //  return getCross(p1,p2,p) * getCross(p3,p4,p) >= 0 && getCross(p2,p3,p) * getCross(p4,p1,p) >= 0;
    }


};

#endif //IMI_NAV_COMMON_H
