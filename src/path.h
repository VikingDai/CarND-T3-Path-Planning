#ifndef PATH_H
#define PATH_H

struct Point {
    double x;
    double y;

    Point(double _x, double _y){
        x = _x; y = _y;
    }
};

class Path {
  public:

    // A Path is just made up of X and Y points
    std::vector<double> x;
    std::vector<double> y;

    Path(){};
    Path(const std::vector<double> &xpts, const std::vector<double> &ypts){x = xpts; y = ypts;};
    ~Path(){};

    int size() const {return x.size();}
};

#endif // __PATH_H__