#ifndef LANE_H
#define LANE_H

class Lane {
  public:

  	// Width of the given lane
    double width;

    // Conditions? Mu? etc.
    // ...

    Lane(){};
    Lane(const double width){this->width = width;}
    ~Lane(){};
};

#endif // __LANE_H__