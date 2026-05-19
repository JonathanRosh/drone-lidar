#ifndef DRONE_LIDAR_IMAP3D_H
#define DRONE_LIDAR_IMAP3D_H

#include "Units.h"

enum Mapping {
  EMPTY = 0,
  OCCUPIED = 1,
  NOT_MAPPED = -1,
  OUTSIDE_BOUNDARY = -2
};

class IMap3D {

public:
  virtual ~IMap3D() = default;

  virtual Mapping get(const Position3D &pos) const = 0;
  virtual bool isInsideBounds(const Position3D &pos) const = 0;
};

class IWritableMap3D : public virtual IMap3D {
public:
  virtual void set(const Position3D &pos, Mapping val) = 0;
};

#endif // DRONE_LIDAR_IMAP3D_H
