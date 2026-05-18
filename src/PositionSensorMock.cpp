#include "../include/PositionSensorMock.h"

PositionSensorMock::PositionSensorMock(
  SimulationState& sim_state
) : sim_state(sim_state) {};

const Position3D PositionSensorMock::getPosition() const {
  return sim_state.drone_position;
}

const Orientation PositionSensorMock::getOrientation() const {
  return sim_state.drone_orientation;
}