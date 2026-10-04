#pragma once
#include "jy_axes.h"
#include <sol/sol.hpp>
sol::usertype<JyShape> bindShape(sol::state &lua);
sol::usertype<JyAxes> bindAxes(sol::state &lua);
void bindEdges(sol::state &lua);
void bindFaces(sol::state &lua);
void bindPrimitives(sol::state &lua);
void bindRobot(sol::state &lua);
