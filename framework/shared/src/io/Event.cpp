#include "io/Event.h"

using namespace slambench::io;

std::ostream& Event::operator << (std::ostream &out, const Event &e) {
      out << e.ts.S << "." << e.ts.Ns << " = " << e.x << " " << e.y << " " << e.polarity;
      return out;
}