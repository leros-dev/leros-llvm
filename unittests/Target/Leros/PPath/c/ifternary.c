#include <stdlib.h>

int main() {
  int x = rand();
  //  int y = 2 * ++x;
  //  int z = 2 * --x;
  x = x > 200 ? (2 * ++x) : (3 * x);
  return x;
}
