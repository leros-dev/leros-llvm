#include <stdlib.h>

int main() {
  int x = rand();
  int y = rand();
  int z = rand();
  if (x > 200) {
    x++;
  } else if (x < 100 && x > 50) {
    x--;
  } else if (x <= 50) {
    x += 3*x;
    y += 2*y;
  } else {
    x += 2;
  }
  return z;
}
