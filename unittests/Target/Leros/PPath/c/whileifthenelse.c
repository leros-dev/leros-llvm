#include <stdlib.h>

int main() {
  int x = rand();
  while (x < 100) {
    x++;
    if (x > 50) {
      x++;
    } else {
      x *= 2;
    }
  }
  return x;
}
