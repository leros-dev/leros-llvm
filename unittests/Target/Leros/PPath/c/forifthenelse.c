#include <stdlib.h>

int main() {
  int x = rand();
  for (int i = 0; i <  100; ++i) {
    if (x > 200) {
      x++;
    } else {
      x--;
    }
  }
  return x;
}
