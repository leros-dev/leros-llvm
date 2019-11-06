#include <stdlib.h>

int main() {
  int x = rand();
  if (x > 200) {
    x++;
  } else if (x < 100) {
    x--;
  } else {
    x += 2;
  }
  return x;
}
