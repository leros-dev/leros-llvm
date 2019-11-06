#include <stdlib.h>

int main() {
  int x = rand();
  int y = 0;
  if (x > 200) {
    x++;
  } else {
    x--;
  }
  return x;
}
