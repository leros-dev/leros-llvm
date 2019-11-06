#include <stdlib.h>

int main() {
  int x = rand();
  do {
    x++;
  } while (x < 100);
  return x;
}
