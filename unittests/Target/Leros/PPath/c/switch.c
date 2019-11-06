#include <stdlib.h>

int main() {
  int x = rand();
  switch(x) {
  case 200:
    x++;
  case 100:
    x--;
  default:
    x += 2;
  }
  return x;
}
