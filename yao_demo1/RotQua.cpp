#include "RotQua.h"
#include <iostream>

Rotation::Rotation() {
	for (int i = 0; i < 9; i++) {
		data[i] = 0;
	}
}

void Rotation::quaternion(double x, double y, double z, double w) {
  data[0] = 1 - 2 * y * y - 2 * z * z;
  data[1] = 2 * (x * y - w * z);
  data[2] = 2 * (x * z + w * y);
  data[3] = 2 * (x * y + w * z);
  data[4] = 1 - 2 * x * x - 2 * z * z;
  data[5] = 2 * (y * z - w * x);
  data[6] = 2 * (x * z - w * y);
  data[7] = 2 * (y * z + w * x);
  data[8] = 1 - 2 * x * x - 2 * y * y;
}

void Rotation::display() {
	std::cout << std::endl;
	for (int i = 0; i < 9; i++) {
		if (i % 3 == 0) std::cout << std::endl;
		std::cout << data[i] << '\t';
	}
	std::cout << std::endl;
	
}